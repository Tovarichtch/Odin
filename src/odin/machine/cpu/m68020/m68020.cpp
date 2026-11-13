/*
 *  ██████╗ ██████╗ ██╗███╗   ██╗
 * ██╔═══██╗██╔══██╗██║████╗  ██║
 * ██║   ██║██║  ██║██║██╔██╗ ██║
 * ██║   ██║██║  ██║██║██║╚██╗██║
 * ╚██████╔╝██████╔╝██║██║ ╚████║
 *  ╚═════╝ ╚═════╝ ╚═╝╚═╝  ╚═══╝
 *
 * Copyright (C) 2026 Réda Chérif-Touil
 * Copyright (C) 2026 Daniel Martin
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Undefined flags follow Musashi (BSD-3-Clause, Karl Stenerud) */

#include "odin/machine/cpu/m68020/m68020.hpp"

#include <bit>

namespace odin {
namespace {
constexpr uint32_t ADDRESS_MASK = 0xFFFFFF;
constexpr unsigned BUS_CLOCKS = 3;

enum AluOp : unsigned { ALU_OR, ALU_AND, ALU_SUB, ALU_ADD, ALU_EOR, ALU_CMP };
enum BitOp : unsigned { BIT_TEST, BIT_CHANGE, BIT_CLEAR, BIT_SET };
enum ShiftOp : unsigned { SHIFT_ARITH, SHIFT_LOGICAL, ROTATE_X, ROTATE };

bool
alterable(unsigned mode, unsigned reg)
{
	return mode < 7 || reg < 2;
}

bool
control(unsigned mode, unsigned reg)
{
	return mode == 2 || mode == 5 || mode == 6 || (mode == 7 && reg < 4);
}

bool
exists(unsigned mode, unsigned reg)
{
	return mode < 7 || reg < 5;
}

uint32_t
sext16(uint32_t v)
{
	return static_cast<uint32_t>(static_cast<int16_t>(v));
}
}

M68020::M68020(M68020Bus &bus) : m_bus(bus) {}

void
M68020::reset()
{
	m_sr = SR_S | SR_I;
	m_vbr = 0;
	m_cacr = 0;
	m_stopped = false;
	m_nmi_edge = false;
	m_a[7] = m_isp = read32(0);
	m_pc = read32(4);
}

void
M68020::set_ipl(unsigned level)
{
	if (level == 7 && m_ipl != 7)
		m_nmi_edge = true;
	m_ipl = static_cast<uint8_t>(level & 7);
}

void
M68020::save(M68020State &out) const
{
	out = {};
	out.d = m_d;
	out.a = m_a;
	out.usp = m_usp;
	out.isp = m_isp;
	out.msp = m_msp;
	out.pc = m_pc;
	out.vbr = m_vbr;
	out.sfc = m_sfc;
	out.dfc = m_dfc;
	out.cacr = m_cacr;
	out.caar = m_caar;
	out.sr = m_sr;
	out.ipl = m_ipl;
	out.nmi_edge = m_nmi_edge;
	out.stopped = m_stopped;
	out.cycles = m_cycles;
}

void
M68020::load(const M68020State &in)
{
	m_d = in.d;
	m_a = in.a;
	m_usp = in.usp;
	m_isp = in.isp;
	m_msp = in.msp;
	m_pc = in.pc;
	m_vbr = in.vbr;
	m_sfc = in.sfc;
	m_dfc = in.dfc;
	m_cacr = in.cacr;
	m_caar = in.caar;
	m_sr = in.sr;
	m_ipl = in.ipl;
	m_nmi_edge = in.nmi_edge;
	m_stopped = in.stopped;
	m_cycles = in.cycles;
}

uint8_t
M68020::read8(uint32_t addr)
{
	m_cycles += BUS_CLOCKS;
	return m_bus.read8(addr & ADDRESS_MASK);
}

uint16_t
M68020::read16(uint32_t addr)
{
	m_cycles += BUS_CLOCKS;
	if (addr & 1)
		return static_cast<uint16_t>(m_bus.read8(addr & ADDRESS_MASK) << 8 | m_bus.read8((addr + 1) & ADDRESS_MASK));
	return m_bus.read16(addr & ADDRESS_MASK);
}

uint32_t
M68020::read32(uint32_t addr)
{
	m_cycles += BUS_CLOCKS;
	if (addr & 3) {
		if (!(addr & 1))
			return static_cast<uint32_t>(m_bus.read16(addr & ADDRESS_MASK)) << 16 |
			       m_bus.read16((addr + 2) & ADDRESS_MASK);
		uint32_t v = 0;

		for (uint32_t i = 0; i < 4; i++)
			v = v << 8 | m_bus.read8((addr + i) & ADDRESS_MASK);
		return v;
	}
	return m_bus.read32(addr & ADDRESS_MASK);
}

void
M68020::write8(uint32_t addr, uint8_t v)
{
	m_cycles += BUS_CLOCKS;
	m_bus.write8(addr & ADDRESS_MASK, v);
}

void
M68020::write16(uint32_t addr, uint16_t v)
{
	m_cycles += BUS_CLOCKS;
	if (addr & 1) {
		m_bus.write8(addr & ADDRESS_MASK, static_cast<uint8_t>(v >> 8));
		m_bus.write8((addr + 1) & ADDRESS_MASK, static_cast<uint8_t>(v));
		return;
	}
	m_bus.write16(addr & ADDRESS_MASK, v);
}

void
M68020::write32(uint32_t addr, uint32_t v)
{
	m_cycles += BUS_CLOCKS;
	if (addr & 3) {
		if (!(addr & 1)) {
			m_bus.write16(addr & ADDRESS_MASK, static_cast<uint16_t>(v >> 16));
			m_bus.write16((addr + 2) & ADDRESS_MASK, static_cast<uint16_t>(v));
			return;
		}
		for (uint32_t i = 0; i < 4; i++)
			m_bus.write8((addr + i) & ADDRESS_MASK, static_cast<uint8_t>(v >> (24 - 8 * i)));
		return;
	}
	m_bus.write32(addr & ADDRESS_MASK, v);
}

uint32_t
M68020::read(Size s, uint32_t addr)
{
	return s == BYTE ? read8(addr) : s == WORD ? read16(addr) : read32(addr);
}

void
M68020::write(Size s, uint32_t addr, uint32_t v)
{
	if (s == BYTE)
		write8(addr, static_cast<uint8_t>(v));
	else if (s == WORD)
		write16(addr, static_cast<uint16_t>(v));
	else
		write32(addr, v);
}

uint16_t
M68020::fetch16()
{
	const uint32_t at = m_pc;

	m_pc += 2;
	if (at & 1)
		return static_cast<uint16_t>(m_bus.read8(at & ADDRESS_MASK) << 8 | m_bus.read8((at + 1) & ADDRESS_MASK));
	return m_bus.read16(at & ADDRESS_MASK);
}

uint32_t
M68020::fetch32()
{
	const uint32_t hi = fetch16();

	return hi << 16 | fetch16();
}

void
M68020::push16(uint16_t v)
{
	m_a[7] -= 2;
	write16(m_a[7], v);
}

void
M68020::push32(uint32_t v)
{
	m_a[7] -= 4;
	write32(m_a[7], v);
}

uint16_t
M68020::pop16()
{
	const uint16_t v = read16(m_a[7]);

	m_a[7] += 2;
	return v;
}

uint32_t
M68020::pop32()
{
	const uint32_t v = read32(m_a[7]);

	m_a[7] += 4;
	return v;
}

uint32_t
M68020::extension(uint32_t base)
{
	const uint16_t ext = fetch16();
	const unsigned n = (ext >> 12) & 7;
	uint32_t x = (ext & 0x8000) ? m_a[n] : m_d[n];

	if (!(ext & 0x0800))
		x = sext16(x);
	x <<= (ext >> 9) & 3;
	if (!(ext & 0x0100))
		return base + x + static_cast<uint32_t>(static_cast<int8_t>(ext));

	const unsigned bd_size = (ext >> 4) & 3, iis = ext & 7;

	if (ext & 0x80)
		base = 0;
	if (ext & 0x40)
		x = 0;
	m_cycles += 2;

	const uint32_t bd = bd_size == 2 ? sext16(fetch16()) : bd_size == 3 ? fetch32() : 0;

	if (iis == 0)
		return base + bd + x;

	const uint32_t od = (iis & 3) == 2 ? sext16(fetch16()) : (iis & 3) == 3 ? fetch32() : 0;

	if ((ext & 0x40) || iis < 4)
		return read32(base + bd + x) + od;
	return read32(base + bd) + x + od;
}

M68020::Ea
M68020::ea(unsigned mode, unsigned reg, Size s)
{
	Ea e{ Ea::MEM, static_cast<uint8_t>(reg), 0, 0 };
	const uint32_t step = (s == BYTE && reg == 7) ? 2 : bytes(s);

	switch (mode) {
	case 0:
		e.kind = Ea::DREG;
		break;
	case 1:
		e.kind = Ea::AREG;
		break;
	case 2:
		e.addr = m_a[reg];
		break;
	case 3:
		e.addr = m_a[reg];
		m_a[reg] += step;
		break;
	case 4:
		m_a[reg] -= step;
		e.addr = m_a[reg];
		break;
	case 5:
		e.addr = m_a[reg] + sext16(fetch16());
		break;
	case 6:
		e.addr = extension(m_a[reg]);
		break;
	default:
		switch (reg) {
		case 0:
			e.addr = sext16(fetch16());
			break;
		case 1:
			e.addr = fetch32();
			break;
		case 2: {
			const uint32_t base = m_pc;

			e.addr = base + sext16(fetch16());
			break;
		}
		case 3:
			e.addr = extension(m_pc);
			break;
		default:
			e.kind = Ea::IMM;
			e.imm = s == LONG ? fetch32() : (fetch16() & mask(s));
			break;
		}
		break;
	}
	return e;
}

uint32_t
M68020::get(const Ea &e, Size s)
{
	switch (e.kind) {
	case Ea::DREG:
		return m_d[e.reg] & mask(s);
	case Ea::AREG:
		return m_a[e.reg] & mask(s);
	case Ea::MEM:
		return read(s, e.addr);
	default:
		return e.imm;
	}
}

void
M68020::put(const Ea &e, Size s, uint32_t v)
{
	switch (e.kind) {
	case Ea::DREG:
		set_d(e.reg, s, v);
		break;
	case Ea::AREG:
		m_a[e.reg] = s == WORD ? sext16(v) : v;
		break;
	case Ea::MEM:
		write(s, e.addr, v);
		break;
	default:
		break;
	}
}

void
M68020::keep_sp()
{
	if (!(m_sr & SR_S))
		m_usp = m_a[7];
	else if (m_sr & SR_M)
		m_msp = m_a[7];
	else
		m_isp = m_a[7];
}

void
M68020::take_sp()
{
	if (!(m_sr & SR_S))
		m_a[7] = m_usp;
	else if (m_sr & SR_M)
		m_a[7] = m_msp;
	else
		m_a[7] = m_isp;
}

void
M68020::set_sr(uint16_t v)
{
	keep_sp();
	m_sr = v & SR_MASK;
	take_sp();
}

bool
M68020::cond(unsigned cc) const
{
	const bool c = m_sr & SR_C, v = m_sr & SR_V, z = m_sr & SR_Z, n = m_sr & SR_N;

	switch (cc & 15) {
	case 0: return true;
	case 1: return false;
	case 2: return !c && !z;
	case 3: return c || z;
	case 4: return !c;
	case 5: return c;
	case 6: return !z;
	case 7: return z;
	case 8: return !v;
	case 9: return v;
	case 10: return !n;
	case 11: return n;
	case 12: return n == v;
	case 13: return n != v;
	case 14: return !z && n == v;
	default: return z || n != v;
	}
}

void
M68020::flags_nz(Size s, uint32_t r)
{
	uint16_t f = m_sr & static_cast<uint16_t>(~(SR_N | SR_Z | SR_V | SR_C));

	if ((r & mask(s)) == 0)
		f |= SR_Z;
	if (r & msb(s))
		f |= SR_N;
	m_sr = f;
}

void
M68020::flags_add(Size s, uint32_t a, uint32_t b, uint32_t r)
{
	const uint32_t m = msb(s);
	uint16_t f = m_sr & static_cast<uint16_t>(~(SR_X | SR_N | SR_Z | SR_V | SR_C));

	if ((r & mask(s)) == 0)
		f |= SR_Z;
	if (r & m)
		f |= SR_N;
	if ((~(a ^ b) & (a ^ r)) & m)
		f |= SR_V;
	if (((a & b) | (~r & (a | b))) & m)
		f |= SR_C | SR_X;
	m_sr = f;
}

void
M68020::flags_sub(Size s, uint32_t a, uint32_t b, uint32_t r)
{
	const uint32_t m = msb(s);
	uint16_t f = m_sr & static_cast<uint16_t>(~(SR_X | SR_N | SR_Z | SR_V | SR_C));

	if ((r & mask(s)) == 0)
		f |= SR_Z;
	if (r & m)
		f |= SR_N;
	if (((a ^ b) & (a ^ r)) & m)
		f |= SR_V;
	if (((~a & b) | (r & (~a | b))) & m)
		f |= SR_C | SR_X;
	m_sr = f;
}

uint32_t
M68020::alu(unsigned kind, Size s, uint32_t dst, uint32_t src)
{
	uint32_t r;

	switch (kind) {
	case ALU_OR:
		r = dst | src;
		flags_nz(s, r);
		return r;
	case ALU_AND:
		r = dst & src;
		flags_nz(s, r);
		return r;
	case ALU_EOR:
		r = dst ^ src;
		flags_nz(s, r);
		return r;
	case ALU_ADD:
		r = dst + src;
		flags_add(s, dst, src, r);
		return r;
	case ALU_SUB:
		r = dst - src;
		flags_sub(s, dst, src, r);
		return r;
	default: {
		const uint16_t x = m_sr & SR_X;

		r = dst - src;
		flags_sub(s, dst, src, r);
		m_sr = static_cast<uint16_t>((m_sr & ~SR_X) | x);
		return dst;
	}
	}
}

uint32_t
M68020::bcd(bool add, uint32_t dst, uint32_t src)
{
	const uint32_t x = (m_sr & SR_X) ? 1 : 0;
	uint32_t res, carry, v;

	if (add) {
		const uint32_t ss = dst + src + x;
		const uint32_t bc = ((dst & src) | (~ss & (dst | src))) & 0x88;
		const uint32_t dc = (((ss + 0x66) ^ ss) & 0x110) >> 1;
		const uint32_t corf = (bc | dc) - ((bc | dc) >> 2);

		res = ss + corf;
		carry = ((bc | (ss & ~res)) >> 7) & 1;
		v = ((~ss & res) >> 7) & 1;
	} else {
		const uint32_t dd = dst - src - x;
		const uint32_t bc = ((~dst & src) | (dd & ~dst) | (dd & src)) & 0x88;
		const uint32_t corf = bc - (bc >> 2);

		res = dd - corf;
		carry = ((bc | (~dd & res)) >> 7) & 1;
		v = ((dd & ~res) >> 7) & 1;
	}
	uint16_t f = m_sr & static_cast<uint16_t>(~(SR_X | SR_N | SR_V | SR_C));
	if (carry)
		f |= SR_X | SR_C;
	if (v)
		f |= SR_V;
	if (res & 0x80)
		f |= SR_N;
	if (res & 0xFF)
		f &= static_cast<uint16_t>(~SR_Z);
	m_sr = f;
	return res & 0xFF;
}

void
M68020::exception(unsigned vector, uint32_t return_pc, bool format2)
{
	const uint16_t old = m_sr;

	set_sr(static_cast<uint16_t>((m_sr | SR_S) & ~SR_T));
	if (format2) {
		push32(m_ppc);
		push16(static_cast<uint16_t>(0x2000 | vector * 4));
	} else {
		push16(static_cast<uint16_t>(vector * 4));
	}
	push32(return_pc);
	push16(old);
	m_pc = read32(m_vbr + vector * 4);
	m_cycles += 20;
}

void
M68020::interrupt(unsigned level)
{
	const uint16_t old = m_sr;
	const int answer = m_bus.int_ack(level);
	const unsigned vector = answer < 0 ? 24 + level : static_cast<unsigned>(answer);

	m_stopped = false;
	set_sr(static_cast<uint16_t>(((m_sr | SR_S) & ~SR_T & ~SR_I) | level << 8));
	push16(static_cast<uint16_t>(vector * 4));
	push32(m_pc);
	push16(old);
	if (m_sr & SR_M) {
		const uint16_t now = m_sr;

		set_sr(static_cast<uint16_t>(m_sr & ~SR_M));
		push16(static_cast<uint16_t>(0x1000 | vector * 4));
		push32(m_pc);
		push16(now);
	}
	m_pc = read32(m_vbr + vector * 4);
	m_cycles += 26;
}

uint32_t
M68020::run(uint32_t cycles)
{
	const uint64_t start = m_cycles;

	m_run_end = m_cycles + cycles;
	while (m_cycles < m_run_end) {
		const unsigned mask_level = (m_sr & SR_I) >> 8;

		if (m_nmi_edge) {
			m_nmi_edge = false;
			interrupt(7);
			continue;
		}
		if (m_ipl > mask_level) {
			interrupt(m_ipl);
			continue;
		}
		if (m_stopped || (m_pc == m_wait_pc && m_bus.waiting(m_pc))) {
			m_cycles = m_run_end;
			break;
		}
		execute();
	}
	return static_cast<uint32_t>(m_cycles - start);
}

void
M68020::execute()
{
	m_ppc = m_pc;
	m_ir = fetch16();
	m_cycles += 2;
	switch (m_ir >> 12) {
	case 0x0:
		op_0000();
		break;
	case 0x1:
		op_move(BYTE);
		break;
	case 0x2:
		op_move(LONG);
		break;
	case 0x3:
		op_move(WORD);
		break;
	case 0x4:
		op_0100();
		break;
	case 0x5:
		op_0101();
		break;
	case 0x6:
		op_0110();
		break;
	case 0x7:
		if (m_ir & 0x100) {
			illegal();
			break;
		}
		m_d[(m_ir >> 9) & 7] = static_cast<uint32_t>(static_cast<int8_t>(m_ir));
		flags_nz(LONG, m_d[(m_ir >> 9) & 7]);
		break;
	case 0x8:
		op_1000();
		break;
	case 0x9:
		op_add_sub(false);
		break;
	case 0xA:
		illegal(10);
		break;
	case 0xB:
		op_1011();
		break;
	case 0xC:
		op_1100();
		break;
	case 0xD:
		op_add_sub(true);
		break;
	case 0xE:
		op_1110();
		break;
	default:
		illegal(11);
		break;
	}
}

void
M68020::op_0000()
{
	const uint16_t op = m_ir;
	const unsigned mode = (op >> 3) & 7, reg = op & 7, high = (op >> 9) & 7;

	if (op & 0x100) {
		if (mode == 1) {
			const uint32_t addr = m_a[reg] + sext16(fetch16());
			const unsigned dreg = (op >> 9) & 7;

			switch ((op >> 6) & 3) {
			case 0:
				set_d(dreg, WORD, static_cast<uint32_t>(read8(addr)) << 8 | read8(addr + 2));
				break;
			case 1:
				m_d[dreg] = static_cast<uint32_t>(read8(addr)) << 24 | static_cast<uint32_t>(read8(addr + 2)) << 16 |
				            static_cast<uint32_t>(read8(addr + 4)) << 8 | read8(addr + 6);
				break;
			case 2:
				write8(addr, static_cast<uint8_t>(m_d[dreg] >> 8));
				write8(addr + 2, static_cast<uint8_t>(m_d[dreg]));
				break;
			default:
				for (unsigned i = 0; i < 4; i++)
					write8(addr + 2 * i, static_cast<uint8_t>(m_d[dreg] >> (24 - 8 * i)));
				break;
			}
			return;
		}
		op_bit((op >> 6) & 3, true);
		return;
	}
	if (high == 4) {
		op_bit((op >> 6) & 3, false);
		return;
	}
	if (((op >> 6) & 3) == 3) {
		if (high <= 2)
			op_cmp2_chk2();
		else if (high >= 5 && mode == 7 && reg == 4 && high != 5)
			op_cas2();
		else if (high >= 5)
			op_cas();
		else
			illegal();
		return;
	}
	if (high == 7) {
		op_moves();
		return;
	}
	op_immediate();
}

void
M68020::op_bit(unsigned kind, bool dynamic)
{
	const uint16_t op = m_ir;
	const unsigned mode = (op >> 3) & 7, reg = op & 7;
	uint32_t bit = dynamic ? m_d[(op >> 9) & 7] : (fetch16() & 0xFF);

	if (!exists(mode, reg) || mode == 1 || (kind != BIT_TEST && !alterable(mode, reg)) ||
	    (!dynamic && mode == 7 && reg == 4)) {
		illegal();
		return;
	}
	if (mode == 0) {
		bit &= 31;
		uint32_t &v = m_d[reg];

		m_sr = static_cast<uint16_t>((v >> bit) & 1 ? m_sr & ~SR_Z : m_sr | SR_Z);
		if (kind == BIT_CHANGE)
			v ^= 1u << bit;
		else if (kind == BIT_CLEAR)
			v &= ~(1u << bit);
		else if (kind == BIT_SET)
			v |= 1u << bit;
		return;
	}
	const Ea e = ea(mode, reg, BYTE);
	uint32_t v = get(e, BYTE);

	bit &= 7;
	m_sr = static_cast<uint16_t>((v >> bit) & 1 ? m_sr & ~SR_Z : m_sr | SR_Z);
	if (kind == BIT_TEST)
		return;
	if (kind == BIT_CHANGE)
		v ^= 1u << bit;
	else if (kind == BIT_CLEAR)
		v &= ~(1u << bit);
	else
		v |= 1u << bit;
	write8(e.addr, static_cast<uint8_t>(v));
}

void
M68020::op_immediate()
{
	static const unsigned KINDS[8] = { ALU_OR, ALU_AND, ALU_SUB, ALU_ADD, 99, ALU_EOR, ALU_CMP, 99 };
	const uint16_t op = m_ir;
	const unsigned mode = (op >> 3) & 7, reg = op & 7, kind = KINDS[(op >> 9) & 7];
	const Size s = size_bits_to((op >> 6) & 3);

	if (kind == 99) {
		illegal();
		return;
	}
	if (mode == 7 && reg == 4 && (kind == ALU_OR || kind == ALU_AND || kind == ALU_EOR)) {
		const uint32_t src = fetch16();

		if (s == BYTE) {
			set_ccr(alu(kind, BYTE, m_sr & 0x1F, src & 0x1F));
			return;
		}
		if (s != WORD) {
			illegal();
			return;
		}
		if (!(m_sr & SR_S)) {
			privilege();
			return;
		}
		const uint32_t r = kind == ALU_OR ? (m_sr | src) : kind == ALU_AND ? (m_sr & src) : (m_sr ^ src);

		set_sr(static_cast<uint16_t>(r));
		return;
	}
	if (mode == 1 || !exists(mode, reg) || (kind != ALU_CMP && !alterable(mode, reg)) || (mode == 7 && reg == 4)) {
		illegal();
		return;
	}
	const uint32_t src = s == LONG ? fetch32() : (fetch16() & mask(s));
	const Ea e = ea(mode, reg, s);
	const uint32_t r = alu(kind, s, get(e, s), src);

	if (kind != ALU_CMP)
		put(e, s, r);
}

void
M68020::op_cmp2_chk2()
{
	const uint16_t op = m_ir;
	const unsigned mode = (op >> 3) & 7, reg = op & 7;
	const Size s = size_bits_to((op >> 9) & 3);

	if (!control(mode, reg)) {
		illegal();
		return;
	}
	const uint16_t ext = fetch16();
	const unsigned n = (ext >> 12) & 7;
	const bool areg = ext & 0x8000;
	const Ea e = ea(mode, reg, s);
	int64_t lower = read(s, e.addr), upper = read(s, e.addr + bytes(s));
	int64_t value = areg ? m_a[n] : m_d[n];

	if (!areg)
		value &= mask(s);
	if (static_cast<uint64_t>(lower) & msb(s)) {
		lower = static_cast<int32_t>(sext(s, static_cast<uint32_t>(lower)));
		upper = static_cast<int32_t>(sext(s, static_cast<uint32_t>(upper)));
		if (!areg || s == LONG)
			value = static_cast<int32_t>(sext(s, static_cast<uint32_t>(value)));
	}
	uint16_t f = m_sr & static_cast<uint16_t>(~(SR_Z | SR_C));

	if (value < lower || value > upper)
		f |= SR_C;
	if (value == lower || value == upper)
		f |= SR_Z;
	m_sr = f;
	m_cycles += 8;
	if ((f & SR_C) && (ext & 0x0800))
		exception(6, m_pc, true);
}

void
M68020::op_cas()
{
	const uint16_t op = m_ir;
	const unsigned mode = (op >> 3) & 7, reg = op & 7;
	const Size s = size_bits_to(((op >> 9) & 3) - 1);

	if (mode < 2 || !alterable(mode, reg)) {
		illegal();
		return;
	}
	const uint16_t ext = fetch16();
	const unsigned dc = ext & 7, du = (ext >> 6) & 7;
	const Ea e = ea(mode, reg, s);
	const uint32_t dst = read(s, e.addr);

	alu(ALU_CMP, s, dst, m_d[dc] & mask(s));
	if (m_sr & SR_Z)
		write(s, e.addr, m_d[du]);
	else
		set_d(dc, s, dst);
}

void
M68020::op_cas2()
{
	const Size s = (m_ir & 0x0200) ? LONG : WORD;
	const uint16_t ext1 = fetch16(), ext2 = fetch16();
	const unsigned dc1 = ext1 & 7, du1 = (ext1 >> 6) & 7, dc2 = ext2 & 7, du2 = (ext2 >> 6) & 7;
	const uint32_t addr1 = (ext1 & 0x8000) ? m_a[(ext1 >> 12) & 7] : m_d[(ext1 >> 12) & 7];
	const uint32_t addr2 = (ext2 & 0x8000) ? m_a[(ext2 >> 12) & 7] : m_d[(ext2 >> 12) & 7];
	const uint32_t dst1 = read(s, addr1), dst2 = read(s, addr2);

	alu(ALU_CMP, s, dst1, m_d[dc1] & mask(s));
	if (m_sr & SR_Z) {
		alu(ALU_CMP, s, dst2, m_d[dc2] & mask(s));
		if (m_sr & SR_Z) {
			write(s, addr1, m_d[du1]);
			write(s, addr2, m_d[du2]);
			return;
		}
	}
	set_d(dc1, s, dst1);
	set_d(dc2, s, dst2);
}

void
M68020::op_moves()
{
	const uint16_t op = m_ir;
	const unsigned mode = (op >> 3) & 7, reg = op & 7;
	const Size s = size_bits_to((op >> 6) & 3);

	if (!(m_sr & SR_S)) {
		privilege();
		return;
	}
	if (mode < 2 || !alterable(mode, reg)) {
		illegal();
		return;
	}
	const uint16_t ext = fetch16();
	const unsigned n = (ext >> 12) & 7;
	const Ea e = ea(mode, reg, s);

	if (ext & 0x0800) {
		write(s, e.addr, (ext & 0x8000) ? m_a[n] : m_d[n]);
		return;
	}
	const uint32_t v = read(s, e.addr);

	if (ext & 0x8000)
		m_a[n] = sext(s, v);
	else
		set_d(n, s, v);
}

void
M68020::op_move(Size s)
{
	const uint16_t op = m_ir;
	const unsigned smode = (op >> 3) & 7, sreg = op & 7, dmode = (op >> 6) & 7, dreg = (op >> 9) & 7;

	if (!exists(smode, sreg) || (s == BYTE && smode == 1) || !alterable(dmode, dreg) ||
	    (s == BYTE && dmode == 1)) {
		illegal();
		return;
	}
	const uint32_t v = get(ea(smode, sreg, s), s);

	if (dmode == 1) {
		m_a[dreg] = sext(s, v);
		return;
	}
	const Ea d = ea(dmode, dreg, s);

	flags_nz(s, v);
	put(d, s, v);
}

void
M68020::op_0100()
{
	const uint16_t op = m_ir;
	const unsigned mode = (op >> 3) & 7, reg = op & 7;

	if (op & 0x100) {
		const unsigned an = (op >> 9) & 7;

		switch ((op >> 6) & 3) {
		case 0:
			op_chk(LONG);
			return;
		case 2:
			op_chk(WORD);
			return;
		case 3:
			if (an == 4 && mode == 0) {
				m_d[reg] = static_cast<uint32_t>(static_cast<int8_t>(m_d[reg]));
				flags_nz(LONG, m_d[reg]);
				return;
			}
			if (!control(mode, reg)) {
				illegal();
				return;
			}
			m_a[an] = ea(mode, reg, LONG).addr;
			return;
		default:
			illegal();
			return;
		}
	}
	switch ((op >> 8) & 0xF) {
	case 0x0:
	case 0x2:
	case 0x4:
	case 0x6: {
		const unsigned which = (op >> 9) & 3;
		const unsigned bits = (op >> 6) & 3;

		if (bits == 3) {
			if (which == 0) {
				if (!alterable(mode, reg) || mode == 1) {
					illegal();
					return;
				}
				if (!(m_sr & SR_S)) {
					privilege();
					return;
				}
				put(ea(mode, reg, WORD), WORD, m_sr);
			} else if (which == 1) {
				if (!alterable(mode, reg) || mode == 1) {
					illegal();
					return;
				}
				put(ea(mode, reg, WORD), WORD, m_sr & 0x1F);
			} else if (which == 2) {
				if (mode == 1 || !exists(mode, reg)) {
					illegal();
					return;
				}
				set_ccr(get(ea(mode, reg, WORD), WORD));
			} else {
				if (mode == 1 || !exists(mode, reg)) {
					illegal();
					return;
				}
				if (!(m_sr & SR_S)) {
					privilege();
					return;
				}
				set_sr(static_cast<uint16_t>(get(ea(mode, reg, WORD), WORD)));
			}
			return;
		}
		const Size s = size_bits_to(bits);

		if (mode == 1 || !alterable(mode, reg)) {
			illegal();
			return;
		}
		const Ea e = ea(mode, reg, s);

		if (which == 1) {
			flags_nz(s, 0);
			put(e, s, 0);
			return;
		}
		const uint32_t v = get(e, s);
		uint32_t r;

		if (which == 0) {
			r = extend(false, s, 0, v);
		} else if (which == 2) {
			r = 0 - v;
			flags_sub(s, 0, v, r);
		} else {
			r = ~v;
			flags_nz(s, r);
		}
		put(e, s, r);
		return;
	}
	case 0x8: {
		const unsigned bits = (op >> 6) & 3;

		if (bits == 0) {
			if (mode == 1) {
				const int32_t disp = static_cast<int32_t>(fetch32());

				push32(m_a[reg]);
				m_a[reg] = m_a[7];
				m_a[7] += static_cast<uint32_t>(disp);
				return;
			}
			if (mode == 1 || !alterable(mode, reg)) {
				illegal();
				return;
			}
			const Ea e = ea(mode, reg, BYTE);

			put(e, BYTE, bcd(false, 0, get(e, BYTE)));
			return;
		}
		if (bits == 1) {
			if (mode == 0) {
				m_d[reg] = m_d[reg] >> 16 | m_d[reg] << 16;
				flags_nz(LONG, m_d[reg]);
				return;
			}
			if (mode == 1) {
				illegal(4);
				return;
			}
			if (!control(mode, reg)) {
				illegal();
				return;
			}
			push32(ea(mode, reg, LONG).addr);
			return;
		}
		if (mode == 0) {
			if (bits == 2) {
				set_d(reg, WORD, static_cast<uint32_t>(static_cast<int8_t>(m_d[reg])));
				flags_nz(WORD, m_d[reg]);
			} else {
				m_d[reg] = sext16(m_d[reg]);
				flags_nz(LONG, m_d[reg]);
			}
			return;
		}
		op_movem(false);
		return;
	}
	case 0xA: {
		const unsigned bits = (op >> 6) & 3;

		if (op == 0x4AFC) {
			illegal();
			return;
		}
		if (bits == 3) {
			if (mode == 1 || !alterable(mode, reg)) {
				illegal();
				return;
			}
			const Ea e = ea(mode, reg, BYTE);
			const uint32_t v = get(e, BYTE);

			flags_nz(BYTE, v);
			put(e, BYTE, v | 0x80);
			return;
		}
		const Size s = size_bits_to(bits);

		if (!exists(mode, reg) || (mode == 1 && s == BYTE)) {
			illegal();
			return;
		}
		flags_nz(s, get(ea(mode, reg, s), s));
		return;
	}
	case 0xC:
		if (((op >> 6) & 3) == 0) {
			op_mul_long();
			return;
		}
		if (((op >> 6) & 3) == 1) {
			op_div_long();
			return;
		}
		op_movem(true);
		return;
	case 0xE:
		op_misc_4e();
		return;
	default:
		illegal();
		return;
	}
}

void
M68020::op_chk(Size s)
{
	const uint16_t op = m_ir;
	const unsigned mode = (op >> 3) & 7, reg = op & 7;

	if (mode == 1 || !exists(mode, reg)) {
		illegal();
		return;
	}
	const int32_t bound = static_cast<int32_t>(sext(s, get(ea(mode, reg, s), s)));
	const int32_t v = static_cast<int32_t>(sext(s, m_d[(op >> 9) & 7]));

	m_sr = static_cast<uint16_t>(m_sr & ~(SR_Z | SR_V | SR_C));
	if ((v & mask(s)) == 0)
		m_sr |= SR_Z;
	if (v < 0) {
		m_sr |= SR_N;
		exception(6, m_pc, true);
	} else if (v > bound) {
		m_sr = static_cast<uint16_t>(m_sr & ~SR_N);
		exception(6, m_pc, true);
	}
}

void
M68020::op_movem(bool to_registers)
{
	const uint16_t op = m_ir;
	const unsigned mode = (op >> 3) & 7, reg = op & 7;
	const Size s = (op & 0x40) ? LONG : WORD;
	const uint16_t list = fetch16();

	if (to_registers) {
		if (mode < 2 || mode == 4 || !exists(mode, reg)) {
			illegal();
			return;
		}
		uint32_t addr = mode == 3 ? m_a[reg] : ea(mode, reg, s).addr;

		for (unsigned i = 0; i < 16; i++)
			if (list & (1u << i)) {
				const uint32_t v = sext(s, read(s, addr));

				if (i < 8)
					m_d[i] = v;
				else
					m_a[i - 8] = v;
				addr += bytes(s);
			}
		if (mode == 3)
			m_a[reg] = addr;
		return;
	}
	if (mode < 2 || mode == 3 || !alterable(mode, reg)) {
		illegal();
		return;
	}
	if (mode == 4) {
		uint32_t addr = m_a[reg];

		for (unsigned i = 0; i < 16; i++)
			if (list & (1u << i)) {
				const unsigned r = 15 - i;

				addr -= bytes(s);
				write(s, addr, r < 8 ? m_d[r] : m_a[r - 8]);
			}
		m_a[reg] = addr;
		return;
	}
	uint32_t addr = ea(mode, reg, s).addr;

	for (unsigned i = 0; i < 16; i++)
		if (list & (1u << i)) {
			write(s, addr, i < 8 ? m_d[i] : m_a[i - 8]);
			addr += bytes(s);
		}
}

void
M68020::op_mul_long()
{
	const uint16_t op = m_ir;
	const unsigned mode = (op >> 3) & 7, reg = op & 7;

	if (mode == 1 || !exists(mode, reg)) {
		illegal();
		return;
	}
	const uint16_t ext = fetch16();
	const uint32_t src = get(ea(mode, reg, LONG), LONG);
	const unsigned dl = (ext >> 12) & 7, dh = ext & 7;
	uint64_t r;

	if (ext & 0x0800)
		r = static_cast<uint64_t>(static_cast<int64_t>(static_cast<int32_t>(src)) *
		                          static_cast<int32_t>(m_d[dl]));
	else
		r = static_cast<uint64_t>(src) * m_d[dl];
	m_cycles += 41;

	uint16_t f = m_sr & static_cast<uint16_t>(~(SR_N | SR_Z | SR_V | SR_C));

	if (ext & 0x0400) {
		if (r == 0)
			f |= SR_Z;
		if (r >> 63)
			f |= SR_N;
		m_d[dh] = static_cast<uint32_t>(r >> 32);
		m_d[dl] = static_cast<uint32_t>(r);
	} else {
		const uint32_t low = static_cast<uint32_t>(r);

		if (low == 0)
			f |= SR_Z;
		if (low >> 31)
			f |= SR_N;
		if (ext & 0x0800 ? static_cast<int64_t>(r) != static_cast<int32_t>(low) : (r >> 32) != 0)
			f |= SR_V;
		m_d[dl] = low;
	}
	m_sr = f;
}

void
M68020::op_div_long()
{
	const uint16_t op = m_ir;
	const unsigned mode = (op >> 3) & 7, reg = op & 7;

	if (mode == 1 || !exists(mode, reg)) {
		illegal();
		return;
	}
	const uint16_t ext = fetch16();
	const uint32_t divisor = get(ea(mode, reg, LONG), LONG);
	const unsigned dq = (ext >> 12) & 7, dr = ext & 7;
	const bool is_signed = ext & 0x0800, quad = ext & 0x0400;

	if (divisor == 0) {
		exception(5, m_pc, true);
		return;
	}
	m_cycles += 76;
	uint16_t f = m_sr & static_cast<uint16_t>(~(SR_N | SR_Z | SR_V | SR_C));

	if (is_signed) {
		const int64_t dividend = quad ? static_cast<int64_t>(static_cast<uint64_t>(m_d[dr]) << 32 | m_d[dq])
		                              : static_cast<int32_t>(m_d[dq]);
		const int64_t d = static_cast<int32_t>(divisor);

		if (dividend == INT64_MIN && d == -1) {
			m_sr = static_cast<uint16_t>(f | SR_V);
			return;
		}
		const int64_t q = dividend / d, rem = dividend % d;

		if (q != static_cast<int32_t>(q)) {
			m_sr = static_cast<uint16_t>(f | SR_V);
			return;
		}
		if (q == 0)
			f |= SR_Z;
		if (q < 0)
			f |= SR_N;
		m_d[dr] = static_cast<uint32_t>(rem);
		m_d[dq] = static_cast<uint32_t>(q);
	} else {
		const uint64_t dividend = quad ? static_cast<uint64_t>(m_d[dr]) << 32 | m_d[dq] : m_d[dq];
		const uint64_t q = dividend / divisor, rem = dividend % divisor;

		if (q >> 32) {
			m_sr = static_cast<uint16_t>(f | SR_V);
			return;
		}
		if (q == 0)
			f |= SR_Z;
		if (q >> 31)
			f |= SR_N;
		m_d[dr] = static_cast<uint32_t>(rem);
		m_d[dq] = static_cast<uint32_t>(q);
	}
	m_sr = f;
}

void
M68020::op_misc_4e()
{
	const uint16_t op = m_ir;
	const unsigned mode = (op >> 3) & 7, reg = op & 7;

	if ((op & 0xFFC0) == 0x4E80 || (op & 0xFFC0) == 0x4EC0) {
		if (!control(mode, reg)) {
			illegal();
			return;
		}
		const uint32_t target = ea(mode, reg, LONG).addr;

		if ((op & 0xFFC0) == 0x4E80)
			push32(m_pc);
		m_pc = target;
		return;
	}
	if ((op & 0xFFF0) == 0x4E40) {
		exception(32 + (op & 15), m_pc);
		return;
	}
	if ((op & 0xFFF8) == 0x4E50) {
		const uint32_t disp = sext16(fetch16());

		push32(m_a[reg]);
		m_a[reg] = m_a[7];
		m_a[7] += disp;
		return;
	}
	if ((op & 0xFFF8) == 0x4E58) {
		m_a[7] = m_a[reg];
		m_a[reg] = pop32();
		return;
	}
	if ((op & 0xFFF0) == 0x4E60) {
		if (!(m_sr & SR_S)) {
			privilege();
			return;
		}
		if (op & 8)
			m_a[reg] = m_usp;
		else
			m_usp = m_a[reg];
		return;
	}
	switch (op) {
	case 0x4E70:
		if (!(m_sr & SR_S)) {
			privilege();
			return;
		}
		m_bus.reset_devices();
		m_cycles += 512;
		return;
	case 0x4E71:
		return;
	case 0x4E72: {
		const uint16_t v = fetch16();

		if (!(m_sr & SR_S)) {
			privilege();
			return;
		}
		set_sr(v);
		m_stopped = true;
		return;
	}
	case 0x4E73: {
		if (!(m_sr & SR_S)) {
			privilege();
			return;
		}
		for (;;) {
			const uint16_t sr = pop16();
			const uint32_t pc = pop32();
			const uint16_t fv = pop16();
			const unsigned format = fv >> 12;

			if (format == 2)
				m_a[7] += 4;
			else if (format == 9)
				m_a[7] += 12;
			else if (format == 0xA)
				m_a[7] += 24;
			else if (format == 0xB)
				m_a[7] += 84;
			m_pc = pc;
			set_sr(sr);

			if (format != 1)
				break;
		}
		return;
	}
	case 0x4E74: {
		const uint32_t disp = sext16(fetch16());

		m_pc = pop32();
		m_a[7] += disp;
		return;
	}
	case 0x4E75:
		m_pc = pop32();
		return;
	case 0x4E76:
		if (m_sr & SR_V)
			exception(7, m_pc, true);
		return;
	case 0x4E77:
		set_ccr(pop16());
		m_pc = pop32();
		return;
	case 0x4E7A:
	case 0x4E7B:
		op_movec(op & 1);
		return;
	default:
		illegal();
		return;
	}
}

void
M68020::op_movec(bool to_control)
{
	if (!(m_sr & SR_S)) {
		privilege();
		return;
	}
	const uint16_t ext = fetch16();
	const unsigned n = (ext >> 12) & 7;
	uint32_t &r = (ext & 0x8000) ? m_a[n] : m_d[n];

	keep_sp();
	uint32_t *target;

	switch (ext & 0xFFF) {
	case 0x000:
		target = &m_sfc;
		break;
	case 0x001:
		target = &m_dfc;
		break;
	case 0x002:
		target = &m_cacr;
		break;
	case 0x800:
		target = &m_usp;
		break;
	case 0x801:
		target = &m_vbr;
		break;
	case 0x802:
		target = &m_caar;
		break;
	case 0x803:
		target = &m_msp;
		break;
	case 0x804:
		target = &m_isp;
		break;
	default:
		illegal();
		return;
	}
	if (to_control) {
		const uint32_t v = r;

		*target = (target == &m_sfc || target == &m_dfc) ? v & 7 : target == &m_cacr ? v & 3 : v;
		take_sp();
	} else {
		r = *target;
	}
}

void
M68020::op_0101()
{
	const uint16_t op = m_ir;
	const unsigned mode = (op >> 3) & 7, reg = op & 7;

	if (((op >> 6) & 3) == 3) {
		const unsigned cc = (op >> 8) & 15;

		if (mode == 1) {
			const uint32_t disp = sext16(fetch16());

			if (cond(cc))
				return;
			const uint16_t count = static_cast<uint16_t>(m_d[reg] - 1);

			set_d(reg, WORD, count);
			if (count != 0xFFFF)
				m_pc = m_ppc + 2 + disp;
			return;
		}
		if (mode == 7 && reg >= 2 && reg <= 4) {
			if (reg == 2)
				fetch16();
			else if (reg == 3)
				fetch32();
			if (cond(cc))
				exception(7, m_pc, true);
			return;
		}
		if (!alterable(mode, reg)) {
			illegal();
			return;
		}
		put(ea(mode, reg, BYTE), BYTE, cond(cc) ? 0xFF : 0);
		return;
	}
	const Size s = size_bits_to((op >> 6) & 3);
	const uint32_t q = ((op >> 9) & 7) ? (op >> 9) & 7 : 8;

	if (!alterable(mode, reg) || (mode == 1 && s == BYTE)) {
		illegal();
		return;
	}
	if (mode == 1) {
		m_a[reg] = (op & 0x100) ? m_a[reg] - q : m_a[reg] + q;
		return;
	}
	const Ea e = ea(mode, reg, s);

	put(e, s, alu((op & 0x100) ? ALU_SUB : ALU_ADD, s, get(e, s), q));
}

void
M68020::op_0110()
{
	const uint16_t op = m_ir;
	const unsigned cc = (op >> 8) & 15;
	uint32_t disp = static_cast<uint32_t>(static_cast<int8_t>(op));
	const uint32_t base = m_pc;

	if ((op & 0xFF) == 0)
		disp = sext16(fetch16());
	else if ((op & 0xFF) == 0xFF)
		disp = fetch32();
	if (cc == 1) {
		push32(m_pc);
		m_pc = base + disp;
		return;
	}
	if (cond(cc))
		m_pc = base + disp;
}

void
M68020::op_1000()
{
	const uint16_t op = m_ir;
	const unsigned mode = (op >> 3) & 7, reg = op & 7, dreg = (op >> 9) & 7, opmode = (op >> 6) & 7;

	if (opmode == 3 || opmode == 7) {
		op_div_word(opmode == 7);
		return;
	}
	if (opmode == 4 && mode < 2) {
		op_abcd_sbcd(false);
		return;
	}
	if ((opmode == 5 || opmode == 6) && mode < 2) {
		op_pack_unpk(opmode == 5);
		return;
	}
	const Size s = size_bits_to(opmode & 3);

	if (opmode < 4) {
		if (mode == 1 || !exists(mode, reg)) {
			illegal();
			return;
		}
		set_d(dreg, s, alu(ALU_OR, s, m_d[dreg] & mask(s), get(ea(mode, reg, s), s)));
		return;
	}
	if (mode < 2 || !alterable(mode, reg)) {
		illegal();
		return;
	}
	const Ea e = ea(mode, reg, s);

	put(e, s, alu(ALU_OR, s, get(e, s), m_d[dreg] & mask(s)));
}

void
M68020::op_div_word(bool is_signed)
{
	const uint16_t op = m_ir;
	const unsigned mode = (op >> 3) & 7, reg = op & 7, dreg = (op >> 9) & 7;

	if (mode == 1 || !exists(mode, reg)) {
		illegal();
		return;
	}
	const uint32_t src = get(ea(mode, reg, WORD), WORD);

	if (src == 0) {
		exception(5, m_pc, true);
		return;
	}
	uint16_t f = m_sr & static_cast<uint16_t>(~(SR_N | SR_Z | SR_V | SR_C));

	if (is_signed) {
		m_cycles += 54;
		const int32_t dividend = static_cast<int32_t>(m_d[dreg]), divisor = static_cast<int16_t>(src);

		if (dividend == INT32_MIN && divisor == -1) {
			m_sr = static_cast<uint16_t>(f | SR_V);
			return;
		}
		const int32_t q = dividend / divisor, rem = dividend % divisor;

		if (q != static_cast<int16_t>(q)) {
			m_sr = static_cast<uint16_t>(f | SR_V);
			return;
		}
		m_d[dreg] = static_cast<uint32_t>(rem) << 16 | (static_cast<uint32_t>(q) & 0xFFFF);
	} else {
		m_cycles += 42;
		const uint32_t q = m_d[dreg] / src, rem = m_d[dreg] % src;

		if (q > 0xFFFF) {
			m_sr = static_cast<uint16_t>(f | SR_V);
			return;
		}
		m_d[dreg] = rem << 16 | q;
	}
	if ((m_d[dreg] & 0xFFFF) == 0)
		f |= SR_Z;
	if (m_d[dreg] & 0x8000)
		f |= SR_N;
	m_sr = f;
}

void
M68020::op_abcd_sbcd(bool add)
{
	const uint16_t op = m_ir;
	const unsigned rx = op & 7, ry = (op >> 9) & 7;

	if (op & 8) {
		m_a[rx] -= rx == 7 ? 2 : 1;
		const uint32_t src = read8(m_a[rx]);
		m_a[ry] -= ry == 7 ? 2 : 1;
		const uint32_t dst = read8(m_a[ry]);

		write8(m_a[ry], static_cast<uint8_t>(bcd(add, dst, src)));
		return;
	}
	set_d(ry, BYTE, bcd(add, m_d[ry] & 0xFF, m_d[rx] & 0xFF));
}

void
M68020::op_pack_unpk(bool pack)
{
	const uint16_t op = m_ir;
	const unsigned rx = op & 7, ry = (op >> 9) & 7;
	const uint32_t adj = fetch16();

	if (pack) {
		uint32_t v;

		if (op & 8) {
			m_a[rx] -= 2;
			v = static_cast<uint32_t>(read8(m_a[rx])) << 8 | read8(m_a[rx] + 1);
		} else {
			v = m_d[rx] & 0xFFFF;
		}
		v += adj;
		const uint8_t r = static_cast<uint8_t>((v >> 4 & 0xF0) | (v & 0x0F));

		if (op & 8) {
			m_a[ry] -= ry == 7 ? 2 : 1;
			write8(m_a[ry], r);
		} else {
			set_d(ry, BYTE, r);
		}
		return;
	}
	uint32_t v;

	if (op & 8) {
		m_a[rx] -= rx == 7 ? 2 : 1;
		v = read8(m_a[rx]);
	} else {
		v = m_d[rx] & 0xFF;
	}
	v = (((v << 4) & 0x0F00) | (v & 0x0F)) + adj;
	if (op & 8) {
		m_a[ry] -= 2;
		write8(m_a[ry], static_cast<uint8_t>(v >> 8));
		write8(m_a[ry] + 1, static_cast<uint8_t>(v));
	} else {
		set_d(ry, WORD, v);
	}
}

void
M68020::op_add_sub(bool add)
{
	const uint16_t op = m_ir;
	const unsigned mode = (op >> 3) & 7, reg = op & 7, dreg = (op >> 9) & 7, opmode = (op >> 6) & 7;

	if (opmode == 3 || opmode == 7) {
		const Size s = opmode == 3 ? WORD : LONG;

		if (!exists(mode, reg)) {
			illegal();
			return;
		}
		const uint32_t src = sext(s, get(ea(mode, reg, s), s));

		m_a[dreg] = add ? m_a[dreg] + src : m_a[dreg] - src;
		return;
	}
	if (opmode >= 4 && mode < 2) {
		op_addx_subx(add);
		return;
	}
	const Size s = size_bits_to(opmode & 3);
	const unsigned kind = add ? ALU_ADD : ALU_SUB;

	if (opmode < 4) {
		if (!exists(mode, reg) || (mode == 1 && s == BYTE)) {
			illegal();
			return;
		}
		set_d(dreg, s, alu(kind, s, m_d[dreg] & mask(s), get(ea(mode, reg, s), s)));
		return;
	}
	if (!alterable(mode, reg)) {
		illegal();
		return;
	}
	const Ea e = ea(mode, reg, s);

	put(e, s, alu(kind, s, get(e, s), m_d[dreg] & mask(s)));
}

void
M68020::op_addx_subx(bool add)
{
	const uint16_t op = m_ir;
	const unsigned rx = op & 7, ry = (op >> 9) & 7;
	const Size s = size_bits_to((op >> 6) & 3);
	uint32_t src, dst;

	if (op & 8) {
		m_a[rx] -= (s == BYTE && rx == 7) ? 2 : bytes(s);
		src = read(s, m_a[rx]);
		m_a[ry] -= (s == BYTE && ry == 7) ? 2 : bytes(s);
		dst = read(s, m_a[ry]);
	} else {
		src = m_d[rx] & mask(s);
		dst = m_d[ry] & mask(s);
	}
	const uint32_t r = extend(add, s, dst, src);

	if (op & 8)
		write(s, m_a[ry], r);
	else
		set_d(ry, s, r);
}

uint32_t
M68020::extend(bool add, Size s, uint32_t dst, uint32_t src)
{
	const uint32_t x = (m_sr & SR_X) ? 1 : 0, m = msb(s);
	const uint32_t r = add ? dst + src + x : dst - src - x;
	uint16_t f = m_sr & static_cast<uint16_t>(~(SR_X | SR_N | SR_V | SR_C));

	if (add ? ((src ^ r) & (dst ^ r)) & m : ((src ^ dst) & (r ^ dst)) & m)
		f |= SR_V;
	if (add ? ((src & dst) | (~r & (src | dst))) & m : ((src & r) | (~dst & (src | r))) & m)
		f |= SR_C | SR_X;
	if (r & m)
		f |= SR_N;
	if (r & mask(s))
		f = static_cast<uint16_t>(f & ~SR_Z);
	m_sr = f;
	return r & mask(s);
}

void
M68020::op_1011()
{
	const uint16_t op = m_ir;
	const unsigned mode = (op >> 3) & 7, reg = op & 7, dreg = (op >> 9) & 7, opmode = (op >> 6) & 7;

	if (opmode == 3 || opmode == 7) {
		const Size s = opmode == 3 ? WORD : LONG;

		if (!exists(mode, reg)) {
			illegal();
			return;
		}
		alu(ALU_CMP, LONG, m_a[dreg], sext(s, get(ea(mode, reg, s), s)));
		return;
	}
	const Size s = size_bits_to(opmode & 3);

	if (opmode < 4) {
		if (!exists(mode, reg) || (mode == 1 && s == BYTE)) {
			illegal();
			return;
		}
		alu(ALU_CMP, s, m_d[dreg] & mask(s), get(ea(mode, reg, s), s));
		return;
	}
	if (mode == 1) {
		const uint32_t src = read(s, m_a[reg]);

		m_a[reg] += (s == BYTE && reg == 7) ? 2 : bytes(s);
		const uint32_t dst = read(s, m_a[dreg]);

		m_a[dreg] += (s == BYTE && dreg == 7) ? 2 : bytes(s);
		alu(ALU_CMP, s, dst, src);
		return;
	}
	if (!alterable(mode, reg)) {
		illegal();
		return;
	}
	const Ea e = ea(mode, reg, s);

	put(e, s, alu(ALU_EOR, s, get(e, s), m_d[dreg] & mask(s)));
}

void
M68020::op_1100()
{
	const uint16_t op = m_ir;
	const unsigned mode = (op >> 3) & 7, reg = op & 7, dreg = (op >> 9) & 7, opmode = (op >> 6) & 7;

	if (opmode == 3 || opmode == 7) {
		if (mode == 1 || !exists(mode, reg)) {
			illegal();
			return;
		}
		const uint32_t src = get(ea(mode, reg, WORD), WORD);
		const uint32_t r = opmode == 7 ? static_cast<uint32_t>(static_cast<int16_t>(src) * static_cast<int16_t>(m_d[dreg]))
		                               : src * (m_d[dreg] & 0xFFFF);

		m_d[dreg] = r;
		flags_nz(LONG, r);
		m_cycles += 25;
		return;
	}
	if (opmode == 4 && mode < 2) {
		op_abcd_sbcd(true);
		return;
	}
	if (opmode >= 5 && mode < 2) {
		const unsigned kind = op & 0x1F8;

		if (kind == 0x140) {
			std::swap(m_d[dreg], m_d[reg]);
		} else if (kind == 0x148) {
			std::swap(m_a[dreg], m_a[reg]);
		} else if (kind == 0x188) {
			std::swap(m_d[dreg], m_a[reg]);
		} else {
			illegal();
		}
		return;
	}
	const Size s = size_bits_to(opmode & 3);

	if (opmode < 4) {
		if (mode == 1 || !exists(mode, reg)) {
			illegal();
			return;
		}
		set_d(dreg, s, alu(ALU_AND, s, m_d[dreg] & mask(s), get(ea(mode, reg, s), s)));
		return;
	}
	if (!alterable(mode, reg)) {
		illegal();
		return;
	}
	const Ea e = ea(mode, reg, s);

	put(e, s, alu(ALU_AND, s, get(e, s), m_d[dreg] & mask(s)));
}

void
M68020::op_1110()
{
	if (((m_ir >> 6) & 3) == 3) {
		if (m_ir & 0x0800)
			op_bitfield();
		else
			op_shift_memory();
		return;
	}
	op_shift_register();
}

namespace {
struct ShiftResult {
	uint32_t value;
	bool c, x, v;
};

ShiftResult
shift_bits(unsigned kind, bool left, uint32_t v, uint32_t m, uint32_t top, unsigned count, bool x)
{
	ShiftResult r{ v & m, false, x, false };

	if (count == 0) {
		r.c = kind == ROTATE_X ? x : false;
		return r;
	}
	for (unsigned i = 0; i < count; i++) {
		const uint32_t old = r.value;
		bool out;

		if (left) {
			out = (old & top) != 0;
			uint32_t in = 0;

			if (kind == ROTATE)
				in = out;
			else if (kind == ROTATE_X)
				in = r.x;
			r.value = ((old << 1) | in) & m;
			if (kind == SHIFT_ARITH && ((old ^ r.value) & top))
				r.v = true;
		} else {
			out = (old & 1) != 0;
			uint32_t in = 0;

			if (kind == SHIFT_ARITH)
				in = (old & top) ? top : 0;
			else if (kind == ROTATE)
				in = out ? top : 0;
			else if (kind == ROTATE_X)
				in = r.x ? top : 0;
			r.value = ((old >> 1) | in) & m;
		}
		r.c = out;
		if (kind != ROTATE)
			r.x = out;
	}
	if (kind == ROTATE_X)
		r.c = r.x;
	return r;
}
}

void
M68020::op_shift_register()
{
	const uint16_t op = m_ir;
	const Size s = size_bits_to((op >> 6) & 3);
	const unsigned kind = (op >> 3) & 3, reg = op & 7;
	const bool left = (op & 0x100) != 0;
	const unsigned count = (op & 0x20) ? (m_d[(op >> 9) & 7] & 63) : (((op >> 9) & 7) ? ((op >> 9) & 7) : 8);
	const ShiftResult r = shift_bits(kind, left, m_d[reg], mask(s), msb(s), count, (m_sr & SR_X) != 0);
	uint16_t f = m_sr & static_cast<uint16_t>(~(SR_N | SR_Z | SR_V | SR_C | (count && kind != ROTATE ? SR_X : 0)));

	if (r.c)
		f |= SR_C;
	if (count && kind != ROTATE && r.x)
		f |= SR_X;
	if (r.v)
		f |= SR_V;
	if ((r.value & mask(s)) == 0)
		f |= SR_Z;
	if (r.value & msb(s))
		f |= SR_N;
	m_sr = f;
	set_d(reg, s, r.value);
	m_cycles += count > 8 ? 4 : 2;
}

void
M68020::op_shift_memory()
{
	const uint16_t op = m_ir;
	const unsigned kind = (op >> 9) & 3, mode = (op >> 3) & 7, reg = op & 7;
	const bool left = (op & 0x100) != 0;

	if (mode < 2 || !alterable(mode, reg)) {
		illegal();
		return;
	}
	const Ea e = ea(mode, reg, WORD);
	const ShiftResult r = shift_bits(kind, left, get(e, WORD), 0xFFFF, 0x8000, 1, (m_sr & SR_X) != 0);
	uint16_t f = m_sr & static_cast<uint16_t>(~(SR_N | SR_Z | SR_V | SR_C | (kind != ROTATE ? SR_X : 0)));

	if (r.c)
		f |= SR_C;
	if (kind != ROTATE && r.x)
		f |= SR_X;
	if (r.v)
		f |= SR_V;
	if ((r.value & 0xFFFF) == 0)
		f |= SR_Z;
	if (r.value & 0x8000)
		f |= SR_N;
	m_sr = f;
	write16(e.addr, static_cast<uint16_t>(r.value));
}

void
M68020::op_bitfield()
{
	const uint16_t op = m_ir;
	const unsigned kind = (op >> 8) & 7, mode = (op >> 3) & 7, reg = op & 7;
	const bool reads = kind == 0 || kind == 1 || kind == 3 || kind == 5;

	if (mode == 1 || (mode != 0 && (reads ? !control(mode, reg) : (!control(mode, reg) || !alterable(mode, reg))))) {
		illegal();
		return;
	}
	const uint16_t ext = fetch16();
	const unsigned rn = (ext >> 12) & 7;
	const int32_t given =
		(ext & 0x0800) ? static_cast<int32_t>(m_d[(ext >> 6) & 7]) : static_cast<int32_t>((ext >> 6) & 31);
	int32_t offset = given;
	unsigned width = (ext & 0x0020) ? m_d[ext & 7] & 31 : ext & 31;

	if (width == 0)
		width = 32;
	m_cycles += 8;

	const uint32_t field_mask = width == 32 ? 0xFFFFFFFFu : ((1u << width) - 1);
	uint32_t field;
	uint64_t data = 0;
	uint32_t addr = 0;
	unsigned shift = 0;

	if (mode == 0) {
		offset &= 31;
		field = std::rotl(m_d[reg], offset) >> (32 - width);
	} else {
		addr = ea(mode, reg, LONG).addr + static_cast<uint32_t>(offset >> 3);
		offset &= 7;
		for (unsigned i = 0; i < 5; i++)
			data = data << 8 | read8(addr + i);
		shift = 40 - static_cast<unsigned>(offset) - width;
		field = static_cast<uint32_t>(data >> shift) & field_mask;
	}
	field &= field_mask;
	uint16_t f = m_sr & static_cast<uint16_t>(~(SR_N | SR_Z | SR_V | SR_C));

	if (field == 0)
		f |= SR_Z;
	if (field >> (width - 1))
		f |= SR_N;

	uint32_t replace = field;

	switch (kind) {
	case 0:
		break;
	case 1:
		m_d[rn] = field;
		break;
	case 2:
		replace = ~field & field_mask;
		break;
	case 3:
		m_d[rn] = (field >> (width - 1)) ? field | ~field_mask : field;
		break;
	case 4:
		replace = 0;
		break;
	case 5: {
		unsigned i = 0;

		while (i < width && !((field >> (width - 1 - i)) & 1))
			i++;
		m_d[rn] = static_cast<uint32_t>(given) + i;
		break;
	}
	case 6:
		replace = field_mask;
		break;
	default:
		replace = m_d[rn] & field_mask;
		f = m_sr & static_cast<uint16_t>(~(SR_N | SR_Z | SR_V | SR_C));
		if (replace == 0)
			f |= SR_Z;
		if (replace >> (width - 1))
			f |= SR_N;
		break;
	}
	m_sr = f;
	if (reads)
		return;
	if (mode == 0) {
		const uint32_t place = std::rotr(field_mask << (32 - width), offset);
		const uint32_t value = std::rotr(replace << (32 - width), offset);

		m_d[reg] = (m_d[reg] & ~place) | (value & place);
		return;
	}
	const uint64_t place = static_cast<uint64_t>(field_mask) << shift;

	data = (data & ~place) | (static_cast<uint64_t>(replace) << shift & place);
	for (unsigned i = 0; i < 5; i++)
		if ((place >> (32 - 8 * i)) & 0xFF)
			write8(addr + i, static_cast<uint8_t>(data >> (32 - 8 * i)));
}
}
