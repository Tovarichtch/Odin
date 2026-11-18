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
 *
 * Follows MAME's adsp2100 core (BSD-3-Clause, copyright Aaron Giles).
 */

#include "odin/machine/cpu/adsp2115/adsp2115.hpp"

#include <bit>
#include <utility>

namespace odin {
namespace {
constexpr uint32_t
bits(uint32_t v, unsigned at, unsigned n)
{
	return (v >> at) & ((1u << n) - 1);
}

constexpr int32_t
sext14(uint32_t v)
{
	return static_cast<int32_t>(v << 18) >> 18;
}

constexpr int32_t
s16(uint16_t v)
{
	return static_cast<int16_t>(v);
}

uint16_t
reverse14(uint32_t i)
{
	uint32_t r = 0;

	for (unsigned b = 0; b < 14; b++)
		r |= ((i >> b) & 1) << (13 - b);
	return static_cast<uint16_t>(r);
}

uint32_t
mask_of(uint32_t l)
{
	uint32_t mask = 0x3FFF;

	for (uint32_t size = 1; size < l && size < 0x4000; size <<= 1)
		mask = (mask << 1) & 0x3FFF;
	return l > 0x2000 ? 0 : mask;
}

uint16_t
mr_word(uint64_t mr, unsigned n)
{
	return static_cast<uint16_t>(mr >> (16 * n));
}

void
set_mr_word(uint64_t &mr, unsigned n, uint16_t v)
{
	mr = (mr & ~(0xFFFFull << (16 * n))) | static_cast<uint64_t>(v) << (16 * n);
}
}

Adsp2115::Adsp2115(Adsp2115Bus &bus) : m_bus(bus)
{
	reset();
}

void
Adsp2115::reset()
{
	for (unsigned n = 0; n < 8; n++) {
		update_l(n);
		update_i(n);
	}
	m_pc = m_ppc = 0;
	m_loop = 0xFFFF;
	m_loop_condition = 0;
	m_astat_clear = ~(CFLAG | VFLAG | NFLAG | ZFLAG);
	m_mstat = 0;
	m_sstat = 0x55;
	m_idle = 0;
	update_mstat();
	m_pc_sp = m_cntr_sp = m_stat_sp = m_loop_sp = 0;
	m_flagout = m_flagin = m_fl0 = m_fl1 = m_fl2 = 0;
	m_imask = 0;
	m_irq_state.fill(0);
	m_irq_latch.fill(0);
	m_irq_pulsed.fill(0);
}

uint16_t
Adsp2115::data_read(uint32_t addr)
{
	addr &= 0x3FFF;
	if (addr >= 0x3800 && addr < 0x3A00)
		return m_dm[addr - 0x3800];
	if (addr >= 0x3FE0)
		return m_control[addr - 0x3FE0];
	return m_bus.data_read(static_cast<uint16_t>(addr));
}

void
Adsp2115::data_write(uint32_t addr, uint16_t value)
{
	addr &= 0x3FFF;
	if (addr >= 0x3800 && addr < 0x3A00) {
		m_dm[addr - 0x3800] = value;
	} else if (addr >= 0x3FE0) {
		m_control[addr - 0x3FE0] = value;
		m_bus.control_written(addr - 0x3FE0, value);
	} else {
		m_bus.data_write(static_cast<uint16_t>(addr), value);
	}
}

uint32_t
Adsp2115::program_read(uint32_t addr)
{
	addr &= 0x3FFF;
	return addr < m_pm.size() ? m_pm[addr] : m_bus.program_read(static_cast<uint16_t>(addr)) & 0xFFFFFF;
}

void
Adsp2115::program_write(uint32_t addr, uint32_t value)
{
	addr &= 0x3FFF;
	value &= 0xFFFFFF;
	if (addr < m_pm.size())
		m_pm[addr] = value;
	else
		m_bus.program_write(static_cast<uint16_t>(addr), value);
}

void
Adsp2115::update_mstat()
{
	if ((m_mstat ^ m_mstat_prev) & MSTAT_BANK)
		std::swap(m_core, m_alt);
	m_astat_clear = (m_mstat & MSTAT_STICKYV) ? ~(CFLAG | NFLAG | ZFLAG) : ~(CFLAG | VFLAG | NFLAG | ZFLAG);
	m_mstat_prev = m_mstat;
}

void
Adsp2115::pc_stack_push(uint32_t value)
{
	if (m_pc_sp < static_cast<int32_t>(m_pc_stack.size())) {
		m_pc_stack[static_cast<size_t>(m_pc_sp++)] = value;
		m_sstat &= ~PC_EMPTY;
	} else {
		m_sstat |= PC_OVER;
	}
}

uint32_t
Adsp2115::pc_stack_pop()
{
	if (m_pc_sp > 0 && --m_pc_sp == 0)
		m_sstat |= PC_EMPTY;
	return m_pc_stack[static_cast<size_t>(m_pc_sp)];
}

void
Adsp2115::cntr_stack_push()
{
	if (m_cntr_sp < static_cast<int32_t>(m_cntr_stack.size())) {
		m_cntr_stack[static_cast<size_t>(m_cntr_sp++)] = m_cntr;
		m_sstat &= ~COUNT_EMPTY;
	} else {
		m_sstat |= COUNT_OVER;
	}
}

void
Adsp2115::cntr_stack_pop()
{
	if (m_cntr_sp > 0 && --m_cntr_sp == 0)
		m_sstat |= COUNT_EMPTY;
	m_cntr = m_cntr_stack[static_cast<size_t>(m_cntr_sp)];
}

void
Adsp2115::loop_stack_push(uint32_t value)
{
	if (m_loop_sp < static_cast<int32_t>(m_loop_stack.size())) {
		m_loop_stack[static_cast<size_t>(m_loop_sp++)] = value;
		m_loop = value >> 4;
		m_loop_condition = value & 15;
		m_sstat &= ~LOOP_EMPTY;
	} else {
		m_sstat |= LOOP_OVER;
	}
}

void
Adsp2115::loop_stack_pop()
{
	if (m_loop_sp <= 0)
		return;
	if (--m_loop_sp == 0) {
		m_loop = 0xFFFF;
		m_loop_condition = 0;
		m_sstat |= LOOP_EMPTY;
	} else {
		m_loop = m_loop_stack[static_cast<size_t>(m_loop_sp - 1)] >> 4;
		m_loop_condition = m_loop_stack[static_cast<size_t>(m_loop_sp - 1)] & 15;
	}
}

void
Adsp2115::stat_stack_push()
{
	if (m_stat_sp < static_cast<int32_t>(m_stat_stack.size())) {
		m_stat_stack[static_cast<size_t>(m_stat_sp++)] = { static_cast<uint16_t>(m_mstat), m_imask,
		                                                   static_cast<uint16_t>(m_astat) };
		m_sstat &= ~STATUS_EMPTY;
	} else {
		m_sstat |= STATUS_OVER;
	}
}

void
Adsp2115::stat_stack_pop()
{
	if (m_stat_sp > 0 && --m_stat_sp == 0)
		m_sstat |= STATUS_EMPTY;

	const auto &top = m_stat_stack[static_cast<size_t>(m_stat_sp)];

	m_mstat = top[0];
	update_mstat();
	m_imask = top[1];
	m_astat = top[2];
	check_irqs();
}

bool
Adsp2115::condition(unsigned c)
{
	const bool az = m_astat & ZFLAG, an = m_astat & NFLAG, av = m_astat & VFLAG, ac = m_astat & CFLAG;
	const bool mv = m_astat & MVFLAG, as = m_astat & SFLAG;

	switch (c & 15) {
	case 0: return az;
	case 1: return !az;
	case 2: return !((an != av) || az);
	case 3: return (an != av) || az;
	case 4: return an != av;
	case 5: return an == av;
	case 6: return av;
	case 7: return !av;
	case 8: return ac;
	case 9: return !ac;
	case 10: return as;
	case 11: return !as;
	case 12: return mv;
	case 13: return !mv;
	case 14:
		if (static_cast<int32_t>(--m_cntr) > 0)
			return true;
		cntr_stack_pop();
		return false;
	default: return true;
	}
}

void
Adsp2115::update_l(unsigned n)
{
	m_lmask[n] = mask_of(m_l[n] & 0x3FFF);
	m_base[n] = m_i[n] & m_lmask[n];
}

void
Adsp2115::set_i(unsigned n, uint16_t v)
{
	m_i[n & 7] = v & 0x3FFFu;
	update_i(n & 7);
}

void
Adsp2115::post_modify(unsigned ireg, unsigned mreg)
{
	const uint32_t base = m_base[ireg], l = m_l[ireg];
	uint32_t i = (m_i[ireg] + static_cast<uint32_t>(m_m[mreg])) & 0x3FFF;

	if (i < base)
		i += l;
	else if (i >= base + l)
		i -= l;
	m_i[ireg] = i;
}

uint16_t
Adsp2115::dag_read(unsigned ireg, unsigned mreg, bool reverse)
{
	const uint32_t i = m_i[ireg];
	const uint16_t v = data_read(reverse ? reverse14(i & 0x3FFF) : i);

	post_modify(ireg, mreg);
	return v;
}

void
Adsp2115::dag_write(unsigned ireg, unsigned mreg, bool reverse, uint16_t value)
{
	const uint32_t i = m_i[ireg];

	data_write(reverse ? reverse14(i & 0x3FFF) : i, value);
	post_modify(ireg, mreg);
}

uint16_t
Adsp2115::pm_dag_read(uint32_t op)
{
	const unsigned ireg = 4 + (op >> 2 & 3);
	const uint32_t v = program_read(m_i[ireg]);

	m_px = static_cast<uint8_t>(v);
	post_modify(ireg, 4 + (op & 3));
	return static_cast<uint16_t>(v >> 8);
}

void
Adsp2115::pm_dag_write(uint32_t op, int32_t value)
{
	const unsigned ireg = 4 + (op >> 2 & 3);

	program_write(m_i[ireg], (static_cast<uint32_t>(value) << 8) | m_px);
	post_modify(ireg, 4 + (op & 3));
}

void
Adsp2115::write_reg(unsigned group, unsigned reg, int32_t value)
{
	const auto u = static_cast<uint16_t>(value);

	switch (group) {
	case 0:
		switch (reg) {
		case 0x0: m_core.ax0 = u; break;
		case 0x1: m_core.ax1 = u; break;
		case 0x2: m_core.mx0 = u; break;
		case 0x3: m_core.mx1 = u; break;
		case 0x4: m_core.ay0 = u; break;
		case 0x5: m_core.ay1 = u; break;
		case 0x6: m_core.my0 = u; break;
		case 0x7: m_core.my1 = u; break;
		case 0x8: m_core.si = u; break;
		case 0x9: m_core.se = static_cast<uint16_t>(static_cast<int8_t>(value)); break;
		case 0xA: m_core.ar = u; break;
		case 0xB: set_mr_word(m_core.mr, 0, u); break;
		case 0xC:
			set_mr_word(m_core.mr, 1, u);
			set_mr_word(m_core.mr, 2, (u & 0x8000) ? 0xFFFF : 0);
			break;
		case 0xD: set_mr_word(m_core.mr, 2, static_cast<uint16_t>(static_cast<int8_t>(value))); break;
		case 0xE: m_core.sr = (m_core.sr & 0xFFFF0000u) | u; break;
		default: m_core.sr = (m_core.sr & 0xFFFFu) | static_cast<uint32_t>(u) << 16; break;
		}
		break;
	case 1:
	case 2: {
		const unsigned n = (group == 2 ? 4 : 0) + (reg & 3);

		switch (reg >> 2) {
		case 0:
			m_i[n] = static_cast<uint32_t>(value) & 0x3FFF;
			update_i(n);
			break;
		case 1:
			m_m[n] = sext14(static_cast<uint32_t>(value));
			break;
		case 2:
			m_l[n] = static_cast<uint32_t>(value) & 0x3FFF;
			update_l(n);
			break;
		default:
			break;
		}
		break;
	}
	default:
		switch (reg) {
		case 0x0: m_astat = static_cast<uint32_t>(value) & 0xFF; break;
		case 0x1:
			m_mstat = static_cast<uint32_t>(value) & 0x7F;
			update_mstat();
			break;
		case 0x3:
			m_imask = static_cast<uint16_t>(value & 0x3F);
			check_irqs();
			break;
		case 0x4:
			m_icntl = static_cast<uint8_t>(value & 0x1F);
			check_irqs();
			break;
		case 0x5:
			cntr_stack_push();
			m_cntr = static_cast<uint32_t>(value) & 0x3FFF;
			break;
		case 0x6: m_core.sb = static_cast<uint16_t>(static_cast<int32_t>(static_cast<uint32_t>(value) << 27) >> 27); break;
		case 0x7: m_px = static_cast<uint8_t>(value); break;
		case 0x9: m_bus.sport_tx(0, u); break;
		case 0xB: m_bus.sport_tx(1, u); break;
		case 0xC:

			m_ifc = u;
			if (u & 0x0002) m_irq_latch[IRQ0] = 0;
			if (u & 0x0004) m_irq_latch[IRQ1] = 0;
			if (u & 0x0008) m_irq_latch[SPORT0_RX] = 0;
			if (u & 0x0010) m_irq_latch[SPORT0_TX] = 0;
			if (u & 0x0020) m_irq_latch[IRQ2] = 0;
			if (u & 0x0080) m_irq_latch[IRQ0] = 1;
			if (u & 0x0100) m_irq_latch[IRQ1] = 1;
			if (u & 0x0200) m_irq_latch[SPORT0_RX] = 1;
			if (u & 0x0400) m_irq_latch[SPORT0_TX] = 1;
			if (u & 0x0800) m_irq_latch[IRQ2] = 1;
			check_irqs();
			break;
		case 0xD: m_cntr = static_cast<uint32_t>(value) & 0x3FFF; break;
		case 0xF: pc_stack_push(static_cast<uint32_t>(value) & 0x3FFF); break;
		default: break;
		}
		break;
	}
}

int32_t
Adsp2115::read_reg(unsigned group, unsigned reg)
{
	switch (group) {
	case 0:
		switch (reg) {
		case 0x0: return s16(m_core.ax0);
		case 0x1: return s16(m_core.ax1);
		case 0x2: return s16(m_core.mx0);
		case 0x3: return s16(m_core.mx1);
		case 0x4: return s16(m_core.ay0);
		case 0x5: return s16(m_core.ay1);
		case 0x6: return s16(m_core.my0);
		case 0x7: return s16(m_core.my1);
		case 0x8: return s16(m_core.si);
		case 0x9: return s16(m_core.se);
		case 0xA: return s16(m_core.ar);
		case 0xB: return s16(mr_word(m_core.mr, 0));
		case 0xC: return s16(mr_word(m_core.mr, 1));
		case 0xD: return s16(mr_word(m_core.mr, 2));
		case 0xE: return s16(static_cast<uint16_t>(m_core.sr));
		default: return s16(static_cast<uint16_t>(m_core.sr >> 16));
		}
	case 1:
	case 2: {
		const unsigned n = (group == 2 ? 4 : 0) + (reg & 3);

		switch (reg >> 2) {
		case 0: return static_cast<int32_t>(m_i[n]);
		case 1: return m_m[n];
		default: return static_cast<int32_t>(m_l[n]);
		}
	}
	default:
		switch (reg) {
		case 0x0: return static_cast<int32_t>(m_astat);
		case 0x1: return static_cast<int32_t>(m_mstat);
		case 0x2: return static_cast<int32_t>(m_sstat);
		case 0x3: return m_imask;
		case 0x4: return m_icntl;
		case 0x5: return static_cast<int32_t>(m_cntr);
		case 0x6: return s16(m_core.sb);
		case 0x7: return m_px;
		case 0x8: return static_cast<int32_t>(m_bus.sport_rx(0));
		case 0xA: return static_cast<int32_t>(m_bus.sport_rx(1));
		case 0xF: return static_cast<int32_t>(pc_stack_pop());
		default: return 0;
		}
	}
}

uint16_t
Adsp2115::alu_x(unsigned x) const
{
	switch (x & 7) {
	case 0: return m_core.ax0;
	case 1: return m_core.ax1;
	case 2: return m_core.ar;
	case 3: return mr_word(m_core.mr, 0);
	case 4: return mr_word(m_core.mr, 1);
	case 5: return mr_word(m_core.mr, 2);
	case 6: return static_cast<uint16_t>(m_core.sr);
	default: return static_cast<uint16_t>(m_core.sr >> 16);
	}
}

uint16_t
Adsp2115::alu_y(unsigned y) const
{
	switch (y & 3) {
	case 0: return m_core.ay0;
	case 1: return m_core.ay1;
	case 2: return m_core.af;
	default: return 0;
	}
}

uint16_t
Adsp2115::mac_x(unsigned x) const
{
	switch (x & 7) {
	case 0: return m_core.mx0;
	case 1: return m_core.mx1;
	default: return alu_x(x);
	}
}

uint16_t
Adsp2115::mac_y(unsigned y) const
{
	switch (y & 3) {
	case 0: return m_core.my0;
	case 1: return m_core.my1;
	case 2: return m_core.mf;
	default: return 0;
	}
}

uint16_t
Adsp2115::shift_x(unsigned x) const
{
	return (x & 7) < 2 ? m_core.si : alu_x(x);
}

int32_t
Adsp2115::alu(uint32_t op)
{
	const int32_t x = alu_x(bits(op, 8, 3)), y = alu_y(bits(op, 11, 2));
	const int32_t c = (m_astat & CFLAG) ? 1 : 0;
	const auto clear = [this] { m_astat &= m_astat_clear; };
	const auto nz = [this, &clear](int32_t r) {
		clear();
		if ((r & 0xFFFF) == 0)
			m_astat |= ZFLAG;
		m_astat |= static_cast<uint32_t>(r >> 14) & NFLAG;
	};
	const auto nzv = [this, &nz](int32_t s, int32_t d, int32_t r) {
		nz(r);
		m_astat |= static_cast<uint32_t>((s ^ d ^ r ^ (r >> 1)) >> 13) & VFLAG;
	};
	const auto add = [this, &nzv](int32_t s, int32_t d, int32_t r) {
		nzv(s, d, r);
		m_astat |= static_cast<uint32_t>(r >> 13) & CFLAG;
	};
	const auto sub = [this, &nzv](int32_t s, int32_t d, int32_t r) {
		nzv(s, d, r);
		m_astat |= static_cast<uint32_t>(~r >> 13) & CFLAG;
	};
	int32_t r = 0;

	switch (bits(op, 13, 4)) {
	case 0x0:
		r = y;
		nz(r);
		break;
	case 0x1:
		r = y + 1;
		nz(r);
		if (y == 0x7FFF)
			m_astat |= VFLAG;
		else if (y == 0xFFFF)
			m_astat |= CFLAG;
		break;
	case 0x2:
		r = x + y + c;
		add(x, y + c, r);
		break;
	case 0x3:
		r = x + y;
		add(x, y, r);
		break;
	case 0x4:
		r = y ^ 0xFFFF;
		nz(r);
		break;
	case 0x5:
		r = -y;
		nz(r);
		if (y == 0x8000)
			m_astat |= VFLAG;
		if (y == 0)
			m_astat |= CFLAG;
		break;
	case 0x6:
		r = x - y + c - 1;
		sub(x, y, r);
		break;
	case 0x7:
		r = x - y;
		sub(x, y, r);
		break;
	case 0x8:
		r = y - 1;
		nz(r);
		if (y == 0x8000)
			m_astat |= VFLAG;
		else if (y == 0)
			m_astat |= CFLAG;
		break;
	case 0x9:
		r = y - x;
		sub(y, x, r);
		break;
	case 0xA:
		r = y - x + c - 1;
		sub(y, x, r);
		break;
	case 0xB:
		r = x ^ 0xFFFF;
		nz(r);
		break;
	case 0xC:
		r = x & y;
		nz(r);
		break;
	case 0xD:
		r = x | y;
		nz(r);
		break;
	case 0xE:
		r = x ^ y;
		nz(r);
		break;
	default:
		r = (x & 0x8000) ? -x : x;
		clear();
		m_astat &= ~SFLAG;
		if (x == 0)
			m_astat |= ZFLAG;
		if (x == 0x8000)
			m_astat |= NFLAG | VFLAG;
		if (x & 0x8000)
			m_astat |= SFLAG;
		break;
	}
	return r;
}

void
Adsp2115::alu_ar(uint32_t op)
{
	int32_t r = alu(op);

	if ((m_mstat & MSTAT_SATURATE) && (m_astat & VFLAG))
		r = (m_astat & CFLAG) ? -32768 : 32767;
	m_core.ar = static_cast<uint16_t>(r);
}

int64_t
Adsp2115::mac(uint32_t op, bool &done)
{
	const unsigned shift = (m_mstat & MSTAT_INTEGER) ? 0 : 1;
	const unsigned xr = bits(op, 8, 3), yr = bits(op, 11, 2), kind = bits(op, 13, 4);
	const auto mr = static_cast<int64_t>(m_core.mr);

	done = kind != 0;
	if (!done)
		return 0;

	const unsigned form = kind < 4 ? 0 : kind & 3;
	const int64_t x = (form & 2) ? mac_x(xr) : s16(mac_x(xr));
	const int64_t y = (form & 1) ? mac_y(yr) : s16(mac_y(yr));
	const auto product = static_cast<int32_t>(static_cast<uint32_t>(x * y) << shift);
	int64_t r;

	switch (kind) {
	case 1: case 4: case 5: case 6: case 7: r = product; break;
	case 2: case 8: case 9: case 10: case 11: r = mr + product; break;
	default: r = mr - product; break;
	}
	if (kind <= 3) {
		r += 0x8000;
		if ((product & 0xFFFF) == 0x8000)
			r &= ~static_cast<int64_t>(0x10000);
	}
	return r;
}

void
Adsp2115::mac_mr(uint32_t op)
{
	bool done;
	const int64_t r = mac(op, done);

	if (!done)
		return;

	const uint32_t top = static_cast<uint32_t>(static_cast<uint64_t>(r) >> 31) & 0x1FF;

	m_astat &= ~MVFLAG;
	if (top != 0 && top != 0x1FF)
		m_astat |= MVFLAG;
	m_core.mr = static_cast<uint64_t>(r);
}

void
Adsp2115::mac_mf(uint32_t op)
{
	bool done;
	const int64_t r = mac(op, done);

	if (done)
		m_core.mf = static_cast<uint16_t>(static_cast<uint32_t>(r) >> 16);
}

void
Adsp2115::compute(uint32_t op)
{
	switch (bits(op, 17, 2)) {
	case 0: mac_mr(op); break;
	case 1: alu_ar(op); break;
	case 2: mac_mf(op); break;
	default: alu_af(op); break;
	}
}

void
Adsp2115::shift(uint32_t op, int8_t sc, bool immediate)
{
	const unsigned xr = bits(op, 8, 3), kind = bits(op, 11, 4);
	const uint32_t xu = shift_x(xr);
	const int32_t xs = s16(shift_x(xr));
	uint32_t r = 0;
	const auto left = [sc](uint32_t v) -> uint32_t { return sc < 32 ? v << sc : 0; };

	if (immediate && kind >= 0xC)
		return;
	switch (kind >> 1) {
	case 0:
		r = sc > 0 ? left(xu << 16) : sc > -32 ? (xu << 16) >> -sc : 0;
		break;
	case 1:
		r = sc > 0 ? left(xu) : sc > -32 ? xu >> -sc : 0;
		break;
	case 2: {
		const int32_t v = static_cast<int32_t>(static_cast<uint32_t>(xs) << 16);

		r = sc > 0 ? left(static_cast<uint32_t>(v)) : static_cast<uint32_t>(sc > -32 ? v >> -sc : v >> 31);
		break;
	}
	case 3:
		r = sc > 0 ? left(static_cast<uint32_t>(xs)) : static_cast<uint32_t>(sc > -32 ? xs >> -sc : xs >> 31);
		break;
	case 4: {
		const uint32_t v = static_cast<uint32_t>(xs) << 16;

		if (sc > 0) {
			const auto w = static_cast<int32_t>((v >> 1) | (m_astat & CFLAG) << 28);

			r = static_cast<uint32_t>(sc - 1 < 32 ? w >> (sc - 1) : w >> 31);
		} else {
			r = sc > -32 ? v << -sc : 0;
		}
		break;
	}
	case 5:
		r = sc > 0 ? (sc < 32 ? xu >> sc : 0) : sc > -32 ? xu << -sc : 0;
		break;
	default:
		break;
	}
	if (kind < 0xC) {
		if (kind & 1)
			m_core.sr |= r;
		else
			m_core.sr = r;
		return;
	}

	const auto exponent = [](int32_t v) {
		return v < 0 ? std::countl_one(static_cast<uint32_t>(v)) - 17 : std::countl_zero(static_cast<uint32_t>(v)) - 17;
	};

	switch (kind) {
	case 0xC:
		m_astat = xs < 0 ? m_astat | SSFLAG : m_astat & ~SSFLAG;
		m_core.se = static_cast<uint16_t>(-exponent(xs));
		break;
	case 0xD:
		if (m_astat & VFLAG) {
			m_core.se = 1;
			m_astat = xs < 0 ? m_astat & ~SSFLAG : m_astat | SSFLAG;
		} else {
			m_astat = xs < 0 ? m_astat | SSFLAG : m_astat & ~SSFLAG;
			m_core.se = static_cast<uint16_t>(-exponent(xs));
		}
		break;
	case 0xE:
		if (s16(m_core.se) == -15) {
			const uint32_t v = static_cast<uint32_t>((m_astat & SSFLAG) ? ~xs : xs) & 0xFFFF;

			m_core.se = static_cast<uint16_t>(-(std::countl_zero(v) - 1));
		}
		break;
	default: {
		const int32_t e = std::countl_zero(static_cast<uint32_t>(xs < 0 ? ~xs : xs)) - 17;

		if (e < -s16(m_core.sb))
			m_core.sb = static_cast<uint16_t>(-e);
		break;
	}
	}
}

bool
Adsp2115::generate_irq(unsigned line, unsigned index)
{
	if (!(m_imask & (0x20u >> index)))
		return false;
	m_irq_latch[line] = 0;
	pc_stack_push(m_pc);
	stat_stack_push();
	m_pc = 4 + index * 4;
	m_idle = 0;
	if (m_icntl & 0x10)
		m_imask = static_cast<uint16_t>(m_imask & ~(0x3FFu >> index));
	else
		m_imask = static_cast<uint16_t>(m_imask & ~0x3FFu);
	return true;
}

void
Adsp2115::check_irqs()
{
	const auto level = [this](unsigned line, unsigned edge_bit) {
		return (m_icntl & edge_bit) ? m_irq_latch[line] != 0 : m_irq_state[line] != 0;
	};

	if (level(IRQ2, 4) && generate_irq(IRQ2, 0))
		return;
	if (m_irq_latch[SPORT0_TX] && generate_irq(SPORT0_TX, 1))
		return;
	if (m_irq_latch[SPORT0_RX] && generate_irq(SPORT0_RX, 2))
		return;
	if (level(IRQ1, 2) && generate_irq(IRQ1, 3))
		return;
	if (level(IRQ0, 1) && generate_irq(IRQ0, 4))
		return;
	if (m_irq_latch[TIMER])
		generate_irq(TIMER, 5);
}

void
Adsp2115::set_irq(unsigned line, bool asserted)
{
	if (line >= LINES)
		return;
	if (asserted && !m_irq_state[line])
		m_irq_latch[line] = 1;
	m_irq_state[line] = asserted;
	m_irq_pulsed[line] = 0;
}

void
Adsp2115::pulse_irq(unsigned line)
{
	if (line >= LINES)
		return;
	set_irq(line, true);
	m_irq_pulsed[line] = 1;
}

uint32_t
Adsp2115::run(uint32_t cycles)
{
	const uint64_t start = m_cycles;

	check_irqs();
	for (unsigned line = 0; line < LINES; line++)
		if (m_irq_pulsed[line]) {
			m_irq_pulsed[line] = 0;
			m_irq_state[line] = 0;
		}
	if (m_idle) {
		m_cycles += cycles;
		return cycles;
	}
	m_icount = cycles;
	m_spin = false;
	while (m_icount > 0) {
		if (m_pc == m_wait_pc && m_bus.waiting(m_pc)) {
			m_cycles += static_cast<uint64_t>(m_icount);
			m_icount = 0;
			break;
		}
		m_ppc = m_pc;

		const uint32_t op = program_read(m_pc);

		if (m_pc != m_loop) {
			m_pc++;
		} else if (condition(m_loop_condition)) {
			m_pc = pc_stack_top();
		} else {
			loop_stack_pop();
			pc_stack_pop();
			m_pc++;
		}
		execute(op);
		m_icount--;
		m_cycles++;
		if (m_idle || m_spin) {
			m_cycles += static_cast<uint64_t>(m_icount > 0 ? m_icount : 0);
			m_icount = 0;
		}
	}
	return static_cast<uint32_t>(m_cycles - start);
}

void
Adsp2115::execute(uint32_t op)
{
	const unsigned top = bits(op, 16, 8);
	const unsigned dreg = bits(op, 4, 4), cond = bits(op, 0, 4);

	switch (top) {
	case 0x00:
	case 0x01:
	case 0x08:
		break;
	case 0x02:
		if (op & 0x8000) {
			m_idle = 1;
		} else if (condition(cond)) {
			if (op & 0x020) m_flagout = 0;
			if (op & 0x010) m_flagout ^= 1;
			if (op & 0x080) m_fl0 = 0;
			if (op & 0x040) m_fl0 ^= 1;
			if (op & 0x200) m_fl1 = 0;
			if (op & 0x100) m_fl1 ^= 1;
			if (op & 0x800) m_fl2 = 0;
			if (op & 0x400) m_fl2 ^= 1;
		}
		break;
	case 0x03:
		if ((op & 2) ? m_flagin : !m_flagin) {
			if (op & 1)
				pc_stack_push(m_pc);
			m_pc = bits(op, 4, 12) | bits(op, 2, 2) << 12;
		}
		break;
	case 0x04:
		if (op & 0x10)
			pc_stack_pop();
		if (op & 0x08)
			loop_stack_pop();
		if (op & 0x04)
			cntr_stack_pop();
		if (op & 0x02) {
			if (op & 0x01)
				stat_stack_pop();
			else
				stat_stack_push();
		}
		break;
	case 0x05:
		if (m_astat & MVFLAG) {
			const bool negative = mr_word(m_core.mr, 2) & 0x80;

			set_mr_word(m_core.mr, 2, negative ? 0xFFFF : 0x0000);
			set_mr_word(m_core.mr, 1, negative ? 0x8000 : 0x7FFF);
			set_mr_word(m_core.mr, 0, negative ? 0x0000 : 0xFFFF);
		}
		break;
	case 0x06: {
		const uint32_t x = alu_x(bits(op, 8, 3)), y = alu_y(bits(op, 11, 2));
		const uint32_t t = x ^ y;

		m_astat = (m_astat & ~QFLAG) | ((t >> 10) & QFLAG);
		m_core.af = static_cast<uint16_t>((y << 1) | (m_core.ay0 >> 15));
		m_core.ay0 = static_cast<uint16_t>((m_core.ay0 << 1) | (t >> 15));
		break;
	}
	case 0x07: {
		const int32_t x = alu_x(bits(op, 8, 3));
		const int32_t r = (m_astat & QFLAG) ? m_core.af + x : m_core.af - x;
		const auto t = static_cast<uint32_t>(r ^ x);

		m_astat = (m_astat & ~QFLAG) | ((t >> 10) & QFLAG);
		m_core.af = static_cast<uint16_t>((static_cast<uint32_t>(r) << 1) | (m_core.ay0 >> 15));
		m_core.ay0 = static_cast<uint16_t>((m_core.ay0 << 1) | ((~t >> 15) & 1));
		break;
	}
	case 0x09: {
		const unsigned ireg = bits(op, 2, 3);

		post_modify(ireg, (ireg & 4) | (op & 3));
		break;
	}
	case 0x0A:
		if (condition(cond)) {
			m_pc = pc_stack_pop();
			if (op & 0x10)
				stat_stack_pop();
		}
		break;
	case 0x0B:
		if (condition(cond)) {
			if (op & 0x10)
				pc_stack_push(m_pc);
			m_pc = m_i[4 + bits(op, 6, 2)] & 0x3FFF;
		}
		break;
	case 0x0C:
		if (op & 0x0008) m_mstat = (op & 0x0004) ? m_mstat | MSTAT_GOMODE : m_mstat & ~MSTAT_GOMODE;
		if (op & 0x2000) m_mstat = (op & 0x1000) ? m_mstat | MSTAT_INTEGER : m_mstat & ~MSTAT_INTEGER;
		if (op & 0x8000) m_mstat = (op & 0x4000) ? m_mstat | MSTAT_TIMER : m_mstat & ~MSTAT_TIMER;
		if (op & 0x0020) m_mstat = (op & 0x0010) ? m_mstat | MSTAT_BANK : m_mstat & ~MSTAT_BANK;
		if (op & 0x0080) m_mstat = (op & 0x0040) ? m_mstat | MSTAT_REVERSE : m_mstat & ~MSTAT_REVERSE;
		if (op & 0x0200) m_mstat = (op & 0x0100) ? m_mstat | MSTAT_STICKYV : m_mstat & ~MSTAT_STICKYV;
		if (op & 0x0800) m_mstat = (op & 0x0400) ? m_mstat | MSTAT_SATURATE : m_mstat & ~MSTAT_SATURATE;
		update_mstat();
		break;
	case 0x0D:
		write_reg(bits(op, 10, 2), dreg, read_reg(bits(op, 8, 2), cond));
		break;
	case 0x0E:
		if (condition(cond))
			shift(op, static_cast<int8_t>(m_core.se), false);
		break;
	case 0x0F:
		shift(op, static_cast<int8_t>(op), true);
		break;
	case 0x10:
		shift(op, static_cast<int8_t>(m_core.se), false);
		write_reg(0, dreg, read_reg(0, cond));
		break;
	case 0x11:
	case 0x12:
	case 0x13: {
		const auto read = [&]() -> uint16_t {
			return top == 0x11 ? pm_dag_read(op) : top == 0x12 ? dag1_read(op) : dag2_read(op);
		};

		if (op & 0x8000) {
			const int32_t v = read_reg(0, dreg);

			if (top == 0x11)
				pm_dag_write(op, v);
			else if (top == 0x12)
				dag1_write(op, v);
			else
				dag2_write(op, v);
			shift(op, static_cast<int8_t>(m_core.se), false);
		} else {
			shift(op, static_cast<int8_t>(m_core.se), false);
			write_reg(0, dreg, read());
		}
		break;
	}
	default:
		if (top >= 0x14 && top <= 0x17) {
			loop_stack_push(op & 0x3FFFF);
			pc_stack_push(m_pc);
		} else if (top >= 0x18 && top <= 0x1B) {
			if (condition(cond)) {
				m_pc = bits(op, 4, 14);
				if (m_pc == m_ppc)
					m_spin = true;
			}
		} else if (top >= 0x1C && top <= 0x1F) {
			if (condition(cond)) {
				pc_stack_push(m_pc);
				m_pc = bits(op, 4, 14);
			}
		} else if (top >= 0x20 && top <= 0x27) {
			if (condition(cond))
				compute(op);
		} else if (top >= 0x28 && top <= 0x2F) {
			const int32_t v = read_reg(0, cond);

			compute(op);
			write_reg(0, dreg, v);
		} else if (top >= 0x30 && top <= 0x3F) {
			write_reg(bits(op, 18, 2), cond, sext14(op >> 4));
		} else if (top >= 0x40 && top <= 0x4F) {
			write_reg(0, cond, static_cast<int32_t>(bits(op, 4, 16)));
		} else if (top >= 0x50 && top <= 0x7F) {
			const unsigned where = bits(op, 20, 2);

			if (op & 0x80000) {
				const int32_t v = read_reg(0, dreg);

				if (where == 1)
					pm_dag_write(op, v);
				else if (where == 2)
					dag1_write(op, v);
				else
					dag2_write(op, v);
				compute(op);
			} else {
				compute(op);
				write_reg(0, dreg, where == 1 ? pm_dag_read(op) : where == 2 ? dag1_read(op) : dag2_read(op));
			}
		} else if (top >= 0x80 && top <= 0x8F) {
			write_reg(bits(op, 18, 2), cond, data_read(bits(op, 4, 14)));
		} else if (top >= 0x90 && top <= 0x9F) {
			data_write(bits(op, 4, 14), static_cast<uint16_t>(read_reg(bits(op, 18, 2), cond)));
		} else if (top >= 0xA0 && top <= 0xAF) {
			dag1_write(op, static_cast<int32_t>(bits(op, 4, 16)));
		} else if (top >= 0xB0 && top <= 0xBF) {
			dag2_write(op, static_cast<int32_t>(bits(op, 4, 16)));
		} else {
			uint16_t *const xs[] = { &m_core.ax0, &m_core.ax1, &m_core.mx0, &m_core.mx1 };
			uint16_t *const ys[] = { &m_core.ay0, &m_core.ay1, &m_core.my0, &m_core.my1 };

			if (op & 0x20000)
				alu_ar(op);
			else
				mac_mr(op);
			*xs[bits(op, 18, 2)] = dag1_read(op);
			*ys[bits(op, 20, 2)] = pm_dag_read(op >> 4);
		}
		break;
	}
}

void
Adsp2115::save(Adsp2115State &out) const
{
	out.pm = m_pm;
	out.dm = m_dm;
	out.control = m_control;
	out.core = m_core;
	out.alt = m_alt;
	out.i = m_i;
	out.l = m_l;
	out.base = m_base;
	out.m = m_m;
	out.pc_stack = m_pc_stack;
	out.cntr_stack = m_cntr_stack;
	out.loop_stack = m_loop_stack;
	out.stat_stack = m_stat_stack;
	out.pc = m_pc;
	out.loop = m_loop;
	out.loop_condition = m_loop_condition;
	out.cntr = m_cntr;
	out.astat = m_astat;
	out.sstat = m_sstat;
	out.mstat = m_mstat;
	out.pc_sp = m_pc_sp;
	out.cntr_sp = m_cntr_sp;
	out.stat_sp = m_stat_sp;
	out.loop_sp = m_loop_sp;
	out.imask = m_imask;
	out.ifc = m_ifc;
	out.icntl = m_icntl;
	out.px = m_px;
	out.flagin = m_flagin;
	out.flagout = m_flagout;
	out.fl0 = m_fl0;
	out.fl1 = m_fl1;
	out.fl2 = m_fl2;
	out.idle = m_idle;
	out.irq_state = m_irq_state;
	out.irq_latch = m_irq_latch;
	out.irq_pulsed = m_irq_pulsed;
	out.cycles = m_cycles;
}

void
Adsp2115::load(const Adsp2115State &in)
{
	m_pm = in.pm;
	m_dm = in.dm;
	m_control = in.control;
	m_core = in.core;
	m_alt = in.alt;
	m_i = in.i;
	m_l = in.l;
	m_m = in.m;
	for (unsigned n = 0; n < 8; n++)
		m_lmask[n] = mask_of(m_l[n]);
	m_base = in.base;
	m_pc_stack = in.pc_stack;
	m_cntr_stack = in.cntr_stack;
	m_loop_stack = in.loop_stack;
	m_stat_stack = in.stat_stack;
	m_pc = m_ppc = in.pc;
	m_loop = in.loop;
	m_loop_condition = in.loop_condition;
	m_cntr = in.cntr;
	m_astat = in.astat;
	m_sstat = in.sstat;
	m_mstat = m_mstat_prev = in.mstat;
	m_astat_clear = (m_mstat & MSTAT_STICKYV) ? ~(CFLAG | NFLAG | ZFLAG) : ~(CFLAG | VFLAG | NFLAG | ZFLAG);
	m_pc_sp = in.pc_sp;
	m_cntr_sp = in.cntr_sp;
	m_stat_sp = in.stat_sp;
	m_loop_sp = in.loop_sp;
	m_imask = in.imask;
	m_ifc = in.ifc;
	m_icntl = in.icntl;
	m_px = in.px;
	m_flagin = in.flagin;
	m_flagout = in.flagout;
	m_fl0 = in.fl0;
	m_fl1 = in.fl1;
	m_fl2 = in.fl2;
	m_idle = in.idle;
	m_irq_state = in.irq_state;
	m_irq_latch = in.irq_latch;
	m_irq_pulsed = in.irq_pulsed;
	m_cycles = in.cycles;
}
}
