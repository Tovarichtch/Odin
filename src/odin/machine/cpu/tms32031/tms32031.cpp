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

#include "odin/machine/cpu/tms32031/tms32031.hpp"

#include <bit>
#include <cmath>
#include <cstring>

namespace odin {
namespace {
constexpr uint32_t RAM_BASE = 0x809800, BUS_CONTROL = 0x808064;

constexpr uint32_t HIZ = 0x004, HOLDST = 0x001, BUS_WRITABLE = 0x1FFE;

uint32_t
reversed(uint32_t v)
{
	uint32_t out = 0;

	for (unsigned b = 0; b < 32; b++)
		out |= ((v >> b) & 1) << (31 - b);
	return out;
}
}

Tms32031::Tms32031(Tms32031Bus &bus, std::span<const uint8_t> boot)
	: m_bus(bus)
{
	for (size_t i = 0; i < m_boot.size() && i * 4 + 3 < boot.size(); i++)
		m_boot[i] = static_cast<uint32_t>(boot[i * 4]) | static_cast<uint32_t>(boot[i * 4 + 1]) << 8 |
		            static_cast<uint32_t>(boot[i * 4 + 2]) << 16 | static_cast<uint32_t>(boot[i * 4 + 3]) << 24;
	reset();
}

void
Tms32031::reset()
{
	m_pc = read(0);
	ireg(IE) = 0;
	ireg(IF) = m_irq_state & 0x0F;
	ireg(ST) = 0;
	ireg(IOF) = 0;
	m_bus_control = 0x000010F8;
	m_delayed = m_irq_pending = m_idling = m_low_power = false;
}

void
Tms32031::set_reset(bool held)
{
	if (m_reset_held && !held)
		reset();
	m_reset_held = held;
}

void
Tms32031::set_int(unsigned line, bool asserted)
{
	const uint16_t bit = static_cast<uint16_t>(1u << (line & 3));

	if (asserted) {
		m_irq_state |= bit;
		ireg(IF) |= bit;
	} else {
		m_irq_state &= static_cast<uint16_t>(~bit);
	}
	ireg(IF) |= m_irq_state & 0x0F;
}

uint32_t
Tms32031::read(uint32_t addr)
{
	addr &= 0xFFFFFF;
	if (addr < 0x1000)
		return m_boot[addr];
	if (addr - RAM_BASE < m_ram.size())
		return m_ram[addr - RAM_BASE];
	if ((addr & 0xFFFF00) == 0x808000)
		return addr == BUS_CONTROL ? m_bus_control : 0;
	return m_bus.read(addr);
}

void
Tms32031::write(uint32_t addr, uint32_t value)
{
	addr &= 0xFFFFFF;
	if (addr - RAM_BASE < m_ram.size()) {
		m_ram[addr - RAM_BASE] = value;
		return;
	}
	if ((addr & 0xFFFF00) == 0x808000) {
		if (addr == BUS_CONTROL) {
			if ((m_bus_control ^ value) & HIZ)
				m_bus_control ^= HOLDST;
			m_bus_control = value & BUS_WRITABLE;
		}
		return;
	}

	m_bus.write(addr, value);
}

void
Tms32031::check_irqs()
{
	const uint32_t live = ireg(IF) & ireg(IE) & 0x0FFF;

	if (!live || !(ireg(ST) & GIE))
		return;
	m_idling = false;
	if (m_delayed) {
		m_irq_pending = true;
		return;
	}

	const unsigned which = static_cast<unsigned>(std::countr_zero(live));

	ireg(IF) &= ~(1u << which);
	trap(which + 1);

	ireg(IF) |= m_irq_state & 0x0F;
}

void
Tms32031::trap(unsigned number)
{
	write(++ireg(SP), m_pc);
	ireg(ST) &= ~GIE;
	m_pc = read(number);
	burn(4);
}

void
Tms32031::update_special(unsigned reg)
{
	if (reg == BK) {
		uint32_t mask = ireg(BK);

		for (uint32_t bits = mask; bits >>= 1;)
			mask |= bits;
		m_bkmask = mask;
	} else if (reg == ST || reg == IF || reg == IE) {
		check_irqs();
	}
}

bool
Tms32031::condition(unsigned which) const
{
	const uint32_t f = m_r[ST].man;
	const bool c = f & C, v = f & V, z = f & Z, n = f & N, uf = f & UF, lv = f & LV, luf = f & LUF;

	switch (which & 31) {
	case 0: return true;
	case 1: return c;
	case 2: return c || z;
	case 3: return !c && !z;
	case 4: return !c;
	case 5: return z;
	case 6: return !z;
	case 7: return n;
	case 8: return n || z;
	case 9: return !n && !z;
	case 10: return !n;
	case 12: return !v;
	case 13: return v;
	case 14: return !uf;
	case 15: return uf;
	case 16: return !lv;
	case 17: return lv;
	case 18: return !luf;
	case 19: return luf;
	case 20: return uf || z;
	default: return false;
	}
}

uint32_t
Tms32031::run(uint32_t cycles)
{
	if (m_reset_held) {
		m_cycles += cycles;
		return cycles;
	}
	m_icount = cycles;
	check_irqs();
	if (m_idling)
		m_icount = 0;
	while (m_icount > 0) {
		if ((ireg(ST) & RM) && m_pc == ireg(RE) + 1) {
			if (static_cast<int32_t>(--ireg(RC)) >= 0) {
				m_pc = ireg(RS);
			} else {
				ireg(ST) &= ~RM;
				if (m_delayed) {
					m_delayed = false;
					if (m_irq_pending) {
						m_irq_pending = false;
						check_irqs();
					}
				}
			}
			continue;
		}
		execute_one();
	}

	const uint32_t spent = static_cast<uint32_t>(static_cast<int64_t>(cycles) - m_icount);

	m_cycles += spent;
	return spent;
}

void
Tms32031::execute_one()
{
	const uint32_t op = read(m_pc);

	burn(1);
	m_pc = (m_pc + 1) & 0xFFFFFF;
	if (op >> 31)
		op >> 30 == 3 ? op_parallel_store(op) : op_parallel_multiply(op);
	else if (op >> 29 == 3)
		op_flow(op);
	else if (op >> 29 == 2)
		op_conditional_load(op);
	else if (op >> 29 == 1)
		op_three(op);
	else
		op_two(op);
}

void
Tms32031::execute_delayed(uint32_t target, bool taken)
{
	m_delayed = true;
	execute_one();
	execute_one();
	execute_one();
	if (taken)
		m_pc = target & 0xFFFFFF;
	m_delayed = false;
	if (m_irq_pending) {
		m_irq_pending = false;
		check_irqs();
	}
}

uint32_t
Tms32031::indirect(unsigned field, uint32_t disp, Deferred *deferred)
{
	const unsigned mode = (field >> 3) & 31, reg = AR0 + (field & 7);
	const uint32_t ar = ireg(reg);
	uint32_t addr = ar, updated = ar;

	if (mode >= 26)
		return 0;
	if (mode == 24)
		return ar;
	if (mode == 25) {
		updated = reversed(reversed(ar) + reversed(ireg(IR0)));
	} else {
		const uint32_t step = mode < 8 ? disp : mode < 16 ? ireg(IR0) : ireg(IR1);

		switch (mode & 7) {
		case 0:
			return ar + step;
		case 1:
			return ar - step;
		case 2:
			addr = updated = ar + step;
			break;
		case 3:
			addr = updated = ar - step;
			break;
		case 4:
			updated = ar + step;
			break;
		case 5:
			updated = ar - step;
			break;
		case 6: {
			int32_t at = static_cast<int32_t>((ar & m_bkmask) + step);

			if (static_cast<uint32_t>(at) >= ireg(BK))
				at = static_cast<int32_t>(static_cast<uint32_t>(at) - ireg(BK));
			updated = (ar & ~m_bkmask) | (static_cast<uint32_t>(at) & m_bkmask);
			break;
		}
		default: {
			int32_t at = static_cast<int32_t>((ar & m_bkmask) - step);

			if (at < 0)
				at = static_cast<int32_t>(static_cast<uint32_t>(at) + ireg(BK));
			updated = (ar & ~m_bkmask) | (static_cast<uint32_t>(at) & m_bkmask);
			break;
		}
		}
	}
	if (deferred)
		*deferred = { reg, updated, true };
	else
		ireg(reg) = updated;
	return addr;
}

Tms32031::Reg
Tms32031::long_to_fp(uint32_t v)
{
	Reg r;

	r.man = v << 8;
	r.set_e(static_cast<int32_t>(v) >> 24);
	return r;
}

Tms32031::Reg
Tms32031::short_to_fp(uint32_t v)
{
	Reg r;

	if ((v & 0xFFFF) == 0x8000) {
		r.man = 0;
		r.exp = -128;
	} else {
		r.man = v << 20;
		r.set_e(static_cast<int16_t>(v) >> 12);
	}
	return r;
}

uint32_t
Tms32031::fp_to_long(const Reg &r)
{
	return static_cast<uint32_t>(r.exp) << 24 | r.man >> 8;
}

float
Tms32031::to_float(uint32_t dsp)
{
	const Reg r = long_to_fp(dsp);
	uint32_t bits;

	if (r.man == 0 && r.exp == -128)
		return 0.0f;
	if (r.m() >= 0)
		bits = (static_cast<uint32_t>(r.exp + 127) << 23) + (r.man >> 8);
	else
		bits = 0x80000000u + (static_cast<uint32_t>(r.exp + 127) << 23) +
		       (static_cast<uint32_t>(static_cast<int32_t>(0u - r.man) >> 8) & 0x00FFFFFF);
	return std::bit_cast<float>(bits);
}

uint32_t
Tms32031::from_float(float value)
{
	const uint64_t bits = std::bit_cast<uint64_t>(static_cast<double>(value));
	const uint32_t hi = static_cast<uint32_t>(bits >> 32), lo = static_cast<uint32_t>(bits);
	const int32_t mantissa = static_cast<int32_t>(((hi & 0x000FFFFF) << 11) | ((lo & 0xFFE00000) >> 21));
	const int exponent = static_cast<int>((hi & 0x7FF00000) >> 20) - 1023;
	Reg r;

	if (exponent < -128) {
		r.man = 0;
		r.exp = -128;
	} else if (exponent > 127) {
		r.man = static_cast<int32_t>(hi) >= 0 ? 0x7FFFFFFF : 0x80000001;
		r.exp = 127;
	} else if (static_cast<int32_t>(hi) >= 0) {
		r.set_m(mantissa);
		r.exp = exponent;
	} else if (mantissa != 0) {
		r.man = 0x80000000u | (0u - static_cast<uint32_t>(mantissa));
		r.exp = exponent;
	} else {
		r.man = 0x80000000u;
		r.set_e(exponent - 1);
	}
	return fp_to_long(r);
}

void
Tms32031::int2float(Reg &r)
{
	uint32_t man = r.man;
	int exp;

	clr(N | Z | V | UF);
	if (man == 0) {
		man = 0x80000000;
		exp = -128;
	} else if (man == 0xFFFFFFFF) {
		man = 0;
		exp = -1;
	} else {
		const int count = static_cast<int32_t>(man) > 0 ? std::countl_zero(man) : std::countl_one(man);

		man <<= count;
		exp = 31 - count;
	}
	r.man = man ^ 0x80000000;
	r.set_e(exp);
	or_nzf(r);
}

void
Tms32031::float2int(Reg &r, bool flags)
{
	const int32_t man = r.m();
	const int shift = 31 - r.exp;

	if (flags)
		clr(N | Z | V | UF);
	if (shift <= 0) {
		r.man = man >= 0 ? 0x7FFFFFFF : 0x80000000;
		if (flags)
			st() |= V | LV;
	} else if (shift > 31) {
		r.set_m(man >> 31);
	} else {
		r.set_m((man >> shift) ^ static_cast<int32_t>(1u << (31 - shift)));
	}
	if (flags)
		or_nz(r.man);
}

void
Tms32031::negf(Reg &dst, Reg src)
{
	const int32_t man = src.m();

	clr(N | Z | V | UF);
	if (src.exp == -128) {
		dst.man = 0;
		dst.exp = -128;
	} else if (man & 0x7FFFFFFF) {
		dst.man = 0u - src.man;
		dst.exp = src.exp;
	} else {
		dst.man = src.man ^ 0x80000000;
		dst.set_e(man == 0 ? src.exp - 1 : src.exp + 1);
	}
	or_nzf(dst);
}

namespace {
void
normalize(int64_t &man, int &exp)
{
	if (man == 0 || exp == -128) {
		exp = -128;
		man = 0x80000000;
	} else if (man >= int64_t(0x100000000) || man < -int64_t(0x100000000)) {
		man >>= 1;
		exp++;
	} else if (man < int64_t(0x80000000) && man >= -int64_t(0x80000000)) {
		const uint32_t low = static_cast<uint32_t>(man);
		const int count = man > 0 ? std::countl_zero(low) : std::countl_one(low);

		man *= int64_t(1) << count;
		exp -= count;
	}
}
}

void
Tms32031::addf(Reg &dst, Reg a, Reg b)
{
	clr(N | Z | V | UF);
	if (a.exp == -128) {
		dst = b;
		or_nzf(dst);
		return;
	}
	if (b.exp == -128) {
		dst = a;
		or_nzf(dst);
		return;
	}

	int64_t m1 = static_cast<int64_t>(a.m()) ^ 0x80000000, m2 = static_cast<int64_t>(b.m()) ^ 0x80000000;
	int exp;

	if (a.exp > b.exp) {
		exp = a.exp;
		if (exp - b.exp >= 32) {
			dst = a;
			or_nzf(dst);
			return;
		}
		m2 >>= exp - b.exp;
	} else {
		exp = b.exp;
		if (exp - a.exp >= 32) {
			dst = b;
			or_nzf(dst);
			return;
		}
		m1 >>= exp - a.exp;
	}

	int64_t man = m1 + m2;

	normalize(man, exp);
	if (exp <= -128) {
		man = 0x80000000;
		exp = -128;
		st() |= UF | LUF;
	} else if (exp > 127) {
		man = man < 0 ? 0x00000000 : 0xFFFFFFFF;
		exp = 127;
		st() |= V | LV;
	}
	dst.man = static_cast<uint32_t>(man) ^ 0x80000000;
	dst.set_e(exp);
	or_nzf(dst);
}

void
Tms32031::subf(Reg &dst, Reg a, Reg b)
{
	clr(N | Z | V | UF);
	if (b.exp == -128) {
		dst = a;
		or_nzf(dst);
		return;
	}

	int64_t m1 = static_cast<int64_t>(a.m()) ^ 0x80000000, m2 = static_cast<int64_t>(b.m()) ^ 0x80000000;
	int exp;

	if (a.exp > b.exp) {
		exp = a.exp;
		if (exp - b.exp >= 32) {
			dst = a;
			or_nzf(dst);
			return;
		}
		m2 >>= exp - b.exp;
	} else {
		exp = b.exp;
		if (exp - a.exp >= 32) {
			negf(dst, b);
			return;
		}
		m1 >>= exp - a.exp;
	}

	int64_t man = m1 - m2;

	normalize(man, exp);
	if (exp <= -128) {
		if (man != 0 || exp < -128)
			st() |= UF | LUF;
		man = 0x80000000;
		exp = -128;
	} else if (exp > 127) {
		man = man < 0 ? 0x00000000 : 0xFFFFFFFF;
		exp = 127;
		st() |= V | LV;
	}
	dst.man = static_cast<uint32_t>(man) ^ 0x80000000;
	dst.set_e(exp);
	or_nzf(dst);
}

void
Tms32031::mpyf(Reg &dst, Reg a, Reg b)
{
	clr(N | Z | V | UF);
	if (a.exp == -128 || b.exp == -128) {
		dst.man = 0;
		dst.exp = -128;
		or_nzf(dst);
		return;
	}

	const int32_t m1 = (a.m() >> 8) ^ 0x800000, m2 = (b.m() >> 8) ^ 0x800000;
	int64_t man = static_cast<int64_t>(m1) * m2;
	int exp = a.exp + b.exp;

	man >>= 46 - 31;
	if (man == 0) {
		exp = -128;
		man = 0x80000000;
	} else if (man >= int64_t(0x100000000)) {
		man >>= 1;
		exp++;
		if (man >= int64_t(0x100000000)) {
			man >>= 1;
			exp++;
		}
	} else if (man < -int64_t(0x100000000)) {
		man >>= 1;
		exp++;
	}
	if (exp <= -128) {
		man = 0x80000000;
		exp = -128;
		st() |= UF | LUF;
	} else if (exp > 127) {
		man = man < 0 ? 0x00000000 : 0xFFFFFFFF;
		exp = 127;
		st() |= V | LV;
	}
	dst.man = static_cast<uint32_t>(man) ^ 0x80000000;
	dst.set_e(exp);
	or_nzf(dst);
}

void
Tms32031::norm(Reg &dst, Reg src)
{
	uint32_t man = src.man;
	int exp = src.exp;

	clr(N | Z | V | UF);
	if (exp == -128 || man == 0) {
		if (man != 0)
			st() |= UF | LUF;
		man = 0;
		exp = -128;
	} else {
		const int count = static_cast<int32_t>(man) > 0 ? std::countl_zero(man) : std::countl_one(man);

		man <<= count;
		exp -= count;
		if (exp <= -128) {
			man = 0;
			exp = -128;
			st() |= UF | LUF;
		}
	}
	dst.man = man;
	dst.set_e(exp);
	or_nzf(dst);
}

void
Tms32031::absf(unsigned dreg, Reg src)
{
	const int32_t man = src.m();
	Reg &dst = m_r[dreg];

	clr(N | Z | V | UF);
	dst = src;
	if (src.exp == -128) {
		dst.man = 0;
	} else if (man < 0) {
		if (src.man != 0x80000000) {
			dst.man = 0u - src.man;
		} else if (src.exp == 127) {
			st() |= V | LV;
			dst.man = 0x7FFFFFFF;
		} else {
			dst.set_e(src.exp + 1);
			dst.man = 0;
		}
	}
	or_nzf(dst);
}

void
Tms32031::rnd(unsigned dreg)
{
	Reg &r = m_r[dreg];

	clr(N | V | UF);
	if (r.m() < 0x7FFFFF80) {
		r.man = (r.man + 0x80) & 0xFFFFFF00;
		or_nuf(r);
	} else if (r.exp < 127) {
		r.man = (r.man + 0x80) & 0x7FFFFF00;
		r.set_e(r.exp + 1);
		or_nuf(r);
	} else {
		r.man = 0x7FFFFF00;
		st() |= V | LV;
	}
}

void
Tms32031::save(Tms32031State &out) const
{
	for (unsigned i = 0; i < 32; i++) {
		out.r[i] = m_r[i].man;
		out.e[i] = m_r[i].exp;
	}
	out.ram = m_ram;
	out.cycles = m_cycles;
	out.pc = m_pc;
	out.bus_control = m_bus_control;
	out.irq_state = m_irq_state;
	out.delayed = m_delayed;
	out.irq_pending = m_irq_pending;
	out.idling = m_idling;
	out.low_power = m_low_power;
	out.reset_held = m_reset_held;
}

void
Tms32031::load(const Tms32031State &in)
{
	for (unsigned i = 0; i < 32; i++) {
		m_r[i].man = in.r[i];
		m_r[i].set_e(in.e[i]);
	}
	m_ram = in.ram;
	m_cycles = in.cycles;
	m_pc = in.pc;
	m_bus_control = in.bus_control;
	m_irq_state = in.irq_state;
	m_delayed = in.delayed;
	m_irq_pending = in.irq_pending;
	m_idling = in.idling;
	m_low_power = in.low_power;
	m_reset_held = in.reset_held;
	update_special(BK);
}
}
