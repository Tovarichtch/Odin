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

#include <algorithm>

namespace odin {
namespace {
int64_t
sext24(uint32_t v)
{
	return static_cast<int32_t>(v << 8) >> 8;
}

int32_t
sext7(uint32_t v)
{
	return static_cast<int32_t>(v << 25) >> 25;
}

bool
overflow_add(uint32_t a, uint32_t b, uint32_t r)
{
	return static_cast<int32_t>((a ^ r) & (b ^ r)) < 0;
}

bool
overflow_sub(uint32_t a, uint32_t b, uint32_t r)
{
	return static_cast<int32_t>((a ^ b) & (a ^ r)) < 0;
}

uint32_t
saturated(uint32_t a)
{
	return static_cast<int32_t>(a) < 0 ? 0x80000000 : 0x7FFFFFFF;
}
}

Tms32031::Reg
Tms32031::float_operand(uint32_t op, unsigned mode, unsigned reg_mask)
{
	switch (mode) {
	case 0:
		return m_r[op & reg_mask];
	case 1:
		return long_to_fp(read(direct(op)));
	case 2:
		return long_to_fp(read(indirect_d(op)));
	default:
		return short_to_fp(op);
	}
}

uint32_t
Tms32031::int_operand(uint32_t op, unsigned mode, bool sign)
{
	switch (mode) {
	case 0:
		return ireg(op & 31);
	case 1:
		return read(direct(op));
	case 2:
		return read(indirect_d(op));
	default:
		return sign ? static_cast<uint32_t>(static_cast<int16_t>(op)) : op & 0xFFFF;
	}
}

void
Tms32031::load_int(unsigned dreg, uint32_t value)
{
	ireg(dreg) = value;
	if (dreg < 8) {
		clr(N | Z | V | UF);
		or_nz(value);
	} else if (dreg >= BK) {
		update_special(dreg);
	}
}

void
Tms32031::logic(unsigned dreg, uint32_t result)
{
	load_int(dreg, result);
}

void
Tms32031::absi(unsigned dreg, uint32_t src)
{
	const uint32_t res = static_cast<int32_t>(src) < 0 ? 0u - src : src;

	ireg(dreg) = !ovm() || res != 0x80000000 ? res : 0x7FFFFFFF;
	if (dreg < 8) {
		clr(N | Z | V | UF);
		or_nz(res);
		if (res == 0x80000000)
			st() |= V | LV;
	} else if (dreg >= BK) {
		update_special(dreg);
	}
}

void
Tms32031::addc(unsigned dreg, uint32_t a, uint32_t b)
{
	const uint32_t carry = st() & C, res = a + b + carry;

	ireg(dreg) = !ovm() || !overflow_add(a, b, res) ? res : saturated(a);
	if (dreg < 8) {
		clr(N | Z | C | V | UF);
		st() |= static_cast<uint64_t>(a) + b + carry > 0xFFFFFFFF ? C : 0;
		or_v_add(a, b, res);
		or_nz(res);
	} else if (dreg >= BK) {
		update_special(dreg);
	}
}

void
Tms32031::addi(unsigned dreg, uint32_t a, uint32_t b)
{
	const uint32_t res = a + b;

	ireg(dreg) = !ovm() || !overflow_add(a, b, res) ? res : saturated(a);
	if (dreg < 8) {
		clr(N | Z | C | V | UF);
		st() |= a > res ? C : 0;
		or_v_add(a, b, res);
		or_nz(res);
	} else if (dreg >= BK) {
		update_special(dreg);
	}
}

void
Tms32031::subb(unsigned dreg, uint32_t a, uint32_t b)
{
	const uint32_t carry = st() & C, res = a - b - carry;

	ireg(dreg) = !ovm() || !overflow_sub(a, b, res) ? res : saturated(a);
	if (dreg < 8) {
		clr(N | Z | C | V | UF);
		st() |= static_cast<int64_t>(a) - b - carry < 0 ? C : 0;
		or_v_sub(a, b, res);
		or_nz(res);
	} else if (dreg >= BK) {
		update_special(dreg);
	}
}

void
Tms32031::subi(unsigned dreg, uint32_t a, uint32_t b)
{
	const uint32_t res = a - b;

	ireg(dreg) = !ovm() || !overflow_sub(a, b, res) ? res : saturated(a);
	if (dreg < 8) {
		clr(N | Z | C | V | UF);
		st() |= b > a ? C : 0;
		or_v_sub(a, b, res);
		or_nz(res);
	} else if (dreg >= BK) {
		update_special(dreg);
	}
}

void
Tms32031::negb(unsigned dreg, uint32_t src)
{
	const uint32_t carry = st() & C, res = 0u - src - carry;

	ireg(dreg) = !ovm() || !overflow_sub(0, src, res) ? res : saturated(src);
	if (dreg < 8) {
		clr(N | Z | C | V | UF);
		st() |= static_cast<int64_t>(0) - src - carry < 0 ? C : 0;
		or_v_sub(0, src, res);
		or_nz(res);
	} else if (dreg >= BK) {
		update_special(dreg);
	}
}

void
Tms32031::negi(unsigned dreg, uint32_t src)
{
	const uint32_t res = 0u - src;

	ireg(dreg) = !ovm() || !overflow_sub(0, src, res) ? res : saturated(src);
	if (dreg < 8) {
		clr(N | Z | C | V | UF);
		st() |= src > 0 ? C : 0;
		or_v_sub(0, src, res);
		or_nz(res);
	} else if (dreg >= BK) {
		update_special(dreg);
	}
}

void
Tms32031::cmpi(uint32_t a, uint32_t b)
{
	const uint32_t res = a - b;

	clr(N | Z | C | V | UF);
	st() |= b > a ? C : 0;
	or_v_sub(a, b, res);
	or_nz(res);
}

void
Tms32031::ash(unsigned dreg, uint32_t src, uint32_t count)
{
	const int32_t n = sext7(count), s = static_cast<int32_t>(src);
	const uint32_t res = n < 0 ? static_cast<uint32_t>(s >> (n >= -31 ? -n : 31)) : n <= 31 ? src << n : 0;

	ireg(dreg) = res;
	if (dreg < 8) {
		clr(N | Z | C | V | UF);
		or_nz(res);
		if (n < 0)
			st() |= static_cast<uint32_t>(s >> (n >= -32 ? -n - 1 : 31)) & C;
		else if (n > 0 && n <= 32)
			st() |= (src << (n - 1)) >> 31;
	} else if (dreg >= BK) {
		update_special(dreg);
	}
}

void
Tms32031::lsh(unsigned dreg, uint32_t src, uint32_t count)
{
	const int32_t n = sext7(count);
	const uint32_t res = n < 0 ? (n >= -31 ? src >> -n : 0) : n <= 31 ? src << n : 0;

	ireg(dreg) = res;
	if (dreg < 8) {
		clr(N | Z | C | V | UF);
		or_nz(res);
		if (n < 0 && n >= -32)
			st() |= (src >> (-n - 1)) & C;
		else if (n > 0 && n <= 32)
			st() |= (src << (n - 1)) >> 31;
	} else if (dreg >= BK) {
		update_special(dreg);
	}
}

void
Tms32031::mpyi(unsigned dreg, uint32_t a, uint32_t b)
{
	const int64_t res = sext24(a) * sext24(b);
	const bool over = res < -int64_t(0x80000000) || res > int64_t(0x7FFFFFFF);

	ireg(dreg) = !ovm() || !over ? static_cast<uint32_t>(res) : res < 0 ? 0x80000000 : 0x7FFFFFFF;
	if (dreg < 8) {
		clr(N | Z | V | UF);
		or_nz(static_cast<uint32_t>(res));
		if (over)
			st() |= V | LV;
	} else if (dreg >= BK) {
		update_special(dreg);
	}
}

void
Tms32031::op_two(uint32_t op)
{
	const unsigned mode = (op >> 21) & 3, d = (op >> 16) & 31, f = (op >> 16) & 7;
	const bool memory = mode == 1 || mode == 2;

	switch (op >> 23) {
	case 0x00:
		absf(f, float_operand(op, mode));
		break;
	case 0x01:
		absi(d, int_operand(op, mode, true));
		break;
	case 0x02: {
		const uint32_t src = int_operand(op, mode, true);

		addc(d, ireg(d), src);
		break;
	}
	case 0x03:
		addf(m_r[f], m_r[f], float_operand(op, mode));
		break;
	case 0x04: {
		const uint32_t src = int_operand(op, mode, true);

		addi(d, ireg(d), src);
		break;
	}
	case 0x05: {
		const uint32_t src = int_operand(op, mode, false);

		logic(d, ireg(d) & src);
		break;
	}
	case 0x06: {
		const uint32_t src = int_operand(op, mode, false);

		logic(d, ireg(d) & ~src);
		break;
	}
	case 0x07: {
		const uint32_t count = int_operand(op, mode, true);

		ash(d, ireg(d), count);
		break;
	}
	case 0x08: {
		const Reg src = float_operand(op, mode);

		subf(m_r[TEMP2], m_r[f], src);
		break;
	}
	case 0x09: {
		const uint32_t src = int_operand(op, mode, true);

		cmpi(ireg(d), src);
		break;
	}
	case 0x0A: {
		Reg value = float_operand(op, mode);

		float2int(value, d < 8);
		m_r[d].man = value.man;
		break;
	}
	case 0x0B:
		ireg(f) = int_operand(op, mode, true);
		int2float(m_r[f]);
		break;
	case 0x0C:
		m_idling = true;
		st() |= GIE;
		check_irqs();
		if (m_idling)
			m_icount = 0;
		break;
	case 0x0D: {
		const Reg src = float_operand(op, mode);

		m_r[f].exp = src.exp;
		if (src.exp == -128)
			m_r[f].man = 0;
		break;
	}
	case 0x0F:
		if (!memory)
			break;
		[[fallthrough]];
	case 0x0E:
		m_r[f] = float_operand(op, mode);
		clr(N | Z | V | UF);
		or_nzf(m_r[f]);
		break;
	case 0x11:
		if (!memory)
			break;
		[[fallthrough]];
	case 0x10:
		load_int(d, int_operand(op, mode, true));
		break;
	case 0x12:
		m_r[f].man = mode == 0 ? m_r[op & 7].man : mode == 3 ? short_to_fp(op).man : int_operand(op, mode, true);
		break;
	case 0x13: {
		const uint32_t count = int_operand(op, mode, true);

		lsh(d, ireg(d), count);
		break;
	}
	case 0x14:
		mpyf(m_r[d], m_r[d], float_operand(op, mode, 31));
		break;
	case 0x15: {
		const uint32_t src = int_operand(op, mode, true);

		mpyi(d, ireg(d), src);
		break;
	}
	case 0x16:
		negb(d, int_operand(op, mode, true));
		break;
	case 0x17:
		negf(m_r[f], float_operand(op, mode));
		break;
	case 0x18:
		negi(d, int_operand(op, mode, true));
		break;
	case 0x19:
		if (mode == 2)
			read(indirect_d(op));
		break;
	case 0x1A:
		norm(m_r[f], float_operand(op, mode));
		break;
	case 0x1B:
		logic(d, ~int_operand(op, mode, false));
		break;
	case 0x1C:
		if (mode == 1)
			load_int(d, read(ireg(SP)--));
		break;
	case 0x1D:
		if (mode == 1) {
			m_r[f] = long_to_fp(read(ireg(SP)--));
			clr(N | Z | V | UF);
			or_nzf(m_r[f]);
		}
		break;
	case 0x1E:
		if (mode == 1)
			write(++ireg(SP), ireg(d));
		break;
	case 0x1F:
		if (mode == 1)
			write(++ireg(SP), fp_to_long(m_r[f]));
		break;
	case 0x20: {
		const uint32_t src = int_operand(op, mode, false);

		logic(d, ireg(d) | src);
		break;
	}
	case 0x21:
		m_low_power = op & 1;
		break;
	case 0x22:
		m_r[f] = float_operand(op, mode);
		rnd(f);
		break;
	case 0x23:
	case 0x24:
	case 0x25:
	case 0x26: {
		if (mode != 3)
			break;

		const uint32_t v = ireg(d), carry = st() & C;
		uint32_t res, out;

		if ((op >> 23) < 0x25) {
			out = v >> 31;
			res = v << 1 | ((op >> 23) == 0x23 ? out : carry);
		} else {
			out = v & 1;
			res = v >> 1 | ((op >> 23) == 0x25 ? out : carry) << 31;
		}
		ireg(d) = res;
		if (d < 8) {
			clr(N | Z | C | V | UF);
			or_nz(res);
			st() |= out;
		} else if (d >= BK) {
			update_special(d);
		}
		break;
	}
	case 0x27:
		ireg(RC) = int_operand(op, mode, false);
		ireg(RS) = m_pc;
		ireg(RE) = m_pc;
		st() |= RM;
		burn(3);
		m_delayed = true;
		break;
	case 0x28:
	case 0x29:
		if (memory)
			write(mode == 1 ? direct(op) : indirect_d(op), fp_to_long(m_r[f]));
		break;
	case 0x2A:
	case 0x2B:
		if (memory)
			write(mode == 1 ? direct(op) : indirect_d(op), ireg(d));
		break;
	case 0x2C:
		break;
	case 0x2D: {
		const uint32_t src = int_operand(op, mode, true);

		subb(d, ireg(d), src);
		break;
	}
	case 0x2E: {
		const uint32_t src = int_operand(op, mode, true), dst = ireg(d);

		ireg(d) = dst >= src ? (dst - src) << 1 | 1 : dst << 1;
		if (d >= BK)
			update_special(d);
		break;
	}
	case 0x2F:
		subf(m_r[f], m_r[f], float_operand(op, mode));
		break;
	case 0x30: {
		const uint32_t src = int_operand(op, mode, true);

		subi(d, ireg(d), src);
		break;
	}
	case 0x31: {
		const uint32_t src = int_operand(op, mode, true);

		subb(d, src, ireg(d));
		break;
	}
	case 0x32: {
		const Reg src = float_operand(op, mode);

		subf(m_r[f], src, m_r[f]);
		break;
	}
	case 0x33: {
		const uint32_t src = int_operand(op, mode, true);

		subi(d, src, ireg(d));
		break;
	}
	case 0x34: {
		const uint32_t src = int_operand(op, mode, false);

		clr(N | Z | V | UF);
		or_nz(ireg(d) & src);
		break;
	}
	case 0x35: {
		const uint32_t src = int_operand(op, mode, false);

		logic(d, ireg(d) ^ src);
		break;
	}
	case 0x36:
		if (memory) {
			const uint32_t addr = mode == 1 ? direct(op) : indirect_d(op);

			m_bus.iack(addr & 0xFFFFFF);
			read(addr);
		}
		break;
	default:
		break;
	}
}

void
Tms32031::op_three(uint32_t op)
{
	const unsigned which = (op >> 23) - 0x40, col = (op >> 21) & 3, d = (op >> 16) & 31, f = (op >> 16) & 7;

	if (which > 0x10)
		return;

	const bool is_float = which == 0x01 || which == 0x06 || which == 0x09 || which == 0x0D;
	const unsigned mask = is_float ? 7 : 31;
	Deferred deferred;
	uint32_t a = 0, b = 0;
	bool a_mem = false, b_mem = false;

	switch (col) {
	case 0:
		a = ireg((op >> 8) & mask);
		b = ireg(op & mask);
		break;
	case 1:
		a = read(indirect_1(op >> 8));
		a_mem = true;
		b = ireg(op & mask);
		break;
	case 2:
		b = read(indirect_1(op));
		b_mem = true;
		a = ireg((op >> 8) & mask);
		break;
	default:
		a = read(indirect_1(op >> 8, &deferred));
		b = read(indirect_1(op));
		apply(deferred);
		a_mem = b_mem = true;
		break;
	}
	if (is_float) {
		const Reg fa = a_mem ? long_to_fp(a) : m_r[(op >> 8) & 7], fb = b_mem ? long_to_fp(b) : m_r[op & 7];

		switch (which) {
		case 0x01:
			addf(m_r[f], fa, fb);
			break;
		case 0x06:
			subf(m_r[TEMP1], fa, fb);
			break;
		case 0x09:
			mpyf(m_r[f], fa, fb);
			break;
		default:
			subf(m_r[f], fa, fb);
			break;
		}
		return;
	}
	switch (which) {
	case 0x00:
		addc(d, a, b);
		break;
	case 0x02:
		addi(d, a, b);
		break;
	case 0x03:
		logic(d, a & b);
		break;
	case 0x04:
		logic(d, a & ~b);
		break;
	case 0x05:
		ash(d, a, b);
		break;
	case 0x07:
		cmpi(a, b);
		break;
	case 0x08:
		lsh(d, a, b);
		break;
	case 0x0A:
		mpyi(d, a, b);
		break;
	case 0x0B:
		logic(d, a | b);
		break;
	case 0x0C:
		subb(d, a, b);
		break;
	case 0x0E:
		subi(d, a, b);
		break;
	case 0x0F:
		clr(N | Z | V | UF);
		or_nz(a & b);
		break;
	default:
		logic(d, a ^ b);
		break;
	}
}

void
Tms32031::op_conditional_load(uint32_t op)
{
	const unsigned top = op >> 23, cond = top & 0x1F, mode = (op >> 21) & 3;

	if (cond == 0x0B || cond > 0x14)
		return;
	if (top < 0xA0) {
		Reg &dst = m_r[(op >> 16) & 7];

		if (condition(cond))
			dst = float_operand(op, mode);
		else if (mode == 2)
			indirect_d(op);
		return;
	}

	const unsigned d = (op >> 16) & 31;
	const uint32_t value = mode == 0 ? 0 : int_operand(op, mode, true);

	if (condition(cond)) {
		ireg(d) = mode == 0 ? ireg(op & 31) : value;
		if (d >= BK)
			update_special(d);
	}
}

void
Tms32031::op_flow(uint32_t op)
{
	const uint32_t top11 = op >> 21;
	const int32_t disp = static_cast<int16_t>(op);
	const bool imm = op & 0x02000000, delayed = op & 0x00200000;

	switch (op >> 24) {
	case 0x60:
		m_pc = op & 0xFFFFFF;
		burn(3);
		return;
	case 0x61:
		execute_delayed(op & 0xFFFFFF, true);
		return;
	case 0x62:
		write(++ireg(SP), m_pc);
		m_pc = op & 0xFFFFFF;
		burn(3);
		return;
	case 0x64:
		ireg(RS) = m_pc;
		ireg(RE) = op & 0xFFFFFF;
		st() |= RM;
		burn(3);
		return;
	default:
		break;
	}
	if (top11 == 0x340 || top11 == 0x341 || top11 == 0x350 || top11 == 0x351) {
		const bool take = condition(op >> 16);

		if (delayed) {
			execute_delayed(imm ? m_pc + 2 + static_cast<uint32_t>(disp) : ireg(op & 31), take);
		} else if (take) {
			m_pc = (imm ? m_pc + static_cast<uint32_t>(disp) : ireg(op & 31)) & 0xFFFFFF;
			burn(3);
		}
	} else if (op >> 26 == 0x1B) {
		const unsigned reg = AR0 + ((op >> 22) & 7);
		const uint32_t res = (ireg(reg) - 1) & 0xFFFFFF;

		ireg(reg) = res | (ireg(reg) & 0xFF000000);

		const bool take = condition(op >> 16) && !(res & 0x800000);

		if (delayed) {
			execute_delayed(imm ? m_pc + 2 + static_cast<uint32_t>(disp) : ireg(op & 31), take);
		} else if (take) {
			m_pc = (imm ? m_pc + static_cast<uint32_t>(disp) : ireg(op & 31)) & 0xFFFFFF;
			burn(3);
		}
	} else if (top11 == 0x380 || top11 == 0x390) {
		if (condition(op >> 16)) {
			write(++ireg(SP), m_pc);
			m_pc = (imm ? m_pc + static_cast<uint32_t>(disp) : ireg(op & 31)) & 0xFFFFFF;
			burn(3);
		}
	} else if (top11 == 0x3A0) {
		if (condition(op >> 16))
			trap(op & 0x3F);
	} else if (top11 == 0x3C0 || top11 == 0x3C4) {
		if (condition(op >> 16)) {
			m_pc = read(ireg(SP)--) & 0xFFFFFF;
			burn(3);
			if (top11 == 0x3C0) {
				st() |= GIE;
				check_irqs();
			}
		}
	}
}

void
Tms32031::op_parallel_multiply(uint32_t op)
{
	const unsigned kind = (op >> 26) & 3, how = (op >> 24) & 3;
	const unsigned r19 = (op >> 19) & 7, r16 = (op >> 16) & 7;
	const unsigned product = (op >> 23) & 1, sum = ((op >> 22) & 1) | 2;
	Deferred deferred;

	if (kind < 2) {
		const uint32_t m3 = read(indirect_1(op >> 8, &deferred)), m4 = read(indirect_1(op));
		const Reg s3 = long_to_fp(m3), s4 = long_to_fp(m4), s1 = m_r[r19], s2 = m_r[r16];
		Reg mul_a, mul_b, add_a, add_b;

		switch (how) {
		case 0:
			mul_a = s3, mul_b = s4, add_a = s1, add_b = s2;
			break;
		case 1:
			mul_a = s3, mul_b = s1, add_a = s4, add_b = s2;
			break;
		case 2:
			mul_a = s1, mul_b = s2, add_a = s3, add_b = s4;
			break;
		default:
			mul_a = s3, mul_b = s1, add_a = s2, add_b = s4;
			break;
		}
		mpyf(m_r[TEMP3], mul_a, mul_b);
		if (kind == 0)
			addf(m_r[sum], add_a, add_b);
		else
			subf(m_r[sum], add_a, add_b);
		m_r[product] = m_r[TEMP3];
		apply(deferred);
		return;
	}

	const uint32_t s1 = ireg(r19), s2 = ireg(r16);
	const uint32_t s3 = read(indirect_1(op >> 8, &deferred)), s4 = read(indirect_1(op));
	uint32_t mul_a, mul_b, add_a, add_b;

	switch (how) {
	case 0:
		mul_a = s3, mul_b = s4, add_a = s1, add_b = s2;
		break;
	case 1:
		mul_a = s3, mul_b = s1, add_a = s4, add_b = s2;
		break;
	case 2:
		mul_a = s1, mul_b = s2, add_a = s3, add_b = s4;
		break;
	default:
		mul_a = s3, mul_b = s1, add_a = s2, add_b = s4;
		break;
	}

	int64_t mres = sext24(mul_a) * sext24(mul_b);
	uint32_t ares = kind == 2 ? add_a + add_b : add_a - add_b;

	clr(N | Z | V | UF);
	if (ovm()) {
		mres = std::clamp(mres, -int64_t(0x80000000), int64_t(0x7FFFFFFF));
		if (kind == 2 ? overflow_add(add_a, add_b, ares) : overflow_sub(add_a, add_b, ares))
			ares = saturated(add_a);
	}
	ireg(product) = static_cast<uint32_t>(mres);
	ireg(sum) = ares;
	apply(deferred);
}

void
Tms32031::op_parallel_store(uint32_t op)
{
	const unsigned which = op >> 25;

	if (which < 0x60 || which > 0x77)
		return;

	const unsigned d22 = (op >> 22) & 7, r19 = (op >> 19) & 7, r16 = (op >> 16) & 7;
	Deferred deferred;

	switch (which) {
	case 0x60:
		write(indirect_1(op >> 8, &deferred), fp_to_long(m_r[r16]));
		write(indirect_1(op), fp_to_long(m_r[d22]));
		apply(deferred);
		return;
	case 0x61:
		write(indirect_1(op >> 8, &deferred), ireg(r16));
		write(indirect_1(op), ireg(d22));
		apply(deferred);
		return;
	case 0x62:
		m_r[r19] = long_to_fp(read(indirect_1(op >> 8, &deferred)));
		m_r[d22] = long_to_fp(read(indirect_1(op)));
		apply(deferred);
		return;
	case 0x63:
		ireg(r19) = read(indirect_1(op >> 8, &deferred));
		ireg(d22) = read(indirect_1(op));
		apply(deferred);
		return;
	default:
		break;
	}

	const bool stores_float = which == 0x64 || which == 0x66 || which == 0x6B || which == 0x6C ||
	                          which == 0x6F || which == 0x71 || which == 0x75;
	const uint32_t stored = stores_float ? fp_to_long(m_r[r16]) : ireg(r16);
	const uint32_t src = read(indirect_1(op, &deferred));

	switch (which) {
	case 0x64:
		absf(d22, long_to_fp(src));
		break;
	case 0x65:
		absi(d22, src);
		break;
	case 0x66:
		addf(m_r[d22], m_r[r19], long_to_fp(src));
		break;
	case 0x67:
		addi(d22, ireg(r19), src);
		break;
	case 0x68:
		logic(d22, ireg(r19) & src);
		break;
	case 0x69:
		ash(d22, src, ireg(r19));
		break;
	case 0x6A:
		m_r[d22] = long_to_fp(src);
		float2int(m_r[d22], true);
		break;
	case 0x6B:
		ireg(d22) = src;
		int2float(m_r[d22]);
		break;
	case 0x6C:
		m_r[d22] = long_to_fp(src);
		break;
	case 0x6D:
		ireg(d22) = src;
		break;
	case 0x6E:
		lsh(d22, src, ireg(r19));
		break;
	case 0x6F:
		mpyf(m_r[d22], m_r[r19], long_to_fp(src));
		break;
	case 0x70:
		mpyi(d22, ireg(r19), src);
		break;
	case 0x71:
		negf(m_r[d22], long_to_fp(src));
		break;
	case 0x72:
		negi(d22, src);
		break;
	case 0x73:
		logic(d22, ~src);
		break;
	case 0x74:
		logic(d22, ireg(r19) | src);
		break;
	case 0x75:
		subf(m_r[d22], long_to_fp(src), m_r[r19]);
		break;
	case 0x76:
		subi(d22, src, ireg(r19));
		break;
	default:
		logic(d22, ireg(r19) ^ src);
		break;
	}
	write(indirect_1(op >> 8), stored);
	apply(deferred);
}
}
