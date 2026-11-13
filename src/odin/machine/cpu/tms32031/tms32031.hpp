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

#ifndef ODIN_TMS32031_HPP
#define ODIN_TMS32031_HPP

#include <array>
#include <cstdint>
#include <span>

namespace odin {
struct Tms32031State {
	std::array<uint32_t, 32> r;
	std::array<int32_t, 32> e;
	std::array<uint32_t, 2048> ram;
	uint64_t cycles;
	uint32_t pc;
	uint32_t bus_control;
	uint16_t irq_state;
	uint8_t delayed, irq_pending, idling, low_power, reset_held;

	bool operator==(const Tms32031State &) const = default;
};

class Tms32031Bus {
public:
	virtual ~Tms32031Bus() = default;
	virtual uint32_t read(uint32_t addr) = 0;
	virtual void write(uint32_t addr, uint32_t value) = 0;

	virtual void iack(uint32_t addr) { (void)addr; }
};

/* From MAME's tms320c3x (BSD-3-Clause, Aaron Giles) */
class Tms32031 {
public:

	Tms32031(Tms32031Bus &bus, std::span<const uint8_t> boot);

	void set_reset(bool held);

	void set_int(unsigned line, bool asserted);

	uint32_t run(uint32_t cycles);

	uint64_t cycles() const { return m_cycles; }
	uint32_t pc() const { return m_pc; }

	void save(Tms32031State &out) const;
	void load(const Tms32031State &in);

	static float to_float(uint32_t dsp);
	static uint32_t from_float(float value);

private:

	struct Reg {
		uint32_t man = 0;
		int32_t exp = 0;

		int32_t m() const { return static_cast<int32_t>(man); }
		void set_m(int32_t v) { man = static_cast<uint32_t>(v); }
		void set_e(int v) { exp = static_cast<int8_t>(v); }
	};

	struct Deferred {
		unsigned reg = 0;
		uint32_t value = 0;
		bool pending = false;
	};

	enum : unsigned {
		AR0 = 8, DP = 16, IR0, IR1, BK, SP, ST, IE, IF, IOF, RS, RE, RC,
		TEMP1 = 32, TEMP2, TEMP3, REGS
	};
	static constexpr uint32_t C = 0x0001, V = 0x0002, Z = 0x0004, N = 0x0008, UF = 0x0010, LV = 0x0020,
	                          LUF = 0x0040, OVM = 0x0080, RM = 0x0100, GIE = 0x2000;

	void reset();
	uint32_t read(uint32_t addr);
	void write(uint32_t addr, uint32_t value);
	void burn(int cycles) { m_icount -= cycles * (m_low_power ? 16 : 1); }
	void check_irqs();
	void trap(unsigned number);
	void execute_one();
	void execute_delayed(uint32_t target, bool taken);
	void update_special(unsigned reg);
	bool condition(unsigned which) const;

	uint32_t &ireg(unsigned n) { return m_r[n].man; }
	uint32_t &st() { return m_r[ST].man; }
	uint32_t direct(uint32_t op) { return (ireg(DP) & 0xFF) << 16 | (op & 0xFFFF); }
	uint32_t indirect(unsigned field, uint32_t disp, Deferred *deferred);
	uint32_t indirect_d(uint32_t op) { return indirect(op >> 8, op & 0xFF, nullptr); }
	uint32_t indirect_1(uint32_t field, Deferred *deferred = nullptr) { return indirect(field, 1, deferred); }
	void apply(const Deferred &deferred)
	{
		if (deferred.pending)
			ireg(deferred.reg) = deferred.value;
	}

	static Reg long_to_fp(uint32_t v);
	static Reg short_to_fp(uint32_t v);
	static uint32_t fp_to_long(const Reg &r);
	Reg float_operand(uint32_t op, unsigned mode, unsigned reg_mask = 7);
	uint32_t int_operand(uint32_t op, unsigned mode, bool sign);

	void clr(uint32_t flags) { st() &= ~flags; }
	void or_nz(uint32_t v) { st() |= ((v >> 28) & N) | (v == 0 ? Z : 0); }
	void or_nzf(const Reg &r) { st() |= ((r.man >> 28) & N) | (r.exp == -128 ? Z : 0); }
	void or_nuf(const Reg &r)
	{
		const uint32_t uf = r.exp == -128 ? UF : 0;
		st() |= ((r.man >> 28) & N) | uf | uf << 2;
	}
	void or_v_add(uint32_t a, uint32_t b, uint32_t r)
	{
		const uint32_t v = (((a ^ r) & (b ^ r)) >> 30) & V;
		st() |= v | v << 4;
	}
	void or_v_sub(uint32_t a, uint32_t b, uint32_t r)
	{
		const uint32_t v = (((a ^ b) & (a ^ r)) >> 30) & V;
		st() |= v | v << 4;
	}
	bool ovm() const { return m_r[ST].man & OVM; }

	void int2float(Reg &r);
	void float2int(Reg &r, bool flags);
	void negf(Reg &dst, Reg src);
	void addf(Reg &dst, Reg a, Reg b);
	void subf(Reg &dst, Reg a, Reg b);
	void mpyf(Reg &dst, Reg a, Reg b);
	void norm(Reg &dst, Reg src);
	void absf(unsigned dreg, Reg src);
	void rnd(unsigned dreg);

	void load_int(unsigned dreg, uint32_t value);
	void absi(unsigned dreg, uint32_t src);
	void addc(unsigned dreg, uint32_t a, uint32_t b);
	void addi(unsigned dreg, uint32_t a, uint32_t b);
	void subb(unsigned dreg, uint32_t a, uint32_t b);
	void subi(unsigned dreg, uint32_t a, uint32_t b);
	void logic(unsigned dreg, uint32_t result);
	void ash(unsigned dreg, uint32_t src, uint32_t count);
	void lsh(unsigned dreg, uint32_t src, uint32_t count);
	void mpyi(unsigned dreg, uint32_t a, uint32_t b);
	void negb(unsigned dreg, uint32_t src);
	void negi(unsigned dreg, uint32_t src);
	void cmpi(uint32_t a, uint32_t b);

	void op_two(uint32_t op);
	void op_three(uint32_t op);
	void op_conditional_load(uint32_t op);
	void op_flow(uint32_t op);
	void op_parallel_multiply(uint32_t op);
	void op_parallel_store(uint32_t op);

	Tms32031Bus &m_bus;
	std::array<uint32_t, 4096> m_boot{};
	std::array<uint32_t, 2048> m_ram{};
	std::array<Reg, REGS> m_r{};
	uint32_t m_bkmask = 0;
	uint32_t m_pc = 0;
	uint32_t m_bus_control = 0;
	uint16_t m_irq_state = 0;
	bool m_delayed = false, m_irq_pending = false, m_idling = false, m_low_power = false, m_reset_held = false;
	int64_t m_icount = 0;
	uint64_t m_cycles = 0;
};
}

#endif
