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

#ifndef ODIN_M68020_HPP
#define ODIN_M68020_HPP

#include <array>
#include <cstdint>

namespace odin {
struct M68020State {
	std::array<uint32_t, 8> d;
	std::array<uint32_t, 8> a;
	uint32_t usp, isp, msp;
	uint32_t pc, vbr, sfc, dfc, cacr, caar;
	uint16_t sr;
	uint8_t ipl;
	uint8_t nmi_edge;
	uint8_t stopped;
	uint8_t pad[3];
	uint64_t cycles;

	bool operator==(const M68020State &) const = default;
};

class M68020Bus {
public:
	virtual ~M68020Bus() = default;
	virtual uint8_t read8(uint32_t addr) = 0;
	virtual uint16_t read16(uint32_t addr) = 0;
	virtual uint32_t read32(uint32_t addr) = 0;
	virtual void write8(uint32_t addr, uint8_t value) = 0;
	virtual void write16(uint32_t addr, uint16_t value) = 0;
	virtual void write32(uint32_t addr, uint32_t value) = 0;

	virtual int int_ack(unsigned level)
	{
		(void)level;
		return -1;
	}
	virtual void reset_devices() {}
	virtual bool waiting(uint32_t pc)
	{
		(void)pc;
		return false;
	}
};

/* HEURISTIC: clocks, 2 an instruction and 3 a data cycle */
class M68020 {
public:
	explicit M68020(M68020Bus &bus);

	void reset();

	uint32_t run(uint32_t cycles);
	void set_ipl(unsigned level);

	void end_run() { m_run_end = m_cycles; }
	void set_wait_pc(uint32_t pc) { m_wait_pc = pc; }

	void save(M68020State &out) const;
	void load(const M68020State &in);

	uint64_t cycles() const { return m_cycles; }
	uint32_t pc() const { return m_pc; }
	uint32_t d(unsigned n) const { return m_d[n & 7]; }
	uint32_t a(unsigned n) const { return m_a[n & 7]; }
	uint16_t sr() const { return m_sr; }

private:
	enum : uint16_t {
		SR_C = 0x0001, SR_V = 0x0002, SR_Z = 0x0004, SR_N = 0x0008, SR_X = 0x0010,
		SR_I = 0x0700, SR_M = 0x1000, SR_S = 0x2000, SR_T = 0xC000,
		SR_MASK = 0xF71F
	};
	enum Size : uint8_t { BYTE, WORD, LONG };

	struct Ea {
		enum Kind : uint8_t { DREG, AREG, MEM, IMM } kind;
		uint8_t reg;
		uint32_t addr;
		uint32_t imm;
	};

	uint8_t read8(uint32_t addr);
	uint16_t read16(uint32_t addr);
	uint32_t read32(uint32_t addr);
	void write8(uint32_t addr, uint8_t v);
	void write16(uint32_t addr, uint16_t v);
	void write32(uint32_t addr, uint32_t v);
	uint32_t read(Size s, uint32_t addr);
	void write(Size s, uint32_t addr, uint32_t v);
	uint16_t fetch16();
	uint32_t fetch32();
	void push16(uint16_t v);
	void push32(uint32_t v);
	uint16_t pop16();
	uint32_t pop32();

	Ea ea(unsigned mode, unsigned reg, Size s);
	uint32_t get(const Ea &e, Size s);
	void put(const Ea &e, Size s, uint32_t v);
	uint32_t extension(uint32_t base);

	void set_sr(uint16_t v);
	void set_ccr(uint32_t v) { m_sr = static_cast<uint16_t>((m_sr & 0xFF00) | (v & 0x1F)); }
	void keep_sp();
	void take_sp();
	bool cond(unsigned cc) const;
	void flags_nz(Size s, uint32_t r);
	void flags_add(Size s, uint32_t a, uint32_t b, uint32_t r);
	void flags_sub(Size s, uint32_t a, uint32_t b, uint32_t r);
	uint32_t alu(unsigned kind, Size s, uint32_t dst, uint32_t src);
	uint32_t bcd(bool add, uint32_t dst, uint32_t src);
	uint32_t extend(bool add, Size s, uint32_t dst, uint32_t src);

	void exception(unsigned vector, uint32_t return_pc, bool format2 = false);
	void interrupt(unsigned level);
	void illegal(unsigned vector = 4) { exception(vector, m_ppc); }
	void privilege() { exception(8, m_ppc); }

	void execute();
	void op_0000();
	void op_move(Size s);
	void op_0100();
	void op_0101();
	void op_0110();
	void op_1000();
	void op_add_sub(bool add);
	void op_1011();
	void op_1100();
	void op_1110();
	void op_bit(unsigned kind, bool dynamic);
	void op_immediate();
	void op_cmp2_chk2();
	void op_cas();
	void op_cas2();
	void op_moves();
	void op_movem(bool to_registers);
	void op_misc_4e();
	void op_movec(bool to_control);
	void op_mul_long();
	void op_div_long();
	void op_div_word(bool is_signed);
	void op_addx_subx(bool add);
	void op_abcd_sbcd(bool add);
	void op_pack_unpk(bool pack);
	void op_shift_register();
	void op_shift_memory();
	void op_bitfield();
	void op_chk(Size s);

	static uint32_t mask(Size s) { return s == BYTE ? 0xFFu : s == WORD ? 0xFFFFu : 0xFFFFFFFFu; }
	static uint32_t msb(Size s) { return s == BYTE ? 0x80u : s == WORD ? 0x8000u : 0x80000000u; }
	static uint32_t bytes(Size s) { return s == BYTE ? 1u : s == WORD ? 2u : 4u; }
	static uint32_t sext(Size s, uint32_t v)
	{
		return s == BYTE ? static_cast<uint32_t>(static_cast<int8_t>(v))
		       : s == WORD ? static_cast<uint32_t>(static_cast<int16_t>(v)) : v;
	}
	static Size size_bits_to(unsigned bits) { return bits == 0 ? BYTE : bits == 1 ? WORD : LONG; }
	void set_d(unsigned n, Size s, uint32_t v)
	{
		const uint32_t m = mask(s);
		m_d[n & 7] = (m_d[n & 7] & ~m) | (v & m);
	}

	M68020Bus &m_bus;
	std::array<uint32_t, 8> m_d{}, m_a{};
	uint32_t m_usp = 0, m_isp = 0, m_msp = 0;
	uint32_t m_pc = 0, m_ppc = 0;
	uint32_t m_vbr = 0, m_sfc = 0, m_dfc = 0, m_cacr = 0, m_caar = 0;
	uint16_t m_sr = SR_S | SR_I;
	uint16_t m_ir = 0;
	uint8_t m_ipl = 0;
	bool m_nmi_edge = false;
	bool m_stopped = false;
	uint64_t m_cycles = 0;
	uint64_t m_run_end = 0;
	uint32_t m_wait_pc = 0xFFFFFFFFu;
};
}

#endif
