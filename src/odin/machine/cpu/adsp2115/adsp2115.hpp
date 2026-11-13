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

#ifndef ODIN_ADSP2115_HPP
#define ODIN_ADSP2115_HPP

#include <array>
#include <cstdint>
#include <span>

namespace odin {
struct Adsp2115Core {
	uint16_t ax0, ax1, ay0, ay1, ar, af;
	uint16_t mx0, mx1, my0, my1, mf;
	uint64_t mr;
	uint16_t si, se, sb;
	uint32_t sr;

	bool operator==(const Adsp2115Core &) const = default;
};

struct Adsp2115State {
	std::array<uint32_t, 1024> pm;
	std::array<uint16_t, 512> dm;
	std::array<uint16_t, 32> control;
	Adsp2115Core core, alt;
	std::array<uint32_t, 8> i, l, base;
	std::array<int32_t, 8> m;
	std::array<uint32_t, 16> pc_stack;
	std::array<uint32_t, 4> cntr_stack, loop_stack;
	std::array<std::array<uint16_t, 3>, 4> stat_stack;
	uint32_t pc, loop, loop_condition, cntr;
	uint32_t astat, sstat, mstat;
	int32_t pc_sp, cntr_sp, stat_sp, loop_sp;
	uint16_t imask, ifc;
	uint8_t icntl, px, flagin, flagout, fl0, fl1, fl2, idle;
	std::array<uint8_t, 6> irq_state, irq_latch, irq_pulsed;
	uint64_t cycles;

	bool operator==(const Adsp2115State &) const = default;
};

class Adsp2115Bus {
public:
	virtual ~Adsp2115Bus() = default;

	virtual uint16_t data_read(uint16_t addr) = 0;
	virtual void data_write(uint16_t addr, uint16_t value) = 0;

	virtual bool waiting(uint32_t pc)
	{
		(void)pc;
		return false;
	}
	virtual uint32_t program_read(uint16_t addr)
	{
		(void)addr;
		return 0;
	}
	virtual void program_write(uint16_t addr, uint32_t value)
	{
		(void)addr;
		(void)value;
	}

	virtual void control_written(unsigned reg, uint16_t value)
	{
		(void)reg;
		(void)value;
	}

	virtual void sport_tx(unsigned port, uint32_t value)
	{
		(void)port;
		(void)value;
	}
	virtual uint32_t sport_rx(unsigned port)
	{
		(void)port;
		return 0;
	}
};

class Adsp2115 {
public:

	enum Line : unsigned {
		IRQ0 = 0, SPORT1_RX = 0,
		IRQ1 = 1, SPORT1_TX = 1,
		IRQ2 = 2,
		SPORT0_RX = 3,
		SPORT0_TX = 4,
		TIMER = 5,
		LINES = 6
	};

	enum Control : unsigned {
		S1_AUTOBUF = 15, S1_RFSDIV, S1_SCLKDIV, S1_CONTROL,
		S0_AUTOBUF, S0_RFSDIV, S0_SCLKDIV, S0_CONTROL,
		S0_MCTXLO, S0_MCTXHI, S0_MCRXLO, S0_MCRXHI,
		TIMER_SCALE, TIMER_COUNT, TIMER_PERIOD, WAITSTATES, SYSCONTROL
	};

	explicit Adsp2115(Adsp2115Bus &bus);

	void reset();

	uint32_t run(uint32_t cycles);
	void set_irq(unsigned line, bool asserted);

	void pulse_irq(unsigned line);

	std::span<uint32_t> program_ram() { return m_pm; }
	std::span<uint16_t> data_ram() { return m_dm; }
	const std::array<uint16_t, 32> &control() const { return m_control; }
	uint16_t i(unsigned n) const { return static_cast<uint16_t>(m_i[n & 7]); }
	uint16_t m(unsigned n) const { return static_cast<uint16_t>(m_m[n & 7]); }
	uint16_t l(unsigned n) const { return static_cast<uint16_t>(m_l[n & 7]); }
	void set_i(unsigned n, uint16_t v);
	uint64_t cycles() const { return m_cycles; }
	uint32_t pc() const { return m_pc; }
	void set_wait_pc(uint32_t pc) { m_wait_pc = pc; }
	bool idle() const { return m_idle != 0; }

	void save(Adsp2115State &out) const;
	void load(const Adsp2115State &in);

private:
	enum : uint32_t {
		SSFLAG = 0x80, MVFLAG = 0x40, QFLAG = 0x20, SFLAG = 0x10,
		CFLAG = 0x08, VFLAG = 0x04, NFLAG = 0x02, ZFLAG = 0x01
	};
	enum : uint32_t {
		MSTAT_BANK = 0x01, MSTAT_REVERSE = 0x02, MSTAT_STICKYV = 0x04, MSTAT_SATURATE = 0x08,
		MSTAT_INTEGER = 0x10, MSTAT_TIMER = 0x20, MSTAT_GOMODE = 0x40
	};
	enum : uint32_t {
		PC_EMPTY = 0x01, PC_OVER = 0x02, COUNT_EMPTY = 0x04, COUNT_OVER = 0x08,
		STATUS_EMPTY = 0x10, STATUS_OVER = 0x20, LOOP_EMPTY = 0x40, LOOP_OVER = 0x80
	};

	void execute(uint32_t op);

	uint16_t data_read(uint32_t addr);
	void data_write(uint32_t addr, uint16_t value);
	uint32_t program_read(uint32_t addr);
	void program_write(uint32_t addr, uint32_t value);

	void update_mstat();
	void pc_stack_push(uint32_t value);
	uint32_t pc_stack_pop();
	uint32_t pc_stack_top() const { return m_pc_stack[m_pc_sp > 0 ? m_pc_sp - 1 : 0]; }
	void cntr_stack_push();
	void cntr_stack_pop();
	void loop_stack_push(uint32_t value);
	void loop_stack_pop();
	void stat_stack_push();
	void stat_stack_pop();
	bool condition(unsigned c);

	void update_i(unsigned n) { m_base[n] = m_i[n] & m_lmask[n]; }
	void update_l(unsigned n);
	void post_modify(unsigned ireg, unsigned mreg);
	uint16_t dag_read(unsigned ireg, unsigned mreg, bool reverse);
	void dag_write(unsigned ireg, unsigned mreg, bool reverse, uint16_t value);
	uint16_t pm_dag_read(uint32_t op);
	void pm_dag_write(uint32_t op, int32_t value);
	uint16_t dag1_read(uint32_t op) { return dag_read(op >> 2 & 3, op & 3, m_mstat & MSTAT_REVERSE); }
	void dag1_write(uint32_t op, int32_t value)
	{
		dag_write(op >> 2 & 3, op & 3, m_mstat & MSTAT_REVERSE, static_cast<uint16_t>(value));
	}
	uint16_t dag2_read(uint32_t op) { return dag_read(4 + (op >> 2 & 3), 4 + (op & 3), false); }
	void dag2_write(uint32_t op, int32_t value)
	{
		dag_write(4 + (op >> 2 & 3), 4 + (op & 3), false, static_cast<uint16_t>(value));
	}

	void write_reg(unsigned group, unsigned reg, int32_t value);
	int32_t read_reg(unsigned group, unsigned reg);
	uint16_t alu_x(unsigned x) const;
	uint16_t alu_y(unsigned y) const;
	uint16_t mac_x(unsigned x) const;
	uint16_t mac_y(unsigned y) const;
	uint16_t shift_x(unsigned x) const;

	int32_t alu(uint32_t op);
	void alu_ar(uint32_t op);
	void alu_af(uint32_t op) { m_core.af = static_cast<uint16_t>(alu(op)); }
	int64_t mac(uint32_t op, bool &done);
	void mac_mr(uint32_t op);
	void mac_mf(uint32_t op);

	void compute(uint32_t op);
	void shift(uint32_t op, int8_t sc, bool immediate);

	void check_irqs();
	bool generate_irq(unsigned line, unsigned index);

	Adsp2115Bus &m_bus;

	std::array<uint32_t, 1024> m_pm{};
	std::array<uint16_t, 512> m_dm{};
	std::array<uint16_t, 32> m_control{};
	Adsp2115Core m_core{}, m_alt{};
	std::array<uint32_t, 8> m_i{}, m_l{}, m_lmask{}, m_base{};
	std::array<int32_t, 8> m_m{};
	std::array<uint32_t, 16> m_pc_stack{};
	std::array<uint32_t, 4> m_cntr_stack{}, m_loop_stack{};
	std::array<std::array<uint16_t, 3>, 4> m_stat_stack{};
	uint32_t m_pc = 0, m_ppc = 0, m_loop = 0xFFFF, m_loop_condition = 0, m_cntr = 0;
	uint32_t m_astat = 0, m_sstat = 0x55, m_mstat = 0, m_mstat_prev = 0;
	uint32_t m_astat_clear = ~(CFLAG | VFLAG | NFLAG | ZFLAG);
	int32_t m_pc_sp = 0, m_cntr_sp = 0, m_stat_sp = 0, m_loop_sp = 0;
	uint16_t m_imask = 0, m_ifc = 0;
	uint8_t m_icntl = 0, m_px = 0, m_flagin = 0, m_flagout = 0, m_fl0 = 0, m_fl1 = 0, m_fl2 = 0, m_idle = 0;
	std::array<uint8_t, LINES> m_irq_state{}, m_irq_latch{}, m_irq_pulsed{};
	uint64_t m_cycles = 0;
	int64_t m_icount = 0;

	bool m_spin = false;
	uint32_t m_wait_pc = 0xFFFFFFFFu;
};
}

#endif
