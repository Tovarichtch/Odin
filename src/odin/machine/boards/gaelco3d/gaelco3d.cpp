// license:BSD-3-Clause
// copyright-holders:Aaron Giles, Couriersud
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
 * Odin is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * Odin is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with Odin.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Based on MAME's gaelco3d driver, its serial link and its serial EEPROM
 * (BSD-3-Clause, Aaron Giles, Couriersud); those parts keep their licence */

#include "odin/machine/boards/gaelco3d/gaelco3d.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <memory>
#include <thread>
#include <type_traits>
#include <utility>

#include "odin/machine/boards/atvtrack/atvtrack.hpp"
#include "odin/machine/core/state_file.hpp"

namespace odin {
namespace {
constexpr uint64_t MAIN_CLOCK = 25'000'000, MAIN16_CLOCK = 15'000'000;
constexpr uint64_t TMS_CYCLES = 25'000'000, TMS16_CYCLES = 30'000'000;
constexpr uint64_t ADSP_CLOCK = 16'000'000;
constexpr unsigned SOUND_CHANNELS = 4;
/* Football Power's ball PICs: XL1 at 16 MHz (ACEL_V2 schematic), four
 * clock periods per instruction cycle */
constexpr uint64_t PIC_CLOCK = 16'000'000, PIC_CYCLES = PIC_CLOCK / 4;
/* The two TDA1552Q amplifiers of the 68000 boards are muted while their pin 11
 * is below 6.4 V, and play above 8.5 V (Philips, July 1994). On Speed Up's
 * sheet 6 (3D-AUDIO), the reset IC holds that pin at 3.9 V, then C1 (100 uF)
 * charges through R2 and R1 (6K8, 3K3): 6.4 V after 0.37 s, 8.5 V after 0.85 s.
 * PROVEN for Speed Up; Surf Planet's CPU board has the same amplifiers (Atari
 * kit manual p. 32), its wiring INFERRED. HEURISTIC: the reset ends at
 * power-on; the sound starts at 0.85 s, 51 fields */
constexpr unsigned AMPLIFIERS_MUTED_FIELDS = 51;

constexpr uint64_t SLICES_PER_FRAME = 200;
/* Each clock runs a whole number of cycles per half line */
static_assert(MAIN_CLOCK % Gaelco3d::HALF_LINES_PER_SECOND == 0 && MAIN16_CLOCK % Gaelco3d::HALF_LINES_PER_SECOND == 0 &&
	      TMS_CYCLES % Gaelco3d::HALF_LINES_PER_SECOND == 0 && TMS16_CYCLES % Gaelco3d::HALF_LINES_PER_SECOND == 0 &&
	      ADSP_CLOCK % Gaelco3d::HALF_LINES_PER_SECOND == 0 && PIC_CYCLES % Gaelco3d::HALF_LINES_PER_SECOND == 0);

/* D14 and D15 of 0x51002C on Football Power, pulled up. HEURISTIC: as on Speed
 * Up's ROM board (sheet 7, RR2); there is no schematic of Football Power's. A
 * read starts with INPUT high until D14 reads 0 (pm 0x2F15E-0x2F18A, the mask
 * of an older wiring). With D14 low, INPUT stays high for 5 to 7 PIC cycles,
 * less than one pass of their main loop (11); a PIC that misses it sends its
 * frame two edges late, which the game takes as a kick: 976 of 1778 reads from
 * power-on. With D14 high, INPUT stays high for about 1400 cycles (MEASURED) */
constexpr uint32_t BALL_D14_D15 = 0xC0000000u;
/* The ball PIC's main loop (pm 0x1F6-0x1FF of fop_621_pic16c710.u1, Ghidra
 * gaelco3d-pic): while RB4 is steady, each pass only clears the watchdog
 * and sets 0x1E from RB4, the same way each time, until an interrupt */
constexpr uint16_t BALL_WAIT_PC = 0x1F6;
constexpr std::array<uint16_t, 10> BALL_WAIT = { 0x0064, 0x1A06, 0x141E, 0x1E06, 0x019E,
	                                         0x189E, 0x29F6, 0x181E, 0x2165, 0x29F6 };

/* The main CPU's wait for the next frame is skipped where the code
 * matches Radikal Bikers or a Football Power version */
struct MainWait {
	uint32_t pc;
	uint32_t loop, loop_size;
	uint64_t loop_hash;
	uint32_t sub, sub_size;
	uint64_t sub_hash;
	uint32_t counter;
};
constexpr MainWait MAIN_WAITS[] = {
	{ 0x72E, 0x72E, 14, 0xc6eb82c318a1f7b2ull, 0x31F84, 0x6A, 0x9337122deae2e8f4ull, 0 },
	{ 0x2FB14, 0x2FB14, 0x16, 0x27b09cd77ca8450full, 0x1A18, 0x24, 0xc81b3e013d5f0d6bull, 0xFE64CC },
	{ 0x30556, 0x30556, 0x16, 0x4ca46700dbeb0b61ull, 0x1A18, 0x24, 0xc81b3e013d5f0d6bull, 0xFE64E2 },
	{ 0x3054C, 0x3054C, 0x16, 0x4ca46700dbeb0b61ull, 0x1A18, 0x24, 0xc81b3e013d5f0d6bull, 0xFE64E2 },
	{ 0x30520, 0x30520, 0x16, 0x4ca46700dbeb0b61ull, 0x1A18, 0x24, 0xc81b3e013d5f0d6bull, 0xFE64E2 },
};
/* The 68000's wait for the next frame (attentes-prouvees.md § 1.1, 1.2).
 * Speed Up: jsr routine / tst.w (cell).l / beq, where the routine tests
 * bit 5 of a development option (flag) and returns if it is clear. Surf
 * Planet: move.w #1,(flag).l / move #0x2000,SR / tst.w (cell).l / beq. The
 * words matched are built from these addresses, so the test reads what the
 * loop reads */
struct Main16Wait {
	uint32_t pc, routine, flag, cell;	/* routine 0: Surf Planet's loop */
};
constexpr Main16Wait MAIN16_WAITS[] = {
	{ 0x740, 0x85E2, 0xFE3225, 0xFE2818 },	/* Speed Up 2.0, 2.0a, 2.1, 2.2 */
	{ 0x732, 0x8568, 0xFE321D, 0xFE2812 },	/* Speed Up 1.2 */
	{ 0x72C, 0x844C, 0xFE321B, 0xFE2812 },	/* Speed Up 1.0 */
	{ 0x10A4, 0, 0xFE0512, 0xFE04B4 },	/* Surf Planet 4.0, 4.1 */
	{ 0x10A4, 0, 0xFE0510, 0xFE04B2 },	/* Surf Planet 1.0 to 3.1 */
};

/* The 68000 program's word at `at` */
uint16_t
word_at(std::span<const uint8_t> program, size_t at)
{
	return static_cast<uint16_t>(program[at] << 8 | program[at + 1]);
}

/* Whether the 68000 program holds these words from `at` on */
bool
holds(std::span<const uint8_t> program, uint32_t at, std::initializer_list<uint32_t> words)
{
	for (const uint32_t w : words) {
		if (at + 1 >= program.size() || word_at(program, at) != w)
			return false;
		at += 2;
	}
	return true;
}

/* Whether the 68000 program contains that wait */
bool
holds_wait(std::span<const uint8_t> program, const Main16Wait &wait)
{
	const uint16_t hi = static_cast<uint16_t>(wait.cell >> 16), lo = static_cast<uint16_t>(wait.cell);
	const uint16_t flag_hi = static_cast<uint16_t>(wait.flag >> 16), flag_lo = static_cast<uint16_t>(wait.flag);

	if (!wait.routine)
		return holds(program, wait.pc,
		             { 0x33FC, 0x0001, flag_hi, flag_lo, 0x46FC, 0x2000, 0x4A79, hi, lo, 0x67EC });
	if (!holds(program, wait.pc, { 0x4EB9, wait.routine >> 16, wait.routine & 0xFFFF, 0x4A79, hi, lo, 0x67F2 }) ||
	    !holds(program, wait.routine, { 0x0839, 0x0005, flag_hi, flag_lo }) || wait.routine + 9 >= program.size() ||
	    (word_at(program, wait.routine + 8) >> 8) != 0x67)
		return false;
	/* beq.b to an rts */
	return holds(program, wait.routine + 10 + static_cast<int8_t>(word_at(program, wait.routine + 8)), { 0x4E75 });
}

/* Speed Up's motor command, as its program makes it each frame (2.2:
 * 0x0C1C-0x0C4C): move.w (spring).l,d0 / muls.w (gain).l,d0 / asr.l #4,d0 /
 * add.w (effects).l,d0 / cmpi.w #-128,d0, then bounded to -128..127 and
 * written to 0x510044: the wheel's position times its gain, plus the
 * effects (its buzz, skids and hits, cleared after). Each version keeps
 * them elsewhere (2.x: 0xFE40F8, 1.2: 0xFE3A90, 1.0: 0xFE3A7C): taken from
 * the one routine found (preuves/ffb-20261117). The routine tests first,
 * 0x1A bytes before (tst.w (on).l), whether the spring is on: the program
 * lets go of it while a hit or a jump plays. Its hit routine (2.2: 0x15F14)
 * starts move.b #0,(cut).l / move.b (hit).l,d0 / beq: the hit played, cars
 * +-1 to +-3, a wall +-4 (preuves/ffb-manette-etude-20261118; the same in the
 * six versions) */
struct MotorAt {
	uint32_t spring, gain, on, hit;
};

std::optional<MotorAt>
motor_at(std::span<const uint8_t> program)
{
	const auto word = [&](size_t at) { return word_at(program, at); };
	const auto lng = [&](size_t at) { return static_cast<uint32_t>(word(at) << 16 | word(at + 2)); };
	std::optional<MotorAt> found;

	std::optional<uint32_t> hit;

	for (size_t at = 26; at + 26 <= program.size(); at += 2)
		if (word(at) == 0x3039 && word(at + 6) == 0xC1F9 && word(at + 12) == 0xE880 && word(at + 14) == 0xD079 &&
		    word(at + 20) == 0x0C40 && word(at + 22) == 0xFF80 && word(at - 26) == 0x4A79) {
			if (found)
				return std::nullopt;
			found = MotorAt{ lng(at + 2), lng(at + 8), lng(at - 24), 0 };
		}
	for (size_t at = 0; at + 14 <= program.size(); at += 2)
		if (word(at) == 0x13FC && word(at + 2) == 0x0000 && word(at + 8) == 0x1039 && word(at + 14) >> 8 == 0x67) {
			if (hit)
				return std::nullopt;
			hit = lng(at + 10);
		}
	if (!found || !hit)
		return std::nullopt;
	found->hit = *hit;
	return found;
}

/* The ADSP's wait for its next sample is skipped where the code
 * matches; `at` is the first data memory word the loop reads */
struct AdspWait {
	uint32_t pc;
	std::array<uint32_t, 8> code;
	unsigned at;
};
constexpr AdspWait ADSP_WAITS[] = {
	{ 0x43, { 0x838040, 0x838054, 0x22E00F, 0x1C1435, 0x8380C0, 0x22780F, 0x1C04B1, 0x18043F }, 4 },
	{ 0x44, { 0x838050, 0x838064, 0x22E00F, 0x1C13D5, 0x8380D0, 0x22780F, 0x1C04C1, 0x18044F }, 5 },
};

uint64_t
fnv(std::span<const uint8_t> bytes)
{
	uint64_t h = 0xcbf29ce484222325ull;

	for (const uint8_t b : bytes)
		h = (h ^ b) * 0x100000001b3ull;
	return h;
}

/* The goal that ends the ADSP's thread */
constexpr uint64_t ADSP_STOP = ~uint64_t{ 0 };

/* A short pause for a polling thread */
inline void
relax()
{
#if defined(__x86_64__) || defined(__i386__)
	__builtin_ia32_pause();
#elif defined(__aarch64__) || defined(__arm__)
	asm volatile("yield");
#endif
}

uint64_t
target(uint64_t slices, uint64_t rate)
{
	return slices * (rate / Gaelco3d::HALF_LINES_PER_SECOND) * Gaelco3d::HALF_LINES_PER_FIELD / SLICES_PER_FRAME;
}
}

/* Saved as it is: its padding is named, so no byte is left to chance */
static_assert(std::has_unique_object_representations_v<Eeprom93c66::State>);

Eeprom93c66::Eeprom93c66()
{
	m_s = {};
	m_s.data.fill(0xFFFF);
	m_s.locked = 1;
}

namespace {
enum EepromState : uint8_t { IN_RESET, START_BIT, COMMAND, READING, WAIT_DATA, COMPLETION };
enum EepromCommand : uint8_t { LOCK, WRITEALL, ERASEALL, UNLOCK, WRITE, READ, ERASE };
constexpr unsigned EEPROM_ADDRESS_BITS = 8, EEPROM_DATA_BITS = 16;
}

void
Eeprom93c66::set_cs(bool on)
{
	if (on == static_cast<bool>(m_s.cs))
		return;
	m_s.cs = on;
	if (!on)
		m_s.state = IN_RESET;
	else if (m_s.state == IN_RESET)
		m_s.state = START_BIT;
}

void
Eeprom93c66::set_clk(bool on)
{
	const bool rising = on && !m_s.clk;

	m_s.clk = on;
	if (!rising)
		return;
	switch (m_s.state) {
	case START_BIT:
		if (m_s.di) {
			m_s.shift = m_s.accum = 0;
			m_s.state = COMMAND;
		}
		break;
	case COMMAND:
		m_s.shift = m_s.shift << 1 | m_s.di;
		if (++m_s.accum == 2 + EEPROM_ADDRESS_BITS)
			command();
		break;
	case READING: {
		const uint32_t index = m_s.accum++;

		if (index % EEPROM_DATA_BITS == 0)
			m_s.shift = static_cast<uint32_t>(m_s.data[(m_s.address + m_s.accum / EEPROM_DATA_BITS) & (WORDS - 1)])
			            << (32 - EEPROM_DATA_BITS);
		else
			m_s.shift = m_s.shift << 1 | 1;
		break;
	}
	case WAIT_DATA:
		m_s.shift = m_s.shift << 1 | m_s.di;
		if (++m_s.accum == EEPROM_DATA_BITS) {
			if (m_s.locked) {
				m_s.state = IN_RESET;
				break;
			}
			if (m_s.command == WRITE)
				m_s.data[m_s.address] = static_cast<uint16_t>(m_s.shift);
			else
				for (uint16_t &word : m_s.data)
					word = static_cast<uint16_t>(word & m_s.shift);
			m_s.state = COMPLETION;
		}
		break;
	default:
		break;
	}
}

void
Eeprom93c66::command()
{
	const uint32_t op = m_s.shift >> EEPROM_ADDRESS_BITS;

	m_s.address = static_cast<uint16_t>(m_s.shift & (WORDS - 1));
	m_s.accum = 0;
	if (op == 0) {
		m_s.command = static_cast<uint8_t>(m_s.address >> (EEPROM_ADDRESS_BITS - 2));
		m_s.address = 0;
	} else {
		m_s.command = op == 1 ? WRITE : op == 2 ? READ : ERASE;
	}
	switch (m_s.command) {
	case READ:
		m_s.shift = 0;
		m_s.state = READING;
		break;
	case WRITE:
	case WRITEALL:
		m_s.shift = 0;
		m_s.state = WAIT_DATA;
		break;
	case ERASE:
		if (!m_s.locked)
			m_s.data[m_s.address] = 0xFFFF;
		m_s.state = m_s.locked ? IN_RESET : COMPLETION;
		break;
	case ERASEALL:
		if (!m_s.locked)
			m_s.data.fill(0xFFFF);
		m_s.state = m_s.locked ? IN_RESET : COMPLETION;
		break;
	default:
		m_s.locked = m_s.command == LOCK;
		m_s.state = IN_RESET;
		break;
	}
}

bool
Eeprom93c66::do_read() const
{
	if (m_s.state == READING)
		return (m_s.shift & 0x80000000u) != 0;
	return true;
}

Gaelco3d::Gaelco3d(Roms roms)
	: m_main_bus(*this), m_main16_bus(*this), m_dsp_bus(*this), m_sound_bus(*this), m_roms(std::move(roms)),
	  m_video(m_roms.texture, m_roms.texmask)
{
	/* The DSP's ROMs, as 32-bit words at 0x400000-0x7FFFFF */
	m_roms.tms.resize(0x1000000);
	m_tms_rom.resize(m_roms.tms.size() / 4);
	for (size_t i = 0; i < m_tms_rom.size(); i++) {
		const uint8_t *const p = &m_roms.tms[i * 4];

		m_tms_rom[i] = static_cast<uint32_t>(p[0]) | p[1] << 8 | p[2] << 16 | static_cast<uint32_t>(p[3]) << 24;
	}
	m_roms.tms.clear();
	m_roms.tms.shrink_to_fit();
	if (m_roms.m68000)
		m_main16 = std::make_unique<M68000>(m_main16_bus);
	else
		m_main = std::make_unique<M68020>(m_main_bus);
	m_main_clock = m_roms.m68000 ? MAIN16_CLOCK : MAIN_CLOCK;
	m_tms_cycles = m_roms.m68000 ? TMS16_CYCLES : TMS_CYCLES;
	m_tms = std::make_unique<Tms32031>(m_dsp_bus, m_roms.boot);
	m_tms->set_rom(0x400000, m_tms_rom);
	m_adsp = std::make_unique<Adsp2115>(m_sound_bus);
	if (m_roms.kick_sensors && m_roms.pic.size() >= 1024) {
		/* 512 words of 14 bits, low byte first */
		std::vector<uint16_t> program(512);

		for (size_t i = 0; i < program.size(); i++)
			program[i] = static_cast<uint16_t>((m_roms.pic[2 * i] | m_roms.pic[2 * i + 1] << 8) & 0x3FFF);
		for (std::unique_ptr<Pic16c71x> &ball : m_balls) {
			ball = std::make_unique<Pic16c71x>(program, 36, PIC_CLOCK);
			if (std::equal(BALL_WAIT.begin(), BALL_WAIT.end(), program.begin() + BALL_WAIT_PC))
				ball->set_wait_pc(BALL_WAIT_PC);
		}
	}
	m_roms.texture.clear();
	m_roms.texture.shrink_to_fit();
	m_roms.texmask.clear();
	m_roms.texmask.shrink_to_fit();
	m_adsp_thread = std::thread(&Gaelco3d::adsp_loop, this);
}

Gaelco3d::~Gaelco3d()
{
	adsp_to(ADSP_STOP);
	adsp_send();
	m_adsp_thread.join();
}

void
Gaelco3d::reset()
{
	adsp_settle();

	const auto rom16 = [this](size_t i) {
		return static_cast<uint32_t>(m_roms.adsp[2 * i] | m_roms.adsp[2 * i + 1] << 8);
	};
	const std::span<uint32_t> pm = m_adsp->program_ram();
	/* Boot page 0: (length + 1) × 8 words (ADSP-2100 Family User's Manual
	 * § 10.4.1) */
	const size_t words = std::min<size_t>(((rom16(3) & 0xFF) + 1) * 8, pm.size());

	for (size_t i = 0; i < words; i++)
		pm[i] = (rom16(i * 4) & 0xFF) << 16 | (rom16(i * 4 + 1) & 0xFF) << 8 | (rom16(i * 4 + 2) & 0xFF);
	m_adsp_bank = 0;
	m_adsp->reset();
	m_adsp->set_wait_pc(0xFFFFFFFFu);
	for (const AdspWait &wait : ADSP_WAITS)
		if (std::equal(wait.code.begin(), wait.code.end(), pm.begin() + wait.pc)) {
			m_adsp->set_wait_pc(wait.pc);
			m_adsp_wait_at = wait.at;
		}
	m_wait_counter = 0;
	m_wait16 = {};
	m_spring_at = m_gain_at = m_spring_on_at = m_hit_at = 0;
	if (m_main16) {
		m_main16->set_wait_pc(0xFFFFFFFFu);
		for (const Main16Wait &wait : MAIN16_WAITS)
			if (holds_wait(m_roms.program, wait)) {
				m_main16->set_wait_pc(wait.pc);
				m_wait16 = { wait.routine != 0, wait.flag, wait.cell };
			}
		if (const std::optional<MotorAt> at = motor_at(m_roms.program)) {
			m_spring_at = at->spring;
			m_gain_at = at->gain;
			m_spring_on_at = at->on;
			m_hit_at = at->hit;
		}
	} else {
		const std::span<const uint8_t> code(m_roms.program);

		m_main->set_wait_pc(0xFFFFFFFFu);
		for (const MainWait &wait : MAIN_WAITS)
			if (fnv(code.subspan(wait.loop, wait.loop_size)) == wait.loop_hash &&
			    fnv(code.subspan(wait.sub, wait.sub_size)) == wait.sub_hash) {
				m_main->set_wait_pc(wait.pc);
				m_wait_counter = wait.counter;
			}
	}

	m_motor = 0;
	m_motor_parts = {};
	/* The balls power up with the board: their connector JP1 has no reset
	 * line. INPUT is low, as the latch is cleared */
	for (std::unique_ptr<Pic16c71x> &ball : m_balls)
		if (ball) {
			ball->reset();
			ball->set_portb_in(0xEF);
		}
	/* HEURISTIC: the conversions at power-up are those of the inputs at
	 * rest */
	m_adc_sample = { 0x80, 0x80, 0x80, 0x80 };
	m_adc_clocks = 0;
	m_tms_reset = true;
	m_tms->set_reset(true);
	/* With the latch cleared, INT0 is low (sheet 3, IC5 Q6) */
	m_tms->set_int(0, true);
	m_mainlatch = m_outlatch = 0;
	m_vblank_irq = false;
	m_serial_irq = m_sending = false;
	/* With the latch cleared, REQBUS is low (m_rts is its inverse) */
	m_rts = true;
	m_serial_ready = m_other_rts = true;
	m_rts_of.fill(true);
	for (std::vector<uint16_t> &held : m_held)
		held.clear();
	m_serial_in = 0;
	m_out_byte = 0;
	m_seen_free = true;
	m_rx.clear();
	m_link_open = false;
	m_paced = false;
	m_ahead = 0;
	if (m_link)
		m_link->flush();
	m_sound_pending = false;
	m_sound_status = 0;
	m_adsp->set_irq(Adsp2115::IRQ2, false);
	stop_autobuffer();
	if (m_main)
		m_main->reset();
	else
		m_main16->reset();
	set_ipl();
	m_video.reset();
	m_frame = 0;
	m_slices = 0;
	m_base = { main_cycles(), m_tms->cycles(), m_adsp->cycles() };
}

void
Gaelco3d::tell_once(const char *what, uint32_t addr, uint32_t value)
{
	if (!m_verbose)
		return;
	const std::lock_guard lock(m_told_mutex);

	if (!m_told.emplace(what, addr).second)
		return;
	std::fprintf(stderr, "gaelco3d: %s %06X (%08X)\n", what, addr, value);
}

uint32_t
Gaelco3d::io_read(uint32_t addr, unsigned size)
{
	const uint32_t dword = addr & ~3u;
	const unsigned at = addr & 3;
	const bool low_byte = at + size > 3;
	const bool second_low = at <= 1 && at + size > 1;
	uint32_t v = 0;

	switch (dword) {
	case 0x51000C:
		v = 0xFFFF0000u & ~(static_cast<uint32_t>(m_switches[0]) << 16);
		break;
	case 0x51001C:
		v = 0xFFFF0000u & ~(static_cast<uint32_t>(m_switches[1]) << 16);
		break;
	case 0x51002C:
		v = 0x0FFF0000u & ~(static_cast<uint32_t>(m_switches[2]) << 24);
		if (m_balls[0]) {
			/* OUTPUT (RB1) of balls B and A */
			balls_to_main();
			v |= static_cast<uint32_t>(m_balls[1]->portb() >> 1 & 1) << 28 |
			     static_cast<uint32_t>(m_balls[0]->portb() >> 1 & 1) << 29 | BALL_D14_D15;
		} else
			/* Speed Up's two TLC548s on D12 and D13; D14 and D15 are
			 * pulled up (sheet 7) */
			for (unsigned i = 0; i < 4; i++)
				v |= static_cast<uint32_t>(m_roms.speed_up && i >= 2 ? 1 : m_analog_shift[i] >> 7) << (28 + i);
		break;
	case 0x51003C:
		v = 0xFFFF0000u & ~(static_cast<uint32_t>(m_switches[3]) << 16);
		break;
	case 0x510040:
		adsp_settle();
		v = low_byte ? m_sound_status : 0xFFFF;
		break;
	case 0x510100: {
		uint32_t eeprom = 0xFFFF;

		if (second_low) {
			serve_link();
			/* NOT IBUSY, and NOT IREQ: the OR of every board's REQBUS,
			 * including this one's (sheet 3, IC8 and IC9) */
			eeprom = (eeprom & ~3u) | (m_serial_ready ? 1u : 0u) | (m_other_rts && m_rts ? 2u : 0u);
			if (m_serial_ready)
				m_seen_free = true;
		}
		if (m_eeprom.do_read())
			eeprom ^= 0x0004;
		v = eeprom << 16;
		if (low_byte)
			v |= serial_read();
		break;
	}
	default:
		tell_once("read", addr, 0);
		break;
	}
	const unsigned shift = 8 * (4 - at - size);

	return size == 4 ? v : (v >> shift) & (size == 1 ? 0xFFu : 0xFFFFu);
}

void
Gaelco3d::io_write(uint32_t addr, uint32_t value, unsigned size)
{
	for (unsigned i = 0; i < size; i++) {
		const uint32_t at = addr + i;
		const uint8_t byte = static_cast<uint8_t>(value >> (8 * (size - 1 - i)));

		if (at == 0x510041) {
			/* The ADSP runs up to this moment; then the latch takes the
			 * byte, and the write pulse is the edge on IRQ2 (both sound
			 * programs set ICNTL = 4; note 121 § 1, X4) */
			adsp_to(m_base[2] + (main_cycles() - m_base[0]) * ADSP_CLOCK / m_main_clock);
			adsp_settle();
			m_sound_latch = byte;
			m_sound_pending = true;
			m_adsp->pulse_irq(Adsp2115::IRQ2);

			end_main_run();
		} else if (at == 0x510044 || at == 0x510045) {
			const unsigned shift = at == 0x510044 ? 8 : 0;

			m_motor = static_cast<int16_t>((m_motor & ~(0xFF << shift)) | byte << shift);
			/* The spring the program added in, from the same cells, as a
			 * word (muls.w, asr.l #4, add.w), whether it was on, and the
			 * hit played */
			if (m_spring_at) {
				m_motor_parts.spring = static_cast<int16_t>(static_cast<int16_t>(ram16(m_spring_at)) *
				                                            static_cast<int16_t>(ram16(m_gain_at)) >> 4);
				m_motor_parts.spring_on = ram16(m_spring_on_at) != 0;
				m_motor_parts.hit = static_cast<int8_t>(ram8(m_hit_at));
			}
		} else if (at == 0x510101) {
			/* /CLRINT takes the low byte only (/LDS, sheet 3 IC19) */
			m_vblank_irq = false;
			set_ipl();
		} else if ((at & ~0x38u) == 0x510103) {
			main_latch((at >> 3) & 7, byte & 1);
		} else if (at == 0x510105) {
			serial_write(byte);
		} else if ((at & ~0x70u) == 0x510107) {
			out_latch((at >> 4) & 7, byte & 1);
		} else {
			tell_once("write", at, byte);
		}
	}
}

void
Gaelco3d::main_latch(unsigned bit, bool on)
{
	m_mainlatch = static_cast<uint8_t>((m_mainlatch & ~(1u << bit)) | (on ? 1u << bit : 0));
	switch (bit) {
	case 0:
		m_sending = on;
		break;
	case 1:
		if (m_rts != !on) {
			m_rts = !on;
			if (m_link)
				link_send(m_rts ? 0x200 : 0);
		}
		break;
	case 2:
		m_eeprom.set_di(on);
		break;
	case 3:
		m_eeprom.set_clk(on);
		break;
	case 4:
		m_eeprom.set_cs(on);
		break;
	case 5:
		m_tms_reset = !on;
		m_tms->set_reset(!on);
		break;
	case 6:
		/* INT0 comes from this latch only (sheet 3): the DSP runs up to this
		 * moment, so it sees the line low for as long as the 68000 holds it */
		if (const uint64_t to = m_base[1] + (main_cycles() - m_base[0]) * m_tms_cycles / m_main_clock;
		    m_tms->cycles() < to)
			m_tms->run(static_cast<uint32_t>(to - m_tms->cycles()));
		m_tms->set_int(0, !on);
		break;
	default:
		break;
	}
}

void
Gaelco3d::out_latch(unsigned bit, bool on)
{
	const bool was = (m_outlatch >> bit) & 1;

	m_outlatch = static_cast<uint8_t>((m_outlatch & ~(1u << bit)) | (on ? 1u << bit : 0));
	if (bit == 5 && m_balls[0]) {
		/* INPUT of both balls (JP1 pin 2, RB4) */
		balls_to_main();
		for (std::unique_ptr<Pic16c71x> &ball : m_balls)
			ball->set_portb_in(on ? 0xFF : 0xEF);
		return;
	}
	/* The TLC548s (Speed Up's sheet 7; HEURISTIC for Radikal Bikers, read
	 * the same way): the next bit on each falling clock edge, a conversion
	 * on the 8th; when /CS falls, DATA OUT gives the conversion of the
	 * previous read (data sheet SLAS067 p. 2) */
	if (bit == 5 && !on && was) {
		for (uint8_t &port : m_analog_shift)
			port = static_cast<uint8_t>(port << 1);
		if (++m_adc_clocks == 8)
			m_adc_sample = m_analog;
	}
	if (bit == 6 && !on && was) {
		m_analog_shift = m_adc_sample;
		m_adc_clocks = 0;
	}
}

void
Gaelco3d::balls_to_main()
{
	const uint64_t to = (main_cycles() - m_base[0]) * PIC_CYCLES / m_main_clock;

	for (std::unique_ptr<Pic16c71x> &ball : m_balls)
		ball->run_to(to);
}

void
Gaelco3d::set_ball_sensors(const BallSensors &sensors)
{
	for (unsigned i = 0; i < m_balls.size(); i++)
		if (m_balls[i])
			for (unsigned k = 0; k < 3; k++)
				m_balls[i]->set_analog(k, sensors[i][k]);
}

uint8_t
Gaelco3d::Main::read8(uint32_t addr)
{
	if (addr < 0x200000)
		return board.m_roms.program[addr];
	if (addr >= 0xFE0000)
		return board.ram8(addr);
	if (addr >= 0x400000 && addr < 0x410000)
		return static_cast<uint8_t>(board.m_palette_ram[(addr & 0xFFFF) >> 2] >> (8 * (3 - (addr & 3))));
	return static_cast<uint8_t>(board.io_read(addr, 1));
}

uint16_t
Gaelco3d::Main::read16(uint32_t addr)
{
	if (addr < 0x200000)
		return static_cast<uint16_t>(board.m_roms.program[addr] << 8 | board.m_roms.program[addr + 1]);
	if (addr >= 0xFE0000)
		return static_cast<uint16_t>(board.m_ram[(addr & 0xFFFF) >> 2] >> ((addr & 2) ? 0 : 16));
	if (addr >= 0x400000 && addr < 0x410000)
		return static_cast<uint16_t>(board.m_palette_ram[(addr & 0xFFFF) >> 2] >> ((addr & 2) ? 0 : 16));
	return static_cast<uint16_t>(board.io_read(addr, 2));
}

uint32_t
Gaelco3d::Main::read32(uint32_t addr)
{
	if (addr < 0x200000) {
		const uint8_t *const p = &board.m_roms.program[addr];

		return static_cast<uint32_t>(p[0]) << 24 | p[1] << 16 | p[2] << 8 | p[3];
	}
	if (addr >= 0xFE0000)
		return board.m_ram[(addr & 0xFFFF) >> 2];
	if (addr >= 0x400000 && addr < 0x410000)
		return board.m_palette_ram[(addr & 0xFFFF) >> 2];
	return board.io_read(addr, 4);
}

bool
Gaelco3d::Main::waiting(uint32_t)
{
	if (board.m_wait_counter)
		return static_cast<int16_t>(board.ram16(board.m_wait_counter)) <= 0 &&
		       (board.ram16(0xFE27C4) != 0 || board.ram16(0xFE7FFC) == 0);
	if (board.ram16(0xFE42E4) != 0)
		return false;

	const uint16_t value = board.ram16(0xFE4314);
	const uint16_t wanted = (board.ram8(0xFE7F8B) & 1) ? 0x3E0 : 0;

	return board.ram16(0xFE42DA) >= 0x1F || !(board.ram8(0xFE431D) & 1) || value == wanted;
}

/* True while the cell the frame interrupt sets is still 0 and Speed Up's
 * option is clear, or Surf Planet's ready cell is already 1 (the loop
 * writes 1) */
bool
Gaelco3d::Main16::waiting(uint32_t)
{
	const Wait16 &wait = board.m_wait16;

	return board.ram16(wait.cell) == 0 &&
	       (wait.routine ? !(board.ram8(wait.flag) & 0x20) : board.ram16(wait.flag) == 1);
}

void
Gaelco3d::Main::write8(uint32_t addr, uint8_t value)
{
	const unsigned shift = 8 * (3 - (addr & 3));
	const uint32_t keep = ~(0xFFu << shift), put = static_cast<uint32_t>(value) << shift;

	if (addr >= 0xFE0000) {
		uint32_t &word = board.m_ram[(addr & 0xFFFF) >> 2];

		word = (word & keep) | put;
	} else if (addr >= 0x400000 && addr < 0x410000) {
		const uint32_t index = (addr & 0xFFFF) >> 2;
		uint32_t &word = board.m_palette_ram[index];

		word = (word & keep) | put;
		board.m_video.set_palette(index * 2 + ((addr & 2) ? 1 : 0),
		                          static_cast<uint16_t>(word >> ((addr & 2) ? 0 : 16)));
	} else if (addr >= 0x500000) {
		board.io_write(addr, value, 1);
	} else if (addr >= 0x200000) {
		board.tell_once("write", addr, value);
	}
}

void
Gaelco3d::Main::write16(uint32_t addr, uint16_t value)
{
	const unsigned shift = (addr & 2) ? 0 : 16;
	const uint32_t keep = ~(0xFFFFu << shift), put = static_cast<uint32_t>(value) << shift;

	if (addr >= 0xFE0000) {
		uint32_t &word = board.m_ram[(addr & 0xFFFF) >> 2];

		word = (word & keep) | put;
	} else if (addr >= 0x400000 && addr < 0x410000) {
		const uint32_t index = (addr & 0xFFFF) >> 2;
		uint32_t &word = board.m_palette_ram[index];

		word = (word & keep) | put;
		board.m_video.set_palette(index * 2 + ((addr & 2) ? 1 : 0), value);
	} else if (addr >= 0x500000) {
		board.io_write(addr, value, 2);
	} else if (addr >= 0x200000) {
		board.tell_once("write", addr, value);
	}
}

void
Gaelco3d::Main::write32(uint32_t addr, uint32_t value)
{
	if (addr >= 0xFE0000) {
		board.m_ram[(addr & 0xFFFF) >> 2] = value;
	} else if (addr >= 0x400000 && addr < 0x410000) {
		const uint32_t index = (addr & 0xFFFF) >> 2;

		board.m_palette_ram[index] = value;
		board.m_video.set_palette(index * 2, static_cast<uint16_t>(value >> 16));
		board.m_video.set_palette(index * 2 + 1, static_cast<uint16_t>(value));
	} else if (addr >= 0x500000) {
		board.io_write(addr, value, 4);
	} else if (addr >= 0x200000) {
		board.tell_once("write", addr, value);
	}
}

uint32_t
Gaelco3d::Dsp::read(uint32_t addr)
{
	if (addr < 0x8000) {
		const uint32_t word = board.m_ram[(addr >> 1) & 0x3FFF];

		return static_cast<uint32_t>(static_cast<int16_t>((addr & 1) ? word : word >> 16));
	}
	if (addr >= 0x400000 && addr < 0x800000)
		return board.m_tms_rom[addr - 0x400000];
	board.tell_once("tms read", addr, 0);
	return 0;
}

void
Gaelco3d::Dsp::write(uint32_t addr, uint32_t value)
{
	if (addr < 0x8000) {
		uint32_t &word = board.m_ram[(addr >> 1) & 0x3FFF];

		word = (addr & 1) ? (word & 0xFFFF0000u) | (value & 0xFFFF) : (word & 0xFFFF) | value << 16;
		return;
	}
	if (addr >= 0xC00000 && addr < 0xC00008) {
		board.m_video.word(value);
		return;
	}
	board.tell_once("tms write", addr, value);
}

/* The C31 acknowledges INT0 just before the first word of its frame. Its
 * IACK goes only to CUS-RASTER (sheet 4): INT0 stays as the 68000 holds it */
void
Gaelco3d::Dsp::iack(uint32_t)
{
	board.m_video.frame_start();
}

uint16_t
Gaelco3d::Sound::data_read(uint16_t addr)
{
	if (addr < 0x2000) {
		const size_t at = (static_cast<size_t>(board.m_adsp_bank) * 0x2000 + addr) * 2;

		return static_cast<uint16_t>(board.m_roms.adsp[at] | board.m_roms.adsp[at + 1] << 8);
	}
	if (addr == 0x2000) {
		board.m_sound_pending = false;
		return board.m_sound_latch;
	}
	board.tell_once("adsp read", addr, 0);
	return 0;
}

void
Gaelco3d::Sound::data_write(uint16_t addr, uint16_t value)
{
	if (addr < 2) {
		board.m_adsp_bank = (addr & 1) * 0x80 + (value & 0x7F);
		return;
	}
	if (addr == 0x2000) {
		board.m_sound_status = static_cast<uint8_t>(value);
		return;
	}
	board.tell_once("adsp write", addr, value);
}

bool
Gaelco3d::Sound::waiting(uint32_t)
{
	const std::span<uint16_t> ram = board.m_adsp->data_ram();
	const unsigned at = board.m_adsp_wait_at;
	const int32_t ahead = static_cast<int16_t>(ram[at]) - static_cast<int16_t>(ram[at + 1]);

	return ahead < 0 && ram[at + 8] == 0;
}

void
Gaelco3d::Sound::control_written(unsigned reg, uint16_t value)
{
	if ((reg == Adsp2115::SYSCONTROL && !(value & 0x800)) || (reg == Adsp2115::S1_AUTOBUF && !(value & 2)))
		board.stop_autobuffer();
}

void
Gaelco3d::Sound::sport_tx(unsigned port, uint32_t)
{
	if (port == 1)
		board.start_autobuffer();
}

void
Gaelco3d::start_autobuffer()
{
	const auto &control = m_adsp->control();

	if (!(control[Adsp2115::SYSCONTROL] & 0x800) || !(control[Adsp2115::S1_AUTOBUF] & 2)) {
		stop_autobuffer();
		return;
	}
	m_ab_ireg = (control[Adsp2115::S1_AUTOBUF] >> 9) & 7;

	const unsigned mreg = ((control[Adsp2115::S1_AUTOBUF] >> 7) & 3) | (m_ab_ireg & 4);

	m_ab_incs = m_adsp->m(mreg);
	m_ab_size = m_adsp->l(m_ab_ireg);
	if (m_ab_incs == 0 || m_ab_size == 0) {
		stop_autobuffer();
		return;
	}
	const uint16_t source = static_cast<uint16_t>(m_adsp->i(m_ab_ireg) - m_ab_incs);

	m_adsp->set_i(m_ab_ireg, source);
	m_ab_base = source;

	const uint64_t frame = 2ull * (control[Adsp2115::S1_SCLKDIV] + 1) * 16 * SOUND_CHANNELS;

	m_ab_period = std::max<uint64_t>(1, frame * m_ab_size / (SOUND_CHANNELS * m_ab_incs));
	m_ab_next = m_adsp->cycles() + m_ab_period;
	m_ab_on = true;
}

void
Gaelco3d::stop_autobuffer()
{
	m_ab_on = false;
}

void
Gaelco3d::autobuffer()
{
	int32_t reg = m_adsp->i(m_ab_ireg);
	const std::span<uint16_t> ram = m_adsp->data_ram();
	const uint32_t frames = m_ab_size / (SOUND_CHANNELS * m_ab_incs);
	const int32_t from = (reg - 0x3800) / 2;
	const bool muted = m_main16 && m_frame < AMPLIFIERS_MUTED_FIELDS;

	for (uint32_t f = 0; f < frames; f++) {
		std::array<float, SOUND_CHANNELS> ch{};

		for (unsigned c = 0; c < SOUND_CHANNELS && !muted; c++) {
			const int32_t at = from + static_cast<int32_t>(c * m_ab_incs + f * SOUND_CHANNELS * m_ab_incs);

			if (at >= 0 && static_cast<size_t>(at) < ram.size())
				ch[c] = static_cast<int16_t>(ram[static_cast<size_t>(at)]);
		}
		const auto put = [](std::vector<int16_t> &out, float left, float right) {
			out.push_back(static_cast<int16_t>(std::clamp(left, -32768.0f, 32767.0f)));
			out.push_back(static_cast<int16_t>(std::clamp(right, -32768.0f, 32767.0f)));
		};

		if (m_roms.speed_up) {
			/* The seat's left and right speakers, its woofer, and
			 * the dashboard's two speakers (note 121 § 1.1) */
			put(m_sound, ch[3], ch[3]);
			put(m_option_sound, ch[0], ch[1]);
			m_woofer_sound.push_back(static_cast<int16_t>(ch[2]));
		} else {
			/* The sound tests play their channel 1 on slot 3, channel 2 on
			 * slot 0 and channel 3 on slot 1, which the manuals call the
			 * left speaker, the right speaker and the woofer; slot 2 goes
			 * to no known speaker */
			put(m_sound, ch[3], ch[0]);
			put(m_option_sound, ch[1], ch[2]);
		}
	}
	reg += static_cast<int32_t>(m_ab_size);
	if (reg >= m_ab_base + static_cast<int32_t>(m_ab_size)) {
		reg = m_ab_base;
		m_adsp->pulse_irq(Adsp2115::IRQ1);
	}
	m_adsp->set_i(m_ab_ireg, static_cast<uint16_t>(reg));
}

void
Gaelco3d::set_ipl()
{
	const unsigned level = m_serial_irq ? 6 : m_vblank_irq ? 2 : 0;

	if (m_main)
		m_main->set_ipl(level);
	else
		m_main16->set_ipl(level);
}

void
Gaelco3d::main_run(uint32_t cycles)
{
	if (m_main)
		m_main->run(cycles);
	else
		m_main16->run(cycles);
}

void
Gaelco3d::end_main_run()
{
	if (m_main)
		m_main->end_run();
	else
		m_main16->end_run_at(m_main16->cycles());
}

/* /COMWR: the byte goes onto the link's eight wires (sheet 3, IC11; no
 * UART, no baud rate), and IRQ 6 and BUSY are set on every board, this one
 * included */
void
Gaelco3d::serial_write(uint8_t byte)
{
	m_out_byte = byte;
	m_serial_ready = false;
	m_serial_irq = true;
	set_ipl();
	m_link_open = true;
	if (m_link)
		link_send(static_cast<uint16_t>(0x100 | (m_rts ? 0x200 : 0) | byte));
}

/* /COMRD: reads the wires as they are (this board's own byte while it
 * drives them) and releases its IRQ 6 and BUSY. HEURISTIC: the other
 * boards have read the byte too, so the bus is free again */
uint8_t
Gaelco3d::serial_read()
{
	serve_link();
	m_serial_irq = false;
	set_ipl();
	m_serial_ready = true;
	/* A byte from another cabinet was taken */
	if (!m_sending)
		m_seen_free = false;
	return m_sending ? m_out_byte : m_serial_in;
}

/* HEURISTIC: another cabinet's next byte comes once the program has seen
 * the bus free after the last one (all the programs wait for that before
 * sending the next byte) */
void
Gaelco3d::serve_link()
{
	if (!m_link)
		return;
	while (!m_rx.empty() && !m_serial_irq && !m_sending && m_serial_ready && m_seen_free) {
		const uint16_t word = m_rx.front();

		m_rx.pop_front();
		m_rts_of[word >> 12] = (word & 0x200) != 0;
		m_other_rts = std::all_of(m_rts_of.begin(), m_rts_of.end(), [](bool rts) { return rts; });
		if (word & 0x100) {
			m_serial_in = static_cast<uint8_t>(word);
			m_serial_ready = false;
			m_serial_irq = true;
			m_seen_free = false;
			set_ipl();
		}
	}
}

/* The link is a bus shared by up to four Speed Up cabinets. Each word
 * carries the number of the cabinet that sent it and travels around the
 * ring to every other cabinet; frame ticks only go to the next cabinet.
 * HEURISTIC: words arrive a frame late, too late for the programs to stay
 * off the bus, so what a cabinet sends while it holds RTS arrives in one
 * piece, and nothing arrives while a cabinet is sending */
void
Gaelco3d::link_send(uint16_t word)
{
	m_link->send(static_cast<uint16_t>(word | m_cabinet << 12));
}

void
Gaelco3d::drain_link()
{
	while (const std::optional<uint16_t> word = m_link->take()) {
		if (m_cabinet && *word >> 12 == m_cabinet)
			continue;
		if (*word & 0x400) {
			m_ahead = m_paced ? m_ahead - 1 : 0;
			m_paced = true;
			continue;
		}
		if (m_cabinet)
			m_link->send(*word);
		if (!m_link_open)
			continue;

		std::vector<uint16_t> &held = m_held[*word >> 12];

		held.push_back(*word);
		if (*word & 0x200) {
			m_rx.insert(m_rx.end(), held.begin(), held.end());
			held.clear();
		}
	}
}

/* HEURISTIC: linked cabinets stay within a frame of each other; a cabinet
 * that is silent for 2 s is given up on */
void
Gaelco3d::pace_link()
{
	if (!m_link)
		return;
	drain_link();

	const auto start = std::chrono::steady_clock::now();

	while (m_paced && m_ahead > 1) {
		if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(2000)) {
			m_paced = false;
			m_ahead = 0;
			break;
		}
		std::this_thread::sleep_for(std::chrono::microseconds(200));
		drain_link();
	}
}

bool
Gaelco3d::set_link(const NetLinkSettings &link)
{
	m_link.reset();
	m_link_failed = false;
	if (link.board != NetLinkSettings::Board::RING)
		return true;
	m_cabinet = std::min(link.cabinet, 15u);
	m_link = std::make_unique<GaelcoLink>(m_name, link);
	if (!m_link->ok()) {
		m_link.reset();
		m_link_failed = true;
		return false;
	}
	return true;
}

void
Gaelco3d::run_adsp_to(uint64_t to)
{
	while (m_adsp->cycles() < to) {
		uint64_t until = to;

		if (m_ab_on && m_ab_next < until)
			until = m_ab_next;
		if (m_adsp->cycles() < until)
			m_adsp->run(static_cast<uint32_t>(until - m_adsp->cycles()));
		while (m_ab_on && m_adsp->cycles() >= m_ab_next) {
			autobuffer();
			m_ab_next += m_ab_period;
		}
	}
}

void
Gaelco3d::adsp_to(uint64_t to)
{
	/* The ring is full: send what was given and let the ADSP make room */
	while (m_adsp_given - m_adsp_done.load(std::memory_order_acquire) >= ADSP_GOALS) {
		adsp_send();
		relax();
	}
	m_adsp_goals[m_adsp_given % ADSP_GOALS] = to;
	m_adsp_given++;
	if (m_adsp_given - m_adsp_sent.load(std::memory_order_relaxed) >= ADSP_BATCH)
		adsp_send();
}

void
Gaelco3d::adsp_send()
{
	m_adsp_sent.store(m_adsp_given);
	if (m_adsp_resting.load())
		m_adsp_sent.notify_one();
}

void
Gaelco3d::adsp_settle()
{
	if (m_adsp_done.load(std::memory_order_acquire) == m_adsp_given)
		return;
	adsp_send();
	while (m_adsp_done.load(std::memory_order_acquire) != m_adsp_given)
		relax();
}

/* Polls a little for the next goals, then sleeps until they are sent */
void
Gaelco3d::adsp_loop()
{
	uint32_t done = 0;

	for (;;) {
		uint32_t sent = m_adsp_sent.load(std::memory_order_acquire);

		for (unsigned i = 0; sent == done && i < 256; i++) {
			relax();
			sent = m_adsp_sent.load(std::memory_order_acquire);
		}
		if (sent == done) {
			m_adsp_resting.store(true);
			if (m_adsp_sent.load() == done)
				m_adsp_sent.wait(done);
			m_adsp_resting.store(false);
			continue;
		}
		for (; done != sent; done++) {
			const uint64_t to = m_adsp_goals[done % ADSP_GOALS];

			if (to == ADSP_STOP)
				return;
			run_adsp_to(to);
			m_adsp_done.store(done + 1, std::memory_order_release);
		}
	}
}

void
Gaelco3d::run_frame()
{
	m_sound.clear();
	m_option_sound.clear();
	m_woofer_sound.clear();
	pace_link();
	for (uint64_t s = 0; s < SLICES_PER_FRAME; s++) {
		const uint64_t slice = ++m_slices;
		const uint64_t main_to = m_base[0] + target(slice, m_main_clock);
		const uint64_t tms_to = m_base[1] + target(slice, m_tms_cycles);
		const uint64_t adsp_end = m_base[2] + target(slice, ADSP_CLOCK);

		serve_link();

		while (main_cycles() < main_to) {
			main_run(static_cast<uint32_t>(main_to - main_cycles()));
			adsp_to(m_base[2] + (main_cycles() - m_base[0]) * ADSP_CLOCK / m_main_clock);
		}
		if (m_tms->cycles() < tms_to)
			m_tms->run(static_cast<uint32_t>(tms_to - m_tms->cycles()));
		for (std::unique_ptr<Pic16c71x> &ball : m_balls)
			if (ball)
				ball->run_to(target(slice, PIC_CYCLES));
		adsp_to(adsp_end);
	}

	/* A new scene each field: the designer's timing slide, and the main
	 * loops step once per vertical blank */
	m_video.next_field();
	m_video.scan_out();
	/* All the samples of the frame are made */
	adsp_settle();
	m_vblank_irq = true;
	set_ipl();
	m_frame++;
	if (m_link) {
		link_send(static_cast<uint16_t>(0x400 | (m_frame & 0xFF)));
		if (m_paced)
			m_ahead++;
	}
}

namespace {

/* Where a game keeps its settings in the EEPROM and how they are checked:
 * Radikal Bikers sums words 0xF7 to 0xFF into 0xF6, inverted; Speed Up sums
 * words 0 to 0x52 into 0x53, with the inverse in 0x54. Speed Up only uses
 * its stored switches (0x4F) once bit 1 of 0x52 is set, as its test menu
 * does on SAVE AND EXIT. */
struct Settings {
	unsigned from, to, sum, inverse;
	bool inverted;
	unsigned kept;
	uint16_t kept_bit;
};
constexpr Settings SETTINGS = { 0xF7, 0x100, 0xF6, 0, true, 0, 0 },
		   SPEEDUP_SETTINGS = { 0, 0x53, 0x53, 0x54, false, 0x52, 2 };
/* Surf Planet's Machine num. (1 or 2) and Link mode (0 on, 1 off), with no
 * checksum */
constexpr unsigned SURF_MACHINE = 0xE9, SURF_LINK = 0xE8;

uint16_t
settings_sum(const Settings &settings, std::span<const uint16_t> words)
{
	uint32_t sum = 0;

	for (unsigned i = settings.from; i < settings.to; i++)
		sum += words[i];
	return static_cast<uint16_t>(settings.inverted ? ~sum : sum);
}

bool
settings_kept(const Settings &settings, std::span<const uint16_t> words)
{
	const uint16_t sum = settings_sum(settings, words);

	return words[settings.sum] == sum && (settings.inverted || words[settings.inverse] == static_cast<uint16_t>(~sum));
}

} /* namespace */

bool
Gaelco3d::set_eeprom_setting(uint32_t byte, unsigned shift, unsigned bits, uint8_t value)
{
	const Settings &settings = m_roms.speed_up ? SPEEDUP_SETTINGS : SETTINGS;
	const std::span<uint16_t> words = m_eeprom.data();

	if (m_roms.surf_planet) {
		if (byte / 2 != SURF_MACHINE || words[SURF_LINK] > 1)
			return false;
		words[SURF_LINK] = value ? 0 : 1;
		words[SURF_MACHINE] = std::max<uint16_t>(value, 1);
		return true;
	}

	if (byte / 2 < settings.from || byte / 2 >= settings.to || !settings_kept(settings, words))
		return false;

	uint16_t &word = words[byte / 2];
	const unsigned at = shift + ((byte & 1) ? 8 : 0);
	const auto mask = static_cast<uint16_t>(((1u << bits) - 1) << at);

	word = static_cast<uint16_t>((word & ~mask) | ((static_cast<unsigned>(value) << at) & mask));
	words[settings.kept] |= settings.kept_bit;
	words[settings.sum] = settings_sum(settings, words);
	if (!settings.inverted)
		words[settings.inverse] = static_cast<uint16_t>(~words[settings.sum]);
	return true;
}

bool
Gaelco3d::load_kept(const std::string &path)
{
	std::ifstream in(path, std::ios::binary);

	if (!in)
		return false;

	std::array<uint16_t, Eeprom93c66::WORDS> words{};

	in.read(reinterpret_cast<char *>(words.data()), sizeof(words));
	if (in.gcount() != static_cast<std::streamsize>(sizeof(words))) {
		std::fprintf(stderr, "ERROR: '%s' is not an EEPROM of %s\n", path.c_str(), m_name.c_str());
		return false;
	}
	std::copy(words.begin(), words.end(), m_eeprom.data().begin());
	return true;
}

bool
Gaelco3d::save_kept(const std::string &path) const
{
	const auto &words = m_eeprom.state().data;
	const std::string fresh = path + ".new";
	std::error_code error;

	std::filesystem::create_directories(std::filesystem::path(path).parent_path(), error);
	{
		std::ofstream out(fresh, std::ios::binary);

		out.write(reinterpret_cast<const char *>(words.data()), sizeof(words));
		if (!out) {
			std::fprintf(stderr, "ERROR: gaelco3d: cannot write '%s'\n", fresh.c_str());
			return false;
		}
	}
	std::filesystem::rename(fresh, path, error);
	if (error)
		std::fprintf(stderr, "ERROR: gaelco3d: cannot write '%s': %s\n", path.c_str(), error.message().c_str());
	return !error;
}

bool
Gaelco3d::save_state(const std::string &path)
{
	adsp_settle();

	StateFile state(path, StateFile::Mode::SAVE);

	if (!state.ok()) {
		std::fprintf(stderr, "ERROR: save state: cannot write '%s'\n", path.c_str());
		return false;
	}
	char name[32] = "";
	auto main = std::make_unique<M68020State>();
	auto main16 = std::make_unique<M68000State>();
	auto tms = std::make_unique<Tms32031State>();
	auto adsp = std::make_unique<Adsp2115State>();
	std::array<Pic16c71xState, 2> balls;
	/* The last word: the field length its times are counted in */
	const uint64_t words[] = { m_slices, m_ab_period, m_ab_next,
		                   m_adsp_bank, m_ab_ireg, static_cast<uint32_t>(m_ab_base), m_ab_incs, m_ab_size,
		                   HALF_LINES_PER_FIELD };
	const uint8_t bytes[] = { m_mainlatch, m_outlatch, m_tms_reset, m_vblank_irq, m_sound_latch, m_sound_status,
		                  m_sound_pending, m_ab_on, m_analog_shift[0], m_analog_shift[1], m_analog_shift[2],
		                  m_analog_shift[3], m_serial_ready, m_serial_irq, m_sending, m_rts, m_other_rts,
		                  m_serial_in, m_out_byte, m_seen_free, m_adc_sample[0],
		                  m_adc_sample[1], m_adc_sample[2], m_adc_sample[3], m_adc_clocks };

	std::strncpy(name, m_name.c_str(), sizeof(name) - 1);
	std::memset(main.get(), 0, sizeof(*main));
	std::memset(main16.get(), 0, sizeof(*main16));
	std::memset(tms.get(), 0, sizeof(*tms));
	std::memset(adsp.get(), 0, sizeof(*adsp));
	if (m_main)
		m_main->save(*main);
	else
		m_main16->save(*main16);
	m_tms->save(*tms);
	m_adsp->save(*adsp);
	std::memset(balls.data(), 0, sizeof(balls));
	for (unsigned i = 0; i < balls.size(); i++)
		if (m_balls[i])
			m_balls[i]->save(balls[i]);

	const bool ok = state.put(name) && state.put(m_frame) && state.put(words) && state.put(m_base) &&
	                state.put(bytes) && (m_main ? state.put(*main) : state.put(*main16) && state.put(main_cycles())) &&
	                state.put(*tms) && state.put(*adsp) && state.put(balls) && state.put(m_eeprom.state()) &&
	                state.put(m_ram) && state.put(m_palette_ram) && m_video.save(state);

	if (!ok)
		std::fprintf(stderr, "ERROR: save state: cannot write '%s'\n", path.c_str());
	return ok;
}

bool
Gaelco3d::load_state(const std::string &path)
{
	StateFile state(path, StateFile::Mode::LOAD);

	if (!state.ok()) {
		std::fprintf(stderr, "ERROR: load state: '%s' is missing or is not a state of this build\n", path.c_str());
		return false;
	}
	char name[32] = "";

	if (!state.get(name) || m_name.compare(0, sizeof(name) - 1, name) != 0) {
		std::fprintf(stderr, "ERROR: load state: '%s' is not a state of %s\n", path.c_str(), m_name.c_str());
		return false;
	}
	reset();

	auto main = std::make_unique<M68020State>();
	auto main16 = std::make_unique<M68000State>();
	uint64_t main16_cycles = 0;
	auto tms = std::make_unique<Tms32031State>();
	auto adsp = std::make_unique<Adsp2115State>();
	auto eeprom = std::make_unique<Eeprom93c66::State>();
	std::array<Pic16c71xState, 2> balls{};
	uint64_t words[9];
	std::array<uint64_t, 3> base{};
	uint8_t bytes[25];
	unsigned frame = 0;

	const bool ok = state.get(frame) && state.get(words) && state.get(base) && state.get(bytes) &&
	                (m_main ? state.get(*main) : state.get(*main16) && state.get(main16_cycles)) && state.get(*tms) &&
	                state.get(*adsp) && state.get(balls) && state.get(*eeprom) && state.get(m_ram) && state.get(m_palette_ram) &&
	                m_video.load(state);

	if (!ok || words[8] != HALF_LINES_PER_FIELD) {
		std::fprintf(stderr, "ERROR: load state: '%s' is not a state of this build\n", path.c_str());
		reset();
		return false;
	}
	m_frame = frame;
	if (m_main) {
		m_main->load(*main);
	} else {
		m_main16->load(*main16);
		m_main16->skip(main16_cycles - m_main16->cycles());
	}
	m_tms->load(*tms);
	m_adsp->load(*adsp);
	for (unsigned i = 0; i < balls.size(); i++)
		if (m_balls[i])
			m_balls[i]->load(balls[i]);
	m_eeprom.set_state(*eeprom);
	m_slices = words[0];
	m_base = base;
	m_ab_period = words[1];
	m_ab_next = words[2];
	m_adsp_bank = static_cast<uint32_t>(words[3]);
	m_ab_ireg = static_cast<unsigned>(words[4]);
	m_ab_base = static_cast<int32_t>(static_cast<uint32_t>(words[5]));
	m_ab_incs = static_cast<uint32_t>(words[6]);
	m_ab_size = static_cast<uint32_t>(words[7]);
	m_mainlatch = bytes[0];
	m_outlatch = bytes[1];
	m_tms_reset = bytes[2];
	m_vblank_irq = bytes[3];
	m_sound_latch = bytes[4];
	m_sound_status = bytes[5];
	m_sound_pending = bytes[6];
	m_ab_on = bytes[7];
	std::copy(bytes + 8, bytes + 12, m_analog_shift.begin());
	m_serial_ready = bytes[12];
	m_serial_irq = bytes[13];
	m_sending = bytes[14];
	m_rts = bytes[15];
	m_other_rts = bytes[16];
	m_serial_in = bytes[17];
	m_out_byte = bytes[18];
	m_seen_free = bytes[19];
	std::copy(bytes + 20, bytes + 24, m_adc_sample.begin());
	m_adc_clocks = bytes[24];
	set_ipl();
	for (uint32_t i = 0; i < m_palette_ram.size(); i++) {
		m_video.set_palette(i * 2, static_cast<uint16_t>(m_palette_ram[i] >> 16));
		m_video.set_palette(i * 2 + 1, static_cast<uint16_t>(m_palette_ram[i]));
	}
	m_video.scan_out();
	return true;
}
}
