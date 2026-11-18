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

/* Based on MAME's gaelco3d driver (BSD-3-Clause, Aaron Giles) */

#include "odin/machine/boards/gaelco3d/gaelco3d.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <filesystem>
#include <thread>
#include <fstream>
#include <iterator>
#include <memory>

#include "odin/machine/boards/atvtrack/atvtrack.hpp"
#include "odin/machine/core/state_file.hpp"

namespace odin {
namespace {
constexpr uint64_t MAIN_CLOCK = 25'000'000, MAIN16_CLOCK = 15'000'000;
constexpr uint64_t TMS_CYCLES = 25'000'000, TMS16_CYCLES = 30'000'000;
constexpr uint64_t ADSP_CLOCK = 16'000'000;

constexpr uint64_t SLICES_PER_SECOND = 12'000, SLICES_PER_FRAME = SLICES_PER_SECOND / 60;
constexpr uint64_t SERIAL_RATE = 115200 * 4 / 10;
constexpr unsigned SOUND_CHANNELS = 4;
constexpr float SOUND_LEVEL = 0.45f, SPEAKER_LEVEL = 1.0f;

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
/* The main CPU waits a dozen NOPs between the bytes of a sound command, so
 * the ADSP takes each byte before the next one arrives */
constexpr unsigned LATCH_STEPS = 64, LATCH_STEP = 16;
/* At power-on Speed Up's motor turns the wheel to each end so the
 * program learns its travel. Nothing turns the player's wheel, so while the
 * motor pushes during the first frames, the wheel reads as being at that end */
constexpr unsigned MOTOR_SWEEP_FRAMES = 900;
/* Football Power hits a TRAP #1 breakpoint in play, whose monitor
 * waits for a debug host; the vector is pointed at the RTE at 0x246 */
constexpr uint32_t TRAP1_VECTOR = 0x84, TRAP1_HANDLER = 0x184, TRAP1_RTE = 0x246;

uint64_t
fnv(std::span<const uint8_t> bytes)
{
	uint64_t h = 0xcbf29ce484222325ull;

	for (const uint8_t b : bytes)
		h = (h ^ b) * 0x100000001b3ull;
	return h;
}

enum Control : unsigned { S1_AUTOBUF = 15, S1_SCLKDIV = 17, SYSCONTROL = 31 };

uint64_t
target(uint64_t slices, uint64_t rate)
{
	return slices * rate / SLICES_PER_SECOND;
}
}

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
	  m_palette(32768), m_screen(WIDTH * HEIGHT), m_zbuffer(WIDTH * HEIGHT), m_polydata(4096 * 21),
	  m_picture(WIDTH * HEIGHT, 0xFF000000u)
{
	m_roms.tms.resize(0x1000000);
	if (m_roms.m68000)
		m_main16 = std::make_unique<M68000>(m_main16_bus);
	else
		m_main = std::make_unique<M68020>(m_main_bus);
	m_main_clock = m_roms.m68000 ? MAIN16_CLOCK : MAIN_CLOCK;
	m_tms_cycles = m_roms.m68000 ? TMS16_CYCLES : TMS_CYCLES;
	m_serial_byte = m_main_clock / SERIAL_RATE;
	m_tms = std::make_unique<Tms32031>(m_dsp_bus, m_roms.boot);
	m_adsp = std::make_unique<Adsp2115>(m_sound_bus);
	expand_textures(m_roms.texture, m_roms.texmask);
	m_roms.texture.clear();
	m_roms.texture.shrink_to_fit();
	m_roms.texmask.clear();
	m_roms.texmask.shrink_to_fit();
}

Gaelco3d::~Gaelco3d() = default;

void
Gaelco3d::reset()
{
	const auto rom16 = [this](size_t i) {
		return static_cast<uint32_t>(m_roms.adsp[2 * i] | m_roms.adsp[2 * i + 1] << 8);
	};
	const std::span<uint32_t> pm = m_adsp->program_ram();
	const size_t words = std::min<size_t>((rom16(3) & 0xFF) * 8, pm.size());

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
	{
		const auto at32 = [this](uint32_t a) {
			return static_cast<uint32_t>(m_roms.program[a]) << 24 | m_roms.program[a + 1] << 16 |
			       m_roms.program[a + 2] << 8 | m_roms.program[a + 3];
		};

		m_trap1_rte = at32(TRAP1_VECTOR) == TRAP1_HANDLER && at32(TRAP1_HANDLER) == 0x33FC0000u &&
		              at32(TRAP1_HANDLER + 4) == 0x00FE7FCCu && (at32(TRAP1_RTE) >> 16) == 0x4E73;
	}
	m_wait_counter = 0;
	if (m_main)
		m_main->set_wait_pc(0xFFFFFFFFu);
	for (const MainWait &wait : MAIN_WAITS) {
		const std::span<const uint8_t> code(m_roms.program);

		if (m_main && fnv(code.subspan(wait.loop, wait.loop_size)) == wait.loop_hash &&
		    fnv(code.subspan(wait.sub, wait.sub_size)) == wait.sub_hash) {
			m_main->set_wait_pc(wait.pc);
			m_wait_counter = wait.counter;
		}
	}

	m_motor = 0;
	m_kick_clock = 27;
	m_kick_state = false;
	m_kick_ports = {};
	m_kick_length = {};
	m_tms_reset = true;
	m_tms->set_reset(true);
	m_tms->set_int(0, false);
	m_mainlatch = m_outlatch = 0;
	m_vblank_irq = false;
	m_serial_irq = m_sending = m_rts = false;
	m_serial_ready = m_other_rts = true;
	m_rts_of.fill(true);
	for (std::vector<uint16_t> &held : m_held)
		held.clear();
	m_serial_ready_at = 0;
	m_serial_in = 0;
	m_rx.clear();
	m_rx_at = 0;
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
	m_polydata_count = 0;
	forget_images();
	m_frame = 0;
	m_slices = 0;
	m_base = { main_cycles(), m_tms->cycles(), m_adsp->cycles() };
}

void
Gaelco3d::tell_once(const char *what, uint32_t addr, uint32_t value)
{
	if (!m_verbose || !m_told.insert(static_cast<uint64_t>(addr) << 8 | static_cast<uint8_t>(what[0])).second)
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
		if (m_roms.kick_sensors)
			v |= ((m_kick_ports[1] >> m_kick_clock) & 1) << 28 | ((m_kick_ports[0] >> m_kick_clock) & 1) << 29;
		else
			for (unsigned i = 0; i < 4; i++)
				v |= static_cast<uint32_t>(m_analog_shift[i] >> 7) << (28 + i);
		break;
	case 0x51003C:
		v = 0xFFFF0000u & ~(static_cast<uint32_t>(m_switches[3]) << 16);
		break;
	case 0x510040:
		v = low_byte ? m_sound_status : 0xFFFF;
		break;
	case 0x510100: {
		uint32_t eeprom = 0xFFFF;

		if (second_low) {
			serve_link();
			eeprom = (eeprom & ~3u) | (m_serial_ready ? 1u : 0u) | (m_other_rts ? 2u : 0u);
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
			for (unsigned step = 0; m_sound_pending && step < LATCH_STEPS; step++)
				run_adsp_to(m_adsp->cycles() + LATCH_STEP);
			m_sound_latch = byte;
			m_sound_pending = true;
			m_adsp->set_irq(Adsp2115::IRQ2, true);

			end_main_run();
		} else if (at == 0x510044 || at == 0x510045) {
			const unsigned shift = at == 0x510044 ? 8 : 0;

			m_motor = static_cast<int16_t>((m_motor & ~(0xFF << shift)) | byte << shift);
		} else if (at == 0x510100 || at == 0x510101) {
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
		m_tms->set_int(0, !on);
		break;
	default:
		break;
	}
}

void
Gaelco3d::out_latch(unsigned bit, bool on)
{
	m_outlatch = static_cast<uint8_t>((m_outlatch & ~(1u << bit)) | (on ? 1u << bit : 0));
	if (bit == 5 && m_roms.kick_sensors) {
		kick_clock(on);
		return;
	}
	if (bit == 5 && !on)
		for (uint8_t &port : m_analog_shift)
			port = static_cast<uint8_t>(port << 1);
	if (bit == 6 && !on) {
		m_analog_shift = m_analog;
		if (m_roms.speed_up && m_frame < MOTOR_SWEEP_FRAMES && m_motor)
			m_analog_shift[0] = m_motor > 0 ? 0x00 : 0xFF;
	}
}

/* Football Power's ball sensors, 28 bits per read: the right, left and
 * middle sensors, and bit 1 set unless the ball moves away from rest */
void
Gaelco3d::kick_clock(bool on)
{
	if (on == m_kick_state)
		return;
	m_kick_state = on;
	if (++m_kick_clock < 28)
		return;
	m_kick_clock = 0;
	for (unsigned i = 0; i < 2; i++) {
		const std::array<uint8_t, 3> &ball = m_ball[i];
		uint32_t length = 0;

		for (unsigned k = 0; k < 3; k++) {
			const int32_t d = static_cast<int32_t>(ball[k]) - BALL_AT_REST[k];

			length += static_cast<uint32_t>(d * d);
		}
		m_kick_ports[i] = static_cast<uint32_t>(ball[2]) << 18 | static_cast<uint32_t>(ball[1]) << 10 |
		                  static_cast<uint32_t>(ball[0]) << 2 | (length <= m_kick_length[i] ? 2u : 0u) | 1;
		m_kick_length[i] = length;
	}
}

void
Gaelco3d::palette_write(uint32_t index, uint16_t value)
{
	m_palette[index] = static_cast<uint32_t>(value & 0x7FE0) << 6 | (value & 0x1F);
	m_palette_changed = true;
}

uint8_t
Gaelco3d::Main::read8(uint32_t addr)
{
	if (addr < 0x200000)
		return board.m_roms.program[addr];
	if (addr >= 0xFE0000)
		return static_cast<uint8_t>(board.m_ram[(addr & 0xFFFF) >> 2] >> (8 * (3 - (addr & 3))));
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
	if (addr == TRAP1_VECTOR && board.m_trap1_rte)
		return TRAP1_RTE;
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
		board.palette_write(index * 2 + ((addr & 2) ? 1 : 0), static_cast<uint16_t>(word >> ((addr & 2) ? 0 : 16)));
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
		board.palette_write(index * 2 + ((addr & 2) ? 1 : 0), value);
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
		board.palette_write(index * 2, static_cast<uint16_t>(value >> 16));
		board.palette_write(index * 2 + 1, static_cast<uint16_t>(value));
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
	if (addr >= 0x400000 && addr < 0x800000) {
		const uint8_t *const p = &board.m_roms.tms[(addr - 0x400000) * 4];

		return static_cast<uint32_t>(p[0]) | p[1] << 8 | p[2] << 16 | static_cast<uint32_t>(p[3]) << 24;
	}
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
		board.render_write(value);
		return;
	}
	board.tell_once("tms write", addr, value);
}

void
Gaelco3d::Dsp::iack(uint32_t)
{
	board.m_tms->set_int(0, false);
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
		board.m_adsp->set_irq(Adsp2115::IRQ2, false);
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
	if ((reg == SYSCONTROL && !(value & 0x800)) || (reg == S1_AUTOBUF && !(value & 2)))
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

	if (!(control[SYSCONTROL] & 0x800) || !(control[S1_AUTOBUF] & 2)) {
		stop_autobuffer();
		return;
	}
	m_ab_ireg = (control[S1_AUTOBUF] >> 9) & 7;

	const unsigned mreg = ((control[S1_AUTOBUF] >> 7) & 3) | (m_ab_ireg & 4);

	m_ab_incs = m_adsp->m(mreg);
	m_ab_size = m_adsp->l(m_ab_ireg);
	if (m_ab_incs == 0 || m_ab_size == 0) {
		stop_autobuffer();
		return;
	}
	const uint16_t source = static_cast<uint16_t>(m_adsp->i(m_ab_ireg) - m_ab_incs);

	m_adsp->set_i(m_ab_ireg, source);
	m_ab_base = source;

	const uint64_t frame = 2ull * (control[S1_SCLKDIV] + 1) * 16 * SOUND_CHANNELS;

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

	for (uint32_t f = 0; f < frames; f++) {
		std::array<float, SOUND_CHANNELS> ch{};

		for (unsigned c = 0; c < SOUND_CHANNELS; c++) {
			const int32_t at = from + static_cast<int32_t>(c * m_ab_incs + f * SOUND_CHANNELS * m_ab_incs);

			if (at >= 0 && static_cast<size_t>(at) < ram.size())
				ch[c] = static_cast<int16_t>(ram[static_cast<size_t>(at)]);
		}
		const auto put = [](std::vector<int16_t> &out, float left, float right) {
			out.push_back(static_cast<int16_t>(std::clamp(left, -32768.0f, 32767.0f)));
			out.push_back(static_cast<int16_t>(std::clamp(right, -32768.0f, 32767.0f)));
		};

		if (m_roms.speed_up) {
			/* Rear left, rear right, the seat and the front */
			const float front = (ch[2] + ch[3]) * SPEAKER_LEVEL;

			put(m_sound, front, front);
			put(m_rear_sound, ch[0] * SPEAKER_LEVEL, ch[1] * SPEAKER_LEVEL);
		} else {
			const float mono = (ch[0] + ch[1] + ch[2] + ch[3]) * SOUND_LEVEL;

			put(m_sound, mono, mono);
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

void
Gaelco3d::serial_write(uint8_t byte)
{
	m_serial_ready = false;
	m_serial_ready_at = main_cycles() + m_serial_byte;
	m_link_open = true;
	if (m_link)
		link_send(static_cast<uint16_t>(0x100 | (m_rts ? 0x200 : 0) | byte));
}

uint8_t
Gaelco3d::serial_read()
{
	serve_link();
	m_serial_irq = false;
	set_ipl();
	if (!m_sending)
		m_serial_ready = true;
	return m_serial_in;
}

/* HEURISTIC: a received byte is seen on the first poll after it arrives */
void
Gaelco3d::serve_link()
{
	if (m_serial_ready_at && main_cycles() >= m_serial_ready_at) {
		m_serial_ready = true;
		m_serial_ready_at = 0;
	}
	if (!m_link)
		return;
	while (!m_rx.empty() && !m_serial_irq && !m_sending && main_cycles() >= m_rx_at) {
		const uint16_t word = m_rx.front();

		m_rx.pop_front();
		m_rts_of[word >> 12] = (word & 0x200) != 0;
		m_other_rts = std::all_of(m_rts_of.begin(), m_rts_of.end(), [](bool rts) { return rts; });
		if (word & 0x100) {
			m_serial_in = static_cast<uint8_t>(word);
			m_serial_ready = false;
			m_serial_irq = true;
			m_rx_at = main_cycles() + m_serial_byte;
			set_ipl();
		}
	}
}

/* The link is a bus that up to four Speed Up cabinets share. Each word
 * carries the number of the cabinet that sent it and travels round the ring
 * to every other cabinet; frame ticks only go to the next one.
 * HEURISTIC: words arrive a frame late, too late for the programs to keep
 * off the bus, so a cabinet's transmission while it holds RTS arrives whole,
 * and nothing arrives while a cabinet is sending */
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
Gaelco3d::run_frame()
{
	m_sound.clear();
	m_rear_sound.clear();
	pace_link();
	for (uint64_t s = 0; s < SLICES_PER_FRAME; s++) {
		const uint64_t slice = ++m_slices;
		const uint64_t main_to = m_base[0] + target(slice, m_main_clock);
		const uint64_t tms_to = m_base[1] + target(slice, m_tms_cycles);
		const uint64_t adsp_to = m_base[2] + target(slice, ADSP_CLOCK);

		serve_link();

		while (main_cycles() < main_to) {
			main_run(static_cast<uint32_t>(main_to - main_cycles()));
			run_adsp_to(m_base[2] + (main_cycles() - m_base[0]) * ADSP_CLOCK / m_main_clock);
		}
		if (m_tms->cycles() < tms_to)
			m_tms->run(static_cast<uint32_t>(tms_to - m_tms->cycles()));
		run_adsp_to(adsp_to);
	}

	if (m_gpu)
		end_scene();
	else
		make_picture();
	m_polydata_count = 0;
	m_vblank_irq = true;
	set_ipl();
	m_frame++;
	if (m_link) {
		link_send(static_cast<uint16_t>(0x400 | (m_frame & 0xFF)));
		if (m_paced)
			m_ahead++;
	}
}

void
Gaelco3d::make_picture()
{
	for (size_t y = 0; y < HEIGHT; y++) {
		const uint16_t *const from = &m_screen[(HEIGHT - 1 - y) * WIDTH];
		uint32_t *const to = &m_picture[y * WIDTH];

		for (size_t x = 0; x < WIDTH; x++) {
			const uint32_t p = from[x];
			const uint32_t r = (p >> 10) & 31, g = (p >> 5) & 31, b = p & 31;

			to[x] = 0xFF000000u | (b << 3 | b >> 2) << 16 | (g << 3 | g >> 2) << 8 | (r << 3 | r >> 2);
		}
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

namespace {
template <typename T>
bool
put_big(StateFile &state, const T &thing)
{
	const auto block = std::make_unique<T>(thing);

	return state.put(*block);
}
}

bool
Gaelco3d::save_state(const std::string &path)
{
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
	const uint64_t words[] = { m_slices, m_serial_ready_at, m_ab_period, m_ab_next, m_polydata_count,
		                   m_adsp_bank, m_ab_ireg, static_cast<uint32_t>(m_ab_base), m_ab_incs, m_ab_size };
	const uint8_t bytes[] = { m_mainlatch, m_outlatch, m_tms_reset, m_vblank_irq, m_sound_latch, m_sound_status,
		                  m_sound_pending, m_ab_on, m_analog_shift[0], m_analog_shift[1], m_analog_shift[2],
		                  m_analog_shift[3], m_serial_ready, m_serial_irq, m_sending, m_rts, m_other_rts,
		                  m_serial_in, m_kick_clock, m_kick_state };

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

	const bool ok = state.put(name) && state.put(m_frame) && state.put(words) && state.put(m_base) &&
	                state.put(bytes) && state.put(m_kick_ports) && state.put(m_kick_length) &&
	                (m_main ? state.put(*main) : state.put(*main16) && state.put(main_cycles())) &&
	                state.put(*tms) && state.put(*adsp) && put_big(state, m_eeprom.state()) &&
	                put_big(state, m_ram) && put_big(state, m_palette_ram) &&
	                state.put(m_screen.data(), static_cast<uint32_t>(m_screen.size() * 2)) &&
	                state.put(m_zbuffer.data(), static_cast<uint32_t>(m_zbuffer.size() * 2)) &&
	                state.put(m_polydata.data(), static_cast<uint32_t>(m_polydata.size() * 4));

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
	uint64_t words[10];
	std::array<uint64_t, 3> base{};
	uint8_t bytes[20];
	unsigned frame = 0;

	const bool ok = state.get(frame) && state.get(words) && state.get(base) && state.get(bytes) &&
	                state.get(m_kick_ports) && state.get(m_kick_length) &&
	                (m_main ? state.get(*main) : state.get(*main16) && state.get(main16_cycles)) && state.get(*tms) &&
	                state.get(*adsp) && state.get(*eeprom) && state.get(m_ram) && state.get(m_palette_ram) &&
	                state.get(m_screen.data(), static_cast<uint32_t>(m_screen.size() * 2)) &&
	                state.get(m_zbuffer.data(), static_cast<uint32_t>(m_zbuffer.size() * 2)) &&
	                state.get(m_polydata.data(), static_cast<uint32_t>(m_polydata.size() * 4));

	if (!ok) {
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
	m_eeprom.set_state(*eeprom);
	m_slices = words[0];
	m_base = base;
	m_serial_ready_at = words[1];
	m_ab_period = words[2];
	m_ab_next = words[3];
	m_polydata_count = static_cast<uint32_t>(words[4]);
	m_adsp_bank = static_cast<uint32_t>(words[5]);
	m_ab_ireg = static_cast<unsigned>(words[6]);
	m_ab_base = static_cast<int32_t>(static_cast<uint32_t>(words[7]));
	m_ab_incs = static_cast<uint32_t>(words[8]);
	m_ab_size = static_cast<uint32_t>(words[9]);
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
	m_kick_clock = bytes[18];
	m_kick_state = bytes[19];
	set_ipl();
	for (uint32_t i = 0; i < m_palette_ram.size(); i++) {
		palette_write(i * 2, static_cast<uint16_t>(m_palette_ram[i] >> 16));
		palette_write(i * 2 + 1, static_cast<uint16_t>(m_palette_ram[i]));
	}
	make_picture();
	return true;
}
}
