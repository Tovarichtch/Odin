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

#include "odin/machine/boards/rollext/rollext.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <span>

#include "odin/common/repeats.hpp"
#include "odin/machine/core/state_file.hpp"

namespace odin {

namespace {

constexpr uint32_t SDRAM_SIZE = 0x1000000, FLASH_SIZE = 0x400000, ROM0_SIZE = 0x1000000;
constexpr uint32_t PALETTE_SIZE = 0x10000, TABLE_SIZE = 0x80000;

/* The program clocks 0x3C575 bytes, low bit first, into the FLEX 10K
 * chips (0x4003F050, 0x4003F748), then waits for bit 9 of 0xA0000002.
 * The two chips are chained in passive serial, the EPF10K100B first, then
 * the EPF10K50E (schematic sheet INT_PWR): about 1,985,000 bits (Altera's
 * EPC data sheet), the 1,977,256 sent. HEURISTIC: CONF_DONE rises once
 * that many DCLK rising edges came */
constexpr uint32_t FPGA_BITS = 0x3C575 * 8;

/* 0xA0000002: bit 8 EEPROM DI and FLEX DATA0, bit 9 EEPROM CS, bit 10
 * EEPROM SK and FLEX DCLK written; bit 8 EEPROM DO and bit 9 CONF_DONE read.
 * 0xA0000004: bit 4 the ADCs' I/O CLOCK, bit 5 their /CS
 * (the program at 0x4003E754, 0x4003F628, 0x4003F748; the
 * schematic, sheets INT_IO and AEREO IN/OUT) */
enum : uint16_t { DATA = 0x100, CS = 0x200, CLOCK = 0x400, ADC_CLOCK = 0x10, ADC_SELECT = 0x20 };

/* The sound's request: the program puts its ring of frames on XPT2's list
 * (0x40041050) and leaves XPT1 and XPT3 to the link (0x40042204,
 * 0x40042214) */
constexpr unsigned XPT2 = 2;

/* The outputs of 0xA0000004 (Rollext::outputs): JP3-4 COUNTER CPU, JP3-3
 * LAMP START, JP3-2 BLOCK through the ULN2064, the amplifiers' MUTING
 * (interface sheet, IC4) */
enum : uint16_t { COUNTER = 0x1, LAMP = 0x2, BLOCK = 0x4, SOUND_ON = 0x40 };

/* The settings at 0x400D2E30: 0xF4 bytes, their words 0-0x79 and
 * 0x7A-0xF3 of the 93C66, a byte of the struct at the same byte of its
 * copy (0x4001085C, 0x4003E98C); the sum of bytes 0-0xEF, big-endian at
 * 0xF0 (0x40010BE0, 0x40010C24) */
constexpr uint32_t SETTINGS_BYTES = 0xF4, SETTINGS_SUM = 0xF0;

/* The range the program learns for its handlebar, or the seat, ADC
 * channel 0: four records {low, reading, high,
 * value} of 32 bits from 0x400D3EDC, one a channel, 100, 0, 156, 0.5 from
 * the C start-up's table; main sets channel 0's to 114, 128, 142, 0.5
 * (0x40003134) before it reads the EEPROM, then each pass of its loop
 * (0x40003478, base 0x40003338) lowers the low end or raises the high one
 * by 1 while the reading is more than 5 beyond it. Channels 1-3 are never
 * learned. Read once these words of the program are there */
constexpr uint32_t RANGES = 0x400D3EDC;
constexpr int WINDOW_LOW = 0x72, WINDOW_HIGH = 0x8E;
struct ProgramWords {
	uint32_t at;
	std::array<uint32_t, 8> words;
	size_t count;
};
constexpr ProgramWords RANGES_CODE[] = {
	{ 0x40003134, { 0x1032F000, RANGES, 0x180B8072, 0x18990000, 0x180B8080, 0x18990004, 0x180B808E, 0x18990008 }, 8 },
	{ 0x40003338, { 0xB032F000, RANGES }, 2 },
	{ 0x40003478, { 0x1D910004, 0x15910000, 0x20EC8005, 0x213A0002, 0x41258004, 0x10ACFFFF, 0x15990000 }, 7 },
};

uint16_t
settings_sum(std::span<const uint8_t> copy)
{
	uint16_t sum = 0;

	for (uint32_t i = 0; i < SETTINGS_SUM; i++)
		sum = static_cast<uint16_t>(sum + copy[i]);
	return sum;
}

/* The PPs' waits (C82Pp::set_wait).
 * PP0 at 0x400002A0-0x40000358: Q of comm set (a transfer queued), or no
 * packet left to send ([0x88] = 0), or bit 10 of 0xA0000002 at 0 (the 3D
 * chip's FIFO full), then the packets made by the MP ([0x90]) against
 * those done ([0x94]): none new, back to 0x400002A0. PP1 at
 * 0x4004C080-0x4004C0C0: the cursor of XPT2's list ([0x1010]) against the
 * frame it waits for (d1), back while they match */
constexpr uint32_t PP0_WAIT_PC = 0x400002A0, PP1_WAIT_PC = 0x4004C080;
constexpr uint64_t PP0_WAIT[] = {
	0xf441ce80001041c0, 0x97811bcb40000320, 0x8800000000104100, 0x8800000000104100, 0x88000051df920022,
	0x9829000000104100, 0x97811bca40000320, 0x8800000000104100, 0x8800000000104100, 0x992818405ffffffe,
	0x8800000000104100, 0x88000051b1120000, 0xf441250000104100, 0x97801bcb40000a00, 0x8800000000104100,
	0x8800000000104100, 0x9780184000000094, 0x978018c000000090, 0x88000051d1120000, 0x8800000000134b40,
	0x9829110000104100, 0x97811bc6400002a0, 0x8800000000104100, 0x8800000000104100,
};
/* The MP's wait for the next frame, 0x4003F348-0x4003F360: the hook at
 * [0x400D3F2C] (0: none) and the frame count at [0x354], which the /EINT1
 * handler raises (0x4003EF78), back while it is 0 */
constexpr uint32_t MP_WAIT_IP = 0x4003F348;
constexpr uint32_t MP_WAIT[] = { 0x1d510000, 0x90e68004, 0x1032e01e, 0xf838a003, 0xf7ac8001, 0x15110000, 0x90a6fffa };
constexpr uint64_t PP1_WAIT[] = {
	0x9780184000001010, 0x8800000000104100, 0x8800005251120000, 0x982a138000104100, 0xbb024e8000104100,
	0xf82a310000104100, 0x97811bca4004c080, 0x8800000000104100, 0x8800000000104100,
};

} /* namespace */

Rollext::Rollext(Roms roms) : m_eeprom(static_cast<uint32_t>(CLOCK_HZ))
{
	/* rom0: each 32-bit word of the interleaved dump is little-endian (the
	 * reset vector at 0xFFFFFFF8); kept here in address order */
	m_rom0.assign(ROM0_SIZE, 0xFF);
	for (size_t i = 0; i + 4 <= std::min<size_t>(roms.rom0.size(), ROM0_SIZE); i += 4)
		for (size_t b = 0; b < 4; b++)
			m_rom0[i + b] = roms.rom0[i + 3 - b];
	m_flash = std::move(roms.flash);
	m_flash.resize(FLASH_SIZE, 0xFF);
	m_sdram.assign(SDRAM_SIZE, 0);
	m_palette.assign(PALETTE_SIZE, 0);
	m_table.assign(TABLE_SIZE, 0);
	m_d3plus.attach(m_palette.data(), m_table.data());
	m_d3plus.set_textures(roms.texture);
	m_chip = std::make_unique<Tms320c82>(*this);
	/* SDRAM 0x40000000 (16 MB DEDUCED: two 64-Mbit SDRAMs), ic38
	 * 0xC0000000, ic17+ic18 0xFF000000, as the game's program uses them.
	 * HEURISTIC: writes to the flash are left out (its programming by the
	 * rescue program is not emulated) */
	m_chip->map_external(0x40000000, SDRAM_SIZE, m_sdram.data(), true);
	m_chip->map_external(0xC0000000, FLASH_SIZE, m_flash.data(), false);
	m_chip->map_external(0xFF000000, ROM0_SIZE, m_rom0.data(), false);
	m_chip->pp(0).set_wait(PP0_WAIT_PC, PP0_WAIT);
	m_chip->pp(1).set_wait(PP1_WAIT_PC, PP1_WAIT);
	m_chip->mp().set_wait(MP_WAIT_IP, MP_WAIT);
	reset();
}

Rollext::~Rollext()
{
	if (m_sound_dump)
		std::fclose(m_sound_dump);
}

void
Rollext::reset()
{
	m_chip->reset();
	m_eeprom.reset();
	m_d3plus.reset();
	/* Output ports at 0, the FLEX chips not configured, the ADCs not
	 * selected, the DACs' latch at 0, no XPT. HEURISTIC: the ADCs' last
	 * conversion 0 */
	m_s = State{};
	m_chip->set_xpt(0);
	m_sound.clear();
	m_option_sound.clear();
}

void
Rollext::open_sound_dump(const std::string &path)
{
	m_sound_dump = std::fopen(path.c_str(), "w");
	if (!m_sound_dump)
		std::fprintf(stderr, "ERROR: ODIN_SOUND_DUMP: cannot write '%s'\n", path.c_str());
}

/* One frame: /EINT1 rises (HEURISTIC: once a frame, at its start, 57.208 Hz),
 * then the C82 runs to the next frame's cycle, stopping at each sample
 * period */
void
Rollext::run_frame()
{
	const uint64_t end = frame_end(m_s.frame + 1);

	m_sound.clear();
	m_option_sound.clear();
	m_d3plus.frame_start(m_s.frame);
	if (m_sound_dump)
		std::fprintf(m_sound_dump, "F %u %llu %08X %08X %08X\n", m_s.frame,
		             static_cast<unsigned long long>(m_chip->cycles()), m_chip->internal32(0x1010),
		             m_chip->internal32(0x1AC4), m_chip->pp(1).pc());
	m_chip->set_eint(1, false);
	m_chip->set_eint(1, true);
	/* The host's rounding read once: nothing here changes it */
	m_chip->check_host_rounding();
	for (uint64_t at = period_cycle(m_s.sound_periods + 1); at <= end; at = period_cycle(m_s.sound_periods + 1)) {
		m_chip->run_rounded(at);
		sound_period();
	}
	m_chip->run_rounded(end);
	m_d3plus.run(m_chip->cycles());
	m_s.frame++;
}

/* A sample period: the DACs take the frame in their latch, then the board
 * asks the next one with XPT2, held until the TC takes it (SPRU261 § 7.2).
 * The EPF10K100 does it (sheet INT_CUS: S-CH1, S-CH2, WS to the two
 * TDA1543; SO-REQ to the EPM7160, which drives /XPT). HEURISTIC: one
 * period between the frame's arrival and its conversion (in the FPGA) */
void
Rollext::sound_period()
{
	const auto word = [this](unsigned w) {
		return static_cast<int16_t>(m_s.dac[2 * w] << 8 | m_s.dac[2 * w + 1]);
	};

	m_s.sound_periods++;
	m_sound.push_back(word(0));
	m_sound.push_back(word(1));
	m_option_sound.push_back(word(2));
	m_option_sound.push_back(word(3));
	if (m_s.xpt2)
		return;
	m_s.xpt2 = 1;
	if (m_sound_dump)
		std::fprintf(m_sound_dump, "X %llu %08X\n", static_cast<unsigned long long>(m_chip->cycles()),
		             m_chip->internal32(0x1010));
	m_chip->set_xpt(XPT2);
}

/* The TC put XPT2's code on AD[39:36]: the board lets go of the pins
 * (SPRU261 § 7.4.3) */
void
Rollext::xpt_acknowledged(unsigned number)
{
	if (number != XPT2 || !m_s.xpt2)
		return;
	m_s.xpt2 = 0;
	if (m_sound_dump)
		std::fprintf(m_sound_dump, "A %llu\n", static_cast<unsigned long long>(m_chip->cycles()));
	m_chip->set_xpt(0);
}

void
Rollext::unmapped(const char *what, uint32_t addr)
{
	repeats::print(stderr, "rollext: %s at 0x%08X: nothing there yet\n", what, addr);
}

/* For a waiting PP: the palette and the table as read; the ports as read,
 * bit 10 as the 3D chip would answer at `now` (D3plus::ready_at) */
bool
Rollext::peek(uint32_t addr, unsigned size, uint64_t now, uint64_t &value)
{
	if ((addr & 0xFFFFFFF8u) == 0xA0000000u) {
		value = size == 1 ? port_read((addr >> 1) & 3, &now) : ports(addr, size, &now);
		return true;
	}
	if ((addr & 0xFFFF0000u) == 0x80000000u || (addr & 0xFFF80000u) == 0x90000000u) {
		value = read(addr, size);
		return true;
	}
	return false;
}

/* For a waiting PP: PP0's poll of a port in 16 bits in one pass, the
 * port as peek() answers it, then read as read16() reads it. Port 1 (bit
 * 10, PP0's poll): its other bits the same both times (do_read() is const,
 * nothing changes them between), bit 10 by the 3D chip's queue looked at
 * then polled (D3plus::ready_if) at the same bus time (`now` is the chip's
 * cycles(), which read16() gives ready()) */
bool
Rollext::read_if(uint32_t addr, unsigned size, uint64_t now, uint64_t value)
{
	if (size == 1 && (addr & 0xFFFFFFF8u) == 0xA0000000u) {
		const unsigned port = (addr >> 1) & 3;

		if (port == 1) {
			const uint16_t eeprom = m_eeprom.do_read() ? DATA : 0;
			const uint16_t done = m_s.fpga_done ? CS : 0;
			const uint64_t rest = 0xF800u | done | eeprom | (~m_closed[1] & 0xFFu);

			return (value & ~uint64_t{ CLOCK }) == rest && m_d3plus.ready_if(now, (value & CLOCK) != 0);
		}
		if (port_read(port, &now) != value)
			return false;
		port_read(port);
		return true;
	}
	return C82Bus::read_if(addr, size, now, value);
}

/* Port 1 in 16 bits, PP0's poll, the board reached by nothing else
 * and nothing set from outside the C82's run (C82Bus::answers_until): its
 * bits but bit 10 do not change (the EEPROM's DO and CONF_DONE change only
 * by a write of port 1, the switches between runs); bit 10 as the 3D chip
 * promises (D3plus::ready_until). No promise for the rest */
uint64_t
Rollext::answers_until(uint32_t addr, unsigned size, uint64_t now, uint64_t value)
{
	if (size != 1 || (addr & 0xFFFFFFF8u) != 0xA0000000u || ((addr >> 1) & 3) != 1)
		return 0;

	const uint16_t eeprom = m_eeprom.do_read() ? DATA : 0;
	const uint16_t done = m_s.fpga_done ? CS : 0;
	const uint64_t rest = 0xF800u | done | eeprom | (~m_closed[1] & 0xFFu);

	return (value & ~uint64_t{ CLOCK }) == rest ? m_d3plus.ready_until(now, (value & CLOCK) != 0) : 0;
}

/* `count` reads of port 1 promised by answers_until(), the last at
 * `now`: what port_read(1) does for each, but at their bus times (`now`,
 * not the chip's cycles): the 3D chip's polls (D3plus::polled) */
void
Rollext::reads_made(uint32_t addr, unsigned size, uint64_t now, uint64_t value, uint64_t count)
{
	if (size == 1 && (addr & 0xFFFFFFF8u) == 0xA0000000u && ((addr >> 1) & 3) == 1) {
		m_d3plus.polled(now, (value & CLOCK) != 0, count);
		return;
	}
	C82Bus::reads_made(addr, size, now, value, count);
}

/* What is not plain memory (the SDRAM and the ROMs are, in Tms320c82). The
 * 16-bit ports read and written in other sizes: HEURISTIC, their bytes in
 * big-endian order */
uint64_t
Rollext::read(uint32_t addr, unsigned size)
{
	/* The ports first: PP0 reads bit 10 most */
	if ((addr & 0xFFFFFFF8u) == 0xA0000000u)
		return ports(addr, size, nullptr);
	if ((addr & 0xFFFF0000u) == 0x80000000u)
		return c82_get(m_palette.data() + (addr & (PALETTE_SIZE - 1)), size);
	if ((addr & 0xFFF80000u) == 0x90000000u)
		return c82_get(m_table.data() + (addr & (TABLE_SIZE - 1)), size);
	if ((addr & 0xFFFFFFF0u) == 0x80010000u) {
		/* A register of the video side, written 0 by the MP (0x400427AC)
		 * and read by the link's XPT3 list (0x40042154): its meaning is
		 * unknown. HEURISTIC: reads 0 */
		unmapped("a read of the video register", addr);
		return 0;
	}
	if ((addr & 0xF0000000u) == 0xB0000000u) {
		/* The link between two cabinets, not emulated yet. HEURISTIC:
		 * no word ready (bit 0 of 0xB0000000 at 0) */
		unmapped("a read of the link (not emulated yet)", addr);
		return 0;
	}
	/* HEURISTIC: nothing answers with 0 */
	unmapped("a read", addr);
	return 0;
}

void
Rollext::write(uint32_t addr, unsigned size, uint64_t value)
{
	/* The DACs first: XPT2 writes their frame a byte at a time */
	if ((addr & 0xFFFFFFF8u) == 0x60000000u) {
		/* The four DACs' frame, fed by XPT2: 4 x 16 bits, left, right,
		 * woofer and rear (the OUTPUT TEST,
		 * 0x4000E8F8), taken by two TDA1543 as two's complement words
		 * (Philips data sheet p. 6) */
		const unsigned at = addr & 7, bytes = std::min(1u << size, 8 - at);

		for (unsigned b = 0; b < bytes; b++)
			m_s.dac[at + b] = static_cast<uint8_t>(value >> (8 * ((1u << size) - 1 - b)));
		if (m_sound_dump && at + bytes == 8)
			std::fprintf(m_sound_dump, "D %llu %02X%02X %02X%02X %02X%02X %02X%02X\n",
			             static_cast<unsigned long long>(m_chip->cycles()), m_s.dac[0], m_s.dac[1], m_s.dac[2],
			             m_s.dac[3], m_s.dac[4], m_s.dac[5], m_s.dac[6], m_s.dac[7]);
		return;
	}
	if ((addr & 0xFFFF0000u) == 0x80000000u) {
		/* The palette, filled from rom0 0xFFF40000 (0x400402D4), read
		 * by the 3D chip: once it has drawn what it took */
		m_d3plus.settle();
		c82_put(m_palette.data() + (addr & (PALETTE_SIZE - 1)), size, value);
		m_d3plus.palette_written(addr & (PALETTE_SIZE - 1), 1u << size);
		return;
	}
	if ((addr & 0xFFF80000u) == 0x90000000u) {
		/* The 512 KB unpacked from rom0 0xFFF80000 (0x4004040C), read
		 * by the 3D chip: once it has drawn what it took */
		m_d3plus.settle();
		c82_put(m_table.data() + (addr & (TABLE_SIZE - 1)), size, value);
		m_d3plus.table_written();
		return;
	}
	if ((addr & 0xFFFFFFF8u) == 0xA0000000u) {
		const unsigned port = (addr >> 1) & 3;

		switch (size) {
		case 0: {
			const unsigned shift = addr & 1 ? 0 : 8;

			port_write(port, static_cast<uint16_t>((m_s.out[port] & ~(0xFF << shift)) | ((value & 0xFF) << shift)));
			break;
		}
		case 1:
			port_write(port, static_cast<uint16_t>(value));
			break;
		case 2:
			port_write(port, static_cast<uint16_t>(value >> 16));
			port_write((port + 1) & 3, static_cast<uint16_t>(value));
			break;
		default:
			for (unsigned p = 0; p < 4; p++)
				port_write(p, static_cast<uint16_t>(value >> (48 - 16 * p)));
			break;
		}
		return;
	}
	if ((addr & 0xFFFFFFF0u) == 0x80010000u) {
		m_s.video = static_cast<uint32_t>(value);
		return;
	}
	if ((addr & 0xF0000000u) == 0xB0000000u) {
		unmapped("a write to the link (not emulated yet)", addr);
		return;
	}
	unmapped("a write", addr);
}

/* A read of the ports in any size; `peek`: bit 10 as at that cycle,
 * nothing done */
uint64_t
Rollext::ports(uint32_t addr, unsigned size, const uint64_t *peek)
{
	const unsigned port = (addr >> 1) & 3;

	switch (size) {
	case 0:
		return (port_read(port, peek) >> (addr & 1 ? 0 : 8)) & 0xFF;
	case 1:
		return port_read(port, peek);
	case 2:
		return static_cast<uint32_t>(port_read(port, peek) << 16) | port_read((port + 1) & 3, peek);
	default:
		return (static_cast<uint64_t>(port_read(0, peek)) << 48) | (static_cast<uint64_t>(port_read(1, peek)) << 32) |
		       (static_cast<uint64_t>(port_read(2, peek)) << 16) | port_read(3, peek);
	}
}

/* The inputs are active low (the word of inputs at
 * 0x400D3ED0 inverts them). HEURISTIC: the bits no code was seen to read
 * are high, as pulled up */
uint16_t
Rollext::port_read(unsigned port, const uint64_t *peek)
{
	switch (port) {
	case 0: {
		/* bits 0-3: inputs (bit 1 service, bit 3 serve the program over
		 * the link); bits 4-5: DATA OUT of the two ADCs, MSB first (IC6,
		 * IC5); bits 6-7: wired to nothing. HEURISTIC: high while their
		 * /CS is high (DATA OUT then floats), and bits 6-7 high */
		uint16_t adc = 0xC0;

		for (unsigned k = 0; k < ADCS; k++)
			if (!m_s.adc_select || (m_s.adc_shift[k] & 0x80))
				adc = static_cast<uint16_t>(adc | (0x10u << k));
		return static_cast<uint16_t>(0xFF00 | adc | (~m_closed[0] & 0x0F));
	}
	case 1: {
		/* bits 0-7: buttons, bit 8 EEPROM DO, bit 9 CONF_DONE. Bit 10 is
		 * read by PP0 alone, as the 3D chip's go-ahead for a transfer of
		 * polygons: its FIFO has room */
		const uint16_t eeprom = m_eeprom.do_read() ? DATA : 0;
		const uint16_t done = m_s.fpga_done ? CS : 0;
		const bool ready = peek ? m_d3plus.ready_at(*peek) : m_d3plus.ready(m_chip->cycles());
		const uint16_t room = ready ? CLOCK : 0;

		return static_cast<uint16_t>(0xF800 | room | done | eeprom | (~m_closed[1] & 0xFF));
	}
	case 2:
		return static_cast<uint16_t>(0xFF00 | (~m_closed[2] & 0xFF));
	default:
		/* bit 0: the coin chute */
		return static_cast<uint16_t>(0xFFC0 | (~m_closed[3] & 0x3F));
	}
}

void
Rollext::port_write(unsigned port, uint16_t value)
{
	m_s.out[port] = value;
	if (port == 0) {
		/* Output port 0: written 0 once by the MP at its start
		 * (0x4003F018), then by PP0 at the start of each of its tasks,
		 * once its transfers are done (0x40000230): a marker in the 3D
		 * chip's stream (its meaning is not known) */
		m_d3plus.marker(m_chip->cycles());
	} else if (port == 1) {
		const bool clock = value & CLOCK;

		/* The FLEX chips take DATA0 on each DCLK rising edge until
		 * configured (passive serial, FLEX 10K data sheet p. 126) */
		if (clock && !m_s.fpga_clock && !m_s.fpga_done && ++m_s.fpga_clocks >= FPGA_BITS) {
			m_s.fpga_done = 1;
			if (m_verbose)
				std::printf("rollext: the FLEX chips are configured (%u clocks)\n", m_s.fpga_clocks);
		}
		m_s.fpga_clock = clock;
		/* Bit 13, VIDOUT, goes to the EPF10K50 (sheet INT_IO, IC36 Q7),
		 * set once the first pictures are drawn (0x4003F270): what that
		 * chip does with it is in the FPGA. Bits 14-15 light the board's
		 * LEDs D2 and D1 */

		const uint64_t now = m_chip->cycles();

		m_eeprom.tick(static_cast<uint32_t>(std::min<uint64_t>(now - m_s.eeprom_tick, 0xFFFFFFFFu)));
		m_s.eeprom_tick = now;
		/* The 93C66 is organized by 16 bits (the program sends 8
		 * address bits, 16 data bits) */
		m_eeprom.set_pins(value & CS, clock, value & DATA, true);
	} else if (port == 2) {
		/* The two TLC549 share their I/O CLOCK and /CS (interface sheet,
		 * IC5 and IC6; TI data sheet: the MSB of the last conversion out
		 * when /CS falls, the other bits on the next seven falling clocks,
		 * a conversion started by the 8th). HEURISTIC: the input taken at
		 * the 8th falling clock */
		const bool select = !(value & ADC_SELECT), clock = value & ADC_CLOCK;

		if (select && !m_s.adc_select) {
			m_s.adc_shift = m_s.adc_held;
			m_s.adc_clocks = 0;
		}
		if (select && m_s.adc_clock && !clock) {
			for (uint8_t &shift : m_s.adc_shift)
				shift = static_cast<uint8_t>(shift << 1);
			if (++m_s.adc_clocks == 8)
				m_s.adc_held = m_analog;
		}
		m_s.adc_select = select;
		m_s.adc_clock = clock;
	}
}

uint8_t
Rollext::read8(uint32_t addr)
{
	return static_cast<uint8_t>(read(addr, 0));
}

/* A port as ports() reads it in 16 bits, without its switch: PP0 reads
 * bit 10 this way */
uint16_t
Rollext::read16(uint32_t addr)
{
	if ((addr & 0xFFFFFFF8u) == 0xA0000000u)
		return port_read((addr >> 1) & 3);
	return static_cast<uint16_t>(read(addr, 1));
}

uint32_t
Rollext::read32(uint32_t addr)
{
	return static_cast<uint32_t>(read(addr, 2));
}

uint64_t
Rollext::read64(uint32_t addr)
{
	return read(addr, 3);
}

void
Rollext::write8(uint32_t addr, uint8_t value)
{
	write(addr, 0, value);
}

void
Rollext::write16(uint32_t addr, uint16_t value)
{
	write(addr, 1, value);
}

void
Rollext::write32(uint32_t addr, uint32_t value)
{
	write(addr, 2, value);
}

void
Rollext::write64(uint32_t addr, uint64_t value)
{
	write(addr, 3, value);
}

/* PP0's PT2 reads [0x8c] + 0x10000000 as a peripheral device (0x40000A48),
 * [0x8c] following the packets PT1 put in the SDRAM at 0x4005xxxx
 * (0x40000958): DEDUCED, the SDRAM answers 0x5xxxxxxx and the 3D chip
 * latches the data (SPRU261 § 8.3.4, table 10-2: AD[35:32] = 0010). Where
 * the board decodes it (the MAX 7000?) is not known. HEURISTIC: 64 bits at
 * a time, the SDRAM's width (two 32-bit K4S643232), no wait */
void
Rollext::pdpt_read(uint32_t addr)
{
	if ((addr & 0xFF000000u) == 0x50000000u) {
		m_d3plus.receive(c82_get(m_sdram.data() + (addr & (SDRAM_SIZE - 1)), 3), addr, m_chip->cycles());
		return;
	}
	unmapped("a peripheral device read", addr);
}

/* No device of the board was seen to write memory this way */
void
Rollext::pdpt_write(uint32_t addr, uint8_t)
{
	unmapped("a peripheral device write", addr);
}

bool
Rollext::save_state(const std::string &path)
{
	StateFile state(path, StateFile::Mode::SAVE);

	if (!state.ok()) {
		std::fprintf(stderr, "ERROR: save state: cannot write '%s'\n", path.c_str());
		return false;
	}

	char name[32] = "";
	auto chip = std::make_unique<C82ChipState>();
	C82MpState mp;
	C82TcState tc;
	MicrowireEepromState<512> eeprom;
	auto d3plus = std::make_unique<D3plus::State>();

	std::memset(chip.get(), 0, sizeof(*chip));
	std::memset(&mp, 0, sizeof(mp));
	std::memset(&tc, 0, sizeof(tc));
	std::memset(&eeprom, 0, sizeof(eeprom));
	std::strncpy(name, m_name.c_str(), sizeof(name) - 1);
	m_chip->save(*chip);
	m_chip->mp().save(mp);
	m_chip->tc().save(tc);
	m_eeprom.save(eeprom);
	m_d3plus.save(*d3plus);

	const bool ok = state.put(name) && state.put(m_s) && state.put(mp) && state.put(tc) && state.put(*chip) &&
	                state.put(eeprom) && state.put(m_sdram.data(), SDRAM_SIZE) &&
	                state.put(m_palette.data(), PALETTE_SIZE) && state.put(m_table.data(), TABLE_SIZE) &&
	                state.put(*d3plus) && m_d3plus.save_buffers(state);

	if (!ok)
		std::fprintf(stderr, "ERROR: save state: cannot write '%s'\n", path.c_str());
	return ok;
}

bool
Rollext::load_state(const std::string &path)
{
	StateFile state(path, StateFile::Mode::LOAD);

	if (!state.ok()) {
		std::fprintf(stderr, "ERROR: load state: cannot read '%s'\n", path.c_str());
		return false;
	}

	char name[32] = "";
	auto chip = std::make_unique<C82ChipState>();
	C82MpState mp;
	C82TcState tc;
	MicrowireEepromState<512> eeprom;
	auto d3plus = std::make_unique<D3plus::State>();
	State s;

	/* The palette and the table change under the 3D chip's drawing */
	m_d3plus.settle();
	if (!state.get(name) || m_name.compare(0, sizeof(name) - 1, name) != 0 || !state.get(s) || !state.get(mp)) {
		std::fprintf(stderr, "ERROR: load state: '%s' is not a state of %s\n", path.c_str(), m_name.c_str());
		return false;
	}
	/* A state is saved between two frames: the C82 stands at the frame's
	 * cycle, a slice past it at most. Another clock's or frame rate's
	 * stands elsewhere */
	if (mp.cycles < frame_end(s.frame) || mp.cycles - frame_end(s.frame) > frame_end(1)) {
		std::fprintf(stderr,
		             "ERROR: load state: '%s' was saved with another C82 clock or frame rate than %llu Hz and "
		             "%llu C82 cycles a frame\n",
		             path.c_str(), static_cast<unsigned long long>(CLOCK_HZ),
		             static_cast<unsigned long long>(frame_end(1)));
		return false;
	}
	if (!state.get(tc) || !state.get(*chip) || !state.get(eeprom) || !state.get(m_sdram.data(), SDRAM_SIZE) ||
	    !state.get(m_palette.data(), PALETTE_SIZE) || !state.get(m_table.data(), TABLE_SIZE) ||
	    !state.get(*d3plus) || !m_d3plus.load_buffers(state)) {
		m_d3plus.palette_written(0, PALETTE_SIZE);
		m_d3plus.table_written();
		std::fprintf(stderr, "ERROR: load state: '%s' is not a state of %s\n", path.c_str(), m_name.c_str());
		return false;
	}
	m_s = s;
	m_chip->load(*chip);
	m_chip->mp().load(mp);
	m_chip->tc().load(tc);
	m_eeprom.load(eeprom);
	m_d3plus.load(*d3plus);
	m_d3plus.palette_written(0, PALETTE_SIZE);
	m_d3plus.table_written();
	return true;
}

uint8_t
Rollext::outputs() const
{
	return static_cast<uint8_t>((m_s.out[2] & COUNTER ? 1 : 0) | (m_s.out[2] & LAMP ? 2 : 0) |
	                            (m_s.out[2] & BLOCK ? 4 : 0) | (m_s.out[2] & SOUND_ON ? 8 : 0));
}

double
Rollext::motor_jolt() const
{
	const auto value = [this](uint32_t addr) {
		const uint8_t *const at = &m_sdram[addr - 0x40000000u];
		const uint32_t word = static_cast<uint32_t>(at[0] << 24 | at[1] << 16 | at[2] << 8 | at[3]);
		float f;

		std::memcpy(&f, &word, sizeof(f));
		return static_cast<double>(f);
	};
	const double jolt = (value(0x400D390C) + 10.0 * value(0x400D3878)) * value(0x400D38A4) * 63.5;

	return std::isfinite(jolt) ? jolt : 0.0;
}

/* Channel 0's record when it holds at least the game's window and the
 * ADCs were read once (the start-up leaves their readings at 0): never a
 * memory test's patterns */
std::optional<LearnedRanges>
Rollext::learned_ranges() const
{
	const auto word = [this](uint32_t addr) {
		return static_cast<uint32_t>(c82_get(&m_sdram[addr - 0x40000000u], 2));
	};

	for (const ProgramWords &code : RANGES_CODE)
		for (size_t i = 0; i < code.count; i++)
			if (word(code.at + 4 * static_cast<uint32_t>(i)) != code.words[i])
				return std::nullopt;

	const int low = static_cast<int32_t>(word(RANGES)), reading = static_cast<int32_t>(word(RANGES + 4));
	const int high = static_cast<int32_t>(word(RANGES + 8));
	bool read = false;

	for (uint32_t channel = 0; channel < 4; channel++)
		read = read || word(RANGES + 16 * channel + 4) != 0;
	if (!read || low < 0 || low > WINDOW_LOW || high < WINDOW_HIGH || high > 255 || reading < 0 || reading > 255)
		return std::nullopt;

	LearnedRanges ranges;

	ranges[0] = { low, high };
	return ranges;
}

/* Only the two ends, and only when they change: the program makes the
 * value from them at its next pass. Never ends an 8-bit ADC cannot read */
void
Rollext::widen_ranges(const LearnedRanges &kept)
{
	const std::optional<LearnedRanges> now = learned_ranges();

	if (!now)
		return;

	const LearnedRange wide = widened(*now, kept)[0];

	if ((wide.low == (*now)[0].low && wide.high == (*now)[0].high) || wide.low < 0 || wide.high > 255)
		return;
	c82_put(&m_sdram[RANGES - 0x40000000u], 2, static_cast<uint32_t>(wide.low));
	c82_put(&m_sdram[RANGES + 8 - 0x40000000u], 2, static_cast<uint32_t>(wide.high));
}

bool
Rollext::load_kept(const std::string &path)
{
	std::ifstream in(path, std::ios::binary);

	if (!in)
		return true;

	std::array<uint8_t, decltype(m_eeprom)::SIZE> bytes{};

	in.read(reinterpret_cast<char *>(bytes.data()), bytes.size());
	if (in.gcount() != static_cast<std::streamsize>(bytes.size())) {
		std::fprintf(stderr, "ERROR: '%s' is not an EEPROM of %s\n", path.c_str(), m_name.c_str());
		return false;
	}
	m_eeprom.load(bytes);
	return true;
}

bool
Rollext::save_kept(const std::string &path) const
{
	MicrowireEepromState<decltype(m_eeprom)::SIZE> chip;
	const std::string fresh = path + ".new";
	std::error_code error;

	m_eeprom.save(chip);
	std::filesystem::create_directories(std::filesystem::path(path).parent_path(), error);
	{
		std::ofstream out(fresh, std::ios::binary);

		out.write(reinterpret_cast<const char *>(chip.data.data()), chip.data.size());
		if (!out) {
			std::fprintf(stderr, "ERROR: rollext: cannot write '%s'\n", fresh.c_str());
			return false;
		}
	}
	std::filesystem::rename(fresh, path, error);
	if (error)
		std::fprintf(stderr, "ERROR: rollext: cannot write '%s': %s\n", path.c_str(), error.message().c_str());
	return !error;
}

bool
Rollext::set_eeprom_setting(uint32_t byte, uint8_t value, std::optional<uint8_t> only_from)
{
	const std::span<uint8_t> chip = m_eeprom.data();
	bool kept = false;

	if (byte >= SETTINGS_SUM)
		return false;
	for (uint32_t at = 0; at + SETTINGS_BYTES <= chip.size(); at += SETTINGS_BYTES) {
		const std::span<uint8_t> copy = chip.subspan(at, SETTINGS_BYTES);

		if (settings_sum(copy) != (copy[SETTINGS_SUM] << 8 | copy[SETTINGS_SUM + 1]))
			continue;
		kept = true;
		if (only_from && copy[byte] != *only_from)
			continue;
		copy[byte] = value;

		const uint16_t sum = settings_sum(copy);

		copy[SETTINGS_SUM] = static_cast<uint8_t>(sum >> 8);
		copy[SETTINGS_SUM + 1] = static_cast<uint8_t>(sum);
	}
	return kept;
}

void
Rollext::dump_memories() const
{
	const struct {
		const char *name;
		const uint8_t *bytes;
		size_t size;
	} memories[] = {
		{ "sdram", m_sdram.data(), m_sdram.size() },
		{ "c82ram", m_chip->block(Tms320c82::PP0_DATA0).data(), Tms320c82::BLOCKS * 4096u },
		{ "palette", m_palette.data(), m_palette.size() },
		{ "table", m_table.data(), m_table.size() },
	};

	for (const auto &memory : memories) {
		const std::string path = m_name + "-" + memory.name + ".bin";

		if (std::FILE *const file = std::fopen(path.c_str(), "wb")) {
			std::fwrite(memory.bytes, 1, memory.size, file);
			std::fclose(file);
		}
	}
}

} /* namespace odin */
