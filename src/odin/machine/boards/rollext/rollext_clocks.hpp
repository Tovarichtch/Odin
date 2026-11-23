/*
 *  ██████╗ ██████╗ ██╗███╗   ██╗
 * ██╔═══██╗██╔══██╗██║████╗  ██║
 * ██║   ██║██║  ██║██║██╔██╗ ██║
 * ██║   ██║██║  ██║██║██║╚██╗██║
 * ╚██████╔╝██████╔╝██║██║ ╚████║
 *  ╚═════╝ ╚═════╝ ╚═╝╚═╝  ╚═══╝
 *
 * Copyright (C) 2026 Réda Chérif-Touil
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

#ifndef ODIN_ROLLEXT_CLOCKS_HPP
#define ODIN_ROLLEXT_CLOCKS_HPP

#include <cstdint>

/* The two oscillators of Rolling Extreme's board, as Gaelco's schematic
 * "3D-INTEGRAL (VERSION SERIE)" rev. 3 draws them */
namespace odin::rollext_clock {

/* The C82's: XTAL1 "50MHz" into CLKIN (ball B14), its PLL's loop filter
 * fitted (sheet INT_PROC: 10 ohms, 150 nF and 3.3 nF on LF, TI's figure 2):
 * the PLL on, CLKOUT as CLKIN (SPRS048 p. 128-129, lock range 40-80 MHz).
 * The chip is a GGP60, run below its grade */
inline constexpr uint64_t C82_HZ = 50'000'000;
/* The rest of the board's: XTAL2 "90MHz" into CKT of the 3D chip (sheet
 * INT_CUS), whose clocks run the FPGAs, the video, the sound and the link */
inline constexpr uint64_t BOARD_HZ = 90'000'000;
/* HEURISTIC: a frame, /EINT1 to /EINT1, is 437 lines of 3,600 periods of
 * BOARD_HZ, 57.2082 Hz. The rate is MEASURED: 57.208 +/- 0.005 Hz from the
 * beat of real screens against cameras, and the race's TIME on four
 * cabinets agrees. The split into lines is a guess: it gives the 25.000 kHz
 * line of the monitors' manuals. A frequency counter on a real board will
 * replace it */
inline constexpr uint64_t LINE_PERIODS = 3'600;
inline constexpr uint64_t FRAME_LINES = 437;
inline constexpr uint64_t FRAME_PERIODS = LINE_PERIODS * FRAME_LINES;

} /* namespace odin::rollext_clock */

#endif /* ODIN_ROLLEXT_CLOCKS_HPP */
