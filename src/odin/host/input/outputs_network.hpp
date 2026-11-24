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

#ifndef ODIN_OUTPUTS_NETWORK_HPP
#define ODIN_OUTPUTS_NETWORK_HPP

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "odin/host/app/games.hpp"

namespace odin {

/* The outputs of the cabinet for other programs (lamps, locks), as the
 * network output of MAME gives them (src/osd/modules/output/network.cpp):
 * a TCP server on port 8000 of this computer (127.0.0.1, where MAME takes
 * every address); a client gets "mame_start = <game>", then "<id> = <0 or
 * 1>" each time an output of the list changes, and "mame_stop = 1" at the
 * end, each line ended by a carriage return. A game with a drive board
 * (NASCAR) also sends its command as "m3ffb = <XX>", the byte in hex, as
 * Supermodel and Flycast do: each time it changes, and to a client that
 * comes in. What clients send is read and dropped. Odin turns it on with
 * "[outputs] network = on" in odin.ini. */
class OutputsNetwork {
public:
	static constexpr uint16_t PORT = 8000;
	/* A client whose lines pile up beyond this is closed */
	static constexpr size_t MOST_QUEUED = 16384;
	/* The reads of a client a frame, at most */
	static constexpr int MOST_READS = 16;

	/* nullptr, said on stderr, when the port cannot be opened.
	 * drive_first_output: the first output of the command of the drive
	 * board (Game::drive_first_output), 0 for none. */
	static std::unique_ptr<OutputsNetwork> open(const std::string &game, const std::vector<GameOutput> &outputs,
	                                            unsigned drive_first_output);
	~OutputsNetwork();
	OutputsNetwork(const OutputsNetwork &) = delete;
	OutputsNetwork &operator=(const OutputsNetwork &) = delete;

	/* Once a frame, the outputs of the board: clients come in, the changes
	 * go out. */
	void show(std::span<const uint8_t> outputs);
	/* The clients that come in, while no frame runs; how many there are */
	void welcome();
	size_t clients() const { return m_clients.size(); }

	/* The lines the outputs of the list send as they go from "was" to
	 * "now"; "was" follows. */
	static std::string changes(const std::vector<GameOutput> &list, std::vector<bool> &was,
	                           std::span<const uint8_t> now);
	/* The line of a command of the drive board */
	static std::string drive_line(uint8_t command);

private:
	struct Client {
		intptr_t socket;
		std::string queued;
	};

	OutputsNetwork(intptr_t listener, const std::string &game, const std::vector<GameOutput> &outputs,
	               unsigned drive_first_output);
	void accept_all();
	void flush();

	intptr_t m_listener;
	std::string m_game;
	std::vector<GameOutput> m_outputs;
	std::vector<bool> m_was;
	unsigned m_drive_first_output;
	/* The command of the drive board last sent, -1 before the first frame */
	int m_drive = -1;
	std::vector<Client> m_clients;
};

} /* namespace odin */

#endif /* ODIN_OUTPUTS_NETWORK_HPP */
