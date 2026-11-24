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

#include "odin/host/input/outputs_network.hpp"

#include <cstdio>

#include "odin/host/input/sockets.hpp"
#include "odin/machine/cabinet/drive_board.hpp"

namespace odin {

using namespace sockets;

std::unique_ptr<OutputsNetwork>
OutputsNetwork::open(const std::string &game, const std::vector<GameOutput> &outputs, unsigned drive_first_output)
{
	if (!network_up()) {
		std::fprintf(stderr, "ERROR: outputs: no network on this computer\n");
		return nullptr;
	}

	const intptr_t listener = listen_on(PORT, true);

	if (listener == NONE || !nonblocking(listener)) {
		std::fprintf(stderr, "ERROR: outputs: port %u of 127.0.0.1 cannot be opened\n", PORT);
		if (listener != NONE)
			close_socket(listener);
		return nullptr;
	}
	std::printf("outputs: for other programs on 127.0.0.1:%u\n", PORT);
	return std::unique_ptr<OutputsNetwork>(new OutputsNetwork(listener, game, outputs, drive_first_output));
}

OutputsNetwork::OutputsNetwork(intptr_t listener, const std::string &game, const std::vector<GameOutput> &outputs,
                               unsigned drive_first_output)
	: m_listener(listener), m_game(game), m_outputs(outputs), m_was(outputs.size(), false),
	  m_drive_first_output(drive_first_output)
{
}

OutputsNetwork::~OutputsNetwork()
{
	for (Client &client : m_clients)
		client.queued += "mame_stop = 1\r";
	flush();
	for (const Client &client : m_clients)
		close_socket(client.socket);
	close_socket(m_listener);
}

std::string
OutputsNetwork::changes(const std::vector<GameOutput> &list, std::vector<bool> &was, std::span<const uint8_t> now)
{
	std::string lines;

	was.resize(list.size(), false);
	for (size_t o = 0; o < list.size(); o++) {
		const bool on = list[o].on(now);

		if (on == was[o])
			continue;
		was[o] = on;
		lines += list[o].id + (on ? " = 1\r" : " = 0\r");
	}
	return lines;
}

std::string
OutputsNetwork::drive_line(uint8_t command)
{
	char line[16];

	std::snprintf(line, sizeof line, "m3ffb = %02X\r", command);
	return line;
}

void
OutputsNetwork::welcome()
{
	accept_all();
	flush();
}

void
OutputsNetwork::accept_all()
{
	for (;;) {
		const intptr_t client = accept_client(m_listener);

		if (client == NONE)
			break;
		if (!nonblocking(client)) {
			close_socket(client);
			continue;
		}
		m_clients.push_back({ client, "mame_start = " + m_game + "\r" +
		                              (m_drive < 0 ? "" : drive_line(static_cast<uint8_t>(m_drive))) });
	}
}

void
OutputsNetwork::show(std::span<const uint8_t> outputs)
{
	accept_all();

	std::string lines = changes(m_outputs, m_was, outputs);

	if (m_drive_first_output) {
		const uint8_t command = DriveBoard::command(outputs, m_drive_first_output);

		if (command != m_drive) {
			m_drive = command;
			lines += drive_line(command);
		}
	}
	for (Client &client : m_clients)
		client.queued += lines;
	flush();
}

/* What each client is owed goes as far as it takes it, and what it sent
 * is read; a client gone, failing or too slow is closed. */
void
OutputsNetwork::flush()
{
	char dropped[256];

	for (size_t c = 0; c < m_clients.size();) {
		Client &client = m_clients[c];
		bool gone = client.queued.size() > MOST_QUEUED;

		while (!gone && !client.queued.empty()) {
			const long sent = send_some(client.socket, client.queued.data(), client.queued.size());

			if (sent <= 0) {
				gone = !would_block();
				break;
			}
			client.queued.erase(0, static_cast<size_t>(sent));
		}
		for (int reads = 0; !gone && reads < MOST_READS; reads++) {
			const long got = receive_some(client.socket, dropped, sizeof dropped);

			if (got > 0)
				continue;
			gone = got == 0 || !would_block();
			break;
		}
		if (!gone) {
			c++;
			continue;
		}
		close_socket(client.socket);
		m_clients.erase(m_clients.begin() + static_cast<std::ptrdiff_t>(c));
	}
}

} /* namespace odin */
