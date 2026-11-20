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

#ifndef ODIN_D3PLUS_HPP
#define ODIN_D3PLUS_HPP

#include <array>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "odin/machine/boards/rollext/d3plus_scene.hpp"
#include "odin/machine/boards/rollext/rollext_clocks.hpp"

namespace odin {
class StateFile;
class Worker;

/* Gaelco's 3D chip D3PLUS-4 on Rolling Extreme's board.
 *
 * Its input: a FIFO of 64-bit doublewords filled by PP0's peripheral device
 * transfers (SPRU261 § 8.3), 12 to a packet of 24
 * words, the word at the lower address in bits 63-32 (big-endian, SPRU261 §
 * 12.1).
 *
 * Its drawing, as worked out from the game's packets and videos of real
 * cabinets: each packet it takes is a polygon drawn at once into a
 * 512 x 384 frame with a depth buffer, its texels read from the texture
 * ROMs, the table of 0x90000000 and the palette. What nothing proved is
 * marked HEURISTIC; the arithmetic is in doubles.
 *
 * The chip's timing (its FIFO, bit 10, the markers) runs with the C82; the
 * packets it takes are drawn on a thread of their own, in the order taken.
 * Nothing drawn goes back to the program, so the machine runs the same.
 * At the end of a frame the pictures finished are drawn (run waits for
 * them); only the packets of the next picture may still be drawing. What
 * reads the drawing's buffers and what changes what the drawing reads (the
 * palette, the table) waits for all of it first (settle).
 *
 * Above the board's resolution, the packets go to the host GPU as scenes
 * instead, one a picture, and none is drawn here (set_gpu). */
class D3plus {
public:
	/* A packet: 96 bytes */
	static constexpr unsigned PACKET = 12;
	/* PP0 sends at most 16 packets at a time (0x40000A28) and looks at
	 * bit 10 only between transfers (0x400002A0) */
	static constexpr unsigned BURST = 16 * PACKET;
	/* HEURISTIC: the depth is not known; the FIFO holds one transfer of
	 * PP0 at its largest */
	static constexpr unsigned DEPTH = BURST;
	/* HEURISTIC: the chip spends 1536 periods of its 90 MHz clock on each
	 * packet, about 17 us: enough for bit 10 to hold PP0 back at times,
	 * little enough for each frame's packets to get through before PP0
	 * restarts. The drawing does not set it: how long the chip takes for
	 * a polygon of a given size is not known. In cycles of the C82, which
	 * counts the time here, rounded down */
	static constexpr uint64_t PACKET_CLOCKS = 1536;
	static constexpr uint64_t PACKET_CYCLES = PACKET_CLOCKS * rollext_clock::C82_HZ / rollext_clock::BOARD_HZ;
	/* Room kept past the depth, so a transfer into a full FIFO loses
	 * nothing (the program never makes one) */
	static constexpr unsigned CAPACITY = 1024;
	/* Markers the chip has not reached yet */
	static constexpr unsigned MARKERS = 16;
	/* The picture: 512 x 384, its point (0, 0) at the centre, x to the
	 * right, y up */
	static constexpr unsigned WIDTH = 512, HEIGHT = 384;

	/* For save states. Plain data. */
	struct State {
		std::array<uint64_t, CAPACITY> words;
		std::array<uint64_t, PACKET> current;
		uint64_t next_take, received, taken, markers, marker_at;
		/* Where each marker not reached yet stands in the stream (the
		 * doublewords before it), and the frame it came in */
		std::array<uint64_t, MARKERS> marker_place;
		std::array<uint32_t, MARKERS> marker_frame;
		uint32_t head, count;
		/* The clipping rectangle of the last setup packet: x from, x
		 * to, y from, y to */
		std::array<int32_t, 4> clip;
		uint32_t waiting_markers;
		/* The frame in which the picture drawn now began, and the
		 * number of its marker since power-up */
		uint32_t drawn_frame;
		uint64_t drawn_marker;
	};

	D3plus();
	~D3plus();
	D3plus(const D3plus &) = delete;
	D3plus &operator=(const D3plus &) = delete;

	/* What the chip reads besides its FIFO: the board's palette
	 * (0x80000000, 32,768 big-endian words) and table (0x90000000, 512
	 * KB), and the four texture ROMs ic47, ic60, ic45, ic58 */
	void attach(const uint8_t *palette, const uint8_t *table);
	/* The palette's bytes from `offset` (`bytes` of them) have changed,
	 * the drawing settled: the colours drawn follow */
	void palette_written(uint32_t offset, uint32_t bytes);
	/* The same for the table */
	void table_written() { m_table_changed = true; }
	void set_textures(const std::array<std::vector<uint8_t>, 4> &roms);

	void reset();
	/* A doubleword latched from the data bus at cycle `now`, at the
	 * address `addr` of the transfer */
	void receive(uint64_t word, uint32_t addr, uint64_t now);
	/* The write of 0xA0000000, PP0's at the start of each frame */
	void marker(uint64_t now);
	/* Bit 10 of 0xA0000002 at cycle `now`. "The chip can take a transfer"
	 * (DEDUCED): room for the largest, since PP0 does
	 * not look again until it ends */
	bool ready(uint64_t now)
	{
		update(now);

		const bool room = m_s.count + BURST <= DEPTH;

		m_polls++;
		m_refused += !room;
		return room;
	}
	/* ready(now) made only when it would answer `room` (true),
	 * nothing done otherwise (a waiting PP's poll, C82Bus::read_if):
	 * ready_at(now), then ready(now). With nothing due by `now`, update()
	 * does nothing and ready_at() answers the room as it is: looked at
	 * once */
	bool ready_if(uint64_t now, bool room)
	{
		if (!(m_s.count >= PACKET && m_s.next_take <= now) &&
		    !(m_s.waiting_markers && m_s.marker_place[0] <= m_s.taken * PACKET)) {
			const bool here = m_s.count + BURST <= DEPTH;

			if (here != room)
				return false;
			m_polls++;
			m_refused += !here;
			return true;
		}
		if (ready_at(now) != room)
			return false;
		ready(now);
		return true;
	}
	/* After a poll at `now` answered `room`: the cycle before which
	 * polls still answer `room`, nothing received, no marker, no reset
	 * or load meanwhile. The FIFO then only empties, a packet at a time,
	 * at next_take, next_take + PACKET_CYCLES... while one is whole: room
	 * once there stays; otherwise it comes with the packet that leaves
	 * enough of it (a poll at that cycle takes it), or never. 0 (no
	 * promise) unless `room` is the answer at `now` with nothing due */
	uint64_t ready_until(uint64_t now, bool room) const
	{
		uint64_t count = m_s.count, next = m_s.next_take;

		if ((count + BURST <= DEPTH) != room || (count >= PACKET && next <= now) ||
		    (m_s.waiting_markers && m_s.marker_place[0] <= m_s.taken * PACKET))
			return 0;
		if (room)
			return UINT64_MAX;
		while (count >= PACKET) {
			count -= PACKET;
			if (count + BURST <= DEPTH)
				return next;
			next += PACKET_CYCLES;
		}
		return UINT64_MAX;
	}
	/* What `count` polls answering `room` (ready_until promised
	 * it), the last at `now`, do: update() once, at the last (take_due
	 * depends on its time only as a bound: update(t1) ... update(tn) is
	 * update(tn)), each poll counted */
	void polled(uint64_t now, bool room, uint64_t count);
	/* What ready(now) would answer, nothing done (no packet taken, no
	 * poll counted): the packets update() would take by then */
	bool ready_at(uint64_t now) const
	{
		uint64_t count = m_s.count, next = m_s.next_take;

		while (count >= PACKET && next <= now) {
			count -= PACKET;
			next += PACKET_CYCLES;
		}
		return count + BURST <= DEPTH;
	}
	/* The chip does what it has to do by cycle `now` (the end of a
	 * frame, before the picture is shown), the pictures it finished
	 * drawn: the packets of the next one may still be drawing */
	void run(uint64_t now)
	{
		update(now);
		finished();
	}
	/* Waits until the packets taken so far are drawn */
	void settle();

	/* The picture last finished: RGBA bytes, WIDTH x HEIGHT, top line
	 * first */
	const std::vector<uint32_t> &picture() const { return m_shown; }
	/* Above the board's resolution: `on`, the
	 * packets taken go to the host GPU as scenes and none is drawn here;
	 * `native`, the flat ones (Q the same everywhere) are kept on the
	 * board's own points. The chip's timing does not depend on it. When
	 * this draws again, its frames are those of the last picture it drew */
	void set_gpu(bool on, bool native);
	/* With set_gpu on: the scene of the picture last finished (black
	 * before the first) */
	std::shared_ptr<const D3plusScene> scene();

	/* Doublewords in the FIFO; the n-th from its head */
	unsigned waiting() const { return m_s.count; }
	uint64_t at(unsigned n) const { return m_s.words[(m_s.head + n) % CAPACITY]; }
	/* The last packet the chip took, and how many it took */
	const std::array<uint64_t, PACKET> &current() const { return m_s.current; }
	uint64_t taken() const { return m_s.taken; }
	/* For the tests: bit 10 read, and read at 0, since power-up (not in
	 * states) */
	uint64_t polls() const { return m_polls; }
	uint64_t refused() const { return m_refused; }

	/* ODIN_D3PLUS_DUMP: the stream in a text file, a line for each frame
	 * ("F frame polls refused", the counts since power-up), marker ("M
	 * cycle doublewords-before-it") and packet ("P cycle address" and its
	 * 24 words in hex) */
	bool open_dump(const std::string &path);
	void frame_start(unsigned frame);
	/* ODIN_D3PLUS_FRAMES: each picture the chip finishes, as drawn, in
	 * <prefix>-f<frame its marker came in>-m<marker number>.ppm */
	void set_frames(const std::string &prefix)
	{
		settle();
		m_frames = prefix;
	}

	void save(State &out) const { out = m_s; }
	void load(const State &in)
	{
		m_s = in;
		m_scene = {};
	}
	/* The two frames and the depth buffer */
	bool save_buffers(StateFile &state);
	bool load_buffers(StateFile &state);

private:
	/* What the drawing thread is given: a packet with the clipping
	 * rectangle it is drawn in, or the end of a picture (for
	 * ODIN_D3PLUS_FRAMES: its frame and marker) */
	struct Job {
		std::array<uint64_t, PACKET> packet;
		std::array<int32_t, 4> clip;
		uint64_t marker;
		uint32_t frame;
		bool end;
	};
	/* Jobs are sent once the thread is idle, some at a time or the end
	 * of a picture at once, and the C82 waits for it once a batch is
	 * full */
	static constexpr size_t SOME = 128, BATCH = 1024;

	/* What take_due would do by `now`, when there is anything: a packet
	 * whole and due, or a marker reached */
	void update(uint64_t now)
	{
		if ((m_s.count >= PACKET && m_s.next_take <= now) ||
		    (m_s.waiting_markers && m_s.marker_place[0] <= m_s.taken * PACKET))
			take_due(now);
	}
	void take_due(uint64_t now);
	void finish_pictures();
	void next_picture();
	void take(const std::array<uint64_t, PACKET> &packet);
	void queue(const Job &job);
	/* After a job is put in m_queued */
	void queued();
	void send();
	/* Waits until the pictures finished so far are drawn */
	void finished();
	/* On the drawing thread */
	void end_picture(uint32_t frame, uint64_t marker);
	void draw(const std::array<uint64_t, PACKET> &packet, const std::array<int32_t, 4> &clip);
	/* Above the board's resolution: a packet put into the scene, the
	 * scene of a picture finished */
	void scene_packet(const std::array<uint64_t, PACKET> &packet);
	void end_scene();
	std::shared_ptr<const std::vector<uint16_t>> gpu_palette();
	std::shared_ptr<const std::vector<uint8_t>> gpu_table();

	State m_s{};
	std::FILE *m_dump = nullptr;
	std::array<uint64_t, PACKET> m_line{};
	uint32_t m_line_addr = 0;
	uint64_t m_polls = 0, m_refused = 0;
	unsigned m_frame = 0;
	std::string m_frames;

	const uint8_t *m_palette = nullptr, *m_table = nullptr;
	/* Each palette entry as a point is written: its 8-bit levels, red
	 * low, opaque (§ 3) */
	std::vector<uint32_t> m_colours;
	/* The texture ROMs as one sheet of 2048 x 16384 texels, a byte each,
	 * row after row */
	std::vector<uint8_t> m_sheet;
	/* The frame drawn into, its depth buffer, and the frame shown */
	std::vector<uint32_t> m_drawn, m_shown;
	std::vector<double> m_depth;
	std::vector<Job> m_queued, m_sent;
	/* The end of a picture is in m_queued; in m_sent, maybe not drawn */
	bool m_end_queued = false, m_end_sent = false;
	/* Above the board's resolution: the scene of the picture drawn now,
	 * the last finished, and what they read (made again once changed) */
	bool m_gpu = false, m_native = false, m_palette_changed = true, m_table_changed = true;
	D3plusScene m_scene;
	std::shared_ptr<const D3plusScene> m_last_scene;
	std::shared_ptr<const std::vector<uint8_t>> m_gpu_sheet, m_gpu_table;
	std::shared_ptr<const std::vector<uint16_t>> m_gpu_palette;
	/* The last member: its thread ends first */
	std::unique_ptr<Worker> m_drawing;
};

} /* namespace odin */

#endif /* ODIN_D3PLUS_HPP */
