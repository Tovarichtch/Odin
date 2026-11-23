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

#ifndef ODIN_DAC_STAGE_HPP
#define ODIN_DAC_STAGE_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>

namespace odin {

/* The analog stage after a current-output DAC, as Rolling Extreme's board
 * has one for each of its four channels (Gaelco's schematic, sheet INT_IO,
 * TDA1543 to TL074): an op-amp turns the current into a voltage through
 * r_iv in parallel with c_iv (a first-order low-pass), then r1 with c1 to
 * ground, loaded by a unity-gain Sallen-Key: r2 from there to its middle,
 * c2 from its middle back to its output, r3 on to its input, c3 to ground.
 * A fourth-order low-pass in all, at the gain r_iv / reference_ohms. Then,
 * with a coupling time constant (0: none), the capacitor into the input of
 * an amplifier: a first-order high-pass, no DC through. Made digital as the
 * board plays it: the DAC holds each sample until the next, the stage
 * filters that, its output taken at the samples' times (step invariance:
 * exact for held samples). */
class DacStage {
public:
	struct Parts {
		double r_iv, c_iv, r1, c1, r2, c2, r3, c3;
	};

	DacStage(const Parts &p, double rate, double reference_ohms, double coupling = 0)
		: m_gain(p.r_iv / reference_ohms), m_held(coupling > 0 ? 0 : 1)
	{
		/* After the I/V stage, Vin / Vout = 1 + a1 s + a2 s^2 + a3 s^3
		 * (the currents of its three nodes) */
		const double a1 = (p.r2 + p.r3) * p.c3 + p.r1 * p.c1 + p.r1 * p.c3;
		const double a2 = p.r2 * p.r3 * p.c2 * p.c3 + p.r1 * p.c1 * (p.r2 + p.r3) * p.c3 + p.r1 * p.r3 * p.c2 * p.c3;
		const double a3 = p.r1 * p.r2 * p.r3 * p.c1 * p.c2 * p.c3;
		const auto cubic = [&](double s) { return ((a3 * s + a2) * s + a1) * s + 1; };

		/* Its real pole: below 0, its coefficients being positive */
		double low = -1.0 / a1, high = 0;

		while (cubic(low) > 0)
			low *= 2;
		for (int i = 0; i < 200; i++) {
			const double mid = (low + high) / 2;

			(cubic(mid) > 0 ? high : low) = mid;
		}

		/* The other two: the roots of what is left once (s - real) is
		 * taken out */
		const double real = (low + high) / 2, q1 = a2 + real * a3, q0 = a1 + real * q1;
		const std::complex<double> root = std::sqrt(std::complex<double>(q1 * q1 - 4 * a3 * q0));
		const std::array<std::complex<double>, 4> poles{ -1.0 / (p.r_iv * p.c_iv), real, (-q1 + root) / (2 * a3),
			                                         (-q1 - root) / (2 * a3) };

		/* A step held from t = 0 comes out as 1 + sum r_i e^(p_i t), r_i
		 * the residue of H(s) / s at p_i, H(0) = 1 (distinct poles, as
		 * these parts give). With the capacitor, H(s) times s / (s - cut):
		 * the 1 goes, a fifth pole at cut, and the step dies away */
		const double cut = coupling > 0 ? -1.0 / coupling : 0;
		std::complex<double> k = 1;

		for (const std::complex<double> &pole : poles)
			k *= -pole;
		for (size_t i = 0; i < poles.size(); i++) {
			std::complex<double> r = k / (poles[i] - cut);

			for (size_t j = 0; j < poles.size(); j++)
				if (j != i)
					r /= poles[i] - poles[j];
			m_branches[i] = { r, std::exp(poles[i] / rate), 0 };
		}
		if (coupling > 0) {
			std::complex<double> r = k;

			for (const std::complex<double> &pole : poles)
				r /= cut - pole;
			m_branches[4] = { r, std::exp(cut / rate), 0 };
		}
	}

	/* y[n] = x[n] + sum s_i[n] (without the capacitor; with it, no x[n]),
	 * s_i[n] = e^(p_i T) s_i[n - 1] + r_i (x[n] - x[n - 1]): the step
	 * response sampled, each step of the input */
	int16_t operator()(int16_t x)
	{
		const double step = x - m_last;
		std::complex<double> sum = 0;

		m_last = x;
		for (Branch &b : m_branches) {
			b.state = b.pole * b.state + b.residue * step;
			/* Nothing heard below it: never the slow denormals of a
			 * sound dying away */
			if (std::norm(b.state) < 1e-24)
				b.state = 0;
			sum += b.state;
		}
		const long out = std::lround((m_held * x + sum.real()) * m_gain);

		return static_cast<int16_t>(std::clamp<long>(out, -0x8000, 0x7FFF));
	}

private:
	struct Branch {
		std::complex<double> residue, pole, state;
	};

	/* 1 without the capacitor, 0 with it */
	double m_gain, m_held, m_last = 0;
	std::array<Branch, 5> m_branches{};
};

} /* namespace odin */

#endif /* ODIN_DAC_STAGE_HPP */
