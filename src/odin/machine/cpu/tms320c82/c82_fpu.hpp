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

#ifndef ODIN_C82_FPU_HPP
#define ODIN_C82_FPU_HPP

#include <bit>
#include <cstdint>

namespace odin {

/* The floating-point unit of the TMS320C82's master processor (TI SPRU109
 * ch. 8 and appendices A and C), in integers: every result and flag is the
 * same on every computer. Values travel as raw bits: a single or a 32-bit
 * integer in the low word, a double in all 64 bits. */

/* The types of operands and results, as P1, P2 and PD code them (table
 * 10-7; fmpy p. 10-97) */
enum C82FpType : uint8_t { FP_SINGLE = 0, FP_DOUBLE = 1, FP_INT = 2, FP_UINT = 3 };

/* Rounding modes, as drm and RM code them (§ 3.3.3, table 8-3) */
enum C82FpRound : uint8_t { ROUND_NEAREST = 0, ROUND_ZERO = 1, ROUND_PLUS = 2, ROUND_MINUS = 3 };

/* The current-status bits of FPST (fig. 3-3) */
enum : uint32_t { FP_X = 1, FP_U = 2, FP_O = 4, FP_Z = 8, FP_I = 16 };

struct C82FpEnv {
	/* What the instruction runs with */
	C82FpRound round = ROUND_NEAREST;
	/* FPST fm = 0: denormal operands and results are zero (§ 8.5.1) */
	bool fast = false;
	/* The overflow, underflow and inexact interrupts enabled: they change
	 * some results (§ 8.5.1, C.5.4, C.5.5) */
	bool trap_o = false, trap_u = false, trap_x = false;

	/* What it found */
	uint32_t flags = 0;
	/* The invalid of a conversion to an integer that overflows: an output
	 * exception, not an input one (§ 8.4.3, C.5.6) */
	bool output_invalid = false;
	/* FPST mo: an integer product beyond 32 bits (C.5.7) */
	bool mo = false;
	/* FPST e0, e1: bits 8 and 9 of the exponent of a wrapped double to
	 * single conversion (C.5.4, C.5.5) */
	uint8_t e01 = 0;
};

/* source1 + source2, or source1 - source2 (fadd, fsub, vadd, vsub) */
uint64_t c82_fp_add(uint64_t a, C82FpType ta, uint64_t b, C82FpType tb, bool subtract, C82FpType td, C82FpEnv &env);
/* source1 x source2 (fmpy, vmpy); all three integers of one kind: the 32
 * low bits of the integer product (fmpy.iii, fmpy.uuu) */
uint64_t c82_fp_mul(uint64_t a, C82FpType ta, uint64_t b, C82FpType tb, C82FpType td, C82FpEnv &env);
/* source1 / source2 (fdiv) */
uint64_t c82_fp_div(uint64_t a, C82FpType ta, uint64_t b, C82FpType tb, C82FpType td, C82FpEnv &env);
uint64_t c82_fp_sqrt(uint64_t a, C82FpType ta, C82FpType td, C82FpEnv &env);
/* A conversion with the rounding of env (frnd, vrnd) */
uint64_t c82_fp_convert(uint64_t a, C82FpType ta, C82FpType td, C82FpEnv &env);
/* The status word of source1 - source2 (fcmp, fig. 10-3, table 10-18) */
uint32_t c82_fp_compare(uint64_t a, C82FpType ta, uint64_t b, C82FpType tb, C82FpEnv &env);
/* The vmac family: the exact double product of two singles (or of a single
 * and 1.0), added to or taken from a double (an accumulator or zero).
 * product_first: product - other (vmsub), else other + product (vmac) or
 * other - product (vmsc). A single result is rounded toward zero and is
 * zero when it is below the single range before rounding (§ 10.3.4, C.2) */
uint64_t c82_fp_mac(uint32_t s1, uint32_t s2, bool by_one, uint64_t other, bool subtract, bool product_first,
                    C82FpType td, C82FpEnv &env);

/* The same operations on the host's IEEE 754 floats, for the common cases
 * only, where they give exactly the bits and flags of the integer functions:
 * operands normal or zero, results well inside the normal range, rounding to
 * nearest (and a vmac single toward zero, done on the bits), the host
 * rounding to nearest too (its default; the caller checks it). Every host
 * Odin runs on (x86-64 SSE2, ARM64) rounds +, -, x, / and conversions
 * correctly, the same way. No denormal is ever an operand or a result of a
 * host operation here, so flush-to-zero settings change nothing; a product of
 * two singles is exact in a double, so a fused multiply-add would give the
 * same bits. A single op done in double then rounded to single is correctly
 * rounded: 53 >= 2 x 24 + 2 (Figueroa, double rounding). Each returns false,
 * changing nothing, for any other case: the integer function then runs. Only
 * inexact can rise here; mo, e0 and e1 stay 0. The conditions are combined
 * with & and | (no branch on the data); a result computed from operands it
 * then refuses is thrown away. */
namespace c82_fast {

/* A single normal or zero */
inline bool
single_in(uint32_t v)
{
	/* The sign dropped: an exponent of 1 to 254, or all zero */
	const uint32_t t = v << 1;

	return (t - 0x01000000u < 0xFE000000u) | (t == 0);
}

/* A double of at least 2^-969 (every sum with it is then a multiple of
 * 2^-1021 at least: no denormal), finite, or zero */
inline bool
double_in(uint64_t v)
{
	/* The sign dropped: an exponent of 54 to 2046, or all zero */
	const uint64_t t = v << 1;

	return (t - (54ull << 53) < (2047ull - 54) << 53) | (t == 0);
}

/* 2^-125 <= |f| < 2^127: neither tiny nor overflowing, in any rounding */
inline bool
single_out(float f)
{
	return ((std::bit_cast<uint32_t>(f) >> 23) & 0xFF) - 2u < 252u;
}

/* 2^-1021 <= |d| < 2^1023 */
inline bool
double_out(double d)
{
	return (static_cast<uint32_t>(std::bit_cast<uint64_t>(d) >> 52) & 0x7FF) - 2u < 2044u;
}

/* The exact error of s = a + b rounded to nearest (Knuth's TwoSum) */
inline double
sum_error(double a, double b, double s)
{
	const double bb = s - a;

	return (a - (s - bb)) + (b - bb);
}

/* fmpy.sss, vmpy.sss: the double product is exact */
inline bool
mul_sss(uint32_t a, uint32_t b, uint32_t &result, uint32_t &flags)
{
	const double p = static_cast<double>(std::bit_cast<float>(a)) * std::bit_cast<float>(b);
	const float f = static_cast<float>(p);

	result = std::bit_cast<uint32_t>(f);
	flags = static_cast<uint32_t>(static_cast<double>(f) != p) * FP_X;
	return single_in(a) & single_in(b) & (single_out(f) | (p == 0));
}

/* fadd.sss, fsub.sss */
inline bool
add_sss(uint32_t a, uint32_t b, bool subtract, uint32_t &result, uint32_t &flags)
{
	const double x = std::bit_cast<float>(a);
	const double y = subtract ? -static_cast<double>(std::bit_cast<float>(b)) : std::bit_cast<float>(b);
	const double s = x + y;
	const float f = static_cast<float>(s);

	result = std::bit_cast<uint32_t>(f);
	flags = static_cast<uint32_t>((sum_error(x, y, s) != 0) | (static_cast<double>(f) != s)) * FP_X;
	return single_in(a) & single_in(b) & (single_out(f) | (s == 0));
}

/* fdiv.sss: inexact unless quotient x divisor (exact) is the dividend */
inline bool
div_sss(uint32_t a, uint32_t b, uint32_t &result, uint32_t &flags)
{
	if (!single_in(a) || !single_in(b) || !(b & 0x7FFFFFFFu))
		return false;

	const double x = std::bit_cast<float>(a), y = std::bit_cast<float>(b);
	const float f = static_cast<float>(x / y);

	if (x != 0 && !single_out(f))
		return false;
	result = std::bit_cast<uint32_t>(f);
	flags = static_cast<double>(f) * y != x ? FP_X : 0u;
	return true;
}

/* A single or a double, normal or zero, as a host double (exact) */
inline bool
operand(uint64_t v, C82FpType t, double &x)
{
	if (t == FP_SINGLE) {
		if (!single_in(static_cast<uint32_t>(v)))
			return false;
		x = std::bit_cast<float>(static_cast<uint32_t>(v));
		return true;
	}
	if (t != FP_DOUBLE || !((static_cast<uint32_t>(v >> 52) & 0x7FF) - 1u < 0x7FEu || !(v << 1)))
		return false;
	x = std::bit_cast<double>(v);
	return true;
}

/* fadd, fsub with a double result: s1 and s2 singles (normal or zero) or
 * doubles of double_in; the error of the double sum tells inexact */
inline bool
add_double(uint64_t a, C82FpType ta, uint64_t b, C82FpType tb, bool subtract, uint64_t &result, uint32_t &flags)
{
	double x = 0, y = 0;

	if ((ta == FP_DOUBLE && !double_in(a)) || (tb == FP_DOUBLE && !double_in(b)) || !operand(a, ta, x) ||
	    !operand(b, tb, y))
		return false;
	if (subtract)
		y = -y;

	const double s = x + y;

	if (s != 0 && !double_out(s))
		return false;
	result = std::bit_cast<uint64_t>(s);
	flags = s != 0 && sum_error(x, y, s) != 0 ? FP_X : 0u;
	return true;
}

/* fmpy.ssd: the product of two singles, exact in a double, in any rounding */
inline bool
mul_ssd(uint32_t a, uint32_t b, uint64_t &result)
{
	if (!single_in(a) || !single_in(b))
		return false;
	result = std::bit_cast<uint64_t>(static_cast<double>(std::bit_cast<float>(a)) * std::bit_cast<float>(b));
	return true;
}

/* fcmp: the status word, no flag */
inline bool
compare(uint64_t a, C82FpType ta, uint64_t b, C82FpType tb, uint32_t &status)
{
	enum : uint32_t {
		EQ = 1u << 20, NE = 1u << 21, GT = 1u << 22, LE = 1u << 23, LT = 1u << 24, GE = 1u << 25,
		OU = 1u << 26, IN = 1u << 27, IB = 1u << 28, OB = 1u << 29, OR = 1u << 31
	};
	double x = 0, y = 0;

	if (!operand(a, ta, x) || !operand(b, tb, y))
		return false;

	const int c = x < y ? -1 : x > y ? 1 : 0, z = x < 0 ? -1 : x > 0 ? 1 : 0;

	status = OR | (c == 0 ? EQ | LE | GE : NE) | (c > 0 ? GT | GE : 0) | (c < 0 ? LT | LE : 0);
	if (!(y < 0)) {
		if (z < 0 || c > 0)
			status |= OU;
		if (z > 0 && c < 0)
			status |= IN;
		if (z >= 0 && c <= 0)
			status |= IB;
		if (z <= 0 || c >= 0)
			status |= OB;
	}
	return true;
}

/* sig with its `drop` low bits rounded off (any number of them), as the
 * integer functions round */
inline uint64_t
round_off(uint64_t sig, int drop, bool sign, C82FpRound mode, bool &inexact)
{
	uint64_t kept = 0;
	bool above = false, tie = false;

	if (drop == 0) {
		inexact = false;
		return sig;
	}
	if (drop >= 64) {
		inexact = sig != 0;
		above = drop == 64 && sig > 1ull << 63;
		tie = drop == 64 && sig == 1ull << 63;
	} else {
		const uint64_t rest = sig & ((1ull << drop) - 1), half = 1ull << (drop - 1);

		kept = sig >> drop;
		inexact = rest != 0;
		above = rest > half;
		tie = rest == half;
	}

	bool up = false;

	switch (mode) {
	case ROUND_NEAREST:
		up = above || (tie && (kept & 1));
		break;
	case ROUND_ZERO:
		break;
	case ROUND_PLUS:
		up = inexact && !sign;
		break;
	case ROUND_MINUS:
		up = inexact && sign;
		break;
	}
	return kept + up;
}

/* frnd, vrnd: a conversion between singles, doubles and integers in any
 * rounding, all in integers; the float result normal (2^-125 or 2^-1021 up,
 * below 2^127 or 2^1023), the integer one in range */
inline bool
convert(uint64_t a, C82FpType ta, C82FpType td, C82FpRound round, uint64_t &result, uint32_t &flags)
{
	bool sign = false;
	uint64_t sig = 0;
	int exp = 0;

	/* sig * 2^(exp - 63), sig's bit 63 set */
	if (ta == FP_SINGLE || ta == FP_DOUBLE) {
		const bool single = ta == FP_SINGLE;
		const uint64_t raw = single ? static_cast<uint32_t>(a) : a;
		const int fbits = single ? 23 : 52;
		const uint32_t e = static_cast<uint32_t>(raw >> fbits) & (single ? 0xFFu : 0x7FFu);

		sign = (raw >> (single ? 31 : 63)) & 1;
		if (e == 0 && !(raw << (single ? 33 : 1))) {
			/* zero */
			if (td == FP_SINGLE || td == FP_DOUBLE)
				result = static_cast<uint64_t>(sign) << (td == FP_SINGLE ? 31 : 63);
			else
				result = 0;
			flags = 0;
			return true;
		}
		if (e == 0 || e == (single ? 0xFFu : 0x7FFu))
			return false;
		sig = (1ull << 63) | (raw & ((1ull << fbits) - 1)) << (63 - fbits);
		exp = static_cast<int>(e) - (single ? 127 : 1023);
	} else {
		const uint32_t w = static_cast<uint32_t>(a);
		uint64_t magnitude = w;

		if (ta == FP_INT && static_cast<int32_t>(w) < 0) {
			sign = true;
			magnitude = static_cast<uint64_t>(-static_cast<int64_t>(static_cast<int32_t>(w)));
		}
		if (!magnitude) {
			result = 0;
			flags = 0;
			return td == FP_SINGLE || td == FP_DOUBLE || td == ta;
		}

		const int lz = std::countl_zero(magnitude);

		sig = magnitude << lz;
		exp = 63 - lz;
	}

	bool inexact = false;

	if (td == FP_SINGLE || td == FP_DOUBLE) {
		const bool single = td == FP_SINGLE;
		const int p = single ? 24 : 53, bias = single ? 127 : 1023;
		uint64_t mantissa = round_off(sig, 64 - p, sign, round, inexact);

		if (mantissa >> p) {
			mantissa >>= 1;
			exp++;
		}

		const int biased = exp + bias;

		if (biased < 2 || biased > (single ? 253 : 2045))
			return false;
		result = static_cast<uint64_t>(sign) << (single ? 31 : 63) | static_cast<uint64_t>(biased) << (p - 1) |
		         (mantissa & ((1ull << (p - 1)) - 1));
		flags = inexact ? FP_X : 0u;
		return true;
	}
	/* An integer: a negative to unsigned is invalid, beyond 2^40 saturated */
	if ((sign && td == FP_UINT) || exp >= 40)
		return false;

	const uint64_t magnitude = round_off(sig, 63 - exp, sign, round, inexact);

	if (magnitude > (td == FP_INT ? (sign ? 0x80000000ull : 0x7FFFFFFFull) : 0xFFFFFFFFull))
		return false;
	result = static_cast<uint32_t>(sign ? 0 - magnitude : magnitude);
	flags = inexact ? FP_X : 0u;
	return true;
}

/* vmac, vmsc: other + s1 x s2, or other - s1 x s2 (exact product); a double
 * result rounded to nearest, a single one toward zero: s rounded to nearest
 * and its exact error e give the truncation of s, one single step lower when
 * s is a single and the exact sum lies below it */
inline bool
mac(uint32_t s1, uint32_t s2, uint64_t other, bool subtract, bool single, uint64_t &result, uint32_t &flags)
{
	double p = static_cast<double>(std::bit_cast<float>(s1)) * std::bit_cast<float>(s2);

	if (subtract)
		p = -p;

	const double a = std::bit_cast<double>(other), s = a + p, e = sum_error(a, p, s);
	const uint64_t bits = std::bit_cast<uint64_t>(s);
	/* Two zeros or an exact cancellation give a zero sum, exact: e = 0 */
	const bool zero = !(bits << 1), in = single_in(s1) & single_in(s2) & double_in(other);

	if (!single) {
		result = bits;
		flags = static_cast<uint32_t>(e != 0) * FP_X;
		return in & (double_out(s) | zero);
	}

	/* 2^-125 <= |s| < 2^127, or zero */
	const bool out = ((static_cast<uint32_t>(bits >> 52) & 0x7FF) - (1023u - 125u) < 252u) | zero;
	const uint64_t t = bits & ~((1ull << 29) - 1);
	const bool below = (t == bits) & (e != 0) & ((e < 0) != (s < 0));

	result = std::bit_cast<uint32_t>(static_cast<float>(std::bit_cast<double>(t - (static_cast<uint64_t>(below) << 29))));
	flags = static_cast<uint32_t>((e != 0) | (t != bits)) * FP_X;
	return in & out;
}

} /* namespace c82_fast */

} /* namespace odin */

#endif /* ODIN_C82_FPU_HPP */
