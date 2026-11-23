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

#include "odin/machine/cpu/tms320c82/c82_fpu.hpp"

#include <bit>
#include <utility>

namespace odin {

namespace {

__extension__ typedef unsigned __int128 Wide;

enum Kind : uint8_t { ZERO, FINITE, INF, QNAN, SNAN };

/* A number: FINITE is sig * 2^(exp - 63), sig's bit 63 set; a NaN keeps
 * its fraction left-aligned in sig */
struct Num {
	Kind kind = ZERO;
	bool sign = false;
	int32_t exp = 0;
	uint64_t sig = 0;
};

struct Format {
	int p, bias, emin, emax, ebits, fbits, width;
};

constexpr Format SINGLE{ 24, 127, -126, 127, 8, 23, 32 }, DOUBLE{ 53, 1023, -1022, 1023, 11, 52, 64 };

/* How a result that leaves the range is given when its interrupt is on:
 * a wrapped exponent for products and quotients, a denormal for sums, the
 * input's exponent bits for a double to single conversion (C.5.4, C.5.5) */
enum Class : uint8_t { SUM, PRODUCT, DOUBLE_TO_SINGLE };

const Format &
format_of(C82FpType t)
{
	return t == FP_DOUBLE ? DOUBLE : SINGLE;
}

bool
is_float(C82FpType t)
{
	return t == FP_SINGLE || t == FP_DOUBLE;
}

int
leading_zeros(Wide v)
{
	const uint64_t high = static_cast<uint64_t>(v >> 64);

	return high ? std::countl_zero(high) : 64 + std::countl_zero(static_cast<uint64_t>(v));
}

Num
unpack(uint64_t bits, C82FpType t, const C82FpEnv &env)
{
	Num n;

	if (!is_float(t)) {
		const uint32_t w = static_cast<uint32_t>(bits);
		uint64_t magnitude = w;

		if (t == FP_INT && static_cast<int32_t>(w) < 0) {
			n.sign = true;
			magnitude = static_cast<uint64_t>(-static_cast<int64_t>(static_cast<int32_t>(w)));
		}
		if (magnitude) {
			const int lz = std::countl_zero(magnitude);

			n.kind = FINITE;
			n.sig = magnitude << lz;
			n.exp = 63 - lz;
		}
		return n;
	}

	const Format &f = format_of(t);
	const uint64_t raw = t == FP_SINGLE ? static_cast<uint32_t>(bits) : bits;
	const uint32_t e = static_cast<uint32_t>(raw >> f.fbits) & ((1u << f.ebits) - 1);
	const uint64_t frac = raw & ((1ull << f.fbits) - 1);

	n.sign = (raw >> (f.width - 1)) & 1;
	if (e == (1u << f.ebits) - 1) {
		if (!frac) {
			n.kind = INF;
			return n;
		}
		/* The first fraction bit tells a quiet NaN from a signaling one
		 * (A.5) */
		n.kind = (frac >> (f.fbits - 1)) & 1 ? QNAN : SNAN;
		n.sig = frac << (64 - f.fbits);
		return n;
	}
	if (e == 0) {
		/* A denormal is zero in fast mode, single or double (§ 8.5.1) */
		if (!frac || env.fast)
			return n;

		const int lz = std::countl_zero(frac);

		n.kind = FINITE;
		n.sig = frac << lz;
		n.exp = (63 - lz) + f.emin - f.fbits;
		return n;
	}
	n.kind = FINITE;
	n.sig = (1ull << 63) | (frac << (63 - f.fbits));
	n.exp = static_cast<int32_t>(e) - f.bias;
	return n;
}

uint64_t
pack_raw(bool sign, uint64_t exponent_and_fraction, const Format &f)
{
	return (static_cast<uint64_t>(sign) << (f.width - 1)) | exponent_and_fraction;
}

uint64_t
fields(uint32_t biased, uint64_t fraction, const Format &f)
{
	return (static_cast<uint64_t>(biased) << f.fbits) | fraction;
}

/* The top bits of sig, rounded: `drop` low bits go, which may be all of
 * them and more (IEEE 754 rounding, § 8.2.4: nearest rounds a tie to even) */
uint64_t
round_drop(Wide sig, int drop, bool sign, C82FpRound mode, bool &inexact)
{
	uint64_t kept = 0;
	bool above = false, tie = false;

	if (drop >= 129) {
		inexact = sig != 0;
	} else if (drop == 128) {
		const Wide half = static_cast<Wide>(1) << 127;

		inexact = sig != 0;
		above = sig > half;
		tie = sig == half;
	} else {
		const Wide rest = sig & ((static_cast<Wide>(1) << drop) - 1);
		const Wide half = drop ? static_cast<Wide>(1) << (drop - 1) : 0;

		kept = static_cast<uint64_t>(sig >> drop);
		inexact = rest != 0;
		above = drop && rest > half;
		tie = drop && rest == half;
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

uint64_t
all_ones_nan(C82FpType t)
{
	return t == FP_DOUBLE ? ~0ull : 0xFFFFFFFFull;
}

/* An integer result that cannot be: the largest positive for a positive
 * source or a NaN, the most negative for a negative one; 0 for a negative
 * source of an unsigned one (C.5.6) */
uint64_t
saturated(bool negative, C82FpType td)
{
	if (td == FP_INT)
		return negative ? 0x80000000u : 0x7FFFFFFFu;
	return negative ? 0u : 0xFFFFFFFFu;
}

/* The result of an invalid operation: the QNaN of all ones (A.5, C.5.1),
 * or a saturated integer (C.5.6) */
uint64_t
invalid(C82FpType td, C82FpEnv &env, bool negative = false)
{
	env.flags |= FP_I;
	return is_float(td) ? all_ones_nan(td) : saturated(negative, td);
}

/* A quiet NaN given to td: its sign, an exponent of ones, its fraction cut
 * or filled with zeros (A.5). An integer destination cannot hold it. */
uint64_t
nan_to(const Num &n, C82FpType td, C82FpEnv &env)
{
	if (!is_float(td))
		return invalid(td, env);

	const Format &f = format_of(td);

	return pack_raw(n.sign, fields((1u << f.ebits) - 1, n.sig >> (64 - f.fbits), f), f);
}

uint64_t
wrapped(bool sign, int32_t exponent, uint64_t mantissa, C82FpType td, Class cls, C82FpEnv &env)
{
	const Format &f = format_of(td);
	const uint64_t fraction = mantissa & ((1ull << f.fbits) - 1);

	if (cls == DOUBLE_TO_SINGLE) {
		/* The double's 11-bit exponent: bits 7-0 in the result, 8 and 9 in
		 * FPST e0 and e1 (C.5.4) */
		const uint32_t biased = static_cast<uint32_t>(exponent + DOUBLE.bias);

		env.e01 = static_cast<uint8_t>((biased >> 8) & 3);
		return pack_raw(sign, fields(biased & 0xFF, fraction, f), f);
	}
	/* The exponent wraps around (A.6) */
	const uint32_t biased = static_cast<uint32_t>(exponent + f.bias) & ((1u << f.ebits) - 1);

	return pack_raw(sign, fields(biased, fraction, f), f);
}

/* sig * 2^(exp - 127), sig's bit 127 set, rounded into a float */
uint64_t
pack_float(bool sign, int32_t exp, Wide sig, C82FpType td, Class cls, C82FpEnv &env)
{
	const Format &f = format_of(td);
	const uint32_t ones = (1u << f.ebits) - 1;
	bool inexact = false;
	uint64_t mantissa = round_drop(sig, 128 - f.p, sign, env.round, inexact);
	int32_t rounded = exp;

	if (mantissa >> f.p) {
		mantissa >>= 1;
		rounded++;
	}
	if (rounded > f.emax) {
		if (env.trap_o) {
			env.flags |= FP_O | (inexact ? FP_X : 0u);
			return wrapped(sign, rounded, mantissa, td, cls, env);
		}
		/* Overflow is inexact when its interrupt is off (C.5.3) */
		env.flags |= FP_O | FP_X;

		const bool infinite = env.round == ROUND_NEAREST || (env.round == ROUND_PLUS && !sign) ||
		                      (env.round == ROUND_MINUS && sign);

		return pack_raw(sign, infinite ? fields(ones, 0, f) : fields(ones - 1, (1ull << f.fbits) - 1, f), f);
	}
	if (rounded >= f.emin) {
		if (inexact)
			env.flags |= FP_X;
		return pack_raw(sign, fields(static_cast<uint32_t>(rounded + f.bias), mantissa & ((1ull << f.fbits) - 1), f),
		                f);
	}
	/* Tiny. HEURISTIC: tininess is seen after rounding, as IEEE 754 allows;
	 * TI does not say which (§ 8.4.2) */
	if (env.trap_u && cls != SUM) {
		env.flags |= FP_U | (inexact ? FP_X : 0u);
		return wrapped(sign, rounded, mantissa, td, cls, env);
	}
	if (env.fast && !env.trap_u && !env.trap_x) {
		/* Fast mode: zero, inexact and underflow (§ 8.5.1) */
		env.flags |= FP_U | FP_X;
		return pack_raw(sign, 0, f);
	}

	bool denormal_inexact = false;
	const uint64_t denormal = round_drop(sig, 128 - f.p + (f.emin - exp), sign, env.round, denormal_inexact);

	if (env.trap_u || denormal_inexact)
		env.flags |= FP_U;
	if (denormal_inexact)
		env.flags |= FP_X;
	/* Fast mode with the inexact interrupt on: every denormal result is
	 * inexact (§ 8.5.1) */
	if (env.fast && env.trap_x && denormal < (1ull << (f.p - 1)))
		env.flags |= FP_X;
	/* A mantissa rounded up to 2^(p-1) is the smallest normal */
	return pack_raw(sign, denormal, f);
}

/* sig * 2^(exp - 127) rounded into an integer, with the rules of C.5.6 */
uint64_t
pack_int(bool sign, int32_t exp, Wide sig, C82FpType td, C82FpEnv &env)
{
	if (!sig)
		return 0;
	/* Any negative number but zero is invalid for an unsigned integer
	 * (§ 8.4.1, C.5.1) */
	if (sign && td == FP_UINT)
		return invalid(td, env, true);
	if (exp >= 40) {
		env.flags |= FP_I;
		env.output_invalid = true;
		return saturated(sign, td);
	}

	bool inexact = false;
	const uint64_t magnitude = round_drop(sig, 127 - exp, sign, env.round, inexact);
	const uint64_t most = td == FP_INT ? (sign ? 0x80000000ull : 0x7FFFFFFFull) : 0xFFFFFFFFull;

	if (magnitude > most) {
		/* An overflow is an output exception (C.5.6) */
		env.flags |= FP_I;
		env.output_invalid = true;
		return saturated(sign, td);
	}
	if (inexact)
		env.flags |= FP_X;
	return static_cast<uint32_t>(sign ? 0 - magnitude : magnitude);
}

uint64_t
pack_wide(bool sign, int32_t exp, Wide sig, C82FpType td, Class cls, C82FpEnv &env)
{
	return is_float(td) ? pack_float(sign, exp, sig, td, cls, env) : pack_int(sign, exp, sig, td, env);
}

uint64_t
from_num(const Num &n, C82FpType td, Class cls, C82FpEnv &env)
{
	switch (n.kind) {
	case ZERO:
		return is_float(td) ? pack_raw(n.sign, 0, format_of(td)) : 0;
	case INF:
		if (!is_float(td))
			return invalid(td, env, n.sign);
		return pack_raw(n.sign, fields((1u << format_of(td).ebits) - 1, 0, format_of(td)), format_of(td));
	case QNAN:
		return nan_to(n, td, env);
	case SNAN:
		return invalid(td, env);
	case FINITE:
		break;
	}
	return pack_wide(n.sign, n.exp, static_cast<Wide>(n.sig) << 64, td, cls, env);
}

/* The NaN rules of A.5: a signaling NaN is invalid; else source1's quiet
 * NaN, else source2's. True when one of them was a NaN. */
bool
nan_operands(const Num &x, const Num &y, C82FpType td, C82FpEnv &env, uint64_t &result)
{
	if (x.kind == SNAN || y.kind == SNAN) {
		result = invalid(td, env);
		return true;
	}
	if (x.kind == QNAN || y.kind == QNAN) {
		result = nan_to(x.kind == QNAN ? x : y, td, env);
		return true;
	}
	return false;
}

/* x + y for two finite numbers, exact: sig's bit 127 set, bit 0 also
 * standing for anything lost below it; zero when they cancel */
Wide
exact_sum(const Num &x, const Num &y, bool &sign, int32_t &exp)
{
	const Num *big = &x, *small = &y;

	if (y.exp > x.exp || (y.exp == x.exp && y.sig > x.sig))
		std::swap(big, small);

	const int shift = big->exp - small->exp;
	const Wide a = static_cast<Wide>(big->sig) << 63;
	Wide b = static_cast<Wide>(small->sig) << 63;
	bool lost = false;

	if (shift >= 127) {
		b = 0;
		lost = true;
	} else if (shift > 0) {
		lost = (b & ((static_cast<Wide>(1) << shift) - 1)) != 0;
		b >>= shift;
	}

	Wide s;

	if (big->sign == small->sign)
		s = a + b;
	else
		s = a - b - (lost ? 1 : 0);
	if (lost)
		s |= 1;
	sign = big->sign;
	if (!s)
		return 0;

	const int lz = leading_zeros(s);

	exp = big->exp + 1 - lz;
	return s << lz;
}

uint64_t
add_nums(Num x, Num y, C82FpType td, Class cls, C82FpEnv &env, bool tiny_is_zero = false)
{
	uint64_t result = 0;

	if (nan_operands(x, y, td, env, result))
		return result;
	if (x.kind == INF || y.kind == INF) {
		/* The magnitude subtraction of infinities (C.5.1) */
		if (x.kind == INF && y.kind == INF && x.sign != y.sign)
			return invalid(td, env);
		return from_num(x.kind == INF ? x : y, td, cls, env);
	}
	if (x.kind == ZERO && y.kind == ZERO) {
		/* IEEE 754: opposite zeros sum to +0, or -0 rounding down */
		x.sign = x.sign == y.sign ? x.sign : env.round == ROUND_MINUS;
		return from_num(x, td, cls, env);
	}
	if (x.kind == ZERO || y.kind == ZERO) {
		const Num &n = x.kind == ZERO ? y : x;

		if (tiny_is_zero && n.exp < SINGLE.emin) {
			env.flags |= FP_U | FP_X;
			return pack_raw(n.sign, 0, SINGLE);
		}
		return from_num(n, td, cls, env);
	}

	bool sign = false;
	int32_t exp = 0;
	const Wide sum = exact_sum(x, y, sign, exp);

	if (!sum)
		return from_num(Num{ ZERO, env.round == ROUND_MINUS, 0, 0 }, td, cls, env);
	/* vmac.sss: a sum below the single range before rounding is zero,
	 * inexact and underflow (C.2) */
	if (tiny_is_zero && exp < SINGLE.emin) {
		env.flags |= FP_U | FP_X;
		return pack_raw(sign, 0, SINGLE);
	}
	return pack_wide(sign, exp, sum, td, cls, env);
}

/* x * y exact: two significands of 64 bits give 128 */
void
exact_product(const Num &x, const Num &y, bool &sign, int32_t &exp, Wide &sig)
{
	sig = static_cast<Wide>(x.sig) * y.sig;
	sign = x.sign != y.sign;
	exp = x.exp + y.exp + 1;
	if (!(sig >> 127)) {
		sig <<= 1;
		exp--;
	}
}

/* Most significant 64 bits of a root of n, and whether the root is exact
 * (digit by digit, two bits of n at a time) */
uint64_t
root_of(Wide n, bool &exact)
{
	Wide rest = 0, root = 0;

	for (int i = 0; i < 64; i++) {
		rest = (rest << 2) | (n >> 126);
		n <<= 2;
		root <<= 1;

		const Wide trial = (root << 1) | 1;

		if (rest >= trial) {
			rest -= trial;
			root |= 1;
		}
	}
	exact = rest == 0;
	return static_cast<uint64_t>(root);
}

int
compare_nums(const Num &x, const Num &y)
{
	const auto signum = [](const Num &n) { return n.kind == ZERO ? 0 : n.sign ? -1 : 1; };
	const int sx = signum(x), sy = signum(y);

	if (sx != sy)
		return sx < sy ? -1 : 1;
	if (!sx)
		return 0;

	/* Same sign: compare the magnitudes, then turn for negatives */
	int magnitude = 0;

	if (x.kind == INF || y.kind == INF)
		magnitude = x.kind == y.kind ? 0 : x.kind == INF ? 1 : -1;
	else if (x.exp != y.exp)
		magnitude = x.exp > y.exp ? 1 : -1;
	else if (x.sig != y.sig)
		magnitude = x.sig > y.sig ? 1 : -1;
	return sx < 0 ? -magnitude : magnitude;
}

} /* namespace */

uint64_t
c82_fp_add(uint64_t a, C82FpType ta, uint64_t b, C82FpType tb, bool subtract, C82FpType td, C82FpEnv &env)
{
	const Num x = unpack(a, ta, env);
	Num y = unpack(b, tb, env);

	if (subtract && y.kind != QNAN && y.kind != SNAN)
		y.sign = !y.sign;
	return add_nums(x, y, td, SUM, env);
}

uint64_t
c82_fp_mul(uint64_t a, C82FpType ta, uint64_t b, C82FpType tb, C82FpType td, C82FpEnv &env)
{
	/* fmpy.iii, fmpy.uuu: the 32 low bits; mo when the product does not fit
	 * in 32 bits, and no other exception (p. 10-97, C.5.7) */
	if (!is_float(ta) && ta == tb && tb == td) {
		if (ta == FP_INT) {
			const int64_t p = static_cast<int64_t>(static_cast<int32_t>(a)) * static_cast<int32_t>(b);

			env.mo = p < INT32_MIN || p > INT32_MAX;
			return static_cast<uint32_t>(p);
		}

		const uint64_t p = static_cast<uint64_t>(static_cast<uint32_t>(a)) * static_cast<uint32_t>(b);

		env.mo = p > 0xFFFFFFFFull;
		return static_cast<uint32_t>(p);
	}

	const Num x = unpack(a, ta, env), y = unpack(b, tb, env);
	uint64_t result = 0;

	if (nan_operands(x, y, td, env, result))
		return result;

	const bool sign = x.sign != y.sign;

	if ((x.kind == INF && y.kind == ZERO) || (x.kind == ZERO && y.kind == INF))
		return invalid(td, env);
	if (x.kind == INF || y.kind == INF)
		return from_num(Num{ INF, sign, 0, 0 }, td, PRODUCT, env);
	if (x.kind == ZERO || y.kind == ZERO)
		return from_num(Num{ ZERO, sign, 0, 0 }, td, PRODUCT, env);

	bool product_sign = false;
	int32_t exp = 0;
	Wide sig = 0;

	exact_product(x, y, product_sign, exp, sig);
	return pack_wide(product_sign, exp, sig, td, PRODUCT, env);
}

uint64_t
c82_fp_div(uint64_t a, C82FpType ta, uint64_t b, C82FpType tb, C82FpType td, C82FpEnv &env)
{
	const Num x = unpack(a, ta, env), y = unpack(b, tb, env);
	uint64_t result = 0;

	if (nan_operands(x, y, td, env, result))
		return result;

	const bool sign = x.sign != y.sign;

	if ((x.kind == INF && y.kind == INF) || (x.kind == ZERO && y.kind == ZERO))
		return invalid(td, env);
	if (x.kind == INF || y.kind == ZERO) {
		/* A finite nonzero dividend over zero: divide by zero and a signed
		 * infinity (§ 8.4.1, C.5.2) */
		if (x.kind != INF)
			env.flags |= FP_Z;
		return from_num(Num{ INF, sign, 0, 0 }, td, PRODUCT, env);
	}
	if (x.kind == ZERO || y.kind == INF)
		return from_num(Num{ ZERO, sign, 0, 0 }, td, PRODUCT, env);

	const Wide dividend = static_cast<Wide>(x.sig) << 64;
	const Wide quotient = dividend / y.sig;
	const bool rest = dividend % y.sig != 0;
	const int lz = leading_zeros(quotient);
	const int32_t exp = (127 - lz) - 64 + x.exp - y.exp;

	return pack_wide(sign, exp, (quotient << lz) | (rest ? 1 : 0), td, PRODUCT, env);
}

uint64_t
c82_fp_sqrt(uint64_t a, C82FpType ta, C82FpType td, C82FpEnv &env)
{
	const Num x = unpack(a, ta, env);

	switch (x.kind) {
	case SNAN:
		return invalid(td, env);
	case QNAN:
		return nan_to(x, td, env);
	case ZERO:
		/* The root of -0 is -0 (§ 8.4.1) */
		return from_num(x, td, PRODUCT, env);
	case INF:
	case FINITE:
		break;
	}
	if (x.sign)
		return invalid(td, env);
	if (x.kind == INF)
		return from_num(x, td, PRODUCT, env);

	/* An even power of two under the root */
	const int32_t k = x.exp - 63;
	const int shift = (k & 1) ? 63 : 64;
	bool exact = false;
	const uint64_t root = root_of(static_cast<Wide>(x.sig) << shift, exact);

	return pack_wide(false, 63 + (k - shift) / 2, (static_cast<Wide>(root) << 64) | (exact ? 0 : 1), td, PRODUCT,
	                 env);
}

uint64_t
c82_fp_convert(uint64_t a, C82FpType ta, C82FpType td, C82FpEnv &env)
{
	const Num x = unpack(a, ta, env);

	return from_num(x, td, ta == FP_DOUBLE && td == FP_SINGLE ? DOUBLE_TO_SINGLE : SUM, env);
}

uint32_t
c82_fp_compare(uint64_t a, C82FpType ta, uint64_t b, C82FpType tb, C82FpEnv &env)
{
	enum : uint32_t {
		EQ = 1u << 20, NE = 1u << 21, GT = 1u << 22, LE = 1u << 23, LT = 1u << 24, GE = 1u << 25,
		OU = 1u << 26, IN = 1u << 27, IB = 1u << 28, OB = 1u << 29, UO = 1u << 30, OR = 1u << 31
	};
	const Num x = unpack(a, ta, env), y = unpack(b, tb, env);

	/* A signaling NaN is invalid; any NaN is unordered, every other bit
	 * then 0 (A.5, table 10-18 note 2) */
	if (x.kind == SNAN || y.kind == SNAN)
		env.flags |= FP_I;
	if (x.kind == SNAN || y.kind == SNAN || x.kind == QNAN || y.kind == QNAN)
		return UO;

	const int c = compare_nums(x, y);
	uint32_t status = OR | (c == 0 ? EQ | LE | GE : NE) | (c > 0 ? GT | GE : 0) | (c < 0 ? LT | LE : 0);

	/* The range bits, all 0 when source2 is negative (note 3) */
	if (!(y.kind != ZERO && y.sign)) {
		const Num zero;
		const int to_zero = compare_nums(x, zero);

		if (to_zero < 0 || c > 0)
			status |= OU;
		if (to_zero > 0 && c < 0)
			status |= IN;
		if (to_zero >= 0 && c <= 0)
			status |= IB;
		if (to_zero <= 0 || c >= 0)
			status |= OB;
	}
	return status;
}

uint64_t
c82_fp_mac(uint32_t s1, uint32_t s2, bool by_one, uint64_t other, bool subtract, bool product_first, C82FpType td,
           C82FpEnv &env)
{
	const Num x = unpack(s1, FP_SINGLE, env);
	const Num y = by_one ? Num{ FINITE, false, 0, 1ull << 63 } : unpack(s2, FP_SINGLE, env);
	Num acc = unpack(other, FP_DOUBLE, env);
	Num product;

	/* The multiply: an invalid one gives the add the QNaN of all ones
	 * (C.2); otherwise exact, two singles fit in a double */
	if (x.kind == SNAN || y.kind == SNAN || (x.kind == INF && y.kind == ZERO) ||
	    (x.kind == ZERO && y.kind == INF)) {
		env.flags |= FP_I;
		product = Num{ QNAN, true, 0, ~0ull };
	} else if (x.kind == QNAN || y.kind == QNAN) {
		product = x.kind == QNAN ? x : y;
	} else if (x.kind == INF || y.kind == INF) {
		product = Num{ INF, x.sign != y.sign, 0, 0 };
	} else if (x.kind == ZERO || y.kind == ZERO) {
		product = Num{ ZERO, x.sign != y.sign, 0, 0 };
	} else {
		bool sign = false;
		int32_t exp = 0;
		Wide sig = 0;

		exact_product(x, y, sign, exp, sig);
		product = Num{ FINITE, sign, exp, static_cast<uint64_t>(sig >> 64) };
	}

	/* A single result rounds toward zero (§ 10.3.4) */
	const bool single = td == FP_SINGLE;

	if (single)
		env.round = ROUND_ZERO;
	if (product_first) {
		if (acc.kind != QNAN && acc.kind != SNAN)
			acc.sign = !acc.sign;
		return add_nums(product, acc, td, SUM, env, single);
	}
	if (subtract && product.kind != QNAN && product.kind != SNAN)
		product.sign = !product.sign;
	return add_nums(acc, product, td, SUM, env, single);
}

} /* namespace odin */
