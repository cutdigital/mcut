/****************************************************************************
* NFG - Numbers for Geometry                     					        *
*                                                                           *
* Consiglio Nazionale delle Ricerche                                        *
* Istituto di Matematica Applicata e Tecnologie Informatiche                *
* Sezione di Genova                                                         *
* IMATI-GE / CNR                                                            *
*                                                                           *
* Authors: Marco Attene                                                     *
* Copyright(C) 2019: IMATI-GE / CNR                                         *
* All rights reserved.                                                      *
*                                                                           *
* This program is free software; you can redistribute it and/or modify      *
* it under the terms of the GNU Lesser General Public License as published  *
* by the Free Software Foundation; either version 3 of the License, or (at  *
* your option) any later version.                                           *
*                                                                           *
* This program is distributed in the hope that it will be useful, but       *
* WITHOUT ANY WARRANTY; without even the implied warranty of                *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser  *
* General Public License for more details.                                  *
*                                                                           *
* You should have received a copy of the GNU Lesser General Public License  *
* along with this program.  If not, see http://www.gnu.org/licenses/.       *
*                                                                           *
****************************************************************************/

///////////////////////////////////////////////////////////////////////////////////////////////////////
//
// To compile on MSVC: use /fp:strict
// On GNU GCC: use -frounding-math
//
///////////////////////////////////////////////////////////////////////////////////////////////////////

#include "numerics.h"
#include <string>
#include <cstring>
#include <algorithm>

#pragma intrinsic(fabs)

inline void initFPU()
{
#ifdef IS64BITPLATFORM
#ifdef USE_SIMD_INSTRUCTIONS
//	_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
#endif
#else
#ifdef USE_SIMD_INSTRUCTIONS
#error "USE_SIMD_INSTRUCTIONS cannot be defined in 32-bit mode"
#endif
#ifdef ISVISUALSTUDIO
	_control87(_PC_53, _MCW_PC); /* Set FPU control word for double precision. */
#else
	int cword;
	cword = 4722;                 /* set FPU control word for double precision */
	_FPU_SETCW(cword);
#endif
#endif
}

#ifdef USE_SIMD_INSTRUCTIONS

#ifdef USE_AVX2_INSTRUCTIONS
inline interval_number interval_number::operator*(const interval_number& b) const
{
	// This version exploits 256bit registers provided by AVX2 architectures
	// to compute the product using the "naive" eight-multiplications method.
	// The advantage wrt to the non-avx version is due to the absence of
	// branches in the execution, which increses the processor's throughput.

	// Fill i1 and i2 with two copies of 'this' and 'b' respectively
	__m256d i1 = _mm256_castpd128_pd256(interval);
	i1 = _mm256_insertf128_pd(i1, interval, 1);
	__m256d i2 = _mm256_castpd128_pd256(b.interval);
	i2 = _mm256_insertf128_pd(i2, b.interval, 1);

	// Swizzle and change sign appropriately to produce all the eight configs
	__m256d x2 = _mm256_shuffle_pd(i1, i1, 5);
	__m256d x3 = _mm256_xor_pd(i2, _mm256_set_pd(-0.0, -0.0, 0.0, 0.0));
	__m256d x4 = _mm256_xor_pd(i2, _mm256_set_pd(0.0, 0.0, -0.0, -0.0));
	x3 = _mm256_mul_pd(i1, x3);
	x2 = _mm256_mul_pd(x2, x4);
	x3 = _mm256_max_pd(x3, x2);
	x4 = _mm256_shuffle_pd(x3, x3, 5);
	x3 = _mm256_max_pd(x3, x4);
	x3 = _mm256_permute4x64_pd(x3, 72);

	// The first two vals of the 256 vector are the resulting product
	return _mm256_castpd256_pd128(x3);
}
#else
inline interval_number interval_number::operator*(const interval_number& b) const
{
	// <a0,a1> * <b0,b1>
	__m128d ssg;
	__m128d llhh, lhhl, ip;

	switch ((_mm_movemask_pd(interval) << 2) | _mm_movemask_pd(b.interval))
	{
	case 0: // -+ * -+: <min(<a0*b1>,<a1*b0>), max(<a0*b0>,<a1*b1>)>
		llhh = _mm_mul_pd(interval, b.interval);
		lhhl = _mm_mul_pd(interval, _mm_shuffle_pd(b.interval, b.interval, 1));
		return interval_number(_mm_max_pd(_mm_unpacklo_pd(llhh, lhhl), _mm_unpackhi_pd(llhh, lhhl)));
	case 1: // -+ * --: <b0*a1, b0*a0>
		return interval_number(_mm_mul_pd(_mm_shuffle_pd(b.interval, b.interval, 3), _mm_shuffle_pd(interval, interval, 1)));
	case 2: // -+ * ++: <b1*a0, b1*a1>
		return interval_number(_mm_mul_pd(_mm_shuffle_pd(b.interval, b.interval, 0), interval));
	case 4: // -- * -+: <a0*b1, a0*b0>
		return interval_number(_mm_mul_pd(_mm_shuffle_pd(interval, interval, 3), _mm_shuffle_pd(b.interval, b.interval, 1)));
	case 5: // -- * --: <a1*b1, a0*b0>
		ip = _mm_mul_pd(_mm_xor_pd(interval, sign_high_mask()), b.interval);
		return interval_number(_mm_shuffle_pd(ip, ip, 1));
	case 6: // -- * ++: <a0*b1, a1*b0>
		ssg = _mm_xor_pd(b.interval, sign_low_mask());
		return interval_number(_mm_mul_pd(interval, _mm_shuffle_pd(ssg, ssg, 1)));
	case 8: // ++ * -+: <a1*b0, a1*b1>
		return interval_number(_mm_mul_pd(_mm_shuffle_pd(interval, interval, 0), b.interval));
	case 9: // ++ * --: <b0*a1, b1*a0>
		ssg = _mm_xor_pd(interval, sign_low_mask());
		return interval_number(_mm_mul_pd(b.interval, _mm_shuffle_pd(ssg, ssg, 1)));
	case 10: // ++ * ++: <a0*b0, a1*b1>
		return interval_number(_mm_mul_pd(interval, _mm_xor_pd(b.interval, sign_low_mask())));
	}

	return interval_number(NAN);
}
#endif

inline interval_number interval_number::operator*(const double b) const {
	if (b >= 0) return interval_number(_mm_mul_pd(interval, _mm_set1_pd(b)));
	else return interval_number(_mm_mul_pd(_mm_shuffle_pd(interval, interval, 1), _mm_set1_pd(-b)));
}

inline interval_number interval_number::operator/(const double b) const {
	if (b >= 0) return interval_number(_mm_div_pd(interval, _mm_set1_pd(b)));
	else return interval_number(_mm_div_pd(_mm_shuffle_pd(interval, interval, 1), _mm_set1_pd(-b)));
}

inline interval_number interval_number::abs() const {
	switch (_mm_movemask_pd(interval))
	{
	case 0: // Hi>0, Lo<0
		return _mm_and_pd(_mm_max_pd(interval, _mm_shuffle_pd(interval, interval, 1)), all_high_mask());
	case 1: // Hi<0, Lo<0
		return _mm_shuffle_pd(interval, interval, 1);
	}
	return *this; // If Hi>0, Lo>0 OR invalid interval == case 3 above
}

inline interval_number interval_number::sqr() const {
	const interval_number av = abs();
	return interval_number(_mm_mul_pd(av.getLowSwitched(), av.interval));
}

inline interval_number min(const interval_number& a, const interval_number& b) {
	const __m128d ai = a.getLowSwitched(), bi = b.getLowSwitched();
	const __m128d m = _mm_min_pd(ai, bi);
	return interval_number(m).getLowSwitched();
}

inline interval_number max(const interval_number& a, const interval_number& b) {
	const __m128d ai = a.getLowSwitched(), bi = b.getLowSwitched();
	const __m128d m = _mm_max_pd(ai, bi);
	return interval_number(m).getLowSwitched();
}

inline interval_number interval_number::inverse() const
{
	const int m = _mm_movemask_pd(interval);
	if (m == 1 || m == 2)
	{
		const __m128d den = _mm_shuffle_pd(interval, interval, 1);
		const __m128d frac = _mm_div_pd(minus_one(), den);
		return interval_number(frac);
	}
	else
	{
		return interval_number(NAN);
	}
}

inline interval_number interval_number::pow(unsigned int e) const {
	if (e == 0) return interval_number(1.0);

	__m128d uns = _mm_and_pd(interval, sign_fabs_mask());
	__m128d ui = interval;

	if (!(e & (unsigned int)1)) { // e is even
		const int s = _mm_movemask_pd(interval);
		if (s == 0) {
			__m128d swapped = _mm_shuffle_pd(uns, uns, 1);
			if (_mm_comigt_sd(swapped, uns)) {
				ui = _mm_shuffle_pd(uns, uns, 1);
				uns = swapped;
			}
			uns = _mm_and_pd(uns, all_high_mask());
		}
		else if (s == 1) {
			ui = _mm_shuffle_pd(ui, ui, 1);
			uns = _mm_shuffle_pd(uns, uns, 1);
		}
	}

	while (--e) ui = _mm_mul_pd(ui, uns);
	return interval_number(ui);
}

#else
inline interval_number interval_number::operator*(const interval_number& b) const
{
	casted_double l1(min_low), h1(high), l2(b.min_low), h2(b.high);
	uint64_t cfg = (l1.is_negative() << 3) + (h1.is_negative() << 2) + (l2.is_negative() << 1) + (h2.is_negative());

	switch (cfg)
	{
	case 10: return interval_number(min_low * (-b.min_low), high * b.high);
	case 8: return interval_number(high * b.min_low, high * b.high);
	case 9: return interval_number(high * b.min_low, (-min_low) * b.high);
	case 2: return interval_number(min_low * b.high, high * b.high);
	case 0:
		double ll, lh, hl, hh;
		ll = min_low * b.min_low; lh = (min_low * b.high); hl = (high * b.min_low); hh = high * b.high;
		if (hl > lh) lh = hl;
		if (ll > hh) hh = ll;
		return interval_number(lh, hh);
	case 1: return interval_number(high * b.min_low, min_low * b.min_low);
	case 6: return interval_number(min_low * b.high, high * (-b.min_low));
	case 4: return interval_number(min_low * b.high, min_low * b.min_low);
	case 5: return interval_number((-high) * b.high, min_low * b.min_low);
	};

	return interval_number(NAN);
}

inline interval_number interval_number::operator*(const double b) const 
{
	if (b >= 0) return interval_number(min_low * b, high * b);
	else return interval_number(high * (-b), min_low * (-b));
}

inline interval_number interval_number::operator/(const double b) const 
{
	if (b >= 0) return interval_number(min_low / b, high / b);
	else return interval_number(high / (-b), min_low / (-b));
}

inline interval_number interval_number::abs() const {
	if (min_low < 0) return *this;
	if (high < 0) return interval_number(high, min_low);
	return interval_number(0, std::max(high, min_low));
}

inline interval_number interval_number::sqr() const {
	if (min_low < 0) return interval_number(-min_low * min_low, high * high);
	if (high < 0) return interval_number(-high * high, min_low * min_low);
	if (min_low < high) return interval_number(0, high * high);
	return interval_number(0, min_low * min_low);
}

inline interval_number interval_number::inverse() const
{
	if ((min_low < 0 && high>0) || (min_low > 0 && high < 0))
	{
		return interval_number(-1.0 / high, -1.0 / min_low);
	}
	else
	{
		return interval_number(NAN);
	}
}

inline interval_number interval_number::pow(unsigned int e) const {

	const double _uinf = fabs(min_low), _usup = fabs(high);

	if (e & (unsigned int)1) { // If e is odd
		double _uml = min_low, _uh = high;
		while (--e) { _uml *= _uinf; _uh *= _usup; }
		return interval_number(_uml, _uh);
	}
	else { // e is even
		if (e == 0) return interval_number(1.0);

		if (_uinf > _usup) {
			double _uml = (min_low > 0 && high > 0) ? 0 : (-_usup);
			double _uh = _uinf;
			while (--e) { _uml *= _usup; _uh *= _uinf; }
			return interval_number(_uml, _uh);
		}
		else {
			double _uml = (min_low > 0 && high > 0) ? 0 : (-_uinf);
			double _uh = _usup;
			while (--e) { _uml *= _uinf; _uh *= _usup; }
			return interval_number(_uml, _uh);
		}
	}
}

#endif // USE_SIMD_INSTRUCTIONS

#ifdef USE_AVX2_INSTRUCTIONS
inline void vfast_Two_Sum(__m128d a, __m128d b, __m128d& x, __m128d& y) {
	x = _mm_add_sd(a, b);
	y = _mm_sub_sd(b, _mm_sub_sd(x, a));
}

inline void vtwo_Sum(__m128d a, __m128d b, __m128d& x, __m128d& y) {
	x = _mm_add_sd(a, b);
	__m128d vbv = _mm_sub_sd(x, a);
	y = _mm_add_sd(_mm_sub_sd(a, _mm_sub_sd(x, vbv)), _mm_sub_sd(b, vbv));
}

inline void vtwo_Prod(__m128d a, __m128d b, __m128d& x, __m128d& y) {
	x = _mm_mul_sd(a, b);
	y = _mm_fmsub_sd(a, b, x);
}
#endif

inline void expansionObject::Two_Prod(const double a, const double b, double& x, double& y)
{
#ifdef USE_AVX2_INSTRUCTIONS
	__m128d x1, x2, x3;
	x1 = _mm_set_sd(a);
	x2 = _mm_set_sd(b);
	vtwo_Prod(x1, x2, x3, x2);
	x = _mm_cvtsd_f64(x3);
	y = _mm_cvtsd_f64(x2);
#else
	double _ah, _al, _bh, _bl;
	x = a * b;
	Split(a, _ah, _al); Split(b, _bh, _bl);
	y = ((_ah * _bh - x) + _ah * _bl + _al * _bh) + _al * _bl;
#endif
}

inline void expansionObject::Square(const double a, double& x, double& y)
{
#ifdef USE_AVX2_INSTRUCTIONS
	__m128d x1, x2, x3;
	x1 = _mm_set_sd(a);
	vtwo_Prod(x1, x1, x3, x2);
	x = _mm_cvtsd_f64(x3);
	y = _mm_cvtsd_f64(x2);
#else
	double _ah, _al;
	x = a * a;
	Split(a, _ah, _al);
	y = (_al * _al) - ((x - (_ah * _ah)) - ((_ah + _ah) * _al));
#endif
}

inline void expansionObject::Two_One_Prod(const double a1, const double a0, const double b, double& x3, double& x2, double& x1, double& x0)
{
#ifdef USE_AVX2_INSTRUCTIONS
	double _i, _j, _k, _0;
	Two_Prod(a0, b, _i, x0); Two_Prod(a1, b, _j, _0);
	Two_Sum(_i, _0, _k, x1); Quick_Two_Sum(_j, _k, x3, x2);
#else
	double _bh, _bl, _i, _j, _0, _k;
	Split(b, _bh, _bl);
	Two_Prod_PreSplit(a0, b, _bh, _bl, _i, x0); Two_Prod_PreSplit(a1, b, _bh, _bl, _j, _0);
	Two_Sum(_i, _0, _k, x1); Quick_Two_Sum(_j, _k, x3, x2);
#endif
}

inline void expansionObject::Two_Two_Prod(const double a1, const double a0, const double b1, const double b0, double* h)
{
#ifdef USE_AVX2_INSTRUCTIONS
	double _m, _n, _i, _j, _k, _0, _1, _2, _l;
	Two_Prod(a0, b0, _i, h[0]);
	Two_Prod(a1, b0, _j, _0);
	Two_Sum(_i, _0, _k, _1);
	Quick_Two_Sum(_j, _k, _l, _2);
	Two_Prod(a0, b1, _i, _0);
	Two_Sum(_1, _0, _k, h[1]);
	Two_Sum(_2, _k, _j, _1);
	Two_Sum(_l, _j, _m, _2);
	Two_Prod(a1, b1, _j, _0);
	Two_Sum(_i, _0, _n, _0);
	Two_Sum(_1, _0, _i, h[2]);
	Two_Sum(_2, _i, _k, _1);
	Two_Sum(_m, _k, _l, _2);
	Two_Sum(_j, _n, _k, _0);
	Two_Sum(_1, _0, _j, h[3]);
	Two_Sum(_2, _j, _i, _1);
	Two_Sum(_l, _i, _m, _2);
	Two_Sum(_1, _k, _i, h[4]);
	Two_Sum(_2, _i, _k, h[5]);
	Two_Sum(_m, _k, h[7], h[6]);
#else
	double _ah, _al, _bh, _bl, _ch, _cl, _m, _n, _i, _j, _0, _1, _2, _k, _l;
	Split(a0, _ah, _al);
	Split(b0, _bh, _bl);
	Two_Product_2Presplit(a0, _ah, _al, b0, _bh, _bl, _i, h[0]);
	Split(a1, _ch, _cl);
	Two_Product_2Presplit(a1, _ch, _cl, b0, _bh, _bl, _j, _0);
	Two_Sum(_i, _0, _k, _1);
	Quick_Two_Sum(_j, _k, _l, _2);
	Split(b1, _bh, _bl);
	Two_Product_2Presplit(a0, _ah, _al, b1, _bh, _bl, _i, _0);
	Two_Sum(_1, _0, _k, h[1]);
	Two_Sum(_2, _k, _j, _1);
	Two_Sum(_l, _j, _m, _2);
	Two_Product_2Presplit(a1, _ch, _cl, b1, _bh, _bl, _j, _0);
	Two_Sum(_i, _0, _n, _0);
	Two_Sum(_1, _0, _i, h[2]);
	Two_Sum(_2, _i, _k, _1);
	Two_Sum(_m, _k, _l, _2);
	Two_Sum(_j, _n, _k, _0);
	Two_Sum(_1, _0, _j, h[3]);
	Two_Sum(_2, _j, _i, _1);
	Two_Sum(_l, _i, _m, _2);
	Two_Sum(_1, _k, _i, h[4]);
	Two_Sum(_2, _i, _k, h[5]);
	Two_Sum(_m, _k, h[7], h[6]);
#endif
}

inline int expansionObject::Gen_Sum(const int elen, const double* e, const int flen, const double* f, double* h)
{
	double Q, Qn, hh, s;
	const double* en = e, *fn = f, *elast = e+elen, *flast = f+flen;
	int h_k = 0;

	Q = (fabs(*fn) > fabs(*en)) ? (*en++) : (*fn++);

	if ((en < elast) && (fn < flast))
	{
		s = (fabs(*fn) > fabs(*en)) ? (*en++) : (*fn++);
		Quick_Two_Sum(s, Q, Qn, hh);
		Q = Qn;
		if (hh != 0.0) h[h_k++] = hh;
		while ((en < elast) && (fn < flast))
		{
			s = (fabs(*fn) > fabs(*en)) ? (*en++) : (*fn++);
			Two_Sum(s, Q, Qn, hh);
			Q = Qn;
			if (hh != 0.0) h[h_k++] = hh;
		}
	}

	while (en < elast)
	{
		Two_Sum(Q, (*en), Qn, hh);
		Q = Qn;
		if (hh != 0.0) h[h_k++] = hh;
		en++;
	}

	while (fn < flast)
	{
		Two_Sum(Q, (*fn), Qn, hh);
		Q = Qn;
		if (hh != 0.0) h[h_k++] = hh;
		fn++;
	}
	if ((Q != 0.0) || (h_k == 0)) h[h_k++] = Q;

	return h_k;
}

inline int expansionObject::Gen_Diff(const int elen, const double* e, const int flen, const double* f, double* h)
{
	double Q, Qn, hh, s;
	const double* en = e, * fn = f, * elast = e + elen, * flast = f + flen;
	int h_k = 0;

	Q = (fabs(*fn) > fabs(*en)) ? (*en++) : (-*fn++);

	if ((en < elast) && (fn < flast))
	{
		s = (fabs(*fn) > fabs(*en)) ? (*en++) : (-*fn++);
		Quick_Two_Sum(s, Q, Qn, hh);
		Q = Qn;
		if (hh != 0.0) h[h_k++] = hh;
		while ((en < elast) && (fn < flast))
		{
			s = (fabs(*fn) > fabs(*en)) ? (*en++) : (-*fn++);
			Two_Sum(s, Q, Qn, hh);
			Q = Qn;
			if (hh != 0.0) h[h_k++] = hh;
		}
	}

	while (en < elast)
	{
		Two_Sum(Q, (*en), Qn, hh);
		Q = Qn;
		if (hh != 0.0) h[h_k++] = hh;
		en++;
	}

	while (fn < flast)
	{
		s = *fn++;
		Two_Diff(Q, s, Qn, hh);
		Q = Qn;
		if (hh != 0.0) h[h_k++] = hh;
	}
	if ((Q != 0.0) || (h_k == 0)) h[h_k++] = Q;

	return h_k;
}


inline int expansionObject::Gen_Scale(const int elen, const double* e, const double b, double* h)
{
	double Q, sum, hh, pr1, pr0;
	const double* ei = e, * elast = e + elen;

	int k = 0;
	Two_Prod(*ei, b, Q, hh);
	if (hh != 0) h[k++] = hh;

	while (++ei < elast) {
		Two_Prod(*ei, b, pr1, pr0);
		Two_Sum(Q, pr0, sum, hh);
		if (hh != 0) h[k++] = hh;
		Quick_Two_Sum(pr1, sum, Q, hh);
		if (hh != 0) h[k++] = hh;
	}
	if ((Q != 0.0) || (k == 0)) h[k++] = Q;
	return k;
}


inline void expansionObject::Two_Square(const double& a1, const double& a0, double *x)
{
	double _j, _0, _k, _1, _l, _2;
	Square(a0, _j, x[0]);
	_0 = a0 + a0;
	Two_Prod(a1, _0, _k, _1);
	Two_One_Sum(_k, _1, _j, _l, _2, x[1]);
	Square(a1, _j, _1);
	Two_Two_Sum(_j, _1, _l, _2, x[5], x[4], x[3], x[2]);
}

inline int expansionObject::Sub_product(const int alen, const double *a, const int blen, const double *b, double *h)
{
	if (alen == 1) return Gen_Scale(blen, b, a[0], h);
	int partial = 2 * alen * blen;
	int allmem = 2 * (partial + blen);
	double ph1_p[1024];
	double* ph1 = (allmem > 1024) ? (AllocDoubles(allmem)) : (ph1_p);
	double *ph2 = ph1 + partial;
	double *th = ph2 + partial;
	double *ph[2] = { ph1, ph2 };
	int first = 0;
	int phl = Gen_Scale(blen, b, a[0], ph[0]);

	for (int i = 1; i < alen; i++)
	{
		int thl = Gen_Scale(blen, b, a[i], th);
		first = i & 1;
		phl = Gen_Sum(phl, ph[(i+1)&1], thl, th, ph[first]);
	}
	if (first) for (int i = 0; i < phl; i++) h[i] = ph2[i];
	else for (int i = 0; i < phl; i++) h[i] = ph1[i];
	if (allmem>1024) FreeDoubles(ph1);
	return phl;
}


inline int expansionObject::Gen_Product(const int alen, const double *a, const int blen, const double *b, double *h)
{
	if (blen == 1) return Gen_Scale(alen, a, b[0], h);
	else if (alen < blen) return Sub_product(alen, a, blen, b, h);
	else return Sub_product(blen, b, alen, a, h);
}


inline double expansionObject::To_Double(const int elen, const double *e)
{
	double Q = e[0];
	for (int e_i = 1; e_i < elen; e_i++) Q += e[e_i];
	return Q;
}

inline int expansionObject::Gen_Product_With_Alloc(const int alen, const double* a, const int blen, const double* b, double** h)
{
	int h_len = alen * blen * 2;
	if (h_len < 8) h_len = 8;
	*h = AllocDoubles(h_len);
	return Gen_Product(alen, a, blen, b, *h);
}

inline int expansionObject::Double_With_PreAlloc(const int elen, const double* e, double** h, const int hlen)
{
	int newlen = elen;
	if (hlen < newlen) *h = AllocDoubles(newlen);
	Double(elen, e, *h);
	return newlen;
}

inline int expansionObject::Gen_Scale_With_PreAlloc(const int elen, const double* e, const double& b, double** h, const int hlen)
{
	int newlen = elen * 2;
	if (hlen < newlen) *h = AllocDoubles(newlen);
	return Gen_Scale(elen, e, b, *h);
}

inline int expansionObject::Gen_Sum_With_PreAlloc(const int elen, const double* e, const int flen, const double* f, double** h, const int hlen)
{
	int newlen = elen + flen;
	if (hlen < newlen) *h = AllocDoubles(newlen);
	return Gen_Sum(elen, e, flen, f, *h);
}

inline int expansionObject::Gen_Diff_With_PreAlloc(const int elen, const double* e, const int flen, const double* f, double** h, const int hlen)
{
	int newlen = elen + flen;
	if (hlen < newlen) *h = AllocDoubles(newlen);
	return Gen_Diff(elen, e, flen, f, *h);
}

inline int expansionObject::Gen_Product_With_PreAlloc(const int alen, const double* a, const int blen, const double* b, double** h, const int hlen)
{
	int newlen = alen * blen * 2;
	if (hlen < newlen || hlen < 8)
	{
		if (newlen < 8) newlen = 8;
		*h = AllocDoubles(newlen);
	}
	return Gen_Product(alen, a, blen, b, *h);
}



#ifndef USE_GNU_GMP_CLASSES

inline void bignatural::init(const bignatural& m) {
	m_size = m.m_size;
	m_capacity = m.m_capacity;
	if (m_capacity) {
		digits = (uint32_t*)BN_ALLOC(sizeof(uint32_t) * m_capacity);
		memcpy(digits, m.digits, sizeof(uint32_t) * m_size);
	}
	else digits = NULL;
}

inline void bignatural::init(const uint64_t m) {
	if (m == 0) {
		m_size = m_capacity = 0;
		digits = NULL;
	}
	else if (m <= UINT32_MAX) {
		m_size = m_capacity = 1;
		digits = (uint32_t*)BN_ALLOC(sizeof(uint32_t));
		digits[0] = (uint32_t)m;
	}
	else {
		m_size = m_capacity = 2;
		digits = (uint32_t*)BN_ALLOC(sizeof(uint32_t) * 2);
		digits[0] = (uint32_t)(m >> 32);
		digits[1] = (uint32_t)(m & UINT32_MAX);
	}
}

inline bool bignatural::toUint64(uint64_t& n) const {
	if (m_size == 0) n = 0;
	else if (m_size == 1) n = digits[0];
	else if (m_size == 2) { n = (((uint64_t)digits[0]) << 32) + digits[1]; }
	else return false;

	return true;
}

inline bool bignatural::toUint32(uint32_t& n) const {
	if (m_size == 0) n = 0;
	else if (m_size == 1) n = digits[0];
	else return false;

	return true;
}

inline bignatural& bignatural::operator=(const bignatural& m) {
	if (digits != m.digits) {
		BN_FREE(digits);
		init(m);
	}
	
	return *this;
}

inline bignatural& bignatural::operator=(const uint64_t m) {
	BN_FREE(digits);
	init(m);
	return *this;
}

inline void bignatural::operator<<=(uint32_t n) {
	uint32_t s = n & 0x0000001f;
	uint32_t lz = countLeadingZeroes();
	if (lz < s) { // Need a further limb
		push_back(0);
		s = 32 - s;
		for (int i = (int)m_size - 1; i > 0; i--) {
			digits[i] >>= s;
			digits[i] |= (digits[i - 1] << (32 - s));
		}
		digits[0] >>= s;
	}
	else if (s) { // Leading zeroes are enough
		for (int i = 0; i < (int)m_size - 1; i++) {
			digits[i] <<= s;
			digits[i] |= (digits[i + 1] >> (32 - s));
		}
		back() <<= s;
	}

	while (n >= 32) {
		push_back(0);
		n -= 32;
	}
}

inline void bignatural::operator>>=(uint32_t n) {
	while (n >= 32) {
		pop_back();
		n -= 32;
	}
	if (!n) return;

	for (uint32_t i = m_size; i > 1; i--) {
		digits[i - 1] >>= n;
		digits[i - 1] |= (digits[i - 2] << (32 - n));
	}
	digits[0] >>= n;
	if (digits[0] == 0) pop_front();
}

inline bool bignatural::operator==(const bignatural& b) const {
	if (size() != b.size()) return false;
	for (uint32_t i = 0; i < size(); i++) if (digits[i] == b.digits[i]) return false;
	return true;
}

inline bool bignatural::operator!=(const bignatural& b) const {
	if (size() != b.size()) return true;
	for (uint32_t i = 0; i < size(); i++) if (digits[i] == b.digits[i]) return true;
	return false;
}

inline bool bignatural::operator>=(const bignatural& b) const {
	const int s = (size() > b.size()) - (size() < b.size());
	if (s) return (s > 0);
	uint32_t i;
	for (i = 0; i < size() && digits[i] == b.digits[i]; i++);
	return (i == size() || digits[i] > b.digits[i]);
}

inline bool bignatural::operator>(const bignatural& b) const {
	const int s = (size() > b.size()) - (size() < b.size());
	if (s) return (s > 0);
	uint32_t i;
	for (i = 0; i < size() && digits[i] == b.digits[i]; i++);
	return (i != size() && digits[i] > b.digits[i]);
}

inline bignatural bignatural::operator+(const bignatural& b) const {
	bignatural result;
	result.toSum(*this, b);
	return result;
}

// Assume that b is smaller than or equal to this number!
inline bignatural bignatural::operator-(const bignatural& b) const {
	bignatural result;
	result.toDiff(*this, b);
	return result;
}

inline bignatural bignatural::operator*(const bignatural& b) const {
	bignatural result;
	result.toProd(*this, b);
	return result;
}

// Short division algorithm
inline bignatural bignatural::divide_by(const uint32_t D, uint32_t& remainder) const {
	if (D == 0) ip_error("Division by zero\n");
	if (m_size == 0) return 0;

	// If both dividend fits into 64 bits, use hardware division
	uint64_t n;
	if (toUint64(n)) {
		remainder = n % D;
		return n / D;
	}

	bignatural Q;
	uint32_t next_digit = 0;
	uint64_t dividend = digits[next_digit++];
	for (;;) {
		uint64_t tmp_div = dividend / D;
		if (!Q.empty() || tmp_div) Q.push_back((uint32_t)tmp_div);
		dividend -= (tmp_div * D);
		if (next_digit < m_size) {
			dividend <<= 32;
			dividend += digits[next_digit++];
		}
		else break;
	}
	remainder = (uint32_t)dividend;

	return Q;
}

inline uint32_t bignatural::getNumSignificantBits() const {
	if (!m_size) return 0;
	int nsb = 31;
	while (!(digits[0] & (1 << nsb))) nsb--;
	nsb++;
	return nsb + (m_size - 1) * 32;
}

inline bool bignatural::getBit(uint32_t b) const {
	const uint32_t dig = (m_size - (b >> 5)) - 1;
	const uint32_t bit = b & 31;
	return (digits[dig] & (1 << bit));
}

// Long division
inline bignatural bignatural::divide_by(const bignatural& divisor, bignatural& remainder) const {
	if (divisor.empty()) ip_error("Division by zero\n");
	if (empty()) return 0;

	// If divisor fits into 32 bits, revert to short division
	uint32_t d32, rem;
	if (divisor.toUint32(d32)) {
		bignatural q = divide_by(d32, rem);
		remainder = rem;
		return q;
	}

	// If both dividend and divisor fit into 64 bits, use hardware division
	uint64_t n, d;
	if (toUint64(n) && divisor.toUint64(d)) {
		remainder = n % d;
		return n / d;
	}

	// If divisor is greater than dividend...
	if (divisor > *this) {
		remainder = *this;
		return 0;
	}

	// Use binary (per-bit) long division
	const bignatural& dividend = *this;

	bignatural quotient, loc_dividend;
	uint32_t next_dividend_bit = dividend.getNumSignificantBits();

	do {
		loc_dividend.push_bit_back(dividend.getBit(--next_dividend_bit));
		if (loc_dividend >= divisor) {
			loc_dividend = loc_dividend - divisor;
			quotient.push_bit_back(1);
		}
		else if (!quotient.empty()) quotient.push_bit_back(0);
	} while (next_dividend_bit);

	remainder = loc_dividend;

	return quotient;
}

// Greatest common divisor (Euclidean algorithm)
inline bignatural bignatural::GCD(const bignatural& D) const {
	bignatural A = *this;
	bignatural B = D;
	bignatural R;
	while (!A.empty() && !B.empty()) {
		A.divide_by(B, R);
		A = B;
		B = R;
	}
	if (A.empty()) return B;
	else return A;
}

// String representation in decimal form
inline std::string bignatural::get_dec_str() const {
	std::string st;
	bignatural N = *this;
	uint32_t R;
	if (N.empty()) return "0";
	while (!N.empty()) {
		N = N.divide_by(10, R);
		st += ('0' + R);
	}
	std::reverse(st.begin(), st.end());

	return st;
}

// String representation in binary form
inline std::string bignatural::get_str() const {
	std::string st;
	char s[33];
	s[32] = 0;
	for (uint32_t j = 0; j < m_size; j++) {
		for (int i = 0; i < 32; i++)
			s[i] = (digits[j] & (((uint32_t)1) << (31 - i))) ? '1' : '0';
		st += s;
	}
	return st;
}

// Count number of zeroes on the right (least significant binary digits)
inline uint32_t bignatural::countEndingZeroes() const {
	if (m_size == 0) return 0;
	uint32_t i = m_size - 1;
	uint32_t shft = 0;
	while (!digits[i]) {
		i--; shft += 32;
	}

	const uint32_t d = digits[i];
	uint32_t j = 31;
	while (!(d << j)) j--;
	return shft + 31 - j;

	//uint32_t s = UINT32_MAX;
	//uint32_t m = digits[i];
	//while ((s & m) == m) {
	//	s <<= 1;
	//	shft++;
	//}
	//return shft - 1;
}

inline uint32_t bignatural::countLeadingZeroes() const {
	uint32_t s = UINT32_MAX;
	const uint32_t m = digits[0];
	uint32_t shft = 0;
	while ((s & m) == m) {
		s >>= 1;
		shft++;
	}
	return shft - 1;
}

inline void bignatural::toSum(const bignatural& a, const bignatural& b) {
	if (a.m_size == 0) operator=(b);
	else if (b.m_size == 0) operator=(a);
	else {
		const uint32_t a_s = a.m_size;
		const uint32_t b_s = b.m_size;
		uint64_t carry = 0;
		uint32_t* dig_a = a.digits + a_s;
		uint32_t* dig_b = b.digits + b_s;

		if (a_s == b_s) {
			resize(a_s + 1);
			uint32_t* dig_r = digits + a_s + 1;
			do {
				const uint64_t da = *(--dig_a);
				const uint64_t db = *(--dig_b);
				const uint64_t sm = da + db + carry;
				*(--dig_r) = sm & UINT32_MAX;
				carry = (sm >> 32);
			} while (dig_a != a.digits);
		}
		else if (a_s < b_s) {
			resize(b_s + 1);
			uint32_t* dig_r = digits + b_s + 1;
			do {
				const uint64_t da = *(--dig_a);
				const uint64_t db = *(--dig_b);
				const uint64_t sm = da + db + carry;
				*(--dig_r) = sm & UINT32_MAX;
				carry = (sm >> 32);
			} while (dig_a != a.digits);
			do {
				const uint64_t db = *(--dig_b);
				const uint64_t sm = db + carry;
				*(--dig_r) = sm & UINT32_MAX;
				carry = (sm >> 32);
			} while (dig_b != b.digits);
		}
		else {
			resize(a_s + 1);
			uint32_t* dig_r = digits + a_s + 1;
			do {
				const uint64_t da = *(--dig_a);
				const uint64_t db = *(--dig_b);
				const uint64_t sm = da + db + carry;
				*(--dig_r) = sm & UINT32_MAX;
				carry = (sm >> 32);
			} while (dig_b != b.digits);
			do {
				const uint64_t da = *(--dig_a);
				const uint64_t sm = da + carry;
				*(--dig_r) = sm & UINT32_MAX;
				carry = (sm >> 32);
			} while (dig_a != a.digits);
		}
		if (carry) digits[0] = (uint32_t)carry;
		else {
			uint32_t* dold = digits + 1;
			uint32_t* dnew = digits;
			uint32_t* dend = digits + m_size;
			while (dold < dend) *dnew++ = *dold++;
			m_size --;
		}
	}
}

// a and b must NOT be this number!
// Assume that b is smaller or equal than a!
inline void bignatural::toDiff(const bignatural& a, const bignatural& b) {
	if (b.m_size == 0) operator=(a);
	else {
		const uint32_t a_s = a.m_size;
		const uint32_t b_s = b.m_size;
		uint32_t res_size = a_s;
		if (b_s > res_size) res_size = b_s;
		resize(res_size);

		uint64_t debt = 0;
		for (uint32_t i = 1; i <= res_size; i++) {
			const uint64_t da = (i <= a_s) ? (a.digits[(int)(a_s - i)]) : (0);
			const uint64_t db = ((i <= b_s) ? (b.digits[(int)(b_s - i)]) : (0)) + debt;
			debt = !(da >= db);
			if (debt) digits[(int)(res_size - i)] = (uint32_t)((da + (((uint64_t)1) << 32)) - db);
			else digits[(int)(res_size - i)] = (uint32_t)(da - db);
		}
		pack();
	}
}

// a and b must NOT be this number!
inline void bignatural::toProd(const bignatural& a, const bignatural& b) {
	if (a.empty()) operator=(a);
	else if (b.empty()) operator=(b);
	else {
		resize(a.m_size + b.m_size);
		fill(0);

		uint32_t ls = 0;
		for (uint32_t i = b.m_size; i > 0; i--)
			a.addmul(b[(int)(i - 1)], ls++, *this);

		pack();
	}
}

inline void bignatural::addmul(uint32_t b, uint32_t left_shifts, bignatural& result) const {
	uint64_t carry = 0;
	int d = (int)(result.m_size - m_size - left_shifts);
	for (uint32_t i = m_size; i > 0; i--) {
		uint64_t pm = ((uint64_t)digits[(int)(i - 1)]) * b + carry + result[(int)i + d - 1];
		result[(int)i + d - 1] = (uint32_t)pm;
		carry = pm >> 32;
	}
	result[d - 1] = (uint32_t)carry;
}

inline void bignatural::increaseCapacity(uint32_t new_capacity) {
	m_capacity = new_capacity;
	uint32_t *tmp_d = (uint32_t*)BN_ALLOC(sizeof(uint32_t) * m_capacity);
	memcpy(tmp_d, digits, sizeof(uint32_t) * m_size);
	BN_FREE(digits);
	digits = tmp_d;
}

inline bigfloat::bigfloat(const double d) {
	sign = (d > 0) - (d < 0);

	if (sign) {
		uint64_t dn = *((uint64_t*)(&d));
		const uint64_t m = (dn & 0x000fffffffffffff) + 0x0010000000000000;
		mantissa.push_back(m >> 32);
		mantissa.push_back(m & 0x00000000ffffffff);
		dn <<= 1;
		dn >>= 53;
		exponent = ((int32_t)dn) - 1075; // Exp

		pack();
	}
	else exponent = 0;
}

inline double bigfloat::get_d() const {
	uint64_t dn = 0;
	if (mantissa.empty()) return 0.0;

	uint64_t m;
	int32_t e;
	uint32_t shft;

	if (mantissa.size() == 1) {
		m = ((uint64_t)mantissa[0]);
		shft = mantissa.countLeadingZeroes() + 21;
		m <<= shft;
		e = exponent - shft;
	}
	else {
		m = (((uint64_t)mantissa[0]) << 32) | ((uint64_t)mantissa[1]);
		e = exponent + 32 * ((uint32_t)mantissa.size() - 2);
		shft = mantissa.countLeadingZeroes();

		if (shft < 11) {
			m >>= (11 - shft);
			e += (11 - shft);
		}
		if (shft > 11) {
			m <<= (shft - 11);
			e -= (shft - 11);
			if (mantissa.size() > 2) m |= (mantissa[2] >> (43 - shft));
		}
	}
	m &= (~0x0010000000000000); // Remove implicit digit
	e += 52;

	if (e < (-1022)) return 0.0;
	if (e > 1023) return sign * INFINITY;

	if (sign < 0) dn |= 0x8000000000000000; // Set sign
	dn |= (((uint64_t)(e + 1023)) << 52); // Set exponent
	dn |= m; // Set mantissa

	return *((double*)(&dn));
}

inline bigfloat bigfloat::operator+(const bigfloat& b) const {
	if (mantissa.empty()) return b;
	if (b.mantissa.empty()) return *this;

	if (exponent == b.exponent) {
		bigfloat result;

		if (sign == b.sign) {
			result.mantissa.toSum(mantissa, b.mantissa);
			result.sign = sign;
		}
		else if (b.mantissa >= mantissa) {
			result.mantissa.toDiff(b.mantissa, mantissa);
			result.sign = b.sign;
		}
		else {
			result.mantissa.toDiff(mantissa, b.mantissa);
			result.sign = sign;
		}

		result.exponent = exponent;
		result.pack();
		return result;
	}
	else if (exponent > b.exponent) {
		bigfloat op(*this);
		op.leftShift(exponent - b.exponent);
		return op + b;
	}
	else { // exponent < b.exponent
		bigfloat op(b);
		op.leftShift(b.exponent - exponent);
		return op + *this;
	}
}

inline bigfloat bigfloat::operator-(const bigfloat& b) const {
	if (mantissa.empty()) return b.inverse();
	if (b.mantissa.empty()) return *this;

	if (exponent == b.exponent) {
		bigfloat result;

		if (sign != b.sign) {
			result.mantissa.toSum(mantissa, b.mantissa);
			result.sign = sign;
		}
		else if (b.mantissa >= mantissa) {
			result.mantissa.toDiff(b.mantissa, mantissa);
			result.sign = -sign;
		}
		else {
			result.mantissa.toDiff(mantissa, b.mantissa);
			result.sign = sign;
		}

		result.exponent = exponent;
		result.pack();
		return result;
	}
	else if (exponent > b.exponent) {
		bigfloat op(*this);
		op.leftShift(exponent - b.exponent);
		return op - b;
	}
	else { // exponent < b.exponent
		bigfloat op(b);
		op.leftShift(b.exponent - exponent);
		return *this - op;
	}
}


inline bigfloat bigfloat::operator*(const bigfloat& b) const {
	if (mantissa.empty() || b.mantissa.empty()) return 0;

	// Left-shift operator with highest exponent
	if (exponent == b.exponent) {
		bigfloat result;
		result.mantissa.toProd(mantissa, b.mantissa);
		result.exponent = exponent;
		result.sign = sign * b.sign;
		result.leftShift(result.exponent - exponent);
		result.exponent *= 2;

		result.pack();
		return result;
	}
	else if (exponent > b.exponent) {
		bigfloat op(*this);
		op.leftShift(exponent - b.exponent);
		return op * b;
	} // exponent < b.exponent
	else {
		bigfloat op(b);
		op.leftShift(b.exponent - exponent);
		return op * *this;
	}
}

inline std::string bigfloat::get_str() const {
	std::string s;
	if (sign == 0) s += "0";
	if (sign < 0) s += "-";
	s += mantissa.get_str();
	s += " * 2^";
	s += std::to_string(exponent);
	return s;
}

inline void bigfloat::pack() {
	if (mantissa.empty()) {
		sign = exponent = 0;
		return;
	}

	while (mantissa.back() == 0) {
		mantissa.pop_back();
		exponent += 32;
	}

	const uint32_t s = mantissa.countEndingZeroesLSL();
	if (s) {
		for (int i = (int)mantissa.size() - 1; i > 0; i--) {
			mantissa[i] >>= s;
			mantissa[i] |= (mantissa[i - 1] << (32 - s));
		}
		mantissa[0] >>= s;
		exponent += s;
	}

	mantissa.pack();
}

inline bigrational::bigrational(const bigfloat& f) {
	if (f.sgn() == 0) sign = 0;
	else {
		sign = f.sgn();
		numerator = f.getMantissa();
		denominator = 1;
		int32_t e = f.getExponent();
		if (e >= 0) numerator <<= e;
		else denominator <<= (-e);
	}
}

inline void bigrational::compress() {
	const uint32_t nez = numerator.countEndingZeroes();
	const uint32_t dez = denominator.countEndingZeroes();
	const uint32_t s = std::min(nez, dez);
	numerator >>= s;
	denominator >>= s;
}

inline void bigrational::canonicalize() {
	if (sign) {
		if (numerator.empty()) {
			numerator = denominator = 0;
			sign = 0;
		}
		else {
			compress();
			bignatural r;
			const bignatural gcd = numerator.GCD(denominator);
			numerator = numerator.divide_by(gcd, r);
			denominator = denominator.divide_by(gcd, r);
		}
	}
}

inline bigrational bigrational::operator+(const bigrational& r) const {
	if (sign == 0) return r;
	else if (r.sign == 0) return *this;
	else {
		//bignatural rm;
		//const bignatural gcd = denominator.GCD(r.denominator);
		//const bignatural den3 = (denominator * r.denominator).divide_by(gcd, rm);
		//const bignatural left_den = den3.divide_by(denominator, rm);
		//const bignatural right_den = den3.divide_by(r.denominator, rm);
		//const bignatural left_num = numerator * left_den;
		//const bignatural right_num = r.numerator * right_den;
		//if (sign > 0 && r.sign > 0)	return bigrational(left_num + right_num, den3, 1);
		//else if (sign < 0 && r.sign < 0) return bigrational(left_num + right_num, den3, -1);
		//else if (sign > 0 && r.sign < 0) {
		//	if (left_num >= right_num) return bigrational(left_num - right_num, den3, 1);
		//	else return bigrational(right_num - left_num, den3, -1);
		//}
		//else { // if (sign < 0 && r.sign > 0)
		//	if (left_num >= right_num) return bigrational(left_num - right_num, den3, -1);
		//	else return bigrational(right_num - left_num, den3, 1);
		//}

		const bignatural left_num = numerator * r.denominator;
		const bignatural right_num = r.numerator * denominator;
		if (sign > 0 && r.sign > 0)	return bigrational(left_num + right_num, denominator * r.denominator, 1);
		else if (sign < 0 && r.sign < 0) return bigrational(left_num + right_num, denominator * r.denominator, -1);
		else if (sign > 0 && r.sign < 0) {
			if (left_num >= right_num) return bigrational(left_num - right_num, denominator * r.denominator, 1);
			else return bigrational(right_num - left_num, denominator * r.denominator, -1);
		}
		else { // if (sign < 0 && r.sign > 0)
			if (left_num >= right_num) return bigrational(left_num - right_num, denominator * r.denominator, -1);
			else return bigrational(right_num - left_num, denominator * r.denominator, 1);
		}
	}
}

inline double bigrational::get_d() const {
	if (sign == 0) return 0.0;
	bignatural num = numerator;
	bignatural den = denominator;
	int32_t E = (int32_t)num.getNumSignificantBits() - (int32_t)den.getNumSignificantBits();
	if (E > 0) { den <<= E; if (den > num) { E--; den >>= 1; } }
	else if (E <= 0) { num <<= -E; if (den > num) { E--; num <<= 1; } }

	if (E > 1023) return INFINITY;
	else if (E < -1022) return -INFINITY;

	uint64_t signbit = sign < 0 ? ((uint64_t)1) << 63 : 0;
	uint64_t exponent = ((uint64_t)(1023 + E)) << 52;
	uint64_t mantissa = 0;

	for (int i = 0; i < 53; i++) {
		mantissa <<= 1;
		if (num >= den) {
			mantissa |= 1;
			num = num - den;
		}
		if (!num.empty()) num <<= 1;
	}
	mantissa &= (~(((uint64_t)1) << 52));
	mantissa |= exponent;
	mantissa |= signbit;

	void* ptr = &mantissa;

	return *((double*)ptr);
}

inline std::string bigrational::get_dec_str() const {
	std::string st;
	if (sign < 0) st = "-";
	else if (sign == 0) return "0";
	st += numerator.get_dec_str();
	const std::string dens = denominator.get_dec_str();
	if (dens != "1") {
		st += "/";
		st += dens;
	}
	return st;
}

inline std::string bigrational::get_str() const {
	std::string st;
	if (sign < 0) st = "-";
	else if (sign == 0) return "0";
	st += numerator.get_str();
	st += "/";
	st += denominator.get_str();
	return st;
}

#endif // USE_GNU_GMP_CLASSES
