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

#ifndef NUMERICS_H
#define NUMERICS_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <float.h>
#include <math.h>
#include <fenv.h>
#include <iostream>
#include <climits>
#include "memPool.h"

#ifdef USE_GNU_GMP_CLASSES
#include <gmpxx.h>
#endif

// Call the following function (once per thread) before using these number types
void initFPU();

inline void ip_error(const char* msg)
{
	std::cerr << msg;
	exit(0);
}

#if INTPTR_MAX == INT64_MAX
#define	IS64BITPLATFORM
#endif

#ifdef _MSC_VER
#define	ISVISUALSTUDIO
#endif

#ifdef IS64BITPLATFORM
#ifdef __SSE2__
#define USE_SIMD_INSTRUCTIONS
#endif
#ifdef __AVX2__
#define USE_SIMD_INSTRUCTIONS
#define USE_AVX2_INSTRUCTIONS
#endif
#endif

#ifdef ISVISUALSTUDIO

#pragma fenv_access (on)

inline void setFPUModeToRoundUP() { _controlfp(_RC_UP, _MCW_RC); }
inline void setFPUModeToRoundNEAR() { _controlfp(_RC_NEAR, _MCW_RC); }

#else

#ifdef __AVX2__
#pragma GCC target("fma")
#endif

#pragma STDC FENV_ACCESS ON

inline void setFPUModeToRoundUP() { fesetround(FE_UPWARD); }
inline void setFPUModeToRoundNEAR() { fesetround(FE_TONEAREST); }
#endif

#ifdef USE_SIMD_INSTRUCTIONS

#ifdef USE_AVX2_INSTRUCTIONS
#include <immintrin.h>
#else
#include <emmintrin.h>
#endif

#endif

	/////////////////////////////////////////////////////////////////////
	// 	   
	// 	   I N T E R V A L   A R I T H M E T I C
	// 
	/////////////////////////////////////////////////////////////////////

	// An interval_number is a pair of doubles representing an interval.
	// Operations on interval_number require that the rounding mode is
	// set to +INFINITY. Use setFPUModeToRoundUP().

	class interval_number
	{
#ifdef USE_SIMD_INSTRUCTIONS
		__m128d interval; // interval[1] = min_low, interval[0] = high

		static inline __m128d zero() { return _mm_setzero_pd(); }
		static inline __m128d minus_one() { return _mm_set1_pd(-1.0); }
		static inline __m128d sign_low_mask() { return _mm_castsi128_pd(_mm_set_epi64x(LLONG_MIN, 0)); }
		static inline __m128d sign_high_mask() { return _mm_castsi128_pd(_mm_set_epi64x(0, LLONG_MIN)); }
		static inline __m128d sign_fabs_mask() { return _mm_castsi128_pd(_mm_set_epi64x(~LLONG_MIN, ~LLONG_MIN)); }
		static inline __m128d all_high_mask() { return _mm_castsi128_pd(_mm_set_epi64x(0, -1LL)); }

		__m128d getLowSwitched() const { return _mm_xor_pd(interval, sign_low_mask()); }

	public:
		const double *getInterval() const { return (const double*)&interval; }

		interval_number() { }
		interval_number(const double a) : interval(_mm_set_pd(-a, a)) {}
		interval_number(const double minf, const double sup) : interval(_mm_set_pd(minf, sup)) {}
		interval_number(const __m128d& i) : interval(i) {}
		interval_number(const interval_number& b) : interval(b.interval) {}

		double minus_inf() const { return _mm_cvtsd_f64(_mm_shuffle_pd(interval, interval, 1));	}
		double inf() const { return -minus_inf(); }
		double sup() const { return _mm_cvtsd_f64(interval); }

		interval_number& operator=(const interval_number& b) { interval = b.interval; return *this; }

		interval_number operator+(const interval_number& b) const { return interval_number(_mm_add_pd(interval, b.interval)); }

		interval_number operator-(const interval_number& b) const { return interval_number(_mm_add_pd(interval, _mm_shuffle_pd(b.interval, b.interval, 1))); }

		interval_number operator*(const interval_number& b) const;

		interval_number operator-() const { return interval_number(_mm_shuffle_pd(interval, interval, 1)); }
		interval_number operator+(const double b) const { return interval_number(_mm_add_pd(interval, _mm_set_pd(-b, b))); }
		interval_number operator-(const double b) const { return interval_number(_mm_sub_pd(interval, _mm_set_pd(-b, b))); }
		interval_number operator*(const double b) const;
		interval_number operator/(const double b) const;
		interval_number& operator+=(const interval_number& b) { return operator=(*this + b); }
		interval_number& operator-=(const interval_number& b) { return operator=(*this - b); }
		interval_number& operator*=(const interval_number& b) { return operator=(*this * b); }
		interval_number& operator+=(const double b) { return operator=(*this + b); }
		interval_number& operator-=(const double b) { return operator=(*this - b); }
		interval_number& operator*=(const double b) { return operator=(*this * b); }
		interval_number& operator/=(const double b) { return operator=(*this / b); }

		interval_number abs() const;

		interval_number sqr() const;

		interval_number pow(unsigned int e) const;

		friend inline interval_number min(const interval_number& a, const interval_number& b);
		friend inline interval_number max(const interval_number& a, const interval_number& b);

		bool operator<(const double b) const { return sup() < b; }
		bool operator<=(const double b) const { return sup() <= b; }
		bool operator>(const double b) const { return inf() > b; }
		bool operator>=(const double b) const { return inf() >= b; }
		bool operator==(const double b) const { return sup()==inf() && sup()==b; }

		void negate() { interval = _mm_shuffle_pd(interval, interval, 1); }

		bool isNegative() const { return _mm_comilt_sd(interval, zero()); }
		bool isPositive() const { return _mm_comilt_sd(_mm_shuffle_pd(interval, interval, 1), zero()); }

#ifdef USE_AVX2_INSTRUCTIONS
		int sign() const {
			__m128d m = _mm_cmplt_sd(interval, zero());
			__m128i r = _mm_castpd_si128(_mm_blendv_pd(_mm_castsi128_pd(_mm_set1_epi32(1)), _mm_castsi128_pd(_mm_set1_epi32(-1)), m));
			return _mm_cvtsi128_si32(r);
		} // Zero is not accounted for

		//int sign() const {
		//	__m128d mp = _mm_cmplt_sd(_mm_shuffle_pd(interval, interval, 1), zero());
		//	__m128d rp = _mm_blendv_pd(zero(), _mm_castsi128_pd(_mm_set1_epi32(1)), mp);
		//	__m128d m = _mm_cmplt_sd(interval, zero());
		//	__m128i r = _mm_castpd_si128(_mm_blendv_pd(rp, _mm_castsi128_pd(_mm_set1_epi32(-1)), m));
		//	return _mm_cvtsi128_si32(r);
		//} // Zero is accounted for
#else
		int sign() const { return (isNegative()) ? (-1) : (1); } // Zero is not accounted for
#endif

#else // USE_SIMD_INSTRUCTIONS
		typedef union error_approx_type_t
		{
			double d;
			uint64_t u;

			inline error_approx_type_t() {}
			inline error_approx_type_t(double a) : d(a) {}
			inline uint64_t is_negative() const { return u >> 63; }
		} casted_double;

	public:
		double min_low, high;

		const double* getInterval() const { return (const double*)&min_low; }

		interval_number() { }
		interval_number(double a) : min_low(-a), high(a) {}
		interval_number(double minf, double sup) : min_low(minf), high(sup) {}
		interval_number(const interval_number& b) : min_low(b.min_low), high(b.high) {}

		double inf() const { return -min_low; }
		double sup() const { return high; }

		bool isNegative() const { return (high < 0); }
		bool isPositive() const { return (min_low < 0); }
		void negate() { std::swap(min_low, high); }

		bool operator<(const double b) const { return (high < b); }

		interval_number& operator=(const interval_number& b) { min_low = b.min_low; high = b.high; return *this; }

		interval_number operator+(const interval_number& b) const { return interval_number(min_low + b.min_low, high + b.high); }

		interval_number operator-(const interval_number& b) const { return interval_number(b.high + min_low, high + b.min_low); }

		interval_number operator*(const interval_number& b) const;

		interval_number operator-() const { return interval_number(high, min_low); }
		interval_number operator+(const double b) const { return interval_number(min_low - b, high + b); }
		interval_number operator-(const double b) const { return interval_number(min_low + b, high - b); }
		interval_number operator*(const double b) const;
		interval_number operator/(const double b) const;

		interval_number& operator+=(const interval_number& b) { min_low += b.min_low; high += b.high; return *this; }
		interval_number& operator-=(const interval_number& b) { return operator=(*this - b); }
		interval_number& operator*=(const interval_number& b) { return operator=(*this * b); }
		interval_number& operator+=(const double b) { min_low -= b; high += b; return *this; }
		interval_number& operator-=(const double b) { min_low += b; high -= b; return *this; }
		interval_number& operator*=(const double b) { return operator=(*this * b); }
		interval_number& operator/=(const double b) { return operator=(*this / b); }

		interval_number abs() const;

		interval_number sqr() const;

		interval_number pow(unsigned int e) const;

		friend inline interval_number min(const interval_number& a, const interval_number& b) {
			return interval_number(std::max(a.min_low, b.min_low), std::min(a.high, b.high));
		}

		friend inline interval_number max(const interval_number& a, const interval_number& b) {
			return interval_number(std::min(a.min_low, b.min_low), std::max(a.high, b.high));
		}

		bool operator>(const double b) const { return (min_low < -b); }
		bool operator==(const double b) const { return (high == b && min_low == -b); }

		int sign() const { return (isNegative()) ? (-1) : (1); } // Zero is not accounted for

#endif // USE_SIMD_INSTRUCTIONS
		double width() const { return sup() - inf(); }

		bool signIsReliable() const { return (isNegative() || isPositive()); } // Zero is not accounted for
		bool containsZero() const { return !signIsReliable(); }

		bool isNAN() const { return sup() != sup(); }

		inline double getMid() const { return (inf() + sup()) / 2; }
		inline bool isExact() const { return inf() == sup(); }

		//inline void operator+=(const interval_number& b) { *this = operator+(b); }

		// Can be TRUE only if the intervals are disjoint
		inline bool operator<(const interval_number& b) const { return (sup() < b.inf()); }
		inline bool operator>(const interval_number& b) const { return (inf() > b.sup()); }

		// Can be TRUE only if the interval interiors are disjoint
		inline bool operator<=(const interval_number& b) const { return (sup() <= b.inf()); }
		inline bool operator>=(const interval_number& b) const { return (inf() >= b.sup()); }

		// TRUE if the intervals are identical single values
		inline bool operator==(const interval_number& b) const { return (sup() == inf() && sup() == b.inf() && sup() == b.sup()); }

		// TRUE if the intervals have no common values
		inline bool operator!=(const interval_number& b) const { return operator<(b) || operator>(b); }

		// The inverse of an interval. Returns NAN if the interval contains zero
		interval_number inverse() const;
	};


	// The square root of an interval
	// Returns NAN if the interval contains a negative value
	inline interval_number sqrt(const interval_number& p)
	{
		const double inf = p.inf();
		const double sup = p.sup();
		if (inf < 0 || sup < 0) return interval_number(NAN);
		const double srinf = sqrt(inf);
		const double srsup = sqrt(sup);
		if (srinf * srinf > inf) return interval_number((-nextafter(srinf, 0)), srsup);
		else return interval_number(-srinf, srsup);
	}

	inline std::ostream& operator<<(std::ostream& os, const interval_number& p)
	{
		os << "[ " << p.inf() << ", " << p.sup() << " ]";
		return os;
	}


	/////////////////////////////////////////////////////////////////////
	// 	   
	// 	   E X P A N S I O N   A R I T H M E T I C
	// 
	/////////////////////////////////////////////////////////////////////

	// Allocate extra-memory
	//#define AllocDoubles(n) ((double *)malloc((n) * sizeof(double)))
	//#define FreeDoubles(p) (free(p))
	#define AllocDoubles(n) ((double *)expansionObject::mempool.alloc((n) * sizeof(double)))
	#define FreeDoubles(p) (expansionObject::mempool.release(p))

	// An instance of the following must be created to access functions for expansion arithmetic
	class expansionObject
	{
	public:
		inline static thread_local MultiPool mempool = MultiPool(2048, 64);

		static void Quick_Two_Sum(const double a, const double b, double& x, double& y) { x = a + b; y = b - (x - a); }

		static void Two_Sum(const double a, const double b, double& x, double& y) {
			double _bv;
			x = a + b; _bv = x - a; y = (a - (x - _bv)) + (b - _bv); 
		}

		static void Two_One_Sum(const double a1, const double a0, const double b, double& x2, double& x1, double& x0) {
			double _i;
			Two_Sum(a0, b, _i, x0); Two_Sum(a1, _i, x2, x1);
		}

		static void two_Sum(const double a, const double b, double* xy) { Two_Sum(a, b, xy[1], xy[0]); }

		static void Two_Diff(const double a, const double b, double& x, double& y) {
			double _bv;
			x = a - b; _bv = a - x; y = (a - (x + _bv)) + (_bv - b); 
		}

		static void Two_One_Diff(const double a1, const double a0, const double b, double& x2, double& x1, double& x0) {
			double _i;
			Two_Diff(a0, b, _i, x0); Two_Sum(a1, _i, x2, x1);
		}

		static void two_Diff(const double a, const double b, double* xy) { Two_Diff(a, b, xy[1], xy[0]); }

		// Products
#ifndef USE_AVX2_INSTRUCTIONS 
		static void Split(double a, double& _ah, double& _al) {
			double _c = 1.3421772800000003e+008 * a;
			_ah = _c - (_c - a); _al = a - _ah;
		}
			
		static void Two_Prod_PreSplit(double a, double b, double _bh, double _bl, double& x, double& y) {
			double _ah, _al;
			x = a * b; 
			Split(a, _ah, _al); 
			y = (_al * _bl) - (((x - (_ah * _bh)) - (_al * _bh)) - (_ah * _bl));
		}
		
		static void Two_Product_2Presplit(double a, double _ah, double _al, double b, double _bh, double _bl, double& x, double& y) {
			x = a * b; y = (_al * _bl) - (((x - _ah * _bh) - (_al * _bh)) - (_ah * _bl));
		}
#endif

		// [x,y] = [a]*[b]		 Multiplies two expansions [a] and [b] of length one
		static void Two_Prod(const double a, const double b, double& x, double& y);
		static void Two_Prod(const double a, const double b, double* xy) { Two_Prod(a, b, xy[1], xy[0]); }


		// [x,y] = [a]^2		Squares an expansion of length one
		static void Square(const double a, double& x, double& y);
		static void Square(const double a, double* xy) { Square(a, xy[1], xy[0]); }

		// [x2,x1,x0] = [a1,a0]-[b]		Subtracts an expansion [b] of length one from an expansion [a1,a0] of length two
		static void two_One_Diff(const double a1, const double a0, const double b, double& x2, double& x1, double& x0)
		 { Two_One_Diff(a1, a0, b, x2, x1, x0); }

		static void two_One_Diff(const double* a, const double b, double* x) { two_One_Diff(a[1], a[0], b, x[2], x[1], x[0]); }

		// [x3,x2,x1,x0] = [a1,a0]*[b]		Multiplies an expansion [a1,a0] of length two by an expansion [b] of length one
		static void Two_One_Prod(const double a1, const double a0, const double b, double& x3, double& x2, double& x1, double& x0);
		static void Two_One_Prod(const double* a, const double b, double* x) { Two_One_Prod(a[1], a[0], b, x[3], x[2], x[1], x[0]); }

		// [x3,x2,x1,x0] = [a1,a0]+[b1,b0]		Calculates the sum of two expansions of length two
		static void Two_Two_Sum(const double a1, const double a0, const double b1, const double b0, double& x3, double& x2, double& x1, double& x0) {
			double _j, _0;
			Two_One_Sum(a1, a0, b0, _j, _0, x0); Two_One_Sum(_j, _0, b1, x3, x2, x1);	
		}
		
		static void Two_Two_Sum(const double* a, const double* b, double* xy) { Two_Two_Sum(a[1], a[0], b[1], b[0], xy[3], xy[2], xy[1], xy[0]); }

		// [x3,x2,x1,x0] = [a1,a0]-[b1,b0]		Calculates the difference between two expansions of length two
		static void Two_Two_Diff(const double a1, const double a0, const double b1, const double b0, double& x3, double& x2, double& x1, double& x0) {
			double _j, _0, _u3;
			Two_One_Diff(a1, a0, b0, _j, _0, x0); Two_One_Diff(_j, _0, b1, _u3, x2, x1); x3 = _u3; 
		}

		static void Two_Two_Diff(const double* a, const double* b, double* x) { Two_Two_Diff(a[1], a[0], b[1], b[0], x[3], x[2], x[1], x[0]); }

		// Calculates the second component 'y' of the expansion [x,y] = [a]-[b] when 'x' is known
		static void Two_Diff_Back(const double a, const double b, double& x, double& y) { 
			double _bv;
			_bv = a - x; y = (a - (x + _bv)) + (_bv - b); 
		}

		static void Two_Diff_Back(const double a, const double b, double* xy) { Two_Diff_Back(a, b, xy[1], xy[0]); }

		// [h] = [a1,a0]^2		Squares an expansion of length 2
		// 'h' must be allocated by the caller with 6 components.
		static void Two_Square(const double& a1, const double& a0, double* x);

		// [h7,h6,...,h0] = [a1,a0]*[b1,b0]		Calculates the product of two expansions of length two.
		// 'h' must be allocated by the caller with eight components.
		static void Two_Two_Prod(const double a1, const double a0, const double b1, const double b0, double* h);
		static void Two_Two_Prod(const double* a, const double* b, double* xy) { Two_Two_Prod(a[1], a[0], b[1], b[0], xy); }

		// [e] = -[e]		Inplace inversion
		static void Gen_Invert(const int elen, double* e) { for (int i = 0; i < elen; i++) e[i] = -e[i]; }

		// [h] = [e] + [f]		Sums two expansions and returns number of components of result
		// 'h' must be allocated by the caller with at least elen+flen components.
		static int Gen_Sum(const int elen, const double* e, const int flen, const double* f, double* h);

		// Same as above, but 'h' is allocated internally. The caller must still call 'free' to release the memory.
		static int Gen_Sum_With_Alloc(const int elen, const double* e, const int flen, const double* f, double** h)
		{
			*h = AllocDoubles(elen + flen);
			return Gen_Sum(elen, e, flen, f, *h);
		}

		// [h] = [e] + [f]		Subtracts two expansions and returns number of components of result
		// 'h' must be allocated by the caller with at least elen+flen components.
		static int Gen_Diff(const int elen, const double* e, const int flen, const double* f, double* h);

		// Same as above, but 'h' is allocated internally. The caller must still call 'free' to release the memory.
		static int Gen_Diff_With_Alloc(const int elen, const double* e, const int flen, const double* f, double** h)
		{
			*h = AllocDoubles(elen + flen);
			return Gen_Diff(elen, e, flen, f, *h);
		}

		// [h] = [e] * b		Multiplies an expansion by a scalar
		// 'h' must be allocated by the caller with at least elen*2 components.
		static int Gen_Scale(const int elen, const double* e, const double b, double* h);

		// [h] = [e] * 2		Multiplies an expansion by 2
		// 'h' must be allocated by the caller with at least elen components. This is exact up to overflows.
		static void Double(const int elen, const double* e, double* h) { for (int i = 0; i < elen; i++) h[i] = 2 * e[i]; }
		
		// [h] = [e] * n		Multiplies an expansion by n
		// If 'n' is a power of two, the multiplication is exact
		static void ExactScale(const int elen, double* e, const double n) { for (int i = 0; i < elen; i++) e[i] *= n; }

		// [h] = [a] * [b]
		// 'h' must be allocated by the caller with at least 2*alen*blen components.
		static int Sub_product(const int alen, const double* a, const int blen, const double* b, double* h);

		// [h] = [a] * [b]
		// 'h' must be allocated by the caller with at least MAX(2*alen*blen, 8) components.
		static int Gen_Product(const int alen, const double* a, const int blen, const double* b, double* h);

		// Same as above, but 'h' is allocated internally. The caller must still call 'free' to release the memory.
		static int Gen_Product_With_Alloc(const int alen, const double* a, const int blen, const double* b, double** h);


		// Assume that *h is pre-allocated with hlen doubles.
		// If more elements are required, *h is re-allocated internally.
		// In any case, the function returns the size of the resulting expansion.
		// The caller must verify whether reallocation took place, and possibly call 'free' to release the memory.
		// When reallocation takes place, *h becomes different from its original value.

		static int Double_With_PreAlloc(const int elen, const double* e, double** h, const int hlen);

		static int Gen_Scale_With_PreAlloc(const int elen, const double* e, const double& b, double** h, const int hlen);

		static int Gen_Sum_With_PreAlloc(const int elen, const double* e, const int flen, const double* f, double** h, const int hlen);

		static int Gen_Diff_With_PreAlloc(const int elen, const double* e, const int flen, const double* f, double** h, const int hlen);

		static int Gen_Product_With_PreAlloc(const int alen, const double* a, const int blen, const double* b, double** h, const int hlen);

		// Approximates the expansion to a double
		static double To_Double(const int elen, const double* e);

		static void print(const int elen, const double* e) { for (int i = 0; i < elen; i++) printf("%e ", e[i]); printf("\n");}
	};

#ifdef USE_GNU_GMP_CLASSES
	typedef mpz_class bignatural;
	typedef mpq_class bigfloat;
	typedef mpq_class bigrational;
#else
	/////////////////////////////////////////////////////////////////////
	// 	   
	// 	   B I G   N A T U R A L
	// 
	/////////////////////////////////////////////////////////////////////

	// Preallocates memory for bignaturals having at most 32 limbs.
	// Larger numbers will use the standard heap.
	inline static thread_local MultiPool nfgMemoryPool;

	// A bignatural is an arbitrarily large non-negative integer.
	// It is made of a sequence of digits in base 2^32.
	// Leading zero-digits are not allowed.
	// The value 'zero' is represented by an empty digit sequence.

	class bignatural {
		uint32_t* digits;	// Ptr to the digits
		uint32_t m_size;		// Actual number of digits
		uint32_t m_capacity;	// Current vector capacity

		inline static uint32_t* BN_ALLOC(uint32_t num_bytes) { return (uint32_t*)nfgMemoryPool.alloc(num_bytes); }
		inline static void BN_FREE(uint32_t* ptr) { nfgMemoryPool.release(ptr); }

		void init(const bignatural& m);
		void init(const uint64_t m); 

	public:
		// Creates a 'zero'
		bignatural() : digits(NULL), m_size(0), m_capacity(0) { }

		~bignatural() { BN_FREE(digits); }

		bignatural(const bignatural& m) { init(m); }

		bignatural(bignatural&& m) noexcept : digits(m.digits), m_size(m.m_size), m_capacity(m.m_capacity) { 
			m.digits = nullptr;
		}

		// Creates from uint64_t
		bignatural(uint64_t m) { init(m); }

		// If the number fits a uint64_t convert and return true
		bool toUint64(uint64_t& n) const;

		// If the number fits a uint32_t convert and return true
		bool toUint32(uint32_t& n) const;

		bignatural& operator=(const bignatural& m);

		bignatural& operator=(const uint64_t m);

		inline const uint32_t& back() const { return digits[m_size - 1]; }

		inline const uint32_t& operator[](int i) const { return digits[i]; }

		inline uint32_t size() const { return m_size; }

		inline bool empty() const { return m_size == 0; }

		// Left-shift by n bits and possibly add limbs as necessary
		void operator<<=(uint32_t n);

		// Right-shift by n bits
		void operator>>=(uint32_t n);

		bool operator==(const bignatural& b) const;

		bool operator!=(const bignatural& b) const;

		bool operator>=(const bignatural& b) const;

		bool operator>(const bignatural& b) const;

		bool operator<=(const bignatural& b) const { return b >= *this; }

		bool operator<(const bignatural& b) const { return b > *this; }

		bignatural operator+(const bignatural& b) const;

		// Assume that b is smaller than or equal to this number!
		bignatural operator-(const bignatural& b) const;

		bignatural operator*(const bignatural& b) const;

		// Short division algorithm
		bignatural divide_by(const uint32_t D, uint32_t& remainder) const;

		uint32_t getNumSignificantBits() const;

		bool getBit(uint32_t b) const;

		// Long division
		bignatural divide_by(const bignatural& divisor, bignatural& remainder) const;

		// Greatest common divisor (Euclidean algorithm)
		bignatural GCD(const bignatural& D) const;

		// String representation in decimal form
		std::string get_dec_str() const;

		// String representation in binary form
		std::string get_str() const;

		// Count number of zeroes on the right (least significant binary digits)
		uint32_t countEndingZeroes() const;

	protected:
		inline uint32_t& back() { return digits[m_size - 1]; }

		inline void pop_back() { m_size--; }

		inline uint32_t& operator[](int i) { return digits[i]; }

		inline void push_back(uint32_t b) {
			if (m_size == m_capacity) increaseCapacity((m_capacity|1)<<2);
			digits[m_size++] = b;
		}

		inline void push_bit_back(uint32_t b) {
			if (m_size) {
				operator<<=(1);
				back() |= b;
			}
			else if (b) push_back(1);
		}

		inline void reserve(uint32_t n) {
			if (n > m_capacity) increaseCapacity(n);
		}

		inline void resize(uint32_t n) {
			reserve(n);
			m_size = n;
		}

		inline void fill(uint32_t v) {
			for (uint32_t i = 0; i < m_size; i++) digits[(int)i] = v;
		}

		inline void pop_front() {
			for (uint32_t i = 1; i < m_size; i++) digits[i - 1] = digits[i];
			pop_back();
		}

		// Count number of zeroes on the left (most significant digits)
		uint32_t countLeadingZeroes() const;

		// Count number of zeroes on the right of the last 1 in the least significant limb
		// Assumes that number is not zero and last limb is not zero!
		uint32_t countEndingZeroesLSL() const {
			const uint32_t d = back();
			uint32_t i = 31;
			while (!(d << i)) i--;
			return 31 - i;
		}

		inline void pack() {
			uint32_t i = 0;
			while (i < m_size && digits[i] == 0) i++;
			if (i) {
				uint32_t* dold = digits + i;
				uint32_t* dnew = digits;
				uint32_t* dend = digits + m_size;
				while (dold < dend) *dnew++ = *dold++;
				m_size -= i;
			}
		}

		// a and b must NOT be this number!
		void toSum(const bignatural& a, const bignatural& b);

		// a and b must NOT be this number!
		// Assume that b is smaller or equal than a!
		void toDiff(const bignatural& a, const bignatural& b);

		// a and b must NOT be this number!
		void toProd(const bignatural& a, const bignatural& b);

	private:

		// Multiplies by a single limb, left shift, and add to accumulator. Does not pack!
		void addmul(uint32_t b, uint32_t left_shifts, bignatural& result) const;

		void increaseCapacity(uint32_t new_capacity);

		friend class bigfloat;
	};

	inline std::ostream& operator<<(std::ostream& os, const bignatural& p)
	{
		os << p.get_dec_str();
		return os;
	}

	/////////////////////////////////////////////////////////////////////
	// 	   
	// 	   B I G   F L O A T
	// 
	/////////////////////////////////////////////////////////////////////

	// A bigfloat is a floting point number with arbitrarily large mantissa.
	// In principle, we could have made the exponent arbitrarily large too,
	// but in practice this appears to be useless.
	// Exponents are in the range [-INT32_MAX, INT32_MAX]
	//
	// A bigfloat f evaluates to f = sign * mantissa * 2^exponent
	//
	// mantissa is a bignatural whose least significant bit is 1.
	// Number is zero if mantissa is empty.

	class bigfloat {
		bignatural mantissa; // .back() is less significant. Use 32-bit limbs to avoid overflows using 64-bits
		int32_t exponent; // In principle we might still have under/overflows, but not in practice
		int32_t sign;	// Redundant but keeps alignment

	public:
		// Default constructor creates a zero-valued bigfloat
		bigfloat() : exponent(0), sign(0) {}

		// Lossless conversion from double
		bigfloat(const double d);

		// Truncated approximation
		double get_d() const;
		
		bigfloat operator+(const bigfloat& b) const;

		bigfloat operator-(const bigfloat& b) const;

		bigfloat operator*(const bigfloat& b) const;

		void invert() { sign = -sign; }

		bigfloat inverse() const {
			bigfloat r = *this;
			r.invert();
			return r;
		}

		inline int sgn() const { return sign; }

		std::string get_str() const;

		const bignatural& getMantissa() const { return mantissa; }
		int32_t getExponent() const { return exponent; }

	private:

		// Right-shift as long as the least significant bit is zero
		void pack();

		// Left-shift the mantissa by n bits and reduce the exponent accordingly
		void leftShift(uint32_t n) {
			mantissa <<= n;
			exponent -= n;
		}
	};

	inline int sgn(const bigfloat& f) {
		return f.sgn();
	}

	inline bigfloat operator-(const bigfloat& f) {
		return f.inverse();
	}

	inline bigfloat operator*(double d, const bigfloat& f) {
		return f * bigfloat(d);
	}

	inline std::ostream& operator<<(std::ostream& os, const bigfloat& p)
	{
		if (p.sgn() < 0) os << "-";
		os << p.getMantissa().get_dec_str() << " * 2^" << p.getExponent();
		return os;
	}

/////////////////////////////////////////////////////////////////////
// 	   
// 	   B I G   R A T I O N A L
// 
/////////////////////////////////////////////////////////////////////

// A bigrational is a fraction of two coprime bignaturals with a sign.
// Number is zero if sign is zero

	class bigrational {
		bignatural numerator, denominator;
		int32_t sign;	// Redundant but keeps alignment

		// Iteratively divide both num and den by two as long as they are both even
		void compress();

		// Make numerator and denominator coprime (divide both by GCD)
		void canonicalize();

	public:
		// Create a zero
		bigrational() : sign(0) {}

		// Create from a bigfloat (lossless)
		bigrational(const bigfloat& f);

		// Create from explicit numerator, denominator and sign.
		bigrational(const bignatural& num, const bignatural& den, int32_t s) :
			numerator(num), denominator(den), sign(s) {
			canonicalize();
		}

		// Convert to multiplicative inverse
		void invert() {
			if (!sign) ip_error("bigrational::invert() : inverse zero!\n");
			std::swap(numerator, denominator);
		}

		// Return multiplicative inverse
		bigrational inverse() const { bigrational r = *this; r.invert(); return r; }

		// Invert sign
		void negate() { sign = -sign; }

		// Return additive inverse
		bigrational negation() const { bigrational r = *this; r.negate(); return r; }

		// Standard arithmetic ops
		bigrational operator+(const bigrational& r) const;

		bigrational operator-(const bigrational& r) const { return operator+(r.negation()); }

		bigrational operator*(const bigrational& r) const {
			if (sign == 0 || r.sign == 0) return bigrational();
			else return bigrational(numerator * r.numerator, denominator * r.denominator, sign * r.sign);
		}

		bigrational operator/(const bigrational& r) const {
			if (!r.sign) ip_error("bigrational::operator/ : division by zero!\n");
			return operator*(r.inverse()); 
		}

		// Comparison operators
		bool operator==(const bigrational& r) const {
			return (sign == r.sign && numerator == r.numerator && denominator == r.denominator);
		}

		bool operator!=(const bigrational& r) const {
			return (sign != r.sign || numerator != r.numerator || denominator != r.denominator);
		}

		bool hasGreaterModule(const bigrational& r) const {
			return numerator * r.denominator > r.numerator * denominator;
		}

		bool hasGrtrOrEqModule(const bigrational& r) const {
			return numerator * r.denominator >= r.numerator * denominator;
		}

		bool operator>(const bigrational& r) const {
			return (sign > r.sign || (sign > 0 && r.sign > 0 && hasGreaterModule(r)) || (sign < 0 && r.sign < 0 && r.hasGreaterModule(*this)));
		}

		bool operator>=(const bigrational& r) const {
			return (sign > r.sign || (sign > 0 && r.sign > 0 && hasGrtrOrEqModule(r)) || (sign < 0 && r.sign < 0 && r.hasGrtrOrEqModule(*this)));
		}

		bool operator<(const bigrational& r) const {
			return (sign < r.sign || (sign < 0 && r.sign < 0 && hasGreaterModule(r)) || (sign > 0 && r.sign > 0 && r.hasGreaterModule(*this)));
		}

		bool operator<=(const bigrational& r) const {
			return (sign < r.sign || (sign < 0 && r.sign < 0 && hasGrtrOrEqModule(r)) || (sign > 0 && r.sign > 0 && r.hasGrtrOrEqModule(*this)));
		}

		// Conversion to double (truncated)
		double get_d() const;

		const bignatural& get_num() const { return numerator; }
		const bignatural& get_den() const { return denominator; }
		int32_t sgn() const { return sign; }

		// Return decimal representation
		std::string get_dec_str() const;

		// Return binary representation
		std::string get_str() const;
	};

	inline bigrational operator-(const bigrational& p) { return p.negation(); }

	inline int32_t sgn(const bigrational& p) { return p.sgn(); }

	inline std::ostream& operator<<(std::ostream& os, const bigrational& p)
	{
		os << p.get_dec_str();
		return os;
	}
#endif // USE_GNU_GMP_CLASSES

#include "numerics.hpp"

#endif //NUMERICS_H
