/*
* fraction.h
* Fraction
*
* Created By Tony Huang @ 2015-9-28
* Copyright 2015 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once
#include <boost/version.hpp>
#if defined (BOOST_VERSION) && (BOOST_VERSION >= 107000)
#include <boost/integer/common_factor.hpp> 
using namespace boost::integer;
#else
#include <boost/math/common_factor.hpp>  //this header is deprecated in boost 1.70
using namespace boost::math;
#endif

namespace rpos { namespace system {

	template < class RepT >
	struct Fraction
	{
		RepT numerator;
		RepT denominator;

		Fraction(const RepT& n, const RepT& d = 1)
			: numerator(n)
			, denominator(d)
		{}

		Fraction<RepT> simplify() const
		{
			RepT gcd = gcd(numerator, denominator);

			if (gcd == 0 || gcd == 1)
				return *this;
		}

		Fraction<RepT> inverse() const
		{
			return Fraction<RepT>(denominator, numerator);
		}

		RepT flatten() const
		{
			return numerator / denominator;
		}
	};

	template < class RepT >
	Fraction<RepT> operator+(const Fraction<RepT>& a, const Fraction<RepT>& b)
	{
		if (a.denominator == b.denominator)
			return Fraction<RepT>(a.numerator + b.numerator, a.denominator);
		else
			return Fraction<RepT>(a.numerator * b.denominator + b.numerator * a.denominator, a.denominator * b.denominator);
	}

	template < class RepT >
	Fraction<RepT> operator-(const Fraction<RepT>& a, const Fraction<RepT>& b)
	{
		if (a.denominator == b.denominator)
			return Fraction<RepT>(a.numerator - b.numerator, a.denominator);
		else
			return Fraction<RepT>(a.numerator * b.denominator - b.numerator * a.denominator, a.denominator * b.denominator);
	}

	template < class RepT >
	Fraction<RepT> operator*(const Fraction<RepT>& a, const Fraction<RepT>& b)
	{
		return Fraction<RepT>(a.numerator * b.numerator, a.denominator * b.denominator);
	}

	template < class RepT >
	Fraction<RepT> operator/(const Fraction<RepT>& a, const Fraction<RepT>& b)
	{
		return a * b.inverse();
	}

#define RPOS_SYSTEM_FRACTION_OPERATOR_WITH_SCALAR(o) \
	template < class RepT > \
	Fraction<RepT> operator o(const Fraction<RepT>& a, const RepT& b) \
	{ return a o Fraction<RepT>(b); } \
	template < class RepT > \
	Fraction<RepT> operator o(const RepT& a, const Fraction<RepT>& b) \
	{ return Fraction<RepT>(a) o b; }

	RPOS_SYSTEM_FRACTION_OPERATOR_WITH_SCALAR(+);
	RPOS_SYSTEM_FRACTION_OPERATOR_WITH_SCALAR(-);
	RPOS_SYSTEM_FRACTION_OPERATOR_WITH_SCALAR(*);
	RPOS_SYSTEM_FRACTION_OPERATOR_WITH_SCALAR(/);

} }
