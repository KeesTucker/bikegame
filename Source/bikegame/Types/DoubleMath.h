#pragma once

constexpr double DSmall_Number = 1e-8;

struct FDoubleMath
{
	static double Lerp(const double InA, const double InB, const double InAlpha)
	{
		return InA + (InB - InA) * InAlpha;
	}

	static double Clamp(const double InValue, const double InMin, const double InMax)
	{
		return InValue < InMin ? InMin : InValue > InMax ? InMax : InValue;
	}
};
