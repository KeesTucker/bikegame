#pragma once

constexpr double DSmall_Number = 1e-8;

struct DMath {
	static double Lerp(double A, double B, double Alpha) {
		return A + (B - A) * Alpha;
	}
	static double Clamp(double Value, double Min, double Max) {
		return (Value < Min) ? Min : (Value > Max ? Max : Value);
	}
};
