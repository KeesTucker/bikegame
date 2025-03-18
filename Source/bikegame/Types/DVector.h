#pragma once

#include "CoreMinimal.h"
#include <cmath>

struct DVector {
	double x, y, z;

	DVector() : x(0.0), y(0.0), z(0.0) {}
	DVector(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
	DVector(const FVector& vec) : x(vec.X), y(vec.Y), z(vec.Z) {}

	operator FVector() const {
		return FVector(static_cast<float>(x),
					   static_cast<float>(y),
					   static_cast<float>(z));
	}

	DVector operator+(const DVector& other) const {
		return DVector(x + other.x, y + other.y, z + other.z);
	}
	DVector operator-(const DVector& other) const {
		return DVector(x - other.x, y - other.y, z - other.z);
	}
	DVector operator*(double scalar) const {
		return DVector(x * scalar, y * scalar, z * scalar);
	}
	DVector operator/(double scalar) const {
		return DVector(x / scalar, y / scalar, z / scalar);
	}
	DVector& operator+=(const DVector& other) {
		x += other.x; y += other.y; z += other.z;
		return *this;
	}
	DVector& operator-=(const DVector& other) {
		x -= other.x; y -= other.y; z -= other.z;
		return *this;
	}
	double Size() const {
		return std::sqrt(x * x + y * y + z * z);
	}
	DVector GetSafeNormal() const {
		double s = Size();
		return (s > 1e-8) ? (*this / s) : DVector(0.0, 0.0, 0.0);
	}

	static double Dot(const DVector& a, const DVector& b) {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}
	static DVector Cross(const DVector& a, const DVector& b) {
		return DVector(
			a.y * b.z - a.z * b.y,
			a.z * b.x - a.x * b.z,
			a.x * b.y - a.y * b.x
		);
	}
	static DVector Lerp(const DVector& A, const DVector& B, double Alpha) {
		return A + (B - A) * Alpha;
	}
};
