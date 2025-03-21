#pragma once
#include "DoubleVector.h"

#include "ReverseEulerSpring.generated.h"

USTRUCT()
struct FReverseEulerSpring
{
	GENERATED_BODY()
	
	static FDoubleVector ComputeSpringVelocity(
		const double DeltaTime,
		const FDoubleVector& Error,
		const FDoubleVector& Velocity,
		const double Inertia,
		const double SpringK,
		const double DampingC
	)
	{
		// Construct the implicit integration system matrix:
		// [ 1 + (DampingC*DeltaTime/EffectiveMass)    (SpringK*DeltaTime/EffectiveMass) ]
		// [             -DeltaTime                              1                    ]
		const double A11 = 1.0 + DampingC * DeltaTime / Inertia;
		const double A12 = SpringK * DeltaTime / Inertia;
		const double A21 = -DeltaTime;
		constexpr double A22 = 1.0;

		// Calculate the determinant.
		const double Det = A11 * A22 - A12 * A21;
		if (FMath::Abs(Det) < DSmallNumber)
		{
			// Avoid division by zero; no force is applied.
			return FDoubleVector::Zero();
		}

		// Calculate the inverse for the velocity update (only the first row is needed).
		const double Inv_A11 = A22 / Det;
		const double Inv_A12 = -A12 / Det;

		// Compute the new relative velocity using implicit integration.
		const FDoubleVector NewRelativeVelocity = Velocity * Inv_A11 + Error * Inv_A12;

		return NewRelativeVelocity - Velocity;
	}
};
