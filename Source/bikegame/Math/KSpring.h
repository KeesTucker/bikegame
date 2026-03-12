#pragma once
#include "DoubleVector.h"

#include "KSpring.generated.h"

USTRUCT()
struct FKSpring
{
	GENERATED_BODY()
	
	// Implicit spring solver. Discretises the damped harmonic oscillator (mx'' + cx' + kx = 0) as a
	// 2x2 system over [v, x] and solves it exactly for this timestep. Unconditionally stable so you
	// can crank spring constants without it blowing up, unlike the explicit version.
	static FDoubleVector ComputeReverseEulerSpringVelocityCorrection(
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

		const double Det = A11 * A22 - A12 * A21;
		if (FMath::Abs(Det) < DSmallNumber)
		{
			// Avoid division by zero; no force is applied.
			return FDoubleVector::Zero();
		}

		// Calculate the inverse for the velocity update (only the first row is needed).
		const double Inv_A11 = A22 / Det;
		const double Inv_A12 = -A12 / Det;

		const FDoubleVector NewRelativeVelocity = Velocity * Inv_A11 + Error * Inv_A12;

		return NewRelativeVelocity - Velocity;
	}

	// Explicit spring, just F=ma discretised directly. Goes unstable if stiffness or timestep gets too
	// large so don't push it. Fine for soft stuff like suspension.
	static FDoubleVector ComputeExplicitSpringVelocityCorrection(
	const double DeltaTime,
	const FDoubleVector& Error,
	const FDoubleVector& Velocity,
	const double Inertia,
	const double SpringK,
	const double DampingC
)
	{
		const FDoubleVector SpringForce = -SpringK * Error;
		const FDoubleVector DampingForce = -DampingC * Velocity;
		const FDoubleVector Acceleration = (SpringForce + DampingForce) / Inertia;
		return Acceleration * DeltaTime;
	}
};
