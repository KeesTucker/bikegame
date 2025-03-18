#include "KReverseEulerConstraintComponent.h"

#include "KPhysicsMeshComponent.h"
#include "bikegame/Types/DoubleMath.h"
#include "GameFramework/Actor.h"
#include "Math/UnrealMathUtility.h"

UReverseEulerConstraintComponent::UReverseEulerConstraintComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UReverseEulerConstraintComponent::BeginPlay()
{
	Super::BeginPlay();

	// Optionally, you might want to try finding the two components automatically.
	// For clarity, this example assumes both are explicitly assigned.
}

void UReverseEulerConstraintComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!PhysicsComponentA || !PhysicsComponentB)
	{
		return;
	}

	// Get the current world positions.
	FVector PosA = PhysicsComponentA->GetComponentLocation();
	FVector PosB = PhysicsComponentB->GetComponentLocation();

	// Retrieve velocities. If you are using your own velocity storage inside UKPhysicsMeshComponent,
	// ensure you expose or retrieve that instead of GetComponentVelocity().
	FVector VelA = PhysicsComponentA->GetComponentVelocity();
	FVector VelB = PhysicsComponentB->GetComponentVelocity();

	// Calculate the relative displacement (from A to B) and relative velocity.
	FVector RelativeDisplacement = PosB - PosA;
	FVector RelativeVelocity = VelB - VelA;

	// Retrieve masses.
	float MassA = PhysicsComponentA->GetMass();
	float MassB = PhysicsComponentB->GetMass();

	// Calculate the effective mass for a two-body system.
	// For point masses, m_eff = (mA * mB) / (mA + mB)
	float EffectiveMass = (MassA * MassB) / (MassA + MassB);

	// Compute the spring force using reverse Euler integration.
	FVector SpringForce = ComputeSpringForce(RelativeDisplacement, RelativeVelocity, EffectiveMass, DeltaTime, SpringConstant, DampingConstant);

	// Apply forces to both components:
	// - PhysicsComponentA receives +SpringForce.
	// - PhysicsComponentB receives -SpringForce.
	// (Assuming AddCustomForce adds the force to the internal velocity/impulse accumulator.)
	PhysicsComponentA->AddCustomForce(SpringForce);
	PhysicsComponentB->AddCustomForce(-SpringForce);
}

FVector UReverseEulerConstraintComponent::ComputeSpringForce(const FVector& RelativeDisplacement, const FVector& RelativeVelocity, float EffectiveMass, float DeltaTime, float SpringK, float DampingC) const
{
	// Construct the implicit integration system matrix:
	// [ 1 + (DampingC*DeltaTime/EffectiveMass)    (SpringK*DeltaTime/EffectiveMass) ]
	// [             -DeltaTime                              1                    ]
	const float a11 = 1.0f + (DampingC * DeltaTime / EffectiveMass);
	const float a12 = (SpringK * DeltaTime / EffectiveMass);
	const float a21 = -DeltaTime;
	const float a22 = 1.0f;

	// Calculate the determinant.
	const float det = (a11 * a22) - (a12 * a21);
	if (FMath::Abs(det) < DSmall_Number)
	{
		// Avoid division by zero; no force is applied.
		return FVector::ZeroVector;
	}

	// Calculate the inverse for the velocity update (only the first row is needed).
	const float inv_a11 = a22 / det;
	const float inv_a12 = -a12 / det;

	// Compute the new relative velocity using implicit integration.
	FVector NewRelativeVelocity = inv_a11 * RelativeVelocity + inv_a12 * RelativeDisplacement;

	// The force is computed as the impulse over the time step:
	// F = effective_mass * (NewRelativeVelocity - RelativeVelocity) / DeltaTime
	FVector Force = EffectiveMass * (NewRelativeVelocity - RelativeVelocity) / DeltaTime;
	return Force;
}
