#include "KReverseEulerConstraintComponent.h"

#include "KPhysicsMeshComponent.h"
#include "bikegame/Types/DoubleMath.h"
#include "GameFramework/Actor.h"
#include "Math/UnrealMathUtility.h"

UKReverseEulerConstraintComponent::UKReverseEulerConstraintComponent(): PhysicsComponentA(nullptr),
                                                                      PhysicsComponentB(nullptr)
{
	PrimaryComponentTick.bCanEverTick = true;
}

#if WITH_EDITOR
void UKReverseEulerConstraintComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	
	Init();
}
#endif

void UKReverseEulerConstraintComponent::BeginPlay()
{
	Super::BeginPlay();

	Init();
	
	if (UKPhysicsTickSubsystem* TickSubsystem = GetWorld()->GetSubsystem<UKPhysicsTickSubsystem>())
	{
		TickSubsystem->RegisterComponent(this);
	}
}

void UKReverseEulerConstraintComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UKPhysicsTickSubsystem* TickSubsystem = GetWorld()->GetSubsystem<UKPhysicsTickSubsystem>())
	{
		TickSubsystem->UnregisterComponent(this);
	}
	Super::EndPlay(EndPlayReason);
}

void UKReverseEulerConstraintComponent::Init()
{
	if (const AActor* Owner = GetOwner())
	{
		TArray<UActorComponent*> Components;
		Owner->GetComponents(UKPhysicsMeshComponent::StaticClass(), Components);
		for (UActorComponent* Comp : Components)
		{
			if (Comp->GetName().Equals(PhysicsComponentNameA))
			{
				PhysicsComponentA = Cast<UKPhysicsMeshComponent>(Comp);
				continue;
			}
			if (Comp->GetName().Equals(PhysicsComponentNameB))
			{
				PhysicsComponentB = Cast<UKPhysicsMeshComponent>(Comp);
				continue;
			}
		}
	}
	
	if (!PhysicsComponentA || !PhysicsComponentB)
	{
		return;
	}
	const FDoubleVector PosA = FDoubleVector(PhysicsComponentA->GetComponentLocation());
	const FDoubleVector PosB = FDoubleVector(PhysicsComponentB->GetComponentLocation());
	InitialRelativePosition = PosB - PosA;
}

void UKReverseEulerConstraintComponent::PhysicsTick(const double DeltaTime)
{
	if (!PhysicsComponentA || !PhysicsComponentB)
	{
		return;
	}

	// Get the current world positions.
	const FDoubleVector PosA = FDoubleVector(PhysicsComponentA->GetComponentLocation());
	const FDoubleVector PosB = FDoubleVector(PhysicsComponentB->GetComponentLocation());

	// Compute the current displacement.
	const FDoubleVector CurrentDisplacement = PosB - PosA;
	// Calculate the error relative to the initial separation.
	const FDoubleVector DisplacementError = CurrentDisplacement - InitialRelativePosition;

	// Retrieve velocities (using your existing methods).
	const FDoubleVector VelA = PhysicsComponentA->GetKLinearVelocity();
	const FDoubleVector VelB = PhysicsComponentB->GetKLinearVelocity();
	const FDoubleVector RelativeVelocity = VelB - VelA;

	// Retrieve masses.
	const double MassA = PhysicsComponentA->GetKMass();
	const double MassB = PhysicsComponentB->GetKMass();

	// Calculate the effective mass.
	const double EffectiveMass = MassA * MassB / (MassA + MassB);

	// Compute the spring force correction based on the displacement error.
	const FDoubleVector VelocityCorrection = ComputeSpringVelocity(DeltaTime, DisplacementError, RelativeVelocity, EffectiveMass, SpringConstant, DampingConstant);
	
	// Apply equal and opposite corrections.
	PhysicsComponentA->AddKLinearVelocity(VelocityCorrection * -1.0);
	PhysicsComponentB->AddKLinearVelocity(VelocityCorrection * 1.0);
}

FDoubleVector UKReverseEulerConstraintComponent::ComputeSpringVelocity(
	const double DeltaTime,
	const FDoubleVector& RelativeDisplacement,
	const FDoubleVector& RelativeVelocity,
	const double EffectiveMass,
	const double SpringK,
	const double DampingC
)
{
	// Construct the implicit integration system matrix:
	// [ 1 + (DampingC*DeltaTime/EffectiveMass)    (SpringK*DeltaTime/EffectiveMass) ]
	// [             -DeltaTime                              1                    ]
	const double A11 = 1.0 + (DampingC * DeltaTime / EffectiveMass);
	const double A12 = (SpringK * DeltaTime / EffectiveMass);
	const double A21 = -DeltaTime;
	constexpr double A22 = 1.0;

	// Calculate the determinant.
	const double Det = A11 * A22 - A12 * A21;
	if (FMath::Abs(Det) < DSmall_Number)
	{
		// Avoid division by zero; no force is applied.
		return FDoubleVector::Zero();
	}

	// Calculate the inverse for the velocity update (only the first row is needed).
	const double Inv_A11 = A22 / Det;
	const double Inv_A12 = -A12 / Det;

	// Compute the new relative velocity using implicit integration.
	const FDoubleVector NewRelativeVelocity = RelativeVelocity * Inv_A11 + RelativeDisplacement * Inv_A12;

	return NewRelativeVelocity - RelativeVelocity;
}
