#include "KReverseEulerConstraintComponent.h"

#include "KPhysicsMeshComponent.h"
#include "bikegame/Math/DoubleMath.h"
#include "GameFramework/Actor.h"
#include "Math/UnrealMathUtility.h"

UKReverseEulerConstraintComponent::UKReverseEulerConstraintComponent():
	PhysicsComponentA(nullptr), PhysicsComponentB(nullptr), InitialDistance(0)
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

	// Use component location instead of GetKLocation since KLocation is set in BeginPlay and results in a race condition
	const FDoubleVector PosA = FDoubleVector(PhysicsComponentA->GetComponentLocation());
	const FDoubleVector PosB = FDoubleVector(PhysicsComponentB->GetComponentLocation());
	InitialAToBDisplacement = PosB - PosA;
	InitialBToADisplacement = PosA - PosB;
	InitialDistance = InitialAToBDisplacement.Size();
	InitialAToBDirection = InitialAToBDisplacement.GetSafeNormal();
	InitialBToADirection = InitialBToADisplacement.GetSafeNormal();
	InitialAOrientation = FDoubleQuat(PhysicsComponentA->GetComponentQuat());
	InitialBOrientation = FDoubleQuat(PhysicsComponentB->GetComponentQuat());
}

void UKReverseEulerConstraintComponent::PhysicsTick(const double DeltaTime)
{
	if (!PhysicsComponentA || !PhysicsComponentB)
	{
		return;
	}
	
	const FDoubleVector PosA = PhysicsComponentA->GetKLocation();
	const FDoubleVector PosB = PhysicsComponentB->GetKLocation();
	const FDoubleQuat OrientationA = PhysicsComponentA->GetKOrientation();
	const FDoubleQuat OrientationB = PhysicsComponentB->GetKOrientation();

	ApplyLinearSpring(DeltaTime, PosA, PosB);
	ApplyAngularSpring(DeltaTime, PosA, PosB, OrientationA, OrientationB);
}

void UKReverseEulerConstraintComponent::ApplyLinearSpring(const double DeltaTime, const FDoubleVector& PosA, const FDoubleVector& PosB) const
{
	// Get the current displacement vector and its magnitude.
	const FDoubleVector CurrentDisplacement = PosB - PosA;
	const double CurrentDistance = CurrentDisplacement.Size();

	// Calculate the error in distance.
	const double ErrorDistance = CurrentDistance - InitialDistance;

	// Get the radial direction (if the distance is non-zero).
	const FDoubleVector ErrorDirection = CurrentDisplacement.GetSafeNormal();

	// Create an error vector that only corrects the distance.
	const FDoubleVector ErrorDisplacement = ErrorDirection * ErrorDistance;
	
	// Retrieve velocities.
	const FDoubleVector VelA = PhysicsComponentA->GetKLinearVelocity();
	const FDoubleVector VelB = PhysicsComponentB->GetKLinearVelocity();
	const FDoubleVector RelativeVelocity = VelB - VelA;
	const FDoubleVector SpringRelativeVelocity = FDoubleVector::Dot(RelativeVelocity, ErrorDirection) * ErrorDirection;

	DrawDebugDirectionalArrow(GetWorld(), FVector(PosA), FVector(PosA + SpringRelativeVelocity), 100.0f, FColor::Blue);

	// Retrieve masses.
	const double MassA = PhysicsComponentA->GetKMass();
	const double MassB = PhysicsComponentB->GetKMass();

	// Calculate the effective mass.
	const double EffectiveMass = MassA * MassB / (MassA + MassB);

	// Compute the spring force correction based on the distance error.
	const FDoubleVector VelocityCorrection = ComputeSpringVelocity(
		DeltaTime,
		ErrorDisplacement,
		SpringRelativeVelocity,
		EffectiveMass,
		LinearSpringConstant,
		LinearDampingConstant
	);
    
	// Apply equal and opposite corrections.
	PhysicsComponentA->AddKLinearVelocity(-1 * VelocityCorrection);
	PhysicsComponentB->AddKLinearVelocity(VelocityCorrection);
}

void UKReverseEulerConstraintComponent::ApplyAngularSpring(const double DeltaTime, const FDoubleVector& PosA,
	const FDoubleVector& PosB, const FDoubleQuat& OrientationA, const FDoubleQuat& OrientationB) const
{
	const FDoubleVector PosAToPosBDisplacement = PosB - PosA;
	const FDoubleVector PosBToPosADisplacement = PosA - PosB;
	
	const FDoubleVector SpringAngularVelocityA = GetAngularSpringVelocity(
		DeltaTime,
		InitialAToBDirection,
		PosAToPosBDisplacement.GetSafeNormal(),
		InitialAOrientation,
		OrientationA,
		PhysicsComponentB->GetKAngularVelocity() - PhysicsComponentA->GetKAngularVelocity()
	);
	const FDoubleVector SpringAngularVelocityB = GetAngularSpringVelocity(
		DeltaTime,
		InitialBToADirection,
		PosBToPosADisplacement.GetSafeNormal(),
		InitialBOrientation,
		OrientationB,
		PhysicsComponentA->GetKAngularVelocity() - PhysicsComponentB->GetKAngularVelocity()
	);
	
	PhysicsComponentA->AddKAngularVelocity(-1 * SpringAngularVelocityA);
	PhysicsComponentB->AddKAngularVelocity(-1 * SpringAngularVelocityB);

	PhysicsComponentA->AddKLinearVelocity(FDoubleVector::Cross(-1 * SpringAngularVelocityB, PosAToPosBDisplacement));
	PhysicsComponentB->AddKLinearVelocity(FDoubleVector::Cross(-1 * SpringAngularVelocityA, PosBToPosADisplacement));

	// TODO: This is all kind of wrong. We need to apply a torque to the opposite body due to the difference in
	//  target vs actual direction. This torque then needs to be applied inversely to the current body. We shouldn't be
	//  applying angular velocity to the current body to align them.
}

FDoubleVector UKReverseEulerConstraintComponent::GetAngularSpringVelocity(
	const double DeltaTime,
	const FDoubleVector& InitialDirection,
	const FDoubleVector& Direction,
	const FDoubleQuat& InitialOrientation,
	const FDoubleQuat& Orientation,
	const FDoubleVector& RelativeAngularVelocity) const
{
	const FDoubleVector TargetDirection = (Orientation * InitialOrientation.Inverse()).RotateVector(InitialDirection).GetSafeNormal();
	FDoubleVector ErrorAxis;
	double ErrorAngle;
	FDoubleVector::NormalDifferenceToAxisAngle(TargetDirection, Direction, ErrorAxis, ErrorAngle);
	const FDoubleVector ErrorAngularDisplacement = ErrorAxis * ErrorAngle;

	const double AInertiaAlongErrorAxis = FDoubleVector::Dot(ErrorAxis, PhysicsComponentA->GetKWorldInertiaTensor() * ErrorAxis);
	const double BInertiaAlongErrorAxis = FDoubleVector::Dot(ErrorAxis, PhysicsComponentB->GetKWorldInertiaTensor() * ErrorAxis);
	const double EffectiveInertia = 1.0 / ((AInertiaAlongErrorAxis > DSmallNumber ? 1.0 / AInertiaAlongErrorAxis : 0.0) + 
									  (BInertiaAlongErrorAxis > DSmallNumber ? 1.0 / BInertiaAlongErrorAxis : 0.0));
	
	return ComputeAngularSpringVelocity(DeltaTime, ErrorAngularDisplacement, RelativeAngularVelocity, EffectiveInertia, AngularSpringConstant, AngularDampingConstant);
}

FDoubleVector UKReverseEulerConstraintComponent::ComputeSpringVelocity(
	const double DeltaTime,
	const FDoubleVector& ErrorDisplacement,
	const FDoubleVector& RelativeVelocity,
	const double EffectiveMass,
	const double SpringK,
	const double DampingC
)
{
	// Construct the implicit integration system matrix:
	// [ 1 + (DampingC*DeltaTime/EffectiveMass)    (SpringK*DeltaTime/EffectiveMass) ]
	// [             -DeltaTime                              1                    ]
	const double A11 = 1.0 + DampingC * DeltaTime / EffectiveMass;
	const double A12 = SpringK * DeltaTime / EffectiveMass;
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
	const FDoubleVector NewRelativeVelocity = RelativeVelocity * Inv_A11 + ErrorDisplacement * Inv_A12;

	return NewRelativeVelocity - RelativeVelocity;
}

FDoubleVector UKReverseEulerConstraintComponent::ComputeAngularSpringVelocity(
	const double DeltaTime,
	const FDoubleVector& ErrorAngularDisplacement,
	const FDoubleVector& RelativeAngularVelocity,
	const double EffectiveInertia,
	const double AngularSpringK,
	const double AngularDampingC
)
{
	// Construct the implicit integration system matrix for angular quantities:
	// [ 1 + (AngularDampingC*DeltaTime/EffectiveInertia)    (AngularSpringK*DeltaTime/EffectiveInertia) ]
	// [                 -DeltaTime                                  1                                ]
	const double A11 = 1.0 + AngularDampingC * DeltaTime / EffectiveInertia;
	const double A12 = AngularSpringK * DeltaTime / EffectiveInertia;
	const double A21 = -DeltaTime;
	constexpr double A22 = 1.0;

	// Calculate the determinant.
	const double Det = A11 * A22 - A12 * A21;
	if (FMath::Abs(Det) < DSmallNumber)
	{
		// Avoid division by zero; no torque is applied.
		return FDoubleVector::Zero();
	}

	// Calculate the inverse for the velocity update (only the first row is needed).
	const double Inv_A11 = A22 / Det;
	const double Inv_A12 = -A12 / Det;

	// Compute the new relative angular velocity using implicit integration.
	const FDoubleVector NewRelativeAngularVelocity = RelativeAngularVelocity * Inv_A11 + ErrorAngularDisplacement * Inv_A12;

	return NewRelativeAngularVelocity - RelativeAngularVelocity;
}