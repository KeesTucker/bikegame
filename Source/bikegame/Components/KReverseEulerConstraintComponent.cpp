#include "KReverseEulerConstraintComponent.h"

#include "KPhysicsMeshComponent.h"
#include "bikegame/Math/DoubleMath.h"
#include "bikegame/Math/KSpring.h"
#include "GameFramework/Actor.h"
#include "Math/UnrealMathUtility.h"
#include "DrawDebugHelpers.h"
#include "bikegame/Settings/KPhysicsSettings.h"

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
	InitialAToBDirection = InitialAToBDisplacement.GetNormalized();
	InitialBToADirection = InitialBToADisplacement.GetNormalized();
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
	const FDoubleVector CurrentDisplacement = PosB - PosA;
	const double CurrentDistance = CurrentDisplacement.Size();
	const double ErrorDistance = CurrentDistance - InitialDistance;
	const FDoubleVector ErrorDirection = CurrentDisplacement.GetNormalized();
	const FDoubleVector ErrorDisplacement = ErrorDirection * ErrorDistance;

	const FDoubleVector VelA = PhysicsComponentA->GetKLinearVelocity();
	const FDoubleVector VelB = PhysicsComponentB->GetKLinearVelocity();
	const FDoubleVector RelativeVelocity = VelB - VelA;
	const FDoubleVector SpringRelativeVelocity = FDoubleVector::Dot(RelativeVelocity, ErrorDirection) * ErrorDirection;

	if (GetDefault<UKPhysicsSettings>()->bShowCollisionDiagnostics)
	{
		DrawDebugDirectionalArrow(GetWorld(), FVector(PosA), FVector(PosA + SpringRelativeVelocity), 10.0f, FColor::Blue, false, 0.3f, 0.f, 2.0f);
	}

	const double MassA = PhysicsComponentA->GetKMass();
	const double MassB = PhysicsComponentB->GetKMass();
	const double EffectiveMass = MassA * MassB / (MassA + MassB);

	const FDoubleVector VelocityCorrection = FKSpring::ComputeReverseEulerSpringVelocityCorrection(
		DeltaTime,
		ErrorDisplacement,
		SpringRelativeVelocity,
		EffectiveMass,
		LinearSpringConstant,
		LinearDampingConstant
	);

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
		PosAToPosBDisplacement.GetNormalized(),
		InitialAOrientation,
		OrientationA,
		PhysicsComponentB->GetKAngularVelocity() - PhysicsComponentA->GetKAngularVelocity()
	);
	const FDoubleVector SpringAngularVelocityB = GetAngularSpringVelocity(
		DeltaTime,
		InitialBToADirection,
		PosBToPosADisplacement.GetNormalized(),
		InitialBOrientation,
		OrientationB,
		PhysicsComponentA->GetKAngularVelocity() - PhysicsComponentB->GetKAngularVelocity()
	);
	
	PhysicsComponentA->AddKAngularVelocity(-1 * SpringAngularVelocityA);
	PhysicsComponentB->AddKAngularVelocity(-1 * SpringAngularVelocityB);

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
	const FDoubleVector TargetDirection = (Orientation * InitialOrientation.Inverse()).RotateVector(InitialDirection).GetNormalized();
	FDoubleVector ErrorAxis;
	double ErrorAngle;
	FDoubleVector::NormalDifferenceToAxisAngle(TargetDirection, Direction, ErrorAxis, ErrorAngle);
	const FDoubleVector ErrorAngularDisplacement = ErrorAxis * ErrorAngle;

	// Project the inertia tensor onto the error axis to get the scalar inertia resisting this rotation.
	// Comes from the rotational KE integral projected onto an axis. Then combine both bodies like
	// resistors in parallel to get the effective inertia.
	const double AInertiaAlongErrorAxis = FDoubleVector::Dot(ErrorAxis, PhysicsComponentA->GetKWorldInertiaTensor() * ErrorAxis);
	const double BInertiaAlongErrorAxis = FDoubleVector::Dot(ErrorAxis, PhysicsComponentB->GetKWorldInertiaTensor() * ErrorAxis);
	const double EffectiveInertia = 1.0 / ((AInertiaAlongErrorAxis > DSmallNumber ? 1.0 / AInertiaAlongErrorAxis : 0.0) +
									  (BInertiaAlongErrorAxis > DSmallNumber ? 1.0 / BInertiaAlongErrorAxis : 0.0));
	
	return FKSpring::ComputeReverseEulerSpringVelocityCorrection(
		DeltaTime,
		ErrorAngularDisplacement,
		RelativeAngularVelocity,
		EffectiveInertia,
		AngularSpringConstant,
		AngularDampingConstant
	);
}