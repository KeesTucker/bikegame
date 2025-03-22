// Fill out your copyright notice in the Description page of Project Settings.


#include "KWheelColliderComponent.h"

#include "bikegame/Math/KSpring.h"


void UKWheelColliderComponent::BeginPlay()
{
	Super::BeginPlay();

	RimRadius = RimDiameterInches * 2.54 / 2.0;
	TireSidewall = (TireWidthMM / 10.0) * TireAspectRatio / 100.0;
	TotalWheelRadius = RimRadius + TireSidewall;
}

void UKWheelColliderComponent::PhysicsTick(const double DeltaTime)
{
	Super::PhysicsTick(DeltaTime);
}

void UKWheelColliderComponent::ResolveCollision(const double DeltaTime, const FHitResult& Hit)
{
	const FDoubleVector GroundVector = GetKLocation() - FDoubleVector(Hit.ImpactPoint);
	const FDoubleVector GroundNormal = GroundVector.GetNormalized();
	const double GroundDistance = GroundVector.Size();
	const FDoubleVector Deflection = (GroundDistance - TotalWheelRadius) * GroundNormal;
	DrawDebugDirectionalArrow(GetWorld(), FVector(GetKLocation()), FVector(GetKLocation()) + FVector(Deflection), 100.0f, FColor::Blue, false, 0.1f, 0.f, 0.5f);
	const FDoubleVector NormalVelocity = -1 * FKSpring::ComputeExplicitSpringVelocityCorrection(
		DeltaTime,
		Deflection,
		GetKLinearVelocity().ProjectOnto(FDoubleVector(GroundNormal)),
		GetKMass(),
		NormalSpringConstant,
		NormalDampingConstant
	);
	
	if (const double Dot = FDoubleVector::Dot(NormalVelocity, FDoubleVector(Hit.Normal)); Dot > 0)
	{
		AddKLinearVelocity(NormalVelocity);
	}
}
