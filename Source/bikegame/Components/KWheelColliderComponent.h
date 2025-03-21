// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "KPhysicsMeshComponent.h"
#include "KWheelColliderComponent.generated.h"

UCLASS(ClassGroup=(Physics), meta=(BlueprintSpawnableComponent))
class BIKEGAME_API UKWheelColliderComponent : public UKPhysicsMeshComponent
{
	GENERATED_BODY()

	virtual void PhysicsTick(const double DeltaTime) override;
};
