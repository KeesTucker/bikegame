// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "KPhysicsMeshComponent.h"
#include "KWheelCollider.generated.h"

UCLASS()
class BIKEGAME_API UKWheelCollider : public UKPhysicsMeshComponent
{
	GENERATED_BODY()

	virtual void PhysicsTick(const double DeltaTime) override;
};
