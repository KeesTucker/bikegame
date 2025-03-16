// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "KPhysics.generated.h"


UCLASS(ClassGroup=(Physics), meta=(BlueprintSpawnableComponent))
class BIKEGAME_API UKPhysics : public UActorComponent
{
	GENERATED_BODY()

public:
	// Sets default values for this component's properties
	UKPhysics();

protected:
	// Called when the game starts
	virtual void BeginPlay() override;
	
	UPROPERTY(BlueprintReadWrite)
	UPrimitiveComponent* Target;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int Substeps = 32;
	
	// Mass (in kg or your chosen units)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics")
	float Mass = 1.f;

	// Static friction coefficient (mu_s) - friction that can hold an object at rest
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Friction")
	float StaticFrictionCoefficient = 0.6f;

	// Dynamic friction coefficient (mu_k) - friction once it's sliding
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Friction")
	float DynamicFrictionCoefficient = 0.5f;

	// Coefficient of restitution (bounciness)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics")
	float Restitution = 0.5f;

	// Simple linear damping factor
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics")
	float LinearDamping = 0.1f;
	
	// Simple angular damping factor
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics")
	float AngularDamping = 0.1f;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void AsyncPhysicsTickComponent(float DeltaTime, float SimTime) override;
	
private:
	void ResolveCollision(FHitResult& Hit, float DeltaTime);
	
	FVector Velocity;
	FVector AngularVelocity;
	
	TArray<FHitResult> Collisions;
};
