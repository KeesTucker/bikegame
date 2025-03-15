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
	float FrictionCoefficient = 5.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float Restitution = 0.5f;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void AsyncPhysicsTickComponent(float DeltaTime, float SimTime) override;
	UFUNCTION(BlueprintCallable)
	void OnHitCallback(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit);

private:
	FVector Velocity;
	FVector AngularVelocity;
	FVector Location = FVector(0.f, 0.f, 1000.f);
	FQuat Orientation = FQuat::Identity;
	
	TArray<FHitResult> Collisions;
};
