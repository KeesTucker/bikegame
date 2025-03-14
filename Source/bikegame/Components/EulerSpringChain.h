// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "EulerSpringChain.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class BIKEGAME_API UEulerSpringChain : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UEulerSpringChain();

protected:
	// Called when the game starts
	virtual void BeginPlay() override;
	UPROPERTY(BlueprintReadWrite)
	UPrimitiveComponent* Target;
	UPROPERTY(BlueprintReadWrite)
	UPrimitiveComponent* Anchor;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float Stiffness = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float Damping = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float CollisionStiffness = 10000000;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float CollisionDamping = 10000;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void AsyncPhysicsTickComponent(float DeltaTime, float SimTime) override;
	UFUNCTION(BlueprintCallable)
	void OnHitCallback(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit);

private:
	FVector PreviousX;
	FVector Velocity;
	FVector Location = FVector(0.f, 0.f, 1000.f);
	FVector GetSpringForce(FVector x, FVector v, float m, float dt, float k, float c);
	FVector GetSpringVelocity(FVector x, FVector v, float m, float dt, float k, float c);

	struct FCollision
	{
		FVector Target;
		FVector Normal;
		//TODO: implement this
	};
	TArray<FVector> CollisionConstraints;
};
