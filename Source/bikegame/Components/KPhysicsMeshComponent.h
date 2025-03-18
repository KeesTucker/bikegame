#pragma once

#include "CoreMinimal.h"
#include "bikegame/Types/DVector.h"
#include "Components/StaticMeshComponent.h"
#include "KPhysicsMeshComponent.generated.h"

/**
 * UKPhysics is a custom physics simulation component.
 * It now inherits from UStaticMeshComponent and applies physics directly to itself.
 */
UCLASS(ClassGroup=(Physics), meta=(BlueprintSpawnableComponent))
class BIKEGAME_API UKPhysicsMeshComponent : public UStaticMeshComponent
{
	GENERATED_BODY()

public:
	UKPhysicsMeshComponent();

protected:
	// Called when the game starts.
	virtual void BeginPlay() override;
	
	// Number of integration substeps per frame for improved simulation accuracy.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	double NumHzPhysics = 1000;

	// Mass in kilograms.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	double Mass = 10;
	
	// Friction and restitution properties.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	double StaticFrictionCoefficient = 0.6;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	double DynamicFrictionCoefficient = 0.5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	double RestitutionCoefficient = 0.5;

	// Damping factors.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	double LinearDampingFactor = 0.01;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	double AngularDampingFactor = 0;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	double SlipBias = 0.001;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	double MinInducedSlip = 0.01;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	double InducedSlipBlend = 0.1;

public:	
	// Called every frame to update the physics simulation.
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	
private:
	// Handles collision response when a collision is detected.
	void ResolveCollision(FHitResult& Hit, UPrimitiveComponent* PrimitiveComponent);
	
	// Current velocities.
	DVector LinearVelocity;
	DVector AngularVelocity;
};
