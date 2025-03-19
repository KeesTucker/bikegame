#pragma once

#include "CoreMinimal.h"
#include "bikegame/Subsystems/KPhysicsTickSubsystem.h"
#include "bikegame/Types/DoubleVector.h"
#include "Components/StaticMeshComponent.h"
#include "KPhysicsMeshComponent.generated.h"

/**
 * @class UKPhysicsMeshComponent
 * @brief A component that extends UStaticMeshComponent to simulate advanced physics interactions.
 *
 * This component integrates custom physics simulation logic and enables a high-fidelity physics
 * system with customizable parameters such as mass, friction coefficients, damping factors, and induced slip.
 * It also implements the IKPhysicsTickInterface for per-frame physics updates.
 */
UCLASS(ClassGroup=(Physics), meta=(BlueprintSpawnableComponent))
class BIKEGAME_API UKPhysicsMeshComponent : public UStaticMeshComponent, public IKPhysicsTickInterface
{
	GENERATED_BODY()

public:
	UKPhysicsMeshComponent();

protected:
	// Called when the game starts.
	virtual void BeginPlay() override;
	virtual void EndPlay(EEndPlayReason::Type EndPlayReason) override;
	
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
	virtual void PhysicsTick(const double DeltaTime) override;
	
private:
	void ResolveCollision(FHitResult& Hit, UPrimitiveComponent* PrimitiveComponent);
	
	FDoubleVector LinearVelocity;
	FDoubleVector AngularVelocity;
};
