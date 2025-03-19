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
	virtual void PhysicsTick(const double DeltaTime) override;
	double GetKMass() const;
	FDoubleVector GetKLinearVelocity() const;
	FDoubleVector GetKAngularVelocity() const;
	void AddKLinearVelocity(const FDoubleVector& InLinearVelocity);
	void AddKAngularVelocity(const FDoubleVector& InAngularVelocity);

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
	double DynamicFrictionCoefficient = 0.4;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	double RestitutionCoefficient = 0.4;

	// Damping factors.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	double LinearDampingFactor = 0.01;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	double AngularDampingFactor = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	bool Freeze;
	
private:
	void ResolveCollision(FHitResult& Hit, UPrimitiveComponent* PrimitiveComponent);
	
	FDoubleVector LinearVelocity;
	FDoubleVector AngularVelocity;

	FDoubleVector LinearVelocityBucket;
	FDoubleVector AngularVelocityBucket;
};