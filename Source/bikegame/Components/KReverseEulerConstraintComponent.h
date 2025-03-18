#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "KReverseEulerConstraintComponent.generated.h"

class UKPhysicsMeshComponent;

/**
 * UReverseEulerConstraintComponent applies a spring/damper constraint between two UKPhysicsMeshComponents.
 * It computes the relative displacement and velocity, calculates an effective mass, and applies equal and
 * opposite forces to each component using reverse Euler integration.
 */
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent))
class BIKEGAME_API UReverseEulerConstraintComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UReverseEulerConstraintComponent();

protected:
	virtual void BeginPlay() override;

public:
	// Called every frame.
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** First physics component in the constraint */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Constraint")
	UKPhysicsMeshComponent* PhysicsComponentA;

	/** Second physics component in the constraint */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Constraint")
	UKPhysicsMeshComponent* PhysicsComponentB;

	/** Spring (stiffness) constant */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Constraint")
	float SpringConstant = 100.0f;

	/** Damping constant */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Constraint")
	float DampingConstant = 1.0f;

private:
	/**
	 * Computes the spring force using an implicit reverse Euler integration scheme for two bodies.
	 *
	 * @param RelativeDisplacement The displacement vector from A to B.
	 * @param RelativeVelocity The relative velocity vector (vB - vA).
	 * @param EffectiveMass The effective mass of the two-body system.
	 * @param DeltaTime The time step.
	 * @param SpringK The spring constant.
	 * @param DampingC The damping constant.
	 * @return The computed force to be applied to PhysicsComponentA (PhysicsComponentB receives the negative).
	 */
	FVector ComputeSpringForce(const FVector& RelativeDisplacement, const FVector& RelativeVelocity, float EffectiveMass, float DeltaTime, float SpringK, float DampingC) const;
};
