#pragma once

#include "CoreMinimal.h"
#include "bikegame/Math/DoubleQuat.h"
#include "bikegame/Subsystems/KPhysicsTickSubsystem.h"
#include "bikegame/Math/DoubleVector.h"
#include "Components/ActorComponent.h"
#include "KReverseEulerConstraintComponent.generated.h"

class UKPhysicsMeshComponent;

/**
 * UReverseEulerConstraintComponent applies a spring/damper constraint between two UKPhysicsMeshComponents.
 * It computes the relative displacement and velocity, calculates an effective mass, and applies equal and
 * opposite forces to each component using reverse Euler integration.
 */
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent))
class BIKEGAME_API UKReverseEulerConstraintComponent : public UActorComponent, public IKPhysicsTickInterface
{
	GENERATED_BODY()

public:
	UKReverseEulerConstraintComponent();
#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
	virtual void PhysicsTick(const double DeltaTime) override;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(EEndPlayReason::Type EndPlayReason) override;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Constraint")
	FString PhysicsComponentNameA;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Constraint")
	FString PhysicsComponentNameB;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Constraint")
	double LinearSpringConstant = 100.0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Constraint")
	double LinearDampingConstant = 1.0;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Constraint")
	double AngularSpringConstant = 100.0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Constraint")
	double AngularDampingConstant = 1.0;
	
private:
	void Init();
	static FDoubleVector ComputeSpringVelocity(double DeltaTime, const FDoubleVector& RelativeDisplacement,
		const FDoubleVector& RelativeVelocity, double EffectiveMass, double SpringK, double DampingC);

	UPROPERTY(EditAnywhere)
	UKPhysicsMeshComponent* PhysicsComponentA;
	UPROPERTY(EditAnywhere)
	UKPhysicsMeshComponent* PhysicsComponentB;
	UPROPERTY(EditAnywhere)
	FDoubleVector InitialRelativePosition;
	UPROPERTY(EditAnywhere)
	FDoubleQuat InitialRelativeOrientation;
	UPROPERTY(EditAnywhere)
	FDoubleVector InitialDirection;
	UPROPERTY(EditAnywhere)
	double InitialDistance;
};
