#pragma once

#include "CoreMinimal.h"
#include "bikegame/Math/DoubleQuat.h"
#include "bikegame/Subsystems/KPhysicsTickSubsystem.h"
#include "bikegame/Math/DoubleVector.h"
#include "Components/ActorComponent.h"
#include "KReverseEulerConstraintComponent.generated.h"

class UKPhysicsMeshComponent;

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
	void ApplyLinearSpring(double DeltaTime, const FDoubleVector& PosA, const FDoubleVector& PosB) const;
	void ApplyAngularSpring(double DeltaTime, const FDoubleVector& PosA, const FDoubleVector& PosB,
	                                 const FDoubleQuat& OrientationA, const FDoubleQuat& OrientationB) const;
	FDoubleVector GetAngularSpringVelocity(double DeltaTime, const FDoubleVector& InitialDirection,
	                                       const FDoubleVector& Direction,
	                                       const FDoubleQuat& InitialOrientation, const FDoubleQuat& Orientation, const FDoubleVector& RelativeAngularVelocity) const;
	
	UPROPERTY()
	UKPhysicsMeshComponent* PhysicsComponentA;
	UPROPERTY()
	UKPhysicsMeshComponent* PhysicsComponentB;
	
	FDoubleVector InitialAToBDisplacement;
	FDoubleVector InitialBToADisplacement;
	double InitialDistance;
	FDoubleVector InitialAToBDirection;
	FDoubleVector InitialBToADirection;
	FDoubleQuat InitialAOrientation;
	FDoubleQuat InitialBOrientation;
};
