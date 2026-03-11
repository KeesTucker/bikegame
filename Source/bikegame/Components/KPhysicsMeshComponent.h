#pragma once

#include "CoreMinimal.h"
#include "bikegame/Math/DoubleQuat.h"
#include "bikegame/Subsystems/KPhysicsTickSubsystem.h"
#include "bikegame/Math/DoubleVector.h"
#include "Components/StaticMeshComponent.h"
#include "KPhysicsMeshComponent.generated.h"

UCLASS(ClassGroup=(Physics), meta=(BlueprintSpawnableComponent))
class BIKEGAME_API UKPhysicsMeshComponent : public UStaticMeshComponent, public IKPhysicsTickInterface
{
	GENERATED_BODY()

public:
	UKPhysicsMeshComponent();
	virtual void PhysicsTick(const double DeltaTime) override;
	double GetKMass() const;
	FDoubleVector GetKLocation() const;
	FDoubleQuat GetKOrientation() const;
	FDoubleVector GetKLinearVelocity() const;
	FDoubleVector GetKAngularVelocity() const;
	FDoubleMatrix3X3 GetKWorldInertiaTensor() const;
	void SetKLocation(const FDoubleVector& InLocation);
	void SetKOrientation(const FDoubleQuat& InOrientation);
	void SetKLinearVelocity(const FDoubleVector& InLinearVelocity);
	void SetKAngularVelocity(const FDoubleVector& InAngularVelocity);
	void AddKLinearVelocity(const FDoubleVector& InLinearVelocity);
	void AddKAngularVelocity(const FDoubleVector& InAngularVelocity);

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(EEndPlayReason::Type EndPlayReason) override;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	double Mass = 10;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	double StaticFrictionCoefficient = 0.6;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	double DynamicFrictionCoefficient = 0.4;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	double RestitutionCoefficient = 0.4;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	double LinearDampingFactor = 0.01;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	double AngularDampingFactor = 0;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	bool XAngularFreeze;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	bool YAngularFreeze;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="KPhysics")
	bool ZAngularFreeze;
	
private:
	void ApplyFreeze();
	virtual void ResolveCollision(const double DeltaTime, const FHitResult& Hit);

	FDoubleVector Location;
	FDoubleQuat Orientation;
	
	FDoubleVector LinearVelocity;
	FDoubleVector AngularVelocity;

	FDoubleVector LinearVelocityBucket;
	FDoubleVector AngularVelocityBucket;

	FDoubleMatrix3X3 WorldInertiaTensor;
};