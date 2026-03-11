#pragma once

#include "CoreMinimal.h"
#include "KPhysicsMeshComponent.h"
#include "KWheelColliderComponent.generated.h"

UCLASS(ClassGroup=(Physics), meta=(BlueprintSpawnableComponent))
class BIKEGAME_API UKWheelColliderComponent : public UKPhysicsMeshComponent
{
	GENERATED_BODY()

public:
	virtual void BeginPlay() override;
	virtual void PhysicsTick(const double DeltaTime) override;

protected:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	double NormalSpringConstant = 200000.0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	double NormalDampingConstant = 1000.0;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	double RimDiameterInches = 18;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	double TireWidthMM = 140;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	double TireAspectRatio = 80;
	
private:
	virtual void ResolveCollision(const double DeltaTime, const FHitResult& Hit) override;
	
	double TireSidewall;
	double RimRadius;
	double TotalWheelRadius;
};
