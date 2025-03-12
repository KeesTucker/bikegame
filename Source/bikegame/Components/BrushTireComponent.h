#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include <atomic>
#include <thread>

#include "BrushTireComponent.generated.h"

UCLASS( ClassGroup=(Motorcycle), meta=(BlueprintSpawnableComponent) )
class BIKEGAME_API UBrushTireComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UBrushTireComponent();
	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	UFUNCTION(BlueprintCallable, Category = "Tire Physics")
	FVector GetTireForce() const;

protected:
	UPROPERTY(BlueprintReadWrite)
	UStaticMeshComponent* WheelMesh;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, DisplayName="Rim Diameter (Inches)")
	float RimDiameterInches = 17;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, DisplayName="Rim Width (Inches)")
	float RimWidthInches = 3.5;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, DisplayName="Tire Width (mm)")
	float TireWidthMm = 120;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, DisplayName="Tire Aspect Ratio")
	float TireAspectRatio = 60;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float BrushSegmentsPerWheel = 32;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float BristlesPerSegment = 8;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float ForcePerContactPoint = 8;
	
private:
	std::thread BrushThread;
	std::atomic<bool> BThreadRunning;
    
	// Thread-safe force storage
	std::atomic<float> Fx;
	std::atomic<float> Fy;
	std::atomic<float> Fz;

	const float InchToCm = 2.54;
	const float MmToCm = 0.1;
	
	float RimRadius;
	float RimWidth;
	float TireWidth;
	float TireSidewallHeight;

	// Brush model update loop (runs in its own thread)
	void RunBrushModel();
    
	// Computes brush forces
	void ComputeBrushForces(float& OutFx, float& OutFy, float& OutFz) const;
};