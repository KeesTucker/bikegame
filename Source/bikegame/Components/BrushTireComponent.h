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
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	UStaticMeshComponent* WheelMesh;
	
private:
	std::thread BrushThread;
	std::atomic<bool> BThreadRunning;
    
	// Thread-safe force storage
	std::atomic<float> Fx;
	std::atomic<float> Fy;
	std::atomic<float> Fz; 
	
	// Brush model update loop (runs in its own thread)
	void RunBrushModel();
    
	// Computes brush forces
	void ComputeBrushForces(float& OutFx, float& OutFy, float& OutFz) const;
};