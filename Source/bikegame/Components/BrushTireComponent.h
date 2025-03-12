#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include <atomic>
#include <thread>

#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "BrushTireComponent.generated.h"

UCLASS( ClassGroup=(Motorcycle), meta=(BlueprintSpawnableComponent) )
class BIKEGAME_API UBrushTireComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UBrushTireComponent();
	virtual void BeginPlay() override;
	virtual void PostInitProperties() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void AsyncPhysicsTickComponent(float DeltaTime, float SimTime) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	UFUNCTION(BlueprintCallable, Category = "Tire Physics")
	FVector GetTireForce() const;

protected:
	UPROPERTY(BlueprintReadWrite)
	UStaticMeshComponent* WheelMesh;
	UPROPERTY(BlueprintReadWrite)
	UPhysicsConstraintComponent* AxleConstraint;
	
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
	float TirePressure = 1000;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float RadialDeflectionStiffness = 1000;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float RadialDeflectionDampingCoefficent = 100;
	
private:
	struct FBristle
	{
		FVector Origin;
		FVector Tip;
		FVector Dir;
		FVector Contact;
		FVector ContactNormal;
		float RadialDeflection;
		float RadialDeflectionDelta;
		bool IsContacting;

		FBristle(const FVector& InOrigin, const FVector& InTip, const FVector& InDir, const FVector& InContact,
			const FVector& InContactNormal, const float InRadialDeflection, const float InRadialDeflectionDelta,
			const bool InIsContacting):
			Origin(InOrigin), Tip(InTip), Dir(InDir), Contact(InContact), ContactNormal(InContactNormal),
			RadialDeflection(InRadialDeflection), RadialDeflectionDelta(InRadialDeflectionDelta),
			IsContacting(InIsContacting)
		{
		}

		FBristle()
			: Origin(),
			  Tip(),
			  Dir(),
			  Contact(),
		      ContactNormal(),
			  RadialDeflection(0),
			  RadialDeflectionDelta(0),
			  IsContacting(false)
		{
		}
	};

	std::thread BrushThread;
	std::atomic<bool> BThreadRunning;
    
	// Thread-safe force storage
	std::atomic<float> Fx;
	std::atomic<float> Fy;
	std::atomic<float> Fz;

	const float InchTocm = 2.54;
	const float mmTocm = 0.1;
	const float cMToM = 0.01;
	
	float RimRadius;
	float RimWidth;
	float TireWidth;
	float TireSidewallHeight;
	float BristleRadialStiffness;

	TArray<TArray<FBristle>>  Bristles;
	
	// Brush model update loop (runs in its own thread)
	void RunBrushModel();
    
	// Computes brush forces
	void ComputeBrushForces(float& OutFx, float& OutFy, float& OutFz) const;
};