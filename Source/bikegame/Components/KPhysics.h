#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "KPhysics.generated.h"

//---------------------------------------------------------------------
// DVector: A double-precision 3D vector type
//---------------------------------------------------------------------
struct DVector {
	double x, y, z;

	DVector() : x(0.0), y(0.0), z(0.0) {}
	DVector(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
	DVector(const FVector& vec) : x(vec.X), y(vec.Y), z(vec.Z) {}

	// Conversion operator to FVector (Unreal uses float)
	operator FVector() const {
		return FVector(static_cast<float>(x),
					   static_cast<float>(y),
					   static_cast<float>(z));
	}

	DVector operator+(const DVector& other) const {
		return DVector(x + other.x, y + other.y, z + other.z);
	}
	DVector operator-(const DVector& other) const {
		return DVector(x - other.x, y - other.y, z - other.z);
	}
	DVector operator*(double scalar) const {
		return DVector(x * scalar, y * scalar, z * scalar);
	}
	DVector operator/(double scalar) const {
		return DVector(x / scalar, y / scalar, z / scalar);
	}
	DVector& operator+=(const DVector& other) {
		x += other.x; y += other.y; z += other.z;
		return *this;
	}
	DVector& operator-=(const DVector& other) {
		x -= other.x; y -= other.y; z -= other.z;
		return *this;
	}
	double Size() const {
		return std::sqrt(x * x + y * y + z * z);
	}
	DVector GetSafeNormal() const {
		double s = Size();
		return (s > 1e-8) ? (*this / s) : DVector(0.0, 0.0, 0.0);
	}
};

/**
 * UKPhysics is a custom physics simulation component.
 * It applies forces such as gravity, damping, and collision responses on the actor's root primitive component.
 */
UCLASS(ClassGroup=(Physics), meta=(BlueprintSpawnableComponent))
class BIKEGAME_API UKPhysics : public UActorComponent
{
	GENERATED_BODY()

public:
	// Constructor: Sets default values for this component's properties.
	UKPhysics();

protected:
	// Called when the game starts.
	virtual void BeginPlay() override;
	
	// Number of integration substeps to use per frame for higher simulation accuracy.
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float NumHzPhysics = 1000.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics")
	float Mass = 10.f;
	
	// Static friction coefficient (mu_s) for holding an object at rest.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Friction")
	float StaticFrictionCoefficient = 0.6f;

	// Dynamic friction coefficient (mu_k) for friction while sliding.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Friction")
	float DynamicFrictionCoefficient = 0.5f;

	// Coefficient of restitution representing bounciness.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics")
	float RestitutionCoefficient = 0.5f;

	// Linear damping factor to reduce linear velocity over time.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics")
	float LinearDampingFactor = 0.1f;
	
	// Angular damping factor to reduce angular velocity over time.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics")
	float AngularDampingFactor = 0.1f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Friction")
	float SlipBias = 0.001f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Friction")
	float MinInducedSlip = 0.01f;

public:	
	// Called every frame to update the physics simulation.
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	
private:
	// Resolves collision responses when a collision is detected.
	void ResolveCollision(FHitResult& Hit, UPrimitiveComponent* PrimitiveComponent);
	
	// Current linear velocity of the physics body.
	DVector LinearVelocity;
	// Current angular velocity of the physics body.
	DVector AngularVelocity;

	// Cached reference to the actor's root primitive component.
	UPROPERTY()
	UPrimitiveComponent* RootPrimitive;
};
