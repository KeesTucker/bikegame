// Fill out your copyright notice in the Description page of Project Settings.


#include "EulerSpringChain.h"

#include "MaterialHLSLTree.h"

// Sets default values for this component's properties
UEulerSpringChain::UEulerSpringChain(): Target(nullptr), Anchor(nullptr), PreviousX(), Velocity()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}

// Called when the game starts
void UEulerSpringChain::BeginPlay()
{
	Super::BeginPlay();
	SetActive(true);
	SetAsyncPhysicsTickEnabled(true);
}


// Called every frame
void UEulerSpringChain::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!Target || !GetWorld())
	{
		return;
	}

	/*float DeltaTimeSub = DeltaTime / 10.0f;
	for (int i = 0; i < 10; i++)
	{
		
		Velocity += GetWorld()->GetGravityZ() * FVector(0.f, 0.f, 1.0f) * DeltaTimeSub;
		
		float deflection = Location.Z * -1.0f;
		/*if (deflection > 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("Deflection: %f"), deflection);
			float EffectiveStiffness = Stiffness;
			float EffectiveDamping = Damping;
			if (deflection > 10.0f)
			{
				EffectiveStiffness = CollisionStiffness;
				EffectiveDamping = CollisionDamping;
			}
		
			Velocity = GetSpringVelocity(Location, Velocity, Target->GetMass(), DeltaTimeSub, EffectiveStiffness, EffectiveDamping);
		}#1#
		
		Location += Velocity * DeltaTimeSub;
	}*/
	Velocity += GetWorld()->GetGravityZ() * FVector(0.f, 0.f, 1.0f) * DeltaTime;
	Location += Velocity * DeltaTime;
	Target->SetWorldLocation(Location);
}

void UEulerSpringChain::AsyncPhysicsTickComponent(float DeltaTime, float SimTime)
{
	Super::AsyncPhysicsTickComponent(DeltaTime, SimTime);
}

void UEulerSpringChain::OnHitCallback(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp,
	FVector NormalImpulse, const FHitResult& Hit)
{
	if (!HitComp || !OtherComp)
	{
		return;
	}

	UE_LOG(LogTemp, Warning, TEXT("Impulse: %f"), NormalImpulse.Size());
	
	Velocity = FVector::Zero();
	Location += Hit.Normal * Hit.PenetrationDepth * 2.0f;
}

FVector UEulerSpringChain::GetSpringForce(FVector x, FVector v, float m, float dt, float k, float c)
{
	// Construct the implicit integration system
	float a11 = 1.0f + (c * dt / m);
	float a12 = (k * dt / m);
	float a21 = -dt;
	float a22 = 1.0f;
    
	// Solve for new velocity using implicit integration
	float det = (a11 * a22) - (a12 * a21);
	if (fabs(det) < 1e-6) return FVector::Zero(); // Avoid division by zero

	float inv_a11 = a22 / det;
	float inv_a12 = -a12 / det;
	//float inv_a21 = -a21 / det;
	//float inv_a22 = a11 / det;

	FVector v_new = inv_a11 * v + inv_a12 * x;
	// Optionally, compute new relative displacement if needed:
	// FVector x_new = inv_a21 * v + inv_a22 * x;

	// Calculate the spring force (impulse over the time step)
	FVector F = m * (v_new - v) / dt;
	return F;
}

FVector UEulerSpringChain::GetSpringVelocity(FVector x, FVector v, float m, float dt, float k, float c)
{
	// Construct the implicit integration system
	float a11 = 1.0f + (c * dt / m);
	float a12 = (k * dt / m);
	float a21 = -dt;
	float a22 = 1.0f;
    
	// Solve for new velocity using implicit integration
	float det = (a11 * a22) - (a12 * a21);
	if (fabs(det) < 1e-6) return FVector::Zero(); // Avoid division by zero

	float inv_a11 = a22 / det;
	float inv_a12 = -a12 / det;
	//float inv_a21 = -a21 / det;
	//float inv_a22 = a11 / det;

	FVector v_new = inv_a11 * v + inv_a12 * x;
	return v_new;
}