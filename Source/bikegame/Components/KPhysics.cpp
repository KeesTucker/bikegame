// Fill out your copyright notice in the Description page of Project Settings.


#include "KPhysics.h"


// Sets default values for this component's properties
UKPhysics::UKPhysics()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void UKPhysics::BeginPlay()
{
	Super::BeginPlay();
	SetActive(true);
	SetAsyncPhysicsTickEnabled(true);
}


// Called every frame
void UKPhysics::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (!Target || !GetWorld())
    {
        return;
    }
	
    Velocity += GetWorld()->GetGravityZ() * FVector(0.f, 0.f, 1.0f) * DeltaTime;
	
    for (FHitResult& Hit : Collisions)
    {
        float Dot = FVector::DotProduct(Velocity, Hit.Normal);
        if (Dot < 0.0f)
        {
        	FVector LocalInertiaDiag = Target->BodyInstance.GetBodyInertiaTensor();
        	FMatrix InertiaLocal = FMatrix::Identity;
        	InertiaLocal.M[0][0] = LocalInertiaDiag.X;
        	InertiaLocal.M[1][1] = LocalInertiaDiag.Y;
        	InertiaLocal.M[2][2] = LocalInertiaDiag.Z;
        	FMatrix R = FRotationMatrix::Make(Orientation.Rotator());
        	FMatrix InertiaWorld = R * InertiaLocal * R.GetTransposed();
        	FMatrix InertiaWorldInv = InertiaWorld.InverseFast();
        	
        	
            // Debug arrow for the impact normal
            DrawDebugDirectionalArrow(
                GetWorld(),
                Hit.Location,
                Hit.Location + (Hit.ImpactNormal * 100.f),
                10.f, 
                FColor::Red, 
                false,
                0.1f
            );
        	
            FVector ParallelComponent = Dot * Hit.Normal;
            Velocity -= (1.f + Restitution) * ParallelComponent;  // bounce factor
        	
            Location -= (Hit.PenetrationDepth * Hit.ImpactNormal);
        	
            float Mass = Target->GetMass();
            FVector CollisionImpulse = -(1.f + Restitution) * ParallelComponent * Mass;
        	
            FVector r = (Hit.Location - Location); 
            FVector Torque = FVector::CrossProduct(r, CollisionImpulse);
        	FVector Alpha = InertiaWorldInv.TransformVector(Torque);
        	AngularVelocity += Alpha * DeltaTime;

        	// Friction
        	
        	FVector Normal = Hit.Normal;
        	FVector VelocityNormalPart = (FVector::DotProduct(Velocity, Normal)) * Normal; 
        	FVector VelocityTangentPart = Velocity - VelocityNormalPart;

        	FVector FrictionForce = -(FrictionCoefficient) * VelocityTangentPart;
        	
        	float MassInv = 1.0f / Mass;
        	Velocity += (FrictionForce * MassInv) * DeltaTime;
        	
        	FVector FrictionTorque = FVector::CrossProduct(r, FrictionForce);
        	FVector FrictionAlpha = InertiaWorldInv.TransformVector(FrictionTorque);
        	AngularVelocity += FrictionAlpha * DeltaTime; 
        }
    }

    // Clear collision array for next frame
    Collisions.Empty();

    // 3) Update linear position
    Location += Velocity * DeltaTime;

    // 4) Update orientation via quaternion integration
    //    We assume AngularVelocity is in radians/sec.
    //    If you’re storing it in degrees/sec, convert here with * (PI/180.f).
    {
        // Angular displacement this frame
        FVector W = AngularVelocity * DeltaTime; // total rotation in radians
        float Theta = W.Size();                  // angle magnitude

    	if (Theta > KINDA_SMALL_NUMBER)
    	{
    		FVector Axis = W / Theta;
    		float HalfTheta = 0.5f * Theta;
    		float s = FMath::Sin(HalfTheta);

    		FQuat DeltaQ(Axis.X * s, Axis.Y * s, Axis.Z * s, FMath::Cos(HalfTheta));
    		Orientation = DeltaQ * Orientation;
    		Orientation.Normalize();
    	}
    }

    // 5) Apply final transform
    Target->SetWorldLocation(Location);
    // If your “Target” is e.g. a StaticMeshComponent, it accepts a quaternion:
    Target->SetWorldRotation(Orientation);

	Target->SetPhysicsLinearVelocity(Velocity);
	Target->SetPhysicsAngularVelocityInRadians(AngularVelocity);
}

void UKPhysics::AsyncPhysicsTickComponent(float DeltaTime, float SimTime)
{
	Super::AsyncPhysicsTickComponent(DeltaTime, SimTime);
}

void UKPhysics::OnHitCallback(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp,
	FVector NormalImpulse, const FHitResult& Hit)
{
	Collisions.Add(Hit);
}