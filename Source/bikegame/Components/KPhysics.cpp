// Fill out your copyright notice in the Description page of Project Settings.


#include "KPhysics.h"


// Sets default values for this component's properties
UKPhysics::UKPhysics(): Target(nullptr), Velocity(), AngularVelocity()
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

    DeltaTime = DeltaTime / static_cast<float>(Substeps);
    
    for (int i = 0; i < Substeps; ++i)
    {
        // Simple linear damping
        Velocity -= (Velocity * LinearDamping * DeltaTime);
        // Simple angular damping
        AngularVelocity -= (AngularVelocity * AngularDamping * DeltaTime);

        //--------------------------------------
        // 1) Gravity & Angular Damping
        //--------------------------------------
        // Gravity in Unreal is negative Z, so GetWorld()->GetGravityZ() is negative
        // Use it in a vector
        Velocity += (GetWorld()->GetGravityZ() * FVector(0.f, 0.f, 1.f)) * DeltaTime;

        //--------------------------------------
        // 3) Update Orientation from Angular Velocity
        //--------------------------------------
        FQuat Orientation = Target->GetComponentQuat();
        {
            FVector W = AngularVelocity * DeltaTime;
            float Theta = W.Size();

            if (Theta > KINDA_SMALL_NUMBER)
            {
                FVector Axis = W / Theta;
                float halfTheta = 0.5f * Theta;
                float s = FMath::Sin(halfTheta);

                FQuat DeltaQ(Axis.X * s, Axis.Y * s, Axis.Z * s, FMath::Cos(halfTheta));
                Orientation = DeltaQ * Orientation;
                Orientation.Normalize();
            }
        }

        // For linear motion:
        FHitResult MoveHit;
        Target->AddWorldOffset(Velocity * DeltaTime, /*bSweep=*/true, &MoveHit);
        if (MoveHit.bBlockingHit)
        {
            ResolveCollision(MoveHit, DeltaTime);
        }

        // For angular motion:
        FQuat CurrentQuat = Target->GetComponentQuat();
        FQuat DeltaQuat   = Orientation * CurrentQuat.Inverse();
        FHitResult SweepHit;
        Target->AddWorldRotation(DeltaQuat.Rotator(), /*bSweep=*/true, &SweepHit);
        if (SweepHit.bBlockingHit)
        {
            ResolveCollision(SweepHit, DeltaTime);
        }
    }
}

void UKPhysics::ResolveCollision(FHitResult& Hit, float DeltaTime)
{
    FVector COM = Target->GetCenterOfMass();
    FQuat Orientation = Target->GetComponentQuat();
    
    FVector ImpactPoint = Hit.ImpactPoint;
    FVector r = (ImpactPoint - COM);
    
    // Relative velocity at contact
    FVector vContact = Velocity + FVector::CrossProduct(AngularVelocity, r);

    // Normal velocity component
    float dot = FVector::DotProduct(vContact, Hit.Normal);

    // Build inertia in world space
    FVector LocalInertiaDiag = Target->BodyInstance.GetBodyInertiaTensor();
    FMatrix InertiaLocal = FMatrix::Identity;
    InertiaLocal.M[0][0] = LocalInertiaDiag.X;
    InertiaLocal.M[1][1] = LocalInertiaDiag.Y;
    InertiaLocal.M[2][2] = LocalInertiaDiag.Z;
    
    FMatrix R = FRotationMatrix::Make(Orientation);
    FMatrix InertiaWorld = R * InertiaLocal * R.GetTransposed();
    FMatrix InertiaWorldInv = InertiaWorld.InverseFast();

    //--------------------------
    // 2a) Collision (Normal Impulse)
    //--------------------------
    if (dot < -KINDA_SMALL_NUMBER)
    {
        // Compute the effective mass denominator:
        // Denom = 1/mass + n dot [ (I⁻¹*(r x n)) x r ]
        FVector r_cross_n = FVector::CrossProduct(r, Hit.Normal);
        FVector I_inv_r_cross_n = InertiaWorldInv.TransformVector(r_cross_n);
        FVector rotational_component = FVector::CrossProduct(I_inv_r_cross_n, r);
        float effectiveMass = (1.0f / Mass) + FVector::DotProduct(Hit.Normal, rotational_component);
        
        // Compute the impulse magnitude
        float j = -(1.f + Restitution) * dot / effectiveMass;
        
        // Impulse vector along the normal
        FVector CollisionImpulse = j * Hit.Normal;
        
        // Update linear velocity
        Velocity += CollisionImpulse / Mass;

        DrawDebugDirectionalArrow(
            GetWorld(),
            ImpactPoint,
            ImpactPoint + CollisionImpulse,
            10.f,
            FColor::Red,
            false,
            0.1f
        );

        // Update angular velocity using the impulse torque:
        // Torque = r x CollisionImpulse, and Δω = I⁻¹ * Torque
        FVector Torque = FVector::CrossProduct(r, CollisionImpulse);
        FVector AngularImpulse = InertiaWorldInv.TransformVector(Torque);
        AngularVelocity += AngularImpulse;
    }

    //--------------------------
    // 2b) Static + Coulomb Friction
    //--------------------------
    // Tangential velocity
    FVector Normal = Hit.Normal;
    FVector vN = FVector::DotProduct(vContact, Normal) * Normal;
    FVector vT = vContact - vN;  // tangential component
    float vTmag = vT.Size();

    // We'll do a simple approach: assume normal force = mass * g
    // (Ignoring slopes or multiple collisions.)
    float normalForce = Mass * -GetWorld()->GetGravityZ() * 
               FMath::Abs(FVector::DotProduct(Hit.Normal, FVector::UpVector));
    
    // If tangential speed is extremely small, attempt static friction
    // We'll compute how much friction is needed to bring tangential velocity to zero in one frame.
    // frictionNeeded = - (Mass * vT) / DeltaTime
    // If that is <= mu_s * normalForce, we remain fully at rest. Otherwise, we apply dynamic friction.
    FVector frictionForce = FVector::ZeroVector;
    if (vTmag < KINDA_SMALL_NUMBER)
    {
        // Probably (nearly) at rest in the tangential direction
        float maxStaticFriction = StaticFrictionCoefficient * normalForce;

        // Force needed to zero tangential velocity this frame
        FVector frictionNeeded = -(Mass * vT) / (DeltaTime * Substeps);
        float frictionNeededMag = frictionNeeded.Size();

        if (frictionNeededMag <= maxStaticFriction)
        {
            // We can stay completely at rest
            frictionForce = frictionNeeded; 
        }
        else
        {
            // Exceeds static friction => use dynamic friction
            // Magnitude = mu_k * normalForce, direction opposite vT
            // Safeguard against zero length
            if (vTmag > SMALL_NUMBER)
            {
                frictionForce = -(DynamicFrictionCoefficient * normalForce) * (vT / vTmag);
            }
        }
    }
    else
    {
        // Sliding => dynamic friction
        float maxDynamicFriction = DynamicFrictionCoefficient * normalForce;
        frictionForce = -(maxDynamicFriction) * (vT / vTmag);
    }

    // Apply friction (force) to linear velocity
    Velocity += (frictionForce / Mass) * DeltaTime;

    // Draw friction arrow (optional). Scaling it so arrow length ~ friction impulse
    DrawDebugDirectionalArrow(
        GetWorld(),
        ImpactPoint,
        ImpactPoint + frictionForce * (0.01f),  // reduce for visibility
        10.f, 
        FColor::Green, 
        false,
        0.1f
    );

    // Apply friction torque to angular velocity
    FVector frictionTorque = FVector::CrossProduct(r, frictionForce);
    FVector frictionAlpha = InertiaWorldInv.TransformVector(frictionTorque);
    AngularVelocity += frictionAlpha * DeltaTime;
}

void UKPhysics::AsyncPhysicsTickComponent(float DeltaTime, float SimTime)
{
	Super::AsyncPhysicsTickComponent(DeltaTime, SimTime);
}