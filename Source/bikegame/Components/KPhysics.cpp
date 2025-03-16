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
        Target->AddWorldOffset(Velocity * DeltaTime, true, &MoveHit);
        if (MoveHit.bBlockingHit)
        {
            ResolveCollision(MoveHit, DeltaTime);
        }

        // For angular motion:
        FQuat CurrentQuat = Target->GetComponentQuat();
        FQuat DeltaQuat   = Orientation * CurrentQuat.Inverse();
        FHitResult SweepHit;
        Target->AddWorldRotation(DeltaQuat.Rotator(), true, &SweepHit);
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
    
    // Relative velocity at the contact point
    FVector vContact = Velocity + FVector::CrossProduct(AngularVelocity, r);
    
    // Compute the normal component of the relative velocity
    float dot = FVector::DotProduct(vContact, Hit.Normal);
    
    // Build inertia tensor in world space
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
        // Compute effective mass for the collision impulse:
        // effectiveMass = 1/mass + n • [ (I⁻¹*(r×n))×r ]
        FVector r_cross_n = FVector::CrossProduct(r, Hit.Normal);
        FVector I_inv_r_cross_n = InertiaWorldInv.TransformVector(r_cross_n);
        FVector rotational_component = FVector::CrossProduct(I_inv_r_cross_n, r);
        float effectiveMassNormal = (1.0f / Mass) + FVector::DotProduct(Hit.Normal, rotational_component);
        
        // Compute the normal impulse magnitude:
        // j = -(1+Restitution) * (v_rel · n) / effectiveMass
        float j = -(1.f + Restitution) * dot / effectiveMassNormal;
        
        // Apply the collision impulse to linear and angular velocities
        FVector CollisionImpulse = j * Hit.Normal;
        Velocity += CollisionImpulse / Mass;
        FVector Torque = FVector::CrossProduct(r, CollisionImpulse);
        AngularVelocity += InertiaWorldInv.TransformVector(Torque);
        
        //--------------------------
        // 2b) Impulse-based Friction
        //--------------------------
        // Recompute relative velocity after applying normal impulse
        vContact = Velocity + FVector::CrossProduct(AngularVelocity, r);
        // Separate the normal and tangential components
        FVector vN_post = FVector::DotProduct(vContact, Hit.Normal) * Hit.Normal;
        FVector vT_post = vContact - vN_post;
        float vT_post_mag = vT_post.Size();
        
        // Only apply friction if there is a significant tangential velocity
        if (vT_post_mag > KINDA_SMALL_NUMBER)
        {
            // Friction direction (tangent at contact)
            FVector frictionDir = vT_post / vT_post_mag;
            
            // Compute effective mass for friction:
            // effectiveMassFriction = 1/mass + f • [ (I⁻¹*(r×f))×r ]
            FVector r_cross_f = FVector::CrossProduct(r, frictionDir);
            FVector I_inv_r_cross_f = InertiaWorldInv.TransformVector(r_cross_f);
            FVector friction_rot_component = FVector::CrossProduct(I_inv_r_cross_f, r);
            float effectiveMassFriction = (1.0f / Mass) + FVector::DotProduct(frictionDir, friction_rot_component);
            
            // Candidate friction impulse magnitude to cancel the tangential velocity
            float j_f_candidate = - FVector::DotProduct(vT_post, frictionDir) / effectiveMassFriction;
            
            // Determine maximum friction impulses based on the normal impulse magnitude.
            // Typically, the maximum static friction impulse is μ_s * |j|,
            // and for sliding, dynamic friction uses μ_k * |j|.
            float maxStaticImpulse = StaticFrictionCoefficient * FMath::Abs(j);
            float maxDynamicImpulse = DynamicFrictionCoefficient * FMath::Abs(j);
            
            float j_f = 0.f;
            // If the candidate impulse is within the static friction limit, use it.
            if (FMath::Abs(j_f_candidate) <= maxStaticImpulse)
            {
                j_f = j_f_candidate;
            }
            else
            {
                // Otherwise, use dynamic friction (clamped impulse)
                j_f = -maxDynamicImpulse * FMath::Sign(FVector::DotProduct(vT_post, frictionDir));
            }
            
            FVector frictionImpulse = j_f * frictionDir;
            
            // Apply friction impulse to linear and angular velocities
            Velocity += frictionImpulse / Mass;
            FVector frictionTorque = FVector::CrossProduct(r, frictionImpulse);
            AngularVelocity += InertiaWorldInv.TransformVector(frictionTorque);
        }
    }
}

void UKPhysics::AsyncPhysicsTickComponent(float DeltaTime, float SimTime)
{
	Super::AsyncPhysicsTickComponent(DeltaTime, SimTime);
}