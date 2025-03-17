#include "KPhysics.h"
#include "GameFramework/Actor.h"
#include "Components/PrimitiveComponent.h"
#include "DrawDebugHelpers.h" // Optional: for visualizing impulses

/////////////////////////////////////////////////////
// Constructor and Initialization
/////////////////////////////////////////////////////

UKPhysics::UKPhysics() 
	: LinearVelocity()
	, AngularVelocity()
	, RootPrimitive(nullptr)
{
	// Enable ticking for this component.
	PrimaryComponentTick.bCanEverTick = true;
}

// Called when the game starts.
void UKPhysics::BeginPlay()
{
	Super::BeginPlay();
	
	// Cache the actor's root component (assumed to be a UPrimitiveComponent).
	RootPrimitive = Cast<UPrimitiveComponent>(GetOwner()->GetRootComponent());
	if (!RootPrimitive)
	{
		UE_LOG(LogTemp, Warning, TEXT("KPhysics: Actor's RootComponent is not a UPrimitiveComponent."));
		return;
	}

	RootPrimitive->SetMassOverrideInKg(NAME_None, Mass, true);
	RootPrimitive->RecreatePhysicsState();
	RootPrimitive->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	RootPrimitive->SetCollisionResponseToAllChannels(ECR_Block);
	RootPrimitive->SetSimulatePhysics(false);
}

/////////////////////////////////////////////////////
// Main Physics Simulation Update
/////////////////////////////////////////////////////

void UKPhysics::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (!RootPrimitive || !GetWorld())
    {
        return;
    }
	
    const float TargetSubstepDeltaTime = 1.f / NumHzPhysics;
    // Calculate the number of substeps needed to cover the frame's DeltaTime.
    int DynamicSubsteps = FMath::CeilToInt(DeltaTime / TargetSubstepDeltaTime);
    // Ensure there is at least one substep.
    DynamicSubsteps = FMath::Max(1, DynamicSubsteps);
    
    // Recalculate the delta time for each integration substep.
    const float SubstepDeltaTime = DeltaTime / static_cast<float>(DynamicSubsteps);

    for (int SubstepIndex = 0; SubstepIndex < DynamicSubsteps; ++SubstepIndex)
    {
        // ----------------------------------------------------------------
        // 1) Apply Damping to Linear and Angular Velocities
        // ----------------------------------------------------------------
        LinearVelocity -= (LinearVelocity * LinearDampingFactor * SubstepDeltaTime);
        AngularVelocity -= (AngularVelocity * AngularDampingFactor * SubstepDeltaTime);

        // ----------------------------------------------------------------
        // 2) Apply Gravity
        // ----------------------------------------------------------------
        LinearVelocity += (GetWorld()->GetGravityZ() * FVector(0.f, 0.f, 1.f)) * SubstepDeltaTime;

        // ----------------------------------------------------------------
        // 3) Update Target Orientation Based on Angular Velocity
        // ----------------------------------------------------------------
        FQuat CurrentOrientation = RootPrimitive->GetComponentQuat();
        {
            FVector DeltaAngularVelocity = AngularVelocity * SubstepDeltaTime;
            float RotationAngle = DeltaAngularVelocity.Size();

            if (RotationAngle > KINDA_SMALL_NUMBER)
            {
                FVector RotationAxis = DeltaAngularVelocity / RotationAngle;
                float HalfRotationAngle = 0.5f * RotationAngle;
                float SineHalfAngle = FMath::Sin(HalfRotationAngle);
                FQuat RotationDeltaQuat(
                    RotationAxis.X * SineHalfAngle,
                    RotationAxis.Y * SineHalfAngle,
                    RotationAxis.Z * SineHalfAngle,
                    FMath::Cos(HalfRotationAngle)
                );
                CurrentOrientation = RotationDeltaQuat * CurrentOrientation;
                CurrentOrientation.Normalize();
            }
        }

        // ----------------------------------------------------------------
        // 4) Apply Linear Motion and Detect Collisions
        // ----------------------------------------------------------------
        FHitResult LinearHitResult;
        RootPrimitive->AddWorldOffset(LinearVelocity * SubstepDeltaTime, true, &LinearHitResult);
        if (LinearHitResult.bBlockingHit)
        {
            ResolveCollision(LinearHitResult, RootPrimitive);
        }

        // ----------------------------------------------------------------
        // 5) Apply Angular Motion and Detect Collisions
        // ----------------------------------------------------------------
        FQuat InitialOrientation = RootPrimitive->GetComponentQuat();
        FQuat RotationDeltaQuat = CurrentOrientation * InitialOrientation.Inverse();
        FHitResult AngularHitResult;
        RootPrimitive->AddWorldRotation(RotationDeltaQuat.Rotator(), true, &AngularHitResult);
        if (AngularHitResult.bBlockingHit)
        {
            ResolveCollision(AngularHitResult, RootPrimitive);
        }
    }
}


/////////////////////////////////////////////////////
// Collision Resolution
/////////////////////////////////////////////////////

void UKPhysics::ResolveCollision(FHitResult& Hit, UPrimitiveComponent* PrimitiveComponent)
{
    // Retrieve the center of mass and current orientation of the physics body.
    FVector CenterOfMass = PrimitiveComponent->GetCenterOfMass();
    FQuat CurrentOrientation = PrimitiveComponent->GetComponentQuat();
    
    // Compute the offset from the center of mass to the contact impact point.
    FVector ContactImpactPoint = Hit.ImpactPoint;
    FVector ContactOffset = ContactImpactPoint - CenterOfMass;
    
    // Calculate the relative velocity at the contact point.
    FVector ContactRelativeVelocity = LinearVelocity + FVector::CrossProduct(AngularVelocity, ContactOffset);
    
    // Determine the velocity component along the collision normal.
    float RelativeNormalVelocity = FVector::DotProduct(ContactRelativeVelocity, Hit.Normal);
    
    // Build the local inertia tensor matrix using the body's inertia tensor diagonal.
    FVector LocalInertiaTensorDiagonal = PrimitiveComponent->BodyInstance.GetBodyInertiaTensor();
    FMatrix LocalInertiaTensorMatrix = FMatrix::Identity;
    LocalInertiaTensorMatrix.M[0][0] = LocalInertiaTensorDiagonal.X;
    LocalInertiaTensorMatrix.M[1][1] = LocalInertiaTensorDiagonal.Y;
    LocalInertiaTensorMatrix.M[2][2] = LocalInertiaTensorDiagonal.Z;
    
    // Transform the local inertia tensor to world space.
    FMatrix RotationMatrix = FRotationMatrix::Make(CurrentOrientation);
    FMatrix WorldInertiaTensor = RotationMatrix * LocalInertiaTensorMatrix * RotationMatrix.GetTransposed();
    FMatrix WorldInertiaTensorInverse = WorldInertiaTensor.InverseFast();
    
    // ----------------------------------------------------------------
    // 1) Resolve Normal Collision Impulse
    // ----------------------------------------------------------------
    if (RelativeNormalVelocity < -KINDA_SMALL_NUMBER)
    {
        FVector LeverArmCrossNormal = FVector::CrossProduct(ContactOffset, Hit.Normal);
        FVector InertiaInverseCrossResult = WorldInertiaTensorInverse.TransformVector(LeverArmCrossNormal);
        FVector RotationalComponent = FVector::CrossProduct(InertiaInverseCrossResult, ContactOffset);
        
        // Add small regularization to prevent instability when dividing
        float RegularizationTerm = 0.0001f;
        float EffectiveNormalMass = (1.0f / Mass) + FVector::DotProduct(Hit.Normal, RotationalComponent) + RegularizationTerm;
        
        float NormalImpulseMagnitude = -(1.f + RestitutionCoefficient) * RelativeNormalVelocity / EffectiveNormalMass;
        FVector CollisionImpulse = NormalImpulseMagnitude * Hit.Normal;
        
        LinearVelocity += CollisionImpulse / Mass;
        FVector AngularTorque = FVector::CrossProduct(ContactOffset, CollisionImpulse);
        AngularVelocity += WorldInertiaTensorInverse.TransformVector(AngularTorque);
        
        // ----------------------------------------------------------------
        // 2) Resolve Friction Impulse
        // ----------------------------------------------------------------
        ContactRelativeVelocity = LinearVelocity + FVector::CrossProduct(AngularVelocity, ContactOffset);
        FVector PostCollisionNormalVelocity = FVector::DotProduct(ContactRelativeVelocity, Hit.Normal) * Hit.Normal;
        FVector PostCollisionTangentialVelocity = ContactRelativeVelocity - PostCollisionNormalVelocity;
        float TangentialVelocityMagnitude = PostCollisionTangentialVelocity.Size();

        // Introduce a small bias to prevent exactly zero slip
        const float SlipBias = 0.01f; 
        if (TangentialVelocityMagnitude < KINDA_SMALL_NUMBER)
        {
            FVector InducedSlipVelocity = FVector::CrossProduct(AngularVelocity, ContactOffset);
            // Blend instead of fully replacing the velocity
            PostCollisionTangentialVelocity = FMath::Lerp(PostCollisionTangentialVelocity, InducedSlipVelocity, 0.5f);
            TangentialVelocityMagnitude = PostCollisionTangentialVelocity.Size();
        }

        if (TangentialVelocityMagnitude > KINDA_SMALL_NUMBER)
        {
            FVector FrictionDirection = PostCollisionTangentialVelocity / TangentialVelocityMagnitude;
            FVector LeverArmCrossFriction = FVector::CrossProduct(ContactOffset, FrictionDirection);
            FVector InertiaInverseCrossFriction = WorldInertiaTensorInverse.TransformVector(LeverArmCrossFriction);
            FVector RotationalFrictionComponent = FVector::CrossProduct(InertiaInverseCrossFriction, ContactOffset);

            // Regularized effective friction mass
            float EffectiveFrictionMass = (1.0f / Mass) + FVector::DotProduct(FrictionDirection, RotationalFrictionComponent) + RegularizationTerm;

            float CandidateFrictionImpulse = -FVector::DotProduct(PostCollisionTangentialVelocity, FrictionDirection) / EffectiveFrictionMass;

            // Smooth transition between static and dynamic friction using a blend function
            float MaxStaticImpulse = StaticFrictionCoefficient * FMath::Abs(NormalImpulseMagnitude);
            float MaxDynamicImpulse = DynamicFrictionCoefficient * FMath::Abs(NormalImpulseMagnitude);

            float FrictionBlendFactor = FMath::Clamp(TangentialVelocityMagnitude / SlipBias, 0.0f, 1.0f);
            float BlendedFrictionImpulse = FMath::Lerp(MaxStaticImpulse, MaxDynamicImpulse, FrictionBlendFactor);

            float FinalFrictionImpulseMagnitude = 0.f;
            if (FMath::Abs(CandidateFrictionImpulse) <= BlendedFrictionImpulse)
            {
                FinalFrictionImpulseMagnitude = CandidateFrictionImpulse;
            }
            else
            {
                FinalFrictionImpulseMagnitude = -BlendedFrictionImpulse * FMath::Sign(FVector::DotProduct(PostCollisionTangentialVelocity, FrictionDirection));
            }

            FVector FrictionImpulse = FinalFrictionImpulseMagnitude * FrictionDirection;
            LinearVelocity += FrictionImpulse / Mass;
            FVector FrictionAngularTorque = FVector::CrossProduct(ContactOffset, FrictionImpulse);
            AngularVelocity += WorldInertiaTensorInverse.TransformVector(FrictionAngularTorque);
        }
    }
}
