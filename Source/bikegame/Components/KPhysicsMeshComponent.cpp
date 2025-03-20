#include "KPhysicsMeshComponent.h"

#include "bikegame/Math/DoubleMath.h"
#include "bikegame/Math/DoubleMatrix3X3.h"
#include "bikegame/Math/DoubleQuat.h"
#include "GameFramework/Actor.h"
#include "Components/PrimitiveComponent.h"

UKPhysicsMeshComponent::UKPhysicsMeshComponent(): Freeze(false)
{
}

void UKPhysicsMeshComponent::BeginPlay()
{
    Super::BeginPlay();
    
    if (UKPhysicsTickSubsystem* TickSubsystem = GetWorld()->GetSubsystem<UKPhysicsTickSubsystem>())
    {
        TickSubsystem->RegisterComponent(this);
    }
    
    SetMassOverrideInKg(NAME_None, Mass, true);
    RecreatePhysicsState();
    SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    SetCollisionResponseToAllChannels(ECR_Block);
    SetSimulatePhysics(false);

    Location = FDoubleVector(GetComponentLocation());
    Orientation = FDoubleQuat(GetComponentQuat());

    SetUsingAbsoluteLocation(true);
    SetUsingAbsoluteRotation(true);
}

void UKPhysicsMeshComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (UKPhysicsTickSubsystem* TickSubsystem = GetWorld()->GetSubsystem<UKPhysicsTickSubsystem>())
    {
        TickSubsystem->UnregisterComponent(this);
    }
    Super::EndPlay(EndPlayReason);
}

void UKPhysicsMeshComponent::PhysicsTick(const double DeltaTime)
{
    if (!Freeze)
    {
        // Apply external velocities
        LinearVelocity += LinearVelocityBucket;
        LinearVelocityBucket = FDoubleVector::Zero();
        AngularVelocity += AngularVelocityBucket;
        AngularVelocityBucket = FDoubleVector::Zero();
        
        // Apply damping.
        LinearVelocity *= FMath::Exp(-LinearDampingFactor * DeltaTime);
        AngularVelocity *= FMath::Exp(-AngularDampingFactor * DeltaTime);

        // Apply gravity.
        const double GravityZ = GetWorld()->GetGravityZ();
        LinearVelocity += FDoubleVector(0.0, 0.0, GravityZ) * DeltaTime;

        // Update location using linear velocity
        Location += LinearVelocity * DeltaTime;
        
        // Update orientation using angular velocity.
        const FDoubleVector DeltaAngularVelocity = AngularVelocity * DeltaTime;

        if (const double RotationAngle = DeltaAngularVelocity.Size(); RotationAngle > DSmallNumber)
        {
            const FDoubleVector RotationAxis = DeltaAngularVelocity / RotationAngle;
            const double HalfRotationAngle = 0.5 * RotationAngle;
            const double SineHalfAngle = std::sin(HalfRotationAngle);
            const FDoubleQuat RotationDeltaQuat(
                RotationAxis.X * SineHalfAngle,
                RotationAxis.Y * SineHalfAngle,
                RotationAxis.Z * SineHalfAngle,
                std::cos(HalfRotationAngle)
            );
            Orientation = RotationDeltaQuat * Orientation;
            Orientation.Normalize();
        }
    }
    
    // 4) Finally set location and rotation.
    FHitResult TransformationHitResult;
    SetWorldLocationAndRotation(FVector(Location), FQuat(Orientation), true, &TransformationHitResult);
    if (TransformationHitResult.bBlockingHit)
    {
        ResolveCollision(TransformationHitResult, this);
    }
}

double UKPhysicsMeshComponent::GetKMass() const
{
    return Mass;
}

FDoubleVector UKPhysicsMeshComponent::GetKLocation() const
{
    return Location;
}

FDoubleQuat UKPhysicsMeshComponent::GetKOrientation() const
{
    return Orientation;
}

FDoubleVector UKPhysicsMeshComponent::GetKLinearVelocity() const
{
    return LinearVelocity;
}

FDoubleVector UKPhysicsMeshComponent::GetKAngularVelocity() const
{
    return AngularVelocity;
}

void UKPhysicsMeshComponent::AddKLinearVelocity(const FDoubleVector& InLinearVelocity)
{
    LinearVelocityBucket += InLinearVelocity;
}

void UKPhysicsMeshComponent::AddKAngularVelocity(const FDoubleVector& InAngularVelocity)
{
    AngularVelocityBucket += InAngularVelocity;
}

void UKPhysicsMeshComponent::ResolveCollision(FHitResult& Hit, UPrimitiveComponent* PrimitiveComponent)
{
    FDoubleVector CenterOfMass(PrimitiveComponent->GetCenterOfMass());
    
    FDoubleVector ContactImpactPoint(Hit.ImpactPoint);
    FDoubleVector ContactOffset = ContactImpactPoint - CenterOfMass;
    
    FDoubleVector ContactRelativeVelocity = LinearVelocity + FDoubleVector::Cross(AngularVelocity, ContactOffset);
    
    FDoubleVector HitNormal(Hit.Normal);

    // Move back out of the overlapping collider
    Location += HitNormal * Hit.PenetrationDepth;
    
    double RelativeNormalVelocity = FDoubleVector::Dot(ContactRelativeVelocity, HitNormal);
    
    // Convert the local inertia tensor diagonal into a DMatrix3x3.
    FDoubleVector LocalInertiaTensorDiagonal(PrimitiveComponent->BodyInstance.GetBodyInertiaTensor());
    FDoubleMatrix3X3 LocalInertia(
        LocalInertiaTensorDiagonal.X, 0.0, 0.0,
        0.0, LocalInertiaTensorDiagonal.Y, 0.0,
        0.0, 0.0, LocalInertiaTensorDiagonal.Z
    );
    
    // Build the rotation matrix from the current orientation.
    FDoubleMatrix3X3 RotationMatrix = Orientation.ToRotationMatrix();
    
    // Compute world inertia tensor.
    FDoubleMatrix3X3 WorldInertiaTensor = RotationMatrix * LocalInertia * FDoubleMatrix3X3::Transpose(RotationMatrix);
    FDoubleMatrix3X3 WorldInertiaTensorInverse = FDoubleMatrix3X3::Inverse(WorldInertiaTensor);
    
    // 1) Normal collision impulse.
    if (RelativeNormalVelocity < -DSmallNumber)
    {
        FDoubleVector LeverArmCrossNormal = FDoubleVector::Cross(ContactOffset, HitNormal);
        FDoubleVector InertiaInverseCrossResult = WorldInertiaTensorInverse * LeverArmCrossNormal;
        FDoubleVector RotationalComponent = FDoubleVector::Cross(InertiaInverseCrossResult, ContactOffset);
        
        double EffectiveNormalMass = 1.0 / Mass + FDoubleVector::Dot(HitNormal, RotationalComponent);
        
        double NormalImpulseMagnitude = -(1.0 + RestitutionCoefficient) * RelativeNormalVelocity / EffectiveNormalMass;
        FDoubleVector CollisionImpulse = HitNormal * NormalImpulseMagnitude;
        LinearVelocity += CollisionImpulse / Mass;
        
        FDoubleVector AngularTorque = FDoubleVector::Cross(ContactOffset, CollisionImpulse);
        FDoubleVector AngularVelocityDelta = WorldInertiaTensorInverse * AngularTorque;
        AngularVelocity += AngularVelocityDelta;
        
        // 2) Friction impulse.
        ContactRelativeVelocity = LinearVelocity + FDoubleVector::Cross(AngularVelocity, ContactOffset);
        FDoubleVector PostCollisionNormalVelocity = HitNormal * FDoubleVector::Dot(ContactRelativeVelocity, HitNormal);
        FDoubleVector PostCollisionTangentialVelocity = ContactRelativeVelocity - PostCollisionNormalVelocity;
        double TangentialVelocityMagnitude = PostCollisionTangentialVelocity.Size();
        FDoubleVector FrictionDirection = PostCollisionTangentialVelocity / TangentialVelocityMagnitude;
        FDoubleVector LeverArmCrossFriction = FDoubleVector::Cross(ContactOffset, FrictionDirection);
        FDoubleVector InertiaInverseCrossFriction = WorldInertiaTensorInverse * LeverArmCrossFriction;
        FDoubleVector RotationalFrictionComponent = FDoubleVector::Cross(InertiaInverseCrossFriction, ContactOffset);

        double EffectiveFrictionMass = 1.0 / Mass + FDoubleVector::Dot(FrictionDirection, RotationalFrictionComponent);

        double CandidateFrictionImpulse = -FDoubleVector::Dot(PostCollisionTangentialVelocity, FrictionDirection) / EffectiveFrictionMass;

        double MaxStaticImpulse = StaticFrictionCoefficient * std::abs(NormalImpulseMagnitude);
        double MaxDynamicImpulse = DynamicFrictionCoefficient * std::abs(NormalImpulseMagnitude);

        double FinalFrictionImpulseMagnitude;

        // Determine whether to use static or dynamic friction
        if (std::abs(CandidateFrictionImpulse) <= MaxStaticImpulse)
        {
            FinalFrictionImpulseMagnitude = CandidateFrictionImpulse;
        }
        else
        {
            // Apply dynamic friction opposite to the tangential direction
            FinalFrictionImpulseMagnitude = -MaxDynamicImpulse;
        }
        
        FDoubleVector FrictionImpulse = FrictionDirection * FinalFrictionImpulseMagnitude;

        // Apply the friction impulse to linear and angular velocities
        LinearVelocity += FrictionImpulse / Mass;
        
        FDoubleVector FrictionAngularTorque = FDoubleVector::Cross(ContactOffset, FrictionImpulse);
        FDoubleVector FrictionAngularDelta = WorldInertiaTensorInverse * FrictionAngularTorque;
        AngularVelocity += FrictionAngularDelta;
    }
}
