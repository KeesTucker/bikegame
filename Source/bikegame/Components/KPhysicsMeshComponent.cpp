#include "KPhysicsMeshComponent.h"

#include "bikegame/Math/DoubleMath.h"
#include "bikegame/Math/DoubleMatrix3X3.h"
#include "bikegame/Math/DoubleQuat.h"
#include "bikegame/Math/ReverseEulerSpring.h"
#include "GameFramework/Actor.h"
#include "Components/PrimitiveComponent.h"

UKPhysicsMeshComponent::UKPhysicsMeshComponent(): FreezeXOrientation(false), FreezeYOrientation(false),
                                                  FreezeZOrientation(false)
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
    WorldInertiaTensor = FDoubleMatrix3X3::WorldInertiaTensor(GetInertiaTensor(), Orientation.ToRotationMatrix());

    const FDoubleVector InitLocation = Location;
    const FDoubleQuat InitOrientation = Orientation;
    
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
    
    // 4) Finally set location and rotation.
    // We first move the component to the original location and orientation (this stops parent movement effecting the sweep,
    // and allows us to avoid using absolute movement which is painful to deal with setup).
    SetWorldLocationAndRotation(FVector(InitLocation), FQuat(InitOrientation));

    // Then we sweep to the new location and orientation
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

FDoubleMatrix3X3 UKPhysicsMeshComponent::GetKWorldInertiaTensor() const
{
    return WorldInertiaTensor;
}

void UKPhysicsMeshComponent::AddKLinearVelocity(const FDoubleVector& InLinearVelocity)
{
    LinearVelocityBucket += InLinearVelocity;
}

void UKPhysicsMeshComponent::AddKAngularVelocity(const FDoubleVector& InAngularVelocity)
{
    AngularVelocityBucket += InAngularVelocity;
}

void UKPhysicsMeshComponent::ApplyFreeze(const double DeltaTime)
{
    constexpr double FreezeK = 10000000.0;
    constexpr double FreezeC = 100000.0;
    if (FreezeXOrientation)
    {
        const double XError = -1 * FDoubleQuat::GetTwistAngleRadians(Orientation, FDoubleVector::Forward());
        const FDoubleVector Error = FDoubleVector(XError, 0.0, 0.0);
        const FDoubleVector RelativeVelocity = FDoubleVector(AngularVelocity.X, 0.0, 0.0);
        const double Inertia = FDoubleVector::Dot(FDoubleVector::Forward(), WorldInertiaTensor * FDoubleVector::Forward());
        AddKAngularVelocity(FReverseEulerSpring::ComputeSpringVelocity(DeltaTime, Error, RelativeVelocity, Inertia, FreezeK, FreezeC));
    }
    if (FreezeYOrientation)
    {
        const double YError = -1 * FDoubleQuat::GetTwistAngleRadians(Orientation, FDoubleVector::Right());
        const FDoubleVector Error = FDoubleVector(0.0, YError, 0.0);
        const FDoubleVector RelativeVelocity = FDoubleVector(0.0, AngularVelocity.Y, 0.0);
        const double Inertia = FDoubleVector::Dot(FDoubleVector::Right(), WorldInertiaTensor * FDoubleVector::Right());
        AddKAngularVelocity(FReverseEulerSpring::ComputeSpringVelocity(DeltaTime, Error, RelativeVelocity, Inertia, FreezeK, FreezeC));
    }
    if (FreezeZOrientation)
    {
        const double ZError = -1 * FDoubleQuat::GetTwistAngleRadians(Orientation, FDoubleVector::Up());
        const FDoubleVector Error = FDoubleVector(0.0, 0.0, ZError);
        const FDoubleVector RelativeVelocity = FDoubleVector(0.0, 0.0, AngularVelocity.Z);
        const double Inertia = FDoubleVector::Dot(FDoubleVector::Up(), WorldInertiaTensor * FDoubleVector::Up());
        AddKAngularVelocity(FReverseEulerSpring::ComputeSpringVelocity(DeltaTime, Error, RelativeVelocity, Inertia, FreezeK, FreezeC));
    }
}

void UKPhysicsMeshComponent::ResolveCollision(const FHitResult& Hit, const UPrimitiveComponent* PrimitiveComponent)
{
    const FDoubleVector CenterOfMass(PrimitiveComponent->GetCenterOfMass());

    const FDoubleVector ContactImpactPoint(Hit.ImpactPoint);
    const FDoubleVector ContactOffset = ContactImpactPoint - CenterOfMass;
    
    FDoubleVector ContactRelativeVelocity = LinearVelocity + FDoubleVector::Cross(AngularVelocity, ContactOffset);

    const FDoubleVector HitNormal(Hit.Normal);

    // Move back out of the overlapping collider
    Location += HitNormal * Hit.PenetrationDepth;

    const double RelativeNormalVelocity = FDoubleVector::Dot(ContactRelativeVelocity, HitNormal);

    const FDoubleMatrix3X3 WorldInertiaTensorInverse = FDoubleMatrix3X3::Inverse(WorldInertiaTensor);
    
    // 1) Normal collision impulse.
    if (RelativeNormalVelocity < -DSmallNumber)
    {
        const FDoubleVector LeverArmCrossNormal = FDoubleVector::Cross(ContactOffset, HitNormal);
        const FDoubleVector InertiaInverseCrossResult = WorldInertiaTensorInverse * LeverArmCrossNormal;
        const  FDoubleVector RotationalComponent = FDoubleVector::Cross(InertiaInverseCrossResult, ContactOffset);
        
        const  double EffectiveNormalMass = 1.0 / Mass + FDoubleVector::Dot(HitNormal, RotationalComponent);
        
        const double NormalImpulseMagnitude = -(1.0 + RestitutionCoefficient) * RelativeNormalVelocity / EffectiveNormalMass;
        const FDoubleVector CollisionImpulse = HitNormal * NormalImpulseMagnitude;
        LinearVelocity += CollisionImpulse / Mass;
        
        const FDoubleVector AngularTorque = FDoubleVector::Cross(ContactOffset, CollisionImpulse);
        const FDoubleVector AngularVelocityDelta = WorldInertiaTensorInverse * AngularTorque;
        AngularVelocity += AngularVelocityDelta;
        
        // 2) Friction impulse.
        ContactRelativeVelocity = LinearVelocity + FDoubleVector::Cross(AngularVelocity, ContactOffset);
        const FDoubleVector PostCollisionNormalVelocity = HitNormal * FDoubleVector::Dot(ContactRelativeVelocity, HitNormal);
        const FDoubleVector PostCollisionTangentialVelocity = ContactRelativeVelocity - PostCollisionNormalVelocity;
        const double TangentialVelocityMagnitude = PostCollisionTangentialVelocity.Size();
        const FDoubleVector FrictionDirection = PostCollisionTangentialVelocity / TangentialVelocityMagnitude;
        const FDoubleVector LeverArmCrossFriction = FDoubleVector::Cross(ContactOffset, FrictionDirection);
        const FDoubleVector InertiaInverseCrossFriction = WorldInertiaTensorInverse * LeverArmCrossFriction;
        const FDoubleVector RotationalFrictionComponent = FDoubleVector::Cross(InertiaInverseCrossFriction, ContactOffset);

        const double EffectiveFrictionMass = 1.0 / Mass + FDoubleVector::Dot(FrictionDirection, RotationalFrictionComponent);

        const double CandidateFrictionImpulse = -FDoubleVector::Dot(PostCollisionTangentialVelocity, FrictionDirection) / EffectiveFrictionMass;

        const double MaxStaticImpulse = StaticFrictionCoefficient * std::abs(NormalImpulseMagnitude);
        const double MaxDynamicImpulse = DynamicFrictionCoefficient * std::abs(NormalImpulseMagnitude);

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
        
        const FDoubleVector FrictionImpulse = FrictionDirection * FinalFrictionImpulseMagnitude;

        // Apply the friction impulse to linear and angular velocities
        LinearVelocity += FrictionImpulse / Mass;
        
        const FDoubleVector FrictionAngularTorque = FDoubleVector::Cross(ContactOffset, FrictionImpulse);
        const FDoubleVector FrictionAngularDelta = WorldInertiaTensorInverse * FrictionAngularTorque;
        AngularVelocity += FrictionAngularDelta;
    }
}
