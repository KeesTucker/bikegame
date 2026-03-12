#include "KPhysicsMeshComponent.h"

#include "bikegame/Math/DoubleMath.h"
#include "bikegame/Math/DoubleMatrix3X3.h"
#include "bikegame/Math/DoubleQuat.h"
#include "GameFramework/Actor.h"
#include "Components/PrimitiveComponent.h"
#include "DrawDebugHelpers.h"
#include "bikegame/Settings/KPhysicsSettings.h"

UKPhysicsMeshComponent::UKPhysicsMeshComponent(): XAngularFreeze(false), YAngularFreeze(false),
                                                  ZAngularFreeze(false)
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
    
    LinearVelocity += LinearVelocityBucket;
    LinearVelocityBucket = FDoubleVector::Zero();
    AngularVelocity += AngularVelocityBucket;
    AngularVelocityBucket = FDoubleVector::Zero();
    
    // exp(-λΔt) is the exact solution to dv/dt = -λv so damping is stable at any timestep,
    // naive v *= (1 - λΔt) blows up when the physics runs fast.
    LinearVelocity *= FMath::Exp(-LinearDampingFactor * DeltaTime);
    AngularVelocity *= FMath::Exp(-AngularDampingFactor * DeltaTime);

    const double GravityZ = GetWorld()->GetGravityZ();
    LinearVelocity += FDoubleVector(0.0, 0.0, GravityZ) * DeltaTime;

    ApplyFreeze();
    
    // Explicit Euler, integrating dx/dt = v. Simple and cheap, works fine at 1000Hz.
    Location += LinearVelocity * DeltaTime;

    // Same deal for rotation but you can't just add to a quaternion. Build a delta quat from the
    // axis-angle of angular displacement (w * dt) and multiply it on. Comes from integrating dq/dt = 0.5*w*q.
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
    
    // We first move the component to the original location and orientation (this stops parent movement effecting the sweep,
    // and allows us to avoid using absolute movement which is painful to deal with setup).
    SetWorldLocationAndRotation(FVector(InitLocation), FQuat(InitOrientation));

    // Then we sweep to the new location and orientation
    FHitResult TransformationHitResult;
    SetWorldLocationAndRotation(FVector(Location), FQuat(Orientation), true, &TransformationHitResult);
    if (TransformationHitResult.bBlockingHit)
    {
        ResolveCollision(DeltaTime, TransformationHitResult);
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

void UKPhysicsMeshComponent::SetKLocation(const FDoubleVector& InLocation)
{
    Location = InLocation;
}

void UKPhysicsMeshComponent::SetKOrientation(const FDoubleQuat& InOrientation)
{
    Orientation = InOrientation;
}

void UKPhysicsMeshComponent::SetKLinearVelocity(const FDoubleVector& InLinearVelocity)
{
    LinearVelocity = InLinearVelocity;
}

void UKPhysicsMeshComponent::SetKAngularVelocity(const FDoubleVector& InAngularVelocity)
{
    AngularVelocity = InAngularVelocity;
}

void UKPhysicsMeshComponent::AddKLinearVelocity(const FDoubleVector& InLinearVelocity)
{
    LinearVelocityBucket += InLinearVelocity;
}

void UKPhysicsMeshComponent::AddKAngularVelocity(const FDoubleVector& InAngularVelocity)
{
    AngularVelocityBucket += InAngularVelocity;
}

void UKPhysicsMeshComponent::ApplyFreeze()
{
    if (XAngularFreeze)
    {
        AngularVelocity.X = 0.0;
    }
    if (YAngularFreeze)
    {
        AngularVelocity.Y = 0.0;
    }
    if (ZAngularFreeze)
    {
        AngularVelocity.Z = 0.0;
    }
}

void UKPhysicsMeshComponent::ResolveCollision(const double DeltaTime, const FHitResult& Hit)
{
    const FDoubleVector CenterOfMass(GetCenterOfMass());

    const FDoubleVector ContactImpactPoint(Hit.ImpactPoint);
    const FDoubleVector ContactOffset = ContactImpactPoint - CenterOfMass;
    
    FDoubleVector ContactRelativeVelocity = LinearVelocity + FDoubleVector::Cross(AngularVelocity, ContactOffset);

    const FDoubleVector HitNormal(Hit.Normal);

    // Move back out of the overlapping collider
    Location += HitNormal * Hit.PenetrationDepth;

    if (GetDefault<UKPhysicsSettings>()->bShowCollisionDiagnostics)
    {
        DrawDebugDirectionalArrow(GetWorld(), FVector(ContactImpactPoint), FVector(ContactImpactPoint) + FVector(HitNormal) * 20.0f, 10.0f, FColor::Yellow, false, 0.3f, 0.f, 2.0f);
    }

    const double RelativeNormalVelocity = FDoubleVector::Dot(ContactRelativeVelocity, HitNormal);

    const FDoubleMatrix3X3 WorldInertiaTensorInverse = FDoubleMatrix3X3::Inverse(WorldInertiaTensor);
    
    if (RelativeNormalVelocity < -DSmallNumber)
    {
        // How much does an impulse here actually move the body along the hit normal?
        // Not just 1/mass, the lever arm means some of it goes into rotation instead.
        // Same idea as F=ma but projected onto the contact normal and accounting for the inertia tensor.
        const FDoubleVector LeverArmCrossNormal = FDoubleVector::Cross(ContactOffset, HitNormal);
        const FDoubleVector InertiaInverseCrossResult = WorldInertiaTensorInverse * LeverArmCrossNormal;
        const  FDoubleVector RotationalComponent = FDoubleVector::Cross(InertiaInverseCrossResult, ContactOffset);

        const  double EffectiveNormalMass = 1.0 / Mass + FDoubleVector::Dot(HitNormal, RotationalComponent);

        // Standard impulse formula, derived by integrating the contact constraint over the collision.
        // Restitution 0 = dead stop, 1 = perfectly elastic.
        const double NormalImpulseMagnitude = -(1.0 + RestitutionCoefficient) * RelativeNormalVelocity / EffectiveNormalMass;
        const FDoubleVector CollisionImpulse = HitNormal * NormalImpulseMagnitude;
        LinearVelocity += CollisionImpulse / Mass;
        if (GetDefault<UKPhysicsSettings>()->bShowCollisionDiagnostics)
        {
            DrawDebugDirectionalArrow(GetWorld(), FVector(ContactImpactPoint), FVector(ContactImpactPoint) + FVector(CollisionImpulse) * 0.001f, 10.0f, FColor::Green, false, 0.3f, 0.f, 2.0f);
        }

        const FDoubleVector AngularTorque = FDoubleVector::Cross(ContactOffset, CollisionImpulse);
        const FDoubleVector AngularVelocityDelta = WorldInertiaTensorInverse * AngularTorque;
        AngularVelocity += AngularVelocityDelta;
        
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

        // Coulomb friction. If the impulse needed to kill sliding is within the static friction cone we grab,
        // otherwise cap it at kinetic friction and let it slide.
        const double MaxStaticImpulse = StaticFrictionCoefficient * std::abs(NormalImpulseMagnitude);
        const double MaxDynamicImpulse = DynamicFrictionCoefficient * std::abs(NormalImpulseMagnitude);

        double FinalFrictionImpulseMagnitude;

        if (std::abs(CandidateFrictionImpulse) <= MaxStaticImpulse)
        {
            FinalFrictionImpulseMagnitude = CandidateFrictionImpulse;
        }
        else
        {
            FinalFrictionImpulseMagnitude = -MaxDynamicImpulse;
        }
        
        const FDoubleVector FrictionImpulse = FrictionDirection * FinalFrictionImpulseMagnitude;

        LinearVelocity += FrictionImpulse / Mass;
        if (GetDefault<UKPhysicsSettings>()->bShowCollisionDiagnostics)
        {
            DrawDebugDirectionalArrow(GetWorld(), FVector(ContactImpactPoint), FVector(ContactImpactPoint) + FVector(FrictionImpulse) * 0.1f, 10.0f, FColor::Orange, false, 0.3f, 0.f, 2.0f);
        }

        const FDoubleVector FrictionAngularTorque = FDoubleVector::Cross(ContactOffset, FrictionImpulse);
        const FDoubleVector FrictionAngularDelta = WorldInertiaTensorInverse * FrictionAngularTorque;
        AngularVelocity += FrictionAngularDelta;
    }
}
