#include "KPhysicsMeshComponent.h"

#include "bikegame/Types/DoubleMath.h"
#include "bikegame/Types/DoubleMatrix3X3.h"
#include "GameFramework/Actor.h"
#include "Components/PrimitiveComponent.h"

UKPhysicsMeshComponent::UKPhysicsMeshComponent()
{}

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
    if (Freeze)
    {
        return;
    }
    
    // 1) Apply damping.
    LinearVelocity = LinearVelocity - LinearVelocity * (LinearDampingFactor * DeltaTime);
    AngularVelocity = AngularVelocity - AngularVelocity * (AngularDampingFactor * DeltaTime);

    // 2) Apply gravity.
    const double GravityZ = GetWorld()->GetGravityZ();
    LinearVelocity = LinearVelocity + FDoubleVector(0.0, 0.0, GravityZ) * DeltaTime;

    // 2.5) Apply external velocities
    LinearVelocity += LinearVelocityBucket;
    LinearVelocityBucket = FDoubleVector::Zero();
    AngularVelocity += AngularVelocityBucket;
    AngularVelocityBucket = FDoubleVector::Zero();
    
    // 3) Update orientation using angular velocity.
    FQuat CurrentOrientation = GetComponentQuat();
    {
        const FDoubleVector DeltaAngularVelocity = AngularVelocity * DeltaTime;

        if (const double RotationAngle = DeltaAngularVelocity.Size(); RotationAngle > DSmall_Number)
        {
            const FDoubleVector RotationAxis = DeltaAngularVelocity / RotationAngle;
            const double HalfRotationAngle = 0.5 * RotationAngle;
            const double SineHalfAngle = std::sin(HalfRotationAngle);
            const FQuat RotationDeltaQuat(
                static_cast<float>(RotationAxis.X * SineHalfAngle),
                static_cast<float>(RotationAxis.Y * SineHalfAngle),
                static_cast<float>(RotationAxis.Z * SineHalfAngle),
                static_cast<float>(std::cos(HalfRotationAngle))
            );
            CurrentOrientation = RotationDeltaQuat * CurrentOrientation;
            CurrentOrientation.Normalize();
        }
    }

    // 4) Linear movement.
    FHitResult LinearHitResult;
    const FDoubleVector LinearOffset = LinearVelocity * DeltaTime;
    AddWorldOffset(FVector(LinearOffset), true, &LinearHitResult);
    if (LinearHitResult.bBlockingHit)
    {
        ResolveCollision(LinearHitResult, this);
    }

    // 5) Angular movement.
    const FQuat InitialOrientation = GetComponentQuat();
    const FQuat RotationDeltaQuat = CurrentOrientation * InitialOrientation.Inverse();
    FHitResult AngularHitResult;
    AddWorldRotation(RotationDeltaQuat.Rotator(), true, &AngularHitResult);
    if (AngularHitResult.bBlockingHit)
    {
        ResolveCollision(AngularHitResult, this);
    }
}

double UKPhysicsMeshComponent::GetKMass() const
{
    return Mass;
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
    FQuat CurrentOrientation = PrimitiveComponent->GetComponentQuat();
    
    FDoubleVector ContactImpactPoint(Hit.ImpactPoint);
    FDoubleVector ContactOffset = ContactImpactPoint - CenterOfMass;
    
    FDoubleVector ContactRelativeVelocity = LinearVelocity + FDoubleVector::Cross(AngularVelocity, ContactOffset);
    
    FDoubleVector HitNormal(Hit.Normal);
    double RelativeNormalVelocity = FDoubleVector::Dot(ContactRelativeVelocity, HitNormal);
    
    // Convert the local inertia tensor diagonal into a DMatrix3x3.
    FDoubleVector LocalInertiaTensorDiagonal(PrimitiveComponent->BodyInstance.GetBodyInertiaTensor());
    FDoubleMatrix3X3 LocalInertia(
        LocalInertiaTensorDiagonal.X, 0.0, 0.0,
        0.0, LocalInertiaTensorDiagonal.Y, 0.0,
        0.0, 0.0, LocalInertiaTensorDiagonal.Z
    );
    
    // Build the rotation matrix from the current orientation.
    FMatrix RotationMatrixF = FRotationMatrix::Make(CurrentOrientation);
    FDoubleMatrix3X3 R(
        RotationMatrixF.M[0][0], RotationMatrixF.M[0][1], RotationMatrixF.M[0][2],
        RotationMatrixF.M[1][0], RotationMatrixF.M[1][1], RotationMatrixF.M[1][2],
        RotationMatrixF.M[2][0], RotationMatrixF.M[2][1], RotationMatrixF.M[2][2]
    );
    
    // Compute world inertia tensor.
    FDoubleMatrix3X3 WorldInertiaTensor = R * LocalInertia * FDoubleMatrix3X3::Transpose(R);
    FDoubleMatrix3X3 WorldInertiaTensorInverse = FDoubleMatrix3X3::Inverse(WorldInertiaTensor);
    
    // 1) Normal collision impulse.
    if (RelativeNormalVelocity < -DSmall_Number)
    {
        FDoubleVector LeverArmCrossNormal = FDoubleVector::Cross(ContactOffset, HitNormal);
        FDoubleVector InertiaInverseCrossResult = WorldInertiaTensorInverse * LeverArmCrossNormal;
        FDoubleVector RotationalComponent = FDoubleVector::Cross(InertiaInverseCrossResult, ContactOffset);
        
        double RegularizationTerm = 0.0001;
        double EffectiveNormalMass = (1.0 / Mass) + FDoubleVector::Dot(HitNormal, RotationalComponent) + RegularizationTerm;
        
        double NormalImpulseMagnitude = -(1.0 + RestitutionCoefficient) * RelativeNormalVelocity / EffectiveNormalMass;
        FDoubleVector CollisionImpulse = HitNormal * NormalImpulseMagnitude;
        
        LinearVelocity = LinearVelocity + CollisionImpulse / Mass;
        FDoubleVector AngularTorque = FDoubleVector::Cross(ContactOffset, CollisionImpulse);
        FDoubleVector AngularVelocityDelta = WorldInertiaTensorInverse * AngularTorque;
        AngularVelocity = AngularVelocity + AngularVelocityDelta;
        
        // 2) Friction impulse.
        ContactRelativeVelocity = LinearVelocity + FDoubleVector::Cross(AngularVelocity, ContactOffset);
        FDoubleVector PostCollisionNormalVelocity = HitNormal * FDoubleVector::Dot(ContactRelativeVelocity, HitNormal);
        FDoubleVector PostCollisionTangentialVelocity = ContactRelativeVelocity - PostCollisionNormalVelocity;

        if (double TangentialVelocityMagnitude = PostCollisionTangentialVelocity.Size(); TangentialVelocityMagnitude > DSmall_Number)
        {
            FDoubleVector FrictionDirection = PostCollisionTangentialVelocity / TangentialVelocityMagnitude;
            FDoubleVector LeverArmCrossFriction = FDoubleVector::Cross(ContactOffset, FrictionDirection);
            FDoubleVector InertiaInverseCrossFriction = WorldInertiaTensorInverse * LeverArmCrossFriction;
            FDoubleVector RotationalFrictionComponent = FDoubleVector::Cross(InertiaInverseCrossFriction, ContactOffset);

            double EffectiveFrictionMass = 1.0 / Mass + FDoubleVector::Dot(FrictionDirection, RotationalFrictionComponent) + RegularizationTerm;

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

            // Clamp to ensure no invalid values (optional safety measure)
            FinalFrictionImpulseMagnitude = FDoubleMath::Clamp(FinalFrictionImpulseMagnitude, -MaxDynamicImpulse, MaxDynamicImpulse);

            FDoubleVector FrictionImpulse = FrictionDirection * FinalFrictionImpulseMagnitude;

            // Apply the friction impulse to linear and angular velocities
            LinearVelocity = LinearVelocity + FrictionImpulse / Mass;
            FDoubleVector FrictionAngularTorque = FDoubleVector::Cross(ContactOffset, FrictionImpulse);
            FDoubleVector FrictionAngularDelta = WorldInertiaTensorInverse * FrictionAngularTorque;
            AngularVelocity = AngularVelocity + FrictionAngularDelta;
        }
    }
}
