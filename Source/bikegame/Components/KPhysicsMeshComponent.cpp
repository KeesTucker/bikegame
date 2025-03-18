#include "KPhysicsMeshComponent.h"

#include "bikegame/Types/DoubleMath.h"
#include "bikegame/Types/DoubleMatrix3X3.h"
#include "bikegame/Types/DoubleMatrix3X3.h"
#include "GameFramework/Actor.h"
#include "Components/PrimitiveComponent.h"

UKPhysicsMeshComponent::UKPhysicsMeshComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UKPhysicsMeshComponent::BeginPlay()
{
    Super::BeginPlay();
    
    SetMassOverrideInKg(NAME_None, Mass, true);
    RecreatePhysicsState();
    SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    SetCollisionResponseToAllChannels(ECR_Block);
    SetSimulatePhysics(false);
}

void UKPhysicsMeshComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (!GetWorld())
    {
        return;
    }
    
    // Determine substeps for more accurate simulation.
    const double TargetSubstepDeltaTime = 1.0 / NumHzPhysics;
    int DynamicSubsteps = static_cast<int>(std::ceil(DeltaTime / TargetSubstepDeltaTime));
    DynamicSubsteps = FMath::Max(1, DynamicSubsteps);
    const double SubstepDeltaTime = DeltaTime / static_cast<double>(DynamicSubsteps);

    for (int SubstepIndex = 0; SubstepIndex < DynamicSubsteps; ++SubstepIndex)
    {
        // 1) Apply damping.
        LinearVelocity = LinearVelocity - LinearVelocity * (LinearDampingFactor * SubstepDeltaTime);
        AngularVelocity = AngularVelocity - AngularVelocity * (AngularDampingFactor * SubstepDeltaTime);

        // 2) Apply gravity.
        double GravityZ = GetWorld()->GetGravityZ();
        LinearVelocity = LinearVelocity + FDoubleVector(0.0, 0.0, GravityZ) * SubstepDeltaTime;

        // 3) Update orientation using angular velocity.
        FQuat CurrentOrientation = GetComponentQuat();
        {
            FDoubleVector DeltaAngularVelocity = AngularVelocity * SubstepDeltaTime;
            double RotationAngle = DeltaAngularVelocity.Size();

            if (RotationAngle > DSmall_Number)
            {
                FDoubleVector RotationAxis = DeltaAngularVelocity / RotationAngle;
                double HalfRotationAngle = 0.5 * RotationAngle;
                double SineHalfAngle = std::sin(HalfRotationAngle);
                FQuat RotationDeltaQuat(
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
        FDoubleVector LinearOffset = LinearVelocity * SubstepDeltaTime;
        AddWorldOffset(FVector(LinearOffset), true, &LinearHitResult);
        if (LinearHitResult.bBlockingHit)
        {
            ResolveCollision(LinearHitResult, this);
        }

        // 5) Angular movement.
        FQuat InitialOrientation = GetComponentQuat();
        FQuat RotationDeltaQuat = CurrentOrientation * InitialOrientation.Inverse();
        FHitResult AngularHitResult;
        AddWorldRotation(RotationDeltaQuat.Rotator(), true, &AngularHitResult);
        if (AngularHitResult.bBlockingHit)
        {
            ResolveCollision(AngularHitResult, this);
        }
    }
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
    
    double det = FDoubleMatrix3X3::Determinant(WorldInertiaTensor);
    if (std::abs(det) < DSmall_Number)
    {
        UE_LOG(LogTemp, Warning, TEXT("Singular inertia tensor matrix encountered in collision resolution."));
        return;
    }
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
        double TangentialVelocityMagnitude = PostCollisionTangentialVelocity.Size();

        if (TangentialVelocityMagnitude < DSmall_Number)
        {
            FDoubleVector InducedSlipVelocity = FDoubleVector::Cross(AngularVelocity, ContactOffset);
            if (InducedSlipVelocity.Size() < MinInducedSlip)
            {
                InducedSlipVelocity = InducedSlipVelocity.GetSafeNormal() * MinInducedSlip;
            }
            PostCollisionTangentialVelocity = FDoubleVector::Lerp(PostCollisionTangentialVelocity, InducedSlipVelocity, InducedSlipBlend);
            TangentialVelocityMagnitude = PostCollisionTangentialVelocity.Size();
        }

        if (TangentialVelocityMagnitude > DSmall_Number)
        {
            FDoubleVector FrictionDirection = PostCollisionTangentialVelocity / TangentialVelocityMagnitude;
            FDoubleVector LeverArmCrossFriction = FDoubleVector::Cross(ContactOffset, FrictionDirection);
            FDoubleVector InertiaInverseCrossFriction = WorldInertiaTensorInverse * LeverArmCrossFriction;
            FDoubleVector RotationalFrictionComponent = FDoubleVector::Cross(InertiaInverseCrossFriction, ContactOffset);

            double EffectiveFrictionMass = 1.0 / Mass + FDoubleVector::Dot(FrictionDirection, RotationalFrictionComponent) + RegularizationTerm;

            double CandidateFrictionImpulse = -FDoubleVector::Dot(PostCollisionTangentialVelocity, FrictionDirection) / EffectiveFrictionMass;

            double MaxStaticImpulse = StaticFrictionCoefficient * std::abs(NormalImpulseMagnitude);
            double MaxDynamicImpulse = DynamicFrictionCoefficient * std::abs(NormalImpulseMagnitude);

            double FrictionBlendFactor = FDoubleMath::Clamp(TangentialVelocityMagnitude / SlipBias, 0.0, 1.0);
            double BlendedFrictionImpulse = FDoubleMath::Lerp(MaxStaticImpulse, MaxDynamicImpulse, FrictionBlendFactor);

            double FinalFrictionImpulseMagnitude = FDoubleMath::Clamp(CandidateFrictionImpulse, -BlendedFrictionImpulse, BlendedFrictionImpulse);

            FDoubleVector FrictionImpulse = FrictionDirection * FinalFrictionImpulseMagnitude;
            LinearVelocity = LinearVelocity + FrictionImpulse / Mass;
            FDoubleVector FrictionAngularTorque = FDoubleVector::Cross(ContactOffset, FrictionImpulse);
            FDoubleVector FrictionAngularDelta = WorldInertiaTensorInverse * FrictionAngularTorque;
            AngularVelocity = AngularVelocity + FrictionAngularDelta;
        }
    }
}
