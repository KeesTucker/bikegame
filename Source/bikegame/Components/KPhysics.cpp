#include "KPhysics.h"
#include "GameFramework/Actor.h"
#include "Components/PrimitiveComponent.h"

/////////////////////////////////////////////////////
// Constructor and Initialization
/////////////////////////////////////////////////////

UKPhysics::UKPhysics() 
    : RootPrimitive(nullptr)
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UKPhysics::BeginPlay()
{
    Super::BeginPlay();
    
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

void UKPhysics::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (!RootPrimitive || !GetWorld())
    {
        return;
    }
    
    // Use double for substep timing
    const double TargetSubstepDeltaTime = 1.0 / NumHzPhysics;
    int DynamicSubsteps = static_cast<int>(std::ceil(DeltaTime / TargetSubstepDeltaTime));
    DynamicSubsteps = FMath::Max(1, DynamicSubsteps);
    const double SubstepDeltaTime = DeltaTime / static_cast<double>(DynamicSubsteps);

    for (int SubstepIndex = 0; SubstepIndex < DynamicSubsteps; ++SubstepIndex)
    {
        // 1) Damping (using double math)
        LinearVelocity = LinearVelocity - LinearVelocity * (LinearDampingFactor * SubstepDeltaTime);
        AngularVelocity = AngularVelocity - AngularVelocity * (AngularDampingFactor * SubstepDeltaTime);

        // 2) Gravity: convert Unreal’s gravity (float) to double
        double GravityZ = GetWorld()->GetGravityZ();
        LinearVelocity = LinearVelocity + DVector(0.0, 0.0, GravityZ) * SubstepDeltaTime;

        // 3) Update Orientation using angular velocity (compute in double, then cast to FQuat)
        FQuat CurrentOrientation = RootPrimitive->GetComponentQuat();
        {
            DVector DeltaAngularVelocity = AngularVelocity * SubstepDeltaTime;
            double RotationAngle = DeltaAngularVelocity.Size();

            if (RotationAngle > DSmall_Number)
            {
                DVector RotationAxis = DeltaAngularVelocity / RotationAngle;
                double HalfRotationAngle = 0.5 * RotationAngle;
                double SineHalfAngle = std::sin(HalfRotationAngle);
                FQuat RotationDeltaQuat(
                    static_cast<float>(RotationAxis.x * SineHalfAngle),
                    static_cast<float>(RotationAxis.y * SineHalfAngle),
                    static_cast<float>(RotationAxis.z * SineHalfAngle),
                    static_cast<float>(std::cos(HalfRotationAngle))
                );
                CurrentOrientation = RotationDeltaQuat * CurrentOrientation;
                CurrentOrientation.Normalize();
            }
        }

        // 4) Linear motion: convert the double offset to FVector when applying
        FHitResult LinearHitResult;
        FVector LinearOffset = (LinearVelocity * SubstepDeltaTime);
        RootPrimitive->AddWorldOffset(LinearOffset, true, &LinearHitResult);
        if (LinearHitResult.bBlockingHit)
        {
            ResolveCollision(LinearHitResult, RootPrimitive);
        }

        // 5) Angular motion: use computed quaternion delta
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

void UKPhysics::ResolveCollision(FHitResult& Hit, UPrimitiveComponent* PrimitiveComponent)
{
    DVector CenterOfMass(PrimitiveComponent->GetCenterOfMass());
    FQuat CurrentOrientation = PrimitiveComponent->GetComponentQuat();
    
    DVector ContactImpactPoint(Hit.ImpactPoint);
    DVector ContactOffset = ContactImpactPoint - CenterOfMass;
    
    DVector ContactRelativeVelocity = LinearVelocity + DVector::Cross(AngularVelocity, ContactOffset);
    
    DVector HitNormal(Hit.Normal);
    double RelativeNormalVelocity = DVector::Dot(ContactRelativeVelocity, HitNormal);
    
    // Convert the local inertia tensor diagonal into a DMatrix3x3.
    DVector LocalInertiaTensorDiagonal(PrimitiveComponent->BodyInstance.GetBodyInertiaTensor());
    DMatrix3x3 LocalInertia(
        LocalInertiaTensorDiagonal.x, 0.0, 0.0,
        0.0, LocalInertiaTensorDiagonal.y, 0.0,
        0.0, 0.0, LocalInertiaTensorDiagonal.z
    );
    
    // Build the rotation matrix from the current orientation.
    FMatrix RotationMatrixF = FRotationMatrix::Make(CurrentOrientation);
    DMatrix3x3 R(
        RotationMatrixF.M[0][0], RotationMatrixF.M[0][1], RotationMatrixF.M[0][2],
        RotationMatrixF.M[1][0], RotationMatrixF.M[1][1], RotationMatrixF.M[1][2],
        RotationMatrixF.M[2][0], RotationMatrixF.M[2][1], RotationMatrixF.M[2][2]
    );
    
    // Compute the world inertia tensor using: WorldInertia = R * LocalInertia * Transpose(R)
    DMatrix3x3 WorldInertiaTensor = R * LocalInertia * DMatrix3x3::Transpose(R);
    
    double det = DMatrix3x3::Determinant(WorldInertiaTensor);
    if (std::abs(det) < DSmall_Number)
    {
        UE_LOG(LogTemp, Warning, TEXT("Singular inertia tensor matrix encountered in collision resolution."));
        return;
    }
    DMatrix3x3 WorldInertiaTensorInverse = DMatrix3x3::Inverse(WorldInertiaTensor);
    
    // 1) Resolve Normal Collision Impulse
    if (RelativeNormalVelocity < -DSmall_Number)
    {
        DVector LeverArmCrossNormal = DVector::Cross(ContactOffset, HitNormal);
        DVector InertiaInverseCrossResult = WorldInertiaTensorInverse * LeverArmCrossNormal;
        DVector RotationalComponent = DVector::Cross(InertiaInverseCrossResult, ContactOffset);
        
        double RegularizationTerm = 0.0001;
        double EffectiveNormalMass = (1.0 / Mass) + DVector::Dot(HitNormal, RotationalComponent) + RegularizationTerm;
        
        double NormalImpulseMagnitude = -(1.0 + RestitutionCoefficient) * RelativeNormalVelocity / EffectiveNormalMass;
        DVector CollisionImpulse = HitNormal * NormalImpulseMagnitude;
        
        LinearVelocity = LinearVelocity + CollisionImpulse / Mass;
        DVector AngularTorque = DVector::Cross(ContactOffset, CollisionImpulse);
        DVector AngularVelocityDelta = WorldInertiaTensorInverse * AngularTorque;
        AngularVelocity = AngularVelocity + AngularVelocityDelta;
        
        // 2) Resolve Friction Impulse
        ContactRelativeVelocity = LinearVelocity + DVector::Cross(AngularVelocity, ContactOffset);
        DVector PostCollisionNormalVelocity = HitNormal * DVector::Dot(ContactRelativeVelocity, HitNormal);
        DVector PostCollisionTangentialVelocity = ContactRelativeVelocity - PostCollisionNormalVelocity;
        double TangentialVelocityMagnitude = PostCollisionTangentialVelocity.Size();

        if (TangentialVelocityMagnitude < DSmall_Number)
        {
            DVector InducedSlipVelocity = DVector::Cross(AngularVelocity, ContactOffset);
            if (InducedSlipVelocity.Size() < MinInducedSlip)
            {
                InducedSlipVelocity = InducedSlipVelocity.GetSafeNormal() * MinInducedSlip;
            }
            PostCollisionTangentialVelocity = DVector::Lerp(PostCollisionTangentialVelocity, InducedSlipVelocity, InducedSlipBlend);
            TangentialVelocityMagnitude = PostCollisionTangentialVelocity.Size();
        }

        if (TangentialVelocityMagnitude > DSmall_Number)
        {
            DVector FrictionDirection = PostCollisionTangentialVelocity / TangentialVelocityMagnitude;
            DVector LeverArmCrossFriction = DVector::Cross(ContactOffset, FrictionDirection);
            DVector InertiaInverseCrossFriction = WorldInertiaTensorInverse * LeverArmCrossFriction;
            DVector RotationalFrictionComponent = DVector::Cross(InertiaInverseCrossFriction, ContactOffset);

            double EffectiveFrictionMass = 1.0 / Mass + DVector::Dot(FrictionDirection, RotationalFrictionComponent) + RegularizationTerm;

            double CandidateFrictionImpulse = -DVector::Dot(PostCollisionTangentialVelocity, FrictionDirection) / EffectiveFrictionMass;

            double MaxStaticImpulse = StaticFrictionCoefficient * std::abs(NormalImpulseMagnitude);
            double MaxDynamicImpulse = DynamicFrictionCoefficient * std::abs(NormalImpulseMagnitude);

            double FrictionBlendFactor = DMath::Clamp(TangentialVelocityMagnitude / SlipBias, 0.0, 1.0);
            double BlendedFrictionImpulse = DMath::Lerp(MaxStaticImpulse, MaxDynamicImpulse, FrictionBlendFactor);

            double FinalFrictionImpulseMagnitude = DMath::Clamp(CandidateFrictionImpulse, -BlendedFrictionImpulse, BlendedFrictionImpulse);

            DVector FrictionImpulse = FrictionDirection * FinalFrictionImpulseMagnitude;
            LinearVelocity = LinearVelocity + FrictionImpulse / Mass;
            DVector FrictionAngularTorque = DVector::Cross(ContactOffset, FrictionImpulse);
            DVector FrictionAngularDelta = WorldInertiaTensorInverse * FrictionAngularTorque;
            AngularVelocity = AngularVelocity + FrictionAngularDelta;
        }
    }
}