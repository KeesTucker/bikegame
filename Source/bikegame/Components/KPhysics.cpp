#include "KPhysics.h"
#include "GameFramework/Actor.h"
#include "Components/PrimitiveComponent.h"
#include "DrawDebugHelpers.h" // Optional: for visualizing impulses

// Dot and Cross product functions for DVector
inline double Dot(const DVector& a, const DVector& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline DVector Cross(const DVector& a, const DVector& b) {
    return DVector(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

// Utility functions for double precision.
inline double Lerp(double A, double B, double Alpha) {
    return A + (B - A) * Alpha;
}
inline double Clamp(double Value, double Min, double Max) {
    return (Value < Min) ? Min : (Value > Max ? Max : Value);
}

const double KINDA_SMALL_NUMBER_D = 1e-6;

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
    const double TargetSubstepDeltaTime = 1.0 / static_cast<double>(NumHzPhysics);
    int DynamicSubsteps = static_cast<int>(std::ceil(DeltaTime / TargetSubstepDeltaTime));
    DynamicSubsteps = FMath::Max(1, DynamicSubsteps);
    const double SubstepDeltaTime = DeltaTime / static_cast<double>(DynamicSubsteps);

    for (int SubstepIndex = 0; SubstepIndex < DynamicSubsteps; ++SubstepIndex)
    {
        // 1) Damping (using double math)
        LinearVelocity = LinearVelocity - (LinearVelocity * (static_cast<double>(LinearDampingFactor) * SubstepDeltaTime));
        AngularVelocity = AngularVelocity - (AngularVelocity * (static_cast<double>(AngularDampingFactor) * SubstepDeltaTime));

        // 2) Gravity: convert Unreal’s gravity (float) to double
        double GravityZ = GetWorld()->GetGravityZ();
        LinearVelocity = LinearVelocity + DVector(0.0, 0.0, GravityZ) * SubstepDeltaTime;

        // 3) Update Orientation using angular velocity (compute in double, then cast to FQuat)
        FQuat CurrentOrientation = RootPrimitive->GetComponentQuat();
        {
            DVector DeltaAngularVelocity = AngularVelocity * SubstepDeltaTime;
            double RotationAngle = DeltaAngularVelocity.Size();

            if (RotationAngle > KINDA_SMALL_NUMBER_D)
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
    
    DVector ContactRelativeVelocity = LinearVelocity + Cross(AngularVelocity, ContactOffset);
    
    DVector HitNormal(Hit.Normal);
    double RelativeNormalVelocity = Dot(ContactRelativeVelocity, HitNormal);
    
    DVector LocalInertiaTensorDiagonal(PrimitiveComponent->BodyInstance.GetBodyInertiaTensor());
    double LocalInertiaTensor[3][3] = {
        {LocalInertiaTensorDiagonal.x, 0.0, 0.0},
        {0.0, LocalInertiaTensorDiagonal.y, 0.0},
        {0.0, 0.0, LocalInertiaTensorDiagonal.z}
    };
    
    FMatrix RotationMatrixF = FRotationMatrix::Make(CurrentOrientation);
    double RotationMatrix[3][3] = {
        {static_cast<double>(RotationMatrixF.M[0][0]), static_cast<double>(RotationMatrixF.M[0][1]), static_cast<double>(RotationMatrixF.M[0][2])},
        {static_cast<double>(RotationMatrixF.M[1][0]), static_cast<double>(RotationMatrixF.M[1][1]), static_cast<double>(RotationMatrixF.M[1][2])},
        {static_cast<double>(RotationMatrixF.M[2][0]), static_cast<double>(RotationMatrixF.M[2][1]), static_cast<double>(RotationMatrixF.M[2][2])}
    };
    
    double WorldInertiaTensor[3][3] = { {0.0} };
    double TempMatrix[3][3] = { {0.0} };
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            for (int k = 0; k < 3; ++k)
            {
                TempMatrix[i][j] += RotationMatrix[i][k] * LocalInertiaTensor[k][j];
            }
        }
    }
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            for (int k = 0; k < 3; ++k)
            {
                WorldInertiaTensor[i][j] += TempMatrix[i][k] * RotationMatrix[j][k];
            }
        }
    }
    
    double det = WorldInertiaTensor[0][0]*(WorldInertiaTensor[1][1]*WorldInertiaTensor[2][2] - WorldInertiaTensor[1][2]*WorldInertiaTensor[2][1])
                - WorldInertiaTensor[0][1]*(WorldInertiaTensor[1][0]*WorldInertiaTensor[2][2] - WorldInertiaTensor[1][2]*WorldInertiaTensor[2][0])
                + WorldInertiaTensor[0][2]*(WorldInertiaTensor[1][0]*WorldInertiaTensor[2][1] - WorldInertiaTensor[1][1]*WorldInertiaTensor[2][0]);
    double WorldInertiaTensorInverse[3][3] = { {0.0} };
    if (std::abs(det) > 1e-8)
    {
        WorldInertiaTensorInverse[0][0] =  (WorldInertiaTensor[1][1]*WorldInertiaTensor[2][2] - WorldInertiaTensor[1][2]*WorldInertiaTensor[2][1]) / det;
        WorldInertiaTensorInverse[0][1] = -(WorldInertiaTensor[0][1]*WorldInertiaTensor[2][2] - WorldInertiaTensor[0][2]*WorldInertiaTensor[2][1]) / det;
        WorldInertiaTensorInverse[0][2] =  (WorldInertiaTensor[0][1]*WorldInertiaTensor[1][2] - WorldInertiaTensor[0][2]*WorldInertiaTensor[1][1]) / det;
        WorldInertiaTensorInverse[1][0] = -(WorldInertiaTensor[1][0]*WorldInertiaTensor[2][2] - WorldInertiaTensor[1][2]*WorldInertiaTensor[2][0]) / det;
        WorldInertiaTensorInverse[1][1] =  (WorldInertiaTensor[0][0]*WorldInertiaTensor[2][2] - WorldInertiaTensor[0][2]*WorldInertiaTensor[2][0]) / det;
        WorldInertiaTensorInverse[1][2] = -(WorldInertiaTensor[0][0]*WorldInertiaTensor[1][2] - WorldInertiaTensor[0][2]*WorldInertiaTensor[1][0]) / det;
        WorldInertiaTensorInverse[2][0] =  (WorldInertiaTensor[1][0]*WorldInertiaTensor[2][1] - WorldInertiaTensor[1][1]*WorldInertiaTensor[2][0]) / det;
        WorldInertiaTensorInverse[2][1] = -(WorldInertiaTensor[0][0]*WorldInertiaTensor[2][1] - WorldInertiaTensor[0][1]*WorldInertiaTensor[2][0]) / det;
        WorldInertiaTensorInverse[2][2] =  (WorldInertiaTensor[0][0]*WorldInertiaTensor[1][1] - WorldInertiaTensor[0][1]*WorldInertiaTensor[1][0]) / det;
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("Singular inertia tensor matrix encountered in collision resolution."));
        return;
    }
    
    // 1) Resolve Normal Collision Impulse
    if (RelativeNormalVelocity < -KINDA_SMALL_NUMBER_D)
    {
        DVector LeverArmCrossNormal = Cross(ContactOffset, HitNormal);
        DVector InertiaInverseCrossResult(
            WorldInertiaTensorInverse[0][0] * LeverArmCrossNormal.x + WorldInertiaTensorInverse[0][1] * LeverArmCrossNormal.y + WorldInertiaTensorInverse[0][2] * LeverArmCrossNormal.z,
            WorldInertiaTensorInverse[1][0] * LeverArmCrossNormal.x + WorldInertiaTensorInverse[1][1] * LeverArmCrossNormal.y + WorldInertiaTensorInverse[1][2] * LeverArmCrossNormal.z,
            WorldInertiaTensorInverse[2][0] * LeverArmCrossNormal.x + WorldInertiaTensorInverse[2][1] * LeverArmCrossNormal.y + WorldInertiaTensorInverse[2][2] * LeverArmCrossNormal.z
        );
        DVector RotationalComponent = Cross(InertiaInverseCrossResult, ContactOffset);
        
        double RegularizationTerm = 0.0001;
        double EffectiveNormalMass = (1.0 / static_cast<double>(Mass)) + Dot(HitNormal, RotationalComponent) + RegularizationTerm;
        
        double NormalImpulseMagnitude = -(1.0 + static_cast<double>(RestitutionCoefficient)) * RelativeNormalVelocity / EffectiveNormalMass;
        DVector CollisionImpulse = HitNormal * NormalImpulseMagnitude;
        
        LinearVelocity = LinearVelocity + CollisionImpulse / static_cast<double>(Mass);
        DVector AngularTorque = Cross(ContactOffset, CollisionImpulse);
        DVector AngularVelocityDelta(
            WorldInertiaTensorInverse[0][0] * AngularTorque.x + WorldInertiaTensorInverse[0][1] * AngularTorque.y + WorldInertiaTensorInverse[0][2] * AngularTorque.z,
            WorldInertiaTensorInverse[1][0] * AngularTorque.x + WorldInertiaTensorInverse[1][1] * AngularTorque.y + WorldInertiaTensorInverse[1][2] * AngularTorque.z,
            WorldInertiaTensorInverse[2][0] * AngularTorque.x + WorldInertiaTensorInverse[2][1] * AngularTorque.y + WorldInertiaTensorInverse[2][2] * AngularTorque.z
        );
        AngularVelocity = AngularVelocity + AngularVelocityDelta;
        
        // 2) Resolve Friction Impulse
        ContactRelativeVelocity = LinearVelocity + Cross(AngularVelocity, ContactOffset);
        DVector PostCollisionNormalVelocity = HitNormal * Dot(ContactRelativeVelocity, HitNormal);
        DVector PostCollisionTangentialVelocity = ContactRelativeVelocity - PostCollisionNormalVelocity;
        double TangentialVelocityMagnitude = PostCollisionTangentialVelocity.Size();

        if (TangentialVelocityMagnitude < KINDA_SMALL_NUMBER_D)
        {
        	DVector InducedSlipVelocity = Cross(AngularVelocity, ContactOffset);
        	if (InducedSlipVelocity.Size() < MinInducedSlip)
        	{
        		InducedSlipVelocity = InducedSlipVelocity.GetSafeNormal() * MinInducedSlip;
        	}
        	PostCollisionTangentialVelocity = PostCollisionTangentialVelocity * (1.0 - 0.5) + InducedSlipVelocity * 0.5;
            TangentialVelocityMagnitude = PostCollisionTangentialVelocity.Size();
        }

        if (TangentialVelocityMagnitude > KINDA_SMALL_NUMBER_D)
        {
	        DVector FrictionDirection = PostCollisionTangentialVelocity / TangentialVelocityMagnitude;
            DVector LeverArmCrossFriction = Cross(ContactOffset, FrictionDirection);
            DVector InertiaInverseCrossFriction(
                WorldInertiaTensorInverse[0][0] * LeverArmCrossFriction.x + WorldInertiaTensorInverse[0][1] * LeverArmCrossFriction.y + WorldInertiaTensorInverse[0][2] * LeverArmCrossFriction.z,
                WorldInertiaTensorInverse[1][0] * LeverArmCrossFriction.x + WorldInertiaTensorInverse[1][1] * LeverArmCrossFriction.y + WorldInertiaTensorInverse[1][2] * LeverArmCrossFriction.z,
                WorldInertiaTensorInverse[2][0] * LeverArmCrossFriction.x + WorldInertiaTensorInverse[2][1] * LeverArmCrossFriction.y + WorldInertiaTensorInverse[2][2] * LeverArmCrossFriction.z
            );
            DVector RotationalFrictionComponent = Cross(InertiaInverseCrossFriction, ContactOffset);

            double EffectiveFrictionMass = (1.0 / static_cast<double>(Mass)) + Dot(FrictionDirection, RotationalFrictionComponent) + RegularizationTerm;

            double CandidateFrictionImpulse = -Dot(PostCollisionTangentialVelocity, FrictionDirection) / EffectiveFrictionMass;

            double MaxStaticImpulse = StaticFrictionCoefficient * std::abs(NormalImpulseMagnitude);
            double MaxDynamicImpulse = DynamicFrictionCoefficient * std::abs(NormalImpulseMagnitude);

            double FrictionBlendFactor = Clamp(TangentialVelocityMagnitude / static_cast<double>(SlipBias), 0.0, 1.0);
            double BlendedFrictionImpulse = Lerp(MaxStaticImpulse, MaxDynamicImpulse, FrictionBlendFactor);

            double FinalFrictionImpulseMagnitude = (std::abs(CandidateFrictionImpulse) <= BlendedFrictionImpulse) ?
                CandidateFrictionImpulse :
                -BlendedFrictionImpulse * ((Dot(PostCollisionTangentialVelocity, FrictionDirection) < 0) ? -1.0 : 1.0);

            DVector FrictionImpulse = FrictionDirection * FinalFrictionImpulseMagnitude;
            LinearVelocity = LinearVelocity + FrictionImpulse / static_cast<double>(Mass);
            DVector FrictionAngularTorque = Cross(ContactOffset, FrictionImpulse);
            DVector FrictionAngularDelta(
                WorldInertiaTensorInverse[0][0] * FrictionAngularTorque.x + WorldInertiaTensorInverse[0][1] * FrictionAngularTorque.y + WorldInertiaTensorInverse[0][2] * FrictionAngularTorque.z,
                WorldInertiaTensorInverse[1][0] * FrictionAngularTorque.x + WorldInertiaTensorInverse[1][1] * FrictionAngularTorque.y + WorldInertiaTensorInverse[1][2] * FrictionAngularTorque.z,
                WorldInertiaTensorInverse[2][0] * FrictionAngularTorque.x + WorldInertiaTensorInverse[2][1] * FrictionAngularTorque.y + WorldInertiaTensorInverse[2][2] * FrictionAngularTorque.z
            );
            AngularVelocity = AngularVelocity + FrictionAngularDelta;
        }
    }
}