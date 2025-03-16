#include "KPhysics.h"

/////////////////////////////////////////////////////
// Constructor and Initialization
/////////////////////////////////////////////////////

// Sets default values for this component's properties.
UKPhysics::UKPhysics() 
	: TargetComponent(nullptr)
	, LinearVelocity()
	, AngularVelocity()
{
	// Enable ticking for this component.
	PrimaryComponentTick.bCanEverTick = true;
}

// Called when the game starts.
void UKPhysics::BeginPlay()
{
	Super::BeginPlay();
	
	// Activate the component and enable asynchronous physics ticking.
	SetActive(true);
	SetAsyncPhysicsTickEnabled(true);
}

/////////////////////////////////////////////////////
// Main Physics Simulation Update
/////////////////////////////////////////////////////

// Called every frame.
void UKPhysics::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// Ensure that the target component and world exist.
	if (!TargetComponent || !GetWorld())
	{
		return;
	}

	// Calculate the time interval for each integration substep.
	float SubstepDeltaTime = DeltaTime / static_cast<float>(NumIntegrationSubsteps);
    
	// Perform physics simulation using multiple substeps.
	for (int SubstepIndex = 0; SubstepIndex < NumIntegrationSubsteps; ++SubstepIndex)
	{
		// ----------------------------------------------------------------
		// 1) Apply Damping to Linear and Angular Velocities
		// ----------------------------------------------------------------
		// Reduce linear velocity using a simple damping model.
		LinearVelocity -= (LinearVelocity * LinearDampingFactor * SubstepDeltaTime);
		// Reduce angular velocity using a simple damping model.
		AngularVelocity -= (AngularVelocity * AngularDampingFactor * SubstepDeltaTime);

		// ----------------------------------------------------------------
		// 2) Apply Gravity
		// ----------------------------------------------------------------
		// Unreal Engine's gravity acts in the negative Z direction.
		// Adjust the linear velocity using the world's gravity value.
		LinearVelocity += (GetWorld()->GetGravityZ() * FVector(0.f, 0.f, 1.f)) * SubstepDeltaTime;

		// ----------------------------------------------------------------
		// 3) Update Target Orientation Based on Angular Velocity
		// ----------------------------------------------------------------
		// Get the current orientation of the target.
		FQuat CurrentOrientation = TargetComponent->GetComponentQuat();
		
		{
			// Compute the angular change over the substep.
			FVector DeltaAngularVelocity = AngularVelocity * SubstepDeltaTime;
			float RotationAngle = DeltaAngularVelocity.Size();

			// Only update orientation if rotation is significant.
			if (RotationAngle > KINDA_SMALL_NUMBER)
			{
				FVector RotationAxis = DeltaAngularVelocity / RotationAngle;
				float HalfRotationAngle = 0.5f * RotationAngle;
				float SineHalfAngle = FMath::Sin(HalfRotationAngle);

				// Compute the quaternion representing the rotational change.
				FQuat RotationDeltaQuat(RotationAxis.X * SineHalfAngle, RotationAxis.Y * SineHalfAngle, RotationAxis.Z * SineHalfAngle, FMath::Cos(HalfRotationAngle));
				CurrentOrientation = RotationDeltaQuat * CurrentOrientation;
				CurrentOrientation.Normalize();
			}
		}

		// ----------------------------------------------------------------
		// 4) Apply Linear Motion and Detect Collisions
		// ----------------------------------------------------------------
		FHitResult LinearHitResult;
		// Move the target based on the updated linear velocity.
		TargetComponent->AddWorldOffset(LinearVelocity * SubstepDeltaTime, true, &LinearHitResult);
		// If a collision is detected during linear motion, resolve it.
		if (LinearHitResult.bBlockingHit)
		{
			ResolveCollision(LinearHitResult, SubstepDeltaTime);
		}

		// ----------------------------------------------------------------
		// 5) Apply Angular Motion and Detect Collisions
		// ----------------------------------------------------------------
		FQuat InitialOrientation = TargetComponent->GetComponentQuat();
		// Calculate the difference between the new and initial orientations.
		FQuat RotationDeltaQuat = CurrentOrientation * InitialOrientation.Inverse();
		FHitResult AngularHitResult;
		// Apply the rotational movement.
		TargetComponent->AddWorldRotation(RotationDeltaQuat.Rotator(), true, &AngularHitResult);
		// If a collision is detected during angular motion, resolve it.
		if (AngularHitResult.bBlockingHit)
		{
			ResolveCollision(AngularHitResult, SubstepDeltaTime);
		}
	}
}

/////////////////////////////////////////////////////
// Collision Resolution
/////////////////////////////////////////////////////

// Resolves collision responses based on the hit result and the substep delta time.
void UKPhysics::ResolveCollision(FHitResult& Hit, float SubstepDeltaTime)
{
	// Retrieve the center of mass and current orientation of the target.
	FVector CenterOfMass = TargetComponent->GetCenterOfMass();
	FQuat CurrentOrientation = TargetComponent->GetComponentQuat();
	
	// Compute the offset from the center of mass to the contact impact point.
	FVector ContactImpactPoint = Hit.ImpactPoint;
	FVector ContactOffset = ContactImpactPoint - CenterOfMass;
	
	// Calculate the relative velocity at the contact point.
	FVector ContactRelativeVelocity = LinearVelocity + FVector::CrossProduct(AngularVelocity, ContactOffset);
	
	// Determine the velocity component along the collision normal.
	float RelativeNormalVelocity = FVector::DotProduct(ContactRelativeVelocity, Hit.Normal);
	
	// Build the local inertia tensor matrix using the target's inertia tensor diagonal.
	FVector LocalInertiaTensorDiagonal = TargetComponent->BodyInstance.GetBodyInertiaTensor();
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
	// Only process collisions when the object is moving into the contact surface.
	if (RelativeNormalVelocity < -KINDA_SMALL_NUMBER)
	{
		// Compute the effective mass for the normal impulse:
		// effectiveMass = 1/TargetMass + n • [ (I⁻¹ * (r × n)) × r ]
		FVector LeverArmCrossNormal = FVector::CrossProduct(ContactOffset, Hit.Normal);
		FVector InertiaInverseCrossResult = WorldInertiaTensorInverse.TransformVector(LeverArmCrossNormal);
		FVector RotationalComponent = FVector::CrossProduct(InertiaInverseCrossResult, ContactOffset);
		float EffectiveNormalMass = (1.0f / TargetMass) + FVector::DotProduct(Hit.Normal, RotationalComponent);
		
		// Compute the magnitude of the normal impulse:
		// impulseMagnitude = -(1 + RestitutionCoefficient) * (relativeVelocity · n) / effectiveMass
		float NormalImpulseMagnitude = -(1.f + RestitutionCoefficient) * RelativeNormalVelocity / EffectiveNormalMass;
		
		// Calculate the collision impulse vector.
		FVector CollisionImpulse = NormalImpulseMagnitude * Hit.Normal;

		// Optionally, you can draw a debug arrow to visualize the collision impulse.
		/*
		DrawDebugDirectionalArrow(
			GetWorld(),
			ContactImpactPoint,
			ContactImpactPoint + CollisionImpulse,
			10.f,
			FColor::Red,
			false,
			0.1f
		);
		*/
		
		// Update the linear velocity based on the collision impulse.
		LinearVelocity += CollisionImpulse / TargetMass;
		// Compute the angular torque from the collision impulse and update angular velocity.
		FVector AngularTorque = FVector::CrossProduct(ContactOffset, CollisionImpulse);
		AngularVelocity += WorldInertiaTensorInverse.TransformVector(AngularTorque);
		
		// ----------------------------------------------------------------
		// 2) Resolve Friction Impulse
		// ----------------------------------------------------------------
		// Recalculate the relative velocity at the contact point after applying the normal impulse.
		ContactRelativeVelocity = LinearVelocity + FVector::CrossProduct(AngularVelocity, ContactOffset);
		// Decompose the velocity into normal and tangential components relative to the contact surface.
		FVector PostCollisionNormalVelocity = FVector::DotProduct(ContactRelativeVelocity, Hit.Normal) * Hit.Normal;
		FVector PostCollisionTangentialVelocity = ContactRelativeVelocity - PostCollisionNormalVelocity;
		float TangentialVelocityMagnitude = PostCollisionTangentialVelocity.Size();
		
		// If tangential velocity is very small but angular velocity is significant, induce slip.
		if (TangentialVelocityMagnitude < KINDA_SMALL_NUMBER && AngularVelocity.Size() > KINDA_SMALL_NUMBER)
		{
			// Compute induced slip from rotation: inducedSlipVelocity = ω × r.
			FVector InducedSlipVelocity = FVector::CrossProduct(AngularVelocity, ContactOffset);
			PostCollisionTangentialVelocity = InducedSlipVelocity;
			TangentialVelocityMagnitude = PostCollisionTangentialVelocity.Size();
		}

		// Only apply friction if there is significant tangential slip.
		if (TangentialVelocityMagnitude > KINDA_SMALL_NUMBER)
		{
			// Determine the direction of friction based on tangential velocity.
			FVector FrictionDirection = PostCollisionTangentialVelocity / TangentialVelocityMagnitude;
	
			// Compute the effective mass for the friction impulse.
			FVector LeverArmCrossFriction = FVector::CrossProduct(ContactOffset, FrictionDirection);
			FVector InertiaInverseCrossFriction = WorldInertiaTensorInverse.TransformVector(LeverArmCrossFriction);
			FVector RotationalFrictionComponent = FVector::CrossProduct(InertiaInverseCrossFriction, ContactOffset);
			float EffectiveFrictionMass = (1.0f / TargetMass) + FVector::DotProduct(FrictionDirection, RotationalFrictionComponent);
	
			// Calculate the candidate friction impulse magnitude to cancel the tangential velocity.
			float CandidateFrictionImpulse = -FVector::DotProduct(PostCollisionTangentialVelocity, FrictionDirection) / EffectiveFrictionMass;
	
			// Limit the friction impulse based on the static and dynamic friction coefficients.
			float MaxStaticImpulse = StaticFrictionCoefficient * FMath::Abs(NormalImpulseMagnitude);
			float MaxDynamicImpulse = DynamicFrictionCoefficient * FMath::Abs(NormalImpulseMagnitude);
	
			float FinalFrictionImpulseMagnitude = 0.f;
			if (FMath::Abs(CandidateFrictionImpulse) <= MaxStaticImpulse)
			{
				FinalFrictionImpulseMagnitude = CandidateFrictionImpulse;
			}
			else
			{
				FinalFrictionImpulseMagnitude = -MaxDynamicImpulse * FMath::Sign(FVector::DotProduct(PostCollisionTangentialVelocity, FrictionDirection));
			}
	
			// Compute the friction impulse vector.
			FVector FrictionImpulse = FinalFrictionImpulseMagnitude * FrictionDirection;
	
			// Update linear velocity based on the friction impulse.
			LinearVelocity += FrictionImpulse / TargetMass;
			// Compute the friction torque and update angular velocity.
			FVector FrictionAngularTorque = FVector::CrossProduct(ContactOffset, FrictionImpulse);
			AngularVelocity += WorldInertiaTensorInverse.TransformVector(FrictionAngularTorque);
		}
	}
}