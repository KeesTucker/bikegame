#include "BrushTireComponent.h"
#include "GameFramework/Actor.h"

UBrushTireComponent::UBrushTireComponent(): WheelMesh(nullptr)
{
    PrimaryComponentTick.bCanEverTick = true;
    Fx = 0.0f;
    Fy = 0.0f;
    Fz = 0.0f;
    BThreadRunning = false;
}

void UBrushTireComponent::BeginPlay()
{
    Super::BeginPlay();

    RimRadius = RimDiameterInches * InchToCm * 0.5;
    RimWidth = RimWidthInches * InchToCm;
    TireWidth = TireWidthMm * MmToCm;
    TireSidewallHeight = TireWidth * (TireAspectRatio / 100);
    
    // Start brush model thread
    //BThreadRunning = true;
    //BrushThread = std::thread(&UBrushTireComponent::RunBrushModel, this);
}

void UBrushTireComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (WheelMesh && GetWorld())
    {
        int TotalContactPoints = 0;
        FVector Center = WheelMesh->GetComponentLocation();
        FVector UpVector = WheelMesh->GetUpVector();
        FVector BaseVector = WheelMesh->GetRightVector();
        const float AngleStep = 360.0f / BrushSegmentsPerWheel;  // degrees between each segment
    
        for (int32 SegmentIndex = 0; SegmentIndex < BrushSegmentsPerWheel; SegmentIndex++)
        {
            float AngleDeg = AngleStep * SegmentIndex;
            float AngleRad = FMath::DegreesToRadians(AngleDeg);
    
            // Rotate BaseVector around the Up axis by the calculated angle.
            FQuat RotationQuat(UpVector, AngleRad);
            FVector Direction = RotationQuat.RotateVector(BaseVector);
    
            FVector SegmentCenter = Center + Direction * RimRadius;
    
            // Calculate spacing along the rim width (for both origin and tip)
            float OriginSpacing = RimWidth / BristlesPerSegment;
            float TipSpacing = RimWidth / BristlesPerSegment;
    
            for (int32 BristleIndex = 0; BristleIndex < BristlesPerSegment; BristleIndex++)
            {
                // Compute a centered offset for the origin.
                float OriginOffset = (BristleIndex + 0.5f - BristlesPerSegment * 0.5f) * OriginSpacing;
                FVector BristleOrigin = SegmentCenter + UpVector * OriginOffset;
                
                // Compute the "raw" tip with an additional offset along UpVector.
                float TipOffset = (BristleIndex + 0.5f - BristlesPerSegment * 0.5f) * TipSpacing;
                FVector RawTip = BristleOrigin + Direction * TireSidewallHeight + UpVector * TipOffset;
    
                // Determine the direction toward the raw tip.
                FVector DesiredDirection = (RawTip - BristleOrigin).GetSafeNormal();
    
                // Ensure the drawn bristle is always TireSidewallHeight long.
                FVector FinalBristleTip = BristleOrigin + DesiredDirection * TireSidewallHeight;
    
                // Perform a raycast (line trace) from an adjusted origin to the final tip.
                FVector RayOrigin = BristleOrigin - DesiredDirection * RimRadius;
                FHitResult HitResult;
                FCollisionQueryParams QueryParams;
                QueryParams.AddIgnoredActor(GetOwner());  // Ignore the owner of this component
                bool bHit = GetWorld()->LineTraceSingleByChannel(HitResult, RayOrigin, FinalBristleTip, ECC_Visibility, QueryParams);
    
                if (bHit)
                {
                    TotalContactPoints++;
                }
                
                // Choose debug color: green if hit, red if not.
                FColor DebugColor = bHit ? FColor::Green : FColor::Red;
                
                // Draw the debug line (bristle) with the chosen color.
                DrawDebugLine(GetWorld(), BristleOrigin, FinalBristleTip, DebugColor, false, -1.0f, 0, 0.5f);
            }
        }
    
        // Apply upward force based on total contact points
        // (ForcePerContactPoint should be defined as a float variable, e.g., a UPROPERTY.)
        float ForceMagnitude = TotalContactPoints * ForcePerContactPoint;
        if (WheelMesh->IsSimulatingPhysics())
        {
            WheelMesh->AddForce(ForceMagnitude * FVector(0, 0, 1), NAME_None, false);
        }
    }
}

void UBrushTireComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Super::EndPlay(EndPlayReason);

    // Stop thread safely
    BThreadRunning = false;
    if (BrushThread.joinable()) {
        BrushThread.join();
    }
}

// Thread function: Runs brush model in a loop
void UBrushTireComponent::RunBrushModel()
{
    constexpr float UpdateRate = 1.0f / 2000.0f; // 2000 Hz

    while (BThreadRunning)
    {
        float LocalFx, LocalFy, LocalFz;
        ComputeBrushForces(LocalFx, LocalFy, LocalFz);
        
        // Store in atomic variables
        Fx.store(LocalFx);
        Fy.store(LocalFy);
        Fz.store(LocalFz);

        // Sleep to maintain update rate
        FPlatformProcess::Sleep(UpdateRate);
    }
}

// Computes tire forces using brush model logic
void UBrushTireComponent::ComputeBrushForces(float& OutFx, float& OutFy, float& OutFz) const
{
    
}

// Allows Unreal to retrieve computed forces
FVector UBrushTireComponent::GetTireForce() const
{
    return FVector(Fx.load(), Fy.load(), Fz.load());
}
