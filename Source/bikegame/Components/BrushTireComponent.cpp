#include "BrushTireComponent.h"
#include "GameFramework/Actor.h"

UBrushTireComponent::UBrushTireComponent(): WheelMesh(nullptr), AxleConstraint(nullptr), RimRadius(0), RimWidth(0),
    TireWidth(0), TireSidewallHeight(0), BristleRadialStiffness(0)
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
    
    RimRadius = RimDiameterInches * InchTocm * 0.5;
    RimWidth = RimWidthInches * InchTocm;
    TireWidth = TireWidthMm * mmTocm;
    TireSidewallHeight = TireWidth * (TireAspectRatio / 100);
    BristleRadialStiffness = RadialDeflectionStiffness / BrushSegmentsPerWheel / BrushSegmentsPerWheel;

    SetActive(true);
    SetAsyncPhysicsTickEnabled(true);
    
    // Start brush model thread
    //BThreadRunning = true;
    //BrushThread = std::thread(&UBrushTireComponent::RunBrushModel, this);
}

void UBrushTireComponent::PostInitProperties()
{
    Super::PostInitProperties();
    
    
}

void UBrushTireComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UBrushTireComponent::AsyncPhysicsTickComponent(float DeltaTime, float SimTime)
{
    Super::AsyncPhysicsTickComponent(DeltaTime, SimTime);

    if (Bristles.Num() != BrushSegmentsPerWheel)
    {
        Bristles.SetNum(BrushSegmentsPerWheel);
    }

    if (Bristles.Num() > 0 && Bristles[0].Num() != BristlesPerSegment)
    {
        for (int32 i = 0; i < Bristles.Num(); i++)
        {
            Bristles[i].SetNum(BristlesPerSegment);
        }
    }
    
    FVector Center = WheelMesh->GetComponentLocation();
    FVector RightVector = WheelMesh->GetUpVector();
    FVector BaseVector = FVector(1, 0, 0);//WheelMesh->GetRightVector();
    const float AngleStep = 360.0f / BrushSegmentsPerWheel;  // degrees between each segment

    int numBristlesInContact = 0;
    for (int32 SegmentIndex = 0; SegmentIndex < BrushSegmentsPerWheel; SegmentIndex++)
    {
        float AngleDeg = AngleStep * SegmentIndex;
        float AngleRad = FMath::DegreesToRadians(AngleDeg);
        
        FQuat RotationQuat(RightVector, AngleRad);
        FVector Direction = RotationQuat.RotateVector(BaseVector);

        FVector SegmentCenter = Center + Direction * RimRadius;

        // Calculate spacing along the rim width (for both origin and tip)
        float OriginSpacing = RimWidth / BristlesPerSegment;
        float TipSpacing = RimWidth / BristlesPerSegment;

        for (int32 BristleIndex = 0; BristleIndex < BristlesPerSegment; BristleIndex++)
        {
            // Compute a centered offset for the origin.
            float OriginOffset = (BristleIndex + 0.5f - BristlesPerSegment * 0.5f) * OriginSpacing;
            FVector BristleOrigin = SegmentCenter + RightVector * OriginOffset;
            
            // Compute the "raw" tip with an additional offset along UpVector.
            float TipOffset = (BristleIndex + 0.5f - BristlesPerSegment * 0.5f) * TipSpacing;
            FVector RawTip = BristleOrigin + Direction * TireSidewallHeight + RightVector * TipOffset;

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

            FVector DeflectedBristleTip = bHit ? HitResult.Location : FinalBristleTip;
            float RadialDeflection = FVector::Distance(DeflectedBristleTip, FinalBristleTip);

            float DeltaDeflection = 0;
            if (WheelMesh->IsSimulatingPhysics() && bHit)
            {
                numBristlesInContact++;
                DeltaDeflection = RadialDeflection - Bristles[SegmentIndex][BristleIndex].RadialDeflection;
            }

            FBristle Bristle = FBristle(BristleOrigin, FinalBristleTip, DesiredDirection,
                HitResult.Location, HitResult.Normal, RadialDeflection, DeltaDeflection, bHit);
            Bristles[SegmentIndex][BristleIndex] = Bristle;

            FColor DebugColor = bHit ? FColor::Green : FColor::Red;
            DrawDebugLine(GetWorld(), BristleOrigin, FinalBristleTip, DebugColor, false, -1.0f, 0, 0.5f);

        }
    }
    if (numBristlesInContact > 0)
    {
        for (int32 SegmentIndex = 0; SegmentIndex < BrushSegmentsPerWheel; SegmentIndex++)
        {
            for (int32 BristleIndex = 0; BristleIndex < BristlesPerSegment; BristleIndex++)
            {
                if (WheelMesh->IsSimulatingPhysics())
                {
                    FBristle BristleState = Bristles[SegmentIndex][BristleIndex];

                    FVector AxleConstraintForce;
                    FVector AxleConstraintAngularForce;
                    AxleConstraint->GetConstraintForce(AxleConstraintForce, AxleConstraintAngularForce);
                    float AxleWeight = AxleConstraintForce.Size() / 9.81f;
                    float DampingCoefficientAdjustment = 2.0f * FMath::Sqrt(BristleRadialStiffness * (255.0 / (BrushSegmentsPerWheel * BristlesPerSegment)));
                    //UE_LOG(LogTemp, Warning, TEXT("Damping: %f"), RadialDeflectionDampingCoefficent * DampingCoefficientAdjustment);
                
                    float DampingMagnitude = RadialDeflectionDampingCoefficent * DampingCoefficientAdjustment * (BristleState.RadialDeflectionDelta / DeltaTime);
                    float DeflectionMagnitude = BristleRadialStiffness * BristleState.RadialDeflection / cMToM;
                    FVector RadialDeflectionForce = FMath::Max(0, DeflectionMagnitude + DampingMagnitude) * BristleState.ContactNormal;
                    WheelMesh->AddForceAtLocation(RadialDeflectionForce, BristleState.Origin, NAME_None);
                }
            }
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
