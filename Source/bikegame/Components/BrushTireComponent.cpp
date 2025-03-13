#include "BrushTireComponent.h"
#include "GameFramework/Actor.h"

UBrushTireComponent::UBrushTireComponent(): WheelMesh(nullptr), AxleConstraint(nullptr), TireStiffness(0), RimRadius(0),
                                            RimWidth(0), TireWidth(0), TireSidewallHeight(0), PreviousDeflection(0)
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

    FVector AxleConstraintForce;
    FVector AxleConstraintAngularForce;
    AxleConstraint->GetConstraintForce(AxleConstraintForce, AxleConstraintAngularForce);
    float EffectiveMass = AxleConstraintForce.Size() / 9.81f; // Weird constant?? I feel like this should work but its way off.
    UE_LOG(LogTemp, Warning, TEXT("Mass: %f"), EffectiveMass);
    const float TireDampingCoefficient = 2 * FMath::Sqrt(TireStiffness * EffectiveMass) * TireDamping;
    
    /*// Get the current wheel location
    FVector WheelLocation = WheelMesh->GetComponentLocation();
    // Assume the tire extends downward along the negative Z-axis
    FVector DownVector = FVector(0, 0, -1);
    
    // Define the raycast range, e.g., from the wheel center downward by the tire's nominal height
    float NominalHeight = RimRadius + TireSidewallHeight;
    FVector Start = WheelLocation;
    FVector End = Start + DownVector * NominalHeight;
    
    // Perform the raycast to detect the ground
    FHitResult HitResult;
    FCollisionQueryParams QueryParams;
    QueryParams.AddIgnoredActor(GetOwner());
    
    bool bHit = GetWorld()->LineTraceSingleByChannel(HitResult, Start, End, ECC_Visibility, QueryParams);
    
    float NormalForce = 0.0f;
    if (bHit)
    {
        // Ground distance from the wheel center
        float GroundDistance = HitResult.Distance;
        
        // Calculate deflection (compression): if the distance is less than the nominal height,
        // the tire is compressed.
        float Deflection = NominalHeight - GroundDistance;
        
        // Only compute force when the tire is in compression (Deflection > 0)
        if (Deflection > 0.0f)
        {
            // Compute the rate of compression (ḋ)
            float DeflectionRate = (Deflection - PreviousDeflection) / DeltaTime;
            PreviousDeflection = Deflection;

            FVector AxleConstraintForce;
            FVector AxleConstraintAngularForce;
            AxleConstraint->GetConstraintForce(AxleConstraintForce, AxleConstraintAngularForce);
            float EffectiveMass = AxleConstraintForce.Size() / 9.81f; // Weird constant?? I feel like this should work but its way off.
            UE_LOG(LogTemp, Warning, TEXT("Mass: %f"), EffectiveMass);
            const float TireDampingCoefficient = 2 * FMath::Sqrt(TireStiffness * EffectiveMass) * TireDamping;
            
            // Compute the spring-damper force
            NormalForce = ((TireStiffness * Deflection) + (TireDampingCoefficient * DeflectionRate)) / cMToM;
            // Clamp to ensure the force only pushes upward
            NormalForce = FMath::Clamp(NormalForce, -MaxTireForce / cMToM, MaxTireForce / cMToM);
        }
    }
    else
    {
        // Reset deflection if not in contact
        PreviousDeflection = 0.0f;
    }
    
    // Apply the computed vertical force at the wheel's location.
    // Note: The force is applied in the world upward direction.
    FVector ForceVector = FVector(0, 0, NormalForce);
    WheelMesh->AddForce(ForceVector);*/
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
