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
    
    // Start brush model thread
    BThreadRunning = true;
    BrushThread = std::thread(&UBrushTireComponent::RunBrushModel, this);
}

void UBrushTireComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // Draw a debug line that points in the up direction of the wheel transform
    if (WheelMesh)
    {
        // Cast the interface to a UStaticMeshComponent pointer
        if (GetWorld())
        {
            // Get the starting location and the up vector of the wheel
            FVector Start = WheelMesh->GetComponentLocation();
            FVector UpDir = WheelMesh->GetUpVector();
            FVector End = Start + UpDir * 100.0f;  // 100 units long debug line

            // Draw the line: color green, not persistent (will refresh each frame), with a thickness of 2.0f.
            DrawDebugLine(GetWorld(), Start, End, FColor::Green, false, -1.0f, 0, 2.0f);
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
