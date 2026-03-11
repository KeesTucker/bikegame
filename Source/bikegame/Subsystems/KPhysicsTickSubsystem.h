#pragma once

#include "CoreMinimal.h"
#include "bikegame/Settings/KPhysicsSettings.h"
#include "Subsystems/WorldSubsystem.h"
#include "KPhysicsTickSubsystem.generated.h"

// Custom tick interface that components must implement.
class IKPhysicsTickInterface
{
public:
	virtual ~IKPhysicsTickInterface() {}
	virtual void PhysicsTick(double DeltaTime) = 0;
};

UCLASS()
class BIKEGAME_API UKPhysicsTickSubsystem : public UWorldSubsystem, public FTickableGameObject
{
	GENERATED_BODY()

public:
	TArray<IKPhysicsTickInterface*> RegisteredComponents;

	virtual void Tick(const float DeltaTime) override
	{
		if (!GetWorld())
		{
			return;
		}

		const double NumHzPhysics = GetDefault<UKPhysicsSettings>()->NumHzPhysics;
		
		const double TargetSubstepDeltaTime = 1.0 / NumHzPhysics;
		int DynamicSubsteps = static_cast<int>(std::ceil(DeltaTime / TargetSubstepDeltaTime));
		DynamicSubsteps = FMath::Max(1, DynamicSubsteps);
		// Note that this will technically be wrong for the last substep if TargetSubstepDeltaTime doesn't divide cleanly into DeltaTime
		const double SubstepDeltaTime = DeltaTime / static_cast<double>(DynamicSubsteps);

		for (int i = 0; i < DynamicSubsteps; ++i)
		{
			for (IKPhysicsTickInterface* Comp : RegisteredComponents)
			{
				if (Comp)
				{
					Comp->PhysicsTick(SubstepDeltaTime);
				}
			}
		}
	}

	virtual TStatId GetStatId() const override
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(UCustomPhysicsTickSubsystem, STATGROUP_Tickables);
	}

	void RegisterComponent(IKPhysicsTickInterface* Component)
	{
		RegisteredComponents.AddUnique(Component);
	}

	void UnregisterComponent(IKPhysicsTickInterface* Component)
	{
		RegisteredComponents.Remove(Component);
	}
};
