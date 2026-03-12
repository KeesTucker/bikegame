#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "KPhysicsSettings.generated.h"

UCLASS(config = Game, defaultconfig, meta = (DisplayName = "KPhysics Settings"))
class BIKEGAME_API UKPhysicsSettings : public UDeveloperSettings
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, config, Category = "KPhysics")
	float NumHzPhysics = 1000.0;

	UPROPERTY(EditAnywhere, config, Category = "KPhysics|Diagnostics")
	bool bShowCollisionDiagnostics = false;
};
