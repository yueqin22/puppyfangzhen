// PuppyGameMode.h -- C++-native GameMode (no blueprint needed).
// ============================================================
// Fixes the "stale blueprint DefaultPawnClass" problem:
//   Previously BP_PuppyGameMode was created by ue_auto_setup.py and its
//   DefaultPawnClass (BP_PuppyRobotPawn_C) was often lost because the
//   blueprint was saved against a stub / non-compiled GeneratedClass.
//   Using a C++ GameMode with DefaultPawnClass set in the constructor
//   guarantees that PuppyRobotPawn (with its CameraBoom + FollowCamera
//   subobjects) is always spawned. No blueprint / editor-compile step required.
//
// DefaultEngine.ini entry (GlobalDefaultGameMode):
//   /Script/PuppyNav.PuppyGameMode
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "PuppyGameMode.generated.h"

UCLASS()
class PUPPYNAV_API APuppyGameMode : public AGameModeBase
{
    GENERATED_BODY()
public:
    APuppyGameMode();
};
