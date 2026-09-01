// PuppyGameMode.cpp -- C++-native GameMode, forces PuppyRobotPawn as DefaultPawn.
// ============================================================
#include "PuppyGameMode.h"

#include "PuppyRobotPawn.h"
#include "UObject/ConstructorHelpers.h"

APuppyGameMode::APuppyGameMode()
{
    // Force the C++ APuppyRobotPawn class as DefaultPawnClass.
    // Because APuppyRobotPawn's constructor CreateDefaultSubobject<>s the
    // CameraBoom / FollowCamera / RobotBodyMesh components, every spawned
    // Pawn is guaranteed to have them (no blueprint CDO freezing issue).
    static ConstructorHelpers::FClassFinder<APawn> RobotPawnFinder(
        TEXT("/Script/PuppyNav.PuppyRobotPawn"));
    if (RobotPawnFinder.Succeeded() && RobotPawnFinder.Class != nullptr)
    {
        DefaultPawnClass = RobotPawnFinder.Class;
        UE_LOG(LogTemp, Log, TEXT("[PuppyGameMode] DefaultPawnClass = /Script/PuppyNav.PuppyRobotPawn (C++ native)"));
    }
    else
    {
        // Absolute last-resort fallback: set directly using StaticClass.
        DefaultPawnClass = APuppyRobotPawn::StaticClass();
        UE_LOG(LogTemp, Warning, TEXT("[PuppyGameMode] FClassFinder failed; fallback DefaultPawnClass=StaticClass()"));
    }

    // Use standard PlayerController so the FollowCamera in PuppyRobotPawn
    // is properly view-targeted when SetViewTarget() is called.
    PlayerControllerClass = APlayerController::StaticClass();

    UE_LOG(LogTemp, Log, TEXT("[PuppyGameMode] Constructed OK"));
}
