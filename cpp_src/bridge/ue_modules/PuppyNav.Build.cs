// PuppyNav.Build.cs -- UE5 module build file for the PuppyNav module.
// ==================================================================
// Module name : PuppyNav
// Type        : CPlusPlus
// Dependencies: Core, CoreUObject, Engine, InputCore, NavigationSystem,
//                AIModule, Sockets, Networking
using UnrealBuildTool;

public class PuppyNav : ModuleRules
{
    public PuppyNav(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new string[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "InputCore",
            "NavigationSystem",
            "AIModule",
            "Sockets",
            "Networking"
        });
    }
}
