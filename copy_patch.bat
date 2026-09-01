@echo off
echo Copying patched files to UE project...
copy /Y "e:\puppyfangzhen\PuppyRobotPawn_patched.cpp" "D:\puppy_ue\Source\PuppyNav\PuppyRobotPawn.cpp"
if errorlevel 1 (
    echo ERROR: CPP copy failed
) else (
    echo CPP copied successfully
)
copy /Y "e:\puppyfangzhen\PuppyRobotPawn_patched.h" "D:\puppy_ue\Source\PuppyNav\PuppyRobotPawn.h"
if errorlevel 1 (
    echo ERROR: H copy failed
) else (
    echo H copied successfully
)
echo.
echo Verifying...
dir "D:\puppy_ue\Source\PuppyNav\PuppyRobotPawn.cpp" "D:\puppy_ue\Source\PuppyNav\PuppyRobotPawn.h"
