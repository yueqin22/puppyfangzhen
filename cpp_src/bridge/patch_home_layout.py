"""
Patch PuppyRobotPawn.cpp and .h - v3 final
Uses line-by-line replacement for robustness.
"""
import sys, os, shutil

CPP_TARGET = r"D:\puppy_ue\Source\PuppyNav\PuppyRobotPawn.cpp"
H_TARGET   = r"D:\puppy_ue\Source\PuppyNav\PuppyRobotPawn.h"

# New HOME_OBSTACLES array content (centimeters, x100 from scene_home.json v3)
NEW_OBSTACLES = '''static const FHomeObstacle HOME_OBSTACLES[] = {
    // --- Living room furniture ---
    { TEXT("sofa              "), -180.f,-370.f,  80.f,-300.f },
    { TEXT("coffee_table      "),  -50.f,-230.f,  50.f,-170.f },
    { TEXT("tv_stand          "), -150.f, -40.f,  50.f,   0.f },
    // --- Dining area ---
    { TEXT("dining_table      "),   60.f,-130.f, 180.f, -50.f },
    { TEXT("chair_n           "),   90.f, -35.f, 130.f,   0.f },
    { TEXT("chair_s           "),   90.f,-165.f, 130.f,-130.f },
    { TEXT("chair_w           "),   30.f,-110.f,  65.f, -70.f },
    { TEXT("chair_e           "),  175.f,-110.f, 210.f, -70.f },
    // --- Master bedroom ---
    { TEXT("master_bed        "), -450.f, 210.f,-270.f, 390.f },
    { TEXT("nightstand        "), -380.f, 185.f,-330.f, 225.f },
    { TEXT("wardrobe          "), -500.f, 170.f,-440.f, 320.f },
    // --- Second bedroom ---
    { TEXT("single_bed        "), -170.f, 220.f, -70.f, 380.f },
    { TEXT("desk              "), -180.f, 170.f, -80.f, 220.f },
    { TEXT("bookshelf         "),    5.f, 200.f,  35.f, 350.f },
    // --- Bathroom ---
    { TEXT("sink              "),   65.f, 170.f, 145.f, 215.f },
    { TEXT("toilet            "),  160.f, 230.f, 220.f, 290.f },
    { TEXT("shower            "),  150.f, 300.f, 250.f, 400.f },
    // --- Kitchen ---
    { TEXT("counter_north     "),  270.f,  90.f, 480.f, 140.f },
    { TEXT("counter_east      "),  440.f,-120.f, 490.f, 140.f },
    { TEXT("fridge            "),  270.f,  20.f, 330.f,  85.f },
    { TEXT("kitchen_island    "),  330.f, -30.f, 410.f,  20.f },
    // --- Outer boundary walls ---
    { TEXT("wall_north        "), -500.f, 390.f, 500.f, 400.f },
    { TEXT("wall_south_west   "), -500.f,-400.f,-280.f,-390.f },
    { TEXT("wall_south_east   "), -180.f,-400.f, 500.f,-390.f },
    { TEXT("wall_east         "),  490.f,-400.f, 500.f, 400.f },
    { TEXT("wall_west         "), -500.f,-400.f,-490.f, 400.f },
    // --- Internal walls: horizontal divider at y~1.5m (with door gaps) ---
    { TEXT("wall_n1           "), -500.f, 135.f,-300.f, 165.f },
    { TEXT("wall_n2           "), -200.f, 135.f,-100.f, 165.f },
    { TEXT("wall_n2_east      "),    0.f, 135.f,  65.f, 165.f },
    { TEXT("wall_n3           "),  165.f, 135.f, 250.f, 165.f },
    // --- Internal walls: vertical dividers ---
    { TEXT("wall_v_bed_div    "), -215.f, 150.f,-185.f, 400.f },
    { TEXT("wall_v_bath_west  "),   35.f, 150.f,  65.f, 400.f },
    { TEXT("wall_v_kitchen    "),  235.f, 135.f, 265.f, 400.f },
};'''

SPAWN_FUNC = r'''
void APuppyRobotPawn::SpawnObstacleVisuals()
{
    UWorld* World = GetWorld();
    if (!World) return;

    // ---- Cleanup old obstacles from previous scene builder (BSP brushes labeled "OBS_*")
    // and any previously spawned "Obs_*" static mesh actors to avoid double geometry ----
    TArray<AActor*> AllActors;
    UGameplayStatics::GetAllActorsOfClass(World, AActor::StaticClass(), AllActors);
    int32 Destroyed = 0;
    for (AActor* A : AllActors)
    {
        if (!A || A == this) continue;
        FString Label = A->GetActorLabel();
        if (Label.StartsWith(TEXT("OBS_")) || Label.StartsWith(TEXT("Obs_")) || Label.StartsWith(TEXT("PATROL_")))
        {
            A->Destroy();
            ++Destroyed;
        }
    }
    if (Destroyed > 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] Cleaned up %d old obstacle actors (BSP/mesh from previous layout)"), Destroyed);
    }

    // Load assets at runtime (ConstructorHelpers only works in constructors)
    UStaticMesh* CubeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
    UMaterialInterface* BaseMat = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
    if (!CubeMesh) return;

    FActorSpawnParameters Params;
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    int32 Spawned = 0;
    for (int32 i = 0; i < NUM_HOME_OBSTACLES; ++i)
    {
        const FHomeObstacle& O = HOME_OBSTACLES[i];
        FString NameStr(O.Name);
        NameStr.TrimEndInline();
        bool bIsWall = NameStr.Contains(TEXT("wall"));
        bool bIsCounter = NameStr.Contains(TEXT("counter"));

        float Cx = (O.Xmin + O.Xmax) * 0.5f;
        float Cy = (O.Ymin + O.Ymax) * 0.5f;
        float Sizex = FMath::Max(1.0f, O.Xmax - O.Xmin);
        float Sizey = FMath::Max(1.0f, O.Ymax - O.Ymin);

        float Sizez;
        FColor Color;
        if (bIsWall) { Sizez = 280.0f; Color = FColor(225, 220, 210); }
        else if (bIsCounter) { Sizez = 90.0f; Color = FColor(140, 115, 88); }
        else if (NameStr.Contains(TEXT("bed"))) { Sizez = 45.0f; Color = FColor(90, 70, 115); }
        else if (NameStr.Contains(TEXT("sofa"))) { Sizez = 70.0f; Color = FColor(72, 90, 122); }
        else if (NameStr.Contains(TEXT("coffee_table"))) { Sizez = 45.0f; Color = FColor(120, 85, 55); }
        else if (NameStr.Contains(TEXT("dining_table"))) { Sizez = 75.0f; Color = FColor(140, 100, 65); }
        else if (NameStr.Contains(TEXT("chair"))) { Sizez = 80.0f; Color = FColor(90, 70, 50); }
        else if (NameStr.Contains(TEXT("fridge"))) { Sizez = 170.0f; Color = FColor(220, 220, 230); }
        else if (NameStr.Contains(TEXT("shower")) || NameStr.Contains(TEXT("toilet")) || NameStr.Contains(TEXT("sink"))) { Sizez = 80.0f; Color = FColor(200, 220, 230); }
        else if (NameStr.Contains(TEXT("bookshelf"))) { Sizez = 180.0f; Color = FColor(110, 75, 45); }
        else if (NameStr.Contains(TEXT("wardrobe"))) { Sizez = 200.0f; Color = FColor(90, 65, 45); }
        else if (NameStr.Contains(TEXT("desk"))) { Sizez = 75.0f; Color = FColor(130, 90, 60); }
        else if (NameStr.Contains(TEXT("tv"))) { Sizez = 50.0f; Color = FColor(38, 38, 46); }
        else if (NameStr.Contains(TEXT("nightstand"))) { Sizez = 50.0f; Color = FColor(110, 82, 56); }
        else if (NameStr.Contains(TEXT("island"))) { Sizez = 90.0f; Color = FColor(140, 122, 95); }
        else { Sizez = 60.0f; Color = FColor(140, 140, 132); }

        float Zcenter = Sizez * 0.5f;
        FVector Location(Cx, Cy, Zcenter);
        FRotator Rotation(0.f, 0.f, 0.f);

        AActor* ObsActor = World->SpawnActor<AActor>(AActor::StaticClass(), Location, Rotation, Params);
        if (!ObsActor) continue;
        ObsActor->SetActorLabel(FString("Obs_") + NameStr);

        UStaticMeshComponent* MeshComp = NewObject<UStaticMeshComponent>(ObsActor);
        if (ObsActor->GetRootComponent())
            MeshComp->AttachToComponent(ObsActor->GetRootComponent(), FAttachmentTransformRules::KeepWorldTransform);
        else
            ObsActor->SetRootComponent(MeshComp);
        ObsActor->AddInstanceComponent(MeshComp);
        MeshComp->SetStaticMesh(CubeMesh);
        MeshComp->SetWorldScale3D(FVector(Sizex / 100.0f, Sizey / 100.0f, Sizez / 100.0f));
        MeshComp->SetMobility(EComponentMobility::Static);
        MeshComp->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
        MeshComp->SetCollisionProfileName(TEXT("BlockAll"));
        MeshComp->SetGenerateOverlapEvents(true);

        // Create colored MID from base material
        if (BaseMat)
        {
            UMaterialInstanceDynamic* MID = UMaterialInstanceDynamic::Create(BaseMat, MeshComp);
            if (MID) {
                MID->SetVectorParameterValue(TEXT("Color"), FLinearColor(Color));
                MeshComp->SetMaterial(0, MID);
            }
        }
        MeshComp->RegisterComponent();
        ++Spawned;
    }
    UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] SpawnObstacleVisuals: spawned %d obstacle actors (open-plan apartment v3)"), Spawned);
}
'''

H_DECLARATION = "    // Spawns 3D obstacle visuals from HOME_OBSTACLES at BeginPlay (LiDAR + visual)\n    void SpawnObstacleVisuals();\n"

def safe_write(target_path, content_bytes):
    tmp = r"e:\puppyfangzhen\tmp_patch_v3.tmp"
    with open(tmp, 'wb') as f:
        f.write(content_bytes)
    try:
        # Try os.replace which is atomic on Windows
        if os.path.exists(target_path):
            os.remove(target_path)
        shutil.copy2(tmp, target_path)
        os.remove(tmp)
        return True
    except Exception as e:
        print(f"  Copy failed: {e}")
        # Try alternate: copy with different approach
        try:
            with open(target_path, 'wb') as f:
                f.write(content_bytes)
            os.remove(tmp)
            return True
        except Exception as e2:
            print(f"  Direct write also failed: {e2}")
            return False

def read_file(path):
    # Try to restore from original .bak_v3 if it exists, otherwise read current
    # First check for .bak_v3 (from our previous run)
    bak = path + ".bak_v3"
    if os.path.exists(bak):
        print(f"  Restoring from backup: {bak}")
        with open(bak, 'rb') as f:
            raw = f.read()
    else:
        with open(path, 'rb') as f:
            raw = f.read()
    for enc in ['utf-8', 'utf-8-sig', 'gbk', 'latin-1']:
        try:
            return raw.decode(enc), enc
        except:
            continue
    return raw.decode('latin-1'), 'latin-1'

def write_file(path, text, enc='utf-8'):
    return safe_write(path, text.encode(enc))

def patch_cpp():
    print(f"=== Patching {CPP_TARGET} ===")
    content, enc = read_file(CPP_TARGET)

    # 0. Ensure includes
    if "MaterialInstanceDynamic.h" not in content:
        inc_marker = '#include "Materials/MaterialInterface.h"'
        if inc_marker in content:
            content = content.replace(inc_marker, inc_marker + '\n#include "Materials/MaterialInstanceDynamic.h"', 1)
            print("  [0] Added MaterialInstanceDynamic.h include")

    # 1. Replace HOME_OBSTACLES array - find start and end lines
    lines = content.split('\n')
    start_idx = None
    end_idx = None
    for i, line in enumerate(lines):
        if 'static const FHomeObstacle HOME_OBSTACLES[]' in line and '=' in line:
            start_idx = i
        if start_idx is not None and 'static const int NUM_HOME_OBSTACLES' in line and i > start_idx:
            end_idx = i
            break

    if start_idx is not None and end_idx is not None:
        new_lines = NEW_OBSTACLES.split('\n')
        lines = lines[:start_idx] + new_lines + [''] + lines[end_idx:]
        content = '\n'.join(lines)
        print(f"  [1] Replaced HOME_OBSTACLES array (lines {start_idx}-{end_idx})")
    else:
        print(f"  ERROR: Cannot find HOME_OBSTACLES markers (start={start_idx}, end={end_idx})")
        return False

    # 2. Replace ROBOT_RADIUS_CM line
    content = content.replace("static constexpr float ROBOT_RADIUS_CM = 10.0f;",
                              "static constexpr float ROBOT_RADIUS_CM = 5.0f;")
    if "ROBOT_RADIUS_CM = 5.0f" in content:
        print("  [2] Updated ROBOT_RADIUS_CM to 5.0f")

    # 2b. Update pedestrian positions line by line
    ped_replacements = [
        ('{ TEXT("person_1"),  2.0f, -2.5f, -0.04f,  0.00f }', '{ TEXT("person_1"), -0.50f, -2.50f, -0.03f,  0.00f }'),
        ('{ TEXT("person_2"), -2.5f,  2.8f,  0.00f,  0.04f }', '{ TEXT("person_2"),  1.20f, -0.90f,  0.00f,  0.03f }'),
        ('{ TEXT("person_3"), -3.0f, -2.0f,  0.03f,  0.00f }', '{ TEXT("person_3"),  3.50f,  0.50f,  0.02f,  0.00f }'),
        ('{ TEXT("person_4"), -1.0f, -1.0f,  0.04f,  0.00f }', '{ TEXT("person_4"), -3.50f,  3.00f,  0.00f, -0.02f }'),
        ('{ TEXT("person_5"),  1.5f, -1.5f,  0.04f,  0.00f }', '{ TEXT("person_5"), -0.80f,  2.50f,  0.02f,  0.00f }'),
    ]
    ped_ok = 0
    for old, new in ped_replacements:
        if old in content:
            content = content.replace(old, new, 1)
            ped_ok += 1
        elif new in content:
            ped_ok += 1
    print(f"  [2b] Updated {ped_ok}/5 pedestrian positions")

    # 2c. Update initial position
    content = content.replace(
        "constexpr double  INIT_Y_M  = -3.0;   // scene_home.robot.initial_pose.y",
        "constexpr double  INIT_Y_M  = -2.7;   // scene_home.robot.initial_pose.y"
    )
    content = content.replace(
        "static_cast<float>(INIT_Y_M * 100.0),   // -300 cm",
        "static_cast<float>(INIT_Y_M * 100.0),   // -270 cm"
    )
    content = content.replace("(-100,-300) <<<<<", "(-100,-270) <<<<<")
    content = content.replace("(-1m,-3m)", "(-1m,-2.7m)")
    print("  [2c] Updated initial Y position to -2.7m")

    # 3. Remove any old SpawnObstacleVisuals function if present
    func_start_marker = "void APuppyRobotPawn::SpawnObstacleVisuals()"
    if func_start_marker in content:
        # Find the function and remove it (up to the next function)
        idx = content.find(func_start_marker)
        # Find next "void APuppyRobotPawn::" after this
        next_func = content.find("\nvoid APuppyRobotPawn::", idx + 10)
        if next_func >= 0:
            content = content[:idx] + content[next_func+1:]  # +1 to skip the leading \n
            print("  [3-clean] Removed old SpawnObstacleVisuals function")

    # 3b. Add SpawnObstacleVisuals() call in BeginPlay - find SpawnPedestriansFromScene(); that's followed by }
    if "SpawnObstacleVisuals();" not in content:
        # Find the pattern: line with "SpawnPedestriansFromScene();" that is followed by a closing brace
        lines = content.split('\n')
        for i, line in enumerate(lines):
            stripped = line.strip()
            if stripped == "SpawnPedestriansFromScene();" and i+1 < len(lines) and lines[i+1].strip() == "}":
                # This is the one in BeginPlay
                indent = line[:len(line) - len(line.lstrip())]
                lines.insert(i+1, f"{indent}// Spawn 3D obstacle visuals from HOME_OBSTACLES so LiDAR can see them.")
                lines.insert(i+2, f"{indent}SpawnObstacleVisuals();")
                content = '\n'.join(lines)
                print(f"  [3] Added SpawnObstacleVisuals() call after line {i}")
                break
        else:
            print("  WARNING: Could not find SpawnPedestriansFromScene() call site")

    # 4. Add SpawnObstacleVisuals function implementation before SpawnPedestriansFromScene
    if func_start_marker not in content:
        def_marker = "void APuppyRobotPawn::SpawnPedestriansFromScene()"
        idx = content.find(def_marker)
        if idx >= 0:
            content = content[:idx] + SPAWN_FUNC + "\n" + content[idx:]
            print("  [4] Added SpawnObstacleVisuals() implementation")

    # Write
    backup = CPP_TARGET + ".bak_v3"
    if not os.path.exists(backup):
        # We already read from .bak if it existed, so this is the original
        pass

    if write_file(CPP_TARGET, content, enc):
        print(f"  [W] Written {len(content)} chars to {CPP_TARGET}")
        return True
    return False

def patch_h():
    print(f"\n=== Patching {H_TARGET} ===")
    content, enc = read_file(H_TARGET)

    if "SpawnObstacleVisuals" in content:
        print("  SpawnObstacleVisuals already declared, skip")
        return True

    lines = content.split('\n')
    inserted = False
    for i, line in enumerate(lines):
        if line.strip() == "void SpawnPedestriansFromScene();" or line.strip() == "void ReinitRobotDogVisuals();":
            indent = line[:len(line) - len(line.lstrip())]
            lines.insert(i+1, f"{indent}// Spawns 3D obstacle visuals from HOME_OBSTACLES at BeginPlay (LiDAR + visual)")
            lines.insert(i+2, f"{indent}void SpawnObstacleVisuals();")
            inserted = True
            break

    if inserted:
        content = '\n'.join(lines)
        print("  Added SpawnObstacleVisuals() declaration")
    else:
        print("  ERROR: Could not find insertion point in header")
        return False

    if write_file(H_TARGET, content, enc):
        print(f"  Written {len(content)} chars to {H_TARGET}")
        return True
    return False

def verify():
    print(f"\n=== Verifying ===")
    with open(CPP_TARGET, 'rb') as f:
        cpp = f.read().decode('utf-8', errors='replace')
    with open(H_TARGET, 'rb') as f:
        h = f.read().decode('utf-8', errors='replace')

    checks_cpp = [
        ("wall_north", "north outer wall"),
        ("wall_south_west", "south-west outer wall"),
        ("wall_east", "east outer wall"),
        ("wall_v_kitchen", "kitchen vertical wall"),
        ("kitchen_island", "kitchen island"),
        ("ROBOT_RADIUS_CM = 5.0f", "robot radius 5cm"),
        ("INIT_Y_M  = -2.7", "initial Y -2.7m"),
        ("SpawnObstacleVisuals();", "spawn call in BeginPlay"),
        ("void APuppyRobotPawn::SpawnObstacleVisuals()", "spawn function"),
        ("LoadObject<UStaticMesh>", "runtime mesh loading"),
        ("open-plan apartment v3", "v3 log message"),
        ('person_1.*-0.50f.*-2.50f', "person_1 position"),
        ('person_2.*1.20f.*-0.90f', "person_2 position"),
        ('person_3.*3.50f.*0.50f', "person_3 position"),
    ]
    import re
    all_ok = True
    for s, desc in checks_cpp:
        if re.search(s, cpp):
            print(f"  CPP OK: {desc}")
        else:
            print(f"  CPP FAIL: {desc}")
            all_ok = False

    if "void SpawnObstacleVisuals();" in h:
        print(f"  H   OK: header declaration")
    else:
        print(f"  H   FAIL: header declaration")
        all_ok = False

    return all_ok

if __name__ == "__main__":
    ok = patch_cpp() and patch_h()
    if ok:
        ok = verify()
    if ok:
        print("\n*** PATCH SUCCESSFUL ***")
    else:
        print("\n*** PATCH FAILED ***")
    sys.exit(0 if ok else 1)
