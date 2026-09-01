#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Auto Patrol Full Verification & Image Capture
=============================================
Runs full auto-patrol simulation through all rooms and waypoints,
captures screenshots of the robot at each major room and waypoint,
and verifies 100% completion of the patrol loop.
"""
import os
import sys
import time
import math
import json
import shutil
import numpy as np

os.environ['SDL_VIDEODRIVER'] = 'dummy'
os.environ['USE_STVOC'] = '1'
os.environ['USE_ORCA'] = '1'
os.environ['USE_APF'] = '1'
os.environ['USE_VO'] = '1'

import pygame
from visual_sim import VisualSimulator, ALGORITHMS

def run_auto_patrol_verification():
    print("=" * 60)
    print("闂佸憡鍑归崹鐗堟叏閳哄懎瀚夐柛婵嗗閻濄倝鏌ｅ▎鎰垫畽闁搞倖绮撳畷婵嬪Ω閵夈劎鐛ラ梺鐓庡娴滎亪宕崇粙妫靛湱鈧絺鏅濋悷銏ゆ煙閼稿灚灏柣顐㈡閹风娀宕熼鍕尋...")
    print("=" * 60)

    sim = VisualSimulator()
    sim.reset()
    # 婵炶揪缍€濞夋洟寮妶鍥╃＜闁绘挸瀛╅崐銈夋煙椤戭剙妫楅崢鎾煛閸艾浜炬繛鏉戝悑椤洤鈻?A*+CBF 闂佹崘顕х粔鎾箖鎼达絺鍋撴担鍐棈闁?闂備緡鍓欏鈥斥枔閹殿喚涓嶆俊銈勮兌閵?
    sim.current_algo_idx = 9
    sim.current_algo = "A*+CBF"

    snapshots = []
    visited_targets = set()
    patrol_log = []

    start_time = time.time()
    max_frames = 1200  # 40缂備礁顦扮敮濠勬寬閵忋倖鍎戦柣鏂挎啞椤ρ囨⒒閸屾繃褰х紒杈ㄧ箘閹奸箖宕楅崨顒傞瀺闁诲海鎳撻張顒勫垂濮橆厾鈻旈柍褜鍓熷顐﹀级閹寸偟鐣遍悗鐟版閵堝懐鍔?
    trajectory = []
    min_clearance = float('inf')
    closest_obstacle_name = ""

    for frame in range(max_frames):
        sim.step()
        sim.draw()

        rx, ry = sim.sim.robot_x, sim.sim.robot_y
        trajectory.append((frame, rx, ry))

        # 缂備緡鍠栨晶浠嬪灳濡ソ鍦偓锝庡墰閺嗩剟鏌￠崼婵囨儓婵炲懏甯￠幃顏勵渻閸撗呮喛闂佸湱顣介崑鎾绘煛閸繍妲告い鏂跨Ч瀹?婵犫拃鍕噥缂傚秴顑夊鎯ь煥閸滀焦娈归梺缁樸仜閺呮繂鈻撻幋鐘亾閸︻厼浠辨俊缁㈠灣閳ь剛鎳撻ˇ顖炲矗韫囨洘宕夋繝闈涚墱閻?(闂佸搫鐗嗛幖顐⑩枍閹烘鍋戞俊銈傚亾鐎规洘鍔曢?0.15m)
        from auto_patrol_simulation import OBSTACLES_BBOX
        for name, bbox in OBSTACLES_BBOX:
            xmin, ymin, xmax, ymax = bbox
            dx = max(xmin - rx, 0, rx - xmax)
            dy = max(ymin - ry, 0, ry - ymax)
            dist = math.sqrt(dx*dx + dy*dy) - 0.15
            if dist < min_clearance:
                min_clearance = dist
                closest_obstacle_name = name

        idx = sim.target_idx % len(sim.patrol_targets)
        curr_target_name = sim.patrol_targets[idx][3]
        curr_target_room = ""

        # 閻熸粎澧楅幐璇测攦閳ь剟鏌涢敐鍐ㄥ閻庢艾绉甸敍鍐冀椤垵浠归梺鍛婂笚婵粙骞撻鍫濇闁规鍠楃粣妤呮煛瀹ュ懏宸濋柛?闂佽娼欏鈥澄涢崸妤€绫嶉柤鎼佹涧椤鎮橀悙鑼妞ゆ柨娲╅妵?
        if idx not in visited_targets:
            visited_targets.add(idx)
            safe_name = curr_target_name.replace('*', '_star_').replace('/', '_')
            img_name = f"patrol_step_{len(visited_targets):02d}_{safe_name}.png"
            img_path = os.path.abspath(img_name)
            pygame.image.save(sim.screen, img_path)
            snapshots.append({
                'step': len(visited_targets),
                'target': curr_target_name,
                'room': curr_target_room,
                'position': [round(rx, 3), round(ry, 3)],
                'image_path': img_path,
            })
            print(f"  [閻庣懓妲戦妶鍛姷闁哄鏅滅粙鎴犫偓?{len(visited_targets)}/{len(sim.patrol_targets)}] 闂佸憡甯楁刊浠嬪箵?闂佸憡鎸哥粔瀵告? {curr_target_name} ({curr_target_room}) -> 闂佺鍩栭幐鎼佹儓閸℃鈹嶆繝闈涙閹? {img_name}")

        # 婵犵鈧啿鈧綊鎮樻径灞稿亾閻熺増婀伴柛銊︾缁傚秹宕卞顒侇啎闁?1 闁哄鍎愰崰鏍偩椤掑嫬鏋侀柡澶嬪灱缁愭鏌ら崜韫倣缂佽鲸绻冪粙澶愬棘閹稿海顦ラ梺鍛婂浮椤ｏ妇鑺遍敍鍕＜闁割偁鍨规禒顖炴煥濞戞瀚扮憸鐗堢叀楠炴捇骞囬鈧·鍛存倵閻熺増婀伴柛?
        if sim.rounds_completed >= 1 and sim.target_idx == 0:
            print(f"\n[闂佺懓鐡ㄩ崝鏇熸叏濞?闂佸搫鐗嗛幖顐⑩枍閹烘鍋戞俊銈傚亾闁告埊绻濋獮瀣箛椤掆偓椤娀鎮楅悷鐗堟拱闁?100% 闁诲海鎳撻張顒勫汲閿濆拋鍟呴梽鍥春閸涙潙鎹堕柛顐墰绾捐偐绱掓鏍х仴妞ゆ挸鎽滈弫顔款槼闁? {frame}")
            break

    elapsed = round(time.time() - start_time, 2)

    report = {
        'timestamp': time.time(),
        'algorithm': sim.current_algo,
        'elapsed_seconds': elapsed,
        'frames_simulated': sim.frame,
        'rounds_completed': sim.rounds_completed,
        'total_targets_reached': len(visited_targets),
        'total_targets': len(sim.patrol_targets),
        'total_collisions': sim.total_collisions,
        'stall_events': sim.stall_events,
        'min_clearance_meters': round(min_clearance, 4),
        'closest_obstacle': closest_obstacle_name,
        'skip_count': sim.skip_count,
        'near_miss': sim.near_miss,
        'snapshots': snapshots,
    }

    report_path = os.path.abspath("auto_patrol_verification_report.json")
    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    print("\n" + "=" * 60)
    print("閻庣懓妲戦妶鍛姷闂備焦褰冪粔鐢电矈鐎涙鈻旈幖杈剧秵濞奸箖鏌熼崜褍浠掔€殿喗顨堥埀顒勬涧濡瑩鎮ф惔銏╂閻忕偤鏁崑鎾绘嚒閵堝洦灏?")
    print(f"  缂備胶濮甸〃鍡欐兜閸洖瑙︾€广儱娉?         {sim.current_algo}")
    print(f"  闁诲海鎳撻張顒勫垂濮橆剦鍟呴梽鍥春閸涱喗濮滄い鏃傜摂閸?     {sim.rounds_completed}")
    print(f"  闂備緡鍋呭鑽ゅ垝?闂佸憡甯楁刊浠嬪箵椤忓牊鍊烽悷娆忓濞?   {len(visited_targets)}/{len(sim.patrol_targets)}")
    print(f"  缂佺偓澹嗘晶妤呭箰閹烘鐭楅柟瀵稿У閺呭憡绻涢崱蹇撳⒉闁?     {sim.total_collisions}")
    print(f"  闂佸搫鐗冮崑鎾绘倶韫囨挾绠伴柛娆忕箻瀵煡顢涘杈ㄦ闂佺绻堥崝蹇浰夊顓熷劅? {min_clearance:.4f} 缂?(闂佸搫鐗冮崑鎾绘煙閹帒鍔ョ紒璇查叄濮婃儳顭ㄩ崪浣规闂? '{closest_obstacle_name}')")
    print(f"  闂佸憡銇涢埀顒冨皺缁?闂傚倸娲ゅú鈺佺暦閸楃偐鏋庨柍鈺佸暞濞?    {sim.stall_events}")
    print(f"  闂佺鍩栭幐姝屻亹閸ヮ剙绠ｆい蹇撳缁傚牓鏌熼鍌滃笡闁?     {len(snapshots)}")
    print(f"  闁荤姴娲ら敃銉╁蓟閸ヮ剙绠柕澶堝劜閸熺偤鎮规笟顖氱仩缂?     {report_path}")
    print("=" * 60)

    # 闁诲繐绻愬Λ娑㈠极閹捐绠ｉ柟閭﹀枟閻ｉ亶鏌熼懜鍨碍闁活偄妫欏鍕吋閸涱厾鍘梺?artifacts 闂佺儵鏅╅崰鏍礊?
    target_artifact_dir = r"C:\Users\Administrator\.gemini\antigravity\brain\9f43f43f-3ac5-4496-9072-844d8264c252"
    for snap in snapshots:
        if os.path.exists(snap['image_path']):
            shutil.copy(snap['image_path'], target_artifact_dir)

    return report

if __name__ == '__main__':
    run_auto_patrol_verification()
