#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sync_ue_source.py -- 单向同步 Puppy UE 工程源码到本地镜像。

背景 (guihua20260812.md §17 风险 "UE 源码分叉"):
    D:\\puppy_ue\\Source\\PuppyNav\\  是 UE 机器人 Pawn / LiDAR / TCP 服务器的
    **权威源码** (UnrealBuildTool 实际编译它)。
    E:\\puppyfangzhen\\cpp_src\\bridge\\ue_modules\\  曾是其一份手工复制的、长期
    失真的分叉副本 (PuppyRobotPawn.cpp 停在 08-13, 9.3KB)，没有任何构建引用它，
    却持续制造"看着像改了、其实没生效"的困惑。

本脚本的契约 (单一数据源):
    * 权威方向唯一:  D:\\puppy_ue  ->  ue_modules  (单向!)
    * 绝不反向 (ue_modules 的改动不会、也不能流回 UE 工程)。
    * 每次改了 UE 工程后，跑一次本脚本，让镜像成为可信的只读快照/参考。

用法:
    python scripts/sync_ue_source.py                 # 用默认路径同步
    python scripts/sync_ue_source.py --dry-run       # 只列出会复制的文件
    python scripts/sync_ue_source.py --src X --dst Y # 自定义源/目标
"""
import argparse
import os
import shutil
import sys

DEFAULT_SRC = r"D:\puppy_ue\Source\PuppyNav"
DEFAULT_DST = r"E:\puppyfangzhen\cpp_src\bridge\ue_modules"
EXTS = (".cpp", ".h")


def main():
    ap = argparse.ArgumentParser(description="单向同步 UE 源码到本地镜像 (权威: src -> dst)")
    ap.add_argument("--src", default=DEFAULT_SRC, help="权威源目录 (UE 工程)")
    ap.add_argument("--dst", default=DEFAULT_DST, help="镜像目标目录 (ue_modules)")
    ap.add_argument("--dry-run", action="store_true", help="只列出，不复制")
    args = ap.parse_args()

    src = os.path.abspath(args.src)
    dst = os.path.abspath(args.dst)

    if not os.path.isdir(src):
        print(f"[sync_ue_source] ERROR: 源目录不存在: {src}", file=sys.stderr)
        return 2

    if not args.dry_run:
        os.makedirs(dst, exist_ok=True)

    copied = 0
    skipped = 0
    for fn in sorted(os.listdir(src)):
        if not fn.lower().endswith(EXTS):
            continue
        s = os.path.join(src, fn)
        if not os.path.isfile(s):
            continue
        d = os.path.join(dst, fn)
        # 仅在内容不同 (或目标缺失) 时复制，避免无谓写入。
        if os.path.exists(d) and _same_file(s, d):
            skipped += 1
            continue
        if args.dry_run:
            print(f"[sync_ue_source] WOULD COPY  {fn}")
        else:
            shutil.copy2(s, d)
            print(f"[sync_ue_source] COPIED      {fn}")
        copied += 1

    print(f"[sync_ue_source] done. copied={copied} skipped={skipped} "
          f"(src={src} -> dst={dst})")
    return 0


def _same_file(a, b):
    try:
        sa, sb = os.stat(a), os.stat(b)
        if sa.st_size != sb.st_size:
            return False
        # 内容比对 (文件不大, 直接读)。
        with open(a, "rb") as fa, open(b, "rb") as fb:
            return fa.read() == fb.read()
    except OSError:
        return False


if __name__ == "__main__":
    sys.exit(main())
