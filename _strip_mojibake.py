# -*- coding: utf-8 -*-
"""
安全清理 nav_ue_bridge.cpp 中被编码损坏(mojibake)的超长行内注释。

策略(保守):
  - 只处理 len(line) > THRESH 的行
  - 在该行中定位第一个「不在字符串字面量内」的 '//'
  - 保留 '//' 之前的代码原文, 注释替换为简短占位
  - 若找不到安全的 '//', 则跳过该行(不动)
验证:
  - 行数必须不变
  - 每一行 '//' 之前的代码前缀必须逐字符不变
"""
import sys, os, shutil

SRC = r"E:\puppyfangzhen\cpp_src\bridge\nav_ue_bridge.cpp"
THRESH = 2000
PLACEHOLDER = "// [comment stripped: encoding-corrupted]"


def find_safe_comment(line):
    """返回第一个不在字符串/字符字面量内的 '//' 下标, 找不到返回 -1"""
    in_str = False
    in_chr = False
    i = 0
    n = len(line)
    while i < n - 1:
        c = line[i]
        if in_str:
            if c == '\\':
                i += 2
                continue
            if c == '"':
                in_str = False
        elif in_chr:
            if c == '\\':
                i += 2
                continue
            if c == "'":
                in_chr = False
        else:
            if c == '"':
                in_str = True
            elif c == "'":
                in_chr = True
            elif c == '/' and line[i + 1] == '/':
                return i
            elif c == '/' and line[i + 1] == '*':
                # 行内块注释起点, 也当作可截断点
                return i
        i += 1
    return -1


def main():
    raw = open(SRC, 'r', encoding='utf-8', newline='').read()
    # 保留原始换行风格: 按 \n 切分, 记录是否 \r\n
    lines = raw.split('\n')
    n_before = len(lines)
    chars_before = sum(len(l) for l in lines)

    changed = []
    out = []
    for idx, line in enumerate(lines, start=1):
        if len(line) <= THRESH:
            out.append(line)
            continue
        # 处理可能的 \r 结尾
        cr = line.endswith('\r')
        body = line[:-1] if cr else line
        pos = find_safe_comment(body)
        if pos < 0:
            out.append(line)
            print("SKIP  line %d (no safe // found), len=%d" % (idx, len(line)))
            continue
        prefix = body[:pos]
        newbody = prefix + PLACEHOLDER
        out.append(newbody + ('\r' if cr else ''))
        changed.append((idx, len(line), len(newbody), prefix))

    n_after = len(out)
    assert n_after == n_before, "line count changed! %d -> %d" % (n_before, n_after)

    # 验证: 每个被改行的代码前缀必须与原文一致
    for (idx, oldlen, newlen, prefix) in changed:
        orig = lines[idx - 1]
        assert orig.startswith(prefix), "prefix mismatch at line %d" % idx
        assert out[idx - 1].startswith(prefix), "written prefix mismatch at line %d" % idx

    bak = SRC + ".bak_mojibake"
    if not os.path.exists(bak):
        shutil.copy2(SRC, bak)
        print("backup ->", bak)
    else:
        print("backup already exists ->", bak)

    open(SRC, 'w', encoding='utf-8', newline='').write('\n'.join(out))

    chars_after = sum(len(l) for l in out)
    print("=" * 62)
    print("lines           : %d (unchanged)" % n_after)
    print("chars  before   : %,d" % chars_before if False else "chars  before   : %d" % chars_before)
    print("chars  after    : %d" % chars_after)
    print("reduction       : %.2f MB (%.1f%%)" % (
        (chars_before - chars_after) / 1048576.0,
        100.0 * (chars_before - chars_after) / max(1, chars_before)))
    print("lines stripped  : %d" % len(changed))
    print("-" * 62)
    for (idx, oldlen, newlen, prefix) in changed[:40]:
        print("  line %-5d %8d -> %-4d | code: %s" % (idx, oldlen, newlen, prefix.strip()[:64]))
    if len(changed) > 40:
        print("  ... and %d more" % (len(changed) - 40))


if __name__ == '__main__':
    main()
