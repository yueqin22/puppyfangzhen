# -*- coding: utf-8 -*-
r"""
修复 nav_ue_bridge.cpp 中 printf 的双反斜杠换行转义。

问题: 源码里写成 "...\\n" (3 字符: \ \ n) 时, C 编译后输出的是字面量 "\n"
      两个字符, 而不是换行符。日志因此粘成一行, 例如:
          !!WARNING: Astar rate low\n[Bridge] msg#10000 type=...

修复: 把字符串字面量内的 \\n 收敛为 \n。
      只在 printf/std::printf/fprintf 所在行处理, 且只替换 \\n, 不动其他转义。
"""
import os, shutil, re

SRC = r"E:\puppyfangzhen\cpp_src\bridge\nav_ue_bridge.cpp"
BAD = '\\\\n'   # 实际 3 字符: backslash backslash n
GOOD = '\\n'    # 实际 2 字符: backslash n


def main():
    text = open(SRC, 'r', encoding='utf-8', newline='').read()
    lines = text.split('\n')

    n_lines_fixed = 0
    n_occurrences = 0
    report = []

    for i, line in enumerate(lines):
        if BAD not in line:
            continue
        # 仅处理明显的输出语句行，避免误伤
        if not re.search(r'\b(printf|fprintf|puts|fputs)\b', line):
            continue
        cnt = line.count(BAD)
        lines[i] = line.replace(BAD, GOOD)
        n_lines_fixed += 1
        n_occurrences += cnt
        report.append((i + 1, cnt, lines[i].strip()[:96]))

    # 顺带修正过时的窗口文案: 采样窗口已从 2900 帧改为 300 帧 (10s)
    n_label = 0
    for i, line in enumerate(lines):
        if '97s-displacement' in line:
            lines[i] = line.replace('97s-displacement', 'win10s-displacement')
            n_label += 1

    bak = SRC + '.bak_escapes'
    if not os.path.exists(bak):
        shutil.copy2(SRC, bak)
        print('backup ->', bak)

    open(SRC, 'w', encoding='utf-8', newline='').write('\n'.join(lines))

    print('=' * 60)
    print('lines fixed        :', n_lines_fixed)
    print('occurrences fixed  :', n_occurrences)
    print('labels renamed     :', n_label)
    print('-' * 60)
    for n, c, s in report:
        print('  line %-5d x%d  %s' % (n, c, s))


if __name__ == '__main__':
    main()
