#!/usr/bin/env python3
# Copyright 2026 OOMWOO
# SPDX-License-Identifier: Apache-2.0
"""
Generate a self-hosted star-history SVG chart from the GitHub API.

Fetches the repo's stargazer timeline (authenticated, so no rate limits and no
external "sealed token" to expire), then writes light + dark cumulative-stars
SVGs to .github/assets/. Run in CI on a schedule; the README embeds the
committed SVGs via <picture>, so the chart never depends on a third-party embed.

Env:
  GITHUB_TOKEN        auth token (required in CI; higher rate limit)
  STAR_HISTORY_REPO   owner/repo (default: makerspet/oomwoo)
  STAR_HISTORY_OUT    output dir (default: .github/assets)
"""

from datetime import datetime, timezone
import json
import math
import os
import sys
import urllib.request

REPO = os.environ.get('STAR_HISTORY_REPO', 'makerspet/oomwoo')
OUT_DIR = os.environ.get('STAR_HISTORY_OUT', '.github/assets')
TOKEN = os.environ.get('GITHUB_TOKEN', '')
API = 'https://api.github.com'

W, H = 820, 420
ML, MR, MT, MB = 76, 28, 64, 52

THEMES = {
    'light': {'bg': '#ffffff', 'text': '#57606a', 'title': '#24292f',
              'grid': '#eaecef', 'line': '#2f81f7', 'fill': 'rgba(47,129,247,0.12)',
              'axis': '#d0d7de'},
    'dark': {'bg': '#0d1117', 'text': '#8b949e', 'title': '#e6edf3',
             'grid': '#21262d', 'line': '#58a6ff', 'fill': 'rgba(56,139,253,0.15)',
             'axis': '#30363d'},
}


def gh_get(url):
    req = urllib.request.Request(url)
    req.add_header('Accept', 'application/vnd.github.star+json')
    req.add_header('X-GitHub-Api-Version', '2022-11-28')
    if TOKEN:
        req.add_header('Authorization', 'Bearer ' + TOKEN)
    with urllib.request.urlopen(req, timeout=30) as resp:
        return json.loads(resp.read().decode('utf-8'))


def fetch_star_times(repo):
    times = []
    page = 1
    while page <= 500:                       # GitHub caps stargazers at 400 pages
        data = gh_get('%s/repos/%s/stargazers?per_page=100&page=%d'
                      % (API, repo, page))
        if not data:
            break
        for item in data:
            ts = item.get('starred_at')
            if ts:
                times.append(datetime.strptime(ts, '%Y-%m-%dT%H:%M:%SZ')
                             .replace(tzinfo=timezone.utc))
        if len(data) < 100:
            break
        page += 1
    times.sort()
    return times


def build_series(times, max_points=160):
    """Cumulative (time, count) points, down-sampled for a small SVG."""
    n = len(times)
    pts = [(times[i], i + 1) for i in range(n)]
    if n <= max_points:
        return pts
    step = n / float(max_points)
    idxs = sorted(set([0] + [int(i * step) for i in range(1, max_points)] + [n - 1]))
    return [pts[i] for i in idxs]


def nice_max(v):
    if v <= 1:
        return 1
    mag = 10 ** int(math.floor(math.log10(v)))
    for m in (1, 2, 2.5, 5, 10):
        if v <= m * mag:
            return int(m * mag)
    return int(10 * mag)


def render_svg(series, total, repo, theme):
    c = THEMES[theme]
    bg, text, title = c['bg'], c['text'], c['title']
    grid, line, fill, axis = c['grid'], c['line'], c['fill'], c['axis']
    pw, ph = W - ML - MR, H - MT - MB
    t0 = series[0][0].timestamp()
    tspan = (series[-1][0].timestamp() - t0) or 1.0
    ymax = nice_max(total)

    def sx(dt):
        return ML + (dt.timestamp() - t0) / tspan * pw

    def sy(v):
        return MT + ph - (float(v) / ymax) * ph

    line_pts = ' '.join('%.1f,%.1f' % (sx(dt), sy(v)) for (dt, v) in series)
    area = ('%.1f,%.1f ' % (ML, MT + ph)) + line_pts + \
        (' %.1f,%.1f' % (ML + pw, MT + ph))

    p = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
         'viewBox="0 0 %d %d" font-family="-apple-system,Segoe UI,Helvetica,'
         'Arial,sans-serif">' % (W, H, W, H)]
    p.append('<rect width="%d" height="%d" rx="6" fill="%s"/>' % (W, H, bg))
    p.append('<text x="%d" y="30" fill="%s" font-size="17" font-weight="600">'
             'Star History</text>' % (ML, title))
    p.append('<text x="%d" y="30" text-anchor="end" fill="%s" font-size="13">'
             '%s &#183; %s stars</text>' % (W - MR, text, repo, format(total, ',')))
    for i in range(6):
        v = ymax * i / 5.0
        y = sy(v)
        p.append('<line x1="%d" y1="%.1f" x2="%d" y2="%.1f" stroke="%s" '
                 'stroke-width="1"/>' % (ML, y, ML + pw, y, grid))
        p.append('<text x="%d" y="%.1f" text-anchor="end" fill="%s" '
                 'font-size="11">%s</text>'
                 % (ML - 8, y + 4, text, format(int(round(v)), ',')))
    for i in range(5):
        dt = datetime.fromtimestamp(t0 + tspan * i / 4.0, tz=timezone.utc)
        x = ML + pw * i / 4.0
        anchor = 'start' if i == 0 else ('end' if i == 4 else 'middle')
        p.append('<text x="%.1f" y="%d" text-anchor="%s" fill="%s" '
                 'font-size="11">%s</text>'
                 % (x, MT + ph + 20, anchor, text, dt.strftime('%b %d')))
    p.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="%s" '
             'stroke-width="1.5"/>' % (ML, MT + ph, ML + pw, MT + ph, axis))
    p.append('<polygon points="%s" fill="%s" stroke="none"/>' % (area, fill))
    p.append('<polyline points="%s" fill="none" stroke="%s" stroke-width="2.5" '
             'stroke-linejoin="round" stroke-linecap="round"/>' % (line_pts, line))
    p.append('<circle cx="%.1f" cy="%.1f" r="3.5" fill="%s"/>'
             % (sx(series[-1][0]), sy(series[-1][1]), line))
    p.append('</svg>')
    return '\n'.join(p) + '\n'


def main():
    times = fetch_star_times(REPO)
    if not times:
        print('no stargazer timestamps fetched', file=sys.stderr)
        return 1
    total = len(times)
    series = build_series(times)
    os.makedirs(OUT_DIR, exist_ok=True)
    for theme in ('light', 'dark'):
        path = os.path.join(OUT_DIR, 'star-history-%s.svg' % theme)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(render_svg(series, total, REPO, theme))
        print('wrote %s (%d stars, %d points)' % (path, total, len(series)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
