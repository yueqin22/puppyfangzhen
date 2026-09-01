#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
apply=0
include_results=0

for arg in "$@"; do
  case "$arg" in
    --apply) apply=1 ;;
    --include-results) include_results=1 ;;
    *)
      echo "Usage: $0 [--apply] [--include-results]" >&2
      exit 2
      ;;
  esac
done

prune_results=()
if [[ "$include_results" -eq 0 ]]; then
  prune_results=(
    -path "$root/ablation_results" -o
    -path "$root/ablation_results_v2" -o
    -path "$root/ablation_results_v3" -o
    -path "$root/experiment_results" -o
    -path "$root/eval_results" -o
    -path "$root/eval_run" -o
    -path "$root/eval_run2" -o
    -path "$root/eval_run3" -o
    -path "$root/eval_run4" -o
    -path "$root/sensitivity_results" -o
    -path "$root/screenshots"
  )
fi

tmp_file="$(mktemp)"
trap 'rm -f "$tmp_file"' EXIT

if [[ "${#prune_results[@]}" -gt 0 ]]; then
  find "$root" \( "${prune_results[@]}" \) -prune -o \( \
    -name __pycache__ -o \
    -name .pytest_cache -o \
    -name '*.pyc' -o \
    -name '*.pyo' -o \
    -name '*.obj' -o \
    -name '*.exe' -o \
    -name '*.log' -o \
    -name '*_log.txt' -o \
    -name reattach_out.txt -o \
    -name Ogre.log -o \
    -name visual_status.json \
  \) -print > "$tmp_file"
else
  find "$root" \( \
    -name __pycache__ -o \
    -name .pytest_cache -o \
    -name '*.pyc' -o \
    -name '*.pyo' -o \
    -name '*.obj' -o \
    -name '*.exe' -o \
    -name '*.log' -o \
    -name '*_log.txt' -o \
    -name reattach_out.txt -o \
    -name Ogre.log -o \
    -name visual_status.json \
  \) -print > "$tmp_file"
fi

find "$root" -maxdepth 1 -type f \( \
  -name '*.png' -o \
  -name '*.jpg' -o \
  -name '*.jpeg' -o \
  -name 'frames_*.gv' -o \
  -name 'frames_*.pdf' \
\) -print >> "$tmp_file"

sort -u "$tmp_file" -o "$tmp_file"
count="$(wc -l < "$tmp_file" | tr -d ' ')"

if [[ "$count" -eq 0 ]]; then
  echo "No generated artifacts matched the cleanup rules."
  exit 0
fi

if [[ "$apply" -eq 1 ]]; then
  echo "APPLY - removing $count paths"
  while IFS= read -r path; do
    case "$path" in
      "$root"/*) rm -rf -- "$path" ;;
      *) echo "Refusing to remove path outside workspace: $path" >&2; exit 1 ;;
    esac
  done < "$tmp_file"
else
  echo "DRY RUN - matched $count paths"
  cat "$tmp_file"
  echo "Preview only. Re-run with --apply to delete these artifacts."
fi
