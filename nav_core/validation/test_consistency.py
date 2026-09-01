"""M2 §6.3 — Python/C++ PathValidator field-level consistency test.

Reads the scenario+result JSON produced by cpp_src/tests/pv_consistency_harness
(the single source of truth for inputs), reconstructs the identical OccupancyGrid
in Python, runs the mirrored Python PathValidator, and asserts that every output
field matches the C++ reference (valid / code / min_clearance / endpoint_error).

Run:
    python nav_core/validation/test_consistency.py
or (pytest):
    pytest nav_core/validation/test_consistency.py
"""
import os
import sys
import json
import math
import importlib.util

# This mirror module is self-contained (only depends on `math`). Load it
# directly by file to avoid triggering nav_core/__init__.py (which pulls in
# numpy through nav_core.mapping) — the consistency test must run anywhere.
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))


def _load_pv():
    spec = importlib.util.spec_from_file_location(
        "pv_mirror", os.path.join(_HERE, "path_validator.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_pv = _load_pv()
OccupancyGrid = _pv.OccupancyGrid
PathValidator = _pv.PathValidator
PathValidationOptions = _pv.PathValidationOptions
PathValidationCode = _pv.PathValidationCode

DEFAULT_JSON = os.path.join(
    _ROOT, "cpp_src", "tests", "build_pv", "pv_consistency.json")


def _build_grid(d):
    g = OccupancyGrid(width=d["width"], height=d["height"],
                      resolution=d["resolution"],
                      origin_x=d["origin_x"], origin_y=d["origin_y"])
    g.log_odds = list(d["log_odds"])
    return g


def _build_options(d):
    o = PathValidationOptions()
    o.min_path_clearance = d["min_path_clearance"]
    o.goal_tolerance = d["goal_tolerance"]
    o.sample_step = d["sample_step"]
    o.unknown_is_blocked = d["unknown_is_blocked"]
    o.min_path_length_ratio = d["min_path_length_ratio"]
    o.footprint = [tuple(p) for p in d["footprint"]]
    return o


def run_consistency(json_path):
    with open(json_path, "r") as f:
        doc = json.load(f)
    failures = []
    checked = 0
    for sc in doc["scenarios"]:
        grid = _build_grid(sc["grid"])
        opts = _build_options(sc["options"])
        v = PathValidator(grid)
        r = v.validate(sc["path"], sc["start"][0], sc["start"][1],
                       sc["goal"][0], sc["goal"][1], opts)
        ref = sc["cpp_result"]
        ref_code = int(ref["code"])
        checked += 1
        # field-level diff
        if r.valid != ref["valid"]:
            failures.append(f"{sc['name']}: valid {r.valid} != cpp {ref['valid']}")
        if int(r.code) != ref_code:
            failures.append(f"{sc['name']}: code {PathValidationCode.to_string(int(r.code))} "
                            f"({int(r.code)}) != cpp {ref_code}")
        if ref["min_clearance"] is None:
            # C++ left min_clearance as +inf (early-return cases, e.g. EMPTY_PATH);
            # Python uses float('inf') too — both agree it is undefined.
            if math.isfinite(r.min_clearance):
                failures.append(f"{sc['name']}: min_clearance {r.min_clearance:.6f} "
                                f"but cpp is inf")
        elif abs(r.min_clearance - ref["min_clearance"]) > 1e-4:
            failures.append(f"{sc['name']}: min_clearance {r.min_clearance:.6f} "
                            f"!= cpp {ref['min_clearance']:.6f}")
        if abs(r.endpoint_error - ref["endpoint_error"]) > 1e-4:
            failures.append(f"{sc['name']}: endpoint_error {r.endpoint_error:.6f} "
                            f"!= cpp {ref['endpoint_error']:.6f}")
    return checked, failures


def test_cpp_python_consistency():
    json_path = os.environ.get("PV_CONSISTENCY_JSON", DEFAULT_JSON)
    assert os.path.exists(json_path), f"missing consistency JSON: {json_path}"
    checked, failures = run_consistency(json_path)
    assert not failures, (
        f"{len(failures)}/{checked} scenarios diverge Python vs C++:\n  "
        + "\n  ".join(failures)
    )
    print(f"[PASS] Python/C++ PathValidator consistent on {checked} scenarios")


if __name__ == "__main__":
    json_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_JSON
    if not os.path.exists(json_path):
        print(f"ERROR: missing consistency JSON: {json_path}")
        print("Build it first: cpp_src/tests/build_pv/pv_consistency_harness.exe "
              "cpp_src/tests/build_pv/pv_consistency.json")
        sys.exit(2)
    checked, failures = run_consistency(json_path)
    if failures:
        print(f"FAILED: {len(failures)}/{checked} scenarios diverge:")
        for fmsg in failures:
            print("  " + fmsg)
        sys.exit(1)
    print(f"[PASS] Python/C++ PathValidator field-level consistent on {checked} scenarios")
    sys.exit(0)
