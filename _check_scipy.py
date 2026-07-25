#!/usr/bin/env python3
"""Check scipy availability and AMCL syntax."""
try:
    from scipy.ndimage import distance_transform_edt
    print("scipy OK")
except ImportError as e:
    print(f"scipy MISSING: {e}")
    import numpy as np
    # Test manual distance transform fallback
    print("Will use numpy fallback for distance transform")

# Test AMCL import
try:
    import ast
    with open("amcl.py") as f:
        ast.parse(f.read())
    print("amcl.py syntax OK")
except SyntaxError as e:
    print(f"amcl.py syntax ERROR: {e}")

# Test autonomous_nav import
try:
    with open("autonomous_nav.py") as f:
        ast.parse(f.read())
    print("autonomous_nav.py syntax OK")
except SyntaxError as e:
    print(f"autonomous_nav.py syntax ERROR: {e}")
