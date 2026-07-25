"""
Pareto Multi-Objective Optimization (v4.0)
===========================================
Pareto front computation and multi-criteria decision making (MCDM)
for frontier selection and other multi-objective problems.

Methods:
  1. Pareto dominance check
  2. Non-dominated sorting (NSGA-II style)
  3. TOPSIS (Technique for Order Preference by Similarity to Ideal Solution)
  4. Crowding distance for diversity preservation

References:
  - Deb et al. (2002) "A fast and elitist multiobjective genetic algorithm: NSGA-II"
  - Hwang & Yoon (1981) "Multiple attribute decision making: methods and applications"
"""
import math
import numpy as np


def dominates(a_obj, b_obj, higher_is_better):
    """Check if solution `a` Pareto-dominates solution `b`.

    Args:
        a_obj: list of objective values for a
        b_obj: list of objective values for b
        higher_is_better: list of bool, same length as objectives.
                          True = higher is better (maximize), False = lower is better (minimize)

    Returns:
        True if a dominates b (a is >= b in all objectives, and > in at least one)
    """
    assert len(a_obj) == len(b_obj) == len(higher_is_better)
    at_least_one_better = False
    for i, (ai, bi, hib) in enumerate(zip(a_obj, b_obj, higher_is_better)):
        if hib:
            if ai < bi:
                return False
            if ai > bi:
                at_least_one_better = True
        else:
            if ai > bi:
                return False
            if ai < bi:
                at_least_one_better = True
    return at_least_one_better


def find_pareto_front(solutions, higher_is_better):
    """Find the Pareto-optimal front using fast non-dominated sort.

    Args:
        solutions: list of (objectives_tuple, payload) tuples
        higher_is_better: list of bool for each objective

    Returns:
        list of (objectives_tuple, payload) tuples on the Pareto front
    """
    if not solutions:
        return []

    n = len(solutions)
    dominated_count = [0] * n
    dominates_list = [[] for _ in range(n)]

    for i in range(n):
        for j in range(i + 1, n):
            obji = solutions[i][0]
            objj = solutions[j][0]
            if dominates(obji, objj, higher_is_better):
                dominates_list[i].append(j)
                dominated_count[j] += 1
            elif dominates(objj, obji, higher_is_better):
                dominates_list[j].append(i)
                dominated_count[i] += 1

    pareto = [solutions[i] for i in range(n) if dominated_count[i] == 0]
    return pareto


def non_dominated_sort(solutions, higher_is_better):
    """Full non-dominated sorting, returns list of fronts (front 0 = Pareto).

    Returns:
        list of lists, each sublist is one front of (obj_tuple, payload)
    """
    if not solutions:
        return []

    n = len(solutions)
    dominated_count = [0] * n
    dominates_list = [[] for _ in range(n)]

    for i in range(n):
        for j in range(i + 1, n):
            obji = solutions[i][0]
            objj = solutions[j][0]
            if dominates(obji, objj, higher_is_better):
                dominates_list[i].append(j)
                dominated_count[j] += 1
            elif dominates(objj, obji, higher_is_better):
                dominates_list[j].append(i)
                dominated_count[i] += 1

    fronts = []
    current_front = [i for i in range(n) if dominated_count[i] == 0]
    while current_front:
        fronts.append([solutions[i] for i in current_front])
        next_front = []
        for i in current_front:
            for j in dominates_list[i]:
                dominated_count[j] -= 1
                if dominated_count[j] == 0:
                    next_front.append(j)
        current_front = next_front

    return fronts


def crowding_distance(solutions):
    """Compute crowding distance for diversity in the Pareto front.

    Args:
        solutions: list of (obj_tuple, payload) from the same front

    Returns:
        list of crowding distances, same order as input
    """
    n = len(solutions)
    if n == 0:
        return []
    if n <= 2:
        return [float('inf')] * n

    m = len(solutions[0][0])
    distances = [0.0] * n

    for obj_idx in range(m):
        sorted_indices = sorted(range(n), key=lambda i: solutions[i][0][obj_idx])
        distances[sorted_indices[0]] = float('inf')
        distances[sorted_indices[-1]] = float('inf')

        obj_min = solutions[sorted_indices[0]][0][obj_idx]
        obj_max = solutions[sorted_indices[-1]][0][obj_idx]
        if obj_max - obj_min < 1e-12:
            continue

        for k in range(1, n - 1):
            prev_val = solutions[sorted_indices[k - 1]][0][obj_idx]
            next_val = solutions[sorted_indices[k + 1]][0][obj_idx]
            distances[sorted_indices[k]] += (next_val - prev_val) / (obj_max - obj_min)

    return distances


def topsis(solutions, higher_is_better, weights=None):
    """TOPSIS: Technique for Order Preference by Similarity to Ideal Solution.

    Ranks alternatives by distance to ideal positive and negative solutions.

    Args:
        solutions: list of (obj_tuple, payload)
        higher_is_better: list of bool per objective
        weights: list of float per objective (if None, equal weights)

    Returns:
        list of (obj_tuple, payload, score) sorted by descending score (best first)
        score in [0, 1], higher = better
    """
    if not solutions:
        return []

    n = len(solutions)
    m = len(solutions[0][0])

    if weights is None:
        weights = [1.0 / m] * m
    else:
        w_sum = sum(weights)
        weights = [w / w_sum for w in weights]

    matrix = np.array([s[0] for s in solutions])

    # Step 1: Normalize (Euclidean norm per column)
    norms = np.sqrt(np.sum(matrix ** 2, axis=0))
    norms[norms < 1e-12] = 1.0
    norm_matrix = matrix / norms

    # Step 2: Weighted normalized matrix
    weighted = norm_matrix * np.array(weights)

    # Step 3: Ideal positive (A*) and negative (A-) solutions
    ideal_pos = np.zeros(m)
    ideal_neg = np.zeros(m)
    for j in range(m):
        if higher_is_better[j]:
            ideal_pos[j] = np.max(weighted[:, j])
            ideal_neg[j] = np.min(weighted[:, j])
        else:
            ideal_pos[j] = np.min(weighted[:, j])
            ideal_neg[j] = np.max(weighted[:, j])

    # Step 4: Distance to ideal solutions
    d_pos = np.sqrt(np.sum((weighted - ideal_pos) ** 2, axis=1))
    d_neg = np.sqrt(np.sum((weighted - ideal_neg) ** 2, axis=1))

    # Step 5: Closeness coefficient
    denom = d_pos + d_neg
    denom[denom < 1e-12] = 1.0
    closeness = d_neg / denom  # 0 = worst, 1 = best

    # Sort by descending closeness
    scored = [(solutions[i][0], solutions[i][1], float(closeness[i]))
              for i in range(n)]
    scored.sort(key=lambda x: -x[2])
    return scored


def select_pareto_best(solutions, higher_is_better, weights=None, method='topsis'):
    """Select the best solution from Pareto front using MCDM method.

    Args:
        solutions: list of (obj_tuple, payload)
        higher_is_better: list of bool per objective
        weights: optional weights for MCDM
        method: 'topsis' or 'maxmin' or 'first'

    Returns:
        (best_obj_tuple, best_payload, score) or (None, None, 0.0) if empty
    """
    if not solutions:
        return None, None, 0.0

    # Step 1: Find Pareto front
    pareto = find_pareto_front(solutions, higher_is_better)

    if method == 'first' or len(pareto) == 1:
        return pareto[0][0], pareto[0][1], 1.0 if pareto else 0.0

    if method == 'maxmin':
        # Max-min: choose the solution with the best worst-case normalized objective
        m = len(higher_is_better)
        all_obj = np.array([p[0] for p in pareto])
        mins = np.min(all_obj, axis=0)
        maxs = np.max(all_obj, axis=0)
        ranges = maxs - mins
        ranges[ranges < 1e-12] = 1.0
        # Normalize: for maximize objectives, normalize to [0,1]; for minimize, invert
        norm = np.zeros_like(all_obj)
        for j in range(m):
            if higher_is_better[j]:
                norm[:, j] = (all_obj[:, j] - mins[j]) / ranges[j]
            else:
                norm[:, j] = (maxs[j] - all_obj[:, j]) / ranges[j]
        min_scores = np.min(norm, axis=1)
        best_idx = int(np.argmax(min_scores))
        return pareto[best_idx][0], pareto[best_idx][1], float(min_scores[best_idx])

    # Default: TOPSIS
    scored = topsis(pareto, higher_is_better, weights)
    return scored[0][0], scored[0][1], scored[0][2] if scored else (None, None, 0.0)
