"""Session Store: persist map, frontier memory, and run metadata.

Solves the "experience continuity" problem (gaijin1.md section 6.4):
  - Map (log_odds, visited)
  - Unreachable goals
  - Visited frontiers
  - Run parameters summary
  - Algorithm version

This allows the robot to resume exploration with accumulated experience
rather than restarting from scratch each time.
"""
import os
import json
import time
import numpy as np


class SessionStore:
    """Persists navigation session state across runs."""

    def __init__(self, map_file, meta_file=None):
        self.map_file = map_file
        if meta_file is None:
            base, _ = os.path.splitext(map_file)
            meta_file = base + '_meta.json'
        self.meta_file = meta_file
        self.metadata = {
            'version': '1.0.0',
            'created_at': None,
            'updated_at': None,
            'algorithm_version': 'nav_core_1.0',
            'run_count': 0,
            'total_frames': 0,
            'best_coverage': 0.0,
            'params_summary': {},
        }

    def save_map(self, occ_grid):
        """Save occupancy grid to npz file."""
        np.savez_compressed(self.map_file,
                            log_odds=occ_grid.log_odds,
                            visited=occ_grid.visited)
        self._update_meta('map_saved', True)

    def load_map(self, occ_grid):
        """Load occupancy grid from npz file. Returns True if loaded."""
        if not os.path.exists(self.map_file):
            return False
        try:
            data = np.load(self.map_file)
            occ_grid.log_odds = data['log_odds']
            occ_grid.visited = data['visited']
            return True
        except Exception as e:
            print(f"[Session] Failed to load map: {e}")
            return False

    def save_frontier_memory(self, frontier_manager):
        """Save frontier manager state (unreachable_goals, visited_frontiers)."""
        state = frontier_manager.get_state()
        # Convert sets to lists for JSON serialization
        serializable = {
            'unreachable_goals': [list(x) for x in state['unreachable_goals']],
            'visited_frontiers': {f"{k[0]},{k[1]}": v for k, v in state['visited_frontiers'].items()},
            'visited_count': {f"{k[0]},{k[1]}": v for k, v in state['visited_count'].items()},
            'failure_reasons': {f"{k[0]},{k[1]}": v for k, v in state.get('failure_reasons', {}).items()},
        }
        self._update_meta('frontier_memory', serializable)

    def load_frontier_memory(self, frontier_manager):
        """Load frontier manager state. Returns True if loaded."""
        meta = self._load_meta()
        fm_state = meta.get('frontier_memory')
        if not fm_state:
            return False
        try:
            state = {
                'unreachable_goals': [tuple(x) for x in fm_state['unreachable_goals']],
                'visited_frontiers': {tuple(int(v) for v in k.split(',')): v
                                      for k, v in fm_state['visited_frontiers'].items()},
                'visited_count': {tuple(int(v) for v in k.split(',')): v
                                  for k, v in fm_state['visited_count'].items()},
                'failure_reasons': {tuple(int(v) for v in k.split(',')): v
                                    for k, v in fm_state.get('failure_reasons', {}).items()},
            }
            frontier_manager.set_state(state)
            print(f"[Session] Loaded frontier memory: "
                  f"{len(state['unreachable_goals'])} unreachable, "
                  f"{len(state['visited_frontiers'])} visited")
            return True
        except Exception as e:
            print(f"[Session] Failed to load frontier memory: {e}")
            return False

    def save_params_summary(self, params_dict):
        """Save a summary of run parameters for reproducibility."""
        self._update_meta('params_summary', params_dict)

    def record_run(self, frames, coverage):
        """Record run statistics."""
        meta = self._load_meta()
        if meta.get('created_at') is None:
            meta['created_at'] = time.strftime('%Y-%m-%d %H:%M:%S')
        meta['updated_at'] = time.strftime('%Y-%m-%d %H:%M:%S')
        meta['run_count'] = meta.get('run_count', 0) + 1
        meta['total_frames'] = meta.get('total_frames', 0) + frames
        if coverage > meta.get('best_coverage', 0):
            meta['best_coverage'] = coverage
        self._save_meta(meta)

    def _update_meta(self, key, value):
        meta = self._load_meta()
        meta[key] = value
        self._save_meta(meta)

    def _load_meta(self):
        if os.path.exists(self.meta_file):
            try:
                with open(self.meta_file, 'r') as f:
                    return json.load(f)
            except Exception:
                pass
        return dict(self.metadata)

    def _save_meta(self, meta):
        try:
            with open(self.meta_file, 'w') as f:
                json.dump(meta, f, indent=2)
        except Exception as e:
            print(f"[Session] Failed to save metadata: {e}")
