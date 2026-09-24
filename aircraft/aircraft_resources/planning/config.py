"""Planner configuration, loaded from YAML (config/planner.yaml).

Access is by section/key -- `cfg.get("rrt_star", "step")` -- so a new
hyperparameter only needs a line in the YAML. `override` returns a modified
copy, which is what the RSM/DOE runner uses to sweep factors.
"""

from __future__ import annotations

import copy
from pathlib import Path

import yaml

DEFAULT_PATH = Path(__file__).resolve().parent / "config" / "planner.yaml"


class Config:
    def __init__(self, data: dict):
        self.data = data

    @classmethod
    def load(cls, path: str | Path | None = None) -> "Config":
        return cls(yaml.safe_load(Path(path or DEFAULT_PATH).read_text()))

    def section(self, name: str) -> dict:
        return self.data[name]

    def get(self, *keys, default=None):
        node = self.data
        for k in keys:
            if not isinstance(node, dict) or k not in node:
                if default is not None:
                    return default
                raise KeyError("/".join(map(str, keys)))
            node = node[k]
        return node

    def override(self, **dotted) -> "Config":
        """cfg.override(**{"rrt_star.step": 1.5, "astar.resolution": 0.5})"""
        new = copy.deepcopy(self.data)
        for key, value in dotted.items():
            *path, leaf = key.split(".")
            node = new
            for k in path:
                node = node[k]
            if leaf not in node:
                raise KeyError(key)
            node[leaf] = value
        return Config(new)
