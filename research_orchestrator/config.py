"""Environment-backed settings for Base and Atlas workers."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class WorkerSettings:
    temporal_address: str = "localhost:7233"
    temporal_namespace: str = "default"
    agent_task_queue: str = "agent"
    atlas_task_queue: str = "atlas"
    repo_root: Path = Path.cwd()

    @classmethod
    def from_env(cls) -> "WorkerSettings":
        repo_root = Path(os.environ.get("RESEARCH_REPO_ROOT", Path.cwd())).expanduser().resolve()
        return cls(
            temporal_address=os.environ.get("TEMPORAL_ADDRESS", "localhost:7233"),
            temporal_namespace=os.environ.get("TEMPORAL_NAMESPACE", "default"),
            agent_task_queue=os.environ.get("RESEARCH_AGENT_TASK_QUEUE", "agent"),
            atlas_task_queue=os.environ.get("RESEARCH_ATLAS_TASK_QUEUE", "atlas"),
            repo_root=repo_root,
        )
