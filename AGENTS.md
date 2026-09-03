# Go2 controller agent rules

This file supplements the workspace-level rules and applies to this repository.

## Raw run evidence

- Everything below `example/cpp/experiments/_runs/` is raw evidence by default. An invalid, dirty, failed, smoke, stale, unreferenced, or semantically reproducible run is still not disposable.
- Agents must never target `_runs/` with permanent deletion, overwrite, or rename operations, including `rm`, `rmdir`, `find -delete`, and `git clean`.
- Space cleanup may only generate a per-file SHA-256 manifest and atomically move one direct child through `example/cpp/tools/quarantine_raw_run.py` into the workspace archive quarantine outside this Git worktree. Quarantine is not deletion and must not be targeted by `git clean` or filesystem deletion.
- A subagent's cleanup classification is evidence for review, never authorization. The primary agent must independently verify the target and manifest.
- Permanent purge is outside the tool. It requires a new user message naming the exact quarantined path plus an independently verified copy whose SHA-256 manifest matches. Earlier broad authority does not satisfy this gate.
- If any rule conflicts with an older experiment note, apply this stricter retention rule.
