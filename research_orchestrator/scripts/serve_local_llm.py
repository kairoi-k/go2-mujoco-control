"""Start the pinned Atlas local model for a manual benchmark, then cleanly stop it."""

from __future__ import annotations

import asyncio
import os
from pathlib import Path

from ..activities.local_llm import _start_server, _stop_server, settings_from_env


async def main() -> None:
    settings = settings_from_env()
    root = Path(os.environ.get("ATLAS_ARTIFACT_ROOT", ".")).expanduser().resolve()
    log_path = root / "manual-local-llm" / "server.log"
    running = await _start_server(settings, log_path)
    print(
        f"local LLM ready model={settings.model_id} runtime={settings.runtime_version} "
        f"url={settings.base_url} pid={running.process.pid}",
        flush=True,
    )
    try:
        while True:
            await asyncio.sleep(5.0)
    finally:
        await _stop_server(settings, running)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
