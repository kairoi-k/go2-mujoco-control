#!/usr/bin/env python3

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import quarantine_raw_run
from quarantine_raw_run import GuardError, run_guard, validate_source


class QuarantineRawRunTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.workspace = self.root / "workspace"
        self.repo = self.workspace / "current"
        self.runs = self.repo / "example/cpp/experiments/_runs"
        self.quarantine = self.workspace / "archive/quarantine/raw-runs"
        self.runs.mkdir(parents=True)
        (self.workspace / "archive").mkdir()
        self.run = self.runs / "smoke"
        self.run.mkdir()
        (self.run / "data.csv").write_text("t,x\n0,1\n", encoding="utf-8")
        (self.run / "controller.log").write_text("done\n", encoding="utf-8")
        (self.run / "contact_ground_truth.csv").write_text(
            "t,f\n0,1\n", encoding="utf-8"
        )
        (self.run / "simulator.log").write_text("done\n", encoding="utf-8")
        (self.run / "run_manifest.json").write_text("{}\n", encoding="utf-8")
        self._git("init", "-b", "main")

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _git(self, *args: str) -> None:
        import subprocess

        subprocess.run(
            ["git", *args], cwd=self.repo, check=True, capture_output=True
        )

    def test_rejects_runs_root_and_nested_target(self) -> None:
        with self.assertRaises(GuardError):
            validate_source(self.runs, self.runs)
        nested = self.run / "nested"
        nested.mkdir()
        with self.assertRaises(GuardError):
            validate_source(nested, self.runs)

    def test_rejects_source_symlink(self) -> None:
        link = self.runs / "linked"
        link.symlink_to(self.run, target_is_directory=True)
        with self.assertRaises(GuardError):
            validate_source(link, self.runs)

    def test_rejects_noncanonical_runs_root(self) -> None:
        other_runs = self.repo / "other-runs"
        other_run = other_runs / "smoke"
        other_run.mkdir(parents=True)
        with self.assertRaises(GuardError):
            run_guard(
                other_run,
                other_runs,
                self.quarantine,
                self.repo,
                False,
                "20260904T010203Z",
            )

    def test_dry_run_hashes_without_moving(self) -> None:
        result = run_guard(
            self.run,
            self.runs,
            self.quarantine,
            self.repo,
            False,
            "20260904T010203Z",
        )
        self.assertEqual(result["state"], "dry-run")
        self.assertEqual(result["file_count"], 5)
        self.assertTrue(self.run.exists())
        self.assertFalse(self.quarantine.exists())

    def test_apply_moves_atomically_and_keeps_manifest(self) -> None:
        result = run_guard(
            self.run,
            self.runs,
            self.quarantine,
            self.repo,
            True,
            "20260904T010203Z",
            self.root / "experiment.lock",
        )
        destination = Path(str(result["destination"]))
        manifest = Path(str(result["manifest"]))
        self.assertFalse(self.run.exists())
        self.assertTrue((destination / "data.csv").is_file())
        self.assertEqual((destination / "data.csv").stat().st_mode & 0o222, 0)
        self.assertTrue(manifest.is_file())
        saved = json.loads(manifest.read_text(encoding="utf-8"))
        self.assertEqual(saved["state"], "quarantined")
        self.assertEqual(saved["files"], result["files"])

    def test_refuses_destination_collision(self) -> None:
        destination = self.quarantine / "20260904T010203Z__smoke"
        destination.mkdir(parents=True)
        with self.assertRaises(GuardError):
            run_guard(
                self.run,
                self.runs,
                self.quarantine,
                self.repo,
                True,
                "20260904T010203Z",
                self.root / "experiment.lock",
            )
        self.assertTrue(self.run.exists())

    def test_refuses_incomplete_run_on_apply(self) -> None:
        (self.run / "run_manifest.json").unlink()
        with self.assertRaises(GuardError):
            run_guard(
                self.run,
                self.runs,
                self.quarantine,
                self.repo,
                True,
                "20260904T010203Z",
                self.root / "experiment.lock",
            )
        self.assertTrue(self.run.exists())

    def test_refuses_tracked_reference_on_apply(self) -> None:
        note = self.repo / "note.md"
        note.write_text("smoke must remain\n", encoding="utf-8")
        self._git("add", "note.md")
        with self.assertRaises(GuardError):
            run_guard(
                self.run,
                self.runs,
                self.quarantine,
                self.repo,
                True,
                "20260904T010203Z",
                self.root / "experiment.lock",
            )
        self.assertTrue(self.run.exists())

    def test_refuses_unsafe_stamp(self) -> None:
        with self.assertRaises(GuardError):
            run_guard(
                self.run,
                self.runs,
                self.quarantine,
                self.repo,
                False,
                "../escape",
            )

    def test_refuses_quarantine_symlink(self) -> None:
        outside = self.root / "outside"
        outside.mkdir()
        self.quarantine.parent.mkdir(parents=True)
        self.quarantine.symlink_to(outside, target_is_directory=True)
        with self.assertRaises(GuardError):
            run_guard(
                self.run,
                self.runs,
                self.quarantine,
                self.repo,
                False,
                "20260904T010203Z",
            )

    def test_refuses_same_size_change_after_hash(self) -> None:
        original_inventory = quarantine_raw_run.inventory
        calls = 0

        def mutate_after_first(source: Path):
            nonlocal calls
            result = original_inventory(source)
            calls += 1
            if calls == 1:
                (source / "data.csv").write_text("t,x\n0,2\n", encoding="utf-8")
            return result

        with mock.patch.object(
            quarantine_raw_run, "inventory", side_effect=mutate_after_first
        ):
            with self.assertRaises(GuardError):
                run_guard(
                    self.run,
                    self.runs,
                    self.quarantine,
                    self.repo,
                    True,
                    "20260904T010203Z",
                    self.root / "experiment.lock",
                )
        self.assertTrue(self.run.exists())


if __name__ == "__main__":
    unittest.main()
