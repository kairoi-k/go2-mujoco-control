#!/usr/bin/env python3
"""Versioned binding for the registered snapshot/query probe."""
import importlib.util
from pathlib import Path
PACKET=Path(__file__).resolve().parent
REPO=PACKET.parents[3]
if __name__=='__main__':
 path=REPO/'docs/research/evidence/joint_runtime_shadow_20260907/reproduce.py'
 spec=importlib.util.spec_from_file_location('joint_shadow_replay',path)
 module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 module.PACKET=PACKET;module.__file__=__file__;module.main()
