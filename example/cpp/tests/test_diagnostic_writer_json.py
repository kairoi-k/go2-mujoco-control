#!/usr/bin/env python3
import json
import math
import pathlib
import subprocess
import sys

if len(sys.argv) != 3:
    raise SystemExit("usage: test_diagnostic_writer_json.py FIXTURE OUTPUT")
subprocess.run([sys.argv[1], sys.argv[2]], check=True)
lines = pathlib.Path(sys.argv[2]).read_text(encoding="utf-8").splitlines()
assert len(lines) == 3, lines
records = [json.loads(line) for line in lines]
assert [record["record_type"] for record in records] == ["capture", "map", "state"]

capture = records[0]["capture"]
assert capture["payload_repr"] == "absent"
assert capture["serialized_payload"] is None
assert capture["complete_decoded_value"] is None
assert capture["source_message_id"] is None
assert capture["source_stamp"] is None and not capture["source_stamp_valid"]

mapped = records[1]
assert mapped["frame_id"] == 'frame"\\line\n\x01'
assert mapped["width"] == 3 and mapped["height"] == 2
assert len(mapped["cells"]) == 6
assert [(cell["ix"], cell["iy"]) for cell in mapped["cells"]] == [(0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (2, 1)]
assert mapped["cells"][-1]["value_m"] is None
assert mapped["cells"][-1]["cell_stamp"] is None
assert mapped["raw_rays"] is None and not mapped["raw_ray_available"]
assert mapped["capture"]["payload_repr"] == "absent"

state = records[2]
assert state["frame_id"] is None and not state["frame_valid"]
assert state["position"] == [1.0, 2.0, 3.0]
assert state["quaternion"] == [1.0, 0.0, 0.0, 0.0]
assert state["pose_transform"] == [None] * 16
assert not state["valid"] and not state["validity"]["join"]
for line in lines:
    parsed = json.loads(line)
    assert parsed["record_type"] in {"capture", "map", "state"}
    assert "nan" not in line.lower() and "inf" not in line.lower()
