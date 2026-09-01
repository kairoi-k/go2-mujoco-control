# Order117 standalone diagnostic observer

`go2_diagnostic_observer` is a read-only DDS subscriber for the five existing topics: `rt/lowstate`, `rt/lowcmd`, `rt/sportmodestate`, `rt/go2/lidar_heightmap`, and `rt/go2/environment_heightmap`. It does not include production controller headers, link MuJoCo, or publish control messages.

The callback API provides a typed `const void *` only; it provides neither raw serialization nor `SampleInfo`. Every `CaptureRecord` therefore uses `payload_repr: absent`, with empty payload fields and invalid source/provenance. LowState is counted but not decoded. LowCmd uses a typed subscription whose callback only increments `lowcmd_count`; it never reads or tests command values and does not retain a record. The observer remains `interface_partial`; `runtime_probe_authorized=false`. HeightMap retains a typed deep copy of width, height, resolution, origin, and every cell; `complete_value` is true only when the dimensions match. SportModeState retains only its timestamp, position, and quaternion. It has no frame field, so frame and joins remain invalid; no transform is synthesized.

The writer emits one valid JSON object per line. It serializes every Capture/Map/State field, validity flags, null values for unavailable/non-finite values, absent payloads, all cells, position, quaternion, and transforms. Strings use JSON escaping and non-finite numbers are emitted as `null`. Records are append-only immutable copies. No plan, acknowledgement, lease, ROI, or actuation path is present.

The writer fixture and Python `json.loads` test exercise schema fields, all cells, null/absent values, finite-number handling, and quote/backslash/control-character escaping.
