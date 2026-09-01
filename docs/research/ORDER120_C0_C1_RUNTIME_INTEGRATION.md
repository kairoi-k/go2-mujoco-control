# Order120: C0/C1 development bootstrap runtime integration

Status: DEVELOPMENT-ONLY. No B1 acceptance or simulator traversal claim.

This checkpoint wires the previously static bootstrap contract into the Stage-C runtime behind `TROT_TERRAIN_BOOTSTRAP_DEV=1`. Flag-off behavior is unchanged.

The pre-transfer planner is allowed to build Stage-C V2-B timing while Phase 1 still owns locomotion. A generic C0 certificate evaluates the current-foot swept corridor through the production velocity shaper's discrete stopping distance. It uses the existing terrain-feasibility thresholds, map age/frame checks, and the commanded travel direction; it contains no obstacle coordinate, fixed leg order, or scene step height.

Pre-transfer arbitration is deliberately small: no C1 candidate plus valid C0 leaves the existing Phase-1 shaper free to observe; loss of C0 caps the existing shaper at zero; a sensor-derived transfer ROI plus a complete usable V2-B plan also caps at zero to form the transfer hold. The worker never writes velocity directly.

The transfer window can arm only after the measured body speed has settled to <=0.05 m/s, C0 is valid, the exact candidate plan id is still the usable store snapshot, and the execution adapter reports a legal boundary. That exact immutable shared_ptr is frozen for first C1 adoption so an asynchronous planner refresh cannot switch identity between hold and handoff. After adoption, the existing C-004 path remains authoritative: gait and SRBD/WBC consume the adapter's same adopted immutable snapshot.

Follow-up review caught two static defects before runtime authorization: the valid stop-distance accumulator inherited its invalid infinity sentinel, and the Release CMake build compiled away the C0 test's assertions. The accumulator now starts from zero after input validation, the C0 test explicitly re-enables assertions in Release, and the planner worker uses the already-copied nominal-foot snapshot instead of reading TrotTask-owned storage.

This is still only a development bootstrap gate. End-to-end sensor/queue/actuation/halt latency certification and a 5 cm dynamic probe remain separate obligations. The next gate is remote build/CTest; if clean, no further generic observer/infrastructure work is authorized before the bounded 5 cm development probe.
