# Cloud review packet

This directory is the read-only review surface for the exact candidate branch
that was pushed as `fix/phase2-b0-runtime-integrity`. It is a compact,
portable subset of the latest F14 raw evidence; it is not an acceptance claim.

Read in this order:

1. [CURRENT.md](../../../../../CURRENT.md) - active route, status, plan, and invariants.
2. [F14_LOCKSTEP_20260905.md](../F14_LOCKSTEP_20260905.md) - exact canary observations.
3. [DECISIONS.md](../DECISIONS.md) - hypotheses, rejected probes, and next decision.
4. [PACKET_MANIFEST.json](PACKET_MANIFEST.json) - source identity and artifact hashes.
5. The two run directories below - exact manifests, lockstep traces, analyzer output,
   controller/simulator logs, and compact analysis text.

The packet is deliberately not a replacement for the local raw archive. The
large `data.csv` and `contact_ground_truth.csv` files remain local and are
listed with SHA-256 in the packet manifest; they are too large and too easy to
mistake for a promoted result in the main repository. The included lockstep
traces and analyzer outputs are the reviewable raw subset.

Interpretation rules:

- Source under review is exactly commit `6cdf2366bb38aab9db29184efc813be71ae3022f`,
  with a clean detached worktree at run time.
- Baseline and terrain canaries exchanged continuous 2 ms traces with zero
  fail-closed events; both standalone analyzers returned PASS.
- The paired command/trajectory diagnostics were still false. This packet is
  therefore F14 diagnostic evidence, not B0 acceptance and not B1 authorization.
- Do not revive quasi-static/scripted crawl, three-contact entry, stop-to-arm,
  cap-to-zero transfer, consumer-local timing, local swing retiming, V2/V2-B,
  or terrain actuation from this packet.

For the external model, the useful question is: identify the first causal
divergence that leaves lockstep mechanically healthy but makes the paired gait,
acceleration, and WBC target series disagree, then propose the smallest
evidence-producing repair consistent with `CURRENT.md`.

