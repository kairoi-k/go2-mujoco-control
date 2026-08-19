# Provenance note

The 49 simulator runs were executed from the working tree recorded in each run manifest as `git_head=ed63e23ba9e9eba233313838b7296eaa4076d96a`. The controller source, scene, and binary were then committed without controller changes as commit `5135cca` (`Validate unified reactive transition matrix`) on `feature/environment-adaptation`.

The post-run edits only normalized the generated CSV line endings, clarified the protocol/report wording, and added ignore rules and delivery documentation. The committed controller and experiment code therefore reproduce the tested controller configuration; the per-run binary and scene SHA-256 values remain in the manifests.
