# Changelog

## Unreleased — research companion candidate

- README and research index describe this tree as MuJoCo stand-walk-lie + slow trot, not as a Kine2Go/AMP/seam paper repo;
- removed the leftover Genesis `evaluation/quant_eval.py` (it pointed at a local `/tmp` pipeline);
- dropped tracked `controller.log` files (gitignore already excludes `*.log`);
- experiment READMEs no longer claim those logs are in git;
- Isaac Lab velocity-curriculum configs live in `rl/isaaclab_custom/` as a second track;
- seam JSON / AMP evidence stay in `kairoi-k/kine2go-research`.

No controller algorithm is changed by this curation.

## Development milestone `73ac543`

C++ sequencing and trot stack as recorded in `docs/RESEARCH_INDEX.md`. Isaac Lab velocity RL and Kine2Go imitation are outside this snapshot.
