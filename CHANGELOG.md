# Changelog

## 2026-08-15

- Checkout vendors Go2 MJCF only; unused Unitree robot meshes are not in this tree.
- `ctest` registers the CPU unit tests (FK still needs the local MuJoCo library).
- `example/cpp/MODULES.md` points at `docs/CODE_GUIDE.md`.
- `--wbc-full` centroidal wrench (`W = M a + h`); N-step foothold preview now selects the next Raibert touchdown; stance PD is lowered on that path.

## 2026-08-14

- README clips for stand-walk-lie and Isaac Lab 0.5 / 3.5 m/s.
- Isaac Lab track is a gym-registered package (`pip install -e rl`).
- `model_54950` published on GitHub Release v0.1.0.

## 2026-08-13

C++ stand-walk-lie and slow trot as recorded in `docs/RESEARCH_INDEX.md`.
