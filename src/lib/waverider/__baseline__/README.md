# Baseline fixtures

* `baseline-b5fdccc.json` - original pre-refinement capture (Roman's base, before topology / physics-datapack). Historical, no longer read by the tests.
* `baseline-topology-physics-45e7c83.json` - re-capture on topology + physics-datapack (45e7c83). `refinement.test.ts` compares the **`legacy`** preset (and the wedge anchor with no `vol`) bitwise against it.

The mesh-hardening PR did **not** need a new fixture: `legacy` still reproduces this capture bit for bit (meshes, wedge anchor, Euler aero for caret/liftbody; osculating/cone to 1e-3 because the base's cone-shock cache and Euler cache key depend on process history, see `capture.ts`).
The default (`medium`) deliberately differs from the fixture now: it runs the strip Euler to convergence on the mirrored lower wall, so its Euler aero numbers (e.g. caret centreline lower-surface Cp -0.022 -> 0.109 at M8) are *not* compared to the fixture; accuracy is checked against exact oblique-shock / Rayleigh / Billig references in `refinement.test.ts` and `robustness.test.ts`.

To re-capture (only if the legacy path itself must change): `bun run src/lib/waverider/__baseline__/capture.ts <repoRoot> <out.json> [family|alpha]`, one process per vehicle case.
