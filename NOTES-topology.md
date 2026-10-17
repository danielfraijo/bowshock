# Topology / geometry-quality patch set (branch `feat/topology`, base `b5fdccc`)

Worked in `/workspace/bowshock-jobi`, nothing pushed. Apply with `git am *.patch` on top of `b5fdccc`.

## 1. Per-file changes
| file | change |
|---|---|
| `src/lib/waverider/geom-quality.ts` (new) | Metrics (`topoMetrics`: open / non-manifold / inconsistent-winding edges, duplicate verts, degenerate tris, min angle, slivers, body-only slivers and edge ratio, signed volume, G1 normal jumps split into same-surface / LE seam / nose, tips excluded), validators (`checkBinaryStl`, `checkIges`, `checkPlot3d`, `parseStep`/`checkStep`) and geometry read-back (`parseIgesSurfaces`, `parseStepSurfaces`, `maxDistanceToMesh`, `stepBrepVolume`). |
| `src/lib/waverider/topology.ts` (new) | `orientConsistent` (BFS winding + per-component volume sign), `tidyTrailingEdge` (grid-level TE tidy), `collapseTipRows`, `removeDoubleFaces`, `minAngleOf`, `flipForQuality` (unwired). |
| `generate.ts` | Replaces `matchTrailingEdge`/`fairTrailingEdge`/`weldAft` distance-welding (that pinched the TE and left non-manifold/unequal edges) with `tidyTrailingEdge`; rounded planform vertex + nose-station clustering (`spanStations`, `rhoPlan`); cell-shape-aware row coarsening in `lofts`; `collapseTipRows`; final `removeDoubleFaces -> orientOutward -> orientConsistent`; passes `leBlend` to the fillet; **elevon deflection is now applied before the fillet is built** (it used to be applied after, which tore the LE fillet / tip caps off the body, 8–16 open edges at `elevonDeg=10`). |
| `blunt.ts` | New LE fillet (rolling ball in the plane normal to the LE, smooth tangent-length law, Lipschitz-limited along the LE, facet angle ≤10° so the strip is G1-smooth, unresolved stations collapse to the seam). Optional quintic-Bezier G2 blend with zero end curvature (`leBlendProfile` exported for tests). Old `resample` replaced by `resampleFrac`/`arcFractions` (keeps the first cell behind the fillet the same size as its neighbours). `filletTipCap`, `bluntStarNose`, `applyCowlLip` untouched/unreviewed. |
| `math.ts` | `chordFractions` rewritten as a continuous cosine<->uniform blend (same signature). The old regime switch twisted cells between neighbouring rows (main sliver source). |
| `export-nurbs.ts` (new) | Shared global cubic interpolation (chord-length parameters, averaged knots, <=200 samples/direction). |
| `export-iges.ts` | Rewritten: correct 80-col S/G/D/P/T records, unit flag (m/mm/in), pole order (i fastest), interpolating surfaces, no 48-sample cap. `gridsToIges(grids, name, unit = "m")` — new **optional** 3rd arg. |
| `export-step.ts` | NURBS path (`gridsToNurbsStep`) rewritten: interpolating surfaces, geometric edge matching (endpoints + dense polyline distance), consistent loop orientation, outward volume sign, measured uncertainty. `unitBlock(doc, unit, uncertainty = 1e-8)` optional param. Faceted/tessellated paths untouched. |
| `export-stl.ts`, `export-plot3d.ts` | Unchanged (validated: ok for all families). |
| `scripts/topology-report.ts`, `scripts/topology-evidence.ts` (new) | Before/after table and SVG evidence. |
| `topology.test.ts` (new) | 1007 checks, see section 3. |

## 2. Edits to shared / other-owner files (please review)
- `src/lib/waverider/types.ts`: +2 lines, optional `leBlend?: "circular" | "g2"` on `DesignParams`. `DEFAULT_PARAMS` unchanged (circular stays default, per README contract; G2 is a toggle).
- `src/lib/waverider/fidelity.test.ts` (Dina's): `maxAspect(mesh, skipFillet=false)`; aspect checks skip fillet strips; `lead === false` became `lead === true` (the README circular-fillet contract: the default now really has a leading-edge grid); added `maxAspect(caret.mesh) < 500`. Passes.
- Follow-ups for others (not done by me): `export-kit.ts` should pass `params.unit` to `gridsToIges`; export-kit feeds `refineGrids(grids,3,2)` grids to the NURBS/IGES/Plot3D writers (works, see limitations on STEP volume).
- Not modified: `mesh.ts`, `export-kit.ts`, `export-stl.ts`, `export-plot3d.ts`, `package.json`, native, datcom.

## 3. Tests (`bun src/lib/waverider/topology.test.ts`, ~13 s, 1007 checks)
1. All 13 families x 6 cases (default, M5, M12 fine, no fillet, big R wide, coarse): non-manifold, winding, duplicate vertices, degenerates, open edges (ducts must be open), volume>0.
1b. 6 families x 17 parameter variants (half model, lid, dihedral, camber, elevon, fins, planforms, TE sweep, tiny/large R, tiny/big grids, M3/M20): watertight, consistent, outward.
2. Default-grid slivers (body min angle >0.5°, <6% slivers, body edge ratio <80) + caret fine.
3. Fillet families have a leading grid; LE seam jump <20°, nose jump <50°; caret fine LE jump <15°.
4. G2 blend profile (κ=0 at ends, κ=1/R at apex, no inflection, continuous); G2 caret watertight, seam jump <3°.
5. Helper unit tests (`orientConsistent`, `removeDoubleFaces`, interpolation corner).
6. Exports for all families: STL, Plot3D, IGES (+ surface deviation <1% L) valid; STEP valid + B-rep volume within 3% of the mesh for the ten solid families.

Against the base commit (new modules copied in, non-throwing variant) the section 1–6 checks of the first version of the test gave **162 of 683 failing** (`base-failures.txt`). Elevon variant failure on base was verified separately for wedgecone only (open 20 / inconsistent 3); caret/osculating elevon regression was introduced and fixed within this branch.

## 4. Verification (baseline vs final)
| command | baseline (b5fdccc) | final |
|---|---|---|
| `npx tsc --noEmit` | ok | ok |
| `bun src/lib/waverider/fidelity.test.ts` | pass | pass |
| `bun src/lib/waverider/topology.test.ts` | n/a (new) | pass, 1007 checks |
| `node --test scripts/*.test.mjs` | 195 tests, 182 pass, 13 fail | 195 tests, 182 pass, 13 fail (identical, unrelated to this work) |
| `bun scripts/topology-report.ts [--json f]` | `baseline.txt/json` | `after.txt/json` |
Notes: `npm test` as written is unrunnable here (glob quoting; node 20.19 has no strip-types) — use `bun <ts test>` and `node --test scripts/*.test.mjs`. 4 app-data/auth TS tests do not run under bun (pre-existing).

## 5. Before / after (78 cases = 13 families x 6; ducts excluded from totals because they are open by design)
Totals over the 66 closed-family cases: open edges 10 -> 0; non-manifold edges 34 -> 0; inconsistent-winding edges 1800 -> 0; degenerate tris 4 -> 0; cases with any defect 18 -> 0; volume<=0 cases 0 -> 0.

| family (M8 default) | open/nonM/incon base | after | slivers base (all tris) | slivers after (body / all incl. fillet) | LE seam jump base→after (°) | nose jump base→after (°) | vol base→after |
|---|---|---|---|---|---|---|---|
| caret | 0/0/0 | 0/0/0 | 1810 | 76 / 680 | 0.0→10.6 | 145→31 | 1.083→1.075 |
| cone | 0/0/0 | 0/0/0 | 590 | 24 / 938 | 0.0→7.2 | 167→22 | 1.564→1.557 |
| osculating | 0/0/0 | 0/0/0 | 1552 | 96 / 798 | 0.0→10.6 | 153→29 | 1.114→1.105 |
| viscopt | 0/0/0 | 0/0/0 | 1588 | 126 / 954 | 0.0→10.4 | 166→25 | 1.036→1.028 |
| elliptic | 0/0/0 | 0/0/0 | 1578 | 96 / 798 | 0.0→10.3 | 153→29 | 1.093→1.084 |
| wedgecone | 0/2/3 | 0/0/0 | 1578 | 96 / 798 | 161→11.1 | 138→29 | 1.176→1.168 |
| star | 0/0/258 | 0/0/0 | 104 | 104 / 104 | – | 145→145 | 1.499→1.499 |
| liftbody | 0/0/0 | 0/0/0 | 172 | 172 / 172 | – | 21→21 | 1.613→1.613 |
| ramjet (duct) | 132/0/28 | 132/0/0 | 224 | 0 / 224 | – | 162→162 | 0.235→0.235 |
| scramjet (duct) | 132/0/28 | 132/0/0 | 224 | 0 / 224 | – | 162→162 | 0.443→0.444 |
| busemann | 0/0/0 | 0/0/0 | 280 | 32 / 686 | 0.0→16.2 | 134→19 | 1.905→1.897 |
| inward | 0/0/0 | 0/0/0 | 1678 | 110 / 794 | 0.0→10.9 | 158→29 | 1.113→1.103 |
| integrated | 0/0/0 | 0/0/0 | 1536 | 90 / 792 | 0.0→15.3 | 153→29 | 1.311→1.306 |

Reading it: the base default had **no real leading-edge fillet** (LE jump 0.0 = no fillet grid; the "nose jump" of 130–167° was a hard nose ridge). Now every fillet family has a faceted-at-≤10° round LE and a ~20–30° nose jump. "Slivers" in the baseline counted all triangles, the new `bodySlivers` counts non-fillet triangles; fillet strips are intentionally thin along the LE (anisotropic), so the "all" figure is not an improvement metric. Wider cases (M12 fine: caret slivers 7108 → 424 body; edge ratio 164) are in `after.txt`. Column sets differ between the two txt files (`minAng/sliv/aspect` vs `bMinAng/bSliv/bEdgeR`); use the json for like-for-like fields.
Exports (final, via the test): STL ok all 13; Plot3D ok all 13; IGES ok all 13 (surface deviation ≤1.7e-3 m on a ~3.8 m body); STEP closed-solid valid for caret, cone, osculating, viscopt, elliptic, wedgecone, liftbody, busemann, inward, integrated. Baseline IGES had formatting bugs (S/D padding, unit flag, pole order, 48-sample cap).
Evidence: `evidence/le-section-{before,after}.svg`, `evidence/nose-plan-{before,after}.svg` (caret M12, nx80 ny56, R=0.01). I verified the SVGs were generated and have the expected element counts (after: much denser nose grid) but did not visually review rendered images (no SVG rasteriser on the box) — open them in a browser.

## 6. Known limitations
- Behaviour changes: default fillet is now actually tessellated (LE now a ≤10° faceted arc); volume shifts ~0.7% (caret 1.083 -> 1.075); triangle counts change (caret 4190 -> 3198); planform vertex rounding slightly shortens the nose.
- Caret keel crease is inherent to the family; star spokes and duct corners are sharp by design (counted in `sharpEdges`); wing tips are knife edges (excluded from G1 stats).
- Coarse grids (ny ≤ ~18) drop or terminate the fillet with a step. Fillet strips are anisotropic (aspect up to ~350 on the strip, excluded from the aspect test). Nose jump ~27–31° (not G1); liftbody nose jump ~21° unchanged.
- G2 mode: seam jump ~0.8°, curvature continuous, but the apex radius is far tighter than the requested R for slender wedges, nose jump ~100–110° in G2 mode.
- Star (8 fin sheets) and duct STEP/IGES are open sheets, not closed solids. STEP B-rep volume of refined grids deviates ~0.1–4% from the mesh (quadrature dependent; test tolerance 3% with n=48).
- `bluntStarNose`, `applyCowlLip` not reviewed; `flipForQuality` unwired; no `flowThrough=false` duct cases in the report; `semiAxis` unused param is pre-existing lint.

## 7. Recommended next steps
1. Dina: pass `params.unit` to `gridsToIges`; consider feeding unrefined grids to STEP.
2. Mesh-refinement owner: re-baseline refinement tests against the new fillet/row coarsening; consider using `topoMetrics` in CI.
3. Review nose G1 (replace the density-bump clustering with a true spherical blend) and G2 apex radius law.
4. Wire `leBlend` into the UI toggle.
5. Fix `npm test` script quoting / runner.

## Roman's aft-tip non-manifold report (checked 2026-10-16)
Base `weldAft` (8e-4*L, mesh.ts) left non-manifold aft-tip edges (caret ny>=34, wedgecone ny 28/32, inward at 72x52). On `feat/topology` the grid-level TE tidy (`tidyTrailingEdge`, topology.ts) replaces distance welding, so mesh.ts is untouched. Sweep: caret/wedgecone/inward/osculating x nx{40,72} x ny 20..56 step 2 = 152 cases, 0 open, 0 non-manifold, 0 inconsistent-winding edges.
