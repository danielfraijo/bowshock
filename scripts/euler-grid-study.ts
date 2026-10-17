/** 2-D Euler anchor grid study (slow: ~25 s). node --import ./scripts/ts-register.mjs scripts/euler-grid-study.ts */
import { eulerWedgeGrid } from "../src/lib/waverider/euler2d";
for (const [M, th] of [[2, 10], [5, 15]] as [number, number][]) {
  const errs: number[] = [];
  for (const [ni, nj, it] of [[36, 12, 300], [72, 24, 600], [144, 48, 1200], [288, 96, 2400]]) {
    const t = performance.now();
    const r = eulerWedgeGrid(M, th, 1.4, ni, nj, it);
    errs.push(Math.abs(r.relErr));
    console.log(`M${M} θ${th}° ${ni}×${nj} (${it} it): p2/p1 ${r.p2p1.toFixed(4)} exact ${r.exactP.toFixed(4)} err ${(r.relErr * 100).toFixed(2)} %  residual ${r.residual.toExponential(1)}  ${(performance.now() - t).toFixed(0)} ms`);
  }
  console.log(`  observed order (err ratios per halving): ${errs.slice(1).map((e, i) => Math.log2(errs[i] / e).toFixed(2)).join(", ")}`);
}
