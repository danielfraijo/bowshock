/** Longest synchronous slice inside computeDataPackAsync (main-thread blocking proxy). */
import { clearDataPackCache, computeDataPackAsync } from "../src/lib/waverider/datapack";
import { DEFAULT_PARAMS } from "../src/lib/waverider/types";
for (const aeroMethod of ["euler", "mixed"] as const) {
  clearDataPackCache();
  let last = performance.now();
  let worst = 0;
  let worstLabel = "";
  let n = 0;
  const t0 = last;
  await computeDataPackAsync({ ...DEFAULT_PARAMS, aeroMethod }, undefined, {
    onProgress: (p) => {
      const now = performance.now();
      if (now - last > worst) {
        worst = now - last;
        worstLabel = p.label;
      }
      last = now;
      n++;
    },
  });
  console.log(`${aeroMethod}: total ${(performance.now() - t0).toFixed(0)} ms in ${n} slices; longest blocking slice ${worst.toFixed(0)} ms (before "${worstLabel}")`);
}
