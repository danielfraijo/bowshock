/** Per-stage timing of the data pack (Node). node --import ./scripts/ts-register.mjs scripts/datapack-stages.ts [method] */
import { computeDataPackAsync } from "../src/lib/waverider/datapack";
import { DEFAULT_PARAMS, type DesignParams } from "../src/lib/waverider/types";

const method = process.argv[2] ?? "euler";
async function run(tag: string, params: DesignParams) {
  const stage = new Map<string, number>();
  let last = performance.now();
  let lastLabel = "start";
  const t0 = last;
  await computeDataPackAsync(params, undefined, {
    onProgress: (p) => {
      const now = performance.now();
      const k = lastLabel.replace(/[0-9.]+/g, "#").replace(/\(α[+−]#°\)/, "(α±)");
      stage.set(k, (stage.get(k) ?? 0) + (now - last));
      last = now;
      lastLabel = p.label;
    },
  });
  const now = performance.now();
  const k = lastLabel;
  stage.set(k, (stage.get(k) ?? 0) + (now - last));
  console.log(`${tag}: ${(now - t0).toFixed(0)} ms`);
  for (const [a, b] of [...stage].sort((x, y) => y[1] - x[1]).slice(0, 8)) console.log(`   ${b.toFixed(0).padStart(6)} ms  ${a}`);
}
const base = { ...DEFAULT_PARAMS, aeroMethod: method } as unknown as DesignParams;
await run("cold", base);
await run("alpha edit", { ...base, alphaDeg: 4 });
await run("alpha edit 2", { ...base, alphaDeg: 5 });
await run("shockDeg edit", { ...base, shockDeg: 17 });
