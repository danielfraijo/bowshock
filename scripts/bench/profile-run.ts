/** CPU-profile helper: tsx --cpu-prof scripts/bench/profile-run.ts <workload>   (summarise with scripts/bench/profile-sum.mjs) */
import { buildVehicle } from "../../src/lib/waverider/generate";
import { studyVehicle } from "../../src/lib/waverider/study";
import { computeDataPack } from "../../src/lib/waverider/datapack";
import { runCases } from "../../src/lib/waverider/datcom/model";
import { PRESETS } from "../../src/lib/waverider/datcom/presets";
import { paramsFor, slide } from "./suite";
import { presetSettings } from "../../src/lib/waverider/refinement";
const w = process.argv[2] ?? "build";
const t0 = performance.now();
if (w === "build") for (let i = 0; i < 30; i++) for (const f of ["caret", "osculating", "liftbody", "scramjet"] as const) buildVehicle(paramsFor(f, { span: 2 + (i % 11) * 0.01 }));
if (w === "build-fine") for (let i = 0; i < 12; i++) for (const f of ["caret", "osculating"] as const) buildVehicle(paramsFor(f, { refinement: presetSettings("fine"), span: 2 + (i % 11) * 0.01 }));
if (w === "study") for (let i = 0; i < 12; i++) for (const f of ["caret", "osculating", "liftbody"] as const) studyVehicle(buildVehicle(paramsFor(f, { span: 2 + (i % 11) * 0.01 })), {});
if (w === "datapack") for (let i = 0; i < 4; i++) computeDataPack(paramsFor("caret", { alphaDeg: slide(i, 0, 5) }));
if (w === "datcom") for (let i = 0; i < 20; i++) for (const p of PRESETS) runCases(p.config, p.flight);
console.log(w, (performance.now() - t0).toFixed(0), "ms");
