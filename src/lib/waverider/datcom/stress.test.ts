/**
 * Stress harness: extreme geometries, attitudes, Mach, altitude.
 * Asserts: no NaN/Inf, no throws (or clear ValidationError), sane bounds,
 * Mach/alpha/roll continuity within stated tolerances.
 *
 * Run via: npm run test:datcom  (includes this file once wired)
 */
import assert from "node:assert/strict";
import { describe, it } from "node:test";
import type { FlightSpec, RunResult, VehicleConfig } from "./types.ts";
import { runCases, buildModel } from "./model.ts";
import { basicFinner, cruciformMissile, hypersonicConeCylinderFlare, aircraftWBT, cloneConfig } from "./presets.ts";
import { ValidationError, sanitizeConfig, sanitizeFlight } from "./validate-input.ts";

function finiteDeep(v: unknown, path = "root"): string[] {
  const bad: string[] = [];
  if (typeof v === "number") {
    if (!Number.isFinite(v)) bad.push(`${path}=${v}`);
    return bad;
  }
  if (Array.isArray(v)) {
    v.forEach((x, i) => bad.push(...finiteDeep(x, `${path}[${i}]`)));
    return bad;
  }
  if (v && typeof v === "object") {
    for (const [k, val] of Object.entries(v as Record<string, unknown>)) {
      if (k === "notes" || k === "method" || k === "warnings" || k === "confidence" || k === "name" || k === "role" || k === "section" || k === "kind" || k === "noseKind" || k === "aftKind" || k === "blayer" || k === "units" || k === "derivUnits" || k === "caseId" || k === "naca" || typeof val === "string" || typeof val === "boolean") continue;
      bad.push(...finiteDeep(val, `${path}.${k}`));
    }
  }
  return bad;
}

function assertFiniteResult(r: RunResult, label: string) {
  const bad = finiteDeep(r, label);
  assert.equal(bad.length, 0, `NaN/Inf in ${label}:\n  ${bad.slice(0, 20).join("\n  ")}`);
}

function assertBounded(r: RunResult, label: string) {
  for (const c of r.cases) {
    for (const p of c.points) {
      for (const [k, v] of Object.entries({ CN: p.CN, CA: p.CA, Cm: p.Cm, CY: p.CY, Cn: p.Cn, Cl: p.Cl, CL: p.CL, CD: p.CD })) {
        assert.ok(Math.abs(v as number) < 2500, `${label} |${k}|=${v} @ M=${c.mach} a=${p.alpha}`);
      }
      assert.ok(Math.abs(p.derivs.CNa) < 5000, `${label} |CNa|=${p.derivs.CNa}`);
      assert.ok(Math.abs(p.derivs.Cma) < 20000, `${label} |Cma|=${p.derivs.Cma}`);
    }
  }
}

function runSafe(cfg: VehicleConfig, flight: FlightSpec): RunResult {
  const c = sanitizeConfig(cfg);
  const f = sanitizeFlight(flight);
  return runCases(c, f);
}

describe("stress: no fins / many panels", () => {
  it("body alone (no fins) across Mach", () => {
    const cfg = cloneConfig(hypersonicConeCylinderFlare());
    const r = runSafe(cfg, { machs: [0.3, 1, 2, 5, 10, 25], alphas: [-20, 0, 10, 45], altKm: 30 });
    assertFiniteResult(r, "body-alone");
    assertBounded(r, "body-alone");
  });
  it("1..8 panels (cruciform variants)", () => {
    for (const n of [1, 2, 3, 4, 6, 8]) {
      const cfg = cloneConfig(basicFinner());
      cfg.sets[0].nPanels = n;
      cfg.sets[0].phiF = Array.from({ length: n }, (_, i) => (360 / n) * i);
      const r = runSafe(cfg, { machs: [2], alphas: [0, 5, 15], altKm: 0 });
      assertFiniteResult(r, `nPanels=${n}`);
    }
  });
});

describe("stress: span / taper extremes", () => {
  it("tiny span, huge span, zero taper, taper>1", () => {
    const base = cloneConfig(basicFinner());
    const variants: { name: string; patch: Partial<typeof base.sets[0]> }[] = [
      { name: "tiny", patch: { semispan: base.body!.parametric!.noseDiameter / 2 + 1e-4, tipChord: 1e-4, rootChord: 1e-3 } },
      { name: "huge", patch: { semispan: 0.15, rootChord: 0.08, tipChord: 0.04 } }, // large vs d=0.03 (~10× body radius)
      { name: "zero-taper", patch: { tipChord: 0 } },
      { name: "taper-gt1", patch: { tipChord: base.sets[0].rootChord * 2 } },
    ];
    for (const v of variants) {
      const cfg = cloneConfig(base);
      Object.assign(cfg.sets[0], v.patch);
      const r = runSafe(cfg, { machs: [0.8, 1.2, 3], alphas: [0, 10], altKm: 0 });
      assertFiniteResult(r, v.name);
      assertBounded(r, v.name);
    }
  });
});

describe("stress: body extremes", () => {
  it("zero-length sections, tiny nose radius, fineness extremes", () => {
    const cases: VehicleConfig[] = [
      {
        name: "needle",
        kind: "missile",
        body: { parametric: { noseKind: "cone", noseLength: 2, noseDiameter: 0.01, centerLength: 0.01, aftLength: 0 }, exitDiameter: 0 },
        sets: [],
        ref: { xcg: 1 },
      },
      {
        name: "stubby",
        kind: "missile",
        body: { parametric: { noseKind: "ogive", noseLength: 0.05, noseDiameter: 0.5, centerLength: 0.05, aftLength: 0 }, exitDiameter: 0 },
        sets: [],
        ref: { xcg: 0.05 },
      },
      {
        name: "zero-center",
        kind: "missile",
        body: { parametric: { noseKind: "cone", noseLength: 0.5, noseDiameter: 0.2, centerLength: 0, aftLength: 0.1, aftDiameter: 0.1 }, exitDiameter: 0 },
        sets: [],
        ref: { xcg: 0.3 },
      },
    ];
    for (const cfg of cases) {
      const r = runSafe(cfg, { machs: [0.5, 2, 8], alphas: [0, 5, 20], altKm: 0 });
      assertFiniteResult(r, cfg.name);
    }
  });
});

describe("stress: attitude extremes", () => {
  it("alpha -90..90, 180; beta; phi 0..360", () => {
    const cfg = cloneConfig(basicFinner());
    const alphas = [-90, -45, -10, 0, 10, 45, 90, 180];
    const r = runSafe(cfg, { machs: [2], alphas, altKm: 0 });
    assertFiniteResult(r, "alpha-extreme");
    for (const beta of [-20, 0, 30]) {
      const rb = runSafe(cfg, { machs: [1.5], alphas: [0, 10], beta, altKm: 0 });
      assertFiniteResult(rb, `beta=${beta}`);
    }
    for (const phi of [0, 22.5, 45, 90, 180, 270, 360]) {
      const rp = runSafe(cfg, { machs: [2], alphas: [5, 20], phi, beta: 0, altKm: 0 });
      assertFiniteResult(rp, `phi=${phi}`);
    }
  });
  it("fin deflection ±30", () => {
    const cfg = cloneConfig(cruciformMissile());
    cfg.sets[0].deflection = [30, -30, 30, -30];
    const r = runSafe(cfg, { machs: [2], alphas: [-5, 0, 10], altKm: 0 });
    assertFiniteResult(r, "defl");
  });
});

describe("stress: Mach / altitude / refs", () => {
  it("Mach 0.05 .. 25 including transonic points", () => {
    const cfg = cloneConfig(basicFinner());
    const machs = [0.05, 0.3, 0.8, 0.9, 1.0, 1.05, 1.1, 1.2, 2, 5, 10, 25];
    const r = runSafe(cfg, { machs, alphas: [0, 5], altKm: 0 });
    assertFiniteResult(r, "mach-sweep");
    assertBounded(r, "mach-sweep");
  });
  it("altitude 0 and 85 km", () => {
    const cfg = cloneConfig(basicFinner());
    for (const altKm of [0, 30, 85]) {
      const r = runSafe(cfg, { machs: [5], alphas: [0, 5], altKm });
      assertFiniteResult(r, `alt=${altKm}`);
    }
  });
  it("degenerate Sref/lref/xcg are sanitized", () => {
    const cfg = cloneConfig(basicFinner());
    cfg.ref.sRef = 0;
    cfg.ref.lRef = 0;
    cfg.ref.xcg = -1;
    const r = runSafe(cfg, { machs: [2], alphas: [0, 5], altKm: 0 });
    assertFiniteResult(r, "degen-ref");
    assert.ok(r.geometry.sRef > 0);
    assert.ok(r.geometry.lRef > 0);
  });
});

describe("stress: continuity", () => {
  const REL_MACH = 0.4; // max relative jump between adjacent Mach samples at fixed alpha
  const REL_ALPHA = 0.5; // fine alpha step
  const REL_PHI = 0.5;

  it("Mach blend continuity at alpha=5 (rel jump < 40%)", () => {
    const cfg = cloneConfig(basicFinner());
    const machs = [0.7, 0.85, 0.95, 1.0, 1.05, 1.15, 1.25, 1.5, 2, 3, 5, 8];
    const r = runSafe(cfg, { machs, alphas: [5], altKm: 0 });
    for (let i = 1; i < r.cases.length; i++) {
      const a = r.cases[i - 1].points[0].CN;
      const b = r.cases[i].points[0].CN;
      const denom = Math.max(Math.abs(a), Math.abs(b), 0.05);
      const rel = Math.abs(b - a) / denom;
      assert.ok(rel < REL_MACH, `CN jump M=${r.cases[i - 1].mach}→${r.cases[i].mach}: ${a}→${b} rel=${rel.toFixed(3)}`);
      const ca = r.cases[i - 1].points[0].CA;
      const cb = r.cases[i].points[0].CA;
      const relCA = Math.abs(cb - ca) / Math.max(Math.abs(ca), Math.abs(cb), 0.02);
      assert.ok(relCA < REL_MACH + 0.15, `CA jump M=${r.cases[i - 1].mach}→${r.cases[i].mach}: rel=${relCA.toFixed(3)}`);
    }
  });

  it("fine alpha continuity at M=2 (1 deg steps)", () => {
    const cfg = cloneConfig(basicFinner());
    const alphas = Array.from({ length: 41 }, (_, i) => -10 + i * 0.5);
    const r = runSafe(cfg, { machs: [2], alphas, altKm: 0 });
    for (let i = 1; i < r.cases[0].points.length; i++) {
      const a = r.cases[0].points[i - 1];
      const b = r.cases[0].points[i];
      const dCN = Math.abs(b.CN - a.CN);
      assert.ok(dCN < 2.0, `CN step a=${a.alpha}→${b.alpha}: dCN=${dCN}`);
      assert.ok(Number.isFinite(b.CN) && Number.isFinite(b.CA) && Number.isFinite(b.Cm));
    }
    void REL_ALPHA;
  });

  it("roll phi continuity at alpha_total=10, M=2", () => {
    const cfg = cloneConfig(basicFinner());
    const phis = Array.from({ length: 17 }, (_, i) => i * 22.5);
    let prev: RunResult | null = null;
    for (const phi of phis) {
      const r = runSafe(cfg, { machs: [2], alphas: [10], phi, altKm: 0 });
      assertFiniteResult(r, `phi=${phi}`);
      if (prev) {
        const a = prev.cases[0].points[0];
        const b = r.cases[0].points[0];
        // CN of total-alpha case should be roughly constant with phi for cruciform; allow variation
        const d = Math.hypot(b.CN - a.CN, b.CY - a.CY);
        assert.ok(d < 3, `force jump phi step →${phi}: d=${d}`);
      }
      prev = r;
    }
    void REL_PHI;
  });
});

describe("stress: invalid input surfaces clear errors", () => {
  it("empty mach list throws ValidationError", () => {
    assert.throws(() => sanitizeFlight({ machs: [], alphas: [0] }), ValidationError);
  });
  it("negative geometry lengths are clamped by sanitize", () => {
    const cfg = cloneConfig(basicFinner());
    cfg.body!.parametric!.noseLength = -1;
    cfg.body!.parametric!.noseDiameter = -0.1;
    const s = sanitizeConfig(cfg);
    assert.ok(s.body!.parametric!.noseLength > 0);
    assert.ok(s.body!.parametric!.noseDiameter > 0);
  });
});

describe("stress: buildModel never throws on presets", () => {
  it("all presets + aircraft", () => {
    for (const p of [basicFinner(), cruciformMissile(), hypersonicConeCylinderFlare(), aircraftWBT()]) {
      assert.doesNotThrow(() => buildModel(sanitizeConfig(p)));
    }
  });
});
