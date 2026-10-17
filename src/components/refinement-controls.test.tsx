/**
 * SSR smoke test for the refinement panel (no browser): renders every preset plus hostile custom values and checks
 * the markup. bun test src/components/refinement-controls.test.tsx
 */
import assert from "node:assert/strict";
import { describe, it } from "node:test";
import { renderToString } from "react-dom/server";
import { createElement } from "react";
import { RefinementControls } from "@/components/refinement-controls";
import { presetSettings, validateRefinement } from "@/lib/waverider/refinement";
import { DEFAULT_PARAMS, type DesignParams } from "@/lib/waverider/types";

const render = (params: DesignParams) =>
  renderToString(createElement(RefinementControls, { params, patch: () => {}, setDragging: () => {} }));

describe("RefinementControls SSR", () => {
  it("renders for undefined (medium default) and every preset, with the Legacy / Custom options", () => {
    for (const refinement of [undefined, presetSettings("legacy"), presetSettings("coarse"), presetSettings("medium"), presetSettings("fine")]) {
      const html = render({ ...DEFAULT_PARAMS, refinement });
      for (const label of ["Legacy", "Coarse", "Medium", "Fine", "Custom"]) assert.ok(html.includes(label), `${refinement?.preset} has ${label}`);
      assert.ok(!html.includes("NaN") && !html.includes("Infinity") && !html.includes("undefined"), `${refinement?.preset}: no NaN/Infinity/undefined in the markup`);
    }
  });
  it("custom shows tolerance, sequencing, order and curvature controls, also for hostile values", () => {
    for (const volume of [{}, { tol: NaN, sequence: Infinity, nStream: 1e9 }, { tol: 0, order: 2 as const }]) {
      const v = validateRefinement({ preset: "custom", surface: { curvature: 5 }, volume });
      const html = render({ ...DEFAULT_PARAMS, aeroMethod: "euler", refinement: v.settings });
      for (const label of ["Convergence tolerance", "Grid sequencing levels", "Accuracy order", "Surface curvature adaptation", "Wall clustering", "Shock clustering"]) assert.ok(html.includes(label), label);
      assert.ok(!/NaN|Infinity|undefined/.test(html));
    }
  });
});
