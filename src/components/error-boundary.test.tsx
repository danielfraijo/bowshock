/** PanelBoundary: caught render errors become a message and clear when the reset key changes. bun test src/components/error-boundary.test.tsx */
import assert from "node:assert/strict";
import { describe, it } from "node:test";
import { createElement } from "react";
import { renderToString } from "react-dom/server";
import { PanelBoundary } from "@/components/error-boundary";

describe("PanelBoundary", () => {
  it("renders children when healthy", () => {
    assert.ok(renderToString(createElement(PanelBoundary, { label: "x", children: createElement("p", null, "ok") })).includes("ok"));
  });
  it("shows the message for a caught error and resets on a new key", () => {
    const b = new PanelBoundary({ label: "3D viewer", children: null, resetKey: 1 });
    b.state = { ...b.state, ...PanelBoundary.getDerivedStateFromError(new Error("bad mesh")) } as typeof b.state;
    const html = JSON.stringify(b.render());
    assert.ok(html.includes("3D viewer") && html.includes("bad mesh"));
    assert.deepEqual(PanelBoundary.getDerivedStateFromProps({ label: "x", children: null, resetKey: 2 }, b.state), { error: null, key: 2 });
    assert.equal(PanelBoundary.getDerivedStateFromProps({ label: "x", children: null, resetKey: 1 }, b.state), null);
  });
});
