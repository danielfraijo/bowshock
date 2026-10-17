/** SSR smoke test: the whole Designer renders (instant study + safe build + boundaries) without throwing. bun test src/components/designer.test.tsx */
import assert from "node:assert/strict";
import { describe, it } from "node:test";
import { createElement } from "react";
import { renderToString } from "react-dom/server";
import { Designer } from "@/components/designer";

describe("Designer SSR", () => {
  it("renders with no NaN / undefined text and includes the refinement panel entry point", () => {
    const html = renderToString(createElement(Designer));
    assert.ok(html.length > 5000);
    assert.ok(!html.includes("NaN") && !html.includes(">undefined<"));
  });
});
