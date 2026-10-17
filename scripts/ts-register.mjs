// Zero-dependency TypeScript runner for the physics tests (Node >= 20.6).
// Usage: node --import ./scripts/ts-register.mjs src/lib/waverider/fidelity.test.ts
// Uses the repo's own `typescript` to strip types and resolves extensionless / "@/" imports.
import { register } from "node:module";
register("./ts-loader.mjs", import.meta.url);
