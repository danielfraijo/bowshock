# aerodb-perf benchmark

2026-10-22T23:25:19.153Z · fdc467f · Node v20.19.2 · 8× Intel(R) Xeon(R) Processor · pool 7 worker_threads · cuspis-panel wasm 94923 B

TS = generateAeroDatabase engine "ts" (panelAero forcesOnly: heating skipped). TS-legacy = e586e26's loop (heating on). Times include the elevon geometry rebuilds and the dynamic-derivative points. Pool cold = fresh workers (start, TS loader, wasm compile).

## generateAeroDatabase (seconds)

| family | method | tris | points | TS-legacy | TS | wasm | pool | pool cold | wasm vs TS | pool vs TS |
|---|---|---|---|---|---|---|---|---|---|---|
| caret | cbaero | 3398 | 480 | – | 1.51 | 0.48 | 0.16 | 4.65 | ×3.2 | ×9.2 |

## Kernel only (µs per point, one mesh, warm caches)

| family | method | tris | TS | wasm | speedup |
|---|---|---|---|---|---|
| caret | cbaero | 3398 | 1214 | 732 | ×1.66 |
