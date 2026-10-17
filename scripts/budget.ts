import { budgetSummary, errorBudget } from "../src/lib/waverider/errorbudget";
const t0 = performance.now();
const rows = errorBudget();
console.log(`budget: ${rows.length} rows in ${(performance.now() - t0).toFixed(0)} ms`);
const sum = budgetSummary(rows);
console.log("shape | method | n | max|err|% | mean|err|% | tol% | within");
for (const s of sum) console.log(`${s.shape} | ${s.method} | ${s.n} | ${s.maxAbsPct.toFixed(2)} | ${s.meanAbsPct.toFixed(2)} | ${s.tolPct} | ${s.allWithin}`);
if (process.argv[2] === "fails") for (const r of rows) if (!r.within) console.log("OUT", r.shape, r.case, "M" + r.M, r.quantity, r.method, r.reference.toFixed(4), r.value.toFixed(4), r.errPct.toFixed(1));
