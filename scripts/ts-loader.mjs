import { existsSync, readFileSync } from "node:fs";
import { dirname, resolve as pathResolve } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";
import ts from "typescript";

const root = pathResolve(dirname(fileURLToPath(import.meta.url)), "..");
const EXT = [".ts", ".tsx", "/index.ts", "/index.tsx"];

export async function resolve(specifier, context, next) {
  let base = null;
  if (specifier.startsWith("@/")) base = pathResolve(root, "src", specifier.slice(2));
  else if (specifier.startsWith("/") && !specifier.startsWith("//")) base = specifier;
  else if ((specifier.startsWith("./") || specifier.startsWith("../")) && context.parentURL?.startsWith("file:")) {
    base = pathResolve(dirname(fileURLToPath(context.parentURL)), specifier);
  }
  if (base && !/\.(m?js|json|ts|tsx)$/.test(base)) {
    for (const e of EXT) if (existsSync(base + e)) return { url: pathToFileURL(base + e).href, shortCircuit: true };
  }
  if (base && /\.tsx?$/.test(base) && existsSync(base)) return { url: pathToFileURL(base).href, shortCircuit: true };
  return next(specifier, context);
}

export async function load(url, context, next) {
  if (/\.tsx?$/.test(url) && url.startsWith("file:")) {
    const src = readFileSync(fileURLToPath(url), "utf8");
    const out = ts.transpileModule(src, {
      fileName: fileURLToPath(url),
      compilerOptions: {
        target: ts.ScriptTarget.ES2022,
        module: ts.ModuleKind.ESNext,
        jsx: ts.JsxEmit.ReactJSX,
        verbatimModuleSyntax: false,
        isolatedModules: true,
      },
    });
    return { format: "module", source: out.outputText, shortCircuit: true };
  }
  return next(url, context);
}
