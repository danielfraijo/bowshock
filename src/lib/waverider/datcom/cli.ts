#!/usr/bin/env node
/**
 * Tiny DATCOM CLI: read a Missile/USAF DATCOM deck or a JSON VehicleConfig+FlightSpec,
 * print a for006-style table (or csv/json/usaf/sixdof).
 *
 *   node --experimental-strip-types src/lib/waverider/datcom/cli.ts deck.dat
 *   node --experimental-strip-types src/lib/waverider/datcom/cli.ts --format csv --preset finner
 *   node --experimental-strip-types src/lib/waverider/datcom/cli.ts --json config.json
 */
import { readFileSync, writeFileSync } from "node:fs";
import { parseDeck } from "./deck.ts";
import { format, type OutputFormat } from "./output.ts";
import { PRESETS, type Preset } from "./presets.ts";
import { runCases } from "./model.ts";
import type { FlightSpec, VehicleConfig } from "./types.ts";

function usage(): never {
  console.error(`usage: datcom-run [options] [deck-or-json]
  --preset <id>     finner | cruciform | canard | flare | aircraft
  --format <fmt>    missile (default) | usaf | csv | json | sixdof
  --out <file>      write to file instead of stdout
  --json <file>     read { config, flight } JSON
  --per-rad         emit derivatives per radian
  --help`);
  process.exit(2);
}

function main(argv: string[]) {
  let formatName: OutputFormat = "missile";
  let out: string | null = null;
  let preset: string | null = null;
  let jsonFile: string | null = null;
  let perRad = false;
  const positional: string[] = [];
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i];
    if (a === "--help" || a === "-h") usage();
    else if (a === "--format") formatName = argv[++i] as OutputFormat;
    else if (a === "--out") out = argv[++i];
    else if (a === "--preset") preset = argv[++i];
    else if (a === "--json") jsonFile = argv[++i];
    else if (a === "--per-rad") perRad = true;
    else if (a.startsWith("-")) {
      console.error(`unknown flag ${a}`);
      usage();
    } else positional.push(a);
  }

  let config: VehicleConfig;
  let flight: FlightSpec;
  const warnings: string[] = [];

  if (preset) {
    const p = PRESETS.find((x: Preset) => x.id === preset);
    if (!p) {
      console.error(`unknown preset '${preset}'. choose: ${PRESETS.map((x) => x.id).join(", ")}`);
      process.exit(1);
    }
    config = p.config;
    flight = p.flight;
  } else if (jsonFile || (positional[0] && /\.json$/i.test(positional[0]))) {
    const path = jsonFile ?? positional[0];
    const j = JSON.parse(readFileSync(path, "utf8")) as { config: VehicleConfig; flight: FlightSpec };
    config = j.config;
    flight = j.flight;
  } else if (positional[0]) {
    const text = readFileSync(positional[0], "utf8");
    const parsed = parseDeck(text);
    config = parsed.config;
    flight = parsed.flight;
    warnings.push(...parsed.warnings);
    for (const u of parsed.unsupported) warnings.push(`UNSUPPORTED: ${u}`);
  } else {
    const p = PRESETS[0];
    config = p.config;
    flight = p.flight;
    console.error(`(no input — running preset '${p.id}')`);
  }

  const result = runCases(config, flight);
  for (const w of [...warnings, ...result.warnings]) console.error(`WARN: ${w}`);
  const text = format(result, formatName, perRad);
  if (out) writeFileSync(out, text);
  else process.stdout.write(text + (text.endsWith("\n") ? "" : "\n"));
}

main(process.argv.slice(2));
