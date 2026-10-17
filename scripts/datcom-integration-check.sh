#!/usr/bin/env bash
# When origin/feat/integration lands, verify DATCOM still typechecks/tests/builds on top of it.
# Usage: scripts/datcom-integration-check.sh
# Exit 0 = clean; 1 = missing integration / conflict / failure.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

git fetch origin --quiet 2>/dev/null || true

if ! git rev-parse --verify origin/feat/integration >/dev/null 2>&1; then
  echo "datcom-integration-check: origin/feat/integration does not exist yet — nothing to merge."
  echo "Re-run this script after feat/integration is published."
  exit 0
fi

BRANCH="integ-check-datcom-$$"
cleanup() { git checkout feat/datcom >/dev/null 2>&1 || true; git branch -D "$BRANCH" >/dev/null 2>&1 || true; }
trap cleanup EXIT

git checkout -B "$BRANCH" origin/feat/integration
# Prefer merge of local feat/datcom tip
if ! git merge --no-edit feat/datcom; then
  echo "MERGE CONFLICT with feat/datcom — resolve and re-run."
  git merge --abort 2>/dev/null || true
  exit 1
fi

echo "== typecheck =="
npm run typecheck
echo "== test:datcom =="
npm run test:datcom
echo "== vite build =="
npx vite build
echo "datcom-integration-check: OK on origin/feat/integration + feat/datcom"
