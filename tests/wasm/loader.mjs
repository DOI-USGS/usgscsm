// Locates and instantiates the compiled WASM module. It is a build artifact, not
// checked in, so set USGSCSM_WASM=/path/to/dist/usgscsm.js if your build
// directory is not one of the candidates below.

import { existsSync } from 'node:fs';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { dirname, resolve } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const repoRoot = resolve(here, '..', '..');

// Candidate locations for the built module, checked in order.
const candidates = [
  process.env.USGSCSM_WASM,
  resolve(repoRoot, 'build-wasm', 'dist', 'usgscsm.js'),
  resolve(repoRoot, 'wasmbuild', 'dist', 'usgscsm.js'),
  resolve(repoRoot, 'build', 'dist', 'usgscsm.js'),
  resolve(repoRoot, 'dist', 'usgscsm.js'),
].filter(Boolean);

export function findModulePath() {
  for (const p of candidates) {
    if (existsSync(p)) return p;
  }
  return null;
}

// Throws if not found, so a missing build fails loudly instead of passing zero
// assertions.
export async function loadUsgscsm() {
  const modulePath = findModulePath();
  if (!modulePath) {
    throw new Error(
      'Could not find the built usgscsm WASM module. Build it with ' +
      '`emcmake cmake .. && emmake make` and/or set the USGSCSM_WASM ' +
      'environment variable to the path of dist/usgscsm.js. Looked in:\n  ' +
      candidates.join('\n  '));
  }
  const factory = (await import(pathToFileURL(modulePath).href)).default;
  return factory();
}
