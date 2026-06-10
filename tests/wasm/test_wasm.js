/**
 * USGSCSM WebAssembly Test Suite
 * Tests model loading (ISD/state), coordinate transformations, and utility functions
 */

import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

// Find the WASM build output - check common build directories
const repoRoot = path.join(__dirname, '../..');
const buildDirs = ['wasmbuild', 'wasm', 'build-wasm', 'build'];
let wasmModule;

for (const buildDir of buildDirs) {
  const distPath = path.join(repoRoot, buildDir, 'dist', 'usgscsm.js');
  if (fs.existsSync(distPath)) {
    const { default: USGSCSM } = await import(`file://${distPath}`);
    wasmModule = USGSCSM;
    console.log(`Using WASM from: ${buildDir}/dist/\n`);
    break;
  }
}

if (!wasmModule) {
  console.error('ERROR: Could not find WASM build output.');
  console.error('Expected to find usgscsm.js in one of:');
  buildDirs.forEach(dir => console.error(`  - ${dir}/dist/usgscsm.js`));
  console.error('\nPlease build WASM first:');
  console.error('  mkdir wasmbuild && cd wasmbuild');
  console.error('  emcmake cmake .. && emmake make');
  process.exit(1);
}

async function runTests() {
  console.log('Loading USGSCSM WASM...');
  const Module = await wasmModule();
  console.log('[PASS] Module loaded\n');

  // === Model Loading & Transformation Tests ===
  console.log('--- Model Loading & Transformations ---');

  const isdPath = path.join(__dirname, '../data/simpleFramerISD.json');
  const isdStr = fs.readFileSync(isdPath, 'utf8');
  const isd = JSON.parse(isdStr);

  const model = new Module.USGSCSMModel();
  if (!model.loadFromISD(JSON.stringify(isd), 'USGS_ASTRO_FRAME_SENSOR_MODEL')) {
    throw new Error('Failed to load ISD');
  }
  console.log('[PASS] Model loaded from ISD');

  const size = model.getImageSize();
  console.log(`[PASS] Image size: ${size.line} x ${size.samp}`);

  const centerLine = size.line / 2;
  const centerSamp = size.samp / 2;
  const ground = model.imageToGround(centerLine, centerSamp, 0);
  console.log(`[PASS] imageToGround: (${ground.x.toFixed(2)}, ${ground.y.toFixed(2)}, ${ground.z.toFixed(2)})`);

  const pixel = model.groundToImage(ground.x, ground.y, ground.z);
  const error = Math.sqrt(
    Math.pow(pixel.line - centerLine, 2) +
    Math.pow(pixel.samp - centerSamp, 2)
  );
  if (error > 0.01) throw new Error(`Round-trip error too large: ${error}`);
  console.log(`[PASS] Round-trip error: ${error.toFixed(6)} pixels\n`);

  // === State Serialization Tests ===
  console.log('--- State Serialization ---');

  const state = model.getModelState();
  console.log(`[PASS] State saved (${state.length} bytes)`);

  let stateObj;
  try {
    stateObj = JSON.parse(state);
  } catch (e) {
    const lines = state.split('\n');
    stateObj = JSON.parse(lines.slice(1).join('\n'));
  }
  if (!stateObj.m_modelName) throw new Error('State missing m_modelName');
  console.log(`[PASS] State contains model name: ${stateObj.m_modelName}`);

  const model2 = new Module.USGSCSMModel();
  if (!model2.loadFromState(state)) throw new Error('Failed to load from state');
  console.log('[PASS] Model loaded from state');

  const ground2 = model2.imageToGround(centerLine, centerSamp, 0);
  const groundDiff = Math.sqrt(
    Math.pow(ground2.x - ground.x, 2) +
    Math.pow(ground2.y - ground.y, 2) +
    Math.pow(ground2.z - ground.z, 2)
  );
  if (groundDiff > 1e-10) throw new Error(`Models differ by ${groundDiff} meters`);
  console.log(`[PASS] State produces identical results (diff: ${groundDiff.toFixed(10)} m)\n`);

  // === Utility Function Tests ===
  console.log('--- Utility Functions ---');

  const isdCheck = Module.isUsgsCsmIsd(isdStr);
  if (!isdCheck.isIsd || isdCheck.modelName !== 'USGS_ASTRO_FRAME_SENSOR_MODEL') {
    throw new Error('isUsgsCsmIsd failed to detect ISD');
  }
  console.log(`[PASS] isUsgsCsmIsd correctly identifies ISD (${isdCheck.modelName})`);

  const stateCheck = Module.isUsgsCsmState(state);
  if (!stateCheck.isState || stateCheck.modelName !== 'USGS_ASTRO_FRAME_SENSOR_MODEL') {
    throw new Error('isUsgsCsmState failed to detect state');
  }
  console.log(`[PASS] isUsgsCsmState correctly identifies state (${stateCheck.modelName})`);

  const wrongCheck1 = Module.isUsgsCsmState(isdStr);
  if (wrongCheck1.isState) throw new Error('isUsgsCsmState incorrectly detected ISD as state');
  console.log('[PASS] isUsgsCsmState correctly rejects ISD');

  const wrongCheck2 = Module.isUsgsCsmIsd(state);
  if (wrongCheck2.isIsd) throw new Error('isUsgsCsmIsd incorrectly detected state as ISD');
  console.log('[PASS] isUsgsCsmIsd correctly rejects state\n');

  console.log('=== All tests passed! ===');
}

runTests().catch(err => {
  console.error('[FAIL]', err.message);
  process.exit(1);
});
