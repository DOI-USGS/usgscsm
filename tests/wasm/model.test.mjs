// Test suite for the usgscsm WebAssembly JS bindings. Uses Node's built-in test
// runner, so it needs no external dependencies. Run with
// `node --test tests/wasm/*.test.mjs`.

import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

import { loadUsgscsm } from './loader.mjs';

const here = dirname(fileURLToPath(import.meta.url));
const FRAME_ISD = readFileSync(resolve(here, 'fixtures', 'frame_isd.json'), 'utf8');
const FRAME_MODEL_NAME = 'USGS_ASTRO_FRAME_SENSOR_MODEL';

// Instantiate the module once and share it across tests.
const Module = await loadUsgscsm();

// A freshly loaded frame model, for tests that need a loaded model.
function loadedFrameModel() {
  const model = new Module.USGSCSMModel();
  assert.ok(model.loadFromISD(FRAME_ISD, FRAME_MODEL_NAME),
            'loadFromISD should succeed for the frame ISD fixture');
  return model;
}

test('module exposes the USGSCSMModel class', () => {
  assert.equal(typeof Module.USGSCSMModel, 'function');
  const model = new Module.USGSCSMModel();
  assert.equal(model.isLoaded(), false, 'a new model reports not loaded');
});

test('loadFromISD loads a frame model', () => {
  const model = new Module.USGSCSMModel();
  const ok = model.loadFromISD(FRAME_ISD, FRAME_MODEL_NAME);
  assert.equal(ok, true);
  assert.equal(model.isLoaded(), true);
  assert.equal(model.getModelName(), FRAME_MODEL_NAME);
});

test('imageToGround returns an ECEF {x,y,z} point', () => {
  const model = loadedFrameModel();
  const g = model.imageToGround(8.0, 8.0, 0.0);
  for (const k of ['x', 'y', 'z']) {
    assert.equal(typeof g[k], 'number');
    assert.ok(Number.isFinite(g[k]), `${k} should be finite`);
  }
});

test('groundToImage round-trips an imageToGround result', () => {
  const model = loadedFrameModel();
  const line = 8.0;
  const samp = 8.0;
  const g = model.imageToGround(line, samp, 0.0);
  const p = model.groundToImage(g.x, g.y, g.z);
  // The pixel returned uses {line, samp}; it should match the original within a
  // small tolerance (projection is iterative).
  assert.ok(Math.abs(p.line - line) < 1e-3, `line round-trip: ${p.line}`);
  assert.ok(Math.abs(p.samp - samp) < 1e-3, `samp round-trip: ${p.samp}`);
});

test('getImageSize reports positive dimensions', () => {
  const model = loadedFrameModel();
  const size = model.getImageSize();
  assert.ok(size.line > 0, 'image line count should be positive');
  assert.ok(size.samp > 0, 'image sample count should be positive');
});

test('getSensorPosition returns a finite ECEF point', () => {
  const model = loadedFrameModel();
  const pos = model.getSensorPosition(8.0, 8.0);
  for (const k of ['x', 'y', 'z']) {
    assert.ok(Number.isFinite(pos[k]), `${k} should be finite`);
  }
});

test('getModelState round-trips through loadFromState', () => {
  const model = loadedFrameModel();
  const state = model.getModelState();
  assert.ok(state.length > 0, 'model state should be non-empty');

  const restored = new Module.USGSCSMModel();
  assert.equal(restored.loadFromState(state), true,
               'loadFromState should accept getModelState output');
  assert.equal(restored.getModelName(), FRAME_MODEL_NAME);

  // The restored model produces the same projection as the original.
  const a = model.imageToGround(8.0, 8.0, 0.0);
  const b = restored.imageToGround(8.0, 8.0, 0.0);
  assert.ok(Math.abs(a.x - b.x) < 1e-6, 'x matches after state round-trip');
  assert.ok(Math.abs(a.y - b.y) < 1e-6, 'y matches after state round-trip');
  assert.ok(Math.abs(a.z - b.z) < 1e-6, 'z matches after state round-trip');
});

test('loadFromISD rejects non-ISD input', () => {
  const model = new Module.USGSCSMModel();
  // Malformed/incomplete ISD input surfaces as a thrown error from the binding
  // (the underlying CSM construction throws); either way, no model is loaded.
  assert.throws(() => model.loadFromISD('{"not": "an isd"}', FRAME_MODEL_NAME));
  assert.equal(model.isLoaded(), false);
});

// The detectors are how a caller decides which loader to use, so each must also
// reject the other's format rather than only recognizing its own.
test('isUsgsCsmIsd and isUsgsCsmState identify their own format', () => {
  const state = loadedFrameModel().getModelState();

  const isd = Module.isUsgsCsmIsd(FRAME_ISD);
  assert.equal(isd.isIsd, true);
  assert.equal(isd.modelName, FRAME_MODEL_NAME);

  const st = Module.isUsgsCsmState(state);
  assert.equal(st.isState, true);
  assert.equal(st.modelName, FRAME_MODEL_NAME);

  assert.equal(Module.isUsgsCsmState(FRAME_ISD).isState, false);
  assert.equal(Module.isUsgsCsmIsd(state).isIsd, false);
});

test('projecting before loading a model throws', () => {
  const model = new Module.USGSCSMModel();
  // The binding throws when no model is loaded. Embind surfaces the C++
  // exception (which may be a pointer/number rather than a JS Error), so assert
  // only that it throws, not on the thrown value's shape.
  assert.throws(() => model.imageToGround(0, 0, 0));
});
