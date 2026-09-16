// Tests for the byte-buffer and URL/fetch loading bindings. A local HTTP server
// serves a fixture, so the fetch path is exercised without external network access.

import test from 'node:test';
import assert from 'node:assert/strict';
import http from 'node:http';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

import { loadUsgscsm } from './loader.mjs';

const here = dirname(fileURLToPath(import.meta.url));
const FRAME_ISD_PATH = resolve(here, 'fixtures', 'frame_isd.json');
const FRAME_ISD_BYTES = readFileSync(FRAME_ISD_PATH);  // Node Buffer
const FRAME_MODEL_NAME = 'USGS_ASTRO_FRAME_SENSOR_MODEL';

const Module = await loadUsgscsm();

// Serve the frame ISD fixture over HTTP for the duration of a callback.
async function withServer(fn) {
  const server = http.createServer((req, res) => {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(FRAME_ISD_BYTES);
  });
  await new Promise((r) => server.listen(0, '127.0.0.1', r));
  const base = `http://127.0.0.1:${server.address().port}`;
  try {
    await fn(base);
  } finally {
    server.close();
  }
}

test('loadFromBytes decodes a JSON ISD (Uint8Array)', () => {
  const model = new Module.USGSCSMModel();
  const ok = model.loadFromBytes(new Uint8Array(FRAME_ISD_BYTES));
  assert.equal(ok, true);
  assert.equal(model.getModelName(), FRAME_MODEL_NAME);
});

test('loadFromURL fetches a bare http URL', async () => {
  await withServer(async (base) => {
    const model = new Module.USGSCSMModel();
    const ok = await model.loadFromURL(`${base}/frame.json`);
    assert.equal(ok, true);
    assert.equal(model.getModelName(), FRAME_MODEL_NAME);
    // The fetched model actually works.
    const g = model.imageToGround(8, 8, 0);
    assert.ok(Number.isFinite(g.x) && Number.isFinite(g.y) && Number.isFinite(g.z));
  });
});

test('loadFromURL accepts a /vsicurl/ path', async () => {
  await withServer(async (base) => {
    const model = new Module.USGSCSMModel();
    const ok = await model.loadFromURL(`/vsicurl/${base}/frame.json`);
    assert.equal(ok, true);
    assert.equal(model.getModelName(), FRAME_MODEL_NAME);
  });
});

test('loadFrom dispatches on URL, Uint8Array, and ArrayBuffer', async () => {
  // Uint8Array
  let model = new Module.USGSCSMModel();
  assert.equal(await model.loadFrom(new Uint8Array(FRAME_ISD_BYTES)), true);

  // ArrayBuffer (sliced exactly, as a browser fetch's arrayBuffer() would be)
  model = new Module.USGSCSMModel();
  const ab = FRAME_ISD_BYTES.buffer.slice(
    FRAME_ISD_BYTES.byteOffset,
    FRAME_ISD_BYTES.byteOffset + FRAME_ISD_BYTES.byteLength);
  assert.equal(await model.loadFrom(ab), true);

  // URL
  await withServer(async (base) => {
    model = new Module.USGSCSMModel();
    assert.equal(await model.loadFrom(`${base}/frame.json`), true);
  });
});

test('loadFromURL rejects a non-URL source', async () => {
  const model = new Module.USGSCSMModel();
  await assert.rejects(() => model.loadFromURL('not-a-url'));
});

test('loadFromURL rejects on HTTP error status', async () => {
  const server = http.createServer((req, res) => { res.writeHead(404); res.end(); });
  await new Promise((r) => server.listen(0, '127.0.0.1', r));
  const base = `http://127.0.0.1:${server.address().port}`;
  try {
    const model = new Module.USGSCSMModel();
    await assert.rejects(() => model.loadFromURL(`${base}/missing.json`), /HTTP 404/);
  } finally {
    server.close();
  }
});
