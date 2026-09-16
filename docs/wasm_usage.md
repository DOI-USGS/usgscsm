# USGSCSM WebAssembly Usage Guide

This guide explains how to use the USGSCSM WebAssembly module in your web applications.

## Installation

### Via NPM

```bash
npm install @usgs-astrogeology/usgscsm
```

### Via CDN

```html
<script type="module">
  import USGSCSM from 'https://cdn.jsdelivr.net/npm/@usgs-astrogeology/usgscsm/dist/usgscsm.js';
</script>
```

### Local Build

See [building_wasm.md](building_wasm.md) for instructions on building from source.

## Basic Usage

### Loading the Module

```javascript
import USGSCSM from '@usgs-astrogeology/usgscsm';

async function main() {
  // Load the WASM module
  const Module = await USGSCSM();
  
  // Create a sensor model instance
  const model = new Module.USGSCSMModel();
  
  // Now you can use the model...
}

main();
```

### Loading a Camera Model

#### From ISD (Image Support Data) JSON

```javascript
// Load ISD from file or API
const response = await fetch('camera_model.json');
const isdJson = await response.text();

// Create and load model
const model = new Module.USGSCSMModel();
const success = model.loadFromISD(isdJson, 'USGS_ASTRO_FRAME_SENSOR_MODEL');

if (!success) {
  console.error('Failed to load model');
}
```

#### From Model State JSON

Model state is an optimized format that loads faster than ISD:

```javascript
// Load previously saved model state
const stateJson = localStorage.getItem('cameraModelState');

const model = new Module.USGSCSMModel();
model.loadFromState(stateJson);
```

#### From Bytes or a URL (format auto-detected)

When you do not know which of the formats you have — JSON ISD, JSON/`.sup` state,
msgpack state, or STARDS — let the module detect it from the leading bytes:

```javascript
// From a buffer
const buf = await fetch('model.stards').then(r => r.arrayBuffer());
model.loadFromBytes(new Uint8Array(buf));

// Or fetch and load in one call. CORS applies; a "/vsicurl/" prefix is stripped
// before fetching. Rejects on a non-OK HTTP status.
await model.loadFromURL('https://host/model.json');
await model.loadFromURL('/vsicurl/https://host/model.stards');

// loadFrom takes a URL string, a Uint8Array, or an ArrayBuffer.
await model.loadFrom(source);
```

To branch on the format yourself, ask the module rather than parsing:

```javascript
const { isIsd, modelName } = Module.isUsgsCsmIsd(text);
if (isIsd) {
  model.loadFromISD(text, modelName);
} else if (Module.isUsgsCsmState(text).isState) {
  model.loadFromState(text);
}
```

### Coordinate Transformations

#### Image to Ground

Convert pixel coordinates to ground coordinates (ECEF):

```javascript
const line = 512;    // Row
const sample = 1024; // Column
const height = 0;    // Height above ellipsoid in meters

// Throws if no model is loaded or the projection fails.
const ground = model.imageToGround(line, sample, height);
console.log(`ECEF coordinates: (${ground.x}, ${ground.y}, ${ground.z})`);

// Convert ECEF to lat/lon if needed (requires external library)
// const latLon = ecefToLatLon(ground.x, ground.y, ground.z);
```

#### Ground to Image

Convert ground coordinates to pixel coordinates:

```javascript
const ecefX = 1234567.89; // ECEF X in meters
const ecefY = 2345678.90; // ECEF Y in meters
const ecefZ = 3456789.01; // ECEF Z in meters

// The returned fields are line/samp, matching csm::ImageCoord.
const pixel = model.groundToImage(ecefX, ecefY, ecefZ);
console.log(`Pixel coordinates: line=${pixel.line}, samp=${pixel.samp}`);
```

### Sensor Queries

#### Get Sensor Position

```javascript
const position = model.getSensorPosition(512, 1024);

const altitude = Math.sqrt(
  position.x**2 + position.y**2 + position.z**2
) - bodyRadius;

console.log(`Camera altitude: ${altitude} meters`);
```

#### Get Sensor Velocity

```javascript
const velocity = model.getSensorVelocity(512, 1024);

const speed = Math.sqrt(
  velocity.x**2 + velocity.y**2 + velocity.z**2
);

console.log(`Spacecraft speed: ${speed} m/s`);
```

### Model Information

```javascript
// Get model metadata
console.log('Model type:', model.getModelName());
console.log('Image ID:', model.getImageIdentifier());
console.log('Sensor ID:', model.getSensorIdentifier());
console.log('Platform ID:', model.getPlatformIdentifier());

// Get image dimensions
const size = model.getImageSize();
console.log(`Image size: ${size.line} x ${size.samp} pixels`);

// Check if model is loaded
if (!model.isLoaded()) {
  console.error('No model loaded');
}
```

## Supported Sensor Models

The WASM build supports the following sensor model types:

| Model Type | Constant | Description |
|------------|----------|-------------|
| Frame Camera | `USGS_ASTRO_FRAME_SENSOR_MODEL` | Instantaneous framing cameras |
| Line Scanner | `USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL` | Pushbroom/whiskbroom scanners |
| Push Frame | `USGS_ASTRO_PUSH_FRAME_SENSOR_MODEL` | Time-delayed multi-line cameras |
| SAR | `USGS_ASTRO_SAR_SENSOR_MODEL` | Synthetic Aperture Radar |
| Projected | `USGS_ASTRO_PROJECTED_SENSOR_MODEL` | Map-projected images (embedded PROJ) |

**Note:** The projected sensor model uses PROJ, compiled to WebAssembly with its
coordinate database (proj.db) embedded in the module and served from memory, so
no external data files are required. It adds several MB to the module size.

## Complete Example

```javascript
import USGSCSM from '@usgs-astrogeology/usgscsm';

async function processImage() {
  // Load WASM module
  const Module = await USGSCSM();
  
  // Load camera model
  const model = new Module.USGSCSMModel();
  const isdResponse = await fetch('mro_hirise_image.json');
  const isdJson = await isdResponse.text();
  
  if (!model.loadFromISD(isdJson, 'USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL')) {
    throw new Error('Failed to load camera model');
  }
  
  console.log(`Loaded ${model.getModelName()}`);
  console.log(`Image: ${model.getImageIdentifier()}`);
  
  // Get image dimensions
  const size = model.getImageSize();
  console.log(`Processing ${size.line} x ${size.samp} image`);
  
  // Sample points across the image
  const points = [];
  const step = 100;
  
  for (let line = 0; line < size.line; line += step) {
    for (let sample = 0; sample < size.samp; sample += step) {
      // Projection failures throw, so skip the ones that do not intersect.
      try {
        points.push({
          pixel: { line, sample },
          ecef: model.imageToGround(line, sample, 0)
        });
      } catch (e) {
        // Ray missed the body at this height.
      }
    }
  }
  
  console.log(`Computed ${points.length} ground points`);
  
  // Save model state for later use
  const state = model.getModelState();
  localStorage.setItem('lastModelState', state);
  
  return points;
}

processImage().then(points => {
  console.log('Processing complete:', points);
}).catch(error => {
  console.error('Error:', error);
});
```

## TypeScript Support

TypeScript definitions are included:

```typescript
import USGSCSM, { 
  USGSCSMModel, 
  EcefCoord, 
  ImageCoord, 
  ModelName 
} from '@usgs-astrogeology/usgscsm';

async function processWithTypes() {
  const Module = await USGSCSM();
  const model: USGSCSMModel = new Module.USGSCSMModel();
  
  // TypeScript provides autocomplete and type checking
  const ground: EcefCoord = model.imageToGround(100, 200, 0);
  console.log(`X: ${ground.x}, Y: ${ground.y}, Z: ${ground.z}`);
}
```

## Performance Tips

### 1. Reuse Model Instances

Creating new models is expensive. Reuse instances when possible:

```javascript
// Good - reuse model
const model = new Module.USGSCSMModel();
for (const isd of isdList) {
  model.loadFromISD(isd, modelType);
  // process...
}

// Bad - creates unnecessary instances
for (const isd of isdList) {
  const model = new Module.USGSCSMModel(); // Wasteful
  model.loadFromISD(isd, modelType);
}
```

### 2. Use Model State

Model state loads much faster than ISD:

```javascript
// First time: load from ISD
model.loadFromISD(isdJson, modelType);

// Save state
const state = model.getModelState();
sessionStorage.setItem('modelState', state);

// Later: load from state (much faster)
const cachedState = sessionStorage.getItem('modelState');
model.loadFromState(cachedState);
```

### 3. Batch Operations

Minimize JavaScript ↔ WebAssembly calls:

```javascript
// Good - prepare data in JS, then process
const pixels = generatePixelGrid(1024);
const groundPoints = pixels.map(p => 
  model.imageToGround(p.line, p.sample, 0)
);

// Better - but requires custom bindings
// const groundPoints = model.imageToGroundBatch(pixelArray);
```

### 4. Web Workers

For heavy processing, use Web Workers to avoid blocking the main thread:

```javascript
// worker.js
import USGSCSM from '@usgs-astrogeology/usgscsm';

self.onmessage = async (e) => {
  const Module = await USGSCSM();
  const model = new Module.USGSCSMModel();
  model.loadFromState(e.data.modelState);
  
  const results = e.data.pixels.map(p =>
    model.imageToGround(p.line, p.sample, 0)
  );
  
  self.postMessage(results);
};
```

## Error Handling

The loaders return a boolean; everything else throws. Nothing returns `null`.

```javascript
// Loading returns false on a model this build cannot construct, and throws on
// input that is not a valid ISD/state at all.
try {
  if (!model.loadFromISD(isdJson, modelType)) {
    console.error('Failed to load model');
    return;
  }
} catch (e) {
  console.error('Not a usable ISD:', e);
  return;
}

// The accessors throw if no model is loaded or the projection fails.
try {
  const ground = model.imageToGround(line, sample, height);
  const pixel = model.groundToImage(ground.x, ground.y, ground.z);
} catch (e) {
  console.error('Projection failed:', e);
}
```

Embind surfaces C++ exceptions, so a thrown value is not always a JS `Error`;
catch it, but do not rely on `e.message` being present.

## Browser Compatibility

Requires WebAssembly support:

- ✅ Chrome 90+
- ✅ Firefox 88+
- ✅ Safari 14+
- ✅ Edge 90+
- ✅ Mobile browsers (iOS 14+, Android Chrome 90+)

Check compatibility:

```javascript
if (!('WebAssembly' in window)) {
  alert('Your browser does not support WebAssembly');
  return;
}
```

## Bundle Size

| Format | Size |
|--------|------|
| WASM (uncompressed) | ~12 MB |
| JS glue code | ~140 KB |
| **Total gzipped** | **~3 MB** |

Most of that is PROJ and the embedded proj.db. See
[building_wasm.md](building_wasm.md#bundle-size) for the per-file breakdown and
how to trim it.

**Important:** Always serve WASM files with gzip compression enabled.

## Debugging

Enable verbose logging:

```javascript
const Module = await USGSCSM({
  print: (text) => console.log('WASM:', text),
  printErr: (text) => console.error('WASM ERROR:', text)
});
```

Check the browser console for error messages. Most issues are related to:
- Invalid ISD format
- Wrong model type specified
- Coordinates outside image bounds
- Model not loaded before calling methods

## Next Steps

- See [../tests/wasm/](../tests/wasm/) for runnable examples
- Read the TypeScript definitions in
  [../src/wasm/usgscsm.d.ts](../src/wasm/usgscsm.d.ts) for the full API
- Check [building_wasm.md](building_wasm.md) to build from source
- Report issues at [GitHub](https://github.com/USGS-Astrogeology/usgscsm/issues)

## License

USGSCSM is in the public domain. See [LICENSE.md](../LICENSE.md) for details.
