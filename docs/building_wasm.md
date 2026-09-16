# Building USGSCSM for WebAssembly

This document describes how to build USGSCSM as a WebAssembly module for use in web browsers.

## Prerequisites

- **Emscripten SDK (emsdk)** 3.1.58 (the version the build is tested against;
  it pairs with Binaryen 117)
- **CMake** 3.10 or later
- **Node.js** 18+ (for testing; the test suite uses `node:test` and ESM)
- **Git** (with submodules initialized)

## Installing Emscripten

If you don't have Emscripten installed:

```bash
# Clone the Emscripten SDK
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk

# Install and activate the latest version
./emsdk install latest
./emsdk activate latest

# Set up environment variables (required for each terminal session)
source ./emsdk_env.sh
```

**Note:** You'll need to run `source ./emsdk_env.sh` in each new terminal session where you want to build with Emscripten.

## Build Instructions

### 1. Initialize Git Submodules

Make sure all dependencies are checked out:

```bash
cd /path/to/usgscsm
git submodule update --init --recursive
```

### 2. Create Build Directory

```bash
mkdir build-wasm
cd build-wasm
```

### 3. Configure with Emscripten

```bash
emcmake cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DUSGSCSM_BUILD_TESTS=OFF \
  -DUSGSCSM_BUILD_DOCS=OFF
```

**Note:** The `emcmake` wrapper automatically sets up the Emscripten toolchain.

### 4. Build

```bash
emmake make -j4
```

This will compile the WebAssembly module. The build may take several minutes.

### 5. Output Files

After a successful build, you'll find the following files in the `dist/` directory:

- **usgscsm.js** (~140 KB) - JavaScript glue code
- **usgscsm.wasm** (~12 MB uncompressed) - WebAssembly binary
- **usgscsm.d.ts** - TypeScript definitions

The `.js` and `.wasm` work together and must be deployed in the same directory.

## Build Options

### Debug Build

For development and debugging:

```bash
emcmake cmake .. -DCMAKE_BUILD_TYPE=Debug
emmake make -j4
```

Debug builds are larger but include symbols and assertions useful for troubleshooting.

### Optimization Levels

The default Release build uses `-O3` optimization. To customize:

```bash
emcmake cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS="-O2"  # Use -O2 instead of -O3
```

## Bundle Size

Measured for a default Release build with STARDS and the embedded proj.db:

| File | Uncompressed | Gzipped |
|------|--------------|---------|
| usgscsm.wasm | ~12 MB | ~3 MB |
| usgscsm.js | ~140 KB | ~34 KB |

Most of the `.wasm` is PROJ plus the embedded proj.db. `-DUSGSCSM_ENABLE_STARDS=OFF`
trims it; a Debug build is substantially larger. Check your own build with
`npm run size`.

**Tip:** Always serve WASM files with gzip compression enabled for production.

## Testing the Build

### Node.js Test

From the repo root, after building:

```bash
npm test
```

This runs `node --test tests/wasm/*.test.mjs`. The suite finds the module in
`build-wasm/dist/`, `wasmbuild/dist/`, `build/dist/`, or `dist/`; set
`USGSCSM_WASM=/path/to/dist/usgscsm.js` for anywhere else.

### Browser Test

WebAssembly cannot be loaded over `file://`, so serve the build directory:

```bash
cd build-wasm && python3 -m http.server 8000
```

Then open a page that imports `./dist/usgscsm.js`; see
[the browser example in the README](../README.md#building-for-webassembly) for a
minimal one.

## Using the WASM Module

### In a Browser (ES6 Modules)

```javascript
import USGSCSM from './dist/usgscsm.js';

async function loadModel() {
  const Module = await USGSCSM();
  const model = new Module.USGSCSMModel();
  
  // Load from ISD JSON
  const isdData = await fetch('model.json').then(r => r.text());
  model.loadFromISD(isdData, 'USGS_ASTRO_FRAME_SENSOR_MODEL');
  
  // Use the model
  const ground = model.imageToGround(100, 200, 0);
  console.log(`Ground: (${ground.x}, ${ground.y}, ${ground.z})`);
}

loadModel();
```

### In Node.js

The module is an ES module, so import it — `require()` will not load it.

```javascript
import fs from 'node:fs';
import USGSCSM from './dist/usgscsm.js';

const Module = await USGSCSM();
const model = new Module.USGSCSMModel();

const isd = fs.readFileSync('model.json', 'utf8');
model.loadFromISD(isd, 'USGS_ASTRO_FRAME_SENSOR_MODEL');

const image = model.groundToImage(10000, 0, 0);
console.log(`Pixel: (${image.line}, ${image.samp})`);
```

## Troubleshooting

### "emcc: command not found"

Make sure you've activated the Emscripten environment:

```bash
source /path/to/emsdk/emsdk_env.sh
```

### Build Fails with "Cannot find CSM headers"

Ensure submodules are initialized:

```bash
git submodule update --init --recursive
```

### WASM Module Won't Load in Browser

1. Make sure you're serving via HTTP/HTTPS, not `file://` protocol
2. Check browser console for CORS errors
3. Verify both .js and .wasm files are in the same directory

### Out of Memory During Build

Reduce parallel jobs:

```bash
emmake make -j2  # Instead of -j4
```

Or increase swap space on your system.

## Supported Sensor Models

The WebAssembly build includes:

✓ **USGS_ASTRO_FRAME_SENSOR_MODEL** - Frame cameras  
✓ **USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL** - Line scanners  
✓ **USGS_ASTRO_PUSH_FRAME_SENSOR_MODEL** - Push frame cameras  
✓ **USGS_ASTRO_SAR_SENSOR_MODEL** - Synthetic Aperture Radar  
✓ **USGS_ASTRO_PROJECTED_SENSOR_MODEL** - Projected images (via embedded PROJ)

**Note:** The projected sensor model uses the PROJ library, which is compiled to
WebAssembly along with its coordinate database (proj.db). The database is
embedded in the module and served from memory via a custom SQLite VFS, so no
external data files or `PROJ_DATA` configuration are needed at runtime. This does
increase the module size (the embedded database is several MB).

## Next Steps

- See [../tests/wasm/](../tests/wasm/) for runnable usage examples
- Read [wasm_usage.md](wasm_usage.md) and the TypeScript definitions in
  [../src/wasm/usgscsm.d.ts](../src/wasm/usgscsm.d.ts) for the API reference
- Check [../README.md](../README.md) for general USGSCSM documentation

## Performance

WebAssembly is slower than the native build — typically by a small integer factor
— but fast enough for interactive use. No benchmark ships with the project, so
measure your own workload rather than relying on a quoted figure; results vary
widely by browser and hardware.

## Browser Compatibility

Tested and supported browsers:

- Chrome 90+
- Firefox 88+
- Safari 14+
- Edge 90+
- Chrome Android 90+
- Safari iOS 14+

Older browsers without WebAssembly support are not supported.
