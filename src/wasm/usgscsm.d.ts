/**
 * TypeScript definitions for USGSCSM WebAssembly module
 *
 * @module usgscsm-wasm
 * @version 2.0.1
 */

/**
 * ECEF (Earth-Centered Earth-Fixed) coordinate in 3D space
 */
export interface EcefCoord {
  /** X coordinate in meters */
  x: number;
  /** Y coordinate in meters */
  y: number;
  /** Z coordinate in meters */
  z: number;
}

/**
 * Image coordinate (pixel location)
 */
export interface ImageCoord {
  /** Line (row) coordinate, 0-indexed */
  line: number;
  /** Sample (column) coordinate, 0-indexed */
  sample: number;
}

/**
 * Image dimensions
 */
export interface ImageSize {
  /** Number of lines (rows) */
  lines: number;
  /** Number of samples (columns) */
  samples: number;
}

/**
 * Supported sensor model types
 */
export type ModelName =
  | 'USGS_ASTRO_FRAME_SENSOR_MODEL'
  | 'USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL'
  | 'USGS_ASTRO_PUSH_FRAME_SENSOR_MODEL'
  | 'USGS_ASTRO_SAR_SENSOR_MODEL';

/**
 * USGSCSM sensor model wrapper class
 *
 * Provides access to Community Sensor Model (CSM) operations for planetary
 * camera models including coordinate transformations, sensor position/velocity
 * queries, and model state management.
 *
 * @example
 * ```typescript
 * import USGSCSM from 'usgscsm-wasm';
 *
 * const Module = await USGSCSM();
 * const model = new Module.USGSCSMModel();
 *
 * // Load from ISD JSON
 * const isdJson = await fetch('model.json').then(r => r.text());
 * model.loadFromISD(isdJson, 'USGS_ASTRO_FRAME_SENSOR_MODEL');
 *
 * // Convert image to ground coordinates
 * const ground = model.imageToGround(100, 200, 0);
 * console.log(`ECEF: (${ground.x}, ${ground.y}, ${ground.z})`);
 * ```
 */
export class USGSCSMModel {
  /**
   * Creates a new sensor model instance
   */
  constructor();

  /**
   * Load a sensor model from ISD (Image Support Data) JSON
   *
   * @param isdJson - JSON string containing ISD data
   * @param modelName - Sensor model type to instantiate
   * @returns true if model loaded successfully, false otherwise
   *
   * @example
   * ```typescript
   * const success = model.loadFromISD(isdJson, 'USGS_ASTRO_FRAME_SENSOR_MODEL');
   * if (!success) {
   *   console.error('Failed to load model');
   * }
   * ```
   */
  loadFromISD(isdJson: string, modelName: ModelName): boolean;

  /**
   * Load a sensor model from model state JSON
   *
   * Model state is an optimized representation that can be loaded faster
   * than ISD and supports model transformations.
   *
   * @param stateJson - JSON string containing model state
   * @returns true if model loaded successfully, false otherwise
   *
   * @example
   * ```typescript
   * const state = existingModel.getModelState();
   * const newModel = new Module.USGSCSMModel();
   * newModel.loadFromState(state);
   * ```
   */
  loadFromState(stateJson: string): boolean;

  /**
   * Get the current model state as JSON string
   *
   * Model state can be saved and reloaded later for faster initialization.
   *
   * @returns JSON string containing model state, or empty string if no model loaded
   *
   * @example
   * ```typescript
   * const state = model.getModelState();
   * localStorage.setItem('cameraModel', state);
   * ```
   */
  getModelState(): string;

  /**
   * Convert image coordinates to ground coordinates
   *
   * Projects a ray from the camera through the given pixel and intersects
   * it with the reference ellipsoid at the specified height.
   *
   * @param line - Image line coordinate (row), 0-indexed
   * @param sample - Image sample coordinate (column), 0-indexed
   * @param height - Height above reference ellipsoid in meters
   * @returns ECEF ground coordinates, or null if no model loaded
   *
   * @example
   * ```typescript
   * const ground = model.imageToGround(512, 1024, 0);
   * if (ground) {
   *   console.log(`Latitude/Longitude at: (${ground.x}, ${ground.y}, ${ground.z})`);
   * }
   * ```
   */
  imageToGround(line: number, sample: number, height: number): EcefCoord | null;

  /**
   * Convert ground coordinates to image coordinates
   *
   * Finds the pixel location that images the given ground point.
   *
   * @param x - ECEF X coordinate in meters
   * @param y - ECEF Y coordinate in meters
   * @param z - ECEF Z coordinate in meters
   * @returns Image pixel coordinates, or null if no model loaded or point not visible
   *
   * @example
   * ```typescript
   * const pixel = model.groundToImage(1234567, 2345678, 3456789);
   * if (pixel) {
   *   console.log(`Pixel location: (${pixel.line}, ${pixel.sample})`);
   * }
   * ```
   */
  groundToImage(x: number, y: number, z: number): ImageCoord | null;

  /**
   * Get sensor position at a given image coordinate
   *
   * Returns the camera position in ECEF coordinates for the given pixel.
   * For frame cameras, this is constant across the image. For line scanners,
   * it varies with each line.
   *
   * @param line - Image line coordinate
   * @param sample - Image sample coordinate
   * @returns ECEF sensor position, or null if no model loaded
   *
   * @example
   * ```typescript
   * const position = model.getSensorPosition(512, 1024);
   * if (position) {
   *   console.log(`Camera at: (${position.x}, ${position.y}, ${position.z})`);
   * }
   * ```
   */
  getSensorPosition(line: number, sample: number): EcefCoord | null;

  /**
   * Get sensor velocity at a given image coordinate
   *
   * Returns the camera velocity vector in ECEF coordinates. For orbiting
   * sensors, this represents the spacecraft velocity.
   *
   * @param line - Image line coordinate
   * @param sample - Image sample coordinate
   * @returns ECEF velocity vector in m/s, or null if no model loaded
   *
   * @example
   * ```typescript
   * const velocity = model.getSensorVelocity(512, 1024);
   * if (velocity) {
   *   const speed = Math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2);
   *   console.log(`Spacecraft speed: ${speed} m/s`);
   * }
   * ```
   */
  getSensorVelocity(line: number, sample: number): EcefCoord | null;

  /**
   * Get illumination direction (sun vector) for a ground point
   *
   * Returns a unit vector pointing from the ground point toward the sun.
   *
   * @param x - ECEF X coordinate of ground point in meters
   * @param y - ECEF Y coordinate of ground point in meters
   * @param z - ECEF Z coordinate of ground point in meters
   * @returns Unit vector from ground to sun, or null if no model loaded
   *
   * @example
   * ```typescript
   * const sunVec = model.getIlluminationDirection(x, y, z);
   * if (sunVec) {
   *   // Calculate incidence angle, etc.
   * }
   * ```
   */
  getIlluminationDirection(x: number, y: number, z: number): EcefCoord | null;

  /**
   * Get image dimensions
   *
   * @returns Image size in lines and samples, or null if no model loaded
   *
   * @example
   * ```typescript
   * const size = model.getImageSize();
   * if (size) {
   *   console.log(`Image is ${size.lines} x ${size.samples} pixels`);
   * }
   * ```
   */
  getImageSize(): ImageSize | null;

  /**
   * Get image start coordinates
   *
   * Most images start at (0, 0), but some may have different origins.
   *
   * @returns Starting line/sample coordinates, or null if no model loaded
   */
  getImageStart(): ImageCoord | null;

  /**
   * Get the sensor model name
   *
   * @returns Model name (e.g., "USGS_ASTRO_FRAME_SENSOR_MODEL"), or empty string if no model loaded
   */
  getModelName(): string;

  /**
   * Get the image identifier
   *
   * @returns Image ID string from the ISD/model state, or empty string if no model loaded
   */
  getImageIdentifier(): string;

  /**
   * Get the sensor identifier
   *
   * @returns Sensor ID string, or empty string if no model loaded
   */
  getSensorIdentifier(): string;

  /**
   * Get the platform identifier
   *
   * @returns Platform/spacecraft ID string, or empty string if no model loaded
   */
  getPlatformIdentifier(): string;

  /**
   * Check if a model is currently loaded
   *
   * @returns true if a model has been successfully loaded
   *
   * @example
   * ```typescript
   * if (!model.isLoaded()) {
   *   console.error('No model loaded');
   * }
   * ```
   */
  isLoaded(): boolean;
}

/**
 * USGSCSM WebAssembly module interface
 */
export interface USGSCSMModule {
  /**
   * USGSCSMModel class constructor
   */
  USGSCSMModel: typeof USGSCSMModel;

  /**
   * Emscripten virtual filesystem API
   *
   * Allows reading/writing files in the virtual filesystem.
   * Used for loading ISD files or saving model state.
   *
   * @see https://emscripten.org/docs/api_reference/Filesystem-API.html
   */
  FS: {
    /**
     * Write a file to the virtual filesystem
     *
     * @param path - File path (e.g., "/tmp/model.json")
     * @param data - File contents (string or Uint8Array)
     */
    writeFile(path: string, data: string | Uint8Array): void;

    /**
     * Read a file from the virtual filesystem
     *
     * @param path - File path
     * @param opts - Options (e.g., { encoding: 'utf8' })
     * @returns File contents
     */
    readFile(path: string, opts?: { encoding?: string }): string | Uint8Array;

    /**
     * Check if a file or directory exists
     *
     * @param path - File/directory path
     * @returns Analysis object with exists property
     */
    analyzePath(path: string): { exists: boolean; isDirectory: boolean };

    /**
     * Delete a file
     *
     * @param path - File path
     */
    unlink(path: string): void;

    /**
     * Create a directory
     *
     * @param path - Directory path
     */
    mkdir(path: string): void;
  };
}

/**
 * Load the USGSCSM WebAssembly module
 *
 * @returns Promise that resolves to the initialized module
 *
 * @example
 * ```typescript
 * import USGSCSM from 'usgscsm-wasm';
 *
 * const Module = await USGSCSM();
 * const model = new Module.USGSCSMModel();
 * ```
 */
export default function USGSCSM(): Promise<USGSCSMModule>;
