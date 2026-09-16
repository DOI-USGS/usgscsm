// Async URL/fetch helpers over the Embind bindings. Embind methods are synchronous,
// so the network layer lives here and hands bytes to C++ loadFromBytes(). Sources
// may be "/vsicurl/<url>" or a bare http(s) URL; "/vsis3/" is not resolved here.
Module.onRuntimeInitialized = (function (previous) {
  return function () {
    if (typeof previous === 'function') previous();

    const Model = Module.USGSCSMModel;
    if (!Model || !Model.prototype) return;

    // True for "/vsicurl/<url>" or a bare "http(s)://<url>".
    function isUrlSource(source) {
      return typeof source === 'string' && (
        source.startsWith('/vsicurl/') ||
        source.startsWith('http://') ||
        source.startsWith('https://'));
    }

    // Strip the "/vsicurl/" prefix if present, yielding a plain URL.
    function toUrl(source) {
      return source.startsWith('/vsicurl/') ? source.slice('/vsicurl/'.length)
                                            : source;
    }

    // Returns a Promise<boolean>.
    Model.prototype.loadFromURL = async function (source) {
      if (!isUrlSource(source)) {
        throw new Error(
          'loadFromURL expects a URL or a "/vsicurl/<url>" path, got: ' + source);
      }
      const url = toUrl(source);
      const resp = await fetch(url);
      if (!resp.ok) {
        throw new Error(
          'Failed to fetch ' + url + ': HTTP ' + resp.status + ' ' + resp.statusText);
      }
      const buf = await resp.arrayBuffer();
      return this.loadFromBytes(new Uint8Array(buf));
    };

    // Dispatch on source type; always returns a Promise for a uniform async API.
    Model.prototype.loadFrom = function (source) {
      if (isUrlSource(source)) {
        return this.loadFromURL(source);
      }
      if (source instanceof Uint8Array) {
        return Promise.resolve(this.loadFromBytes(source));
      }
      if (source instanceof ArrayBuffer) {
        return Promise.resolve(this.loadFromBytes(new Uint8Array(source)));
      }
      return Promise.reject(new Error(
        'loadFrom expects a URL string, "/vsicurl/<url>" path, Uint8Array, or ' +
        'ArrayBuffer'));
    };
  };
})(Module.onRuntimeInitialized);
