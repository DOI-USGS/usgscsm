#include "usgscsm/ProjDbVfs.h"

#include <cstring>
#include <mutex>

#include <sqlite3.h>

#include "usgscsm/ProjDbData.h"

// A minimal read-only SQLite3 VFS backed by the embedded proj.db byte array.
// PROJ opens with a plain path and no SQLITE_OPEN_URI, so the filename is
// ignored and usgscsm_proj_db_data is always served. Writes and locks are
// no-ops; OS services are delegated to the default VFS.

namespace {

const char* const kVfsName = "usgscsm_proj_db_mem";

// File handle: a fixed view over the embedded buffer.
struct MemFile {
  sqlite3_file base;               // must be first
  const unsigned char* data;
  sqlite3_int64 size;
};

int memClose(sqlite3_file*) { return SQLITE_OK; }

int memRead(sqlite3_file* pFile, void* zBuf, int iAmt, sqlite3_int64 iOfst) {
  MemFile* p = reinterpret_cast<MemFile*>(pFile);
  if (iOfst < 0 || iOfst > p->size) {
    return SQLITE_IOERR_READ;
  }
  // Serve what is available; zero-fill any short read (SQLite contract).
  sqlite3_int64 avail = p->size - iOfst;
  int n = (avail < iAmt) ? static_cast<int>(avail) : iAmt;
  std::memcpy(zBuf, p->data + iOfst, static_cast<size_t>(n));
  if (n < iAmt) {
    std::memset(static_cast<unsigned char*>(zBuf) + n, 0,
                static_cast<size_t>(iAmt - n));
    return SQLITE_IOERR_SHORT_READ;
  }
  return SQLITE_OK;
}

int memWrite(sqlite3_file*, const void*, int, sqlite3_int64) {
  return SQLITE_READONLY;
}

int memTruncate(sqlite3_file*, sqlite3_int64) { return SQLITE_READONLY; }

int memSync(sqlite3_file*, int) { return SQLITE_OK; }

int memFileSize(sqlite3_file* pFile, sqlite3_int64* pSize) {
  *pSize = reinterpret_cast<MemFile*>(pFile)->size;
  return SQLITE_OK;
}

int memLock(sqlite3_file*, int) { return SQLITE_OK; }
int memUnlock(sqlite3_file*, int) { return SQLITE_OK; }

int memCheckReservedLock(sqlite3_file*, int* pResOut) {
  *pResOut = 0;
  return SQLITE_OK;
}

int memFileControl(sqlite3_file*, int, void*) { return SQLITE_NOTFOUND; }

int memSectorSize(sqlite3_file*) { return 1024; }

int memDeviceCharacteristics(sqlite3_file*) {
  return SQLITE_IOCAP_IMMUTABLE;
}

const sqlite3_io_methods mem_io_methods = {
    1,                          // iVersion (no shm / mmap methods)
    memClose,
    memRead,
    memWrite,
    memTruncate,
    memSync,
    memFileSize,
    memLock,
    memUnlock,
    memCheckReservedLock,
    memFileControl,
    memSectorSize,
    memDeviceCharacteristics,
};

int memOpen(sqlite3_vfs*, const char*, sqlite3_file* pFile, int flags,
            int* pOutFlags) {
  MemFile* p = reinterpret_cast<MemFile*>(pFile);
  std::memset(p, 0, sizeof(*p));
  p->base.pMethods = &mem_io_methods;
  p->data = usgscsm_proj_db_data;
  p->size = static_cast<sqlite3_int64>(usgscsm_proj_db_size);
  if (pOutFlags) {
    *pOutFlags = flags;
  }
  return SQLITE_OK;
}

// The embedded DB is the only file; nothing else exists to delete or access.
int memDelete(sqlite3_vfs*, const char*, int) { return SQLITE_OK; }

int memAccess(sqlite3_vfs*, const char*, int flags, int* pResOut) {
  // Report the (single, read-only) database as existing but not writable.
  *pResOut = (flags == SQLITE_ACCESS_READWRITE) ? 0 : 1;
  return SQLITE_OK;
}

int memFullPathname(sqlite3_vfs*, const char* zPath, int nOut, char* zOut) {
  // Paths are meaningless here; echo the input back.
  sqlite3_snprintf(nOut, zOut, "%s", zPath);
  return SQLITE_OK;
}

// Delegate the OS-service methods to the default VFS captured at registration.
sqlite3_vfs* g_defaultVfs = nullptr;

int memRandomness(sqlite3_vfs*, int nByte, char* zOut) {
  return g_defaultVfs->xRandomness(g_defaultVfs, nByte, zOut);
}
int memSleep(sqlite3_vfs*, int microseconds) {
  return g_defaultVfs->xSleep(g_defaultVfs, microseconds);
}
int memCurrentTime(sqlite3_vfs*, double* pTime) {
  return g_defaultVfs->xCurrentTime(g_defaultVfs, pTime);
}
int memGetLastError(sqlite3_vfs*, int n, char* z) {
  return g_defaultVfs->xGetLastError(g_defaultVfs, n, z);
}
int memCurrentTimeInt64(sqlite3_vfs*, sqlite3_int64* pTime) {
  if (g_defaultVfs->iVersion >= 2 && g_defaultVfs->xCurrentTimeInt64) {
    return g_defaultVfs->xCurrentTimeInt64(g_defaultVfs, pTime);
  }
  double t = 0.0;
  g_defaultVfs->xCurrentTime(g_defaultVfs, &t);
  *pTime = static_cast<sqlite3_int64>(t * 86400000.0);
  return SQLITE_OK;
}

}  // namespace

namespace usgscsm {

const char* ensureProjDbVfsRegistered() {
  static bool registered = false;
  static std::once_flag once;
  std::call_once(once, []() {
    g_defaultVfs = sqlite3_vfs_find(nullptr);
    if (!g_defaultVfs) {
      return;
    }

    static sqlite3_vfs mem_vfs;
    std::memset(&mem_vfs, 0, sizeof(mem_vfs));
    mem_vfs.iVersion = 2;
    mem_vfs.szOsFile = sizeof(MemFile);
    mem_vfs.mxPathname = 1024;
    mem_vfs.zName = kVfsName;
    mem_vfs.xOpen = memOpen;
    mem_vfs.xDelete = memDelete;
    mem_vfs.xAccess = memAccess;
    mem_vfs.xFullPathname = memFullPathname;
    mem_vfs.xRandomness = memRandomness;
    mem_vfs.xSleep = memSleep;
    mem_vfs.xCurrentTime = memCurrentTime;
    mem_vfs.xGetLastError = memGetLastError;
    mem_vfs.xCurrentTimeInt64 = memCurrentTimeInt64;

    // Register but do not make default: PROJ selects it by name.
    registered = sqlite3_vfs_register(&mem_vfs, 0) == SQLITE_OK;
  });
  return registered ? kVfsName : nullptr;
}

}  // namespace usgscsm
