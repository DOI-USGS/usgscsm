#ifndef INCLUDE_USGSCSM_PROJDBVFS_H_
#define INCLUDE_USGSCSM_PROJDBVFS_H_

// A read-only SQLite3 VFS serving the embedded proj.db (see ProjDbData.h) from
// memory, so the plugin needs no loose file on disk. sqlite's own memvfs.c is
// unusable here: it takes the buffer address as URI parameters in the filename,
// but PROJ opens with a plain path and no SQLITE_OPEN_URI.

namespace usgscsm {

// Idempotent and thread-safe. Returns the name to pass to
// proj_context_set_sqlite3_vfs_name(), or nullptr if registration failed, in
// which case the caller must leave PROJ on its default (on-disk) VFS.
const char* ensureProjDbVfsRegistered();

}  // namespace usgscsm

#endif  // INCLUDE_USGSCSM_PROJDBVFS_H_
