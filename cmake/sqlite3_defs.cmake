# Compile definitions for every build of the vendored SQLite amalgamation, so the
# static lib, the build-time shell, and the host shell used by cross-compiled
# builds cannot drift apart. PROJ uses the column metadata APIs and opens the DB
# with SQLITE_OPEN_FULLMUTEX.
set(USGSCSM_SQLITE3_DEFS
    SQLITE_ENABLE_COLUMN_METADATA
    SQLITE_THREADSAFE=1
)
