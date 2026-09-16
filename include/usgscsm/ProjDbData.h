#ifndef INCLUDE_USGSCSM_PROJDBDATA_H_
#define INCLUDE_USGSCSM_PROJDBDATA_H_

// proj.db as a byte array, generated at build time by cmake/embed_proj_db.cmake
// and served from memory by ProjDbVfs.h.

extern "C" const unsigned char usgscsm_proj_db_data[];
extern "C" const unsigned long long usgscsm_proj_db_size;

#endif  // INCLUDE_USGSCSM_PROJDBDATA_H_
