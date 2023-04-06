#ifndef INCLUDE_USGSCSM_USGSASTROPLUGINSUPPORT_H_
#define INCLUDE_USGSCSM_USGSASTROPLUGINSUPPORT_H_

#include <RasterGM.h>
#include <csm/Plugin.h>

csm::RasterGM *getUsgsCsmModel(const std::string &stringIsd, const std::string &modelName, csm::WarningList *warnings);

#endif // INCLUDE_USGSCSM_USGSASTROPLUGINSUPPORT_H_
