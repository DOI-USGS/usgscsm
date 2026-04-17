#ifndef INCLUDE_USGSCSM_USGSASTROPLUGINSUPPORT_H_
#define INCLUDE_USGSCSM_USGSASTROPLUGINSUPPORT_H_

#include <RasterGM.h>
#include <csm/Plugin.h>
#include "VariantMap.h"

csm::RasterGM *getUsgsCsmModelFromIsd(const std::string &stringIsd, const std::string &modelName, csm::WarningList *warnings);
csm::RasterGM *getUsgsCsmModelFromJsonState(const std::string &jstring, const std::string &modelName, csm::WarningList *warnings);
std::string getUsgsCsmModelJson(csm::RasterGM *model);
VariantMap getUsgsCsmModelMap(csm::RasterGM *model);

#endif // INCLUDE_USGSCSM_USGSASTROPLUGINSUPPORT_H_
