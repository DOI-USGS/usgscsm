#ifndef INCLUDE_USGSCSM_USGSASTROPLUGINSUPPORT_H_
#define INCLUDE_USGSCSM_USGSASTROPLUGINSUPPORT_H_

#include <RasterGM.h>
#include <csm/Plugin.h>

csm::RasterGM *getUsgsCsmModelFromIsd(const std::string &stringIsd, const std::string &modelName, csm::WarningList *warnings);
csm::RasterGM *getUsgsCsmModelFromState(const std::string &stringState, const std::string &modelName, csm::WarningList *warnings);

#endif // INCLUDE_USGSCSM_USGSASTROPLUGINSUPPORT_H_
