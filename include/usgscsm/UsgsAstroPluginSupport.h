#ifndef INCLUDE_USGSCSM_USGSASTROPLUGINSUPPORT_H_
#define INCLUDE_USGSCSM_USGSASTROPLUGINSUPPORT_H_

#include <RasterGM.h>
#include <csm/Plugin.h>
#include <nlohmann/json_fwd.hpp> // forward declaration

csm::RasterGM *getUsgsCsmModelFromIsd(const std::string &stringIsd, const std::string &modelName, csm::WarningList *warnings);
csm::RasterGM *getUsgsCsmModelFromState(const std::string &stringState, const std::string &modelName, csm::WarningList *warnings);
csm::RasterGM *getUsgsCsmModelFromJson(const nlohmann::json &j, const std::string &modelName, csm::WarningList *warnings);
nlohmann::json getUsgsCsmModelJson(csm::RasterGM *model);

#endif // INCLUDE_USGSCSM_USGSASTROPLUGINSUPPORT_H_
