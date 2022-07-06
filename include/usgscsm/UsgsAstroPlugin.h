/** Copyright  © 2017-2022 BAE Systems Information and Electronic Systems Integration Inc.

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions
and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or other materials provided
with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. **/

#ifndef INCLUDE_USGSCSM_USGSASTROPLUGIN_H_
#define INCLUDE_USGSCSM_USGSASTROPLUGIN_H_

#include <string>
#include<map>
#include<memory>

#include <Plugin.h>
#include <Version.h>

#include <nlohmann/json.hpp>
#include "spdlog/spdlog.h"

class UsgsAstroPlugin : public csm::Plugin {
 public:
  UsgsAstroPlugin();
  ~UsgsAstroPlugin();

  virtual std::string getStateFromISD(csm::Isd imageSupportData) const;
  virtual std::string getPluginName() const;
  virtual std::string getManufacturer() const;
  virtual std::string getReleaseDate() const;
  virtual csm::Version getCsmVersion() const;
  virtual size_t getNumModels() const;
  virtual std::string getModelName(size_t modelIndex) const;
  virtual std::string getModelFamily(size_t modelIndex) const;
  virtual csm::Version getModelVersion(const std::string &modelName) const;
  virtual bool canModelBeConstructedFromState(
      const std::string &modelName, const std::string &modelState,
      csm::WarningList *warnings = NULL) const;
  virtual bool canModelBeConstructedFromISD(
      const csm::Isd &imageSupportData, const std::string &modelName,
      csm::WarningList *warnings = NULL) const;
  virtual csm::Model *constructModelFromState(
      const std::string &modelState, csm::WarningList *warnings = NULL) const;
  virtual csm::Model *constructModelFromISD(
      const csm::Isd &imageSupportData, const std::string &modelName,
      csm::WarningList *warnings = NULL) const;
  virtual std::string getModelNameFromModelState(
      const std::string &modelState, csm::WarningList *warnings = NULL) const;
  virtual bool canISDBeConvertedToModelState(
      const csm::Isd &imageSupportData, const std::string &modelName,
      csm::WarningList *warnings = NULL) const;
  virtual std::string convertISDToModelState(
      const csm::Isd &imageSupportData, const std::string &modelName,
      csm::WarningList *warnings = NULL) const;

  std::string loadImageSupportData(
      const csm::Isd &imageSupportDataOriginal) const;

 private:
  static const UsgsAstroPlugin m_registeredPlugin;
  static const std::string _PLUGIN_NAME;
  static const std::string _MANUFACTURER_NAME;
  static const std::string _RELEASE_DATE;
  static const int _N_SENSOR_MODELS;

  typedef csm::Model *(*sensorConstructor)(void);
  static std::map<std::string, sensorConstructor> MODELS;
  std::shared_ptr<spdlog::logger> m_logger;
};

#endif   // INCLUDE_USGSCSM_USGSASTROPLUGIN_H_
