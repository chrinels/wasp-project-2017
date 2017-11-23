/**
 * Copyright (C) 2017 Ola Benderius
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef LOGIC_LEGACY_COORDINATION_V2IBEACON_HPP
#define LOGIC_LEGACY_COORDINATION_V2IBEACON_HPP

#include <ctype.h>
#include <cstring>

#include "opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h"
#include <opendavinci/odcore/base/Mutex.h>
#include <opendlv/data/environment/Point3.h>
#include <opendlv/data/environment/WGS84Coordinate.h>

namespace opendlv {
namespace logic {
namespace coordination {

class V2IBeacon : public odcore::base::module::TimeTriggeredConferenceClientModule {
 public:
  V2IBeacon(int32_t const &, char **);
  V2IBeacon(V2IBeacon const &) = delete;
  V2IBeacon &operator=(V2IBeacon const &) = delete;
  virtual ~V2IBeacon();

  virtual void nextContainer(odcore::data::Container &c);
  odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

 private:
  void setUp();
  void tearDown();

  bool m_initialized;
  opendlv::data::environment::WGS84Coordinate m_wgs84Reference;
  opendlv::data::environment::WGS84Coordinate m_wgs84Position;
  double m_groundSpeed;
  std::string m_plannedTrajectory;
  uint32_t m_vehicleID;
  double m_positionX;
  double m_positionY;
};

}
}
}

#endif
