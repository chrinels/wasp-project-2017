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

#include <cmath>
#include <cstdio>
#include <ctype.h>
#include <cstring>
#include <algorithm>

#include <iostream>

#include <opendavinci/odcore/data/Container.h>
#include "opendavinci/odcore/data/TimeStamp.h"

#include <opendavinci/odcore/io/conference/ContainerConference.h>

#include <opendlv/data/environment/Line.h>
#include <opendlv/data/environment/Obstacle.h>
#include <opendlv/data/environment/Polygon.h>
#include <opendlv/data/environment/WGS84Coordinate.h>

#include <automotivedata/GeneratedHeaders_AutomotiveData.h>
#include <odvdopendlvdata/GeneratedHeaders_ODVDOpenDLVData.h>
#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>
#include <odvdimu/GeneratedHeaders_ODVDIMU.h>

#include "v2ibeacon.hpp"

namespace opendlv {
namespace logic {
namespace coordination {

V2IBeacon::V2IBeacon(int32_t const &a_argc, char **a_argv)
  : TimeTriggeredConferenceClientModule(a_argc, a_argv,
      "logic-coordination-v2ibeacon"),
      m_initialized(false),
      m_wgs84Reference(),
      m_groundSpeed(0),
      m_plannedTrajectory(),
      m_vehicleID(0)
{
}

V2IBeacon::~V2IBeacon()
{
}

void V2IBeacon::setUp()
{
  double const latitude = getKeyValueConfiguration().getValue<double>(
      "global.reference.WGS84.latitude");
  double const longitude = getKeyValueConfiguration().getValue<double>(
      "global.reference.WGS84.longitude");
  m_wgs84Reference = opendlv::data::environment::WGS84Coordinate(latitude,longitude);
  m_initialized = true;
}

void V2IBeacon::tearDown()
{
}

void V2IBeacon::nextContainer(odcore::data::Container &a_container)
{
  if(!m_initialized) {
    return;
  }
  auto dataType = a_container.getDataType();
  if (dataType == opendlv::knowledge::Insight::ID()) {
    auto insight = a_container.getData<opendlv::knowledge::Insight>();

    std::string insightString = insight.getInsight();
    if(insightString.find("stationId") != std::string::npos) {
      std::size_t found = insightString.find_first_of("0123456789");
      m_vehicleID = std::stoi(insightString.substr(found));
      cout << "ID = " << m_vehicleID << endl;
    } 
  }
  if (dataType == opendlv::proxy::GroundSpeedReading::ID()) {
    auto groundSpeed = a_container.getData<opendlv::proxy::GroundSpeedReading>();
    m_groundSpeed = groundSpeed.getGroundSpeed();
    cout << "Ground speed = " << m_groundSpeed << endl;
  } 
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode V2IBeacon::body()
{
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
    if (!m_initialized) {
      continue;
    }
    odcore::data::TimeStamp now;
    opendlv::logic::coordination::IntersectionAccessRequest iar;
    iar.setVehicleID(m_vehicleID);
    iar.setTimeAtRequest(now);
    iar.setPlannedTrajectory("ES");
    iar.setVelocity(m_groundSpeed);
    iar.setLatitude(55.234);
    iar.setLongitude(27.1234);
    odcore::data::Container c_intersectionAccess(iar);
    getConference().send(c_intersectionAccess);
  }

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

} // V2IBeacon
} // logic
} // opendlv
