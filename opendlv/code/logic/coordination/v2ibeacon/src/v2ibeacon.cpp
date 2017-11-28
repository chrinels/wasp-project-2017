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

#include <iostream>

#include <opendavinci/odcore/data/Container.h>
#include "opendavinci/odcore/data/TimeStamp.h"

#include <opendavinci/odcore/io/conference/ContainerConference.h>

#include <odvdopendlvdata/GeneratedHeaders_ODVDOpenDLVData.h>
#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>

#include "v2ibeacon.hpp"

namespace opendlv {
namespace logic {
namespace coordination {

V2IBeacon::V2IBeacon(int32_t const &a_argc, char **a_argv)
  : TimeTriggeredConferenceClientModule(a_argc, a_argv,
      "logic-coordination-v2ibeacon"),
      m_initialized(false),
      m_wgs84Reference(),
      m_wgs84Position(),
      m_groundSpeed(0),
      m_plannedTrajectory(),
      m_vehicleID(0),
      m_positionX(0),
      m_positionY(0)
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
  m_plannedTrajectory = "ES"; // TODO: Read fom config?
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
    } 
  }
  if (dataType == opendlv::logic::legacy::StateEstimate::ID()) {
    auto stateEstimate = a_container.getData<opendlv::logic::legacy::StateEstimate>();
    double positionX = stateEstimate.getPositionX();
    double positionY = stateEstimate.getPositionY();
    double velocityX = stateEstimate.getVelocityX();
    double velocityY = stateEstimate.getVelocityY();
    
    m_groundSpeed = sqrt(velocityX*velocityX + velocityY*velocityY);
    m_positionX = positionX;
    m_positionY = positionY;

    opendlv::data::environment::Point3 position(positionX, positionY, 0.0);
    m_wgs84Position = m_wgs84Reference.transform(position);
    
  }
  /**
  if (dataType == opendlv::proxy::GroundSpeedReading::ID()) {
    auto groundSpeed = a_container.getData<opendlv::proxy::GroundSpeedReading>();
    m_groundSpeed = groundSpeed.getGroundSpeed();
    cout << "Ground speed = " << m_groundSpeed << endl;
  }
  */
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode V2IBeacon::body()
{
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
    if (!m_initialized) {
      continue;
    }

    if (m_groundSpeed > 1) {
      opendlv::logic::coordination::IntersectionAccessRequest iar;
      iar.setVehicleID(m_vehicleID);
      iar.setPlannedTrajectory(m_plannedTrajectory);
      iar.setVelocity(m_groundSpeed);
      iar.setPositionX(m_positionX);
      iar.setPositionY(m_positionY);
      cout << "Beaconing information" << endl;
      odcore::data::Container c_intersectionAccess(iar);
      getConference().send(c_intersectionAccess);
    }
  }

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

} // V2IBeacon
} // logic
} // opendlv
