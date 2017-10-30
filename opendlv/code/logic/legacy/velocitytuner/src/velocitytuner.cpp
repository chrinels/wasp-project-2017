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

#include <iostream>

#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/io/conference/ContainerConference.h>

#include <opendlv/data/environment/Line.h>
#include <opendlv/data/environment/Obstacle.h>
#include <opendlv/data/environment/Polygon.h>
#include <opendlv/data/environment/WGS84Coordinate.h>

#include <automotivedata/GeneratedHeaders_AutomotiveData.h>
#include <odvdopendlvdata/GeneratedHeaders_ODVDOpenDLVData.h>
#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>
#include <odvdimu/GeneratedHeaders_ODVDIMU.h>

#include "velocitytuner.hpp"

namespace opendlv {
namespace logic {
namespace legacy {

VelocityTuner::VelocityTuner(int32_t const &a_argc, char **a_argv)
  : TimeTriggeredConferenceClientModule(a_argc, a_argv, 
      "logic-legacy-velocitytuner"),
  m_stateMutex(),
  m_position(0,0,0),
  m_orientation(0),
  m_velocity(1,0,0),
  m_yawrate(0),
  m_wgs84Reference()
{
}

VelocityTuner::~VelocityTuner()
{
}

void VelocityTuner::setUp()
{
  double const latitude = getKeyValueConfiguration().getValue<double>(
      "global.reference.WGS84.latitude");
  double const longitude = getKeyValueConfiguration().getValue<double>(
      "global.reference.WGS84.longitude");
  m_wgs84Reference = opendlv::data::environment::WGS84Coordinate(latitude,longitude);

}

void VelocityTuner::tearDown()
{
}
    
void VelocityTuner::nextContainer(odcore::data::Container &a_container) 
{
  if (a_container.getDataType() == opendlv::data::environment::WGS84Coordinate::ID()) {
      
  } else if (a_container.getDataType() == opendlv::proxy::GroundSpeedReading::ID()) {

  } else if (a_container.getDataType() == opendlv::proxy::AccelerometerReading::ID()) {

  } else if (a_container.getDataType() == opendlv::proxy::GyroscopeReading::ID()) {
    
  } else if (a_container.getDataType() == opendlv::logic::legacy::TimeSlot::ID()) {

  }
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode VelocityTuner::body()
{
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
    // Calculate what velocity to set


    // Set velocity
    opendlv::logic::legacy::VelocityRequest velocityRequest;
    //velocityRequest.setVelocity(velocity);
    odcore::data::Container initC(velocityRequest);
    getConference().send(initC);
  }

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}


}
}
}
