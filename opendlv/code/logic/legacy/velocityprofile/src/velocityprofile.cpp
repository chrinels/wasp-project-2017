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

#include "velocityprofile.hpp"

namespace opendlv {
namespace logic {
namespace legacy {

VelocityProfile::VelocityProfile(int32_t const &a_argc, char **a_argv)
  : TimeTriggeredConferenceClientModule(a_argc, a_argv,
      "logic-legacy-velocityprofile"),
  m_testStartTime(),
  m_testEndTime(),
  m_velocitykmh()
{
}

VelocityProfile::~VelocityProfile()
{
}

void VelocityProfile::setUp()
{
  // double const latitude = getKeyValueConfiguration().getValue<double>(
  //     "global.reference.WGS84.latitude");
  // double const longitude = getKeyValueConfiguration().getValue<double>(
  //     "global.reference.WGS84.longitude");
  // m_wgs84Reference = opendlv::data::environment::WGS84Coordinate(latitude,longitude);

  double const testTimeDelay = getKeyValueConfiguration().getValue<double>(
      "logic-legacy-velocityprofile.test-time-delay");
  double const testTimeDuration = getKeyValueConfiguration().getValue<double>(
      "logic-legacy-velocityprofile.test-time-duration");

  odcore::data::TimeStamp currentTime;
  m_testStartTime = currentTime + odcore::data::TimeStamp(testTimeDelay,0);
  m_testEndTime = currentTime + odcore::data::TimeStamp(testTimeDelay+testTimeDuration,0);

  m_velocitykmh = getKeyValueConfiguration().getValue<double>(
      "logic-legacy-velocityprofile.velocitykmh");
}

void VelocityProfile::tearDown()
{
}

// void VelocityProfile::nextContainer(odcore::data::Container &a_container)
// {
//
// }

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode VelocityProfile::body()
{
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
    odcore::data::TimeStamp currentTime;
    double velocity = 0.0;
    if (currentTime > m_testStartTime && currentTime < m_testEndTime) {
      velocity = m_velocitykmh/3.6;
    }

    // Set velocity
    opendlv::logic::legacy::VelocityRequest velocityRequest;
    velocityRequest.setVelocity(velocity);
    odcore::data::Container initC(velocityRequest);
    getConference().send(initC);
  }

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}


}
}
}
