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
// #include <odvdvehicle/generated/opendlv/proxy/StateEstimate.h>
#include <odvdimu/GeneratedHeaders_ODVDIMU.h>

#include "stateestimator.hpp"

namespace opendlv {
namespace logic {
namespace legacy {

StateEstimator::StateEstimator(int32_t const &a_argc, char **a_argv)
  : TimeTriggeredConferenceClientModule(a_argc, a_argv,
      "logic-legacy-stateestimator"),
  m_stateMutex(),
  m_position(0,0,0),
  m_velocity(0,0,0),
  m_orientation(0),
  m_yawRate(0),
  m_wgs84Reference(),
  m_gpsReadingTimeStamp(),
  m_positionSmoothing(),
  m_groundSpeedReadingTimeStamp(),
  m_velocitySmoothing()
{
}

StateEstimator::~StateEstimator()
{
}

void StateEstimator::setUp()
{
  double const latitude = getKeyValueConfiguration().getValue<double>(
      "global.reference.WGS84.latitude");
  double const longitude = getKeyValueConfiguration().getValue<double>(
      "global.reference.WGS84.longitude");
  m_wgs84Reference = opendlv::data::environment::WGS84Coordinate(latitude,longitude);

  m_positionSmoothing = getKeyValueConfiguration().getValue<double>(
      "logic-legacy-stateestimator.position.smoothing");

  m_velocitySmoothing = getKeyValueConfiguration().getValue<double>(
      "logic-legacy-stateestimator.velocity.smoothing");
}

void StateEstimator::tearDown()
{
}

void StateEstimator::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == opendlv::data::environment::WGS84Coordinate::ID()) {
    odcore::base::Lock l(m_stateMutex);
    auto gpsReading = a_container.getData<opendlv::data::environment::WGS84Coordinate>();

    auto position_i = m_wgs84Reference.transform(gpsReading);
    odcore::data::TimeStamp currentTime;

    auto dt = (currentTime - m_gpsReadingTimeStamp).toMicroseconds()*1.0/1000000L;
    auto smoothing_factor = fmin(dt/m_positionSmoothing,1.0);

    m_position += (position_i - m_position)*smoothing_factor;
    m_gpsReadingTimeStamp = currentTime;

  } else if (a_container.getDataType() == opendlv::proxy::GroundSpeedReading::ID()) {
    odcore::base::Lock l(m_stateMutex);
    auto groundSpeedReading = a_container.getData<opendlv::proxy::GroundSpeedReading>();

    auto vx_i = groundSpeedReading.getGroundSpeed();
    auto vx = m_velocity.getX();
    odcore::data::TimeStamp currentTime;

    auto dt = (currentTime - m_groundSpeedReadingTimeStamp).toMicroseconds()*1.0/1000000L;
    auto smoothing_factor = fmin(dt/m_velocitySmoothing,1.0);
    vx += smoothing_factor*(vx_i-vx);

    m_velocity.setX(vx);
    m_groundSpeedReadingTimeStamp = currentTime;

  } else if (a_container.getDataType() == opendlv::proxy::AccelerometerReading::ID()) {

  } else if (a_container.getDataType() == opendlv::proxy::GyroscopeReading::ID()) {

  }

  // Read path
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode StateEstimator::body()
{
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
        odcore::base::Lock l(m_stateMutex);

      // Predict


      // Print current state
      std::cout << "position: (" << std::to_string(m_position.getX()) << ", " << std::to_string(m_position.getY()) << ")" << std::endl;
      std::cout << "velocity: (" << std::to_string(m_velocity.getX()) << ", " << std::to_string(m_velocity.getY()) << ")" << std::endl;
      std::cout << "orientation: " << std::to_string(m_orientation) << std::endl;
      std::cout << "yaw rate: " << std::to_string(m_yawRate) << std::endl;

      // Send StateEstimate
      opendlv::logic::legacy::StateEstimate se;
      se.setPositionX(m_position.getX());
      se.setPositionY(m_position.getY());
      se.setVelocityX(m_velocity.getX());
      se.setVelocityY(m_velocity.getY());
      se.setOrientation(m_orientation);
      se.setYawRate(m_yawRate);
      odcore::data::Container c = odcore::data::Container(se);
      getConference().send(c);
  }

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}


}
}
}
