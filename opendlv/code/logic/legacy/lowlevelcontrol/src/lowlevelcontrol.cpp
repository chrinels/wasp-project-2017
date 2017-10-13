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
#include <odvdvehicle/generated/opendlv/proxy/ActuationRequest.h>
#include <odvdimu/GeneratedHeaders_ODVDIMU.h>

#include "lowlevelcontrol.hpp"

namespace opendlv {
namespace logic {
namespace legacy {

LowLevelControl::LowLevelControl(int32_t const &a_argc, char **a_argv)
  : TimeTriggeredConferenceClientModule(a_argc, a_argv, 
      "logic-legacy-lowlevelcontrol"),
  m_stateMutex(),
  m_position(0,0,0),
  m_orientation(0),
  m_velocity(1,0,0),
  m_yawrate(0),
  m_inputMutex(),
  m_inputAcceleration(),
  m_inputSteeringWheelAngle(),
  m_wgs84Reference()
{
}

LowLevelControl::~LowLevelControl()
{
}

void LowLevelControl::setUp()
{
  double const latitude = getKeyValueConfiguration().getValue<double>(
      "global.reference.WGS84.latitude");
  double const longitude = getKeyValueConfiguration().getValue<double>(
      "global.reference.WGS84.longitude");
  m_wgs84Reference = opendlv::data::environment::WGS84Coordinate(latitude,longitude);

}

void LowLevelControl::tearDown()
{
}
    
void LowLevelControl::nextContainer(odcore::data::Container &a_container) 
{
  if (a_container.getDataType() == opendlv::data::environment::WGS84Coordinate::ID()) {
      
  } else if (a_container.getDataType() == opendlv::proxy::GroundSpeedReading::ID()) {

  } 
  // else if (a_container.getDataType() == opendlv::proxy::AccelerometerReading::ID()) {

  // } else if (a_container.getDataType() == opendlv::proxy::GyroscopeReading::ID()) {
    
  // }
  // VelocityRequestReading
  // Read path
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode LowLevelControl::body()
{
  // Send some zeros first to fix acceleration.
  opendlv::proxy::ActuationRequest initAr;
  initAr.setAcceleration(0.0);
  initAr.setSteering(0.0);
  initAr.setIsValid(true);
  odcore::data::Container initC(initAr);
  getConference().send(initC);

  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
    

    {
      // odcore::base::Lock li(m_inputMutex);
      // odcore::base::Lock ls(m_stateMutex);
      // double const dt = 1.0 / static_cast<double>(getFrequency());

      // double const g = 9.82;
      
      // // chassis params
      // double const m = 2100;
      // double const lf = 1.3;
      // double const lr = 1.5;
      // // double const w = 0.8;
      // double const Izz = 3900;
      // double const k = 1.0/45.0; //Steering ratio

      // // Pacejka Tire Parameters
      // double const muxf = 1.2;
      // double const muxr = 1.2;
      // // double const Bxf = 11.7;
      // // double const Bxr = 11.1;
      // // double const Cxf = 1.69;
      // // double const Cxr = 1.69;
      // // double const Exf = 0.377;
      // // double const Exr = 0.362;
      // double const muyf = 0.935;
      // double const muyr = 0.961;
      // double const Byf = 8.86;
      // double const Byr = 9.3;
      // double const Cyf = 1.19;
      // double const Cyr = 1.19;
      // double const Eyf = -1.21;
      // double const Eyr = -1.11;

      // // Vertical force
      // double const Fzf = lr/(lf+lr)*m*g;
      // double const Fzr = lf/(lf+lr)*m*g;


      double accelerationRequest = 0;
      double steeringWheelAngleRequest = 0;

      // Send ActuationRequest
      opendlv::proxy::ActuationRequest ar;
      ar.setAcceleration(static_cast<float>(accelerationRequest));
      ar.setSteering(static_cast<float>(steeringWheelAngleRequest));
      ar.setIsValid(true);
      odcore::data::Container c = odcore::data::Container(ar);
      getConference().send(c);
    }

  }

  // Stop vehicle.
  opendlv::proxy::ActuationRequest ar;
  ar.setAcceleration(-2.0);
  ar.setSteering(0.0);
  ar.setIsValid(true);
  odcore::data::Container c(ar);
  getConference().send(c);

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}


}
}
}
