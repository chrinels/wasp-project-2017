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

#include "vehiclesim.hpp"

namespace opendlv {
namespace logic {
namespace legacy {

VehicleSim::VehicleSim(int32_t const &a_argc, char **a_argv)
  : TimeTriggeredConferenceClientModule(a_argc, a_argv, 
      "sim-vehiclesim"),
  m_stateMutex(),
  m_position(0,0,0),
  m_orientation(4.05),
  m_velocity(5,0,0),
  m_yawrate(0),
  m_inputMutex(),
  m_inputAcceleration(),
  m_inputSteeringWheelAngle(),
  m_wgs84Reference()
{
}

VehicleSim::~VehicleSim()
{
}

void VehicleSim::setUp()
{
  double const latitude = getKeyValueConfiguration().getValue<double>(
      "global.reference.WGS84.latitude");
  double const longitude = getKeyValueConfiguration().getValue<double>(
      "global.reference.WGS84.longitude");
  m_wgs84Reference = opendlv::data::environment::WGS84Coordinate(latitude,longitude);

}

void VehicleSim::tearDown()
{
}
    
void VehicleSim::nextContainer(odcore::data::Container &a_container) 
{
  if (a_container.getDataType() == opendlv::proxy::ActuationRequest::ID()) {
      auto actuationRequest = a_container.getData<opendlv::proxy::ActuationRequest>();
      if (actuationRequest.getIsValid()) {
        odcore::base::Lock l(m_inputMutex);
        m_inputAcceleration = actuationRequest.getAcceleration();
        m_inputSteeringWheelAngle = actuationRequest.getSteering();
      }
    } 
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode VehicleSim::body()
{

  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
    

    {
      odcore::base::Lock li(m_inputMutex);
      odcore::base::Lock ls(m_stateMutex);
      double const dt = 1.0 / static_cast<double>(getFrequency());

      double const g = 9.82;
      
      // chassis params
      double const m = 2100;
      double const lf = 1.3;
      double const lr = 1.5;
      // double const w = 0.8;
      double const Izz = 3900;
      double const k = 1.0/45.0; //Steering ratio

      // Pacejka Tire Parameters
      double const muxf = 1.2;
      double const muxr = 1.2;
      // double const Bxf = 11.7;
      // double const Bxr = 11.1;
      // double const Cxf = 1.69;
      // double const Cxr = 1.69;
      // double const Exf = 0.377;
      // double const Exr = 0.362;
      double const muyf = 0.935;
      double const muyr = 0.961;
      double const Byf = 8.86;
      double const Byr = 9.3;
      double const Cyf = 1.19;
      double const Cyr = 1.19;
      double const Eyf = -1.21;
      double const Eyr = -1.11;

      // Vertical force
      double const Fzf = lr/(lf+lr)*m*g;
      double const Fzr = lf/(lf+lr)*m*g;

      // Tire slip
      auto delta = k*m_inputSteeringWheelAngle;
      auto vx = m_velocity.getX();
      auto vy = m_velocity.getY();

      auto vxf = cos(delta)*vx + sin(delta)*(vy+lf*m_yawrate);
      auto vyf = -sin(delta)*vx + cos(delta)*(vy+lf*m_yawrate);
      auto vxr = vx;
      auto vyr = vy-lr*m_yawrate;
      auto alphaf = abs(vxf) > 0.00001 ? 0 : -atan(vyf/vxf);
      auto alphar = abs(vxr) > 0.00001 ? 0 : -atan(vyr/vxr);

      // Longitudinal tire Force
      auto Fxf = vx > 0 || m_inputAcceleration > 0 ? m*m_inputAcceleration : 0;
      double const Fxr = 0;

      // Lateral tire Force
      auto Fy0f = muyf*Fzf*sin(Cyf*atan(Byf*alphaf-Eyf*(Byf*alphaf-atan(Byf*alphaf))));
      auto Fy0r = muyr*Fzr*sin(Cyr*atan(Byr*alphar-Eyr*(Byr*alphar-atan(Byr*alphar))));
      auto Fyf = Fy0f*sqrt(1-pow(Fxf/Fzf/muxf,2));
      auto Fyr = Fy0r*sqrt(1-pow(Fxr/Fzr/muxr,2));

      // Forces and moment
      auto FX = Fxf*cos(delta) - Fyf*sin(delta) + Fxr;
      auto FY = Fxf*sin(delta) + Fyf*cos(delta) + Fyr;
      auto MZ = lf*(Fxf*sin(delta)+Fyf*cos(delta)) - lr*Fyr;

      // Equations of motion
      auto dX = vx*cos(m_orientation)-vy*sin(m_orientation);
      auto dY = vx*sin(m_orientation)+vy*cos(m_orientation);
      auto dpsi = m_yawrate;

      auto dvx = FX/m + vy*m_yawrate;
      auto dvy = FY/m - vx*m_yawrate;
      auto dr = MZ/Izz;

      // Euler Forward
      m_position += opendlv::data::environment::Point3(dX,dY,0)*dt;
      m_orientation += dt*dpsi;
      m_velocity += opendlv::data::environment::Point3(dvx,dvy,0)*dt;
      m_yawrate += dt*dr;

      // Send GPS coordinate
      auto wgs84Coordinate = m_wgs84Reference.transform(m_position);
      odcore::data::Container c_coordinate(wgs84Coordinate);
      getConference().send(c_coordinate);

      // Send Groundspeed
      auto groundSpeed = opendlv::proxy::GroundSpeedReading();
      groundSpeed.setGroundSpeed(m_velocity.getX());
      odcore::data::Container c_velocity(groundSpeed);
      getConference().send(c_velocity);

      std::cout << "position: (" << std::to_string(m_position.getX()) << ", " << std::to_string(m_position.getY()) << ")" << std::endl;
      std::cout << "orientation: " << std::to_string(m_orientation) << std::endl;
      std::cout << "velocity: (" << std::to_string(m_velocity.getX()) << ", " << std::to_string(m_position.getY()) << ")" << std::endl;
      std::cout << "yaw rate: " << std::to_string(m_yawrate) << std::endl;
    }

  }

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}


}
}
}
