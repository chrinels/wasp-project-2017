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
  m_wgs84Reference(),
  m_ekf(),
  m_systemModel(100.0,10.0,3.14,0.01),
  m_positionModel(0.1*0.1),
  m_orientationModel(0.1),
  m_yawRateModel(0.1*0.1)
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

  State<T> state;
  state.setZero();
  m_ekf.init(state);
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

    {
      PositionMeasurement<T> p;
      p.x() = position_i.getX();
      p.y() = position_i.getY();
      auto state = m_ekf.update(m_positionModel,p);

      cout << "measured position: (" << p.x() << "," << p.y() << ")" << endl;
      cout << "estimated position: (" << state.px() << "," << state.py() << ")" << endl;
    }

    // Orientation estimate
    {
      OrientationMeasurement<T> o;
      o.psi() = 0.0;
      auto vx = m_ekf.getState().vx();
      auto vy = m_ekf.getState().vy();
      if (vx*vx+vy*vy > 1.0) { // drive faster than 3.6 km/h, otherwise assume constant orientation.
        o.psi() = std::atan2(vy,vx);
      }

      double const pi = 3.1415926;
      // Correct measurement to shortest angle
      while (o.psi() < -pi+m_ekf.getState().psi()) o.psi() += 2*pi;
      while (o.psi() >  pi+m_ekf.getState().psi()) o.psi() -= 2*pi;

      auto state = m_ekf.update(m_orientationModel,o);

      // Correct orientation
      while (state.psi() < -pi) state.psi() += 2*pi;
      while (state.psi() > pi) state.psi() -= 2*pi;

      cout << "received orientation: " << o.psi() << endl;
      cout << "estimated orientation: " << state.psi() << endl;
    }


  } else if (a_container.getDataType() == opendlv::proxy::GroundSpeedReading::ID()) {

  } else if (a_container.getDataType() == opendlv::proxy::AccelerometerReading::ID()) {

  } else if (a_container.getDataType() == opendlv::proxy::GyroscopeReading::ID()) {
    odcore::base::Lock l(m_stateMutex);
    auto yawrateReading = a_container.getData<opendlv::proxy::GyroscopeReading>();

    YawRateMeasurement<T> y;
    y.r() = yawrateReading.getAngularVelocityZ();
    auto state = m_ekf.update(m_yawRateModel,y);

    cout << "measured yaw rate: " << y.r() << endl;
    cout << "estimated yaw rate: " << state.dpsi() << endl;
  }

  // Read path
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode StateEstimator::body()
{
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
    odcore::base::Lock l(m_stateMutex);

    // Predict
    Control<T> u;
    auto state = m_ekf.predict(m_systemModel, u);

    auto velocity = std::sqrt(state.vx()*state.vx()+state.vy()*state.vy());

    // Print current state
    std::cout << "position: (" << state.px() << ", " << state.py() << ")" << std::endl;
    std::cout << "velocity: " << velocity << std::endl;
    std::cout << "orientation: " << state.psi() << std::endl;
    std::cout << "yaw rate: " << state.dpsi() << std::endl;

    // Send StateEstimate
    opendlv::logic::legacy::StateEstimate se;
    se.setPositionX(state.px());
    se.setPositionY(state.py());
    se.setVelocityX(velocity);
    se.setVelocityY(0.0);
    se.setOrientation(state.psi());
    se.setYawRate(state.dpsi());
    odcore::data::Container c = odcore::data::Container(se);
    getConference().send(c);
  }

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}


}
}
}
