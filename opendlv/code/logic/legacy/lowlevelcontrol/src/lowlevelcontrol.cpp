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
  m_wgs84Reference(),
  m_referenceMutex(),
  m_velocityReference(0),
  m_velocitySumReference(0),
  m_longitudinalGain(0),
  m_maxAccelerationLimit(0),
  m_minAccelerationLimit(0),
  m_velocitySumLimit(0),
  m_accelerationSmoothing(0),
  m_locationOnPath(),
  m_aimPointGain(0),
  m_steeringFilterCoefficient(0),
  m_steeringWheelAngle(0),
  m_velocityHorizon(),
  m_velocityHorizonIsValid(false)
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

  m_longitudinalGain = getKeyValueConfiguration().getValue<double>(
    "logic-legacy-lowlevelcontrol.longitudinal-gain");
  m_maxAccelerationLimit = getKeyValueConfiguration().getValue<double>(
    "logic-legacy-lowlevelcontrol.longitudinal-max-acceleration");
  m_minAccelerationLimit = getKeyValueConfiguration().getValue<double>(
    "logic-legacy-lowlevelcontrol.longitudinal-min-acceleration");
  m_velocitySumLimit = getKeyValueConfiguration().getValue<double>(
    "logic-legacy-lowlevelcontrol.longitudinal-max-velocitysum");
  m_accelerationSmoothing = getKeyValueConfiguration().getValue<double>(
    "logic-legacy-lowlevelcontrol.longitudinal-acceleration-smoothing");
  m_aimPointGain = getKeyValueConfiguration().getValue<double>(
    "logic-legacy-lowlevelcontrol.aim_point_gain");
  m_steeringFilterCoefficient = getKeyValueConfiguration().getValue<double>(
    "logic-legacy-lowlevelcontrol.steering_filter_coefficient");

}

void LowLevelControl::tearDown()
{
}

void LowLevelControl::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == opendlv::logic::legacy::StateEstimate::ID()) {
    odcore::base::Lock l(m_stateMutex);
    auto stateEstimate = a_container.getData<opendlv::logic::legacy::StateEstimate>();
    m_velocity.setX(stateEstimate.getVelocityX());
    cout << "Recieved velocity: " << m_velocity.getX() << endl;

  } else if (a_container.getDataType() == opendlv::logic::legacy::VelocityRequest::ID()) {
    odcore::base::Lock l(m_referenceMutex);
    auto velocityRequest = a_container.getData<opendlv::logic::legacy::VelocityRequest>();
    m_velocityReference = velocityRequest.getVelocity();
    m_velocityHorizonIsValid = false;
    cout << "Recieved VelocityReference: " << m_velocityReference << endl;

  } else if (a_container.getDataType() == opendlv::logic::legacy::LocationOnPathToIntersection::ID()) {
      m_locationOnPath = a_container.getData<opendlv::logic::legacy::LocationOnPathToIntersection>();
      std::cout << "Received LocationOnPath, errAngle: " << m_locationOnPath.getErrAngle() << std::endl;

  } else if (a_container.getDataType() == opendlv::logic::legacy::VelocityHorizon::ID()) {
    odcore::base::Lock l(m_referenceMutex);
    m_velocityHorizon = a_container.getData<opendlv::logic::legacy::VelocityHorizon>();
    if (m_velocityHorizon.isEmpty_ListOfTimeStamp() || m_velocityHorizon.isEmpty_ListOfVelocity()) {
        m_velocityHorizonIsValid = false;
        cout << "WARNING: Bad VelocityHorizon." << endl;
    } else {
        m_velocityHorizonIsValid = true;
    }
    cout << "Size of Velocity " << m_velocityHorizon.getSize_ListOfVelocity() << endl;
    cout << "Size of TimeStamp " << m_velocityHorizon.getSize_ListOfTimeStamp() << endl;

  }

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
    odcore::base::Lock ls(m_stateMutex);
    odcore::base::Lock lr(m_referenceMutex);

    double velocityReference = 0.0;
    double accelerationReference = 0.0;

    if (m_velocityHorizonIsValid) {
        odcore::data::TimeStamp currentTime;
        auto tvec = m_velocityHorizon.getListOfTimeStamp();
        auto vvec = m_velocityHorizon.getListOfVelocity();

        auto size = std::min(tvec.size(),vvec.size());

        size_t i=0;
        for (; i < size &&  tvec[i] <= currentTime; ++i);


        if (i > size-1) { // Keep the last specified velocity
          velocityReference = vvec.back();
          accelerationReference = 0.0;

        } else {
          // Feed forward acceleration
          double timeToNextSegment = (tvec[i]-currentTime).toMicroseconds()*1.0/1000000L;
          accelerationReference = (vvec[i]-m_velocity.getX())/timeToNextSegment;

          if (i == 0) { // yet to reach the first specified velocity
            velocityReference = vvec.front();
          } else {
            auto segmentdt = (tvec[i]-tvec[i-1]).toMicroseconds()*1.0/1000000L;
            auto segmentSlope = (vvec[i]-vvec[i-1])/segmentdt;
            auto timeSinceLastSegment = (currentTime - tvec[i-1]).toMicroseconds()*1.0/1000000L;
            velocityReference = vvec[i-1] + segmentSlope*timeSinceLastSegment;
          }

          // Linear interpolation of next accelerationReference and current
          if (m_accelerationSmoothing > 0.0 && timeToNextSegment < m_accelerationSmoothing) {
            double nextSegmentSlope = 0.0;
            if (i < size-1) {
              nextSegmentSlope = (vvec[i+1]-vvec[i])/((tvec[i+1]-tvec[i]).toMicroseconds()*1.0/1000000L);
            }
            auto proportion = (m_accelerationSmoothing-timeToNextSegment)/m_accelerationSmoothing;
            accelerationReference = accelerationReference*(1.0-proportion)+nextSegmentSlope*proportion;
          }


        }

        // uint32_t i=0;
        // for (; i < size &&  tvec[i] <= currentTime; ++i);

        // if (i >= size-1) {
        //     velocityReference = vvec.back();
        //     accelerationReference = 0.0;
        // } else {
        //     double tdiff = (tvec[i+1] - tvec[i]).toMicroseconds()*1.0/1000000L;
        //     accelerationReference = (vvec[i+1]-vvec[i])/tdiff;
        //     velocityReference = accelerationReference*(currentTime-tvec[i]).toMicroseconds()*1.0/1000000L;

        //     // Linear interpolation of next accelerationReference and current
        //     auto tcurrentdiff = (tvec[i+1] - currentTime).toMicroseconds()*1.0/1000000L;
        //     if (m_accelerationSmoothing > 0.0 && tcurrentdiff < m_accelerationSmoothing) {
        //         auto anext = i >= size-2 ? 0.0 : (vvec[i+2]-vvec[i+1])/((tvec[i+2] - tvec[i+1]).toMicroseconds()*1.0/1000000L);
        //         auto proportion = (m_accelerationSmoothing-tcurrentdiff)/m_accelerationSmoothing;
        //         accelerationReference = accelerationReference*proportion+anext*(1.0-proportion);
        //     }
        // }

    } else {
        velocityReference = m_velocityReference;
        accelerationReference = 0;
    }

    double const dt = 1.0 / static_cast<double>(getFrequency());

    // PI velocity control
    {
      double const kp = m_longitudinalGain;
      double const ki = m_velocityHorizonIsValid ? 0.0 : kp/4;

      auto dv = m_velocity.getX() - velocityReference;
      m_inputAcceleration = -kp*dv - ki*m_velocitySumReference + accelerationReference;

      // Anti windup (clamp)
      if (m_inputAcceleration < m_maxAccelerationLimit && m_inputAcceleration > m_minAccelerationLimit) {
        m_inputAcceleration += ki*dt*dv;
        m_velocitySumReference += dt*dv;
        m_velocitySumReference = fmin(m_velocitySumReference,m_velocitySumLimit);
        m_velocitySumReference = fmax(m_velocitySumReference,-m_velocitySumLimit);
      }
      m_inputAcceleration = fmin(m_inputAcceleration,m_maxAccelerationLimit);
      m_inputAcceleration = fmax(m_inputAcceleration,m_minAccelerationLimit);
    }

    // Steergin control
    double aimPointAngle = m_locationOnPath.getErrAngle();
    while (aimPointAngle < -cartesian::Constants::PI) {
      aimPointAngle += 2.0 * cartesian::Constants::PI;
    }
    while (aimPointAngle > cartesian::Constants::PI) {
      aimPointAngle -= 2.0 * cartesian::Constants::PI;
    }
    double const steeringWheelAngleUnfiltered = m_aimPointGain * aimPointAngle;
    double const steeringWheelAngleRate = m_steeringFilterCoefficient *
      (steeringWheelAngleUnfiltered - m_steeringWheelAngle);
    m_steeringWheelAngle += steeringWheelAngleRate * dt;
    m_steeringWheelAngle = (m_steeringWheelAngle > 3.0 * cartesian::Constants::PI) ? 9.3 : m_steeringWheelAngle;
    m_steeringWheelAngle = (m_steeringWheelAngle < -3.0 * cartesian::Constants::PI) ? -9.3 : m_steeringWheelAngle;
    std::cout << "m_steeringWheelAngle: " << m_steeringWheelAngle << std::endl;
    // m_inputSteeringWheelAngle = m_steeringWheelAngle;

    cout << "VelocityReference: " << velocityReference << endl;
    cout << "AccelerationReference: " << accelerationReference << endl;


    cout << "Send AccelerationRequest: " << m_inputAcceleration << endl;
    cout << "Send SteeringRequest: " << m_inputSteeringWheelAngle << endl;

    // Send ActuationRequest
    opendlv::proxy::ActuationRequest ar;
    ar.setAcceleration(static_cast<float>(m_inputAcceleration));
    ar.setSteering(static_cast<float>(m_inputSteeringWheelAngle));
    ar.setIsValid(true);
    odcore::data::Container c = odcore::data::Container(ar);
    getConference().send(c);

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
