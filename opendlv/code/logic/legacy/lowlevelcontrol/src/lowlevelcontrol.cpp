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

  } else if (a_container.getDataType() == opendlv::logic::legacy::VelocityHorizon::ID()) {
    odcore::base::Lock l(m_referenceMutex);
    m_velocityHorizon = a_container.getData<opendlv::logic::legacy::VelocityHorizon>();
    m_velocityHorizonIsValid = true;
    cout << "Recieved VelocityHorizon." << endl;

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
      auto tpair = m_velocityHorizon.iteratorPair_ListOfTimeStamp();
      auto vpair = m_velocityHorizon.iteratorPair_ListOfVelocity();
  		auto it = tpair.first;
  		auto iv = vpair.first;
    	odcore::data::TimeStamp t0;
    	odcore::data::TimeStamp t1;
    	double v0 = 0.0;
    	double v1 = 0.0;
    	bool hast0 = false;
    	bool hast1 = false;
  		for(; it != tpair.second && iv != vpair.second; ++it, ++iv) {
		    if (*it < currentTime) {
		    	v0 = *iv;
		    	t0 = *it;
		    	hast0 = true;
		    } else {
		    	v1 = *iv;
		    	t1 = *it;
		    	hast1 = true;
		    	break;
		    }
  		}
  		if (hast0 && hast1) {
  			auto tdiff = (t1 - t0).toMicroseconds()*1.0/1000000L;
  			accelerationReference = (v1-v0)/tdiff;
  			velocityReference = accelerationReference*(currentTime-t0).toMicroseconds()*1.0/1000000L;
  		} else {
    		velocityReference = *(vpair.second-1);
  			accelerationReference = 0;
  		}

    } else {
  		velocityReference = m_velocityReference;
  		accelerationReference = 0;
  	}

    double const dt = 1.0 / static_cast<double>(getFrequency());

    // PI velocity control
    {
      double const kp = m_longitudinalGain;
      double const ki = kp/4;

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

    cout << "Send velocityReference: " << velocityReference << endl;
    cout << "Send accelerationReference: " << accelerationReference << endl;


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
