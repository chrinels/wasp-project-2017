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

//#include <opendavinci/odcore/data/TimeStamp.h>

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
  m_wgs84Reference(),
  m_referenceMutex(),
	m_maxAccleleration(0),
  m_acclerationPlanningFactor(0),
	m_maxVelocity(0),
	m_timeSlotStart(),
  m_timeToIntersection(0),
	m_targetVelocity(0),
	m_distanceToIntersection(0)
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

  m_maxVelocity = getKeyValueConfiguration().getValue<double>(
    "logic-legacy-velocitytuner.max-velocity");
  m_targetVelocity = getKeyValueConfiguration().getValue<double>(
    "logic-legacy-velocitytuner.target-velocity-at-intersection");
  m_maxAccleleration = getKeyValueConfiguration().getValue<double>(
    "logic-legacy-velocitytuner.max-acceleration");
  m_acclerationPlanningFactor = getKeyValueConfiguration().getValue<double>(
    "logic-legacy-velocitytuner.acceleration-planning-factor");
  odcore::data::TimeStamp currentTime;
  double testTimeToIntersection = getKeyValueConfiguration().getValue<double>(
    "logic-legacy-velocitytuner.test-time-to-intersection");
  m_timeSlotStart = currentTime + odcore::data::TimeStamp(testTimeToIntersection,0);

}

void VelocityTuner::tearDown()
{
}

void VelocityTuner::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == opendlv::data::environment::WGS84Coordinate::ID()) {

  } else if (a_container.getDataType() == opendlv::proxy::GroundSpeedReading::ID()) {
    odcore::base::Lock l(m_stateMutex);
    auto groundSpeedReading = a_container.getData<opendlv::proxy::GroundSpeedReading>();
    m_velocity.setX(groundSpeedReading.getGroundSpeed());
    cout << "Recieved GroundSpeedReading: " << groundSpeedReading.getGroundSpeed() << endl;

  } else if (a_container.getDataType() == opendlv::proxy::AccelerometerReading::ID()) {

  } else if (a_container.getDataType() == opendlv::proxy::GyroscopeReading::ID()) {

  }
  else if (a_container.getDataType() == opendlv::logic::legacy::TimeSlot::ID()) {
    odcore::base::Lock l(m_referenceMutex);
    auto timeSlot = a_container.getData<opendlv::logic::legacy::TimeSlot>();
    m_timeSlotStart = timeSlot.getEntryTime();
    cout << "Recieved TimeSlot, SlotStart: " << m_timeSlotStart << endl;

  }
  else if (a_container.getDataType() == opendlv::logic::legacy::LocationOnPathToIntersection::ID()) {
    odcore::base::Lock l(m_stateMutex);
    auto locationOnPathToIntersection = a_container.getData<opendlv::logic::legacy::LocationOnPathToIntersection>();
    m_distanceToIntersection = locationOnPathToIntersection.getIntersectionLocation() - locationOnPathToIntersection.getCurrentLocation();
  }
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode VelocityTuner::body()
{
  double const Tint = 1.0/static_cast<double>(getFrequency());
  std::cout << "Tint: " << Tint << '\n';

  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
    // Calculate what velocity to set
    if (m_distanceToIntersection > 0) {
      double s = m_distanceToIntersection;
      std::cout << "s: " << s << '\n';
      odcore::data::TimeStamp currentTime;
      auto T = (m_timeSlotStart - currentTime).toMicroseconds()*1.0/1000000L;
      std::cout << "T: " << T << '\n';
      double v1 = m_velocity.getX();
      double vout = m_targetVelocity;
      double a = m_maxAccleleration*m_acclerationPlanningFactor;
      double vmax = m_maxVelocity;

      double acctime = abs(vout-v1)/a;
      //minimal distance for maneuvers ___/, \___
      double mindistance = fmin(v1,vout)*(T-acctime)+(vout+v1)/2*acctime;
      //maximum distance for maneuvers --\, /--
      double maxdistance = fmax(v1,vout)*(T-acctime)+(vout+v1)/2*acctime;
      cout << "Calculated min and max distance for maneuvers ___/: " << mindistance << ", " << maxdistance << endl;
      double averagedistance = (vout+v1)/2*T;

      double v2,v3,v4;
      double t1,t2,t3;
      double commandAcc, commandV;

      if (mindistance > maxdistance)
        cout << "acceleration over limit is required" << endl;

      if (s <= maxdistance && s >= mindistance) {
        //could be done with maneuvers like __/, /--
        if (vout > v1) {
          if (s < averagedistance){ // __/
            t1 = (s-T/2*(v1+vout))/0.5/(v1-vout);
            v2 = v1;
            v3 = vout;
            // disp('__/')
            // mm(i) = 1;
          } else if (s > averagedistance) {
            t1 = (vout*T-s)/0.5/(vout-v1);
            v2 = vout;
            v3 = vout;
            // disp('/--')
            // mm(i) = 2;
          } else {
            t1 = T;
            v2 = vout;
            v3 = vout;
            // mm(i) = 3;
          }
        } else if (vout < v1) {
          if (s > averagedistance) {
            t1 = (s-T/2*(v1+vout))/0.5/(v1-vout);
            v2 = v1;
            v3 = vout;
            // disp('--\')
            // mm(i) = 4;
          } else if (s < averagedistance) {
            t1 = (vout*T-s)/0.5/(vout-v1);
            v2 = vout;
            v3 = vout;
            // disp('\__')
            // mm(i) = 5;
          } else {
            t1 = T;
            v2 = vout;
            v3 = vout;
            // mm(i) = 6;
          }
        } else {
            std::cout << "equal velocities, what to do?" << '\n';
          }
      } else {
        //could be done with maneuvers like __/, /--
        if (s > maxdistance) {
          //accelerate
          double desireda = (2*s - T*v1 - T*vout +  sqrt(2)*
              sqrt(2*pow(s,2) - 2*s*T*v1 + pow(T,2)*pow(v1,2) - 2*s*T*vout + pow(T,2)*pow(vout,2)))/pow(T,2);
          //acceleration over the limit?
          t1 = (desireda*T - v1 + vout)/(2*desireda);
          v2 = v1 + desireda*t1;
          v3 = v1 + desireda*t1 - desireda*(T - t1);
          double scal = (v2 + v1)/2*t1 + (v3 + v2)/2*(T - t1);
          // mm(i) = 10;
          if (v2 > vmax) {
            desireda = (-pow(v1,2) + 2*v1*vmax - 2*pow(vmax,2) + 2*vmax*vout - pow(vout,2))/(2*(s - T*vmax));
            t1 = (vmax - v1)/desireda;
            t3 = (vmax - vout)/desireda;
            t2 = T - t1 - t3;
            // flag = true;
            v2 = vmax;
            v3 = vmax;
            v4 = vout;
            scal = (v1+v2)/2*t1+(v2+v3)/2*t2+(v3+v4)/2*t2;
            // mm(i) = 11;
          }
          std::cout << "s>maxdistance, scal:  " << scal << '\n';
        }
        if (s < mindistance) {
          //decelerate
          double desireda = (-2*s + T*v1 + T*vout +  sqrt(2)*
              sqrt(2*pow(s,2) - 2*s*T*v1 + pow(T,2)*pow(v1,2) - 2*s*T*vout + pow(T,2)*pow(vout,2)))/pow(T,2);
          t1 = (desireda*T + v1 - vout)/(2*desireda);
          // if t1 < 0
          //     disp('t1 negative');
          // end
          v2 = v1 - desireda*t1;
          v3 = v1 - desireda*t1 + desireda*(T - t1);
          double scal = (v2 + v1)/2*t1 + (v3 + v2)/2*(T - t1);
          // mm(i) = 12;
          if (v2 < 0) {
            desireda = (pow(v1,2) + pow(vout,2))/(2*s);
            t1 = v1/desireda;
            v2 = 0;
            v3 = 0;
            // flag = true;
            t2 = vout/desireda;
            v4 = vout;
            scal = (v1+v2)/2*t1+(v3+v4)/2*t2;
            // mm(i) = 13;
          }
          std::cout << "s<mindistance, scal:  " << scal << '\n';
        }

      }
      //if(t1 <= T) was faulty code? always use the first part plan
      if (t1 <= Tint) {//decision based on point for next timeinterval
        commandAcc = (v2-v1)/t1;
        commandV = v1+commandAcc*(Tint);
      } else {
        commandAcc = (v3-v2)/(T-t1);
        commandV = vout - commandAcc*(T-Tint);
      }
      if (abs(commandAcc) > m_maxAccleleration)
        std::cout << "commandacc to high: " << commandAcc << '\n';
      if ((commandV-v1)/Tint > m_maxAccleleration)
        commandV=v1+m_maxAccleleration*Tint;
      if ((v1-commandV)/Tint  > m_maxAccleleration)
        commandV=v1-m_maxAccleleration*Tint;
      if (commandV<0) {
        commandV = 0;
        // mm(i) = 15;
      }
      if (Tint > T) {//the intersection should be passed
        commandV = vout;
      }
      std::cout << "commandAcc: " << commandAcc << '\n';
      cout << "commandV: " << commandV << endl;
      // double commandV;
      // if (T > 0) {
      //   commandV = 7;
      // } else {
      //   commandV = 10;
      // }
      // cout << "commandV: " << commandV << endl;
      // Set velocity
      opendlv::logic::legacy::VelocityRequest velocityRequest;
      velocityRequest.setVelocity(commandV);
      odcore::data::Container initC(velocityRequest);
      getConference().send(initC);
    }


  }

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}


}
}
}
