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

#include <list>
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
	m_distanceToIntersection(0),
  m_start_velocity(0),
  m_up_velocity(0),
  m_down_velocity(0),
  m_end_velocity(0),
  m_start_point(0),
  m_time_segment_seconds(0)
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
  m_start_velocity = getKeyValueConfiguration().getValue<double>(
    "logic-legacy-velocitytuner.start_velocitykmh")/3.6;
  m_up_velocity = getKeyValueConfiguration().getValue<double>(
    "logic-legacy-velocitytuner.up_velocitykmh")/3.6;
  m_down_velocity = getKeyValueConfiguration().getValue<double>(
    "logic-legacy-velocitytuner.down_velocitykmh")/3.6;
  m_end_velocity = getKeyValueConfiguration().getValue<double>(
    "logic-legacy-velocitytuner.end_velocitykmh")/3.6;
  m_start_point = getKeyValueConfiguration().getValue<double>(
    "logic-legacy-velocitytuner.start_point");
  m_time_segment_seconds = getKeyValueConfiguration().getValue<double>(
    "logic-legacy-velocitytuner.time_segment_seconds");

}

void VelocityTuner::tearDown()
{
}

void VelocityTuner::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == opendlv::logic::legacy::StateEstimate::ID()) {
    odcore::base::Lock l(m_stateMutex);
    auto stateEstimate = a_container.getData<opendlv::logic::legacy::StateEstimate>();
    m_velocity.setX(stateEstimate.getVelocityX());

  } else if (a_container.getDataType() == opendlv::logic::legacy::TimeSlot::ID()) {
    odcore::base::Lock l(m_referenceMutex);
    auto timeSlot = a_container.getData<opendlv::logic::legacy::TimeSlot>();
    m_timeSlotStart = timeSlot.getEntryTime();
    cout << "Recieved TimeSlot, SlotStart: " << m_timeSlotStart << endl;

  } else if (a_container.getDataType() == opendlv::logic::legacy::LocationOnPathToIntersection::ID()) {
    odcore::base::Lock l(m_stateMutex);
    auto locationOnPathToIntersection = a_container.getData<opendlv::logic::legacy::LocationOnPathToIntersection>();
    m_distanceToIntersection = locationOnPathToIntersection.getIntersectionLocation() - locationOnPathToIntersection.getCurrentLocation();
  }
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode VelocityTuner::body()
{
  double const Tint = 1.0/static_cast<double>(getFrequency());
  std::cout << "Tint: " << Tint << '\n';
  bool flag = false;

  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
    //debug message
    opendlv::logic::legacy::VelocityTunerState velocityTunerState;
    if (m_distanceToIntersection < m_start_point) {
      opendlv::logic::legacy::VelocityRequest velocityRequest;
      velocityRequest.setVelocity(m_start_velocity);
      odcore::data::Container initC(velocityRequest);
      getConference().send(initC);
      std::cout << "Sending constant velocityRequest: " << m_start_velocity*3.6 << '\n';
    } else {
      if (flag == false) {
        flag = true;
        odcore::data::TimeStamp currentTime;
        opendlv::logic::legacy::VelocityHorizon velocityHorizon;
        // std::vector<double> velocity;
        // std::vector<odcore::data::TimeStamp> timeStamp;
        // velocity.push_back(m_up_velocity);
        // timeStamp.push_back(currentTime);
        // opendlv::logic::legacy::VelocityHorizon velocityHorizon(velocity,timeStamp);
        velocityHorizon.addTo_ListOfTimeStamp(currentTime);
        velocityHorizon.addTo_ListOfVelocity(m_start_velocity);

        velocityHorizon.addTo_ListOfTimeStamp(currentTime+odcore::data::TimeStamp(m_time_segment_seconds,0));
        velocityHorizon.addTo_ListOfVelocity(m_up_velocity);
        velocityHorizon.addTo_ListOfTimeStamp(currentTime+odcore::data::TimeStamp(2*m_time_segment_seconds,0));
        velocityHorizon.addTo_ListOfVelocity(m_up_velocity);
        velocityHorizon.addTo_ListOfTimeStamp(currentTime+odcore::data::TimeStamp(3*m_time_segment_seconds,0));
        velocityHorizon.addTo_ListOfVelocity(m_down_velocity);
        velocityHorizon.addTo_ListOfTimeStamp(currentTime+odcore::data::TimeStamp(4*m_time_segment_seconds,0));
        velocityHorizon.addTo_ListOfVelocity(m_end_velocity);
        velocityHorizon.addTo_ListOfTimeStamp(currentTime+odcore::data::TimeStamp(5*m_time_segment_seconds,0));
        velocityHorizon.addTo_ListOfVelocity(m_end_velocity);

        odcore::data::Container initC3(velocityHorizon);
        getConference().send(initC3);
      }
    }


    // Calculate what velocity to set
    if (m_distanceToIntersection > 0) {
      double s = m_distanceToIntersection;
      velocityTunerState.setS(s);
      std::cout << "s: " << s << '\n';
      odcore::data::TimeStamp currentTime;
      auto T = (m_timeSlotStart - currentTime).toMicroseconds()*1.0/1000000L;
      velocityTunerState.setT(T);
      std::cout << "T: " << T << '\n';
      double v1 = m_velocity.getX();
      velocityTunerState.setV1(v1);
      double vout = m_targetVelocity;
      double a = m_maxAccleleration*m_acclerationPlanningFactor;
      double vmax = m_maxVelocity;

      double acctime = abs(vout-v1)/a;
      velocityTunerState.setAcctime(acctime);
      //minimal distance for maneuvers ___/, \___
      double mindistance = fmin(v1,vout)*(T-acctime)+(vout+v1)/2*acctime;
      velocityTunerState.setMindistance(mindistance);
      //maximum distance for maneuvers --\, /--
      double maxdistance = fmax(v1,vout)*(T-acctime)+(vout+v1)/2*acctime;
      velocityTunerState.setMaxdistance(maxdistance);
      cout << "Calculated min and max distance for maneuvers ___/: " << mindistance << ", " << maxdistance << endl;
      double averagedistance = (vout+v1)/2*T;
      velocityTunerState.setAveragedistance(averagedistance);

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
            velocityTunerState.setMm(1);
          } else if (s > averagedistance) {
            t1 = (vout*T-s)/0.5/(vout-v1);
            v2 = vout;
            v3 = vout;
            // disp('/--')
            // mm(i) = 2;
            velocityTunerState.setMm(2);
          } else {
            t1 = T;
            v2 = vout;
            v3 = vout;
            // mm(i) = 3;
            velocityTunerState.setMm(3);
          }
        } else if (vout < v1) {
          if (s > averagedistance) {
            t1 = (s-T/2*(v1+vout))/0.5/(v1-vout);
            v2 = v1;
            v3 = vout;
            // disp('--\')
            // mm(i) = 4;
            velocityTunerState.setMm(4);
          } else if (s < averagedistance) {
            t1 = (vout*T-s)/0.5/(vout-v1);
            v2 = vout;
            v3 = vout;
            // disp('\__')
            // mm(i) = 5;
            velocityTunerState.setMm(5);
          } else {
            t1 = T;
            v2 = vout;
            v3 = vout;
            // mm(i) = 6;
            velocityTunerState.setMm(6);
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
          velocityTunerState.setMm(10);
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
            velocityTunerState.setMm(11);
          }
          std::cout << "s>maxdistance, scal:  " << scal << '\n';
          velocityTunerState.setDesireda(desireda);
          velocityTunerState.setScal(scal);
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
          velocityTunerState.setMm(12);
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
            velocityTunerState.setMm(13);
          }
          std::cout << "s<mindistance, scal:  " << scal << '\n';
          velocityTunerState.setDesireda(desireda);
          velocityTunerState.setScal(scal);
        }

      }
      velocityTunerState.setT1(t1);
      velocityTunerState.setT2(t2);
      velocityTunerState.setT3(t3);
      velocityTunerState.setV2(v2);
      velocityTunerState.setV3(v3);
      velocityTunerState.setV4(v4);


      //if(t1 <= T) was faulty code? always use the first part plan
      if (t1 <= Tint) {//decision based on point for next timeinterval
        commandAcc = (v2-v1)/t1;
        commandV = v1+commandAcc*(Tint);
      } else {
        commandAcc = (v3-v2)/(T-t1);
        commandV = vout - commandAcc*(T-Tint);
      }
      velocityTunerState.setCommandAcc2(commandAcc);
      velocityTunerState.setCommandV2(commandV);
      if (abs(commandAcc) > m_maxAccleleration)
        std::cout << "commandacc to high: " << commandAcc << '\n';
      if ((commandV-v1)/Tint > m_maxAccleleration)
        commandV=v1+m_maxAccleleration*Tint;
      if ((v1-commandV)/Tint  > m_maxAccleleration)
        commandV=v1-m_maxAccleleration*Tint;
      if (commandV<0) {
        commandV = 0;
        // mm(i) = 15;
        velocityTunerState.setMm(15);
      }
      if (Tint > T) {//the intersection should be passed
        commandV = vout;
      }
      std::cout << "commandAcc: " << commandAcc << '\n';
      cout << "commandV: " << commandV << endl;
      velocityTunerState.setCommandAcc(commandAcc);
      velocityTunerState.setCommandV(commandV);
      // double commandV;
      // if (T > 0) {
      //   commandV = 7;
      // } else {
      //   commandV = 10;
      // }
      // cout << "commandV: " << commandV << endl;
      // Set velocity
      // opendlv::logic::legacy::VelocityRequest velocityRequest;
      // velocityRequest.setVelocity(commandV);
      // odcore::data::Container initC(velocityRequest);
      // getConference().send(initC);

      odcore::data::Container initC2(velocityTunerState);
      getConference().send(initC2);
    }


  }

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}


}
}
}
