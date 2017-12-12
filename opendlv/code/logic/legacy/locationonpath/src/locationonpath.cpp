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
// #include <odvdopendlvdata/GeneratedHeaders_ODVDOpenDLVData.h>
#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>
#include <odvdimu/GeneratedHeaders_ODVDIMU.h>

#include "locationonpath.hpp"

namespace opendlv {
namespace logic {
namespace legacy {

LocationOnPath::LocationOnPath(int32_t const &a_argc, char **a_argv)
  : TimeTriggeredConferenceClientModule(a_argc, a_argv,
      "logic-legacy-locationonpath"),
  m_stateMutex(),
  m_position(0,0,0),
  m_orientation(0),
  m_velocity(1,0,0),
  m_yawrate(0),
  m_wgs84Reference(),
  m_referenceMutex(),
  m_intersectionPosition(0,0,0),
  // m_virtualPosition(false),
  m_vehicleSimState(),
  m_forwardDistance(0)
{
}

LocationOnPath::~LocationOnPath()
{
}

void LocationOnPath::setUp()
{
  double const latitude = getKeyValueConfiguration().getValue<double>(
      "global.reference.WGS84.latitude");
  double const longitude = getKeyValueConfiguration().getValue<double>(
      "global.reference.WGS84.longitude");
  m_wgs84Reference = opendlv::data::environment::WGS84Coordinate(latitude,longitude);
  // double testTimeToIntersection = getKeyValueConfiguration().getValue<double>(
  //   "global.intlat");
  // std::cout << "testTimeToIntersection" << testTimeToIntersection << '\n';
  double const latitudeIntersection = getKeyValueConfiguration().getValue<double>(
      "logic-legacy-locationonpath.intersection-latitude");
  double const longitudeIntersection = getKeyValueConfiguration().getValue<double>(
      "logic-legacy-locationonpath.intersection-longitude");
  auto wgs84IntersectionPosition = opendlv::data::environment::WGS84Coordinate(latitudeIntersection,longitudeIntersection);
  m_intersectionPosition = m_wgs84Reference.transform(wgs84IntersectionPosition);
  if(odcore::base::module::AbstractCIDModule::isVerbose()) {
    std::cout << "m_intersectionPosition" << m_intersectionPosition.getX() << ", " << m_intersectionPosition.getY() << '\n';
  }
  // int32_t virtual_position = getKeyValueConfiguration().getValue<int32_t>(
  //     "logic-legacy-locationonpath.virtualPosition");
  // if (virtual_position == 1)
  //   m_virtualPosition = true;
  m_forwardDistance = getKeyValueConfiguration().getValue<double>(
      "logic-legacy-locationonpath.forward-distance");

}

void LocationOnPath::tearDown()
{
}

void LocationOnPath::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == opendlv::logic::legacy::StateEstimate::ID()) {
    odcore::base::Lock l(m_referenceMutex);
    auto stateEstimate = a_container.getData<opendlv::logic::legacy::StateEstimate>();
    m_position.setX(stateEstimate.getPositionX());
    m_position.setY(stateEstimate.getPositionY());
    m_orientation = stateEstimate.getOrientation();
  } else if (a_container.getDataType() == opendlv::logic::legacy::VehicleSimState::ID()) {
    m_vehicleSimState = a_container.getData<opendlv::logic::legacy::VehicleSimState>();
  }
  // if (a_container.getDataType() == opendlv::data::environment::WGS84Coordinate::ID()) {
  //   odcore::base::Lock l(m_referenceMutex);
  //   auto currentPosition = a_container.getData<opendlv::data::environment::WGS84Coordinate>();
  //   m_position = m_wgs84Reference.transform(currentPosition);
  //   cout << "LOP: Recieved WGS84Coordinate (X, Y): " << m_position.getX() << ", " << m_position.getY() << endl;

  // } else if (a_container.getDataType() == opendlv::proxy::GroundSpeedReading::ID()) {

  // } else if (a_container.getDataType() == opendlv::proxy::AccelerometerReading::ID()) {

  // } else if (a_container.getDataType() == opendlv::proxy::GyroscopeReading::ID()) {

  // }
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode LocationOnPath::body()
{
  auto startPoint = opendlv::data::environment::Point3(0,0,0);
  double currentLocation = 0, intersectionLocation = 0;
  // Line ax+by+c=0 via start (0,0) and m_intersectionPosition
  double x1 = 0, y1 = 0, x2 = m_intersectionPosition.getX(), y2 = m_intersectionPosition.getY();
  double a = 0, b = 0, c = 0;
  if (x1>x2 || x1<x2) {
    a = (y1-y2)/(x1-x2);
    b = -1;
    c = y1-a*x1;
    if(odcore::base::module::AbstractCIDModule::isVerbose()) {
      std::cout << "Line for the road:" << a << ", " << b << ", " << c << '\n';
    }
  } else {
    if(odcore::base::module::AbstractCIDModule::isVerbose()) {
      std::cout << "x1 is equal to x2" << '\n';
    }
  }


  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
      // Calculate location on the path
      double x0 = 0, y0 = 0, h0 = 0;
      double d = m_forwardDistance;
      // if (m_virtualPosition) {
      //   x0 = m_vehicleSimState.getPositionX();
      //   y0 = m_vehicleSimState.getPositionY();
      //   h0 = m_vehicleSimState.getOrientation();
      // }
      // else {
        x0 = m_position.getX();
        y0 = m_position.getY();
        h0 = m_orientation;
      // }


      double x = (b*(b*x0-a*y0)-a*c)/(pow(a,2)+pow(b,2));
      double y = (a*(-b*x0+a*y0)-b*c)/(pow(a,2)+pow(b,2));
      auto projectionPoint = opendlv::data::environment::Point3(x,y,0);
      auto vehiclePoint = opendlv::data::environment::Point3(x0,y0,0);
      double errDistance = (vehiclePoint-projectionPoint).lengthXY();

      intersectionLocation = (startPoint - m_intersectionPosition).lengthXY();
      if (((x1 < x2) && (x <= x2) && (x >= x1)) || ((x1 > x2) && (x <= x1) && (x >= x2))) {
        //point belongs to the segment
        currentLocation = (startPoint-projectionPoint).lengthXY();
      }
      if (((x1 < x2) && (x < x1)) || ((x1 > x2) && (x > x1))) {
        //point outside of the segment, in direction to the intersection
        currentLocation = - (startPoint-projectionPoint).lengthXY();
      }
      if (((x1 < x2) && (x > x2)) || ((x1 > x2) && (x < x2))) {
        //point outside of the segment, the intersection was passed
        currentLocation = intersectionLocation + (m_intersectionPosition-projectionPoint).lengthXY();
      }
      if(odcore::base::module::AbstractCIDModule::isVerbose()) {
        std::cout << "currentLocation: " << currentLocation << '\n';
        std::cout << "intersectionLocation: " << intersectionLocation << '\n';
        std::cout << "errDistance: " << errDistance << '\n';
      }

      // Calculate forward point for steering control
      //(x1,y1) (x2,y2) points for line perpendicularly to orientation at m_forwardDistance
      double xd = x0+d*cos(h0);
      double yd = y0+d*sin(h0);
      double xd2 = xd+d*sin(h0);
      double yd2 = yd-d*cos(h0);

      //(x1,y1) (x2,y2) for road
      double xr = x1;
      double yr = y1;
      // double hr = atan(a);
      double xr2 = x2;
      double yr2 = y2;

      // point for forward goal
      double xp = ((xr*yr2-yr*xr2)*(xd-xd2)-(xr-xr2)*(xd*yd2-yd*xd2))/((xr-xr2)*(yd-yd2)-(yr-yr2)*(xd-xd2));
      double yp = ((xr*yr2-yr*xr2)*(yd-yd2)-(yr-yr2)*(xd*yd2-yd*xd2))/((xr-xr2)*(yd-yd2)-(yr-yr2)*(xd-xd2));

      // angle to forward goal
      double hgoal = atan((yp-y0)/(xp-x0));
      double errAngle = h0-hgoal;
      while (errAngle < -cartesian::Constants::PI) {
        errAngle += 2.0 * cartesian::Constants::PI;
      }
      while (errAngle > cartesian::Constants::PI) {
        errAngle -= 2.0 * cartesian::Constants::PI;
      }
      if(odcore::base::module::AbstractCIDModule::isVerbose()) {
        std::cout << "errAngle, hgoal: " << errAngle << ", " << hgoal << '\n';
      }

      // Set location on the path
      opendlv::logic::legacy::LocationOnPathToIntersection locationOnPath;
      locationOnPath.setIntersectionLocation(intersectionLocation);
      locationOnPath.setCurrentLocation(currentLocation);
      locationOnPath.setErrDistance(errDistance);
      locationOnPath.setErrAngle(-errAngle);
      odcore::data::Container initC(locationOnPath);
      getConference().send(initC);
    }


  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}


}
}
}
