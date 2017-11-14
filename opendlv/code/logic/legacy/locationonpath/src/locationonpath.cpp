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
  m_intersectionPosition(0,0,0)
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
  std::cout << "m_intersectionPosition" << m_intersectionPosition.getX() << ", " << m_intersectionPosition.getY() << '\n';

}

void LocationOnPath::tearDown()
{
}

void LocationOnPath::nextContainer(odcore::data::Container &a_container)
{
  if (a_container.getDataType() == opendlv::data::environment::WGS84Coordinate::ID()) {
    odcore::base::Lock l(m_referenceMutex);
    auto currentPosition = a_container.getData<opendlv::data::environment::WGS84Coordinate>();
    m_position = m_wgs84Reference.transform(currentPosition);
    cout << "LOP: Recieved WGS84Coordinate (X, Y): " << m_position.getX() << ", " << m_position.getY() << endl;

  } else if (a_container.getDataType() == opendlv::proxy::GroundSpeedReading::ID()) {

  } else if (a_container.getDataType() == opendlv::proxy::AccelerometerReading::ID()) {

  } else if (a_container.getDataType() == opendlv::proxy::GyroscopeReading::ID()) {

  }
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
    std::cout << "Line for the road:" << a << ", " << b << ", " << c << '\n';
  } else {
    std::cout << "x1 is equal to x2" << '\n';
  }


  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
      // Calculate location on the path
      double x0 = m_position.getX();
      double y0 = m_position.getY();
      double x = (b*(b*x0-a*y0)-a*c)/(pow(a,2)+pow(b,2));
      double y = (a*(-b*x0+a*y0)-b*c)/(pow(a,2)+pow(b,2));
      auto projectionPoint = opendlv::data::environment::Point3(x,y,0);

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
      std::cout << "currentLocation: " << currentLocation << '\n';
      std::cout << "intersectionLocation: " << intersectionLocation << '\n';

      // Set location on the path
      opendlv::logic::legacy::LocationOnPathToIntersection locationOnPath;
      locationOnPath.setIntersectionLocation(intersectionLocation);
      locationOnPath.setCurrentLocation(currentLocation);
      odcore::data::Container initC(locationOnPath);
      getConference().send(initC);
    }


  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}


}
}
}
