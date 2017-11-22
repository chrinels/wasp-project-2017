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

#ifndef LOGIC_LEGACY_INTERSECTION_INTERSECTION_HPP
#define LOGIC_LEGACY_INTERSECTION_INTERSECTION_HPP

#include <memory>

#include "opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h"
#include <opendavinci/odcore/base/Mutex.h>
#include <opendlv/data/environment/Point3.h>
#include <opendlv/data/environment/WGS84Coordinate.h>

namespace opendlv {
namespace logic {
namespace coordination {

class Intersection : public odcore::base::module::DataTriggeredConferenceClientModule {
 public:
  Intersection(int32_t const &, char **);
  Intersection(Intersection const &) = delete;
  Intersection &operator=(Intersection const &) = delete;
  virtual ~Intersection();

  virtual void nextContainer(odcore::data::Container &c);

 private:
  //! Enumeration of valid trajectories
  //! [W]est/[S]outh/[N]orth/[E]ast - direction of approach
  //! [S]traight/[L]eft/[R]ight - Path plan
  enum Trajectory { WS, WR, WL,
                    SS, SR, SL,
                    NS, NR, NL,
                    ES, ER, EL };

  // TODO: Reuse some existing structure?
  struct GPSCoord
  {
    double x;
    double y;
    double z;
  };

  // TODO: Move to opendlv::collaboration::message?
  struct IntersectionAccessRequest
  {
    int vehicleID;
    Trajectory plannedTrajectory;
    GPSCoord currentPosition;
    // opendlv::data::environment::WGS84Coordinate m_wgs84CurrentPosition;
    float currentSpeed;
  };

  struct SchedulingInfo
  {
    int vehicleID;
    float intersectionAccessTime;
    Trajectory trajectory;
  };

  void setUp();
  void setUpTrajectories();
  void tearDown();

  bool  scheduleVehicle(const IntersectionAccessRequest &);
  float estimateIntersectionAccessTime(const GPSCoord &, float);
  int   determineFirstAccessibleSlot(float);

  void addScheduledVehicleToSlot(int, int, SchedulingInfo);
  bool contains(const std::vector<Trajectory> &, Trajectory);
  bool timeRefreshSlotsTable();

  //! Member variables
  odcore::base::Mutex m_timeRefreshMutex;
  bool  m_initialised;
  float m_slotDuration;	// [seconds]
  float m_nrofSlots;	// Number of schedulable slots

  GPSCoord m_intersectionPosition; // Location of the center of the intersection
  // opendlv::data::environment::WGS84Coordinate m_wgs84IntersectionPosition;

  std::vector<Trajectory>                       m_allTrajectories;
  std::map<Trajectory, std::vector<Trajectory>> m_compatibleTrajectories;
  std::vector<std::vector<SchedulingInfo>>      m_scheduledSlotsTable; // [slot [schedInfo]]
  opendlv::data::environment::WGS84Coordinate   m_wgs84Reference;
};

}
}
}

#endif
