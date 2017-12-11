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
#include <string>

#include "opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h"
#include <opendavinci/odcore/base/Mutex.h>

#include <opendlv/data/environment/WGS84Coordinate.h>
#include <opendlv/data/environment/Point3.h>
#include "opendavinci/odcore/data/TimeStamp.h"

#include <odvdopendlvdata/GeneratedHeaders_ODVDOpenDLVData.h>

namespace opendlv {
namespace logic {
namespace coordination {

struct SchedulingInfo; // forward declaration

class Intersection : public odcore::base::module::DataTriggeredConferenceClientModule {
 public:
  Intersection(int32_t const &, char **);
  Intersection(Intersection const &) = delete;
  Intersection &operator=(Intersection const &) = delete;
  virtual ~Intersection();

  //! Enumeration of valid trajectories
  //! [W]est/[S]outh/[N]orth/[E]ast - direction of approach
  //! [S]traight/[L]eft/[R]ight - Path plan
  enum Trajectory { WS, WR, WL,
                    SS, SR, SL,
                    NS, NR, NL,
                    ES, ER, EL };

  void setUp();
  virtual void nextContainer(odcore::data::Container &c);

 private:
  void setUpTrajectories();
  void tearDown();

  odcore::data::TimeStamp getSlotAbsoluteTime(int slot);
  bool vehicleAlreadyScheduled(int vehicleID);
  bool scheduleVehicle(const opendlv::logic::coordination::IntersectionAccessRequest &);

  void addScheduledVehicleToSlot(int, SchedulingInfo);
  bool contains(const std::vector<Trajectory> &, Trajectory) const;
  bool timeRefreshSlotsTable();
  void printTimeSlotTable();

  //! Member variables
  odcore::base::Mutex m_timeRefreshMutex;
  bool    m_initialised;
  double  m_slotDuration;	  // [seconds]
  double  m_nrofSlots;	    // Number of schedulable slots

  std::vector<Trajectory>                       m_allTrajectories;
  std::map<std::string, Trajectory>             m_trajectoryLookUp;
  std::map<Trajectory, std::vector<Trajectory>> m_compatibleTrajectories;
  std::vector<std::vector<SchedulingInfo>>      m_scheduledSlotsTable; // [slot [schedInfo]]
  odcore::data::TimeStamp                       m_slotTableAbsoluteTime;

};

struct SchedulingInfo
{
  SchedulingInfo();

  int vehicleID;
  Intersection::Trajectory trajectory;
  odcore::data::TimeStamp  scheduledSlotStartTime;
  odcore::data::TimeStamp  intersectionAccessTime;
};

inline SchedulingInfo::SchedulingInfo()
  : vehicleID(-1),
    trajectory(),
    scheduledSlotStartTime(),
    intersectionAccessTime() 
{
}

}
}
}

#endif
