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
#include <ctype.h>
#include <cstring>
#include <algorithm>

#include <iostream>


#include <opendavinci/odcore/data/Container.h>
#include "opendavinci/odcore/data/TimeStamp.h"

#include <opendavinci/odcore/base/Lock.h>

#include <opendavinci/odcore/io/conference/ContainerConference.h>

#include <automotivedata/GeneratedHeaders_AutomotiveData.h>
#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>

#include "intersection.hpp"

namespace opendlv {
namespace logic {
namespace coordination {

//-----------------------------------------------------------------------------
Intersection::Intersection(int32_t const &a_argc, char **a_argv)
  : DataTriggeredConferenceClientModule(a_argc, a_argv,
      "logic-coordination-intersection"),
    m_timeRefreshMutex(),
    m_initialised(false),
    m_slotDuration(5.0),
    m_nrofSlots(20),
    m_wgs84IntersectionPosition(),
    m_intersectionPosition(0,0,0),
    m_allTrajectories(),
    m_trajectoryLookUp(),
    m_compatibleTrajectories(),
    m_scheduledSlotsTable(),
    m_wgs84Reference()
{
}

//-----------------------------------------------------------------------------
Intersection::~Intersection()
{
}

void Intersection::setUp()
{
  cout << "Intersection::setUp()" << endl;
  // Extract parameter values
  odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();
  m_slotDuration = kv.getValue<float>(
      "logic-coordination-intersection.slot_duration");
  m_nrofSlots = kv.getValue<float>(
      "logic-coordination-intersection.nrof_slots");

  double const latitude = getKeyValueConfiguration().getValue<double>(
      "global.reference.WGS84.latitude");
  double const longitude = getKeyValueConfiguration().getValue<double>(
      "global.reference.WGS84.longitude");
  m_wgs84Reference = opendlv::data::environment::WGS84Coordinate(latitude,longitude);

  cout << "\t Read configuration file!" << endl;

  // TODO: Read from config!
  m_wgs84IntersectionPosition = opendlv::data::environment::WGS84Coordinate(latitude,longitude);

  m_intersectionPosition = m_wgs84Reference.transform(m_wgs84IntersectionPosition);

  cout << "\t Tranformed intersection WGS84 -> Local." << endl;

  setUpTrajectories();

  cout << "\t setUpTrajectories() \t DONE!" << endl;


  cout << "\t Initializing m_scheduledSlotsTable \t ..." << endl;
  // Create empty maps to store trajectory information for each slot
  for(int slot = 0; slot < m_nrofSlots; ++slot) {
    m_scheduledSlotsTable.push_back(std::vector<SchedulingInfo>());
  }

  cout << "\t Initializing m_scheduledSlotsTable \t DONE!" << endl;

  m_initialised = true;

  cout << "Intersection::setUp()\t DONE!" << endl;
}

//-----------------------------------------------------------------------------
void Intersection::tearDown()
{
}

//-----------------------------------------------------------------------------
void Intersection::nextContainer(odcore::data::Container &a_container)
{
  if(!m_initialised)
    return;

  odcore::data::TimeStamp now;
  auto timeSent = a_container.getSentTimeStamp();
  auto timeReceived = a_container.getReceivedTimeStamp();

  /**
  cout << " Received dataType ID = " << a_container.getDataType() << endl;
  cout << " Sent at " << timeSent.getYYYYMMDD_HHMMSSms() << endl;
  cout << " Received at " << timeReceived.getYYYYMMDD_HHMMSSms() << endl;
  */
  if (a_container.getDataType() == opendlv::logic::coordination::IntersectionAccessRequest::ID()) {
    cout << "Got an IntersectionAccessRequest!" << endl;
    auto accessRequest = a_container.getData<opendlv::logic::coordination::IntersectionAccessRequest>();
    cout << "opendlv::logic::coordination::IntersectionAccessRequest::ID = " << opendlv::logic::coordination::IntersectionAccessRequest::ID() << endl;
    cout << "opendlv::logic::coordination::IntersectionAccessRequest::vehicleID = " << accessRequest.getVehicleID() << endl;
    cout << "opendlv::logic::coordination::IntersectionAccessRequest::velocity = " << accessRequest.getVelocity() << endl;
    cout << "opendlv::logic::coordination::IntersectionAccessRequest::positionX = " << accessRequest.getPositionX() << endl;
    cout << "opendlv::logic::coordination::IntersectionAccessRequest::positionY = " << accessRequest.getPositionY() << endl;
    cout << "opendlv::logic::coordination::IntersectionAccessRequest::plannedTrajectory = " << accessRequest.getPlannedTrajectory() << endl;
    scheduleVehicle(accessRequest);
  }

}

//-----------------------------------------------------------------------------
bool Intersection::scheduleVehicle(const opendlv::logic::coordination::IntersectionAccessRequest &a_accessReq)
{
  bool schedulingSuccessful = false;

  odcore::base::Lock l(m_timeRefreshMutex);

  int vehicleID = a_accessReq.getVehicleID();

  Trajectory plannedTrajectory = m_trajectoryLookUp[a_accessReq.getPlannedTrajectory()];

  double currentVelocity = a_accessReq.getVelocity();
  double currentPositionX = a_accessReq.getPositionX();
  double currentPositionY = a_accessReq.getPositionY();

  float intersectionAccessTime = estimateIntersectionAccessTime(currentPositionX, currentPositionY,
                                                                currentVelocity);

  // Determine the first slot after the vehicle's access time
  int startSlot = determineFirstAccessibleSlot(intersectionAccessTime);


  // Find the first slot that does not contain any incompatible trajectories
  if(startSlot >= 0 && startSlot < m_nrofSlots) {
    for(int slot = startSlot; slot < m_nrofSlots; ++slot) {

      // Find compatible trajectories for this slot
      std::vector<Trajectory> validTrajectories = m_allTrajectories;
      for(SchedulingInfo slotSchedulingInfo : m_scheduledSlotsTable[slot]) {
        Trajectory scheduledTrajectory = slotSchedulingInfo.trajectory;
        std::vector<Trajectory> compatibleTrajectories = m_compatibleTrajectories[scheduledTrajectory];

        // Update the compatible trajectories
        std::vector<Trajectory> newValidTrajectories;
        for(unsigned int i = 0; i < validTrajectories.size(); ++i) {
          if(contains(compatibleTrajectories, validTrajectories[i])) {
            newValidTrajectories.push_back(validTrajectories[i]);
          }
        }

        // Update the valid trajectories
        validTrajectories = newValidTrajectories;
      }

      // Check if the planned trajectory is valid and add it to this slot
      if(contains(validTrajectories, plannedTrajectory)) {
        // Generate the scheduling info
        SchedulingInfo schedInfo;
        schedInfo.intersectionAccessTime = intersectionAccessTime;
        schedInfo.trajectory = plannedTrajectory;

        addScheduledVehicleToSlot(slot, vehicleID, schedInfo);
        schedulingSuccessful = true;
        break;
      }
    }
  }

  return schedulingSuccessful;
}

//-----------------------------------------------------------------------------
float Intersection::estimateIntersectionAccessTime(double a_positionX,
                                                   double a_positionY,
                                                   double a_currentSpeed) const
{
  // TODO: Logic for determining intersection access time
  double dx = m_intersectionPosition.getX() - a_positionX;
  double dy = m_intersectionPosition.getY() - a_positionY;
  return sqrt(dx*dx + dy*dy)/a_currentSpeed; // Time in seconds to intersection
}

//-----------------------------------------------------------------------------
bool Intersection::timeRefreshSlotsTable()
{
  odcore::base::Lock l(m_timeRefreshMutex);

  // Shift all slots to the left
  std::rotate(m_scheduledSlotsTable.begin(),
              m_scheduledSlotsTable.begin() + 1,
              m_scheduledSlotsTable.end());

  // Assign a new empty map to the last slot
  m_scheduledSlotsTable[m_nrofSlots - 1] = std::vector<SchedulingInfo>();

  return true;
}

//-----------------------------------------------------------------------------
bool Intersection::contains(const std::vector<Trajectory> &a_v, Trajectory a_val)
{
  std::cout << a_val << std::endl;
  return !a_v.empty() && (std::find(a_v.begin(), a_v.end(), a_val) != a_v.end());
}

//-----------------------------------------------------------------------------
int Intersection::determineFirstAccessibleSlot(float a_intersectionAccessTime)
{
  odcore::data::TimeStamp now;
  float currentTime = now.toMicroseconds();
  // seconds -> microseconds
  float intersectionAccessTime = a_intersectionAccessTime*1000*1000;

  int firstAccessibleSlot = ceil((intersectionAccessTime - currentTime) / m_slotDuration) + 1;

  return firstAccessibleSlot;
}

//-----------------------------------------------------------------------------
void Intersection::addScheduledVehicleToSlot(int a_slot, int a_vehicleID, SchedulingInfo a_info)
{
  m_scheduledSlotsTable[a_slot][a_vehicleID] = a_info;
}

//-----------------------------------------------------------------------------
void Intersection::setUpTrajectories()
{
  cout << "Intersection::setUpTrajectories()" << endl;
  // Define all valid trajectories in a member variable
  m_allTrajectories = { Trajectory::WS, Trajectory::WR, Trajectory::WL,
                        Trajectory::SS, Trajectory::SR, Trajectory::SL,
                        Trajectory::ES, Trajectory::ER, Trajectory::EL,
                        Trajectory::NS, Trajectory::NR, Trajectory::NL};

  cout << "\t m_allTrajectories \t DONE!" << endl;

  m_trajectoryLookUp["WS"] = Trajectory::WS;
  m_trajectoryLookUp["WR"] = Trajectory::WR;
  m_trajectoryLookUp["WL"] = Trajectory::WL;

  m_trajectoryLookUp["SR"] = Trajectory::SR;
  m_trajectoryLookUp["SS"] = Trajectory::SS;
  m_trajectoryLookUp["SL"] = Trajectory::SL;

  m_trajectoryLookUp["ES"] = Trajectory::ES;
  m_trajectoryLookUp["ER"] = Trajectory::ER;
  m_trajectoryLookUp["EL"] = Trajectory::EL;

  m_trajectoryLookUp["NS"] = Trajectory::NS;
  m_trajectoryLookUp["NR"] = Trajectory::NR;
  m_trajectoryLookUp["NL"] = Trajectory::NL;

  cout << "\t m_trajectoryLookUp \t DONE!" << endl;

  // Define the compatible trajectories
  // Coming from West
  std::vector<Trajectory> ws_compatible = { Trajectory::ER,
                                            Trajectory::NR,
                                            Trajectory::ES };
  std::vector<Trajectory> wr_compatible = { Trajectory::ER,
                                            Trajectory::NR,
                                            Trajectory::ES,
                                            Trajectory::SL,
                                            Trajectory::SS,
                                            Trajectory::NL,
                                            Trajectory::SR};
  std::vector<Trajectory> wl_compatible = { Trajectory::NR,
                                            Trajectory::SR,
                                            Trajectory::EL };

  cout << "\t ws_compatible, wr_compatible, wl_compatible \t DONE!" << endl;

  // Coming from South
  std::vector<Trajectory> ss_compatible = { Trajectory::NR,
                                            Trajectory::WR,
                                            Trajectory::NS };
  std::vector<Trajectory> sr_compatible = { Trajectory::NR,
                                            Trajectory::WR,
                                            Trajectory::NS,
                                            Trajectory::EL,
                                            Trajectory::ES,
                                            Trajectory::WL,
                                            Trajectory::ER};
  std::vector<Trajectory> sl_compatible = { Trajectory::WR,
                                            Trajectory::ER,
                                            Trajectory::NL };

  // Coming from East
  std::vector<Trajectory> es_compatible = { Trajectory::WR,
                                            Trajectory::SR,
                                            Trajectory::WS };
  std::vector<Trajectory> er_compatible = { Trajectory::WR,
                                            Trajectory::SR,
                                            Trajectory::WS,
                                            Trajectory::NL,
                                            Trajectory::NS,
                                            Trajectory::SL,
                                            Trajectory::NR};
  std::vector<Trajectory> el_compatible = { Trajectory::SR,
                                            Trajectory::NR,
                                            Trajectory::WL };

  // Coming from North
  std::vector<Trajectory> ns_compatible = { Trajectory::SR,
                                            Trajectory::ER,
                                            Trajectory::SS };
  std::vector<Trajectory> nr_compatible = { Trajectory::SR,
                                            Trajectory::ER,
                                            Trajectory::SS,
                                            Trajectory::WL,
                                            Trajectory::WS,
                                            Trajectory::EL,
                                            Trajectory::WR};
  std::vector<Trajectory> nl_compatible = { Trajectory::ER,
                                            Trajectory::WR,
                                            Trajectory::SL };

  m_compatibleTrajectories[Trajectory::WS] = ws_compatible;
  m_compatibleTrajectories[Trajectory::WR] = wr_compatible;
  m_compatibleTrajectories[Trajectory::WL] = wl_compatible;
  m_compatibleTrajectories[Trajectory::SS] = ss_compatible;
  m_compatibleTrajectories[Trajectory::SR] = sr_compatible;
  m_compatibleTrajectories[Trajectory::SL] = sl_compatible;
  m_compatibleTrajectories[Trajectory::ES] = es_compatible;
  m_compatibleTrajectories[Trajectory::ER] = er_compatible;
  m_compatibleTrajectories[Trajectory::EL] = el_compatible;
  m_compatibleTrajectories[Trajectory::NS] = ns_compatible;
  m_compatibleTrajectories[Trajectory::NR] = nr_compatible;
  m_compatibleTrajectories[Trajectory::NL] = nl_compatible;

  cout << "\t m_compatibleTrajectories \t DONE!" << endl;

}

} // coordination
} // logic
} // opendlv
