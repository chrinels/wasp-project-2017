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
#include <iomanip>

#include <opendavinci/odcore/data/Container.h>

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
    m_allTrajectories(),
    m_trajectoryLookUp(),
    m_compatibleTrajectories(),
    m_scheduledSlotsTable(),
    m_rotationTime()
{
}

//-----------------------------------------------------------------------------
Intersection::~Intersection()
{
}

void Intersection::setUp()
{
  // Extract parameter values
  odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();
  m_slotDuration = kv.getValue<double>(
      "logic-coordination-intersection.slot_duration");
  m_nrofSlots = kv.getValue<int>(
      "logic-coordination-intersection.nrof_slots");

  setUpTrajectories();

  // Create empty maps to store trajectory information for each slot
  for(int slot = 0; slot < m_nrofSlots; ++slot) {
    m_scheduledSlotsTable.push_back(std::vector<SchedulingInfo>());
  }

  m_initialised = true;
}

//-----------------------------------------------------------------------------
void Intersection::tearDown()
{
}

//-----------------------------------------------------------------------------
void Intersection::nextContainer(odcore::data::Container &a_container)
{
  if(!m_initialised) return;

  auto timeSent = a_container.getSentTimeStamp();
  auto timeReceived = a_container.getReceivedTimeStamp();

  if (a_container.getDataType() == opendlv::logic::coordination::IntersectionAccessRequest::ID()) {
    auto accessRequest = a_container.getData<opendlv::logic::coordination::IntersectionAccessRequest>();
    cout << "opendlv::logic::coordination::IntersectionAccessRequest::ID = " << opendlv::logic::coordination::IntersectionAccessRequest::ID() << endl;
    cout << "opendlv::logic::coordination::IntersectionAccessRequest::vehicleID = " << accessRequest.getVehicleID() << endl;
    cout << "opendlv::logic::coordination::IntersectionAccessRequest::velocity = " << accessRequest.getVelocity() << endl;
    cout << "opendlv::logic::coordination::IntersectionAccessRequest::distanceToIntersection = " << accessRequest.getDistanceToIntersection() << endl;
    cout << "opendlv::logic::coordination::IntersectionAccessRequest::plannedTrajectory = " << accessRequest.getPlannedTrajectory() << endl;
    scheduleVehicle(accessRequest);
    printTimeSlotTable();  
  }

}

//-----------------------------------------------------------------------------
bool Intersection::scheduleVehicle(const opendlv::logic::coordination::IntersectionAccessRequest &a_accessReq)
{
  bool schedulingSuccessful = false;

  odcore::base::Lock l(m_timeRefreshMutex);

  int vehicleID = a_accessReq.getVehicleID();
  double currentVelocity = a_accessReq.getVelocity();
  double distanceToIntersection = a_accessReq.getDistanceToIntersection();

  Trajectory plannedTrajectory = m_trajectoryLookUp[a_accessReq.getPlannedTrajectory()];

  if (currentVelocity <= 0) return schedulingSuccessful;

  double intersectionAccessTime = distanceToIntersection/currentVelocity; // Distance in seconds
  if (std::isnan(intersectionAccessTime) || intersectionAccessTime < 1) return schedulingSuccessful;

  // Determine the first slot after the vehicle's access time
  int startSlot = ceil(intersectionAccessTime/(m_slotDuration*1000*1000)) + 1;
  if(startSlot < 0 || startSlot > m_nrofSlots) return schedulingSuccessful;

  SchedulingInfo schedInfo;

  // Find the first slot that does not contain any incompatible trajectories
  for(int slot = startSlot; slot < m_nrofSlots; ++slot) {

    if (schedulingSuccessful) break;

    // Find compatible trajectories for this slot
    std::vector<Trajectory> validTrajectories = m_allTrajectories;
    
    std::vector<SchedulingInfo> scheduledAtSlot = m_scheduledSlotsTable[slot];

    /**
     * At the current time slot, check if the vector of already scheduled
     * vehicles is empty -> if true: just add the new vehicle.
     * 
     * If not, check if the planned trajectories of the already 
     * scheduled vehicles is compatible with the desired trajectory.
     * 
     * If not, go to the next slot and try there instead.
     * 
     */
    if (scheduledAtSlot.empty()) {
      // Generate the scheduling info
      schedInfo.intersectionAccessTime = intersectionAccessTime;
      schedInfo.trajectory = plannedTrajectory;
      schedInfo.vehicleID = vehicleID;

      addScheduledVehicleToSlot(slot, schedInfo);
      schedulingSuccessful = true;

    } else {

      bool slotIsCompatible = true;
      for(std::vector<SchedulingInfo>::size_type i = 0; i != scheduledAtSlot.size(); i++) {
        
        SchedulingInfo slotSchedulingInfo = scheduledAtSlot[i];

        // Is the vehicle already scheduled?!
        if (vehicleID == slotSchedulingInfo.vehicleID) return false;

        Trajectory scheduledTrajectory = slotSchedulingInfo.trajectory;
        std::vector<Trajectory> compatibleTrajectories = m_compatibleTrajectories[scheduledTrajectory];

        if (!contains(compatibleTrajectories, plannedTrajectory)) {
          slotIsCompatible = false;
          break;
        }
      }
      if (slotIsCompatible) {
        // Generate the scheduling info
        schedInfo.intersectionAccessTime = intersectionAccessTime;
        schedInfo.trajectory = plannedTrajectory;
        schedInfo.vehicleID = vehicleID;

        addScheduledVehicleToSlot(slot, schedInfo);
        schedulingSuccessful = true;
      }
    }

  }

  if (schedulingSuccessful) {
    odcore::data::TimeStamp now;
    odcore::data::TimeStamp entryTime = now + odcore::data::TimeStamp(ceil(schedInfo.intersectionAccessTime), 0);
    odcore::data::TimeStamp exitTime = entryTime + odcore::data::TimeStamp(m_slotDuration, 0);
    opendlv::logic::legacy::TimeSlot timeSlot;
    timeSlot.setVehicleID(vehicleID);
    timeSlot.setEntryTime(entryTime);
    timeSlot.setExitTime(exitTime);
    cout << "Beaconing information" << endl;
    odcore::data::Container c_intersectionAccess(timeSlot);
    getConference().send(c_intersectionAccess);
  }

  return schedulingSuccessful;
}


//-----------------------------------------------------------------------------
bool Intersection::timeRefreshSlotsTable()
{
  odcore::base::Lock l(m_timeRefreshMutex);
  odcore::data::TimeStamp now;
  odcore::data::TimeStamp rotationTime = now - odcore::data::TimeStamp(m_slotDuration,0);
  if (rotationTime >= m_rotationTime) {
    // Shift all slots to the left
    std::rotate(m_scheduledSlotsTable.begin(),
                m_scheduledSlotsTable.begin() + 1,
                m_scheduledSlotsTable.end());

    // Assign a new empty map to the last slot
    m_scheduledSlotsTable[m_nrofSlots - 1] = std::vector<SchedulingInfo>();
    m_rotationTime = now;
  }
  return true;
}

//-----------------------------------------------------------------------------
bool Intersection::contains(const std::vector<Trajectory> &a_v, Trajectory a_val) const
{
  return !a_v.empty() && (std::find(a_v.begin(), a_v.end(), a_val) != a_v.end());
}

//-----------------------------------------------------------------------------
void Intersection::addScheduledVehicleToSlot(int a_slot, SchedulingInfo a_info)
{
  std::vector<SchedulingInfo> atslot = m_scheduledSlotsTable[a_slot];
  atslot.push_back(a_info);
  m_scheduledSlotsTable[a_slot] = atslot;
}

void Intersection::printTimeSlotTable() 
{
  cout << setw(5) << "Slot" << setw(20) << "Scheduled" << endl;

  for(std::vector<std::vector<SchedulingInfo>>::size_type i = 0; i != m_scheduledSlotsTable.size(); i++) {
    
    std::vector<SchedulingInfo> scheduledAtSlot = m_scheduledSlotsTable[i];

    if(scheduledAtSlot.empty()) {
      cout << setw(5) << i+1 << setw(20) << "-" << endl;
      continue;
    }

    for(std::vector<SchedulingInfo>::size_type j = 0; j != scheduledAtSlot.size(); j++) {
      SchedulingInfo scheduledVehicle = scheduledAtSlot[j];

      int32_t accessSeconds = floor(scheduledVehicle.intersectionAccessTime/(1000*1000));
      int32_t accessMicrosends = scheduledVehicle.intersectionAccessTime - accessSeconds*1000*1000;
      odcore::data::TimeStamp accessTime(accessSeconds, accessMicrosends);
      cout <<  setw(5) << i+1 << "," << j+1 << setw(20) << scheduledVehicle.vehicleID << "\tAccess Time: "<< accessTime.getYYYYMMDD_HHMMSS() << endl;

      opendlv::logic::coordination::IntersectionSchedulerDebug debugMsg;
      debugMsg.setVehicleID(scheduledVehicle.vehicleID);
      debugMsg.setTime(accessTime);
      debugMsg.setTimeSlot(i);
      odcore::data::Container c_debugMsg(debugMsg);
      getConference().send(c_debugMsg);
    }
  }
}

//-----------------------------------------------------------------------------
void Intersection::setUpTrajectories()
{
  // Define all valid trajectories in a member variable
  m_allTrajectories = { Trajectory::WS, Trajectory::WR, Trajectory::WL,
                        Trajectory::SS, Trajectory::SR, Trajectory::SL,
                        Trajectory::ES, Trajectory::ER, Trajectory::EL,
                        Trajectory::NS, Trajectory::NR, Trajectory::NL};

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

}

} // coordination
} // logic
} // opendlv
