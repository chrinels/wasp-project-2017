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
    m_slotTableAbsoluteTime()
{
}

//-----------------------------------------------------------------------------
Intersection::~Intersection()
{
}

//-----------------------------------------------------------------------------
void Intersection::setUp()
{
  // Extract parameter values
  odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();
  m_slotDuration = kv.getValue<double>(
      "logic-coordination-intersection.slot_duration");
  m_nrofSlots = kv.getValue<int>(
      "logic-coordination-intersection.nrof_slots");

  setUpTrajectories();

  // Initialize slot table that stores scheduling information
  odcore::data::TimeStamp currentTime;
  m_slotTableAbsoluteTime = currentTime;
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
  if(!m_initialised) {
    return;
  }


  auto timeSent = a_container.getSentTimeStamp();
  auto timeReceived = a_container.getReceivedTimeStamp();

  if (a_container.getDataType() == opendlv::logic::coordination::IntersectionAccessRequest::ID()) {

    timeRefreshSlotsTable();

    auto accessRequest = a_container.getData<opendlv::logic::coordination::IntersectionAccessRequest>();

    if(odcore::base::module::AbstractCIDModule::isVerbose()) {
      // Print intersection request details
      cout << "|Scheduler| IntersectionAccessRequest::ID = " << opendlv::logic::coordination::IntersectionAccessRequest::ID() << endl;
      cout << "|Scheduler| IntersectionAccessRequest::vehicleID = " << accessRequest.getVehicleID() << endl;
      cout << "|Scheduler| IntersectionAccessRequest::velocity = " << accessRequest.getVelocity() << endl;
      cout << "|Scheduler| IntersectionAccessRequest::distanceToIntersection = " << accessRequest.getDistanceToIntersection() << endl;
      cout << "|Scheduler| IntersectionAccessRequest::plannedTrajectory = " << accessRequest.getPlannedTrajectory() << endl;
    }
    int vehicleID = accessRequest.getVehicleID();
    if(!vehicleAlreadyScheduled(vehicleID)) {
      if(odcore::base::module::AbstractCIDModule::isVerbose()) {
        cout << "|Scheduler| Attempting to schedule vehicle " << vehicleID << endl;
      }
      scheduleVehicle(accessRequest);
      if(odcore::base::module::AbstractCIDModule::isVerbose()) {
        printTimeSlotTable();
      }
    }
  }
}

//-----------------------------------------------------------------------------
odcore::data::TimeStamp Intersection::getSlotAbsoluteTime(int slot)
{
  
   return m_slotTableAbsoluteTime + odcore::data::TimeStamp(slot * m_slotDuration, 0);
}

//-----------------------------------------------------------------------------
bool Intersection::vehicleAlreadyScheduled(int vehicleID) {
  // Check if the vehicle is already scheduled in any slot
  for(int slot = 0; slot < m_nrofSlots; ++slot) {
    std::vector<SchedulingInfo> scheduledAtSlot = m_scheduledSlotsTable[slot];
    for(unsigned int i = 0; i < scheduledAtSlot.size(); ++i) {
      if (vehicleID == scheduledAtSlot[i].vehicleID) {
        if(odcore::base::module::AbstractCIDModule::isVerbose()) {
          cout << "|Scheduler| Vehicle already scheduled: " << vehicleID 
               << " at slot " << slot + 1 << endl;
        }
        return true;
      }
    }
  }
  return false;
}

//-----------------------------------------------------------------------------
bool Intersection::scheduleVehicle(const opendlv::logic::coordination::IntersectionAccessRequest &a_accessReq)
{
  bool schedulingSuccessful = false;

  odcore::base::Lock l(m_timeRefreshMutex);

  int vehicleID = a_accessReq.getVehicleID();
  double currentVelocity = a_accessReq.getVelocity();
  double distanceToIntersection = a_accessReq.getDistanceToIntersection();
  double timeToIntersection = distanceToIntersection / currentVelocity; // [s]
  
  cout << "|Scheduler| Time to intersection [s]: " << timeToIntersection << endl;

  odcore::data::TimeStamp currentTime;
  odcore::data::TimeStamp intersectionAccessTime = currentTime + odcore::data::TimeStamp(timeToIntersection, 0);
             
  Trajectory plannedTrajectory = m_trajectoryLookUp[a_accessReq.getPlannedTrajectory()];

  if (currentVelocity <= 0.0) {
    if(odcore::base::module::AbstractCIDModule::isVerbose()) {
      cout << "|Scheduler| Invalid current velocity " << currentVelocity << endl;
    }
    return false; // Cannot schedule
  }

  if (std::isnan(timeToIntersection) || timeToIntersection < 1.0) {
    if(odcore::base::module::AbstractCIDModule::isVerbose()) {
      cout << "|Scheduler| Invalid access time " << timeToIntersection << endl;
    }
    return false;  // Cannot schedule
  }

  // Determine the first slot after the vehicle's access time
  int startSlot = ceil(timeToIntersection / m_slotDuration);
  
  if(startSlot < 0 || startSlot > m_nrofSlots) {
    if(odcore::base::module::AbstractCIDModule::isVerbose()) {
      cout << "|Scheduler| Invalid start slot " << startSlot << endl;
    }
    return false; // Cannot schedule
  }

  // Create a container to store scheduling info
  SchedulingInfo schedInfo;

  // Find the first slot that does not contain any incompatible trajectories
  int currentSlot = startSlot;
  while(currentSlot < m_nrofSlots) {

    std::vector<SchedulingInfo> scheduledAtSlot = m_scheduledSlotsTable[currentSlot];

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
    bool slotIsCompatible = true;
    if (!scheduledAtSlot.empty()) {

      // Find compatible trajectories for this slot
      std::vector<Trajectory> validTrajectories = m_allTrajectories;
      for(unsigned int i = 0; i < scheduledAtSlot.size(); i++) {
        
        SchedulingInfo slotSchedulingInfo = scheduledAtSlot[i];

        Trajectory scheduledTrajectory = slotSchedulingInfo.trajectory;
        std::vector<Trajectory> compatibleTrajectories = m_compatibleTrajectories[scheduledTrajectory];

        if (!contains(compatibleTrajectories, plannedTrajectory)) {
          slotIsCompatible = false;
          break;
        }
      }
    }

    if (slotIsCompatible) {
      // Generate the scheduling info
      schedInfo.vehicleID = vehicleID;
      schedInfo.trajectory = plannedTrajectory;
      schedInfo.scheduledSlotStartTime = getSlotAbsoluteTime(currentSlot); 
      schedInfo.intersectionAccessTime = intersectionAccessTime; 

      addScheduledVehicleToSlot(currentSlot, schedInfo);
      schedulingSuccessful = true;

      break; // No need to check any more slots
    }
    
    ++currentSlot;
  }

  if (schedulingSuccessful) {
    odcore::data::TimeStamp entryTime = schedInfo.scheduledSlotStartTime;
    odcore::data::TimeStamp exitTime = entryTime + odcore::data::TimeStamp(m_slotDuration, 0);
    opendlv::logic::legacy::TimeSlot timeSlot;
    timeSlot.setVehicleID(vehicleID);
    timeSlot.setEntryTime(entryTime);
    timeSlot.setExitTime(exitTime);
    if(odcore::base::module::AbstractCIDModule::isVerbose()) {
      cout << "|Scheduler| Succesfully scheduled vehicle " << vehicleID << endl;
    }
    odcore::data::Container c_intersectionAccess(timeSlot);
    getConference().send(c_intersectionAccess);
  } else {
    if(odcore::base::module::AbstractCIDModule::isVerbose()) {
      cout << "|Scheduler| No available slot found for vehicle " << vehicleID << endl;
    }
  }
  
  return schedulingSuccessful; 
}


//-----------------------------------------------------------------------------
bool Intersection::timeRefreshSlotsTable()
{
  odcore::base::Lock l(m_timeRefreshMutex);
  odcore::data::TimeStamp currentTime;
  double elapsedTime = (currentTime - m_slotTableAbsoluteTime).toMicroseconds()*1.0/1000000L;

  int elapsedSlots = floor(elapsedTime / m_slotDuration);

  if (elapsedSlots > 0) {

    cout << "|Scheduler| Updating slots. Elapsed (time, slots): (" 
         << elapsedTime << "," << elapsedSlots << ")" << "\n";

    std::rotate(m_scheduledSlotsTable.begin(),
                m_scheduledSlotsTable.begin() + elapsedSlots,
                m_scheduledSlotsTable.end());

    // Assign a new empty map to the last slot
    m_scheduledSlotsTable[m_nrofSlots - 1] = std::vector<SchedulingInfo>();

    m_slotTableAbsoluteTime = currentTime;

    printTimeSlotTable();
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

//-----------------------------------------------------------------------------
void Intersection::printTimeSlotTable() 
{
  cout << "|Scheduler| ---------Time Slot Table--------\n";
  cout << "|Scheduler| Slot \t Vehicle ID \t\t Start / Access Time\n";

  for(unsigned int slot = 0; slot < m_nrofSlots; ++slot) {
    
    std::vector<SchedulingInfo> scheduledAtSlot = m_scheduledSlotsTable[slot];

    cout << "|Scheduler| " << slot + 1<< "\t\t" << "-" << "\t\t"
         << getSlotAbsoluteTime(slot).getYYYYMMDD_HHMMSS() << "\n";

    // Print scheduled vehicle info, if any
    if(!scheduledAtSlot.empty()) {
      for(unsigned int i = 0; i < scheduledAtSlot.size(); ++i) {
        SchedulingInfo scheduledVehicle = scheduledAtSlot[i];

        cout << "|Scheduler| " << slot + 1 << "," << i+1 << "\t\t" 
             << scheduledVehicle.vehicleID << "\t\t-> "
             << scheduledVehicle.intersectionAccessTime.getYYYYMMDD_HHMMSS() 
             << " (original estimate)"
             << "\n";

        opendlv::logic::coordination::IntersectionSchedulerDebug debugMsg;
        debugMsg.setVehicleID(scheduledVehicle.vehicleID);
        debugMsg.setTime(scheduledVehicle.intersectionAccessTime);
        debugMsg.setTimeSlot(slot);
        odcore::data::Container c_debugMsg(debugMsg);
        getConference().send(c_debugMsg);
      }
    }
  }
  cout << endl; // Flush stdout
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
