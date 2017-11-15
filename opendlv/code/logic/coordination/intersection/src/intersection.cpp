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

#include <iostream>


#include <opendavinci/odcore/data/Container.h>
#include "opendavinci/odcore/data/TimeStamp.h"

#include <opendavinci/odcore/io/conference/ContainerConference.h>

#include <opendlv/data/environment/Line.h>
#include <opendlv/data/environment/Obstacle.h>
#include <opendlv/data/environment/Polygon.h>
#include <opendlv/data/environment/WGS84Coordinate.h>

#include <automotivedata/GeneratedHeaders_AutomotiveData.h>
#include <odvdopendlvdata/GeneratedHeaders_ODVDOpenDLVData.h>
#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>
#include <odvdimu/GeneratedHeaders_ODVDIMU.h>

#include "intersection.hpp"

namespace opendlv {
namespace logic {
namespace coordination {

Intersection::Intersection(int32_t const &a_argc, char **a_argv)
  : DataTriggeredConferenceClientModule(a_argc, a_argv,
      "logic-coordination-intersection"),
    m_initialised(false),
    m_compatible_trajectories(),
    m_slot_duration(5.0),
    m_nrof_slots(20)
{
}

Intersection::~Intersection()
{
}

void Intersection::setUp()
{
    // Extract parameter values
    odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();
    m_slot_duration = kv.getValue<float>(
        "logic-coordination-intersection.slot_duration");
    m_nrof_slots = kv.getValue<float>(
        "logic-coordination-intersection.nrof_slots");

    setUpCompatibleTrajectories();

    m_initialised = true;
}

void Intersection::tearDown()
{
}

void Intersection::nextContainer(odcore::data::Container &a_container)
{
  cout << "Message has dataType ID = " << opendlv::knowledge::Message::ID() << endl;
  cout << " - Received dataType ID = " << a_container.getDataType() << endl;
  if(a_container.getDataType() != opendlv::knowledge::Message::ID()){
    cout << "Not a message! Returning!" << endl;
    return;
  }
}

void Intersection::setUpCompatibleTrajectories()
{
    // Define the compatible trajectories
    // Coming from West
    std::vector<VALID_TRAJECTORY> ws_compatible = { VALID_TRAJECTORY::ER,
                                 		    VALID_TRAJECTORY::NR,
		                                    VALID_TRAJECTORY::ES };
    std::vector<VALID_TRAJECTORY> wr_compatible = { VALID_TRAJECTORY::ER,
                                 		    VALID_TRAJECTORY::NR,
		                                    VALID_TRAJECTORY::ES,
                                                    VALID_TRAJECTORY::SL,
                                                    VALID_TRAJECTORY::SS,
                                                    VALID_TRAJECTORY::NL,
                                                    VALID_TRAJECTORY::SR};
    std::vector<VALID_TRAJECTORY> wl_compatible = { VALID_TRAJECTORY::NR,
                                 		    VALID_TRAJECTORY::SR,
		                                    VALID_TRAJECTORY::EL };

    // Coming from South
    std::vector<VALID_TRAJECTORY> ss_compatible = { VALID_TRAJECTORY::NR,
                                 		    VALID_TRAJECTORY::WR,
		                                    VALID_TRAJECTORY::NS };
    std::vector<VALID_TRAJECTORY> sr_compatible = { VALID_TRAJECTORY::NR,
                                 		    VALID_TRAJECTORY::WR,
		                                    VALID_TRAJECTORY::NS,
                                                    VALID_TRAJECTORY::EL,
                                                    VALID_TRAJECTORY::ES,
                                                    VALID_TRAJECTORY::WL,
                                                    VALID_TRAJECTORY::ER};
    std::vector<VALID_TRAJECTORY> sl_compatible = { VALID_TRAJECTORY::WR,
                                 		    VALID_TRAJECTORY::ER,
		                                    VALID_TRAJECTORY::NL };

    // Coming from East
    std::vector<VALID_TRAJECTORY> es_compatible = { VALID_TRAJECTORY::WR,
                                 		    VALID_TRAJECTORY::SR,
		                                    VALID_TRAJECTORY::WS };
    std::vector<VALID_TRAJECTORY> er_compatible = { VALID_TRAJECTORY::WR,
                                 		    VALID_TRAJECTORY::SR,
		                                    VALID_TRAJECTORY::WS,
                                                    VALID_TRAJECTORY::NL,
                                                    VALID_TRAJECTORY::NS,
                                                    VALID_TRAJECTORY::SL,
                                                    VALID_TRAJECTORY::NR};
    std::vector<VALID_TRAJECTORY> el_compatible = { VALID_TRAJECTORY::SR,
                                 		    VALID_TRAJECTORY::NR,
		                                    VALID_TRAJECTORY::WL };

    // Coming from North
    std::vector<VALID_TRAJECTORY> ns_compatible = { VALID_TRAJECTORY::SR,
                                 		    VALID_TRAJECTORY::ER,
		                                    VALID_TRAJECTORY::SS };
    std::vector<VALID_TRAJECTORY> nr_compatible = { VALID_TRAJECTORY::SR,
                                 		    VALID_TRAJECTORY::ER,
		                                    VALID_TRAJECTORY::SS,
                                                    VALID_TRAJECTORY::WL,
                                                    VALID_TRAJECTORY::WS,
                                                    VALID_TRAJECTORY::EL,
                                                    VALID_TRAJECTORY::WR};
    std::vector<VALID_TRAJECTORY> nl_compatible = { VALID_TRAJECTORY::ER,
                                 		    VALID_TRAJECTORY::WR,
		                                    VALID_TRAJECTORY::SL };

    m_compatible_trajectories[VALID_TRAJECTORY::WS] = ws_compatible;
    m_compatible_trajectories[VALID_TRAJECTORY::WR] = ws_compatible;
    m_compatible_trajectories[VALID_TRAJECTORY::WL] = ws_compatible;
    m_compatible_trajectories[VALID_TRAJECTORY::SS] = ss_compatible;
    m_compatible_trajectories[VALID_TRAJECTORY::SR] = ss_compatible;
    m_compatible_trajectories[VALID_TRAJECTORY::SL] = ss_compatible;
    m_compatible_trajectories[VALID_TRAJECTORY::ES] = es_compatible;
    m_compatible_trajectories[VALID_TRAJECTORY::ER] = es_compatible;
    m_compatible_trajectories[VALID_TRAJECTORY::EL] = es_compatible;
    m_compatible_trajectories[VALID_TRAJECTORY::NS] = ns_compatible;
    m_compatible_trajectories[VALID_TRAJECTORY::NR] = ns_compatible;
    m_compatible_trajectories[VALID_TRAJECTORY::NL] = ns_compatible;

}

} // coordination
} // logic
} // opendlv
