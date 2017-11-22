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

#include <opendavinci/odcore/io/conference/ContainerConference.h>

#include <opendlv/data/environment/Line.h>
#include <opendlv/data/environment/Obstacle.h>
#include <opendlv/data/environment/Polygon.h>
#include <opendlv/data/environment/WGS84Coordinate.h>

#include <automotivedata/GeneratedHeaders_AutomotiveData.h>
#include <odvdopendlvdata/GeneratedHeaders_ODVDOpenDLVData.h>
#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>
#include <odvdimu/GeneratedHeaders_ODVDIMU.h>

#include "v2ibeacon.hpp"

namespace opendlv {
namespace logic {
namespace coordination {

V2IBeacon::V2IBeacon(int32_t const &a_argc, char **a_argv)
  : TimeTriggeredConferenceClientModule(a_argc, a_argv,
      "logic-coordination-v2ibeacon"),
      m_initialized(false)
{
}

V2IBeacon::~V2IBeacon()
{
}

void V2IBeacon::setUp()
{
}

void V2IBeacon::tearDown()
{
}

void V2IBeacon::nextContainer(odcore::data::Container &a_container)
{
  if(!m_initialized) {
    return;
  }
  cout << " Received dataType ID = " << a_container.getDataType() << endl;
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode V2IBeacon::body()
{
  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

} // V2IBeacon
} // logic
} // opendlv
