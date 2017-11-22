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

#ifndef LOGIC_LEGACY_COORDINATION_V2IBEACON_HPP
#define LOGIC_LEGACY_COORDINATION_V2IBEACON_HPP

#include <memory>

#include "opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h"
#include <opendavinci/odcore/base/Mutex.h>
#include <opendlv/data/environment/Point3.h>
#include <opendlv/data/environment/WGS84Coordinate.h>

namespace opendlv {
namespace logic {
namespace coordination {

class V2IBeacon : public odcore::base::module::DataTriggeredConferenceClientModule {
 public:
  V2IBeacon(int32_t const &, char **);
  V2IBeacon(V2IBeacon const &) = delete;
  V2IBeacon &operator=(V2IBeacon const &) = delete;
  virtual ~V2IBeacon();

  virtual void nextContainer(odcore::data::Container &c);

 private:
  void setUp();
  void tearDown();
  
};

}
}
}

#endif
