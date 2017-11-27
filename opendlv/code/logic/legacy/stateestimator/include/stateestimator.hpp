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

#ifndef LOGIC_LEGACY_STATEESTIMATOR_STATEESTIMATOR_HPP
#define LOGIC_LEGACY_STATEESTIMATOR_STATEESTIMATOR_HPP

#include <memory>
#include <queue>

#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/base/Mutex.h>
#include <opendlv/data/environment/Point3.h>
#include <opendlv/data/environment/WGS84Coordinate.h>
#include <opendavinci/odcore/data/TimeStamp.h>

namespace opendlv {
namespace logic {
namespace legacy {

class StateEstimator : public odcore::base::module::TimeTriggeredConferenceClientModule {
 public:
  StateEstimator(int32_t const &, char **);
  StateEstimator(StateEstimator const &) = delete;
  StateEstimator &operator=(StateEstimator const &) = delete;
  virtual ~StateEstimator();

  virtual void nextContainer(odcore::data::Container &c);
  odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

 private:
  virtual void setUp();
  virtual void tearDown();

  odcore::base::Mutex m_stateMutex;
  opendlv::data::environment::Point3 m_position;
  opendlv::data::environment::Point3 m_velocity;
  double m_orientation;
  double m_yawRate;

  opendlv::data::environment::WGS84Coordinate m_wgs84Reference;


  odcore::data::TimeStamp m_gpsReadingTimeStamp;
  double m_positionSmoothing;

  odcore::data::TimeStamp m_groundSpeedReadingTimeStamp;
  double m_velocitySmoothing;

  odcore::data::TimeStamp m_orientationReadingTimeStamp;
  double m_orientationSmoothing;
  double m_orientationMinDistance;
  double m_orientationMaxDistance;
  size_t m_maxPositionSize;
  std::queue<opendlv::data::environment::Point3> m_positions;
};

}
}
}

#endif
