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

#include <kalman/ExtendedKalmanFilter.hpp>
#include "SystemModel.hpp"
#include "PositionMeasurementModel.hpp"
#include "OrientationMeasurementModel.hpp"
#include "YawRateMeasurementModel.hpp"

typedef double T;

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

  opendlv::data::environment::WGS84Coordinate m_wgs84Reference;

  Kalman::ExtendedKalmanFilter<State<T>> m_ekf;
  SystemModel<T> m_systemModel;
  PositionMeasurementModel<T> m_positionModel;
  OrientationMeasurementModel<T> m_orientationModel;
  YawRateMeasurementModel<T> m_yawRateModel;
};

}
}
}

#endif
