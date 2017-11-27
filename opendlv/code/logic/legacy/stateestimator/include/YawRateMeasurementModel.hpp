#ifndef LOGIC_LEGACY_STATEESTIMATOR_YAWRATEMEASUREMENTMODEL_HPP_
#define LOGIC_LEGACY_STATEESTIMATOR_YAWRATEMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>

#include "SystemModel.hpp"

namespace opendlv {
namespace logic {
namespace legacy {

/**
 * @brief Measurement vector measuring an angular velocity
 *
 * @param T Numeric scalar type
 */
template<typename T>
class YawRateMeasurement : public Kalman::Vector<T, 1>
{
public:
    KALMAN_VECTOR(YawRateMeasurement, T, 1)
    
    //! Yaw rate
    static constexpr size_t R = 0;
    
    T r()  const { return (*this)[ R ]; }
    T& r() { return (*this)[ R ]; }
};

/**
 * @brief Measurement model 
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class YawRateMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, YawRateMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef State<T> S;
    
    //! Measurement type shortcut definition
    typedef YawRateMeasurement<T> M;
    
    YawRateMeasurementModel(T variance)
    {
        // Setup jacobians.
        this->H.setZero();
        this->H( M::R, S::DPSI ) = 1;
        this->H( M::R, S::BDPSI ) = 1;
        this->V.setIdentity();
        this->V*=variance;
    }
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        M measurement;
        
        // Measurement equation
        measurement.r() = x.dpsi() + x.bdpsi();
        
        return measurement;
    }
};

}
}
}

#endif