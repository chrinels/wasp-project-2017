#ifndef LOGIC_LEGACY_STATEESTIMATOR_ORIENTATIONMEASUREMENTMODEL_HPP_
#define LOGIC_LEGACY_STATEESTIMATOR_ORIENTATIONMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>

#include "SystemModel.hpp"

namespace opendlv {
namespace logic {
namespace legacy {

/**
 * @brief Measurement vector measuring an orientation (i.e. by using a compass)
 *
 * @param T Numeric scalar type
 */
template<typename T>
class OrientationMeasurement : public Kalman::Vector<T, 1>
{
public:
    KALMAN_VECTOR(OrientationMeasurement, T, 1)
    
    //! Orientation
    static constexpr size_t PSI = 0;
    
    T psi()  const { return (*this)[ PSI ]; }
    T& psi() { return (*this)[ PSI ]; }
};

/**
 * @brief Measurement modelr.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class OrientationMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, OrientationMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef State<T> S;
    
    //! Measurement type shortcut definition
    typedef OrientationMeasurement<T> M;
    
    OrientationMeasurementModel(T variance)
    {
        // Setup jacobians.
        this->H.setZero();
        this->H( M::PSI, S::PSI ) = 1;
        this->V.setIdentity();
        this->V*=variance;
    }
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        M measurement;
        
        measurement.psi() = x.psi();
        
        return measurement;
    }
};

}
}
}

#endif