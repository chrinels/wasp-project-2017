#ifndef LOGIC_LEGACY_STATEESTIMATOR_VELOCITYMEASUREMENTMODEL_HPP_
#define LOGIC_LEGACY_STATEESTIMATOR_VELOCITYMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>

#include "SystemModel.hpp"

namespace opendlv {
namespace logic {
namespace legacy {

/**
 * @brief Measurement vector
 *
 * @param T Numeric scalar type
 */
template<typename T>
class VelocityMeasurement : public Kalman::Vector<T, 2>
{
public:
    KALMAN_VECTOR(VelocityMeasurement, T, 2)
    
    //! X coordinate
    static constexpr size_t VX = 0;
    
    //! Y coordinate
    static constexpr size_t VY = 1;
    
    T vx()       const { return (*this)[ VX ]; }
    T vy()       const { return (*this)[ VY ]; }
    
    T& vx()      { return (*this)[ VX ]; }
    T& vy()      { return (*this)[ VY ]; }
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
class VelocityMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, VelocityMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef State<T> S;
    
    //! Measurement type shortcut definition
    typedef VelocityMeasurement<T> M;


    VelocityMeasurementModel(T variance)
    {        
        // Setup noise jacobian.
        this->H.setZero();
        this->H( M::VX, S::VX ) = 1;
        this->H( M::VY, S::VY ) = 1;
        this->V.setIdentity();
        this->V*=variance;
    }
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        M measurement;

        measurement.vx() = x.vx();
        measurement.vy() = x.vy();
        
        return measurement;
    }

};

} 
} 
}


#endif