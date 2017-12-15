#ifndef LOGIC_LEGACY_STATEESTIMATOR_ACCELERATIONMEASUREMENTMODEL_HPP_
#define LOGIC_LEGACY_STATEESTIMATOR_ACCELERATIONMEASUREMENTMODEL_HPP_

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
class AccelerationMeasurement : public Kalman::Vector<T, 2>
{
public:
    KALMAN_VECTOR(AccelerationMeasurement, T, 2)
    
    //! X coordinate
    static constexpr size_t AX = 0;
    
    //! Y coordinate
    static constexpr size_t AY = 1;
    
    T ax()       const { return (*this)[ AX ]; }
    T ay()       const { return (*this)[ AY ]; }
    
    T& ax()      { return (*this)[ AX ]; }
    T& ay()      { return (*this)[ AY ]; }
};


/**
 * @brief Measurement model 
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class AccelerationMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, AccelerationMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef State<T> S;
    
    //! Measurement type shortcut definition
    typedef AccelerationMeasurement<T> M;
    

    AccelerationMeasurementModel(T variance)
    {        
        // Setup noise jacobian.
        this->H.setZero();
        this->H( M::AX, S::AX ) = 1;
        this->H( M::AY, S::AY ) = 1;
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

        measurement.ax() = x.ax();
        measurement.ay() = x.ay();
        
        return measurement;
    }


};

} 
} 
}

#endif