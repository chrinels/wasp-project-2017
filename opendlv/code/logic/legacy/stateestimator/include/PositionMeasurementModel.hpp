#ifndef LOGIC_LEGACY_STATEESTIMATOR_POSITIONMEASUREMENTMODEL_HPP_
#define LOGIC_LEGACY_STATEESTIMATOR_POSITIONMEASUREMENTMODEL_HPP_

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
class PositionMeasurement : public Kalman::Vector<T, 2>
{
public:
    KALMAN_VECTOR(PositionMeasurement, T, 2)
    
    //! X coordinate
    static constexpr size_t X = 0;
    
    //! Y coordinate
    static constexpr size_t Y = 1;
    
    T x()       const { return (*this)[ X ]; }
    T y()       const { return (*this)[ Y ]; }
    
    T& x()      { return (*this)[ X ]; }
    T& y()      { return (*this)[ Y ]; }
};

/**
 * @brief Measurement model 
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class PositionMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, PositionMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef State<T> S;
    
    //! Measurement type shortcut definition
    typedef PositionMeasurement<T> M;
    
    /**
     * @brief Constructor
     */
    PositionMeasurementModel(T variance)
    {        
        // Setup noise jacobian.
        this->H.setZero();
        this->H( M::X, S::PX ) = 1;
        this->H( M::Y, S::PY ) = 1;
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

        measurement.x() = x.px();
        measurement.y() = x.py();
        
        return measurement;
    }

};

}
}
}

#endif