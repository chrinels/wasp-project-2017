#ifndef LOGIC_LEGACY_STATEESTIMATOR_SYSTEMMODEL_HPP_
#define LOGIC_LEGACY_STATEESTIMATOR_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>

namespace opendlv {
namespace logic {
namespace legacy {

template<typename T>
class State : public Kalman::Vector<T, 9>
{
public:
    KALMAN_VECTOR(State, T, 9)
    
    //! X-position
    static constexpr size_t PX = 0;
    //! Y-Position
    static constexpr size_t PY = 1;
    //! X-Velocity
    static constexpr size_t VX = 2;
    //! Y-Velocity
    static constexpr size_t VY = 3;
    //! X-Acceleration
    static constexpr size_t AX = 4;
    //! Y-Acceleration
    static constexpr size_t AY = 5;
    //! Orientation
    static constexpr size_t PSI = 6;
    //! Yaw-rate
    static constexpr size_t DPSI = 7;
    //! Yaw-rate bias
    static constexpr size_t BDPSI = 8;
    
    T px()       const { return (*this)[ PX ]; }
    T py()       const { return (*this)[ PY ]; }
    T vx()       const { return (*this)[ VX ]; }
    T vy()       const { return (*this)[ VY ]; }
    T ax()       const { return (*this)[ AX ]; }
    T ay()       const { return (*this)[ AY ]; }
    T psi()      const { return (*this)[ PSI ]; }
    T dpsi()     const { return (*this)[ DPSI ]; }
    T bdpsi()    const { return (*this)[ BDPSI ]; }
    
    T& px()      { return (*this)[ PX ]; }
    T& py()      { return (*this)[ PY ]; }
    T& vx()      { return (*this)[ VX ]; }
    T& vy()      { return (*this)[ VY ]; }
    T& ax()      { return (*this)[ AX ]; }
    T& ay()      { return (*this)[ AY ]; }
    T& psi()     { return (*this)[ PSI ]; }
    T& dpsi()    { return (*this)[ DPSI ]; }
    T& bdpsi()   { return (*this)[ BDPSI ]; }
};


template<typename T>
class Control : public Kalman::Vector<T, 0>
{
public:
    KALMAN_VECTOR(Control, T, 0)
};

/**
 * @brief System ProcessNoiseParameters
 */
class ProcessNoiseParameters
{
public:
    //! Acceleration noise
    static constexpr size_t QA = 0;
    //! Yaw-rate noise
    static constexpr size_t QR = 1;
    //! Yaw-rate bias noise
    static constexpr size_t QBR = 2;
};

/**
 * @brief System model
 *
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef State<T> S;

    //! Control type shortcut definition
    typedef Control<T> C;

    //! Process noise shortcut definition
    typedef ProcessNoiseParameters P;

    SystemModel(T frequency, T qa, T qr, T qbr) : h(1.0/frequency)
    {
        this->F.setIdentity();

        this->F( S::PX, S::VX ) = h;
        this->F( S::PY, S::VY ) = h;
        this->F( S::PX, S::AX ) = h*h/2;
        this->F( S::PY, S::AY ) = h*h/2;

        this->F( S::VX, S::AX ) = h;
        this->F( S::VY, S::AY ) = h;
        
        this->F( S::PSI, S::DPSI ) = h;

        // W = df/dw (Jacobian of state transition w.r.t. the noise)

        auto G = Kalman::Matrix<T, 9, 3>();
        G.setZero();
        G ( S::PX, P::QA ) = h*h*h/6;
        G ( S::PY, P::QA ) = h*h*h/6;
        G ( S::VX, P::QA ) = h*h/2;
        G ( S::VY, P::QA ) = h*h/2;
        G ( S::AX, P::QA ) = h;
        G ( S::AY, P::QA ) = h;

        G ( S::PSI, P::QR ) = h*h/2;
        G ( S::DPSI, P::QR ) = h;

        G ( S::BDPSI, P::QBR ) = h;

        auto Q = Kalman::SquareMatrix<T,3>();
        Q.setZero();
        Q( P::QA, P::QA ) = qa;
        Q( P::QR, P::QR ) = qr;
        Q( P::QBR, P::QBR ) = qbr;

        this->W = G*Q*G.transpose();

    }
    
    /**
     * @brief Definition of (non-linear) state transition function
     */
    S f(const S& x, const C& u) const
    {
        (void)u; // Silence warning 

        //! Predicted state vector after transition
        S x_;

        x_.px() = x.px() + h*x.vx()+h*h/2*x.ax();
        x_.py() = x.py() + h*x.vy()+h*h/2*x.ay();

        x_.vx() = x.vx() + h*x.ax();
        x_.vy() = x.vy() + h*x.ay();

        x_.ax() = x.ax();
        x_.ay() = x.ay();
        
        x_.psi() = x.psi() + h*x.dpsi();
        x_.dpsi() = x.dpsi();
        x_.bdpsi() = x.bdpsi();

        double const pi = 3.1415926;
        while (x_.psi() < -pi) x_.psi() += 2*pi;
        while (x_.psi() > pi) x_.psi() -= 2*pi;
        
        // Return transitioned state vector
        return x_;
    }

protected:
    //! Timestep length
    T h;
};

}
}
}

#endif