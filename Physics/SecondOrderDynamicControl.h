/******************************************************************************
*	Second Order Dynamic Control, give a new position input, initial position, 
*   and three dynamic system characteristics variables, calculate the 
*   change in velocity and acceleration as a response and output the new position.
*   feedback loop of the next position and time step between will form a 
*   dynamic control loop.
* 
*   Dampened frequency response to input control. Useful for ease in / ease out dynamic
*   response of a mechanical system for position given an input and calculating
*   the first and second derivaties for velocity and acceleration.
*       (output) y + k1 yd + k2 ydd = x + xd (input, and xd can be estimated or provided)
* 
*   f, frequency : how fast the system responds
*   zeta, dampening : 0 never dampens, 0.0001-.9999 under dampen to oscilate to a value, 1 critically dampened at this value and higher does not oscilate and settles to a value with higher values taking longer to reach the value
*   r, initial response : 0 time lapse to begin accelerating, 0.0001-1.0 system reacts immediately, >1 will overreact and overshoot target, <0 anticipates motion
*   
    EX: controlled response that settles is f = 1., zeta = 0.5, r = 2
*   EX: critically damped that quickly seeks its target is f = 1., zeta = 1., r = 0
*   
*   Remarks:    Frame rate inconsistencies where the time step varies often and greatly
*               failes the algorithm, introducing jerky behavior and in some extremes
*               the equation is unstable, going to infinity. Reference #2 solved this
*               issue, making this a generalized function to smooth out motion for use
*               in games.
* 
*   References: 1) Katsuhiko Ogata, System Dynamics 2nd edition, Prentice-Hall 1992
*               2) https://youtu.be/KPoeNZZ6H4s
******************************************************************************/
#pragma once

#include "..\..\MathSIMD\MathSIMD.h"
#include "..\Physics.h"

#ifndef MAX
#define MAX(a,b)            (((a) > (b)) ? (a) : (b))
#endif

namespace King {
namespace Physics {

    class SecondOrderDynamicControl
    {
    protected:
        Position xPrev; // previous input to estimate velocity
        Position y; // output
        Velocity yd;
        float _w, _z, _d, k1, k2, k3; // state constants
    public:
        SecondOrderDynamicControl(float3 x0, float f = 1.0f, float z = 1.0f, float r = 0.0f)
        {
            // compute constants from inputs
            _w = 2.f * UnitOfMeasure::PI * f;
            _z = z;
            _d = _w * sqrtf(fabsf(z * z - 1));
            k1 = z / (UnitOfMeasure::PI * f);
            k2 = 1.f / (_w * _w);
            k3 = r * z / _w;
            // initialize variables
            xPrev = x0;
            y = x0;
        }
        float3 Update(const UnitOfMeasure::Time t, const King::Position x)
        {
            // estimate velocity
            auto v = (x - xPrev) / t;
            xPrev = x;
            return Update(t, x, v);
        }
        float3 Update(const UnitOfMeasure::Time t, const King::Position x, const King::Velocity xd)
        {
            float k1_stable, k2_stable;

            if (_w * t < _z)
            {
                // low frame rates clamp k2 to guarantee stability
                k1_stable = k1;
                k2_stable = MAX(MAX(k2, t * t * 0.5f + t * k1 * 0.5f), k1 * t);
            }
            else
            {
                // use pole matching when the system is very fast
                float t1 = exp(-_z * _w * t);
                float alpha = 2.f * t1 * (_z <= 1.f ? cosf(t * _d) : coshf(t * _d));
                float beta = t1 * t1;
                float t2 = t / (1 + beta - alpha);
                k1_stable = (1 - beta) * t2;
                k2_stable = t * t2; // modified time
            }
            // integration by Semi-implicit Euler method
            y = y + yd * t;
            yd = yd + (float3)(((x + xd * k3 - y - yd * k1_stable) * t) / k2_stable);
            return y;
        }
    };

} // namespace Physics
} // namespace King
