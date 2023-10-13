
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:          PhysicsKing

Description:    Basic C++ class implementation of physics. Within the King namespace
                the UnitOfMeasure namespace contains all the constants and 
                conversion factors for SI and EN units of measure. The default
                internal is SI. By use of the classes, units are automatically
                associated and kept throughout the calculations so you don't
                have to concern yourself with them. For inputs, string literals
                are defined. Operator << overloading allows for formated string 
                output for ostreams and json storage and input/output implemented
                using a 3rd party library.

                Compiled with Visual Studio 2019, intended for 64 Bit Windows 10 but
                may work just fine on 32 Bit Windows 10 with 16 bit alignment.

                This code is a small part of a fully functional DirectX 12 game
                engine and physics simulator.

Contact:        ChrisKing340@gmail.com

References:     json input and output utilize https://github.com/nlohmann/json
                SIMD math functions utilize https://github.com/microsoft/DirectXMath
                Math class wrapper 

MIT License

Copyright (c) 2023 Christopher H. King

Background of the Author:   
                Chris graduated from Purdue University with a Bachelor of 
                Science in Mechanical Engineering (BSME). He is a licensed professional
                engineer and works as a Director of Engineering with more than 25 years 
                of professional experience.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#pragma once
// 3rd Party namespace
#include "..\json\single_include\nlohmann\json.hpp"
using json = nlohmann::json;

// Math
#include "..\MathSIMD\MathSIMD.h"

// Measurement
#include "Physics\UnitOfMeasure.h"
// Linear
#include "Physics/Force.h"
#include "Physics/Acceleration.h"
#include "Physics/Velocity.h"
#include "Physics/Position.h"
#include "Physics/Distance.h"
#include "Physics/Momentum.h"
// Rotational
#include "Physics/Torque.h"
#include "Physics/AngularAcceleration.h"
#include "Physics/AngularVelocity.h"
#include "Physics/Rotation.h"

// Advanced Properties
#include "Physics/SecondOrderDynamicControl.h"
#include "Physics/PhysicsState.h"
#include "Physics/PhysicsRigidBody.h"
#include "Physics/PhysicsMaterial.h"
// do not reference here as they create circular references
//#include "Physics/PhysicsObject.h"

// Below defines a few functions to solve specific physics cases to show use of
// the library. Commented some examples of the classes use and the equations
// that govern.

// Symbols: 𝛼𝛽𝛾𝜃𝛷𝜏𝜔𝜌𝜋𝜎𝜇𝜆𝜀𝛥ζ 𝑖𝑗𝑘 𝑚𝑛𝑟𝑠𝑡 𝒾𝒿𝓀𝓁𝓂𝓃𝒹𝒶𝒷𝒸 ±° ⊙⊚ ∫∬∭∮∯∰∝∞∟∠∡∑√∛∜∴≈⨯• ͢   ͢𝑖  ͢𝑗  ͢𝑘

/******************************************************************************
*	Physic Basics
*
*	Newton's 1st Law:
*		An object at rest stays at rest and an object in motion stays in motion
*		unless acted upon by a net sum external force. Known as Law of inertia.
*       The study of this law and its application is known as "Statics"
* 
*	Newton's 2nd Law:
*		The acceleration, a or 𝛼, of an object as produced by a net force, F or T, 
*       is directly proportional to the magnitude of the net force, in the same direction
*		as the net force, and inversely proportional to the mass, m or I, of the object.
*       The study of this law and its application is known as "Dynamics"
*       Linear:
*		    ͢a = ∑ ͢F / m
*			∑ ͢F = m • ͢a
*       Rotational:
*           ͢𝛼 = ∑ ͢T / I
*           ∑ ͢T = ͢𝛼 • I
*       Linear and rotational accelerations (and rotational velocities)
*           combine to one net linear acceleration on an object by:
*           ͢a = ͢a0 + ͢𝛼 x ͢r + ͢𝜔 x ( ͢𝜔 x ͢r )
*        ͢𝛼 is the angular acceleration vector (rotational acceleration)
*        ͢𝜔 is the angular velocity vector (rotational velocity)
*        ͢r is the distance vector from the point of rotation to the point where the force
*           is applied that is being applied to cause the acceleration of the object
*        I is the inertial tensor that accounts for symetry (or lack of it) in the 
*           distribution of mass about the center of mass
* 
*	Newton's 3rd Law:
*		For every action, there is an equal and opposite reaction.  Therefore,
*       forces on two objects colliding are equal in magnitude and opposite in
*       direction. Such forces cause one object to gain momentum and the other
*       object to lose momentum. Total momentum of the system remains conserved
*       in absence of any external force. This law is therefore the conservation
*       of momentum.
*
*	Discussion on Momentum:
*		Conservation of momentum. An object which is in motion has momentum. The
*		amount of momentum (p) possessed by the moving object is the product of
*		mass (m) and velocity (v).
*			p = m • ͢v
*		Change in momentum:
*			dp/dt = m • d ͢v / dt = m • ͢a
*		With the 2nd Law:
*			∑ ͢F = m • ͢a
*           (i = 0...n) ∑ Fi = dp/dt
*	    Impulse = Momentum Change
*           For two colliding objects:
*		    Impulse = dp = m • ͢v2 - m • ͢v1
*		    Fave • ∆t = dp/dt ; with constant time intervals, smaller ∆t approaches dt
*		    dp/dt = m • d ͢v / dt = m • ∆ ͢v / ∆t; assumes time step is constant
*           Impulse = ∆p/∆t = dp/dt ; with constant time intervals, smaller ∆t approaches dt
*           ∆p/∆t = m • ∆ ͢v2 / ∆t - m • ∆ ͢v1 / ∆t
*           ∆p/∆t = m • ( ͢v2 -  ͢v1 ) / ∆t ; assumes velocity is constant over the time step
*           ∆p/∆t = m • ( ͢a2 -  ͢a1 ) ; assumes acceleration is constant over the time step
*           ∆p/∆t = ( ͢F2 - ͢F1 ) ; assumes force is constant over the time step
*           ∆p = ( ͢F2 - ͢F1 ) • ∆t
* 
*		When two object collide (1) (2), the time of contact is the same
*			t1 = t2
*		The force exerted on object 1 (F1) is equal in magnitude and opposite
*		in direction to the force exerted on object 2 (F2) (Newton's 3rd Law).
*			F1 = - F2
*		The momentum change experienced by object 1 is equal in magnitude and
*		opposite in direction to the momentum change experienced by object 2.
*			m1 • ∆ ͢v1 = - m2 • ∆ ͢v2
*		And so, the sum of the momentum of object 1 and the momentum of object
*		2 before the collision is equal to the sum of the momentum of object 1
*		and the momentum of object 2 after the collision.  v1' and v2' will
*		represent the velocities of objects 1 and 2 after the collision.
*			m1 • ͢v1 + m2 • ͢v2 = m1 • ͢v1' + m2 • ͢v2'
*       This equation represents Newton's 3rd law of conservation of momentum
*       for two object that collide.
******************************************************************************/

// Below is some examples and helper code for function that would be used in games to model motion of objects

// FUTURE for relitivistic considerations, the observer movement must be accounted for. Note: https://www.euclideanspace.com/physics/dynamics/inertia/rotation/rotationfor/index.htm#angularacceleration 

namespace King {

namespace Physics {
    // For a point, P, on a solid body in motion, it has a unit tangent vector equal to its local velocity unit vector.
    float3 __vectorcall         UnitTangentVector(Velocity velIn); // The analogue to the slope of the tangent line is the direction of the tangent line. Since velocity is the derivative of position, it is a tangent function to position.
    // If the motion of P is rotating, P must be accelerating with direction into the curve.
    float3 __vectorcall         UnitNormalPrincipleVector(Acceleration accIn); // principle unit vector. Geometrically, for a curve, this vector is the unique vector that point into the curve.
    // If we know both velocity and acceleration, we can break down the acceleratio into two components, along the linear direction of motion (tangent) and into the curvature (normal)
    Acceleration __vectorcall   AccelerationTangentialComponent(Acceleration accIn, Velocity velIn);
    Acceleration __vectorcall   AccelerationNormalComponent(Acceleration accIn, Velocity velIn);

    
    //*** MECHANICS ***
    // all of the below are a subset topics belonging to mechanics

    //*** Trajectory *** 
    // When the force on an object is constant, and therefore acceleration is constant (such as the force of gravity), simple calculations:
    // p = p0 + v0 t + 1/2 a t^2
    King::Position              Mechanics_Trajectory(const Position& initialPosIn, const Velocity& initialVelIn, const Acceleration& constAccelIn, const UnitOfMeasure::Time& tIn);
    // p = p0 + v0 t + 1/2 g t^2
    King::Position              Mechanics_TrajectoryNegativeYGravity(const Position& initialPosIn, const Velocity& initialVelIn, const UnitOfMeasure::Time& tIn);
    // t1 = v0Y / g
    UnitOfMeasure::Time         Mechanics_TrajectoryTimeAtMaximumHeightWithNegativeYGravity(const Velocity& initialVelIn);
    // h = v0Y^2 / 2g
    UnitOfMeasure::Length       Mechanics_TrajectoryMaximumHeightWithNegativeYGravity(const Velocity& initialVelIn);

    // *** Dynamics ***  
    // Acceleration of motion has two components, one normal to the direction of motion (velocity) and one tangential
    // accIn = (an * ͢N) + (at * ͢T);  ͢N and ͢T are relative to the velocity vector
    // ͢aN = (an * ͢N) ; magnitude and direction
    //Acceleration a;
    //Velocity v0;
    //Acceleration an = AccelerationNormalComponent(a, v0);
    // ͢aT = (at * ͢T) ; magnitude and direction
    //Acceleration at = AccelerationTangentialComponent(a, v0);

    // *** Dynamics of Rotations ***   
    //using namespace UnitOfMeasure; // for string literals
    //UnitOfMeasure::Length l(10.0_m);
    //UnitOfMeasure::AngularAccel aa(1.0_radPerSecSq);
    //UnitOfMeasure::AngularSpeed as(1.0_radPerSec);

    //Distance r(l, float3(0.f, 1.f, 0.f));
    //float3 axis(1.f, 0.f, 0.f);
    //AngularAcceleration 𝛼(aa, axis);
    //AngularVelocity 𝜔(as, axis);

    // AngularVelocity class has methods to calculate linear accelerations of the rotational motion:
    // ͢a = ͢a0 + 𝛼 x ͢r + ͢𝜔 x ( ͢𝜔 x ͢r )
    //auto a1 = 𝜔.CalculateLinearAccelerationFrom(a, 𝛼, r);

    // ͢an = r • | ͢𝜔|^2 ; with direction along radius (and opposite) to maintain curviture
    //auto an1 = 𝜔.CalculateNormalAccelerationAlong_radius(r);
    // since ͢a = ͢at + ͢an,
    // ͢at = ͢a - ͢an
    //auto at1 = a - an;

    // ͢v = ͢𝜔 x ͢r ; 
    //Velocity v = 𝜔.CalculateTangentialVelocityAtEndOf_radius(r);

    //*** Work ***
    // Work links the concept of force and energy and is most useful when force varies with time, and therefore acceleration is not constant
    // Use operators defined in class Distance for Energy = Force * Distance
    // Springs
    UnitOfMeasure::Energy       Mechanics_Work_SpringWorkFromDistance(const float& kSpringConstantIn, const float3& unitVectorSpringLineOfMotion, const Distance&);
    UnitOfMeasure::Energy       Mechanics_Work_SpringWorkFromTwoPositions(const float& kSpringConstantIn, const float3& unitVectorSpringLineOfMotion, const Position& spring_p0In, const Position& p1In, const Position& p2In);
    Distance                    Mechanics_Work_SpringDistanceFromForce(const float& kSpringConstantIn, const float3& unitVectorSpringLineOfMotion, const Force&);

    // Our study and modeling of the universe on a scale close to the earth surface is complete with a study of Physics (exspecially Newton's 3rd law)
    // and a study of Thrmodynamics (exspecially the 1st Law of Thermodynamics). With these, we can represent and model our universe*. Note: * is a reminder that at
    // very high speeds and long distances there is more going on for these models to match observation of the universe. For that, extra terms and understanding
    // of relativistic observation considering the observer is in motion must be accounted for. 

    //*** Thermodynamics ***
    // Since UnitOfMeasure::Energy is a scalar, math in conserving it is straight forward. This study is therefore much easier than physics since there
    // are no vector directions to keep track of or complicated math. The concept is generally briefly touched on in college physics classes, and in depth
    // only in mechanical engineering and physics major curriculums.
    // 
    // Some college level basics here:
    // 1st Law: ∆E = Q - W
    //   ∆E is the total change in the energy of the system
    //   Q is the heat added to the system
    //   W is the work performed on the system by the environment
    // law states that energy is conserved, therefore a change in the systems energy from two states must either exchange work or heat.
    // Work on the external environment (negative work, which is outward)
    // Q release heat to the environment (negative heat, which is outward)
    // ∆E = ∆U + ∆PE + ∆KE
    // ∆E = StateFinal - StateInitial
    // PE = m * h * g
    // KE = 1/2 m * v^2
    // U = P * V = m * R(of the gas) * T ; T in kelvin (note this is the ideal gas law, so assumed uncompressed gas)
    // Q = combustion or other source through radiation, conduction, or convection

} // Physics namespace
} // King namespace
