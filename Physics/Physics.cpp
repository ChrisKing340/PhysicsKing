﻿#include "..\Physics.h"

#ifdef _DEBUG
#pragma comment(lib, "..\\..\\Build\\MathSIMD\\x64\\Debug\\MathSIMD.lib")
#else
#pragma comment(lib, "..\\..\\Build\\MathSIMD\\x64\\Release\\MathSIMD.lib")
#endif

using namespace King;
using namespace UnitOfMeasure;
using namespace Physics;

/******************************************************************************
*   Force
******************************************************************************/
// Streams
std::ostream& King::operator<< (std::ostream& os, const Force& in)   { return os << "{"<< in.Get_magnitude() << " * Dir:" << in.Get_unit_direction() << " }"; } // text out
std::wostream& King::operator<< (std::wostream& os, const Force& in) { return os << L"{"<< in.Get_magnitude() << L" * Dir:" << in.Get_unit_direction() << L" }"; } // text out
std::istream& King::operator>> (std::istream& is, Force& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
std::wistream& King::operator>> (std::wistream& is, Force& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
// json
void King::to_json(json& j, const Force& from) { j = json{ {"Mag", from.Get_magnitude()}, {"Dir", from.Get_unit_direction()} }; }
void King::from_json(const json& j, Force& to) { j.at("Mag").get_to(to.Get_magnitude()); j.at("Dir").get_to(to.Get_unit_direction());}
// operators
// methods
// functions
/******************************************************************************
*   Torque
******************************************************************************/
// Streams
std::ostream& King::operator<< (std::ostream& os, const Torque& in) { return os << "{" <<in.Get_magnitude() << " * Dir:" << in.Get_unit_direction() << " }"; } // text out
std::wostream& King::operator<< (std::wostream& os, const Torque& in) { return os << L"{" <<in.Get_magnitude() << L" * Dir:" << in.Get_unit_direction() << L" }"; } // text out
std::istream& King::operator>> (std::istream& is, Torque& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
std::wistream& King::operator>> (std::wistream& is, Torque& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
// json
void King::to_json(json& j, const Torque& from) { j = json{ {"Mag", from.Get_magnitude()}, {"Dir", from.Get_unit_direction()} }; }
void King::from_json(const json& j, Torque& to) { j.at("Mag").get_to(to.Get_magnitude()); j.at("Dir").get_to(to.Get_unit_direction()); }
// operators
// methods
// functions
/******************************************************************************
*   Acceleration
******************************************************************************/
// Streams
std::ostream& King::operator<< (std::ostream& os, const Acceleration& in) { return os << "{" << in.Get_magnitude() << " * Dir:" << in.Get_unit_direction() << " }"; } // text out
std::wostream& King::operator<< (std::wostream& os, const Acceleration& in) { return os << L"{" << in.Get_magnitude() << L" * Dir:" << in.Get_unit_direction() << L" }"; } // text out
std::istream& King::operator>> (std::istream& is, Acceleration& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
std::wistream& King::operator>> (std::wistream& is, Acceleration& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
// json
void King::to_json(json& j, const Acceleration& from) { j = json{ {"Mag", from.Get_magnitude()}, {"Dir", from.Get_unit_direction()} }; }
void King::from_json(const json& j, Acceleration& to) { j.at("Mag").get_to(to.Get_magnitude()); j.at("Dir").get_to(to.Get_unit_direction()); }
// operators
Acceleration King::operator/(const King::Force& f, const UnitOfMeasure::Mass& m)
{
    return Acceleration(static_cast<float>(f.Get_magnitude()) / static_cast<float>(m), f.Get_unit_direction());
}
Force King::operator*(const UnitOfMeasure::Mass& m, const Acceleration& a)
{
    // ͢F = m * ͢a
    return Force(static_cast<float>(m) * static_cast<float>(a.Get_magnitude()), a.Get_unit_direction());
}
Force King::operator*(const Acceleration& a, const UnitOfMeasure::Mass& m) 
{
    // ͢F = ͢a * m
    return Force(static_cast<float>(m) * static_cast<float>(a.Get_magnitude()), a.Get_unit_direction());
}
// methods
// functions
/******************************************************************************
*   AngularAcceleration
******************************************************************************/
// Streams
std::ostream& King::operator<< (std::ostream& os, const AngularAcceleration& in) { return os << "{" << in.Get_magnitude() << " * About:" << in.Get_unit_direction() << " }"; } // text out
std::wostream& King::operator<< (std::wostream& os, const AngularAcceleration& in) { return os << L"{" << in.Get_magnitude() << L" * About:" << in.Get_unit_direction() << L" }"; } // text out
std::istream& King::operator>> (std::istream& is, AngularAcceleration& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
std::wistream& King::operator>> (std::wistream& is, AngularAcceleration& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
// json
void King::to_json(json& j, const AngularAcceleration& from) { j = json{ {"AngleMag", from.Get_magnitude()}, {"EulerXYZ", from.Get_unit_direction()} }; }
void King::from_json(const json& j, AngularAcceleration& to) { j.at("AngleMag").get_to(to.Get_magnitude()); j.at("EulerXYZ").get_to(to.Get_unit_direction()); }
// operators
AngularAcceleration King::operator/(const Torque& tIn, const UnitOfMeasure::Inertia& inertiaIn)
{
    return AngularAcceleration(tIn / static_cast<float>(inertiaIn));
}
Torque King::operator*(const UnitOfMeasure::Inertia& inertiaIn, const AngularAcceleration& angAccelIn)
{
    return Torque(static_cast<float3>(angAccelIn) * static_cast<float>(inertiaIn));
}
Torque King::operator*(const AngularAcceleration& angAccelIn, const UnitOfMeasure::Inertia& inertiaIn)
{
    return Torque(static_cast<float3>(angAccelIn)* static_cast<float>(inertiaIn));
}
Acceleration King::operator*(const AngularAcceleration& alphaIn, const Distance& rIn)
{
    // acceleration of a point on a rigid body has three components:
    // 1. Linear acceleration due to the acceleration of the centre of mass
    // 2. Linear acceleration due to centrifugal force
    // 3. Linear acceleration due to a change in angular velocity
    // ͢a = ͢a0 + 𝛼 x ͢r + ͢𝜔 x ͢v
    // v = ͢𝜔 x ͢r
   
    // ͢a is tangential to the rotation, thus:
    // at = 𝛼 x ͢r ; tangential
    Acceleration at;
    auto alpha = alphaIn.GetVector();
    auto radius = rIn.GetVector();
    at = Cross(alpha, radius);

    return at;
}
Acceleration King::operator*(const Distance& rIn, const AngularAcceleration& alphaIn)
{
    // ͢a is tangential to the rotation, thus:
    // at = 𝛼 x ͢r ; tangential
    Acceleration at;
    auto alpha = alphaIn.GetVector();
    auto radius = rIn.GetVector();
    at = Cross(alpha, radius);

    return at;
}
// methods

// functions
/******************************************************************************
*   AngularVelocity
******************************************************************************/
// Streams
std::ostream& King::operator<< (std::ostream& os, const AngularVelocity& in) { return os << "{" << " EulerXYZ: " << in.Get_unit_direction() << " EulerXYZ: " << in.Get_magnitude() << " }"; } // text out
std::wostream& King::operator<< (std::wostream& os, const AngularVelocity& in) { return os << L"{" << L" EulerXYZ: " << in.Get_unit_direction() << L" EulerXYZ: " << in.Get_magnitude() << L" }"; } // text out
std::istream& King::operator>> (std::istream& is, AngularVelocity& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
std::wistream& King::operator>> (std::wistream& is, AngularVelocity& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
// json
void King::to_json(json& j, const AngularVelocity& from) { j = json{ {"AngleMag", from.Get_magnitude()}, {"EulerXYZ", from.Get_unit_direction()} }; }
void King::from_json(const json& j, AngularVelocity& to) { j.at("AngleMag").get_to(to.Get_magnitude()); j.at("EulerXYZ").get_to(to.Get_unit_direction()); }
// operators
AngularVelocity King::operator*(const UnitOfMeasure::Time& t, const AngularAcceleration& accIn)
{
    return AngularVelocity(static_cast<float>(accIn.Get_magnitude())* static_cast<float>(t), accIn.Get_unit_direction());
}

AngularVelocity King::operator*(const AngularAcceleration& accIn, const UnitOfMeasure::Time& t)
{
    return AngularVelocity(static_cast<float>(accIn.Get_magnitude())* static_cast<float>(t), accIn.Get_unit_direction());
}
// methods
// v = ͢𝜔 x ͢r ; tangential velocity
Velocity King::AngularVelocity::CalculateTangentialVelocityAtEndOf_radius(const Distance& rIn) const
{
    // v = ͢𝜔 x ͢r ; tangential velocity
    Velocity v = GetVector().CrossProduct(rIn.GetVector());
    return v;
}

Acceleration King::AngularVelocity::CalculateNormalAccelerationAlong_radius(const Distance& rIn) const
{
    // normal acceleration is along r
    // ͢an = r • 𝜔 ^ 2; normal acceleration in m / s ^ 2
    auto dir = -rIn.Get_unit_direction();

    auto omega = Get_magnitude();
    auto magSq = omega * omega; // rad^2/s^2

    Acceleration an(static_cast<float>(magSq) * rIn.Get_magnitude(), dir); // m/s^2
    return an;
}
Acceleration King::AngularVelocity::CalculateLinearAccelerationFrom(const Acceleration& a0In, const AngularAcceleration& alphaIn, const Distance& rIn) //, const AngularVelocity& omegaIn
{
    // acceleration of a point on a rigid body has three components:
    // 1. Linear acceleration due to the acceleration of the centre of mass
    // 2. Linear acceleration due to centrifugal force
    // 3. Linear acceleration due to a change in angular velocity
    // ͢a = ͢a0 + 𝛼 x ͢r + ͢𝜔 x ͢v
    // v = ͢𝜔 x ͢r

    auto alpha = alphaIn.GetVector();
    auto r = rIn.GetVector();
    auto omega = GetVector();

    Acceleration a = a0In + Cross(alpha, r) + Cross(omega, Cross(omega, r));

    return a;
}
// functions
/******************************************************************************
*   Velocity
******************************************************************************/
// Streams
std::ostream& King::operator<< (std::ostream& os, const Velocity& in) { return os << "{" << in.Get_magnitude() << " * Dir:" << in.Get_unit_direction() << " }"; } // text out
std::wostream& King::operator<< (std::wostream& os, const Velocity& in) { return os << L"{" << in.Get_magnitude() << L" * Dir:" << in.Get_unit_direction() << L" }"; } // text out
std::istream& King::operator>> (std::istream& is, Velocity& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
std::wistream& King::operator>> (std::wistream& is, Velocity& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
// json
void King::to_json(json& j, const Velocity& from) { j = json{ {"Mag", from.Get_magnitude()}, {"Dir", from.Get_unit_direction()} }; }
void King::from_json(const json& j, Velocity& to) { j.at("Mag").get_to(to.Get_magnitude()); j.at("Dir").get_to(to.Get_unit_direction()); }
// operators
Velocity King::operator*(const UnitOfMeasure::Time & t, const Acceleration & accIn)
{
    return Velocity(static_cast<float>(accIn.Get_magnitude()) * static_cast<float>(t), accIn.Get_unit_direction());
}

Velocity King::operator*(const Acceleration & accIn, const UnitOfMeasure::Time & t)
{
    return Velocity(static_cast<float>(accIn.Get_magnitude()) * static_cast<float>(t), accIn.Get_unit_direction());
}
Velocity King::operator/(const Distance& disIn, const UnitOfMeasure::Time& t)
{
    return Velocity(static_cast<float>(disIn.Get_magnitude()) / static_cast<float>(t), disIn.Get_unit_direction());
}
// methods
UnitOfMeasure::SpeedSq Velocity::operator* (const Velocity& in) const
{ 
    return Get_magnitude() * in.Get_magnitude(); 
}

float Velocity::operator/ (const Velocity& in) const 
{ 
    return Get_magnitude() / in.Get_magnitude(); // unitless, ratio
} 
// functions

/******************************************************************************
*   Momentum
******************************************************************************/
// Streams
std::ostream& King::operator<< (std::ostream& os, const Momentum& in) { return os << "{" << in.Get_magnitude() << " * Dir:" << in.Get_unit_direction() << " }"; } // text out
std::wostream& King::operator<< (std::wostream& os, const Momentum& in) { return os << L"{" << in.Get_magnitude() << L" * Dir:" << in.Get_unit_direction() << L" }"; } // text out
std::istream& King::operator>> (std::istream& is, Momentum& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
std::wistream& King::operator>> (std::wistream& is, Momentum& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
// json
void King::to_json(json& j, const Momentum& from) { j = json{ {"Mag", from.Get_magnitude()}, {"Dir", from.Get_unit_direction()} }; }
void King::from_json(const json& j, Momentum& to) { j.at("Mag").get_to(to.Get_magnitude()); j.at("Dir").get_to(to.Get_unit_direction()); }
// operators
Momentum King::operator*(const UnitOfMeasure::Time& dt, const Force& forceIn)
{
    Momentum p;
    p.Set_magnitude((float)dt * (float)forceIn.Get_magnitude());
    p.Set_unit_direction(forceIn.Get_unit_direction());
    return p;
}
Momentum King::operator*(const Force& forceIn, const UnitOfMeasure::Time& dt)
{
    Momentum p;
    p.Set_magnitude((float)dt * (float)forceIn.Get_magnitude());
    p.Set_unit_direction(forceIn.Get_unit_direction());
    return p;
}

Momentum King::operator*(const Velocity& v, const UnitOfMeasure::Mass& m)
{
    Momentum p;
    p.Set_magnitude((float)m * (float)v.Get_magnitude());
    p.Set_unit_direction(v.Get_unit_direction());
    return p;
}
Momentum King::operator*(const UnitOfMeasure::Mass& m, const Velocity& v)
{
    Momentum p;
    p.Set_magnitude((float)m * (float)v.Get_magnitude());
    p.Set_unit_direction(v.Get_unit_direction());
    return p;
}

// methods
// functions

/******************************************************************************
*   Distance
******************************************************************************/
// Streams
std::ostream& King::operator<< (std::ostream& os, const Distance& in) { return os << "{" << in.Get_magnitude() << " * Dir:" << in.Get_unit_direction() << " }"; } // text out
std::wostream& King::operator<< (std::wostream& os, const Distance& in) { return os << L"{" << in.Get_magnitude() << L" * Dir:" << in.Get_unit_direction() << L" }"; } // text out
std::istream& King::operator>> (std::istream& is, Distance& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
std::wistream& King::operator>> (std::wistream& is, Distance& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
// json
void King::to_json(json& j, const Distance& from) { j = json{ {"Mag", from.Get_magnitude()}, {"Dir", from.Get_unit_direction()} }; }
void King::from_json(const json& j, Distance& to) { j.at("Mag").get_to(to.Get_magnitude()); j.at("Dir").get_to(to.Get_unit_direction()); }
// operators
Distance King::operator*(const Velocity& velIn, const UnitOfMeasure::Time& t)
{
    return Distance(velIn.Get_magnitude() * t, velIn.Get_unit_direction());
}
Distance King::operator*(const UnitOfMeasure::Time& t, const Velocity& velIn)
{
    return Distance(velIn.Get_magnitude() * t, velIn.Get_unit_direction());
}
Distance King::operator*(const King::Quaternion& qIn, const King::Distance& dIn)
{
    auto dir = qIn * dIn.Get_unit_direction().GetVecConst();
    return Distance(dIn.Get_magnitude(), dir);
}
Distance King::operator*(const King::Distance& dIn, const King::Quaternion& qIn)
{
    auto dir = float3(DirectX::XMVector3InverseRotate(dIn.Get_unit_direction(), qIn.GetVecConst()));
    return Distance(dIn.Get_magnitude(), dir);
}
UnitOfMeasure::Energy King::operator*(const Force& fIn, const Distance& dIn)
{
    UnitOfMeasure::Energy e(fIn.Get_magnitude() * fIn.Get_magnitude()); // N * m = J
    return e;
}
UnitOfMeasure::Energy King::operator*(const Distance& dIn, const Force& fIn)
{
    UnitOfMeasure::Energy e(fIn.Get_magnitude() * fIn.Get_magnitude()); // N * m = J
    return e;
}
// methods
// functions
/******************************************************************************
*   Rotation
******************************************************************************/
// Streams
std::ostream& King::operator<< (std::ostream& os, const Rotation& in) { return os << "{" << in.Get_magnitude() << " * About:" << in.Get_unit_direction() << " }"; } // text out
std::wostream& King::operator<< (std::wostream& os, const Rotation& in) { return os << L"{" << in.Get_magnitude() << L" * About:" << in.Get_unit_direction() << L" }"; } // text out
std::istream& King::operator>> (std::istream& is, Rotation& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
std::wistream& King::operator>> (std::wistream& is, Rotation& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
// json
void King::to_json(json& j, const Rotation& from) { j = json{ {"AngleMag", from.Get_magnitude()}, {"EulerXYZ", from.Get_unit_direction()}, {"Quat", from.GetQuaternion()} }; }
void King::from_json(const json& j, Rotation& to) { j.at("AngleMag").get_to(to.Get_magnitude()); j.at("EulerXYZ").get_to(to.Get_unit_direction()); j.at("Quat").get_to(to.GetQuaternion()); }
// operators
Rotation King::operator*(const AngularVelocity& velIn, const UnitOfMeasure::Time& t)
{
    return Rotation(velIn, t);
}
Rotation King::operator*(const UnitOfMeasure::Time& t, const AngularVelocity& velIn)
{
    return Rotation(velIn, t);
}
// methods
void King::Rotation::SetFrom(AngularAcceleration alphaIn, UnitOfMeasure::TimeSq tSqIn)
{
    // |𝛥𝜃| = 1/2 * |𝛼| * t^2  ; rotation
    float change_theta = 0.5f * alphaIn.Get_magnitude() * tSqIn;
    Set_magnitude(change_theta);
    Set_unit_direction(alphaIn.Get_unit_direction());
    CalculateQuat();
}
void King::Rotation::SetFrom(AngularVelocity omegaIn, UnitOfMeasure::Time tIn)
{
    // |𝛥𝜃| = |𝜔| * t ; rotation
    float change_theta = omegaIn.Get_magnitude() * tIn;
    Set_magnitude(change_theta);
    Set_unit_direction(omegaIn.Get_unit_direction());
    CalculateQuat();
}
// functions
/******************************************************************************
*   Position
******************************************************************************/
// Streams
std::ostream& King::operator<< (std::ostream& os, const Position& in) { return os << "{" << " Pos: " << (float3)in << " }"; } // text out
std::wostream& King::operator<< (std::wostream& os, const Position& in) { return os << L"{" << L" Pos: " << (float3)in << L" }"; } // text out
std::istream& King::operator>> (std::istream& is, Position& out) { return is >> out; } // binary in
std::wistream& King::operator>> (std::wistream& is, Position& out) { return is >> out; } // binary in
// json
void King::to_json(json& j, const Position& from) { j = json{ {"Pos", from} }; }
void King::from_json(const json& j, Position& to) { j.at("Pos").get_to(to); }
// operators
// methods
float3 Position::To_SphericalCoordinates(void) const
{
    // rho: distance to the origin
    // theta: counter-clockwise rotation about the z-axis
    // phi: altitude signed angle from the xy plane, 0 to pi/2 above the plane, 0 to -pi/2 below the plane.
    float rho = GetMagnitude();
    float theta = std::atan2f(GetY(), GetX());
    float phi = std::acosf(GetZ() / rho) - DirectX::XM_PIDIV2;

    return float3(rho, theta, phi);
}
void Position::Set_SphericalCoordinates(const float& rhoIn, const float& thetaIn, const float& phiIn)
{
    float phi = phiIn + DirectX::XM_PIDIV2;
    float sinPhi = std::sinf(phi);
    // unit vector
    float x = std::cosf(thetaIn) * sinPhi;
    float y = std::sinf(thetaIn) * sinPhi;
    float z = std::cosf(phi);
    float3 cartesian(x, y, z);
    // magnitude
    cartesian *= rhoIn;
    Set(cartesian);
}
// functions
Position King::Physics::Mechanics_Trajectory(const Position& initialPosIn, const Velocity& initialVelIn, const Acceleration& constAccelIn, const UnitOfMeasure::Time& tIn)
{
    // p = p0 + v0 t + 1/2 a t^2
    Position p = initialPosIn + initialVelIn * tIn + constAccelIn * tIn * tIn * 0.5f;
    return p;
}

Position King::Physics::Mechanics_TrajectoryNegativeYGravity(const Position& initialPosIn, const Velocity& initialVelIn, const UnitOfMeasure::Time& tIn)
{
    // p = p0 + v0 t + 1/2 g t^2
    auto g = Acceleration(UnitOfMeasure::gravity, float3(0.f, -1.0f, 0.f));
    Position p = initialPosIn + initialVelIn * tIn + g * tIn * tIn * 0.5f;
    return p;
}

UnitOfMeasure::Time King::Physics::Mechanics_TrajectoryTimeAtMaximumHeightWithNegativeYGravity(const Velocity& initialVelIn)
{
    // t1 = v0Y / g
    // t1 = v0 sin(theta) / g
    UnitOfMeasure::Time t1 = UnitOfMeasure::Speed(initialVelIn.GetVector().GetY()) / gravity;
    return t1;
}
// h = v0Y^2 / 2g
UnitOfMeasure::Length King::Physics::Mechanics_TrajectoryMaximumHeightWithNegativeYGravity(const Velocity& initialVelIn)
{
    UnitOfMeasure::Speed v0Y(initialVelIn.GetVector().GetY());
    UnitOfMeasure::Length dy = (v0Y * v0Y) / (gravity * 2.f);
    return dy;
}
// Hooke's Law
// F = -k d ; k = spring constant, d = displacement, and - because the force is opposite the direction of displacement
UnitOfMeasure::Energy King::Physics::Mechanics_Work_SpringWorkFromDistance(const float& kSpringConstantIn, const float3& unitVectorSpringLineOfMotion, const Distance& dIn)
{
    float displacement(unitVectorSpringLineOfMotion.DotProduct(dIn));

    UnitOfMeasure::Energy work;
    work = 0.5f * (float)kSpringConstantIn * (float)displacement * (float)displacement;
    return work;
}
UnitOfMeasure::Energy King::Physics::Mechanics_Work_SpringWorkFromTwoPositions(const float& kSpringConstantIn, const float3& unitVectorSpringLineOfMotion, const Position& spring_p0In, const Position& p1In, const Position& p2In)
{
    auto d01In = p1In - spring_p0In;
    auto d02In = p2In - spring_p0In;

    auto l01 = unitVectorSpringLineOfMotion.DotProduct(d01In);
    auto l02 = unitVectorSpringLineOfMotion.DotProduct(d02In);

    auto displacement = l02 - l01;

    UnitOfMeasure::Energy work;
    work = 0.5f * (float)kSpringConstantIn * (float)displacement * (float)displacement;
    return work;
}

// d = F / k ; if force is in spring line of motion, a positive force causes a positive displacement
Distance King::Physics::Mechanics_Work_SpringDistanceFromForce(const float& kSpringConstantIn, const float3& unitVectorSpringLineOfMotion, const Force& fIn)
{
    auto projectedForce = unitVectorSpringLineOfMotion.DotProduct(fIn);
    return Distance(projectedForce / kSpringConstantIn, unitVectorSpringLineOfMotion);
}
/******************************************************************************
*   PhysicsState
******************************************************************************/
// Streams
// json
void King::to_json(json& j, const PhysicsState& from)
{ 
    j = json { 
        {"_angularAcceleration", from._angularAcceleration},
        {"_angularVelocity", from._angularVelocity},
        {"_angularMomentum", from._angularMomentum},
        {"_pointOnAxisOfRotationLocalSpace", from._pointOnAxisOfRotationLocalSpace},
        {"_rotation", from._rotation},
        //{"_principalMomentsOfInertia", from._principalMomentsOfInertia},
        //{"_productsOfInertia", from._productsOfInertia},
        {"_linearAcceleration", from._linearAcceleration},
        {"_linearVelocity", from._linearVelocity},
        {"_linearMomentum", from._linearMomentum}/*,
        {"_positionWorldSpace", from._positionWorldSpace},
        {"_linearKineticEnergy", from._linearKineticEnergy},
        {"_angularKineticEnergy", from._angularKineticEnergy},
        {"_potentialEnergy", from._potentialEnergy}*/
        }; 
}
void King::from_json(const json& j, PhysicsState& to)
{ 
    j.at("_angularAcceleration").get_to(to._angularAcceleration);
    j.at("_angularVelocity").get_to(to._angularVelocity);
    j.at("_angularMomentum").get_to(to._angularMomentum);
    j.at("_pointOnAxisOfRotationLocalSpace").get_to(to._pointOnAxisOfRotationLocalSpace);
    j.at("_rotation").get_to(to._rotation);
    //j.at("_principalMomentsOfInertia").get_to(to._principalMomentsOfInertia);
    //j.at("_productsOfInertia").get_to(to._productsOfInertia);
    j.at("_linearAcceleration").get_to(to._linearAcceleration);
    j.at("_linearVelocity").get_to(to._linearVelocity);
    j.at("_linearMomentum").get_to(to._linearMomentum);
    //j.at("_positionWorldSpace").get_to(to._positionWorldSpace);
    //j.at("_linearKineticEnergy").get_to(to._linearKineticEnergy);
    //j.at("_angularKineticEnergy").get_to(to._angularKineticEnergy);
    //j.at("_potentialEnergy").get_to(to._potentialEnergy);
}
// operators
// methods
/******************************************************************************
*   PhysicsRigidBody
******************************************************************************/
// Streams
// json
// operators
// methods
void King::PhysicsRigidBody::Update(const UnitOfMeasure::Time& dtIn)
{
    // i is known (make sure it is seeded in the constructor)

    // Work
    Force netForce;
    Torque netTorque;

    for (auto& appliedForce : _forcesActingOnBody)
    {
        Force& F = appliedForce.first;
        Distance& r = appliedForce.second;

        netForce += F;

        // torque, t = r x F
        Torque t(r,F);
        netTorque += t;
    }
    _forcesActingOnBody.clear();

    // Newton's 2nd law
    f._linearAcceleration = netForce / Get_mass();
    f._angularAcceleration = static_cast<King::AngularAcceleration>(netTorque * Get_inertiaTensorInverse());

    // calculate
    f._linearVelocity = i._linearVelocity + f._linearAcceleration * dtIn; // m/s
    f._angularVelocity = i._angularVelocity + f._angularAcceleration * dtIn; // rad/s

    f._positionWorldSpace = i._positionWorldSpace + f._linearVelocity * dtIn;
    f._rotation = i._rotation + f._angularVelocity * dtIn;

    // Momentum
    // *** TODO *** implement momentum classes math operators
    // scalars already made: Motion lMotion; AngularMotion aMotion;
    //f._linearMomentum = f._linearVelocity * _mass; // g * m/s
    //f._angularMomentum = f._angularVelocity * f._principalMomentsOfInertia; // g * rad/s 
    // Newton's 1st law is conservation of momentum

    // Energy
    //f._potentialEnergy = f._positionWorldSpace.Get_position().GetY() * _mass * gravity; // kg m^2 /s^2 = 1 joule
    //f._linearKineticEnergy = f._linearVelocity.Get_magnitude() * f._linearVelocity.Get_magnitude() * _mass * 0.5f;
    //f._angularKineticEnergy;
}

/*
*  License:
    The method below is very special. It is a full collision processor using the impulse method developed.
    Sources do partial implementations that were used as reference (Game Programming Gems 4) and was
    expanded to what we have here by applying a full Physics modeling of an oblique collision, material restitution, friction, linear and
    rotational reactions resulting from collisions. Gems4 (ex: collisions were assumed along the line of contact and not oblique). 
    About the author: I have a bachelor degree in Mechanical Engineering from Purdue University and although the below is trivial and taught to me, computer
    science classes and books implement this like they are reinventing or discovering it for the first time or they peice it together from
    other sources and it leaves out portions. You are free to distribute this code and use it without any
    expressed warranty for fitness of use and purpose. You must cite the author and include this license in the code
    Author: Christopher H. King, PE (c) Copyright 2022
*/
void King::PhysicsRigidBody::ProcessCollisionWithObject(PhysicsRigidBody& moInOut, const float e, const UnitOfMeasure::Time& dt, float3 collisionPtIN, const float penetrationIn)
{
    // Oblique hit, restitution, friction, linear and rotational reactions from collisions

    // Inputs
    // World space positions at time of initial collision:
    const auto& p1i = GetInitialState().Get_positionWorldSpace();
    const auto& p2i = moInOut.GetInitialState().Get_positionWorldSpace();
    // Linear velocities before collision:
    const auto& v1i = GetInitialState().Get_linearVelocity();
    const auto& v2i = moInOut.GetInitialState().Get_linearVelocity();
    // Angular velocities before collision:
    const auto& ω1i = GetInitialState().Get_angularVelocity();
    const auto& ω2i = moInOut.GetInitialState().Get_angularVelocity();
    // Mass of both objects:
    const auto& m1 = Get_mass();
    const auto& m2 = moInOut.Get_mass();
    // Math optimization:
    const auto& m1_inv = Get_invMass();
    const auto& m2_inv = moInOut.Get_invMass();

    // Outputs the new linear and angular velocity of both objects

    // Assumption: The collision has already been verified to occur
    // Determine the normal direction at the point of collision referred to as the line of impact (axis):
    float3 n_axis = p1i - p2i; // o2 -> o1 ; (o1 - o2); object 2 pointing to object 1
    n_axis.MakeNormalize();
    // collision point must be an input
    float3 r1 = collisionPtIN - p1i; // Note: PhysicsBody may have a local offset to center of mass we would also need to subtract
    float3 r2 = collisionPtIN - p2i;

    // Works out the bias to prevent sinking over time (constant repulsive force between the two objects in contact so they behave as solids)
    const float allowedPenetration = 0.1f; // small factor to not repluse outward and leave as is. This way multiple frames do not bounce this back and forth between out of contact and penetrating through contact
    const float biasFactor = 0.1f; // 0.1 to 0.3 /
    float biasFactorValue = PhysicsRigidBody::posCorrectOnOff ? biasFactor : 0.0f;
    float inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;
    auto val = penetrationIn - allowedPenetration;
    float bias = biasFactorValue * inv_dt * fmax(0.0f, val); // to add to normal axis momentum change

    {
        // calculate the impulse, J, required to prevent the objects intersecting 
        float3 J; // ͢J = ͢Fave * 𝛥t; since F = ma,  ͢J = m * 𝛥v ; through integration over time, ti, to time, tf.
        float J_num, J_dem;
        // calculate the relative velocities along the normal axis at the point of contact
        float3 v1atCollisionPt = (float3)v1i + float3::CrossProduct(ω1i, r1); // v = vi + ͢𝜔 x ͢r
        float3 v2atCollisionPt = (float3)v2i + float3::CrossProduct(ω2i, r2); // v = vi + ͢𝜔 x ͢r
        float3 v_rel = (v1atCollisionPt - v2atCollisionPt);
        // magnitude along the normal axis
        float vn = v_rel.DotProduct(n_axis); 

        // rotation about point of collision
        float3 r1_n_axis = r1.CrossProduct(n_axis);
        float3 r2_n_axis = r2.CrossProduct(n_axis);
        // inertia
        // use King::Box::MomentsOfInertiaRotated(const DirectX::XMMATRIX& Iin, const DirectX::XMMATRIX& rotationIn)
        // or  King::Sphere::MomentsOfInertia(const float& densityIn);
        // for Sphere (uniform):
        auto i1 = 2.0f / 5.0f * m1;
        auto I = DirectX::XMMATRIX(i1, 0, 0, 0,
            0, i1, 0, 0,
            0, 0, i1, 0,
            0, 0, 0, 1.f);
        auto I1_inv = DirectX::XMMatrixInverse(nullptr, I);
        // replace with _inertiaTensorInverse when implemented; for games, this is good enough

        auto i2 = 2.0f / 5.0f * m2;
        I = DirectX::XMMATRIX(i2, 0, 0, 0,
            0, i2, 0, 0,
            0, 0, i2, 0,
            0, 0, 0, 1.f);
        auto I2_inv = DirectX::XMMatrixInverse(nullptr, I);
        // replace with moInOut._inertiaTensorInverse when implemented

        // NORMAL contact impulse
        // given n axis is defined as o2 -> o1 ; (o1 - o2); may need to assess signs of + ω1_c_dot and - ω2_c_dot
        J_num = -(1.f + e) * vn + bias; // add a bias to prevent sinking (constant repulsive force)
        J_dem = m1_inv + m2_inv + n_axis.DotProduct(float3::CrossProduct(r1_n_axis * I1_inv, r1) + float3::CrossProduct(r2_n_axis * I2_inv, r2));
        float j = J_num / J_dem;
        j = fmaxf(j, 0.0f);

        J = j * n_axis;
        // Solve for final states from the normal portion of forces involved
        // Line velocity: since momentum is mass * velocity, we just need to divide by mass
        // Angular velocity: cross the linear momentum with the lever (distance from center to the point of contact) for angular momentum the divide by angular inertia
        auto v1n_f = J * m1_inv; 
        auto ω1n_f = r1.CrossProduct(J) * I1_inv;
        auto v2n_f = J * m2_inv;
        auto ω2n_f = r2.CrossProduct(J) * I2_inv;

        // TANGENT contact impulse
        // perpendicular to our line of impact
        float3 t_axis = v_rel - (v_rel.DotProduct(n_axis) * n_axis); // subtract the projected relative velocity at point of collision on the normal axis from the relavtive velocity to get the tangential component
        t_axis.MakeNormalize();

        float vt = v_rel.DotProduct(t_axis);
        J_num = -(1.f + e) * vt;

        float3 r1_t_axis = r1.CrossProduct(t_axis);
        float3 r2_t_axis = r2.CrossProduct(t_axis);
        J_dem = m1_inv + m2_inv + t_axis.DotProduct(float3::CrossProduct(r1_t_axis * I1_inv, r1) + float3::CrossProduct(r2_t_axis * I2_inv, r2));
        j = J_num / J_dem;

        // friction is the force that will apply force tangent to the point of contact
        float frictionCoeff = (Get_coefficientOfFriction() + moInOut.Get_coefficientOfFriction()) * 0.5f; // average the surface roughness together
        float maxJ = frictionCoeff * j;
        j = Clamp(j, -maxJ, maxJ);

        J = j * t_axis;
        // solve for final states
        auto v1t_f = J * m1_inv;
        auto ω1t_f = r1.CrossProduct(J) * I1_inv;
        // v2 is opposite v1 changes
        auto v2t_f = J * m2_inv;
        auto ω2t_f = r2.CrossProduct(J) * I2_inv;

        // Output
        auto& f1 = GetFinalState();
        auto& f2 = moInOut.GetFinalState();

        // Final
        // Add the changes due to normal and tangential forces to the initial state
        f1.Set_linearVelocity(v1i + v1n_f + v1t_f);
        f1.Set_angularVelocity(ω1i + ω2n_f + ω2t_f);

        // Changes to object 2 are opposite of those to object 1 (Newton's 3rd Law)
        f2.Set_linearVelocity(v2i - v2n_f - v2t_f);
        f2.Set_angularVelocity(ω2i - ω2n_f - ω2t_f);
    }
}

// functions

// Functions to remind us of what velocity and acceleration is without their magnitudes, the tangent and principle unit vector of motion.  If Acceleration is zero, we are moving in a straight line.
// mathematics of vectors, tangent and normals, https://ltcconline.net/greenl/courses/202/vectorFunctions/tannorm.htm
float3 __vectorcall King::Physics::UnitTangentVector(Velocity velIn) { return velIn.Get_unit_direction(); }

// The analogue to the slope of the tangent line is the direction of the tangent line. Since velocity is the derivative of position, it is a tangent function to position.
float3 __vectorcall King::Physics::UnitNormalPrincipleVector(Acceleration accIn) { return accIn.Get_unit_direction(); }

// accIn = (at * ͢T) + (an * ͢N)
//  ͢aT = (at * ͢T) ; magnitude and direction
Acceleration __vectorcall King::Physics::AccelerationTangentialComponent(Acceleration accIn, Velocity velIn) { Acceleration aT; aT.Set_unit_direction(UnitTangentVector(velIn)); aT.Set_magnitude(float3::DotProduct(accIn.GetVector(), aT.Get_unit_direction()).GetX()); return aT; }
//  ͢aN = (an * ͢N) ; magnitude and direction
Acceleration __vectorcall King::Physics::AccelerationNormalComponent(Acceleration accIn, Velocity velIn) { Acceleration aN; aN.Set_unit_direction(UnitNormalPrincipleVector(accIn)); aN.Set_magnitude(float3::CrossProduct(velIn.GetVector(), accIn.GetVector()).GetMagnitude() / velIn.Get_magnitude()); return aN; }
