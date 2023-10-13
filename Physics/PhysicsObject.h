/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:  PhysicsObject

Description:    Basic object for physics simulation

Usage:      Call Set_...(float ) methods to define specific properties
            default properties are for steel

Contact:    ChrisKing340@gmail.com

References:        

MIT License

Copyright (c) 2021-2023 Christopher H. King

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
// needed for MS compiler which had breaking change for std::make_shared
#ifndef _ENABLE_EXTENDED_ALIGNED_STORAGE
#define _ENABLE_EXTENDED_ALIGNED_STORAGE
#endif
// std namespace
#include <memory>
#include <string>
#include <iomanip>
// King namespace
#include "..\..\MathSIMD\MathSIMD.h"
#include "..\Physics\UnitOfMeasure.h"

#include "..\Physics\Force.h"
#include "..\Physics\Acceleration.h"
#include "..\Physics\Velocity.h"
#include "..\Physics\Position.h"
#include "..\Physics\Distance.h"
#include "..\Physics\Momentum.h"
#include "..\Physics\Torque.h"
#include "..\Physics\AngularAcceleration.h"
#include "..\Physics\AngularVelocity.h"
#include "..\Physics\Rotation.h"

#include "..\..\GeometryKing\3DGeometryKing\3DGeometry.h"

// 3rd Party namespace
#include "..\..\json\single_include\nlohmann\json.hpp"
using json = nlohmann::json; // for convenience

namespace King {

    class alignas(16) PhysicsObject
    {
        /* variables */
    public:
    protected:
        float _TBD = 0.0f; 
        // Pysical Properties
        std::shared_ptr<PhysicsMaterial>        _sp_material;

        float                                   _coefficientOfDrag = 0.5f; // sphere
        
        bool                                    _hasGravity = true;
        // Physics
        std::vector<std::pair<Force, Distance>> _forcesActingOnBody; // distance from center of mass. After update, this vector is cleared
        
        Acceleration                            _linearAcceleration; // net of forces on object passing through the center of mass
        AngularAcceleration                     _angularAcceleration; // net of forces on object about center of mass

        Velocity                                _linearVelocityCofM; // velocity of the object
        AngularVelocity                         _angularVelocityCofM; // rotational velocity about the center of mass
        AngularVelocity                         _angularVelocityCofExt; // rotational velocity about an external point from the center of mass (turns the _linearVelocityCofM direction)
        // Position and Orientation
        Pose                                    _myPose; // translation, orientation, scale
    
        bool                                    _sleep = false; // collission detection active or inactive
        std::pair<bool, DirectX::XMMATRIX>      _worldMatrix; // first bool is world matrix good (false = needs updating)
        Box                                     _boundingBox; // for broad phase detection
    private:
        // values calculate in Set_mass_volume(mass, volume);
        float                                   _invMass = 1.f; // 1/kg
        UnitOfMeasure::Inertia                  _momentOfInteria; // scalar
        float                                   _crossSectionalArea = 1.f; // used by drag, m^2
        DirectX::XMMATRIX                       _inertiaTensorInverse;
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<PhysicsObject> Create() { return std::make_shared<PhysicsObject>(); }
        // Constructors
        PhysicsObject() { Set_mass_volume(static_cast<King::UnitOfMeasure::Mass>(1.0f / _invMass), King::UnitOfMeasure::Volume(1.0)); } // set default values that are calculated
        PhysicsObject(const PhysicsObject &in) { *this = in; } // forward to copy assignment
        PhysicsObject(PhysicsObject &&in) noexcept { *this = std::move(in); } // forward to move assignment

        ~PhysicsObject() { ; }

        // Conversions
        // Comparators
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<PhysicsObject*>(p)); }
        inline PhysicsObject& operator= (const PhysicsObject& other) = default; // copy assign
        inline PhysicsObject& operator= (PhysicsObject&& other) = default; // move assign
        // Math Operators
        // Init/Start/Stop/Destroy
        // Functionality
        void                                    Update(const UnitOfMeasure::Time& dt);
        void                                    ProcessCollisionWithObject(PhysicsObject& collidedInOut, const float e, const UnitOfMeasure::Time& dt, float3 collisionPtIN, const float penetrationIn);
        // Accessors
        const auto&                             Get_TBD() const { return 0; }
        const auto&                             Get_sleep() const { return _sleep; }
        const auto&                             Get_linearVelocityCofM() const { return _linearVelocityCofM; }
        const auto&                             Get_angularVelocityCofM() const { return _angularVelocityCofM; }
        DirectX::XMMATRIX                       Get_worldMatrix() { if (!_worldMatrix.first) { _worldMatrix.second = _myPose; _worldMatrix.first = true; } return _worldMatrix.second; }
        const auto&                             Get_mass() const { return 1.0f / _invMass; }
        const auto&                             Get_invMass() const { return _invMass; }
        const auto&                             Get_inertiaTensorInverse() const { return _inertiaTensorInverse; }
        const auto&                             Get_crossSectionalArea() const { return _crossSectionalArea; }
        const auto&                             Get_coefficientOfDrag() const { return _coefficientOfDrag; }
        // Assignments
        void                                    Set_TBD(const float& TBDIn) { _TBD = TBDIn; }
        void                                    Set_material(const std::shared_ptr<PhysicsMaterial> sp_material) { _sp_material = sp_material; }
        void                                    Set_sleep(bool sleep = true) { _sleep = sleep; }
        void                                    Set_linearVelocityCofM(const Velocity& linearVelocityCofMIn) { _linearVelocityCofM = linearVelocityCofMIn; _sleep = false; }
        void                                    Set_angularVelocityCofM(const AngularVelocity& angularVelocityCofMIn) { _angularVelocityCofM = angularVelocityCofMIn; _sleep = false; }
        void                                    SetWorldMatrixUpdateRequired() { _worldMatrix.first = false; _sleep = false; }
        void                                    Set_crossSectionalArea(const float& crossSectionalArea) { _crossSectionalArea = crossSectionalArea; }
        void                                    Set_coefficientOfDrag(const float& coefficientOfDrag) { _coefficientOfDrag = coefficientOfDrag; }
        void Set_mass_volume(const UnitOfMeasure::Mass& mass, const UnitOfMeasure::Volume& vol)
        {
            _invMass = 1.0f / mass;

            // assume a sphere
            /*
            Moment Of Inertia of solid sphere
            We will calculate the moment of inertia of a solid sphere by integrating multiple inertias of the disc.

            Inertia is defined for point particles as:
                    n
                Icm = ∑ Mi * ri²
                    i=1
            For a uniform disk of radius r and total mass M we can just integrate:
                Icm = ∫ r² dM

            For a uniform solid sphere, we integrate the discs along one axis of the sphere (since the sphere is uniform every axis is the same)
                Icm = Icmy = Icmx = Icmz
            The integrals for each is then:
                Icmx = ∫ r² dM = (y² + z²) dM
                Icmy = ∫ r² dM = (x² + z²) dM
                Icmz = ∫ r² dM = (x² + y²) dM
            Instead of taking three integrals and knowing that uniform means each axis will be the same, we can take a short cut and average them by stating that the
            sum of each integral will give us three times the moment of inertia for the solid sphere, about the center of mass:
                3 * Icm = Icmx + Icmy + Icmz
                3 * Icm = ∫ (y² + z²) dM + ∫ (x² + z²) dM + ∫ (x² + y²) dM
                3 * Icm = ∫ (x² + x² + y² + y² + z² + z²) dM
                3 * Icm = 2 * ∫ (x² + y²+ z²) dM
            Since r² = x² + y² + z²:
                3 * Icm = 2 * ∫ r² dM
                Icm = 2/3 * ∫ r² dM
            Our problem becomes that we need to know r interms of M, or M in terms of r so we can substitute and solve
            Volume of a sphere:
                V = 4/3 π R³
                dV = 4 * π r²
            We want to integrate the mass of thin wall spheres which is the surface area, dV, * thickness, dr, is:
                dM = M / V * dV * dr
                dM = M / (4/3 π R³) * (4 * π r²) * dr
                dM = 3 * M / R³ * r² * dr
            Substituting for dM:
                Icm = 2/3 * ∫ r² (3 * M / R³ * r² * dr)
                Icm = 2 * M / R³ ∫ r^4 dr
            Integrating with respect to r over the range 0 to our constant R of the sphere gives:
                Icm = 2 * M / R³ * [ R^5 - (0)^5 ] / 5
                Icm = 2/5 * M * R²
            */
            float r = cbrt(vol / ((4.0f / 3.0f) * King::UnitOfMeasure::PI)); // radius
            _crossSectionalArea = King::UnitOfMeasure::PI * r * r;
            _momentOfInteria = 2.0f / 5.0f * _invMass * r * r;

            auto I = DirectX::XMMATRIX(_momentOfInteria, 0, 0, 0,
                0, _momentOfInteria, 0, 0,
                0, 0, _momentOfInteria, 0,
                0, 0, 0, 1.f);

            _inertiaTensorInverse = DirectX::XMMatrixInverse(nullptr, I);
        }
        // Friends
        friend void                             to_json(json& j, const PhysicsObject& from);
        friend void                             from_json(const json& j, PhysicsObject& to);
    protected:
        // Internal Helpers
    };
    // I/O Functions
    std::ostream& operator<< (std::ostream& os, const PhysicsObject& in);
    std::wostream& operator<< (std::wostream& os, const PhysicsObject& in);
    std::istream& operator>> (std::istream& is, PhysicsObject& out);
    std::wistream& operator>> (std::wistream& is, PhysicsObject& out);
    void to_json(json& j, const PhysicsObject& from);
    void from_json(const json& j, PhysicsObject& to);
}

inline std::ostream& King::operator<<(std::ostream& os, const PhysicsObject& in)
{
    return os << "{ TBD:" << in.Get_TBD() << " }"; // text out
}
inline std::wostream& King::operator<<(std::wostream& os, const PhysicsObject& in)
{
    return os << L"{ TBD:" << in.Get_TBD() << L" }"; // text out
}
inline std::istream& King::operator>>(std::istream& is, PhysicsObject& out)
{
    float TBD;
    is >> TBD; // binary in

    out.Set_TBD(TBD);

    return is;
}
inline std::wistream& King::operator>>(std::wistream& is, PhysicsObject& out)
{
    float TBD;
    is >> TBD; // binary in

    out.Set_TBD(TBD);

    return is;
}
inline void King::to_json(json& j, const PhysicsObject& from)
{
    j = json { 
        {"TBD", from._TBD}
    }; 
}
inline void King::from_json(const json& j, PhysicsObject& to)
{
    j.at("TBD").get_to(to._TBD);
}

inline void King::PhysicsObject::Update(const UnitOfMeasure::Time& dt)
{
    if (_sleep && !_forcesActingOnBody.size())
    {
        return;
    }
    // Has drag?
    if (_coefficientOfDrag != 0.f)
    {
        const float den = UnitOfMeasure::DensityEN::Air * UnitOfMeasure::slugTolbm * UnitOfMeasure::lbmPerFtCubedTokgPerMeterCubed;

        float vSq = _linearVelocityCofM.Get_magnitude();
        vSq *= vSq;
        float3 dir = _linearVelocityCofM.Get_unit_direction();
        Force F;
        F.Set_magnitude(0.5f * den * _coefficientOfDrag * _crossSectionalArea * vSq);
        F.Set_unit_direction(-dir);

        std::pair<Force, Distance> Fr(F, Distance());
        _forcesActingOnBody.push_back(Fr);
    }
    // Has force?
    if (_forcesActingOnBody.empty())
    {
        _linearAcceleration.Set_magnitude(0.f);
        _angularAcceleration.Set_magnitude(0.f);
    }
    else
    {
        // Newton's 1st law
        Force netForce;
        Torque netTorque;

        for (auto& appliedForce : _forcesActingOnBody)
        {
            const Force& F = appliedForce.first;
            const Distance& r = appliedForce.second;

            netForce += F;
            netTorque += Torque(r, F); // torque, t = r x F
        }
        _forcesActingOnBody.clear();

        // Newton's 2nd law
        _linearAcceleration = static_cast<Acceleration>(netForce * _invMass);
        _angularAcceleration = static_cast<AngularAcceleration>(netTorque * Get_inertiaTensorInverse());
    }
    // Has Gravity?
    if (_hasGravity)
    {
        const Acceleration g(UnitOfMeasure::gravity, float3(0.f, -1.f, 0.f));
        _linearAcceleration += g;
    }
    // No motion?
    if (_linearAcceleration.Get_magnitude() == 0.f && _angularAcceleration.Get_magnitude() == 0.f && _angularVelocityCofM.Get_magnitude() == 0.f && _linearVelocityCofM.Get_magnitude() == 0.f && _angularVelocityCofExt == 0.f)
    {
        Set_sleep();
    }
    else
    {
        // Integrate acceleration and velocity to rotation
        // _angularVelocityCofExt to rotate the heading vector first
        float3 velDir = _linearVelocityCofM.Get_unit_direction();
        Quaternion quatRotationFinal;

        Quaternion headingQuat;
        headingQuat.Set(float3(0.f, 0.f, -1.f), velDir); // from North to current direction
        Rotation radiansMovedHeading(_angularVelocityCofExt, dt);
        quatRotationFinal = headingQuat * radiansMovedHeading.GetQuaternion();
        _linearVelocityCofM.Set_unit_direction(DirectX::XMVector3Rotate(float3(0.f, 0.f, -1.f), quatRotationFinal));
        // 
        // Dynamics with simple Euler integration with mid-point optimization
        // At a specific time:
        // X = X0 + v0 * t + 1/2 * a0 * t^2
        // Then, between two points we apply Euler integration:
        // Xn = Xn-1 + (vn * tn - vn-1 * tn-1) + 1/2 * (an * tn^2 - an-1 * tn-1^2)
        // our time step will be constant between simulation frames, so...
        // update vn with acceleration (just as vn-1 was) and use the midpoint of 
        // velocity between time steps as a good integration approximation:
        // Xn = Xn-1 + (vn-1 + vn) / 2 * dt, then:
        // Xn = Xn-1 + (vn-1 * dt + vn * dt ) * 0.5 
        // BEFORE:
        Distance disVnLess1(_linearVelocityCofM, dt); // vn-1 * dt
        // AFTER:
        _linearVelocityCofM += _linearAcceleration * dt; // vn
        Distance disVn(_linearVelocityCofM, dt); // vn * dt
        // Combine using midpoint method of Xn = Xn-1 + (vn-1 * dt + vn * dt ) * 0.5 
        float3 pos = _myPose.GetTranslation() + (disVnLess1 + disVn) * 0.5f;
        _myPose.SetTranslation(pos);
    }

    // Rotation about CofM
    // Note: _angularVelocityCofM = Quaternion(axisIn, angleRadPerSec);
    // BEFORE:
    Rotation rotVnLess1(_angularVelocityCofM, dt);
    // AFTER:
    _angularVelocityCofM += _angularAcceleration * dt;
    Rotation rotVn(_angularVelocityCofM, dt);
    // (rotVnLess1 + rotVn) * 0.5f we note that the axis is the same for both, so we can take the midpoint of the magnitudes
    auto midpointMag = (rotVnLess1.Get_magnitude() + rotVn.Get_magnitude()) * 0.5f;
    rotVn.Set_magnitude(midpointMag); // re-use rotVn variable for midpoint
    Rotation rot = rotVn * _myPose.GetRotation();
    _myPose.SetRotation(rot);

    SetWorldMatrixUpdateRequired();
}