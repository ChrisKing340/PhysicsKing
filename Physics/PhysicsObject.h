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
// needed for VS17 which had breaking change for std::make_shared
#ifndef _ENABLE_EXTENDED_ALIGNED_STORAGE
#define _ENABLE_EXTENDED_ALIGNED_STORAGE
#endif
// std namespace
#include <memory>
#include <string>
#include <iomanip>
// King namespace
#include "..\MathSIMD\MathSIMD.h"
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
#include "..\3DGeometryKing\3DGeometry.h"

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
        UnitOfMeasure::Mass                     _mass;
        UnitOfMeasure::Inertia                  _momentOfInteria; // scalar, but needs to be a matrix & when world matrix updates, update as well *** TODO ***
        float                                   _coefficientOfDrag = 0.5f; // sphere
        float                                   _crossSectionalArea = 1.f; // used by drag
        bool                                    _hasGravity = true;
        // Physics
        std::vector<std::pair<Force, Distance>> _forcesActingOnBody; // distance from center of mass. After update, this vector is cleared
        Acceleration                            _linearAcceleration; // net of forces on object passing through the center of mass
        AngularAcceleration                     _angularAcceleration; // net of forces on object about center of mass

        Velocity                                _linearVelocityCofM; // velocity of the object
        AngularVelocity                         _angularVelocityCofM; // rotational velocity about the center of mass

        AngularVelocity                         _angularVelocityCofExt; // rotational velocity about an external point from the center of mass (turns the _linearVelocityCofM direction)
        // Position and Orientation
        King::Pose                              _myPose; // translation, orientation, scale
        // Calculated values
        bool                                    _sleep = false; // collission detection active or inactive
        std::pair<bool, DirectX::XMMATRIX>      _worldMatrix; // first bool is world matrix good (false = needs updating)
        Box                                     _boundingBox; // for broad phase detection
    private:
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<PhysicsObject> Create() { return std::make_shared<PhysicsObject>(); }
        // Constructors
        PhysicsObject() = default;
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
        // Accessors
        const auto&                             Get_TBD() const { return 0; }
        const auto&                             Get_linearVelocityCofM() const { return _linearVelocityCofM; }
        const auto&                             Get_angularVelocityCofM() const { return _angularVelocityCofM; }
        DirectX::XMMATRIX                       Get_worldMatrix() { if (!_worldMatrix.first) { _worldMatrix.second = _myPose; _worldMatrix.first = true; } return _worldMatrix.second; }
        // Assignments
        void                                    Set_TBD(const float& TBDIn) { _TBD = TBDIn; }
        void                                    Set_linearVelocityCofM(const Velocity& linearVelocityCofMIn) { _linearVelocityCofM = linearVelocityCofMIn; _sleep = false; }
        void                                    Set_angularVelocityCofM(const AngularVelocity& angularVelocityCofMIn) { _angularVelocityCofM = angularVelocityCofMIn; _sleep = false; }
        void                                    SetWorldMatrixUpdateRequired() { _worldMatrix.first = false; }
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
        // sleeping
        // note a change in velocities should wake us up, refer to Set_... methods
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
            netTorque += Torque(r, F);
        }
        _forcesActingOnBody.clear();

        // Newton's 2nd law, we calculate each Update() from forces
        _linearAcceleration = netForce / _mass;
        _angularAcceleration = netTorque / _momentOfInteria;
    }
    // Has Gravity?
    if (_hasGravity)
    {
        const Acceleration g(UnitOfMeasure::gravity, float3(0.f, -1.f, 0.f));
        _linearAcceleration += g;
    }
    // Integrate acceleration and velocity to position
    // 
    // 
    // Dynamics with simple Euler integration with mid-point acceptable for game simulations
    // Effect of velocity and acceleration on position
    // X = X0 + v0 * t + 1/2 * a0 * t^2
    // Xn = Xn-1 + (vn * tn - vn-1 * tn-1) + 1/2 * (an * tn^2 - an-1 * tn-1^2)
    // our time step will be constant between simulation frames, so...
    // update vn with acceleration (just as vn-1 was) and use the midpoint of 
    // velocity between time steps as a good integration approximation
    // Xn = Xn-1 + (vn-1 + vn) / 2 * dt 
    // Xn = Xn-1 + (vn-1 * dt + vn * dt ) / 2 
    Distance disVnLess1(_linearVelocityCofM, dt); // vn-1 * dt
    // Update velocity for current frame
    // vn = vn-1 + an * dt
    _linearVelocityCofM += _linearAcceleration * dt; // vn
    // *** TO DO *** _angularVelocityCofExt
    Distance disVn(_linearVelocityCofM, dt); // vn * dt
    float3 dis = _myPose.GetTranslation() + (disVnLess1 + disVn) * 0.5f;
    _myPose.SetTranslation(dis);

    // rotation below is not correct. Evaluate https://link.springer.com/content/pdf/10.1007/s00707-013-0914-2.pdf
    Rotation rotVnLess1(_angularVelocityCofM, dt);
    _angularVelocityCofM += _angularAcceleration * dt;
    Rotation rotVn(_angularVelocityCofM, dt);
    // apply rotation uses Quaternions, why it is multiplication and non-cumulative
    Rotation rot = rotVn * rotVnLess1 * _myPose.GetRotation();
    _myPose.SetRotation(rot);

    SetWorldMatrixUpdateRequired();

    if (_angularVelocityCofM.Get_magnitude() == 0.f && _linearVelocityCofM.Get_magnitude() == 0.f && _angularVelocityCofExt == 0.f)
    {
        // go to sleep
        _sleep = true;
    }
}