﻿/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:          Velocity    

Description:    Class to keep the 3D travel direction and distance per unit time.
                Use the Force class to sum external forces and the Acceleration
                class to convert force to acceleration (also takes Mass class
                as input).  This is known as kinematics and straight from Newton's
                1st, 2nd, and 3rd laws.

Usage:          Velocity(const UnitOfMeasure::Time &t, const Acceleration & accIn)             

Contact:        ChrisKing340@gmail.com

References:        

MIT License

Copyright (c) 2019 Christopher H. King

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
// needed for C++17 which had breaking change for std::make_shared
#ifndef _ENABLE_EXTENDED_ALIGNED_STORAGE
#define _ENABLE_EXTENDED_ALIGNED_STORAGE
#endif
// std namespace
#include <vector>
#include <memory>
#include <string>
#include <iomanip>
// King namespace
#include "..\..\MathSIMD\MathSIMD.h"
#include "..\Physics\UnitOfMeasure.h"
#include "..\Physics\Acceleration.h"
// 3rdPart namespace
#include "..\..\json\single_include\nlohmann\json.hpp"
using json = nlohmann::json;

/******************************************************************************
*    Physic Basics
*
*    Newton's 1st Law:
*        An object at rest stays at rest and an object in motion stays in motion
*        unless acted upon by a net sum external force. Known as Law of inertia.
*
*        The force causes acceleration, according to the 2nd Law. If we consider
*        the condition of constant ͢a, then we can simplify and derive:
*
*    Kinematics:
*        𝛥t = tf - ti ; time in seconds (s)
*        ͢af = ͢ai = ͢a ; acceleration in meters per second squared (m/s^2)
*        ͢vf - ͢vi = ͢a • 𝛥t ; velocity in meters per second (m/s)
*        pf - pi = ͢v • 𝛥t + 1/2 • ͢a • 𝛥t^2 ; position in meters (m)
*        pf - pi = 𝛥t • ( ͢vi + ͢vf ) / 2
*        ͢vf^2 - ͢vi^2 = 2 • ͢a • ( ͢pf - ͢pi )
******************************************************************************/

namespace King {

    /******************************************************************************
    *    Velocity
    ******************************************************************************/
    class Velocity;
    class Distance;
    Velocity operator*(const UnitOfMeasure::Time &t, const Acceleration & accIn); // dv = ͢vf - ͢vi = 𝛥t * ͢a 
    Velocity operator*(const Acceleration & accIn, const UnitOfMeasure::Time &t); // dv = ͢vf - ͢vi = ͢a * 𝛥t
    Velocity operator/(const Distance& disIn, const UnitOfMeasure::Time& t); // v = d / t 
    // also refer to angular acceleration for more tangential Velocity functions

    class alignas(16) Velocity
    {
        /* variables */
    public:
    protected:
        UnitOfMeasure::Speed                _magnitude; // absolute, >= 0
        float3                              _unit_direction;
    private:
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Velocity> Create() { return std::make_shared<Velocity>(); }
        Velocity() = default;
        explicit Velocity(const float &magIn, const float3 &dirIn) { _magnitude = abs(magIn); _unit_direction = float3::Normal(dirIn); if (_magnitude != magIn) { _unit_direction = -_unit_direction; }; }
        explicit Velocity(const UnitOfMeasure::Speed &s, const float3 &dirIn) { _magnitude = abs(s); _unit_direction = float3::Normal(dirIn); if (_magnitude != s) { _unit_direction = -_unit_direction; }; }
        Velocity(const float3 &vectorIn) { Set(vectorIn); }
        explicit Velocity(const Acceleration & accIn, const UnitOfMeasure::Time &t) { _magnitude = UnitOfMeasure::Speed(accIn.Get_magnitude() * t); _unit_direction = accIn.Get_unit_direction(); }
        explicit Velocity(const std::vector<Acceleration> & accelIn, const UnitOfMeasure::Time &t) { float3 sum; for (const auto & e : accelIn) sum += e.GetVector(); *this = Acceleration(sum) * t; }
        explicit Velocity(const float& x, const float& y, const float& z) : Velocity(float3(x, y, z)) { ; }
        Velocity(const Velocity &in) { *this = in; } // forward to copy assignment
        Velocity(Velocity &&in) noexcept { *this = std::move(in); } // forward to move assignment

        ~Velocity() { ; }

        static const std::string Unit() { return UnitOfMeasure::Speed::_unit; }
        static const std::wstring UnitW() { return UnitOfMeasure::Speed::_wunit; }

        // Conversions
        inline explicit operator DirectX::XMVECTOR() const { return GetVector().GetVecConst(); }
        inline operator float3() const { return GetVector(); }
        inline explicit operator float() const { return _magnitude; }
        inline explicit operator UnitOfMeasure::Speed() const { return _magnitude; }
        // Comparators
        inline bool operator<  (const float& rhs) const { return _magnitude < rhs; }
        inline bool operator<= (const float& rhs) const { return _magnitude <= rhs; }
        inline bool operator>  (const float& rhs) const { return _magnitude > rhs; }
        inline bool operator>= (const float& rhs) const { return _magnitude >= rhs; }
        inline bool operator== (const float& rhs) const { return _magnitude == rhs; }
        inline bool operator!= (const float& rhs) const { return _magnitude != rhs; }
        inline bool operator<  (const Velocity& rhs) const { return DirectX::XMVector3Less(GetVector(), rhs.GetVector()); }
        inline bool operator<= (const Velocity& rhs) const { return DirectX::XMVector3LessOrEqual(GetVector(), rhs.GetVector()); }
        inline bool operator>  (const Velocity& rhs) const { return DirectX::XMVector3Greater(GetVector(), rhs.GetVector()); }
        inline bool operator>= (const Velocity& rhs) const { return DirectX::XMVector3GreaterOrEqual(GetVector(), rhs.GetVector()); }
        inline bool operator== (const Velocity& rhs) const { return DirectX::XMVector3Equal(GetVector(), rhs.GetVector()); }
        inline bool operator!= (const Velocity& rhs) const { return DirectX::XMVector3NotEqual(GetVector(), rhs.GetVector()); }
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Velocity*>(p)); }
        inline Velocity & operator= (const Velocity &other) { _magnitude = other._magnitude; _unit_direction = other._unit_direction; return *this; } // copy assign
        inline Velocity & operator= (Velocity &&other) noexcept { std::swap(_magnitude, other._magnitude); std::swap(_unit_direction, other._unit_direction); return *this; } // move assign
        explicit operator bool() const { return (bool)_magnitude && (bool)_unit_direction; } // valid
        bool operator !() const { return !(bool)_magnitude || !(bool)_unit_direction; } // invalid
        // Math Operators
        inline Velocity operator- () const { return Velocity(-_unit_direction); }
        inline Velocity operator+ (const Velocity & in) const { return Velocity(GetVector() + in.GetVector()); }
        inline Velocity operator- (const Velocity & in) const { return Velocity(GetVector() - in.GetVector()); }
        UnitOfMeasure::SpeedSq operator* (const Velocity& in) const;
        float operator/ (const Velocity& in) const; // unitless, ratio
        inline Velocity & operator+= (const Velocity & in) { *this = *this + in; return *this; } 
        inline Velocity & operator-= (const Velocity & in) { *this = *this - in; return *this; }
        inline UnitOfMeasure::SpeedSq operator* (const float& in) const { return UnitOfMeasure::SpeedSq(_magnitude * in); }

        // Init/Start/Stop/Destroy
        // Functionality
        bool                                IsZero() const { return _magnitude == 0.f; }
        bool                                IsOrNearZero() const { return _magnitude <= 1.0e-5f; }
        // Accessors
        const auto&                         Get_magnitude() const { return _magnitude; }
        auto&                               Get_magnitude() { return _magnitude; }
        const auto&                         Get_unit_direction() const { return _unit_direction; }
        auto&                               Get_unit_direction() { return _unit_direction; }
        const float3                        GetVector() const { return _unit_direction * _magnitude; }
        const float                         GetX() const { return _unit_direction.GetX() * _magnitude; }
        const float                         GetY() const { return _unit_direction.GetY() * _magnitude; }
        const float                         GetZ() const { return _unit_direction.GetZ() * _magnitude; }
        float                               GetValueEN() const { return UnitOfMeasure::mPerSecToftPerSec * _magnitude; }
        float                               GetValueSI() const { return UnitOfMeasure::mPerSec * _magnitude; }
        // Assignments
        // Note: set unit direction before magnitude in case sign of magnitude is switched
        void __vectorcall                   Set(const float3 vectorIn) { _magnitude = float3::Magnitude(vectorIn); _unit_direction = float3::Normal(vectorIn); }
        void                                SetX(const float x) { auto d = GetVector(); d.SetX(x); Set(d); }
        void                                SetY(const float y) { auto d = GetVector(); d.SetY(y); Set(d); }
        void                                SetZ(const float z) { auto d = GetVector(); d.SetZ(z); Set(d); }
        void                                Set_magnitude(const float &_magnitude_IN) { _magnitude = abs(_magnitude_IN); if (_magnitude != _magnitude_IN) { _unit_direction = -_unit_direction; }; }
        void __vectorcall                   Set_unit_direction(const float3 _unit_direction_IN) { _unit_direction = float3::Normal(_unit_direction_IN); }
        inline void                         SetZero() { _magnitude = 0.f; _unit_direction = DirectX::g_XMZero; }
        inline void                         SetZeroIfNear(const float epsilon = 0.00005f) { auto mask = DirectX::XMVectorLess(DirectX::XMVectorAbs(_unit_direction), DirectX::XMVectorReplicate(epsilon)); DirectX::XMVectorSelect(_unit_direction, DirectX::XMVectorZero(), mask); _unit_direction.Normalize(); }
        // Input & Output functions that can have access to protected & private data
        friend std::ostream& operator<< (std::ostream& os, const Velocity& in);
        friend std::istream& operator>> (std::istream& is, Velocity& out);
        friend std::wostream& operator<< (std::wostream& os, const Velocity& in);
        friend std::wistream& operator>> (std::wistream& is, Velocity& out);
        friend void to_json(json& j, const Velocity& from);
        friend void from_json(const json& j, Velocity& to);
    };
    // Input & Output Function forward declarations
    std::ostream& operator<< (std::ostream& os, const Velocity& in);
    std::istream& operator>> (std::istream& is, Velocity& out);
    std::wostream& operator<< (std::wostream& os, const Velocity& in);
    std::wistream& operator>> (std::wistream& is, Velocity& out);
    void to_json(json& j, const Velocity& from);
    void from_json(const json& j, Velocity& to);
}
