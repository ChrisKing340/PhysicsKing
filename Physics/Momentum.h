/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:          Momentum    

Description:    Class to keep the 3D travel direction and distance per unit time.
                Use the Force class to sum external forces and the Acceleration
                class to convert force to acceleration (also takes Mass class
                as input).  This is known as kinematics and straight from Newton's
                1st, 2nd, and 3rd laws.

Usage:          Momentum(const UnitOfMeasure::Time &t, const Acceleration & accIn)             

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
#include "..\Physics\Velocity.h"
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
*    Kinematics:
*        𝛥t = tf - ti ; time in seconds (s)
*        p = m * v
*        p = F * 𝛥t
******************************************************************************/

namespace King {

    /******************************************************************************
    *    Momentum
    ******************************************************************************/
    class Momentum;
    Momentum operator*(const UnitOfMeasure::Time& dt, const Force& forceIn);
    Momentum operator*(const Force& forceIn, const UnitOfMeasure::Time &dt); 
    Momentum operator*(const Velocity& v, const UnitOfMeasure::Mass& m);
    Momentum operator*(const UnitOfMeasure::Mass& m, const Velocity& v);

    class alignas(16) Momentum
    {
        /* variables */
    public:
    protected:
        UnitOfMeasure::LinearMotion         _magnitude; // absolute, >= 0
        float3                              _unit_direction;
    private:
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Momentum> Create() { return std::make_shared<Momentum>(); }
        // Constructors
        Momentum() = default;
        explicit Momentum(const Force& f, const UnitOfMeasure::Time& dt) { _magnitude = ((float)dt * (float)f.Get_magnitude()); Set_unit_direction(f.Get_unit_direction()); }
        explicit Momentum(const UnitOfMeasure::LinearMotion& lm, const float3& dirIn) noexcept; // magnitude and direction
        explicit Momentum(const UnitOfMeasure::LinearMotion& lm, const DirectX::XMVECTOR& dirIn) noexcept; // magnitude and direction
        explicit Momentum(const float& magIn, const DirectX::XMVECTOR& dirIn) { _magnitude = abs(magIn); _unit_direction = DirectX::XMVector3Normalize(dirIn); if (_magnitude != magIn) { _unit_direction = -_unit_direction; }; }
        explicit Momentum(const float& magIn, const float3 & dirIn) { _magnitude = abs(magIn); _unit_direction = float3::Normal(dirIn); if (_magnitude != magIn) { _unit_direction = -_unit_direction; }; }
        explicit Momentum(const UnitOfMeasure::Mass & m, const UnitOfMeasure::Speed & s, const float3 dirIn = DirectX::XMVectorZero()) { _magnitude = abs(s * m); _unit_direction = float3::Normal(dirIn); if (_magnitude != s * m) { _unit_direction = -_unit_direction; }; }
        Momentum(const float3 & vectorIn) { _magnitude = float3::Magnitude(vectorIn); _unit_direction = float3::Normal(vectorIn); }
        Momentum(const DirectX::XMVECTOR& vectorIn) noexcept;
        Momentum(const Momentum& m) noexcept : _magnitude(m._magnitude), _unit_direction(m._unit_direction) {}
        Momentum(Momentum&& m) noexcept : _magnitude(m._magnitude), _unit_direction(m._unit_direction) { m._magnitude = 0.0; m._unit_direction = DirectX::XMVectorZero(); }
        // Destructors
        ~Momentum() noexcept {};
        // Copy Assignment
        Momentum& operator=(const Momentum& m) noexcept
        {
            if (this != &m)
            {
                _magnitude = m._magnitude;
                _unit_direction = m._unit_direction;
            }
            return *this;
        }
        // Move Assignment
        Momentum& operator=(Momentum&& m) noexcept
        {
            if (this != &m)
            {
                _magnitude = m._magnitude;
                _unit_direction = m._unit_direction;

                m._magnitude = 0.0;
                m._unit_direction = DirectX::XMVectorZero();
            }
            return *this;
        }
        // Unit of Measure
        static const std::string Unit() { return UnitOfMeasure::LinearMotion::_unit; }
        static const std::wstring UnitW() { return UnitOfMeasure::LinearMotion::_wunit; }
        // Conversions
        inline explicit operator float() const { return _magnitude; }
        inline explicit operator UnitOfMeasure::LinearMotion() const { return _magnitude; }
        inline explicit operator float3() const { return GetVector(); }
        // Comparators
        inline bool operator<  (const float& rhs) const { return _magnitude < rhs; }
        inline bool operator<= (const float& rhs) const { return _magnitude <= rhs; }
        inline bool operator>  (const float& rhs) const { return _magnitude > rhs; }
        inline bool operator>= (const float& rhs) const { return _magnitude >= rhs; }
        inline bool operator== (const float& rhs) const { return _magnitude == rhs; }
        inline bool operator!= (const float& rhs) const { return _magnitude != rhs; }
        inline bool operator<  (const Momentum& rhs) const { return DirectX::XMVector3Less(GetVector(), rhs.GetVector()); }
        inline bool operator<= (const Momentum& rhs) const { return DirectX::XMVector3LessOrEqual(GetVector(), rhs.GetVector()); }
        inline bool operator>  (const Momentum& rhs) const { return DirectX::XMVector3Greater(GetVector(), rhs.GetVector()); }
        inline bool operator>= (const Momentum& rhs) const { return DirectX::XMVector3GreaterOrEqual(GetVector(), rhs.GetVector()); }
        inline bool operator== (const Momentum& rhs) const { return DirectX::XMVector3Equal(GetVector(), rhs.GetVector()); }
        inline bool operator!= (const Momentum& rhs) const { return DirectX::XMVector3NotEqual(GetVector(), rhs.GetVector()); }
        // Operators 
        void* operator new (size_t size) { return _aligned_malloc(size, 16); }
        void operator delete (void* p) { _aligned_free(static_cast<Momentum*>(p)); }
        explicit operator bool() const { return (bool)_magnitude && (bool)_unit_direction; } // valid
        bool operator !() const { return !(bool)_magnitude || !(bool)_unit_direction; } // invalid
        // Math Operators
        inline Momentum operator- () const { return Momentum(-_unit_direction); }
        inline Momentum operator+ (const Momentum& in) const { return Momentum(GetVector() + in.GetVector()); }
        inline Momentum operator- (const Momentum& in) const { return Momentum(GetVector() - in.GetVector()); }
        inline Momentum operator*(const float& s) const { return Momentum(_magnitude * s, _unit_direction); } // for fractions
        inline Momentum operator/(const float& s) const { return Momentum(_magnitude / s, _unit_direction); } // create fractional amount
        float operator/ (const Momentum& in) const; // unitless, ratio
        inline Momentum& operator+= (const Momentum& in) { *this = *this + in; return *this; }
        inline Momentum& operator-= (const Momentum& in) { *this = *this - in; return *this; }
        inline Momentum& operator*=(const float& s) { _magnitude = _magnitude * s; return *this; } // for ratios
        inline Momentum& operator/=(const float& s) { _magnitude = _magnitude / s; return *this; } // for ratios
        // Functionality
        void                                ApplyForceOverTime(const Force& f, const UnitOfMeasure::Time& t);
        void                                ApplyScalarForceOverTime(const UnitOfMeasure::Strength& s, const UnitOfMeasure::Time& t); // in same direction

        bool                                IsZero() const { return _magnitude == 0.f; }
        bool                                IsOrNearZero() const { return _magnitude <= 1.0e-5f; }
        // Accessors
        const auto&                         Get_magnitude() const { return _magnitude; }
        auto&                               Get_magnitude() { return _magnitude; }
        const auto&                         Get_unit_direction() const { return _unit_direction; }
        auto&                               Get_unit_direction() { return _unit_direction; }
        const float3                        GetVector() const { return _unit_direction * _magnitude; }
        float                               GetValueEN() const { return UnitOfMeasure::slugFtPerSec * _magnitude; } // slug⋅ft / s for EN linear momentum
        float                               GetValueSI() const { return UnitOfMeasure::kgMPerSec * _magnitude; } // kg⋅m/s for SI linear momentum
        // Assignments
        // Note: set unit direction before magnitude in case sign of magnitude is switched
        void                                Set_magnitude(const float& _magnitude_IN) { _magnitude = abs(_magnitude_IN); if (_magnitude != _magnitude_IN) { _unit_direction = -_unit_direction; }; }
        void __vectorcall                   Set_unit_direction(const float3 _unit_direction_IN) { _unit_direction = float3::Normal(_unit_direction_IN); }
        inline void                         SetZero() { _magnitude = 0.f; _unit_direction = DirectX::g_XMZero; }
        inline void                         SetZeroIfNear(const float epsilon = 0.00005f) { auto mask = DirectX::XMVectorLess(DirectX::XMVectorAbs(_unit_direction), DirectX::XMVectorReplicate(epsilon)); DirectX::XMVectorSelect(_unit_direction, DirectX::XMVectorZero(), mask); _unit_direction.Normalize(); }
        // Input & Output functions that can have access to protected & private data
        friend std::ostream& operator<< (std::ostream& os, const Momentum& in);
        friend std::istream& operator>> (std::istream& is, Momentum& out);
        friend std::wostream& operator<< (std::wostream& os, const Momentum& in);
        friend std::wistream& operator>> (std::wistream& is, Momentum& out);
        friend void to_json(json& j, const Momentum& from);
        friend void from_json(const json& j, Momentum& to);
    };
    // Input & Output Function forward declarations
    std::ostream& operator<< (std::ostream& os, const Momentum& in);
    std::istream& operator>> (std::istream& is, Momentum& out);
    std::wostream& operator<< (std::wostream& os, const Momentum& in);
    std::wistream& operator>> (std::wistream& is, Momentum& out);
    void to_json(json& j, const Momentum& from);
    void from_json(const json& j, Momentum& to);

    inline Momentum::Momentum(const UnitOfMeasure::LinearMotion& lm, const float3& dirIn) noexcept
    {
        _magnitude = abs(lm);
        _unit_direction = DirectX::XMVector3Normalize(dirIn);
        if (_magnitude != lm)
        {
            _unit_direction = DirectX::XMVectorNegate(_unit_direction);
        };
    }

    inline Momentum::Momentum(const UnitOfMeasure::LinearMotion& lm, const DirectX::XMVECTOR& dirIn) noexcept
    {
        _magnitude = abs(lm);
        _unit_direction = DirectX::XMVector3Normalize(dirIn);
        if (_magnitude != lm)
        {
            _unit_direction = DirectX::XMVectorNegate(_unit_direction);
        };
    }

    inline Momentum::Momentum(const DirectX::XMVECTOR& vectorIn) noexcept
    {
        _magnitude = DirectX::XMVectorGetX(DirectX::XMVector3Length(vectorIn));
        _unit_direction = DirectX::XMVector3Normalize(vectorIn);
    }
    inline void Momentum::ApplyForceOverTime(const Force& f, const UnitOfMeasure::Time& t)
    { 
        auto pi = float3(DirectX::XMVectorScale(_unit_direction, _magnitude));
        auto p1 = f * t;
        auto pf = pi + (float3)p1;
        _magnitude = pf.GetMagnitude();
        _unit_direction = float3::Normal(pf);
    }
    inline void Momentum::ApplyScalarForceOverTime(const UnitOfMeasure::Strength& s, const UnitOfMeasure::Time& t)
    {
        // force direction lacking, assume in current direction of momentum
        _magnitude += s * t;
    }

}
