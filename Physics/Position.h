/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:            Position    

Description:    

Usage:            

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
#include "..\Physics\Velocity.h"
#include "..\Physics\Distance.h"
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
******************************************************************************/

namespace King {

    /******************************************************************************
    *    Position
    *       w = 1.0f so that matrix transformation can be applied to a position
    *       in space.  Distinguishes between a distance (a 3 vector) and a location
    *       (a 4 vector with w=1).
    ******************************************************************************/
    class Position;


    class alignas(16) Position : public float4
    {
        /* variables */
    public:
    protected:
    private:
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Position> Create() { return std::make_shared<Position>(); }
        Position() { SetW(1.f); }
        inline Position(const FloatPoint3& vectorIn) { Set(vectorIn); }
        explicit inline Position(const float x, const float y, const float z) { Set(FloatPoint3(x, y, z)); }
        explicit inline Position(const DirectX::XMVECTOR &vIn) { Set(FloatPoint3(vIn)); }
        explicit inline Position(const DirectX::XMVECTORF32 &vIn) { Set(FloatPoint3(vIn)); }
        explicit inline Position(const FloatPoint2& vIn) { Set(vIn); }
        explicit inline Position(const FloatPoint4& vectorIn) { Set(static_cast<FloatPoint3>(vectorIn)); }
        explicit inline Position(const Distance &distanceIn) { Set(static_cast<FloatPoint3>(distanceIn)); }
        Position(const Position &in) { *this = in; } // forward to copy assignment
        Position(Position &&in) noexcept { *this = std::move(in); } // forward to move assignment

        virtual ~Position() noexcept { ; }

        static const std::string Unit() { return UnitOfMeasure::Length::_unit; }
        static const std::wstring UnitW() { return UnitOfMeasure::Length::_wunit; }

        // Conversions
        inline explicit operator float() const { return King::FloatPoint3::Magnitude(v); }
        inline explicit operator UnitOfMeasure::Length() const { return UnitOfMeasure::Length(King::FloatPoint3::Magnitude(v)); }
        inline explicit operator DirectX::XMVECTOR() const { return v; }
        inline operator King::FloatPoint3() const { return static_cast<King::FloatPoint3>(v); } // allow implicit for a default behavior
        inline explicit operator King::FloatPoint4() const { return v; }
        // Comparators
        inline bool operator<  (const Position& rhs) const { return DirectX::XMVector3Less(v, rhs.GetVecConst()); }
        inline bool operator<= (const Position& rhs) const { return DirectX::XMVector3LessOrEqual(v, rhs.GetVecConst()); }
        inline bool operator>  (const Position& rhs) const { return DirectX::XMVector3Greater(v, rhs.GetVecConst()); }
        inline bool operator>= (const Position& rhs) const { return DirectX::XMVector3GreaterOrEqual(v, rhs.GetVecConst()); }
        inline bool operator== (const Position& rhs) const { return DirectX::XMVector3Equal(v, rhs.GetVecConst()); }
        inline bool operator!= (const Position& rhs) const { return DirectX::XMVector3NotEqual(v, rhs.GetVecConst()); }
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Position*>(p)); }
        inline Position& operator= (const DirectX::XMVECTOR &in) { Set(FloatPoint4(in,1.f)); return *this; } // copy assignment
        //inline Position & operator= (const XMVECTORF32 &in) { Set(FloatPoint4(in,1.f)); return *this; } // copy assignment
        inline Position& operator= (const FloatPoint2& in) { Set(in); return *this; } // copy assignment
        inline Position& operator= (const FloatPoint3 &in) { Set(in); return *this; } // copy assignment
        inline Position& operator= (const FloatPoint4 &in) { Set(static_cast<FloatPoint3>(in)); return *this; } // copy assignment
        inline Position& operator= (const Position &other) { v = other.v; return *this; } // copy assign
        inline Position& operator= (Position &&other) noexcept { std::swap(v, other.v); return *this; } // move assign
        explicit operator bool() const { return (bool)float4::operator bool(); } // valid
        bool operator !() const { return float4::operator !(); } // invalid
        // Math Operators
        inline Position operator- () const { return -Position(v); }
        inline Position operator+ (const Position & in) const { return Position((FloatPoint3)v + in.v); }
        inline Distance operator- (const Position & in) const { return Distance((FloatPoint3)v - in.v); }
        inline Position operator* (const Position & in) const { return Position((FloatPoint3)v * in.v); }
        inline Position operator/ (const Position & in) const { return Position((FloatPoint3)v / in.v); }
        inline Position & operator+= (const Position & in) { *this = *this + in; return *this; } 
        inline Position & operator-= (const Position & in) { *this = Position((FloatPoint3)v - in.v); return *this; }
        inline Position & operator*= (const Position & in) { *this = *this * in; return *this; }
        inline Position & operator/= (const Position & in) { *this = *this / in; return *this; }

        inline Position operator- (const float & in) const { return Position((FloatPoint3)v - in); }
        inline Position operator+ (const float & in) const { return Position((FloatPoint3)v + in); }
        inline Position operator* (const float & in) const { return Position((FloatPoint3)v * in); }
        inline Position operator/ (const float & in) const { return Position((FloatPoint3)v / in); }

        inline Position operator- (const Distance &rhs) const { Position ptOut((FloatPoint3)v - (FloatPoint3)(rhs)); return ptOut; }
        inline Position operator+ (const Distance &rhs) const { Position ptOut((FloatPoint3)v + (FloatPoint3)(rhs)); return ptOut; }
        inline Position operator* (const Distance &rhs) const { Position ptOut((FloatPoint3)v * (FloatPoint3)(rhs)); return ptOut; }
        inline Position operator/ (const Distance &rhs) const { Position ptOut((FloatPoint3)v / (FloatPoint3)(rhs)); return ptOut; }
        inline Position & operator+= (const Distance & in) { *this = *this + in; return *this; }
        inline Position & operator-= (const Distance & in) { *this = *this - in; return *this; }
        inline Position & operator*= (const Distance & in) { *this = *this * in; return *this; }
        inline Position & operator/= (const Distance & in) { *this = *this / in; return *this; }

        inline Position operator- (const FloatPoint3 &rhs) const { Position ptOut((FloatPoint3)v - (FloatPoint3)(rhs)); return ptOut; }
        inline Position operator+ (const FloatPoint3 &rhs) const { Position ptOut((FloatPoint3)v + (FloatPoint3)(rhs)); return ptOut; }
        inline Position operator* (const FloatPoint3 &rhs) const { Position ptOut((FloatPoint3)v * (FloatPoint3)(rhs)); return ptOut; }
        inline Position operator/ (const FloatPoint3 &rhs) const { Position ptOut((FloatPoint3)v / (FloatPoint3)(rhs)); return ptOut; }
        inline Position & operator-= (const FloatPoint3 &rhs) { *this = *this - rhs; return *this; }
        inline Position & operator+= (const FloatPoint3 &rhs) { *this = *this + rhs; return *this; }
        inline Position & operator*= (const FloatPoint3 &rhs) { *this = *this * rhs; return *this; }
        inline Position & operator/= (const FloatPoint3 &rhs) { *this = *this / rhs; return *this; }

        inline Position & operator*= (const DirectX::XMMATRIX &m) { v = DirectX::XMVector4Transform(v, m); return *this; }
        friend Position operator* (Position lhs, const DirectX::XMMATRIX &m) { lhs *= m; return lhs; } // invokes std::move(lhs)

        // Init/Start/Stop/Destroy
        // Functionality
        bool                                IsZero() { return DirectX::XMVector3Equal(GetVector(), DirectX::XMVectorZero()); }
        bool                                IsOrNearZero() { return DirectX::XMVector3NearEqual(GetVector(), DirectX::XMVectorZero(), DirectX::XMVectorReplicate(1.e-5f)); }
        
        // Accessors
        float3                              To_SphericalCoordinates() const; // converts to rho, theta, phi
        [[deprecated("Use GetVector() instead to match other physics classes.")]]
        DirectX::XMVECTOR                   GetVec() { return v; }
        DirectX::XMVECTOR &                 GetVector() { return v; } // modifiable type
        DirectX::XMVECTOR                   GetVecConst() const { return v; }
        // Assignments
        inline void __vectorcall            Set(const float2 positionXYIn) { v = positionXYIn; SetZ(0.0f); SetW(1.0f); }
        inline void __vectorcall            Set(const float3 positionXYZIn) { v = positionXYZIn; SetW(1.0f); }
        inline void __vectorcall            Set(const DirectX::XMVECTOR positionXYZIn) { Set(FloatPoint3(positionXYZIn)); }
        inline void __vectorcall            Set(const DirectX::XMVECTORF32 positionXYZIn) { Set(FloatPoint3(positionXYZIn)); }
        void __vectorcall                   Set_SphericalCoordinates(float3 rhoThetaPhiIn) { DirectX::XMFLOAT3 rtp(rhoThetaPhiIn); Set_SphericalCoordinates(rtp.x, rtp.y, rtp.z); }
        void                                Set_SphericalCoordinates(const float& rhoIn, const float& thetaIn, const float& phiIn);

        // Input & Output functions that can have access to protected & private data
        friend std::ostream& operator<< (std::ostream& os, const Position& in);
        friend std::istream& operator>> (std::istream& is, Position& out);
        friend std::wostream& operator<< (std::wostream& os, const Position& in);
        friend std::wistream& operator>> (std::wistream& is, Position& out);
        friend void to_json(json& j, const Position& from);
        friend void from_json(const json& j, Position& to);
    };
    // Input & Output Function forward declarations
    std::ostream& operator<< (std::ostream& os, const Position& in);
    std::istream& operator>> (std::istream& is, Position& out);
    std::wostream& operator<< (std::wostream& os, const Position& in);
    std::wistream& operator>> (std::wistream& is, Position& out);
    void to_json(json& j, const Position& from);
    void from_json(const json& j, Position& to);
}