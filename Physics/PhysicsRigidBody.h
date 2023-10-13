/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:          PhysicsRigidBody    

Description:    

Usage:            

Contact:        ChrisKing340@gmail.com

References:        

MIT License

Copyright (c) 2020 Christopher H. King

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
#include "..\MathSIMD\MathSIMD.h"

#include "UnitOfMeasure.h" // has own namespace King::UnitOfMesure::
#include "Force.h"
#include "Acceleration.h"
#include "Velocity.h"
#include "Momentum.h"
#include "Distance.h"
#include "Position.h"
#include "Torque.h"
#include "AngularAcceleration.h"
#include "AngularVelocity.h"
#include "Rotation.h"
#include "PhysicsMaterial.h"
#include "PhysicsState.h"

#include "..\..\GeometryKing\3DGeometryKing\3DGeometry.h"

// 3rd Party namespace
#include "..\..\json\single_include\nlohmann\json.hpp"
using json = nlohmann::json;

namespace King {

    class alignas(16) PhysicsRigidBody
    {
        /* variables */
    public:
        std::vector<std::pair<Force, Distance>> _forcesActingOnBody; // after update and moved into velocity, this vector is cleared
        // *** TO DO *** move HasDrag() and HasGravity() from MoveableObj to this Class as force vectors and use data from this class

        static const bool posCorrectOnOff = true; // apply a constant repulsive force so objects do not "sink" into one another

        DirectX::XMMATRIX                       _inertiaTensorInverse; // *** TO DO *** keep a geometry factor (radius) and change this based on mass & radius (for a sphere)

    protected:
        bool                                    _stationary = false;

        std::shared_ptr<PhysicsMaterial>        _sp_material;
        float                                   _coefficientOfFriction = 0.1f; // surface roughness used in collision resolution to impart rotation from an oblique contact
        float                                   _coefficientOfDrag = 0.f;
        float                                   _crossSectionalArea = 1.f; // in direction of motion
        float                                   _volume = 0.f; // m^3
        UnitOfMeasure::Mass                     _mass = { 1.f }; // kg
        // *** TO DO ** geometry based Inertia, do we need pure virtual accessor ?
        //Position                                _centerOfMassLocalSpace;  // affinity transform, set Pose rotation center *** TO DO ***
        
        PhysicsState                            i; // initial before Update(const UnitOfMeasure::Time& dt)
        PhysicsState                            f; // final after Update(const UnitOfMeasure::Time& dt)
    private:
        float                                   _invMass = 1.f;
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<PhysicsRigidBody> Create() { return std::make_shared<PhysicsRigidBody>(); }
        PhysicsRigidBody() 
        { 
            Set_mass(_mass);
        }
        PhysicsRigidBody(const PhysicsRigidBody &in) { *this = in; } // forward to copy assignment
        PhysicsRigidBody(PhysicsRigidBody &&in) noexcept { *this = std::move(in); } // forward to move assignment

        virtual ~PhysicsRigidBody() { ; }

        // Conversions
        // Comparators
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<PhysicsRigidBody*>(p)); }
        inline PhysicsRigidBody& operator= (const PhysicsRigidBody& other) = default; // copy assign
        inline PhysicsRigidBody& operator= (PhysicsRigidBody&& other) = default; // move assign
        // Math Operators
        // Init/Start/Stop/Destroy
        // Functionality
        virtual void    Update(const UnitOfMeasure::Time& dt);

        // Geometry3D has a Contact class. We would loop through that class contact points and call the method below
        // e we should look up from a table of e[mtl1][mtl2];
        void            ProcessCollisionWithObject(PhysicsRigidBody & moInOut, const float e, const UnitOfMeasure::Time& dt, float3 collisionPtIN, const float penetrationIn);

        // Accessors
        auto& GetInitialState() { return i; }
        auto& GetFinalState() { return f; }
        const auto& Get_mass() const { return _mass; }
        const auto& Get_invMass() const { return _invMass; }
        const auto& Get_inertiaTensorInverse() const { return _inertiaTensorInverse; }
        const auto& Get_coefficientOfFriction() const { return _coefficientOfFriction; }
        // Assignments
        void Set(const std::shared_ptr<PhysicsMaterial> sp_material) { _sp_material = sp_material; }

        void Set_volume(const float& volume) { _volume = volume; }
        void Set_crossSectionalArea(const float& crossSectionalArea) { _crossSectionalArea = crossSectionalArea; }
        void Set_coefficientOfDrag(const float& coefficientOfDrag) { _coefficientOfDrag = coefficientOfDrag; }
        void Set_mass(const UnitOfMeasure::Mass& mass) 
        { 
            _mass = mass; 
            _invMass = 1.0f / _mass; 

            // assume a sphere
            /*
            Moment Of Inertia of solid sphere
            We will calculate the moment of inertia of a solid sphere by integrating multiple inertias of the disc.
            
            Inertia is defined as:
                    n
                Icm = ∑ Mi * ri²
                    i=1
            For a uniform disk of radius r and total mass M we can just integrate:
                Icm = ∫ r² dM

            For a uniform solid sphere, we integrate the discs along one axis of the sphere (since the sphere is uniform, it does not mater and every axis is the same)
                Icm = Icmy = Icmx = Icmz
            The integrals for each is then:
                Icmx = ∫ r² dM = (y² + z²) dM
                Icmy = ∫ r² dM = (x² + z²) dM
                Icmz = ∫ r² dM = (x² + y²) dM
            Instead of taking three integrals and knowing that uniform means each axis will be the same, we can take a short cut and average them by stating that the 
            sum of each integral will give us three times the moment of inertia for the solid sphere, about the central axis:
                3 * Icm = Icmx + Icmy + Icmz
                3 * Icm = ∫ (y² + z²) dM + ∫ (x² + z²) dM + ∫ (x² + y²) dM
                3 * Icm = ∫ (x² + x² + y² + y² + z² + z²) dM
                3 * Icm = 2 * ∫ (x² + y²+ z²) dM
            Since r² = x² + y²+ z²:
                3 * Icm = 2 * ∫ r² dM
                Icm = 2/3 * ∫ r² dM
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
            float r = 1.0f; // radius
            auto i = 2.0f / 5.0f * _invMass * r * r;

            auto I = DirectX::XMMATRIX(i, 0, 0, 0,
                0, i, 0, 0,
                0, 0, i, 0,
                0, 0, 0, 1.f);

            _inertiaTensorInverse = DirectX::XMMatrixInverse(nullptr, I);
        }
        //void Set_centerOfMassLocalSpace(const Position& centerOfMassLocalSpace) { _centerOfMassLocalSpace = centerOfMassLocalSpace; }

    protected:
        // Internal Helpers
    };
}