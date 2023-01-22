# PhysicsKing

Basic physics classes representing real world motion. High degree of inline code for readability and compiled only if used. Intended as the foundation for "made from scratch" game engines. Intrinsic SIMD acceleration from project:

Math: [https://github.com/ChrisKing340/MathSIMD](https://github.com/ChrisKing340/MathSIMD)
   
   
C\+\+ classes support json for data transport. For the latest version of json visit:
json: [https://github.com/nlohmann/json](https://github.com/nlohmann/json)

Compiled with Visual Studio 2019, C\+\+17, 64 Bit Windows 10

This code is the foundation of a fully functional DirectX 12 game engine and physics simulator.

## Physics foundation
Foundation classes represent a unit of measure with a scalar.  The classes just keeps one value with internal storage as a SI unit of measure.  String literals implemented to allow definition with the unit of measure desired.  Operator overloading to act as a base type and also supports streams and json from/to;
Ex: Length(10_m)
Ex: Length(10_ft)

    #include "Physics\UnitOfMeasure.h"
    namespace UnitOfMeasure;
    class Mass; // scalar
    class Length; // scalar
    class Area; // scalar
    class Volume; // scalar
    class Energy; // scalar
    class Power; // scalar
    class Strength; // scalar part of a Force vector
    class Accel; // scalar part of an Acceleration vector
    class Speed; // scalar part of a Velocity vector
    class Temperature; // scalar
    class Time; // scalar
    const Accel gravity;
    const Speed speedOfSoundInAir;
    
## Physics simulation

Modeling of force, acceleration, and velocity of dynamic bodies.

    #include "Physics.h"

    // Linear
    class Force ; // keeps a UnitOfMeasure::Strength scalar and a unit direction vector; Implements Newton's 1st Law
    class Acceleration ; // keeps a UnitOfMeasure::Accel scalar and a unit direction vector; Implements Newton's 2nd Law
    class Velocity ; // keeps a UnitOfMeasure::Speed scalar and a unit direction vector; operator for Velocity = Acceleration * Time
    class Distance ; // keeps a UnitOfMeasure::Length scalar and a unit direction vector (essentially a distance to); operator for Distance = Velocity * Time
    class Position ; // keeps 3 floats for x,y,z and operators for arithmetic with Distance
    class Momentum ; // keeps a UnitOfMeasure::LinearMotion scalar and a unit direction vector. Implements Newton's 3rd Law
    // Angular
    class Torque ; // keeps a UnitOfMeasure::AngularStrength scalar and a unit direction vector representing the axis of force is acting about
    class AngularAcceleration ; // keeps a UnitOfMeasure::AngularAccel scalar and a unit direction vector representing the normalized axis angles, 𝛼, in pitch, yaw, roll 𝛼(𝒾, 𝒿, 𝓀)
    class AngularVelocity ; // keeps a UnitOfMeasure::AngularSpeed scalar and a unit direction vector representing the normalized axis angles, 𝛼, in pitch, yaw, roll 𝛼(𝒾, 𝒿, 𝓀)
    class Rotation ; // keeps a UnitOfMeasure::Angle scalar and a unit direction vector representing the normalized axis angles, 𝛼, in pitch, yaw, roll 𝛼(𝒾, 𝒿, 𝓀). Also keeps a Quaternion. Advantage is that rotation can be > 2 PI and the quaternion will be calculated correctly (domain of a quaternion is -2 PI to +2 PI) and also the unit rotation vector is the axis of rotation in x,y,z