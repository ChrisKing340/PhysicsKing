# PhysicsKing

This repository contains basic physics classes to implement [Sir Isac Newton's](https://en.wikipedia.org/wiki/Isaac_Newton) 3 Laws of physics. 
## Why do you need this?
    1) Data management of scalar and vector direction speeds up your code when accessed > 1 per update.
    2) Data validation along with zeroing component wise vectors when near epsilon (resetting floating point error accumulation).
    3) SIMD Math acceleration.
    4) High degree of inline code for readability and compiled only if used. 
    5) Some basic derived formula are included for both linear and angular motion.
    6) Optional Unit of measures for both SI and English Imperial.
    7) Data visualization enable through stream output formating.
    8) JSON built in for serializing and saving/transporting data in a standard format.
    
Intrinsic SIMD acceleration from project:
Math: [https://github.com/ChrisKing340/MathSIMD](https://github.com/ChrisKing340/MathSIMD)  
   
C\+\+ classes support json for data transport. For the latest version of json visit:
json: [https://github.com/nlohmann/json](https://github.com/nlohmann/json)

  stream output:
##  Scalar:
  Mass m;
  m = 10._kg;
  Accel a0;
  a0 = 4._mPerSecSq;
  auto F = m * a0;
  
  cout << "F = " << m << " * " << a0 << "\n";
###  F = { Mass 10 kg } * { Accel 4 m/s^2 }
  cout << "F = " << F << "\n";
###  F = { Strength 40 N }

##  Vector:
  Acceleration a;
  auto direction = float3(0.f, 1.f, 0.f);
  a.Set_magnitude(a0);
  a.Set_unit_direction(direction); 
  auto F = m * a;
  cout << "F = " << F << "\n";
###  F = {{ Strength 40 N } * Dir:{ x:         0 y:         1 z:         0 } }
  cout << "F = " << F.GetVector() << "\n";
###  F = { x:         0 y:        40 z:         0 }

Compiled with Visual Studio 2019, C\+\+17, 64 Bit Windows 10

This code is the foundation of a fully functional DirectX 12 game engine and physics simulator.

## Physics foundation
Foundation classes represent a unit of measure with a scalar.  The classes just keeps one value with internal storage as a SI unit of measure.  String literals implemented to allow definition with the unit of measure desired.  Operator overloading to act as a base type and also supports streams and json from/to;
Ex: Length(10_m) Length(10_ft)

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
