
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:          PhysicsKing

Description:    Floating point physics for faster calculations and data storage.
                Not suitable for precise calculations requiring many digits of
                significant figures. Suitable for "text book" math and problems,
                simulations as approximations, and games. Uses fast floating
                points for speed rather than double floating point precision 
                suited for scientific research and precise engineering 
                calculations.

Contact:        ChrisKing340@gmail.com

References:     json https://github.com/nlohmann/json
                SIMD math functions utilize https://github.com/microsoft/DirectXMath
                Math class wrappers 
                Physics primitives 
                Geometric primitives, intersection testing, meshes https://github.com/ChrisKing340/GeometryKing

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

#include <iostream>
// Math
#include "..\MathSIMD\MathSIMD.h"
// Physics
#include "..\PhysicsKing\Physics.h"

using namespace std;
using namespace King;
using namespace UnitOfMeasure;

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */

int main()
{
    cout << CYAN;
    cout << " , __  _                              ,         \n";
    cout << "/|/  \\| |              o             /|   / o   \n";
    cout << " |___/| |           ,      __   ,     |__/      _  _    __, \n";
    cout << " |    |/ \\   |   | / \\_|  /    / \\_   | \\   |  / |/ |  /  | \n";
    cout << " |    |   |_/ \\_/|/ \\/ |_/\\___/ \\/    |  \\_/|_/  |  |_/\\_/|/\n";
    cout << "                /|                                       /| \n";
    cout << "                \\|                                       \\| \n";
    cout << "Author: Christopher H. King, 2021-2023\n";
    cout << RESET;

    // Physics class object vectors
    // Linear           // Rotational
    Force F;            Torque torque;
    Acceleration a;     AngularAcceleration alpha;
    Velocity v;         AngularVelocity omega;
    Position s;         Rotation theta;
    Distance r;         Rotation epsilon;
    Momentum p;

    // Physics class objects for bodies and states
    PhysicsMaterial phyMtl;
    PhysicsState phyState;
    PhysicsRigidBody phyObj;

    // UnitOfMeasure class scalars to keep different units of measured values
    // see the header file for string literals to use (ex: _kg, _s, _m)
    Mass m;
    Inertia I;
    Energy E;
    Power P;
    Temperature T;
    Time t;
    // Linear                   // Rotational
    Length l;                   Angle angle;
    Strength linearStrength;    AngularStrength angularStrength;
    Accel lAccel;               AngularAccel aAccel;
    Speed lSpeed;               AngularSpeed aSpeed;
    LinearMotion lMotion;       AngularMotion aMotion;
    
    Area area;
    Volume vol;   

    // UnitOfmeasure squared terms exist as intermediates to aide in 
    // math and unit tracking (not listed here)

    //>>>>>>>>>>>>>>>> TEST CASE <<<<<<<<<<<<<<<<<<<<
    if (true) 
    {
        cout << CYAN;
        cout << "\nTest 1, Force:\n";
        cout << RESET;
        cout << "  " << "F = m * a\n";
        cout << "  " << "Scalar:\n";
        m = 10._kg;
        lAccel = 4._mPerSecSq;
        linearStrength = m * lAccel;
        cout << "  " << "F = " << m << " * " << lAccel << "\n";
        cout << "  " << "F = " << linearStrength << "\n";
        cout << "  " << (linearStrength == 40.f ? GREEN + "PASS"s + RESET : RED + "FAIL"s + RESET) << "\n";
        cout << "  " << "Vector:\n";
        float3 direction(1.0f, 0.f, 0.f);
        a.Set_magnitude(lAccel);
        a.Set_unit_direction(direction);
        F = m * a;
        cout << "  " << "F = " << m << " * " << a << "\n";
        cout << "  " << "F = " << F << "\n";
        cout << "  " << "F = " << F.GetVector() << "\n";
        cout << "  " << (F == 40.f ? GREEN + "PASS"s + RESET : RED + "FAIL"s + RESET) << "\n";
        cout << "\n";

        cout << CYAN;
        cout << "  Torque:\n";
        cout << RESET;
        cout << "  " << "T = I * alpha\n";
        cout << "  " << "Scalar:\n";
        I = 10._kgMSq;
        aAccel = 4._radPerSecSq;
        angularStrength = I * aAccel;
        cout << "  " << "T = " << I << " * " << aAccel << "\n";
        cout << "  " << "T = " << angularStrength << "\n";
        cout << "  " << (angularStrength == 40.f ? GREEN + "PASS"s + RESET : RED + "FAIL"s + RESET) << "\n";
        cout << "  " << "Vector:\n";
        direction = float3(0.f, 1.f, 0.f);
        alpha.Set_magnitude(aAccel);
        alpha.Set_unit_direction(direction);
        torque = I * alpha;
        cout << "  " << "T = " << I << " * " << alpha << "\n";
        cout << "  " << "T = " << torque << "\n";
        cout << "  " << "T = " << torque.GetVector() << "\n";
        cout << "  " << (torque == 40.f ? GREEN + "PASS"s + RESET : RED + "FAIL"s + RESET) << "\n";
    }
    //>>>>>>>>>>>>>> END TEST CASE <<<<<<<<<<<<<<<<<<
    
    //>>>>>>>>>>>>>>>> TEST CASE <<<<<<<<<<<<<<<<<<<<
    if (true) 
    {
        cout << CYAN;
        cout << "\nTest 2, Energy:\n";
        cout << "  Kinetic Energy of Motion:\n";
        cout << RESET;
        cout << "  " << "KEf - KEi = 1/2 * m * ( vf^2 - vi^2 )\n";
        cout << "  " << "Scalar:\n";
        m = 5._kg;
        Speed si = 10._mPerSec;
        Speed sf = 20._mPerSec;
        E = 0.5f * m * (sf*sf - si*si);
        cout << "  " << "E = 1/2 * " << m << " * (" << sf * sf << " - " << si * si << ")\n";
        cout << "  " << "E = " << E << "\n";
        cout << "  " << (E == 750.f ? GREEN + "PASS"s + RESET : RED + "FAIL"s + RESET) << "\n";
        cout << "  " << "Vector:\n";
        float3 direction(0.f, 0.f, 1.f);
        Velocity vf(sf, direction);
        Velocity vi(si, direction);
        cout << "  vi = " << vi << "\n";
        cout << "  vf = " << vf << "\n";
        E = 0.5f * m * (vf*vf - vi*vi);
        cout << "  " << "E = 1/2 * " << m << " * (" << vf*vf << " - " << vi * vi << ")\n";
        cout << "  " << "E = " << E << "\n";
        cout << "  " << (E == 750.f ? GREEN + "PASS"s + RESET: RED + "FAIL"s + RESET) << "\n";
    }
    //>>>>>>>>>>>>>> END TEST CASE <<<<<<<<<<<<<<<<<<

    //>>>>>>>>>>>>>>>> TEST CASE <<<<<<<<<<<<<<<<<<<<
    if (true)
    {
        cout << CYAN;
        cout << "\nTest 3, Momentum:\n";
        cout << RESET;
        cout << "  " << "p = m * v\n";
        cout << "  " << "Scalar:\n";
        m = 5._kg;
        lSpeed = 20._mPerSec;
        auto pS = m * lSpeed;
        cout << "  " << "p = " << m << " * " << lSpeed << "\n";
        cout << "  " << "p = " << pS << "\n";
        cout << "  " << (pS == 100.f ? GREEN + "PASS"s + RESET : RED + "FAIL"s + RESET) << "\n";
        cout << "  " << "Vector:\n";
        m = 5._kg;
        v = Velocity(20._mPerSec, float3(1.f, 0.f, 0.f));
        p = m * v;
        cout << "  " << "p = " << m << " * " << v << "\n";
        cout << "  " << "p = " << p << "\n";
        cout << "  " << "p = " << p.GetVector() << "\n";
        cout << "  " << (p == 100.f ? GREEN + "PASS"s + RESET : RED + "FAIL"s + RESET) << "\n";
    }
    //>>>>>>>>>>>>>> END TEST CASE <<<<<<<<<<<<<<<<<<
}