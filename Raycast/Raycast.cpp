// Raycast.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "Physics.h"
#include "UnitTest.h"
#include "SpeedTest.h"
#include <iostream>

int main()
{
    bool result = Physics::UnitTest();

    if (result)
    {
        std::cout << "Tests Passed" << std::endl;
        float time = SpeedTest();
        std::cout << "Time = " << time << " ms" << std::endl;
    }
    else
    {
        std::cout << "Tests Failed" << std::endl;
    }

    return result ? 0 : -1;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
