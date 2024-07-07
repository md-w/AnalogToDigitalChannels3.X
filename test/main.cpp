#include <iostream>
#include <catch2/catch_test_macros.hpp>
#include <cstdint>

int get_value(int iADCValCh, signed char cCCh, signed char cMCh)
{
    // y = -(760/819)*(x-921)
    // y = -(760/819)*(x + cC/10 -921)
    // y = -(760/8190)*(10*x + cC -921*10)
    // y = -(760/8190)*(10*x*(127 + cM)/127 + cC -921*10)
    // y = -(760/(8190*127))*(10*x*(127 + cM) + cC*127 -921*10*127)
    // y = -760*(10*x*(127 + cM) + cC*127 -921*10*127)/(8190*127)

    // return -(760 * (((long)iADCValCh + (long)cCCh) * ((127 + cMCh) / 127) - 921)) / (921 - 102);
    return -760 * (10 * iADCValCh * (127 + cMCh) + cCCh * 127 - 921 * 10 * 127) / (8190 * 127);
    // return -((760 + (long)cMCh) * (long)iADCValCh) / 819 + 855 + (long)cCCh;
}
TEST_CASE("minimum 102", "[adc]")
{
    REQUIRE(get_value(102, 1, 0) == 761);
}
TEST_CASE("minimum 103", "[adc]")
{
    REQUIRE(get_value(103, 0, 0) == 760);
}
TEST_CASE("minimum 104", "[adc]")
{
    REQUIRE(get_value(104, 0, 0) == 759);
}
TEST_CASE("minimum 105", "[adc]")
{
    REQUIRE(get_value(105, 0, 0) == 758);
}
TEST_CASE("minimum 512", "[adc]")
{
    REQUIRE(get_value(512, 0, 0) == 380);
}
TEST_CASE("minimum 921", "[adc]")
{
    REQUIRE(get_value(921, 0, 0) == 1);
}
TEST_CASE("minimum 922", "[adc]")
{
    REQUIRE(get_value(922, 0, 0) == 0);
}
// int main(int argc, char const *argv[])
// {
//         /* code */
//         return 0;
// }

// // x :     0     0.5   4.5      5
// // x :     0     102   921      1023
// // y :           760     0

// int iRoomTemp1 = (((long) iADCValCh1 + (long) SetPoint.cCCh1)*((long) ANALOG_FULLSCALE + (long) SetPoint.cMCh1 * 10)) / (long) 10230;
// int iRoomTemp2 = (((long) iADCValCh2 + (long) SetPoint.cCCh2)*((long) ANALOG_FULLSCALE + (long) SetPoint.cMCh2 * 10)) / (long) 10230;
// //        int iRoomTemp3 = (((long) iADCValCh3 + (long) SetPoint.cCCh3)*((long) ANALOG_FULLSCALE + (long) SetPoint.cMCh3 * 10)) / (long) 10230;

// int iRoomTemp3 = -((760 + (long) SetPoint.cMCh3)*(long) iADCValCh3) / 819 + 855 + (long) SetPoint.cCCh3;