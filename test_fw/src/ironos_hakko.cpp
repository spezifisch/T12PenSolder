// code based on IronOS, v2.21, GPLv3: https://github.com/Ralim/IronOS/blob/v2.21/LICENSE
// source files:
// - https://github.com/Ralim/IronOS/blob/v2.21/source/Core/Drivers/Utils.cpp
// - https://github.com/Ralim/IronOS/blob/v2.21/source/Core/BSP/Pinecilv2/ThermoModel.cpp (Pinecil also uses T12 tips)

#include <inttypes.h>

namespace IronOS
{
    // Pinecilv2 T12 curve
    static const int32_t uVtoDegC[] = {
        0, 0,       //
        266, 10,    //
        522, 20,    //
        770, 30,    //
        1010, 40,   //
        1244, 50,   //
        1473, 60,   //
        1697, 70,   //
        1917, 80,   //
        2135, 90,   //
        2351, 100,  //
        2566, 110,  //
        2780, 120,  //
        2994, 130,  //
        3209, 140,  //
        3426, 150,  //
        3644, 160,  //
        3865, 170,  //
        4088, 180,  //
        4314, 190,  //
        4544, 200,  //
        4777, 210,  //
        5014, 220,  //
        5255, 230,  //
        5500, 240,  //
        5750, 250,  //
        6003, 260,  //
        6261, 270,  //
        6523, 280,  //
        6789, 290,  //
        7059, 300,  //
        7332, 310,  //
        7609, 320,  //
        7889, 330,  //
        8171, 340,  //
        8456, 350,  //
        8742, 360,  //
        9030, 370,  //
        9319, 380,  //
        9607, 390,  //
        9896, 400,  //
        10183, 410, //
        10468, 420, //
        10750, 430, //
        11029, 440, //
        11304, 450, //
        11573, 460, //
        11835, 470, //
        12091, 480, //
        12337, 490, //
        12575, 500, //

    };
    static const int uVtoDegCItems = sizeof(uVtoDegC) / (2 * sizeof(uVtoDegC[0]));

    static int32_t LinearInterpolate(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x) { return y1 + (((((x - x1) * 1000) / (x2 - x1)) * (y2 - y1))) / 1000; }

    static int32_t InterpolateLookupTable(const int32_t *lookupTable, const int noItems, const int32_t value)
    {
        if (value)
        {
            for (int i = 1; i < (noItems - 1); i++)
            {
                // If current tip temp is less than current lookup, then this current lookup is the higher point to interpolate
                if (value < lookupTable[i * 2])
                {
                    return LinearInterpolate(lookupTable[(i - 1) * 2], lookupTable[((i - 1) * 2) + 1], lookupTable[i * 2], lookupTable[(i * 2) + 1], value);
                }
            }
            return LinearInterpolate(lookupTable[(noItems - 2) * 2], lookupTable[((noItems - 2) * 2) + 1], lookupTable[(noItems - 1) * 2], lookupTable[((noItems - 1) * 2) + 1], value);
        }
        return 0;
    }

    uint32_t convertuVToDegC(uint32_t tipuVDelta) { return InterpolateLookupTable(uVtoDegC, uVtoDegCItems, tipuVDelta); }
}