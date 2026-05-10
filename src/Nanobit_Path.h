#include <Arduino.h>
#include "Nanobit_Trajectory.h"
class NanobitPath
{
public:
    static void lerp(const nfloat *qA, const nfloat *qB, nfloat u, nfloat *qOut, uint8_t dof)
    {
        if (u < 0)
            u = 0;
        if (u > 1)
            u = 1;
        for (uint8_t i = 0; i < dof; i++)
            qOut[i] = qA[i] + (qB[i] - qA[i]) * u;
    }
};