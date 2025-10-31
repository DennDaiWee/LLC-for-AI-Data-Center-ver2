// Ai_CLLLC_NTC_settings is to calculate the temperature of LLC output sensed from NTC
#include "clllc.h"
#include "Ai_CLLLC_NTC_settings.h"

#define ADC_VREF 3.3
#define Rse 15000    // the resistance series with the NTC
#define TABLE_SIZE 121

// from -10 deg. to 110 deg.
uint32_t NTC_Rtable[TABLE_SIZE] = {
    55340, 52420, 49660, 47080, 44640, 42340, 40160, 38120, 36200, 34380,
    32660, 31040, 29500, 28060, 26680, 25400, 24180, 23020, 21920, 20880,
    19900, 18970, 18090, 17260, 16460, 15710, 15000, 14320, 13680, 13070,
    12490, 11940, 11420, 10920, 10450, 10000, 9574, 9166, 8778, 8408,
    8058, 7722, 7404, 7098, 6808, 6532, 6268, 6016, 5776, 5546,
    5326, 5118, 4918, 4726, 4544, 4368, 4202, 4042, 3888, 3742,
    3602, 3468, 3340, 3216, 3098, 2986, 2878, 2774, 2674, 2580,
    2488, 2400, 2316, 2234, 2158, 2082, 2012, 1942, 1876, 1813,
    1751, 1693, 1637, 1582, 1530, 1480, 1432, 1385, 1341, 1298,
    1256, 1216, 1178, 1141, 1105, 1071, 1038, 1006, 975, 945,
    916, 888, 862, 836, 811, 787, 764, 741, 720, 699,
    679, 659, 640, 622, 605, 588, 571, 555, 540, 525, 510
};

// Firstly, calculate the real value, then find out the close value of resistance.
// Finally do the liner interpolation to insert the decimal into the integer (temperature)
uint16_t Temp_cal(float vTemp_pu)
{
    float vTemp_real, Rntc, R1, R2, T1, T2, Temp_real;
    int i=0;

    vTemp_real = vTemp_pu * ADC_VREF;
    Rntc = Rse * vTemp_real / (ADC_VREF - vTemp_real);

    // obtain the internal
    while (Rntc < NTC_Rtable[i] && i < TABLE_SIZE-1)
        i++;

    // liner interpolation
    R1 = NTC_Rtable[i-1];
    R2 = NTC_Rtable[i];
    T1 = (i-1) - 10;   // corresponding temperature
    T2 = i - 10;

    Temp_real = T1 + (Rntc - R1) * (T2 - T1) / (R2 - R1);

    return Temp_real;
}
