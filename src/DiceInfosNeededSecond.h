#ifndef _DICEINFOSNEEDED_H_
#define _DICEINFOSNEEDED_H_

#include <Arduino.h>

enum DiceSide
{
    SideOne = 1,
    SideTwo = 2,
    SideThree = 3,
    SideFour = 4,
    SideFive = 5,
    SideSix = 6,
    Unknown = 7
};

class DiceInfosNeeded
{
private:
    int16_t firstAx;
    int16_t firstAy;
    int16_t firstAz;
    int16_t currentAx;
    int16_t currentAy;
    int16_t currentAz;

    int16_t onSideValue = 15000;

    int currentDiceSide;

public:
    DiceInfosNeeded();
    ~DiceInfosNeeded();

    int getCurrentDiceSide() { return currentDiceSide; };

    void setInitialValue(int16_t firstAx, int16_t firstAy, int16_t firstAz, int currentDiceSide);
    void getNewSideAlgo(int16_t newAx, int16_t newAy, int16_t newAz);
    void setNewSide(DiceSide diceSide) { currentDiceSide = diceSide; };

    DiceSide wichSide(int16_t ax, int16_t ay, int16_t az);
};

#endif