#include "DiceInfosNeededSecond.h"

DiceInfosNeeded::DiceInfosNeeded()
    : firstAx(0), firstAy(0), firstAz(0), currentDiceSide(DiceSide::Unknown)
{

}

DiceInfosNeeded::~DiceInfosNeeded()
{

}

void DiceInfosNeeded::setInitialValue(int16_t ax, int16_t ay, int16_t az, int diceSide)
{
    firstAx = ax;
    firstAy = ay;
    firstAz = az;
    currentDiceSide = diceSide;
}

void DiceInfosNeeded::getNewSideAlgo(int16_t newAx, int16_t newAy, int16_t newAz)
{
    //TODO: do algorithm if necessary
}

DiceSide DiceInfosNeeded::wichSide(int16_t ax, int16_t ay, int16_t az)
{
    if(ax >= onSideValue)
    {
        return DiceSide::SideSix;
    }
    else if(ax <= (-onSideValue))
    {
        return DiceSide::SideThree;
    }
    else if(ay >= onSideValue)
    {
        return DiceSide::SideFive;
    }
    else if(ay <= (-onSideValue))
    {
        return DiceSide::SideOne;
    }
    else if(az >= onSideValue)
    {
        return DiceSide::SideFour;
    }
    else if(az <= (-onSideValue))
    {
        return DiceSide::SideTwo;
    }
    else 
    {
        return DiceSide::Unknown;
    }
}