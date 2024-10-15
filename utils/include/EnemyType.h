#pragma once

#include <map>

namespace LY_UTILS
{
    enum ENEMY_TYPE
    {
        Base = 0,
        Hero = 1,
        Engineer = 2,
        Infantry3 = 3,
        Infantry4 = 4,
        Infantry5 = 5,
        Sentry = 6,
        Outpost = 7,
        UNKNOW_TYPE = -1
    };

    enum class ARMOR_SIZE
    {
        BIG_ARMOR,
        SMALL_ARMOR,
        UNKNOW
    };

    typedef std::map<ENEMY_TYPE,ARMOR_SIZE> ARMOR_SIZE_MAP;
}