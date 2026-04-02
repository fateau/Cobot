#pragma once

/* Simulator is not used in acontis/Linux build.
   The cyclic task is handled by EcMasterJobTask in AppLayer.cpp. */

#include <cstdio>
#include "Def.h"

class Simulator
{
public:
    static void Start() {}
    static void Stop()  {}
};