#ifndef _HAL_H_
#define _HAL_H_

#include "../../../USER_SETTINGS.h"

#if defined(HW_LILYGO)
#include "hw_lilygo.h"
#elif defined(HW_STARK)
#include "hw_stark.h"
#elif defined(HW_3LB)
#include "hw_3LB.h"
#elif defined(HW_M5_ATOM_CANBUS_BASE)
#include "hw_m5_atom_canbus_base.h"
#endif

#endif
