#ifndef _HAL_H_
#define _HAL_H_

#include "../../../USER_SETTINGS.h"

#if defined(HW_LILYGO)
#include "hw_lilygo.h"
#elif defined(HW_STARK)
#include "hw_stark.h"
#elif defined(HW_SJB_V1)
#include "hw_sjb_v1.h"
#elif defined(HW_M5_ATOM)
#include "hw_m5_atom.h"
#elif defined(HW_3LB)
#include "hw_3LB.h"
#endif

#endif
