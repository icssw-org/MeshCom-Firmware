// telemetry_app.h -- which telemetry source the periodic T# timer serves. Header-only, Arduino-free.
//
// sendTelemetry(ID) (loop_functions.cpp) knows two sources:
//   ID 1 (SOFTSER_APP_ID)  the soft-serial app ("Pegelmesser"): node_parm_1 / strSOFTSERAPP_ID
//   ID 0                   the node's own telemetry: --parm / --unit / --values (node_parm ...)
// Since upstream v4.34x.05.16 the periodic timer passed SOFTSER_APP_ID unconditionally, so a node without the
// soft-serial app never sent its own T# frames. The timer now asks this helper.

#pragma once

static inline int telemetryAppId(bool softserOn, int softserAppId)
{
    return softserOn ? softserAppId : 0;
}
