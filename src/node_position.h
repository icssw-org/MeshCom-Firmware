// node_position.h -- pure helpers for the node's own position (DRY-01). Arduino-free, no access to
// meshcom_settings, so the host test env native_node_position can pin them.
//
// The node stores a position as an unsigned magnitude plus a hemisphere letter (node_lat/node_lat_c
// 'N'|'S', node_lon/node_lon_c 'E'|'W'). nodeSetPosition() in command_functions.cpp applies the split
// for the serial, BLE and RM setters; every signed read goes through nodeSignedLat/Lon.
#ifndef NODE_POSITION_H_
#define NODE_POSITION_H_

#include <math.h>

// Latitude in -90..90 and not NaN (a NaN fails every comparison, so it is refused).
static inline bool nodeLatValid(double lat)
{
    return lat >= -90.0 && lat <= 90.0;
}

// Longitude in -180..180 and not NaN.
static inline bool nodeLonValid(double lon)
{
    return lon >= -180.0 && lon <= 180.0;
}

// Signed latitude -> magnitude + 'N'/'S'. Negative -> 'S'; 0, -0 and positive -> 'N' (magnitude +0).
// Returns false and leaves the outputs untouched when the value is out of range.
static inline bool nodeLatSplit(double lat, double &mag, char &hem)
{
    if (!nodeLatValid(lat))
        return false;
    hem = lat < 0 ? 'S' : 'N';
    mag = lat < 0 ? -lat : lat + 0.0; // + 0.0 turns -0.0 into +0.0
    return true;
}

// Signed longitude -> magnitude + 'E'/'W', same rules.
static inline bool nodeLonSplit(double lon, double &mag, char &hem)
{
    if (!nodeLonValid(lon))
        return false;
    hem = lon < 0 ? 'W' : 'E';
    mag = lon < 0 ? -lon : lon + 0.0;
    return true;
}

// Both axes at once; nothing is written unless both are valid.
static inline bool nodePosSplit(double lat, double lon, double &absLat, char &latHem, double &absLon, char &lonHem)
{
    if (!nodeLatValid(lat) || !nodeLonValid(lon))
        return false;
    return nodeLatSplit(lat, absLat, latHem) && nodeLonSplit(lon, absLon, lonHem);
}

// Stored magnitude + hemisphere letter back to a signed value.
static inline double nodeSignedLat(double mag, char hem)
{
    return hem == 'S' ? -mag : mag;
}

static inline double nodeSignedLon(double mag, char hem)
{
    return hem == 'W' ? -mag : mag;
}

#endif // NODE_POSITION_H_
