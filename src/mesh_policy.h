// BL-01: Bedeutung von "--mesh off" (Betreiberentscheid 2026-10-24).
//
//   MESH_TX_FOREIGN_RELAY  Repeat fremder Frames (POS, Text, HEY via rx_relay;
//                          ACK via rx_ack_fwd)               -> nur bei mesh on
//   MESH_TX_OWN            eigene Frames (HEY, ACK, Text, ping/pong) -> immer
//   MESH_TX_GW_SERVICE     Gateway-ACK, HEY-Upload, Server-Downlink (udp_rx) -> immer
//   MESH_TX_STORE_SERVICE  Store-Knoten mbox/sto               -> immer
//
// Nur die beiden Relay-Schranken (checkMesh(), rx_ack_fwd) rufen diese Funktion.
// Die immer gesendeten Stellen rufen sie bewusst nicht.
#ifndef MESH_POLICY_H
#define MESH_POLICY_H

enum MeshTxClass
{
    MESH_TX_FOREIGN_RELAY = 0,
    MESH_TX_OWN,
    MESH_TX_GW_SERVICE,
    MESH_TX_STORE_SERVICE
};

static inline bool meshTxAllowed(MeshTxClass c, bool mesh_on)
{
    return (c == MESH_TX_FOREIGN_RELAY) ? mesh_on : true;
}

#endif
