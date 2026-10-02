// MeshCom-5-Topologie, Abfrageschicht (Welle 3) -- siehe src/nbr_views.h fuer
// die Schnittstelle und docs/meshcom5-topologie/ 4.6-4.9 und 4.12 fuer das
// Konzept. Arduino-frei wie nbr_matrix.cpp.
//
// Aufbau: jede oeffentliche Funktion nimmt die Scheduler-Klammer von
// nbr_matrix.cpp (nbrPrivLock()/nbrPrivUnlock(), verschachtelbar), liest die
// Felder direkt und rechnet Gleitkomma erst nach der Klammer. #X und Rolle
// kommen aus nbrPrivXCounts() (dieselbe Anteilsregel wie
// nbrRowMeshNeedCount()), das Veto aus nbrPrivRptValid() (dieselbe Regel wie
// die Symmetrie-Annahme). Diese Datei loggt nichts und haelt keinen
// statischen Zustand; die einzigen statischen Daten sind die konstanten
// Namenstabellen (.rodata).

#include <stddef.h>

#include "nbr_views.h"

// Minute, in der ein Direktempfang fuer NCNT zaehlt (D60, Konzept 4.8).
#define NBR_NCNT_D_MIN 60

// --- kleine Helfer (Aufrufer haelt die Klammer) -----------------------------------

static inline bool vReady(const NbrMatrix &m)
{
    return m.call[0] != 0;
}

static inline bool vUsed(const NbrMatrix &m, int i)
{
    return i == 0 || (m.row[i].flags & NBR_FLAG_USED) != 0;
}

static inline bool vFresh(uint16_t last, uint16_t now)
{
    return (uint16_t)(now - last) < NBR_WINDOW_MIN;
}

// Lebende Kante (x, y) oder -1; die Maske ist der schnelle Negativtest.
static int vEdge(const NbrMatrix &m, int x, int y)
{
    if (x < 0 || y < 0 || x >= NBR_MAX_ROWS || y >= NBR_MAX_ROWS || x == y)
        return -1;
    if (!nbrMaskTest(m.heardBy[x], y))
        return -1;
    for (int e = 0; e < NBR_MAX_EDGES; e++)
        if (m.edge[e].x == x && m.edge[e].y == y)
            return e;
    return -1;
}

static int vSlot(const NbrMatrix &m, int row)
{
    uint8_t s = m.row[row].ext;
    return (s < NBR_EXT_SLOTS) ? (int)s : -1;
}

// Kante (x, 0) einer MH-Zeile: lebend, frisch, juenger als window.
static inline bool vMhEdge(const NbrMatrix &m, const NbrEdge &ed, uint16_t now, uint16_t window)
{
    return ed.x != 0xFF && ed.y == 0 && ed.x != 0 && vUsed(m, ed.x) && vFresh(ed.last_min, now) &&
           (uint16_t)(now - ed.last_min) < window;
}

// Sortierschluessel neueste zuerst: Alter (Minuten), dann Sekunde absteigend
// (unbekannt zuletzt).
static uint16_t vMhKey(const NbrMatrix &m, const NbrEdge &ed, uint16_t now)
{
    int s = vSlot(m, ed.x);
    uint32_t sec = (s >= 0) ? nbrBitsGet(m.ext[s], NBR_XO_SEC, NBR_XW_SEC) : NBR_EXT_SEC_UNKNOWN;
    uint16_t sfield = (sec < 60) ? (uint16_t)(59 - sec) : 63;
    return (uint16_t)(((uint16_t)(now - ed.last_min) << 6) | sfield);
}

// --- MHeard (Konzept 4.6) ------------------------------------------------------------

// Auswahl statt Sortierpuffer: je Ausgabeplatz ein Durchlauf ueber den Pool
// (kein zeilengrosses Feld auf dem Loop-Stack). Die Kante (x, 0) ist je x
// eindeutig, darum gibt es keine Doppelten.
int nbrMhRows(const NbrMatrix &m, uint16_t now_min, uint16_t window_min, uint8_t *out, int max)
{
    int total = 0;
    nbrPrivLock();
    if (vReady(m))
    {
        for (int e = 0; e < NBR_MAX_EDGES; e++)
            if (vMhEdge(m, m.edge[e], now_min, window_min))
                total++;
        uint32_t last = 0;
        bool have_last = false;
        int n = (out && max > 0) ? (total < max ? total : max) : 0;
        for (int k = 0; k < n; k++)
        {
            uint32_t best = 0xFFFFFFFFu;
            for (int e = 0; e < NBR_MAX_EDGES; e++)
            {
                const NbrEdge &ed = m.edge[e];
                if (!vMhEdge(m, ed, now_min, window_min))
                    continue;
                uint32_t key = ((uint32_t)vMhKey(m, ed, now_min) << 8) | ed.x;
                if (have_last && key <= last)
                    continue;
                if (key < best)
                    best = key;
            }
            if (best == 0xFFFFFFFFu)
                break;
            out[k] = (uint8_t)(best & 0xFF);
            last = best;
            have_last = true;
        }
    }
    nbrPrivUnlock();
    return total;
}

int nbrMhCount(const NbrMatrix &m, uint16_t now_min, uint16_t window_min)
{
    int n = 0;
    nbrPrivLock();
    if (vReady(m))
        for (int e = 0; e < NBR_MAX_EDGES; e++)
            if (vMhEdge(m, m.edge[e], now_min, window_min))
                n++;
    nbrPrivUnlock();
    return n;
}

// Rolle aus #X (Konzept 4.3): S Super-Node = groesstes #X, mindestens 2 und
// mindestens doppelt so gross wie das zweitgroesste; N noetig (#X >= 1);
// R redundant (#X == 0); 0 fuer "NA".
static char vRole(const uint8_t *xcnt, int row)
{
    if (xcnt[row] == 0xFF)
        return 0;
    int best = -1, second = 0;
    for (int i = 1; i < NBR_MAX_ROWS; i++)
    {
        if (xcnt[i] == 0xFF)
            continue;
        if (best < 0 || xcnt[i] > xcnt[best])
        {
            if (best >= 0 && xcnt[best] > second)
                second = xcnt[best];
            best = i;
        }
        else if (xcnt[i] > second)
            second = xcnt[i];
    }
    int x = xcnt[row];
    if (row == best && x >= 2 && x >= 2 * second)
        return 'S';
    return x >= 1 ? 'N' : 'R';
}

bool nbrMhGet(const NbrMatrix &m, int row, uint16_t now_min, NbrMhView *out)
{
    if (!out || row <= 0 || row >= NBR_MAX_ROWS)
        return false;
    uint64_t call = 0;
    uint8_t slot[NBR_EXT_BYTES];
    bool has_slot = false;
    uint8_t xcnt[NBR_MAX_ROWS];
    NbrRow r;
    uint16_t age = 0;
    int8_t snr = NBR_SNR_UNKNOWN, hm = NBR_SNR_UNKNOWN;
    uint8_t nb = 0;
    bool ok = false;

    nbrPrivLock();
    if (vReady(m) && vUsed(m, row))
    {
        int e = vEdge(m, row, 0);
        if (e >= 0 && vFresh(m.edge[e].last_min, now_min))
        {
            ok = true;
            call = m.call[row];
            r = m.row[row];
            age = (uint16_t)(now_min - m.edge[e].last_min);
            snr = m.edge[e].snr;
            int s = vSlot(m, row);
            if (s >= 0)
            {
                memcpy(slot, m.ext[s], NBR_EXT_BYTES);
                has_slot = true;
            }
            int eh = vEdge(m, 0, row);
            if (eh >= 0 && vFresh(m.edge[eh].last_min, now_min))
                hm = m.edge[eh].snr;
            for (int k = 0; k < NBR_MAX_EDGES; k++)
            {
                const NbrEdge &ed = m.edge[k];
                if (ed.x != 0xFF && ed.y == row && vFresh(ed.last_min, now_min) && nb < 255)
                    nb++;
            }
            nbrPrivXCounts(m, now_min, xcnt, NULL);
        }
    }
    nbrPrivUnlock();
    if (!ok)
        return false;

    memset(out, 0, sizeof(*out));
    nbrCallDecode(call, out->call);
    out->age_min = age;
    out->hw = r.hw;
    out->snr = snr;
    out->ncnt = r.ncnt;
    out->hm_snr = hm;
    out->ex = (xcnt[row] == 0xFF) ? 0 : xcnt[row];
    out->role = vRole(xcnt, row);
    out->nb = nb;
    out->gw = (r.flags & NBR_FLAG_GW) ? 1 : 0;
    out->via = 0;
    if (!has_slot)
    {
        out->sec = 0xFF;
        out->plt = 0;
        out->mod = 0;
        out->rssi = NBR_MH_RSSI_UNKNOWN;
        out->lat = out->lon = NBR_POS_NONE;
        out->alt = NBR_MH_ALT_UNKNOWN;
        out->pl = 0;
        out->mesh = 0;
        out->fw = 0;
        return true;
    }
    uint32_t sec = nbrBitsGet(slot, NBR_XO_SEC, NBR_XW_SEC);
    out->sec = (sec < 60) ? (uint8_t)sec : 0xFF;
    static const char plt_char[4] = {':', '!', '@', 0};
    out->plt = plt_char[nbrBitsGet(slot, NBR_XO_PLT, NBR_XW_PLT)];
    out->mod = (uint8_t)nbrBitsGet(slot, NBR_XO_MOD, NBR_XW_MOD);
    out->rssi = (int16_t)((int)nbrBitsGet(slot, NBR_XO_RSSI, NBR_XW_RSSI) - 160);
    uint32_t lat = nbrBitsGet(slot, NBR_XO_LAT, NBR_XW_LAT);
    uint32_t lon = nbrBitsGet(slot, NBR_XO_LON, NBR_XW_LON);
    bool pos = lat != NBR_EXT_LAT_UNKNOWN && lon != NBR_EXT_LON_UNKNOWN;
    out->lat = pos ? (float)((double)lat / 10000.0 - 90.0) : NBR_POS_NONE;
    out->lon = pos ? (float)((double)lon / 10000.0 - 180.0) : NBR_POS_NONE;
    uint32_t alt = nbrBitsGet(slot, NBR_XO_ALT, NBR_XW_ALT);
    out->alt = (alt == NBR_EXT_ALT_UNKNOWN) ? (int16_t)NBR_MH_ALT_UNKNOWN : (int16_t)((int32_t)alt - 1000);
    out->pl = (uint8_t)nbrBitsGet(slot, NBR_XO_PL, NBR_XW_PL);
    out->mesh = (uint8_t)nbrBitsGet(slot, NBR_XO_MESH, NBR_XW_MESH);
    uint32_t fw = nbrBitsGet(slot, NBR_XO_FW, NBR_XW_FW);
    out->fw = (fw >= 1 && fw <= 26) ? (char)('a' + fw - 1) : 0;
    return true;
}

// --- NCNT (Konzept 4.8) -----------------------------------------------------------------
//
// VETO: vollstaendiger Bericht von x juenger als NBR_REPORT_VALID_MIN
// (nbrPrivRptValid(), die Regel der Symmetrie-Annahme), der mich nicht nennt.
// Ein Bericht, der mich nennt, trifft beim Lesen die Kante (0, x) in seiner
// Minute (nbrNoteReport(), Status "self"); "nennt mich" heisst darum: die
// Kante (0, x) lebt und wurde zur Berichtsminute oder danach getroffen. Ein
// spaeterer Beweis (x wiederholt meinen Rahmen) hebt das Veto ebenso auf.
// SYM: Zustand aus dem Direkt-Slot; ohne Slot (verdraengt) ohne Hysterese
// direkt aus dem SNR-Mittel.
int nbrNcnt(const NbrMatrix &m, uint16_t now_min)
{
    NbrMask d60 = nbrMaskNone(), hm = nbrMaskNone(), sym = nbrMaskNone(), named = nbrMaskNone();
    NbrMask veto = nbrMaskNone();
    nbrPrivLock();
    if (vReady(m))
    {
        for (int e = 0; e < NBR_MAX_EDGES; e++)
        {
            const NbrEdge &ed = m.edge[e];
            if (ed.x == 0xFF || !vFresh(ed.last_min, now_min))
                continue;
            if (ed.y == 0 && ed.x != 0 && vUsed(m, ed.x) && (uint16_t)(now_min - ed.last_min) < NBR_NCNT_D_MIN)
            {
                nbrMaskSet(d60, ed.x);
                int s = vSlot(m, ed.x);
                bool on = (s >= 0) ? nbrBitsGet(m.ext[s], NBR_XO_SYM, NBR_XW_SYM) != 0
                                   : (ed.snr != NBR_SNR_UNKNOWN && ed.snr >= NBR_SYM_MIN_SNR);
                if (on)
                    nbrMaskSet(sym, ed.x);
            }
            else if (ed.x == 0 && ed.y != 0 && vUsed(m, ed.y))
            {
                nbrMaskSet(hm, ed.y);
                uint16_t since = m.row[ed.y].rpt_min;
                if ((uint16_t)(ed.last_min - since) <= (uint16_t)(now_min - since))
                    nbrMaskSet(named, ed.y);
            }
        }
        for (int x = nbrMaskNext(d60, -1); x >= 0; x = nbrMaskNext(d60, x))
            if (nbrPrivRptValid(m.row[x], now_min) && !nbrMaskTest(named, x))
                nbrMaskSet(veto, x);
    }
    nbrPrivUnlock();
    return nbrMaskCount(nbrMaskAndNot(nbrMaskAnd(d60, nbrMaskOr(hm, sym)), veto));
}

int nbrNcntAir(const NbrMatrix &m, uint16_t now_min)
{
    int n = nbrNcnt(m, now_min);
    return n > NBR_NCNT_AIR_MAX ? NBR_NCNT_AIR_MAX : n;
}

// --- Pfade und Horizont (Konzept 4.7) -----------------------------------------------------

// Direkt-Maske (frische Kante (x, 0)) und 2-Hop-Zeilen: belegte Zeilen X, nicht
// direkt, nicht ich, mit frischer Kante (X, B) fuer ein direktes B.
static void vRouteMasks(const NbrMatrix &m, uint16_t now, NbrMask *direct, NbrMask *twohop)
{
    *direct = nbrMaskNone();
    *twohop = nbrMaskNone();
    for (int e = 0; e < NBR_MAX_EDGES; e++)
    {
        const NbrEdge &ed = m.edge[e];
        if (ed.x != 0xFF && ed.y == 0 && ed.x != 0 && vUsed(m, ed.x) && vFresh(ed.last_min, now))
            nbrMaskSet(*direct, ed.x);
    }
    for (int e = 0; e < NBR_MAX_EDGES; e++)
    {
        const NbrEdge &ed = m.edge[e];
        if (ed.x == 0xFF || ed.x == 0 || ed.y == 0 || !vUsed(m, ed.x) || !vFresh(ed.last_min, now))
            continue;
        if (nbrMaskTest(*direct, ed.y) && !nbrMaskTest(*direct, ed.x))
            nbrMaskSet(*twohop, ed.x);
    }
}

// Lebender Horizont-Eintrag: belegt, frisch, und sein Absender steht nicht
// schon als direkte oder 2-Hop-Zeile in der Sicht (shown = direct | twohop;
// letzte Sicherung gegen einen doppelten Eintrag, Advisor Welle 3 R1 --
// nbrNoteFrame() prueft dieselbe Regel beim Anlegen, aber eine Zeile kann
// danach ueber Berichte oder Regel-3-Paare wieder sichtbar werden).
static bool vHzLive(const NbrMatrix &m, int h, uint16_t now, const NbrMask &shown)
{
    if (!m.hz_call[h] || !vFresh(nbrPrivHzLast(m.hz_meta[h]), now) || nbrMaskEmpty(m.hz_entry[h]))
        return false;
    if (m.hz_call[h] == m.call[0])
        return false;
    for (int i = 1; i < NBR_MAX_ROWS; i++)
        if (m.call[i] == m.hz_call[h] && vUsed(m, i) && nbrMaskTest(shown, i))
            return false;
    return true;
}

int nbrRouteCount(const NbrMatrix &m, uint16_t now_min)
{
    int n = 0;
    nbrPrivLock();
    if (vReady(m))
    {
        NbrMask direct, twohop;
        vRouteMasks(m, now_min, &direct, &twohop);
        n = nbrMaskCount(twohop);
        NbrMask shown = nbrMaskOr(direct, twohop);
        for (int h = 0; h < NBR_HZ_ENTRIES; h++)
            if (vHzLive(m, h, now_min, shown))
                n++;
    }
    nbrPrivUnlock();
    return n;
}

// hops: bei einer 2-Hop-Zeile 2, bei einem Horizont-Eintrag die Hops bis mich
// = gespeicherte Hops Absender->Eintrittszeile A (Minimum zweier 6-h-Epochen)
// + 2 (A -> B -> ich), damit beide Arten dieselbe Skala haben.
bool nbrRouteGet(const NbrMatrix &m, int idx, uint16_t now_min, NbrRouteView *out)
{
    if (!out || idx < 0)
        return false;
    bool ok = false;
    uint64_t call = 0;
    memset(out, 0, sizeof(*out));
    nbrPrivLock();
    if (vReady(m))
    {
        NbrMask direct, twohop;
        vRouteMasks(m, now_min, &direct, &twohop);
        int nrow = nbrMaskCount(twohop);
        if (idx < nrow)
        {
            int x = nbrMaskNext(twohop, -1);
            for (int k = 0; k < idx; k++)
                x = nbrMaskNext(twohop, x);
            // Weg ueber jedes direkte B mit frischer Kante (X, B); das Alter eines
            // Wegs ist das seiner aeltesten Kante, gezeigt wird der juengste Weg.
            NbrMask entry = nbrMaskNone();
            uint16_t best = 0xFFFF;
            for (int e = 0; e < NBR_MAX_EDGES; e++)
            {
                const NbrEdge &ed = m.edge[e];
                if (ed.x != x || !nbrMaskTest(direct, ed.y) || !vFresh(ed.last_min, now_min))
                    continue;
                nbrMaskSet(entry, ed.y);
                int eb = vEdge(m, ed.y, 0);
                uint16_t a1 = (uint16_t)(now_min - ed.last_min);
                uint16_t a2 = (eb >= 0) ? (uint16_t)(now_min - m.edge[eb].last_min) : 0xFFFF;
                uint16_t a = a1 > a2 ? a1 : a2;
                if (a < best)
                    best = a;
            }
            call = m.call[x];
            out->hops = 2;
            out->gw = (m.row[x].flags & NBR_FLAG_GW) ? 1 : 0;
            out->age_min = best;
            out->is_row = 1;
            out->entry = entry;
            out->row = (uint8_t)x;
            ok = true;
        }
        else
        {
            int want = idx - nrow;
            NbrMask shown = nbrMaskOr(direct, twohop);
            for (int h = 0; h < NBR_HZ_ENTRIES && !ok; h++)
            {
                if (!vHzLive(m, h, now_min, shown))
                    continue;
                if (want-- > 0)
                    continue;
                call = m.hz_call[h];
                uint8_t hops = nbrPrivHzHops(m.hz_meta[h], now_min);
                out->hops = (uint8_t)(hops + 2);
                out->gw = m.hz_meta[h][1] & 1;
                out->age_min = (uint16_t)(now_min - nbrPrivHzLast(m.hz_meta[h]));
                out->is_row = 0;
                out->entry = m.hz_entry[h];
                out->row = 0xFF;
                ok = true;
            }
        }
    }
    nbrPrivUnlock();
    if (ok)
        nbrCallDecode(call, out->call);
    return ok;
}

// --- Namen (aus mheard_functions.cpp, Arduino-frei) ------------------------------------------

const char *nbrPayloadTypeName(char plt)
{
    if (plt == ':')
        return "TXT";
    if (plt == '!')
        return "POS";
    if (plt == '@')
        return "HEY";
    return "???";
}

// Tabelle und Alt-ID-Uebersetzung unveraendert aus getHardwareLong()
// (src/mheard_functions.cpp), T-Deck mit den kurzen Namen.
#define NBR_MAX_HARDWARE 36
#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
static const char *const nbrHwNames[NBR_MAX_HARDWARE] = {"no info", "TLO_V2", "TLO_V1", "TLV2_1p6", "TBEAM", "TB_1268", "TB_0p7", "TECHO", "TDECK", "RAK4631", "HELTV21", "HELTV1", "TB_2101", "EB_E22", "HELTV3", "HELT_E290", "TB_1262", "TDECK+", "TB_SUPR", "ES3_E22", "TRACKER_V3", "STICK_V3", "T5_EPAPER", "TPAGER", "TDECKpro", "TBEAM_1W", "HETLV4", "T_ETH_EL", "HETL_T114", "T3S3V13", "TCONPRO", "WLPAPER", "HELT_E213", "ESP32_LORAPRS_E22", "ESP32_LORAPRS_RA01", "T_WATCH_S3"};
#else
static const char *const nbrHwNames[NBR_MAX_HARDWARE] = {"no info", "TLORA_V2", "TLORA_V1", "TLORA_V2_1_1p6", "TBEAM", "TBEAM_1268", "TBEAM_0p7", "T_ECHO", "TDECK", "RAK4631", "HELTEC_V2_1", "HELTEC_V1", "TBEAM_AXP2101", "EBYTE_E22", "HELTEC_V3", "HELTEC_E290", "TBEAM_1262", "TDECK_PLUS", "TBEAM_SUPREME", "ESP_S3_E22", "TRACK_V3", "STICK_V3", "T5_EPAPER", "TPAGER", "TDECKpro", "TBEAM_1W", "HELTEC_V4", "T_ETH_ELITE", "HELTEC_T114", "T3_S3_V13", "T_CON_PRO", "WIRELESS_PAPER", "HELTEC_E213", "ESP32_LORAPRS_E22", "ESP32_LORAPRS_RA01", "T_WATCH_S3"};
#endif

// Alte IDs 39..61 -> Tabellenindex (getHardwareLong(): 39->13, 40->22, 41->20,
// 42->21, 43->14 ... 57->31, 58->32, 59->33, 60->34, 61->35).
static const uint8_t nbrHwLegacy[61 - 39 + 1] = {13, 22, 20, 21, 14, 15, 16, 17, 18, 19, 23, 24,
                                                   25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35};

const char *nbrHardwareName(uint8_t hw)
{
    int ihw = hw;
    if (ihw >= 39 && ihw <= 61)
        ihw = nbrHwLegacy[ihw - 39];
    if (ihw < 0 || ihw >= NBR_MAX_HARDWARE)
        ihw = 0;
    return nbrHwNames[ihw];
}

// --- Sicherung (Konzept 4.12) -------------------------------------------------------------
//
// Abbild = 24 Byte Kopf + die Felder der NbrMatrix roh, in fester
// Reihenfolge (vSaveParts()). Der Kopf nennt alle Groessen; ein Abbild eines
// anderen Builds (andere Zeilen-/Kanten-/Slot-/Horizontzahl, Maskenbreite
// oder Formatversion) wird verworfen. Zahlen im Kopf in der Bytefolge des
// Geraets (die Datei bleibt auf dem Geraet, das sie schrieb).

#define NBR_SAVE_MAGIC   "NBT5"
#define NBR_SAVE_VERSION 1
#define NBR_SAVE_HDR     24

struct NbrSavePart
{
    size_t off;
    size_t len;
};

#define NBR_SAVE_NPARTS 15
static void vSaveParts(NbrSavePart *p)
{
    int i = 0;
#define NBR_PART(field) (p[i].off = offsetof(NbrMatrix, field), p[i].len = sizeof(NbrMatrix::field), i++)
    NBR_PART(call);
    NBR_PART(row);
    NBR_PART(hears);
    NBR_PART(heardBy);
    NBR_PART(edge);
    NBR_PART(ext);
    NBR_PART(hz_call);
    NBR_PART(hz_entry);
    NBR_PART(hz_meta);
    NBR_PART(echo_id);
    NBR_PART(echo_first);
    NBR_PART(echo_second);
    NBR_PART(echo_min);
    NBR_PART(echo_type);
    NBR_PART(last_halve);
#undef NBR_PART
}

static size_t vBodySize(void)
{
    NbrSavePart p[NBR_SAVE_NPARTS];
    vSaveParts(p);
    size_t n = 0;
    for (int i = 0; i < NBR_SAVE_NPARTS; i++)
        n += p[i].len;
    return n;
}

static void vPut16(uint8_t *b, uint16_t v)
{
    memcpy(b, &v, 2);
}

static void vPut32(uint8_t *b, uint32_t v)
{
    memcpy(b, &v, 4);
}

static uint16_t vGet16(const uint8_t *b)
{
    uint16_t v;
    memcpy(&v, b, 2);
    return v;
}

static uint32_t vGet32(const uint8_t *b)
{
    uint32_t v;
    memcpy(&v, b, 4);
    return v;
}

// Kopf: Magic 4 | Version 2 | Zeilen 2 | Kanten 2 | Slots 2 | Horizont 2 |
// Maskenwoerter 2 | Epoche beim Sichern 4 | Minute beim Sichern 2 | 0 2.
static void vHeader(uint8_t *b, uint32_t epoch, uint16_t min)
{
    memcpy(b, NBR_SAVE_MAGIC, 4);
    vPut16(b + 4, NBR_SAVE_VERSION);
    vPut16(b + 6, NBR_MAX_ROWS);
    vPut16(b + 8, NBR_MAX_EDGES);
    vPut16(b + 10, NBR_EXT_SLOTS);
    vPut16(b + 12, NBR_HZ_ENTRIES);
    vPut16(b + 14, NBR_MASK_WORDS);
    vPut32(b + 16, epoch);
    vPut16(b + 20, min);
    vPut16(b + 22, 0);
}

static bool vHeaderOk(const uint8_t *buf, size_t len)
{
    if (!buf || len < NBR_SAVE_HDR + vBodySize())
        return false;
    uint8_t want[NBR_SAVE_HDR];
    vHeader(want, 0, 0);
    return memcmp(buf, want, 16) == 0 && vGet16(buf + 22) == 0;
}

size_t nbrSaveSize(void)
{
    return NBR_SAVE_HDR + vBodySize();
}

uint32_t nbrSavedEpoch(const uint8_t *buf, size_t len)
{
    return vHeaderOk(buf, len) ? vGet32(buf + 16) : 0;
}

size_t nbrSave(const NbrMatrix &m, uint32_t now_epoch, uint16_t now_min, uint8_t *buf, size_t len)
{
    size_t need = nbrSaveSize();
    if (!buf || len < need)
        return 0;
    vHeader(buf, now_epoch, now_min);
    NbrSavePart p[NBR_SAVE_NPARTS];
    vSaveParts(p);
    size_t pos = NBR_SAVE_HDR;
    const uint8_t *src = (const uint8_t *)&m;
    nbrPrivLock();
    for (int i = 0; i < NBR_SAVE_NPARTS; i++)
    {
        memcpy(buf + pos, src + p[i].off, p[i].len);
        pos += p[i].len;
    }
    nbrPrivUnlock();
    return need;
}

// Minute t (aus dem Abbild) auf die neue Bootzeit umrechnen: Alter beim
// Sichern plus vergangene Minuten; false, wenn das NBR_WINDOW_MIN erreicht.
static bool vRebase(uint16_t *t, uint16_t saved_min, uint32_t elapsed, uint16_t now_min)
{
    uint32_t age = (uint32_t)(uint16_t)(saved_min - *t) + elapsed;
    if (age >= NBR_WINDOW_MIN)
        return false;
    *t = (uint16_t)(now_min - age);
    return true;
}

static void vBlankRow(NbrMatrix &m, int i)
{
    memset(&m.row[i], 0, sizeof(NbrRow));
    m.row[i].lat16 = 0x7FFF;
    m.row[i].lon16 = 0x7FFF;
    m.row[i].ext = 0xFF;
    m.call[i] = 0;
    m.gw_min[i] = 0;
}

bool nbrLoad(NbrMatrix &m, const uint8_t *buf, size_t len, uint32_t now_epoch, uint16_t now_min)
{
    if (!vHeaderOk(buf, len))
        return false;
    uint32_t saved_epoch = vGet32(buf + 16);
    uint16_t saved_min = vGet16(buf + 20);
    if (saved_epoch == 0 || now_epoch == 0)
        return false;
    uint64_t img_own;
    memcpy(&img_own, buf + NBR_SAVE_HDR, sizeof(img_own)); // call[0] ist das erste Feld
    if (img_own == 0)
        return false;
    uint32_t elapsed = (now_epoch > saved_epoch) ? (now_epoch - saved_epoch) / 60u : 0;
    if (elapsed > 0xFFFFu)
        elapsed = 0xFFFFu;

    NbrSavePart p[NBR_SAVE_NPARTS];
    vSaveParts(p);
    uint8_t *dst = (uint8_t *)&m;
    bool ok = false;
    nbrPrivLock();
    if (!vReady(m) || m.call[0] == img_own)
    {
        ok = true;
        size_t pos = NBR_SAVE_HDR;
        for (int i = 0; i < NBR_SAVE_NPARTS; i++)
        {
            memcpy(dst + p[i].off, buf + pos, p[i].len);
            pos += p[i].len;
        }
        // Zeilen: zu alte fallen weg (Zeile 0 nie); Berichtsminute ebenso.
        m.row[0].flags |= NBR_FLAG_USED;
        for (int i = 0; i < NBR_MAX_ROWS; i++)
        {
            NbrRow &r = m.row[i];
            if (i > 0 && (!(r.flags & NBR_FLAG_USED) || m.call[i] == 0 ||
                          !vRebase(&r.last_min, saved_min, elapsed, now_min)))
            {
                vBlankRow(m, i);
                continue;
            }
            if (i == 0 && !vRebase(&r.last_min, saved_min, elapsed, now_min))
                r.last_min = now_min;
            // Gateway-Flag fremder Zeilen wird nicht wiederhergestellt: der
            // Verfallszeitgeber (gw_min) ist nicht im Abbild, ein gesetztes Flag
            // wuerde zufaellig gehalten oder verfallen. Ein Gateway wird mit seinem
            // naechsten HG neu gelernt. Zeile 0 behaelt ihr Flag (bGATEWAY).
            if (i > 0)
            {
                r.flags &= (uint8_t)~NBR_FLAG_GW;
                m.gw_min[i] = 0;
            }
            if (!vRebase(&r.rpt_min, saved_min, elapsed, now_min))
            {
                r.rpt_min = 0;
                r.flags &= (uint8_t)~NBR_FLAG_RPT;
            }
            if (r.ext != 0xFF && r.ext >= NBR_EXT_SLOTS)
                r.ext = 0xFF;
        }
        // Kanten: ungueltig, zu alt oder doppelt -> frei; Masken neu aus dem Pool.
        for (int i = 0; i < NBR_MAX_ROWS; i++)
            m.hears[i] = m.heardBy[i] = nbrMaskNone();
        for (int e = 0; e < NBR_MAX_EDGES; e++)
        {
            NbrEdge &ed = m.edge[e];
            bool keep = ed.x < NBR_MAX_ROWS && ed.y < NBR_MAX_ROWS && ed.x != ed.y && vUsed(m, ed.x) &&
                        vUsed(m, ed.y) && !nbrMaskTest(m.heardBy[ed.x], ed.y) &&
                        vRebase(&ed.last_min, saved_min, elapsed, now_min);
            if (!keep)
            {
                ed.x = ed.y = 0xFF;
                ed.cnt = 0;
                ed.snr = NBR_SNR_UNKNOWN;
                ed.last_min = 0;
                continue;
            }
            nbrMaskSet(m.hears[ed.y], ed.x);
            nbrMaskSet(m.heardBy[ed.x], ed.y);
        }
        // Slots: nur mit lebender Kante (x, 0), jeder Slot hoechstens einmal.
        uint8_t seen[(NBR_EXT_SLOTS + 7) / 8];
        memset(seen, 0, sizeof(seen));
        for (int i = 1; i < NBR_MAX_ROWS; i++)
        {
            uint8_t s = m.row[i].ext;
            if (s == 0xFF)
                continue;
            if (!nbrMaskTest(m.heardBy[i], 0) || (seen[s >> 3] >> (s & 7)) & 1u)
                m.row[i].ext = 0xFF;
            else
                seen[s >> 3] |= (uint8_t)(1u << (s & 7));
        }
        // Belegte Zeilen fuer die Eintritts- und Echo-Masken.
        NbrMask used = nbrMaskNone();
        for (int i = 1; i < NBR_MAX_ROWS; i++)
            if (vUsed(m, i))
                nbrMaskSet(used, i);
        for (int h = 0; h < NBR_HZ_ENTRIES; h++)
        {
            uint16_t last = nbrPrivHzLast(m.hz_meta[h]);
            m.hz_entry[h] = nbrMaskAnd(m.hz_entry[h], used);
            if (!m.hz_call[h] || nbrMaskEmpty(m.hz_entry[h]) || !vRebase(&last, saved_min, elapsed, now_min))
            {
                m.hz_call[h] = 0;
                continue;
            }
            // Die 6-h-Epochen haengen an der Minute und verschieben sich mit
            // der neuen Bootminute: das Minimum der Epoche des letzten Rahmens
            // bleibt, das der Epoche davor verfaellt.
            uint8_t cur = m.hz_meta[h][0] & 0x0F;
            m.hz_meta[h][0] = cur ? cur : 1;
            m.hz_meta[h][2] = (uint8_t)(last & 0xFF);
            m.hz_meta[h][3] = (uint8_t)(last >> 8);
        }
        for (int k = 0; k < 4; k++)
        {
            m.echo_first[k] = nbrMaskAnd(m.echo_first[k], used);
            m.echo_second[k] = nbrMaskAnd(m.echo_second[k], used);
            if (!m.echo_type[k] || !vRebase(&m.echo_min[k], saved_min, elapsed, now_min))
            {
                // frei wie nach nbrIEchoFold(): alle Felder leer
                m.echo_type[k] = 0;
                m.echo_id[k] = 0;
                m.echo_min[k] = 0;
                m.echo_first[k] = m.echo_second[k] = nbrMaskNone();
            }
        }
        if (!vRebase(&m.last_halve, saved_min, elapsed, now_min))
            m.last_halve = now_min;
        m.call[0] = img_own;
        m.boot_min = now_min;
        m.last_sweep = now_min;
        uint32_t off = (uint32_t)now_min * 60u;
        m.boot_epoch = (now_epoch > off) ? now_epoch - off : 0;
        nbrPrivRowsChanged();
    }
    nbrPrivUnlock();
    return ok;
}
