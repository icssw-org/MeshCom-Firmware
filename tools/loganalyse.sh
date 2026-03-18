#!/usr/bin/env bash
# MeshCom Log-Analyse Tool
# Extracts all analysis data from a MeshCom serial monitor log in a single pass.
# Expects the log format produced by serial_monitor.py:
#   2026-03-06 15:36:58.248  [firmware output here]
#
# Usage: ./tools/loganalyse.sh <logfile> [<logfile2>]
#        With two logfiles: adds CROSS_CORRELATION section
#
# Output: Structured sections separated by "=== SECTION_NAME ===" markers
# for easy parsing by the logauswertung skill.

set -euo pipefail

if [ $# -lt 1 ] || [ ! -f "$1" ]; then
    echo "Usage: $0 <logfile>" >&2
    exit 1
fi

LOGFILE="$1"

# ─── Helper ───
section() { echo ""; echo "=== $1 ==="; }

# ─── 1. LOG OVERVIEW ───
section "OVERVIEW"

# Boot info from first 60 lines
head -60 "$LOGFILE" | grep -iE "POWERON_RESET|SW_CPU_RESET|DEEPSLEEP_RESET|boot|rst:" || true

# Node ID
grep -m1 '\[INIT\].*_GW_ID:' "$LOGFILE" || true

# Callsign
grep -m1 '\[BLE \].*Device started with BLE-Name' "$LOGFILE" || true

# LoRa config
grep '\[LoRa\].*RF_' "$LOGFILE" || true

# CSMA
grep -m1 'CSMA_SLOT_TIME' "$LOGFILE" || true

# Heap/PSRAM at boot
grep -m1 '\[HEAP\]' "$LOGFILE" || true
grep -m1 '\[PSRM\]' "$LOGFILE" || true

# WiFi
grep '\[WIFI\]' "$LOGFILE" | head -10 || true

# Hardware (display, sensors)
grep -iE '\[INIT\].*display|\[INIT\].*BME|INA226|SHT|BMP' "$LOGFILE" | head -5 || true

# First and last timestamp
echo "FIRST_LINE: $(head -1 "$LOGFILE")"
echo "LAST_LINE: $(tail -1 "$LOGFILE")"
echo "TOTAL_LINES: $(wc -l < "$LOGFILE")"

# ─── 2. ACTIVE NODES ───
section "NODES"

# Single-pass awk: correlate "Received packet:" RSSI/SNR with subsequent MH-LoRa lines.
# Extract callsign from route field (first callsign before comma or ">").
awk '
/Received packet:/ {
    for (i=1; i<=NF; i++) {
        if ($i == "RSSI:") {
            for (j=i+1; j<=NF; j++) {
                if ($j != "" && $j ~ /^-?[0-9]/) { last_rssi = $j + 0; break }
            }
        }
        if ($i == "SNR:") {
            for (j=i+1; j<=NF; j++) {
                if ($j != "" && $j ~ /^-?[0-9]/) { last_snr = $j + 0; break }
            }
        }
    }
    has_rssi = 1
    next
}

/MH-LoRa:/ {
    # Serial monitor timestamp is $2 (HH:MM:SS.mmm)
    ts = $2

    # Find route field (contains ">")
    cs = ""
    for (i=1; i<=NF; i++) {
        if ($i ~ />/) {
            # Route like "DK7CH-1,DL7OSX-1>*!..."
            # First callsign is before first comma or ">"
            split($i, r, /[,>]/)
            cs = r[1]
            break
        }
    }
    if (cs == "" || cs ~ /^[0-9]/) next

    cnt[cs]++

    # RSSI/SNR from preceding Received packet line
    if (has_rssi) {
        if (!(cs in rssi_min) || last_rssi < rssi_min[cs]) rssi_min[cs] = last_rssi
        if (!(cs in rssi_max) || last_rssi > rssi_max[cs]) rssi_max[cs] = last_rssi
        if (!(cs in snr_min) || last_snr < snr_min[cs]) snr_min[cs] = last_snr
        if (!(cs in snr_max) || last_snr > snr_max[cs]) snr_max[cs] = last_snr
        snr_sum[cs] += last_snr
        snr_cnt[cs]++
        has_rssi = 0
    }

    # Hop count (H01, H02, ...)
    for (i=1; i<=NF; i++) {
        if ($i ~ /^H[0-9][0-9]$/) {
            h = $i
            if (!(cs in hop_vals)) hop_vals[cs] = h
            else if (index(hop_vals[cs], h) == 0) hop_vals[cs] = hop_vals[cs] "," h
            break
        }
    }

    # HW and FW
    for (i=1; i<=NF; i++) {
        if ($i ~ /^HW:/) hw[cs] = $i
        if ($i ~ /^FW:/) fw[cs] = $i
    }

    # First/last seen (serial monitor time)
    if (!(cs in first_seen)) first_seen[cs] = ts
    last_seen[cs] = ts
}
END {
    for (cs in cnt) {
        savg = (snr_cnt[cs] > 0) ? sprintf("%.1f", snr_sum[cs]/snr_cnt[cs]) : "?"
        if (cs in rssi_min)
            rssi_str = sprintf("%4d..%4d", rssi_min[cs], rssi_max[cs])
        else
            rssi_str = "       n/a"
        printf "%-15s %6d %12s %10s %8s %8s %8s %s\n", \
            cs, cnt[cs], rssi_str, savg, \
            (cs in hop_vals ? hop_vals[cs] : "?"), \
            (cs in hw ? hw[cs] : "?"), \
            (cs in fw ? fw[cs] : "?"), \
            first_seen[cs]
    }
}' "$LOGFILE" | (
    printf "%-15s %6s %12s %10s %8s %8s %8s %s\n" "CALLSIGN" "COUNT" "RSSI" "SNR_AVG" "HOPS" "HW" "FW" "FIRST_SEEN"
    sort -t' ' -k2 -rn
)

echo ""
echo "UNIQUE_NODES: $(grep "MH-LoRa:" "$LOGFILE" | awk '{for(i=1;i<=NF;i++){if($i~/>/){{split($i,r,/[,>]/);if(r[1]!=""&&r[1]!~/^[0-9]/)print r[1];break}}}}' | sort -u | wc -l | tr -d ' ')"
echo "TOTAL_MH_PACKETS: $(grep -c 'MH-LoRa:' "$LOGFILE")"

# ─── 3. MESSAGE TYPES ───
section "MESSAGE_TYPES"

grep "MH-LoRa:" "$LOGFILE" | awk '
{
    if (/H@R/ || /HG@R/) heartbeat++
    else if (/\*!/) position++
    else if (/:/) text++
    else other++
    total++
}
END {
    printf "HEARTBEAT: %d (%.1f%%)\n", heartbeat, (total>0 ? heartbeat*100.0/total : 0)
    printf "POSITION: %d (%.1f%%)\n", position, (total>0 ? position*100.0/total : 0)
    printf "TEXT: %d (%.1f%%)\n", text, (total>0 ? text*100.0/total : 0)
    printf "OTHER: %d (%.1f%%)\n", other, (total>0 ? other*100.0/total : 0)
    printf "TOTAL: %d\n", total
}' || true

# Notable text messages (extract up to 20)
echo ""
echo "--- NOTABLE_TEXTS ---"
grep "RX-LoRa-All:" "$LOGFILE" | grep -v 'H@R\|HG@R\|\*!' | head -20 || true

# ─── 4. HOP DISTRIBUTION ───
section "HOP_DISTRIBUTION"

echo "--- MH-LoRa (Empfangen) ---"
grep "MH-LoRa:" "$LOGFILE" | grep -oE 'H[0-9]{2}' | sort | uniq -c | sort -rn || true

echo "--- RX-LoRa2 (Akzeptiert) ---"
grep "RX-LoRa2:" "$LOGFILE" | grep -oE 'H[0-9]{2}' | sort | uniq -c | sort -rn || true

echo "--- TX-LoRa (Gesendet) ---"
grep "TX-LoRa:" "$LOGFILE" | grep -oE 'H[0-9]{2}' | sort | uniq -c | sort -rn || true

# ─── 5. LOOPS ───
section "LOOPS"

echo "RELAY_LOOP_BLOCKED: $(grep -c 'RELAY_LOOP_BLOCKED' "$LOGFILE" 2>/dev/null; true)"
grep "RELAY_LOOP_BLOCKED" "$LOGFILE" | head -10 || true

# ─── 6. CHANNEL UTILIZATION ───
section "CHANNEL_UTIL"

{ grep "CHANNEL_UTIL" "$LOGFILE" || true; } | awk '{
    # Serial monitor timestamp is $2 (HH:MM:SS.mmm)
    ts = $2
    for (i=1; i<=NF; i++) {
        if ($i ~ /^rx=/) {
            val = substr($i, 4) + 0.0
        }
        if ($i ~ /^util=/) {
            util = substr($i, 6) + 0.0
        }
    }
    # Use util% as the main metric
    if (util != "") print ts, util
}' > /tmp/channel_util.txt

if [ -s /tmp/channel_util.txt ]; then
    awk '
    {
        val = $2 + 0.0
        sum += val
        count++
        if (count == 1 || val > max) max = val
        if (count == 1 || val < min) min = val
        if (val >= 70) high70++
        if (val >= 90) high90++
        vals[count] = val
    }
    END {
        avg = sum / count
        half = int(count / 2)
        sum1 = 0; sum2 = 0
        for (i=1; i<=half; i++) sum1 += vals[i]
        for (i=half+1; i<=count; i++) sum2 += vals[i]
        avg1 = (half > 0) ? sum1 / half : 0
        avg2 = (count - half > 0) ? sum2 / (count - half) : 0
        printf "SAMPLES: %d\n", count
        printf "AVG: %.1f%%\n", avg
        printf "MIN: %.1f%%\n", min
        printf "MAX: %.1f%%\n", max
        printf "FIRST_HALF_AVG: %.1f%%\n", avg1
        printf "SECOND_HALF_AVG: %.1f%%\n", avg2
        printf "TREND: %s\n", (avg2 > avg1 * 1.1) ? "STEIGEND" : (avg1 > avg2 * 1.1) ? "FALLEND" : "STABIL"
        printf "SAMPLES_GE_70: %d\n", high70
        printf "SAMPLES_GE_90: %d\n", high90
    }' /tmp/channel_util.txt

    # Per 10-minute buckets
    echo ""
    echo "--- BUCKETS_10MIN ---"
    awk '{
        # Timestamp is HH:MM:SS.mmm — split by ":" for hour:minute
        split($1, t, ":")
        bucket = t[1] ":" sprintf("%02d", int(t[2]/10)*10)
        sum[bucket] += $2
        cnt[bucket]++
        if (!(bucket in mx) || $2 > mx[bucket]) mx[bucket] = $2
    }
    END {
        printf "%-8s %8s %8s %5s\n", "TIME", "AVG%", "MAX%", "N"
        for (b in sum) {
            printf "%-8s %8.1f %8.1f %5d\n", b, sum[b]/cnt[b], mx[b], cnt[b]
        }
    }' /tmp/channel_util.txt | sort
else
    echo "NO_DATA"
fi

# ─── 7. ACK ANALYSIS ───
section "ACK_ANALYSIS"

echo "ACK_FAST_QUEUED: $(grep -c 'ACK_FAST_QUEUED' "$LOGFILE" 2>/dev/null; true)"
echo "ACK_FAST_TX: $(grep -c 'ACK_FAST_TX' "$LOGFILE" 2>/dev/null; true)"
echo "ACK_RX_CANCEL: $(grep -c 'ACK_RX_CANCEL' "$LOGFILE" 2>/dev/null; true)"
echo "ACK_FWD_DEDUP: $(grep -c 'ACK_FWD_DEDUP' "$LOGFILE" 2>/dev/null; true)"
echo "GW_ACK_DEDUP: $(grep -c 'GW_ACK_DEDUP' "$LOGFILE" 2>/dev/null; true)"
echo "ACK_SLOT_SKIP: $(grep -c 'ACK_SLOT_SKIP' "$LOGFILE" 2>/dev/null; true)"
echo "ACK_FWD_DROPPED: $(grep -c 'ACK_FWD_DROPPED' "$LOGFILE" 2>/dev/null; true)"
echo "GW_ACK_DROPPED: $(grep -c 'GW_ACK_DROPPED' "$LOGFILE" 2>/dev/null; true)"
echo "ACK_CANCEL_RETRANSMIT: $(grep -c 'ACK_CANCEL_RETRANSMIT' "$LOGFILE" 2>/dev/null; true)"
echo "ACK_RECEIVED: $(grep -c 'ACK_RECEIVED' "$LOGFILE" 2>/dev/null; true)"
echo "ACK_FAST_CAD_BUSY: $(grep -c 'ACK_FAST_CAD_BUSY' "$LOGFILE" 2>/dev/null; true)"

# ACK queue length distribution
echo ""
echo "--- ACK_QLEN ---"
grep -oE 'ack_qlen=[0-9]+' "$LOGFILE" | sort | uniq -c | sort -rn || true

# ─── 8. CRC ERRORS ───
section "CRC_ERRORS"

echo "CRC_ERROR_COUNT: $(grep -c 'CRC_ERROR' "$LOGFILE" 2>/dev/null; true)"

grep "CRC_ERROR" "$LOGFILE" | awk '{
    for (i=1; i<=NF; i++) {
        if ($i ~ /^rssi=/) rssi = $i
        if ($i ~ /^snr=/) snr = $i
        if ($i ~ /^freq_err=/) ferr = $i
        if ($i ~ /^size=/) sz = $i
    }
    print $2, rssi, snr, ferr, sz
}' | head -50 || true

# Classify by freq error
echo ""
echo "--- CRC_FREQ_CLASSIFICATION ---"
grep "CRC_ERROR" "$LOGFILE" | grep -oE 'freq_err=[0-9.-]+' | awk -F= '{
    v = $2 + 0; if (v < 0) v = -v
    if (v > 3000) offfreq++
    else if (v > 1000) medium++
    else collision++
}
END {
    printf "OFF_FREQUENCY (>3kHz): %d\n", offfreq+0
    printf "MEDIUM (1-3kHz): %d\n", medium+0
    printf "COLLISION (<1kHz): %d\n", collision+0
}' || true

# ─── 9. RETRIES (RING_STATUS) ───
section "RING_STATUS"

echo "RING_STATUS_COUNT: $(grep -c 'RING_STATUS' "$LOGFILE" 2>/dev/null; true)"

echo "--- RETRYING ---"
grep "RING_STATUS" "$LOGFILE" | grep -oE 'retrying=[0-9]+' | sort | uniq -c | sort -rn || true

echo "--- PENDING ---"
grep "RING_STATUS" "$LOGFILE" | grep -oE 'pending=[0-9]+' | sort | uniq -c | sort -rn || true

echo "--- QUEUED ---"
grep "RING_STATUS" "$LOGFILE" | grep -oE 'queued=[0-9]+' | sort | uniq -c | sort -rn || true

# ─── 10. MISSING ACKS ───
section "MISSING_ACKS"

echo "ACK_TIMEOUT: $(grep -ci 'ACK_TIMEOUT' "$LOGFILE" 2>/dev/null; true)"
echo "ACK_FAIL: $(grep -ci 'ACK_FAIL' "$LOGFILE" 2>/dev/null; true)"
echo "ACK_MISS: $(grep -ci 'ACK_MISS' "$LOGFILE" 2>/dev/null; true)"
echo "ACK_LOST: $(grep -ci 'ACK_LOST' "$LOGFILE" 2>/dev/null; true)"
echo "RETRANSMIT_GIVEUP: $(grep -c 'RETRANSMIT_GIVEUP' "$LOGFILE" 2>/dev/null; true)"

# ─── 11. DEDUP ───
section "DEDUP"

echo "DEDUP_EXPLICIT: $(grep -ci 'dedup\|DEDUP' "$LOGFILE" 2>/dev/null; true)"

MH_COUNT=$(grep -c "MH-LoRa:" "$LOGFILE" 2>/dev/null; true)
RX_COUNT=$(grep -c "RX-LoRa2:" "$LOGFILE" 2>/dev/null; true)
echo "MH_LORA_HEARD: $MH_COUNT"
echo "RX_LORA_ACCEPTED: $RX_COUNT"
echo "IMPLICIT_DEDUP: $((MH_COUNT - RX_COUNT))"

# Top repeated msg_ids
echo ""
echo "--- TOP_REPEATED_MSGIDS ---"
grep "MH-LoRa:" "$LOGFILE" | grep -oE 'x[A-F0-9]{8}' | sort | uniq -c | sort -rn | head -10 || true

# Max duplication factor
echo ""
echo "--- DUPLICATION_DISTRIBUTION ---"
grep "MH-LoRa:" "$LOGFILE" | grep -oE 'x[A-F0-9]{8}' | sort | uniq -c | awk '{print $1}' | sort -n | uniq -c | sort -rn || true

# ─── 12. STATE MACHINE ───
section "STATE_MACHINE"

echo "MC_SM_TOTAL: $(grep -c 'MC-SM' "$LOGFILE" 2>/dev/null; true)"
echo "MC_SM_ERRORS: $(grep 'MC-SM' "$LOGFILE" | grep -vc 'rc=0' 2>/dev/null; true)"

grep "MC-SM" "$LOGFILE" | grep -v "rc=0" | head -10 || true

# ─── 13. ADDITIONAL CHECKS ───
section "ADDITIONAL"

echo "--- WIFI_ISSUES ---"
grep -iE 'disconnect|reconnect|WIFI.*fail' "$LOGFILE" | head -10 || true
echo "WIFI_ISSUE_COUNT: $(grep -ciE 'disconnect|reconnect|WIFI.*fail' "$LOGFILE" 2>/dev/null; true)"

echo ""
echo "--- CRASHES ---"
grep -iE 'panic|abort|watchdog|wdt|backtrace|guru.meditation' "$LOGFILE" | head -10 || true
echo "CRASH_COUNT: $(grep -ciE 'panic|abort|watchdog|wdt|backtrace|guru.meditation' "$LOGFILE" 2>/dev/null; true)"

echo ""
echo "--- HEAP_TREND ---"
{ grep '\[HEAP\]' "$LOGFILE" || true; } | head -5
echo "..."
{ grep '\[HEAP\]' "$LOGFILE" || true; } | tail -5
HEAP_SAMPLES=$(grep -c '\[HEAP\]' "$LOGFILE" 2>/dev/null; true)
echo "HEAP_SAMPLES: $HEAP_SAMPLES"

echo ""
echo "--- ONRXDONE_TIME ---"
grep "ONRXDONE_TIME" "$LOGFILE" | grep -oE 'ms=[0-9]+' | awk -F= '{
    sum += $2; count++
    if (count == 1 || $2 > max) max = $2
    if (count == 1 || $2 < min) min = $2
}
END {
    if (count > 0) printf "AVG: %.0f ms, MIN: %d, MAX: %d, SAMPLES: %d\n", sum/count, min, max, count
    else print "NO_DATA"
}' || true

echo ""
echo "--- RX_TIMEOUT_FIRE ---"
echo "RX_TIMEOUT_FIRE: $(grep -c 'RX_TIMEOUT_FIRE' "$LOGFILE" 2>/dev/null; true)"
grep "RX_TIMEOUT_FIRE" "$LOGFILE" | grep -oE 'wait=[0-9.]+' | awk -F= '{
    sum += $2; count++
    if (count == 1 || $2 > max) max = $2
    if (count == 1 || $2 < min) min = $2
}
END {
    if (count > 0) printf "ADAPTIVE_WAIT: AVG=%.0f MIN=%.0f MAX=%.0f ms (%d samples)\n", sum/count, min, max, count
    else print "NO_WAIT_DATA"
}' || true

echo ""
echo "--- BUFFER_DROPS ---"
echo "BUFFER_DROPS: $(grep -c 'DROPPED.*buffer_full\|_DROPPED' "$LOGFILE" 2>/dev/null; true)"
grep -E 'DROPPED|buffer_full' "$LOGFILE" | head -10 || true

echo ""
echo "--- CAD_STATS ---"
echo "CAD_GIVEUP: $(grep -c 'CAD_GIVEUP' "$LOGFILE" 2>/dev/null; true)"
echo "CAD_FALSE_POSITIVE: $(grep -c 'CAD_FALSE_POSITIVE' "$LOGFILE" 2>/dev/null; true)"
echo "RX_TIMEOUT_DEFERRED: $(grep -c 'RX_TIMEOUT_DEFERRED' "$LOGFILE" 2>/dev/null; true)"

# ─── 14. CRC DETAIL ───
section "CRC_DETAIL"

echo "--- CRC_10MIN_BUCKETS ---"
grep "CRC_ERROR" "$LOGFILE" | awk '{
    split($2, t, ":")
    bucket = t[1] ":" sprintf("%02d", int(t[2]/10)*10)
    cnt[bucket]++
}
END {
    printf "%-8s %6s\n", "TIME", "CRC"
    for (b in cnt) printf "%-8s %6d\n", b, cnt[b]
}' | sort || true

echo ""
echo "--- CRC_HOURLY_RATE ---"
# CRC errors per hour vs received packets per hour
awk '
/CRC_ERROR/ {
    split($2, t, ":")
    h = t[1]
    crc[h]++
}
/MH-LoRa:/ {
    split($2, t, ":")
    h = t[1]
    rx[h]++
}
END {
    printf "%-6s %6s %6s %8s\n", "HOUR", "CRC", "RX", "RATE%"
    for (h in crc) {
        r = (h in rx && rx[h] > 0) ? crc[h] * 100.0 / rx[h] : 0
        printf "%-6s %6d %6d %8.1f\n", h, crc[h], (h in rx ? rx[h] : 0), r
    }
}' "$LOGFILE" | sort || true

echo ""
echo "--- CRC_CLUSTERS ---"
# CRC errors within 2s of each other = collision bursts
grep "CRC_ERROR" "$LOGFILE" | awk '{
    split($2, t, ":")
    split(t[3], s, ".")
    ts = t[1] * 3600 + t[2] * 60 + s[1] + (length(s) > 1 ? ("0." s[2]) + 0 : 0)
    if (NR > 1 && ts - prev_ts < 2.0) {
        if (!in_cluster) { cluster_size = 2; cluster_start = prev_ts_str; in_cluster = 1 }
        else cluster_size++
    } else {
        if (in_cluster) {
            clusters++
            total_in_clusters += cluster_size
            if (cluster_size > max_cluster) max_cluster = cluster_size
            printf "CLUSTER at %s size=%d gap=%.1fs\n", cluster_start, cluster_size, prev_ts - cluster_start_ts
            in_cluster = 0
        }
    }
    if (!in_cluster) { cluster_start_ts = ts }
    prev_ts = ts
    prev_ts_str = $2
}
END {
    if (in_cluster) {
        clusters++
        total_in_clusters += cluster_size
        if (cluster_size > max_cluster) max_cluster = cluster_size
        printf "CLUSTER at %s size=%d\n", cluster_start, cluster_size
    }
    printf "\nCLUSTER_COUNT: %d\n", clusters+0
    printf "TOTAL_CRC_IN_CLUSTERS: %d\n", total_in_clusters+0
    printf "MAX_CLUSTER_SIZE: %d\n", max_cluster+0
}' || true

echo ""
echo "--- CRC_VS_CHANNEL_UTIL ---"
# Correlate CRC buckets with channel utilization buckets
awk '
/CRC_ERROR/ {
    split($2, t, ":")
    bucket = t[1] ":" sprintf("%02d", int(t[2]/10)*10)
    crc[bucket]++
}
/CHANNEL_UTIL/ {
    split($2, t, ":")
    bucket = t[1] ":" sprintf("%02d", int(t[2]/10)*10)
    for (i=1; i<=NF; i++) {
        if ($i ~ /^util=/) {
            util_sum[bucket] += substr($i, 6) + 0.0
            util_cnt[bucket]++
        }
    }
}
END {
    printf "%-8s %6s %8s\n", "TIME", "CRC", "UTIL%"
    for (b in crc) {
        u = (b in util_cnt && util_cnt[b] > 0) ? util_sum[b] / util_cnt[b] : -1
        if (u >= 0)
            printf "%-8s %6d %8.1f\n", b, crc[b], u
        else
            printf "%-8s %6d %8s\n", b, crc[b], "n/a"
    }
}' "$LOGFILE" | sort || true

# ─── 15. CAD ATTEMPT DISTRIBUTION ───
section "CAD_ATTEMPT_DISTRIBUTION"

grep "TX_GATE_ENTER" "$LOGFILE" | grep -oE 'cad_attempt=[0-9]+' | awk -F= '{
    cnt[$2]++
    total++
}
END {
    printf "%-12s %8s %8s %12s\n", "ATTEMPT", "COUNT", "PCT%", "EST_WAIT_MS"
    max_a = 0
    for (a in cnt) { if (a+0 > max_a) max_a = a+0 }
    for (i = 0; i <= max_a; i++) {
        if (i in cnt) {
            if (i == 0) wait = 4675
            else if (i == 1) wait = 3087
            else if (i == 2) wait = 2087
            else wait = 35
            printf "%-12d %8d %8.1f %12d\n", i, cnt[i], cnt[i] * 100.0 / total, wait
        }
    }
    printf "\nTOTAL_TX_ATTEMPTS: %d\n", total

    # Summary buckets
    free_first = cnt[0] + 0
    busy_1 = cnt[1] + 0
    busy_2 = cnt[2] + 0
    starvation = 0
    for (a in cnt) { if (a + 0 >= 3) starvation += cnt[a] }
    printf "SOFORT_FREI (attempt=0): %d (%.1f%%)\n", free_first, (total > 0 ? free_first * 100.0 / total : 0)
    printf "1x_BUSY (attempt=1): %d (%.1f%%)\n", busy_1, (total > 0 ? busy_1 * 100.0 / total : 0)
    printf "2x_BUSY (attempt=2): %d (%.1f%%)\n", busy_2, (total > 0 ? busy_2 * 100.0 / total : 0)
    printf "STARVATION (attempt>=3): %d (%.1f%%)\n", starvation, (total > 0 ? starvation * 100.0 / total : 0)
}' || true

# ─── 16. CAD STORM ANALYSIS ───
section "CAD_STORM"

# Extract CAD storm episodes: sequences where cad_attempt >= 3
# A storm starts at the first TX_GATE_ENTER with cad_attempt>=3 and ends at TX_DONE or cad_attempt=0
awk '
/TX_GATE_ENTER/ {
    split($2, t, ":")
    split(t[3], s, ".")
    ts = t[1] * 3600 + t[2] * 60 + s[1] + (length(s) > 1 ? ("0." s[2]) + 0 : 0)
    ts_str = $2

    for (i=1; i<=NF; i++) {
        if ($i ~ /^cad_attempt=/) {
            attempt = substr($i, 13) + 0
        }
    }

    if (attempt >= 3) {
        if (!in_storm) {
            in_storm = 1
            storm_start = ts
            storm_start_str = ts_str
            storm_max = attempt
            storm_scans = 1
        } else {
            storm_scans++
            if (attempt > storm_max) storm_max = attempt
        }
    }

    if (attempt == 0 && in_storm) {
        # Storm ended
        duration = ts - storm_start
        storms++
        dur_sum += duration
        if (storms == 1 || duration > dur_max) dur_max = duration
        if (storms == 1 || duration < dur_min) dur_min = duration
        durations[storms] = duration

        if (duration < 1) bucket_lt1++
        else if (duration < 2) bucket_1_2++
        else if (duration < 5) bucket_2_5++
        else if (duration < 10) bucket_5_10++
        else if (duration < 30) bucket_10_30++
        else bucket_gt30++

        printf "STORM at %s dur=%.1fs max_attempt=%d scans=%d\n", storm_start_str, duration, storm_max, storm_scans
        in_storm = 0
    }
}
/TX_DONE/ {
    if (in_storm) {
        split($2, t, ":")
        split(t[3], s, ".")
        ts = t[1] * 3600 + t[2] * 60 + s[1] + (length(s) > 1 ? ("0." s[2]) + 0 : 0)
        duration = ts - storm_start
        storms++
        dur_sum += duration
        if (storms == 1 || duration > dur_max) dur_max = duration
        if (storms == 1 || duration < dur_min) dur_min = duration
        durations[storms] = duration

        if (duration < 1) bucket_lt1++
        else if (duration < 2) bucket_1_2++
        else if (duration < 5) bucket_2_5++
        else if (duration < 10) bucket_5_10++
        else if (duration < 30) bucket_10_30++
        else bucket_gt30++

        printf "STORM at %s dur=%.1fs max_attempt=%d scans=%d (ended by TX_DONE)\n", storm_start_str, duration, storm_max, storm_scans
        in_storm = 0
    }
}
END {
    if (in_storm) {
        printf "STORM at %s OPEN (still active at log end) max_attempt=%d scans=%d\n", storm_start_str, storm_max, storm_scans
    }
    printf "\n--- STORM_SUMMARY ---\n"
    printf "TOTAL_STORMS: %d\n", storms+0
    if (storms > 0) {
        printf "DURATION_MIN: %.1fs\n", dur_min
        printf "DURATION_AVG: %.1fs\n", dur_sum / storms
        printf "DURATION_MAX: %.1fs\n", dur_max
        # Sort durations for median
        n = storms
        for (i=1; i<=n; i++) sorted[i] = durations[i]
        for (i=1; i<=n; i++) for (j=i+1; j<=n; j++) if (sorted[j] < sorted[i]) { tmp = sorted[i]; sorted[i] = sorted[j]; sorted[j] = tmp }
        if (n % 2 == 1) median = sorted[int(n/2)+1]
        else median = (sorted[n/2] + sorted[n/2+1]) / 2.0
        printf "DURATION_MEDIAN: %.1fs\n", median
    }
    printf "\n--- STORM_DURATION_HISTOGRAM ---\n"
    printf "<1s:    %d\n", bucket_lt1+0
    printf "1-2s:   %d\n", bucket_1_2+0
    printf "2-5s:   %d\n", bucket_2_5+0
    printf "5-10s:  %d\n", bucket_5_10+0
    printf "10-30s: %d\n", bucket_10_30+0
    printf ">30s:   %d\n", bucket_gt30+0

    printf "\n--- 500ms_RX_LOOP_ANALYSIS ---\n"
    if (storms > 0) {
        cost_500ms = storms * 0.5
        printf "STORMS_COUNT: %d\n", storms
        printf "TOTAL_STORM_TIME: %.1fs\n", dur_sum
        printf "COST_500ms_RX_LOOP: %.1fs (if added before each storm)\n", cost_500ms
        printf "NET_SAVINGS_IF_RX_LOOP_CATCHES_PKT: Depends on hit rate\n"
        printf "NOTE: A 500ms RX loop at storm start would detect preamble+header\n"
        printf "      and receive the packet (~2.5s), avoiding repeated CAD polls.\n"
        printf "      Break-even: if >%.0f%% of storms have a receivable packet.\n", cost_500ms * 100.0 / dur_sum
    }
}' "$LOGFILE" || true

# ─── 17. RING OVERFLOW ───
section "RING_OVERFLOW"

echo "--- RING_DROP_EVENTS ---"
RING_DROPS=$(grep -c 'RING_DROP' "$LOGFILE" 2>/dev/null || true)
echo "RING_DROP_COUNT: $RING_DROPS"
grep "RING_DROP" "$LOGFILE" || true

echo ""
echo "--- RING_WATERMARK ---"
grep "RING_STATUS" "$LOGFILE" | grep -oE 'queued=[0-9]+' | awk -F= '{
    v = $2 + 0
    total++
    if (total == 1 || v > max) max = v
    if (v == 0) q0++
    else if (v <= 5) q1_5++
    else if (v <= 10) q10++
    else if (v <= 15) q15++
    else q20++
    sum += v
}
END {
    printf "SAMPLES: %d\n", total+0
    printf "MAX_QUEUED: %d\n", max+0
    printf "AVG_QUEUED: %.1f\n", (total > 0 ? sum / total : 0)
    printf "\n--- WATERMARK_HISTOGRAM ---\n"
    printf "queued=0:     %6d (%5.1f%%)\n", q0+0, (total > 0 ? (q0+0)*100.0/total : 0)
    printf "queued=1-5:   %6d (%5.1f%%)\n", q1_5+0, (total > 0 ? (q1_5+0)*100.0/total : 0)
    printf "queued=6-10:  %6d (%5.1f%%)\n", q10+0, (total > 0 ? (q10+0)*100.0/total : 0)
    printf "queued=11-15: %6d (%5.1f%%)\n", q15+0, (total > 0 ? (q15+0)*100.0/total : 0)
    printf "queued=16-20: %6d (%5.1f%%)\n", q20+0, (total > 0 ? (q20+0)*100.0/total : 0)
}' || true

echo ""
echo "--- RING_WATERMARK_OVER_TIME ---"
grep "RING_STATUS" "$LOGFILE" | awk '{
    split($2, t, ":")
    bucket = t[1] ":" sprintf("%02d", int(t[2]/10)*10)
    for (i=1; i<=NF; i++) {
        if ($i ~ /^queued=/) {
            v = substr($i, 8) + 0
            sum[bucket] += v
            cnt[bucket]++
            if (!(bucket in mx) || v > mx[bucket]) mx[bucket] = v
        }
    }
}
END {
    printf "%-8s %8s %8s %5s\n", "TIME", "AVG_Q", "MAX_Q", "N"
    for (b in cnt) printf "%-8s %8.1f %8d %5d\n", b, sum[b]/cnt[b], mx[b], cnt[b]
}' | sort || true

# ─── 18. DROPPED PACKETS (comprehensive) ───
section "DROPPED_PACKETS"

echo "--- DROP_CATEGORIES ---"
RING_DROP_N=$(grep -c 'RING_DROP' "$LOGFILE" 2>/dev/null || true); echo "RING_DROP: ${RING_DROP_N:-0}"
CRC_ERR_N=$(grep -c 'CRC_ERROR' "$LOGFILE" 2>/dev/null || true); echo "CRC_ERROR: ${CRC_ERR_N:-0}"
DEDUP_N=$(grep -c 'RX_DEDUP_DUP' "$LOGFILE" 2>/dev/null || true); echo "RX_DEDUP_DUP: ${DEDUP_N:-0}"
LOOP_N=$(grep -c 'RELAY_LOOP_BLOCKED' "$LOGFILE" 2>/dev/null || true); echo "RELAY_LOOP_BLOCKED: ${LOOP_N:-0}"
AFD_N=$(grep -c 'ACK_FWD_DROPPED' "$LOGFILE" 2>/dev/null || true); echo "ACK_FWD_DROPPED: ${AFD_N:-0}"
GAD_N=$(grep -c 'GW_ACK_DROPPED' "$LOGFILE" 2>/dev/null || true); echo "GW_ACK_DROPPED: ${GAD_N:-0}"
AFDD_N=$(grep -c 'ACK_FWD_DEDUP' "$LOGFILE" 2>/dev/null || true); echo "ACK_FWD_DEDUP: ${AFDD_N:-0}"
GADD_N=$(grep -c 'GW_ACK_DEDUP' "$LOGFILE" 2>/dev/null || true); echo "GW_ACK_DEDUP: ${GADD_N:-0}"
RXOE_N=$(grep -c 'RX_OTHER_ERROR' "$LOGFILE" 2>/dev/null || true); echo "RX_OTHER_ERROR: ${RXOE_N:-0}"
RTGU_N=$(grep -c 'RETRANSMIT_GIVEUP' "$LOGFILE" 2>/dev/null || true); echo "RETRANSMIT_GIVEUP: ${RTGU_N:-0}"
IRQS_N=$(grep -c 'RX_IRQ_STALE' "$LOGFILE" 2>/dev/null || true); echo "RX_IRQ_STALE: ${IRQS_N:-0}"

echo ""
echo "--- HOP_LIMIT_DROPS ---"
# Packets where relay was suppressed due to hop limit
# These show up as MH-LoRa but NOT RX-LoRa2 (heard but not accepted)
# We detect by counting path segments in MH-LoRa lines
grep "MH-LoRa:" "$LOGFILE" | awk '{
    for (i=1; i<=NF; i++) {
        if ($i ~ />/) {
            # Count commas in the path before ">"
            split($i, parts, ">")
            path = parts[1]
            n = gsub(/,/, ",", path)
            path_len = n + 1  # number of callsigns = relays
            if (path_len > 4) {
                high_path++
                # Extract msg_id (xHEXHEXHEX)
                for (j=1; j<=NF; j++) {
                    if ($j ~ /^x[A-Fa-f0-9]{8}$/) { mid = $j; break }
                }
                for (j=1; j<=NF; j++) {
                    if ($j ~ /^H[0-9A-F][0-9A-F]$/) { hop = $j; break }
                }
                # payload type
                type = ""
                if (/H@R/ || /HG@R/) type = "HB"
                else if (/\*!/) type = "POS"
                else type = "TXT"
            }
            path_dist[path_len]++
            break
        }
    }
}
END {
    printf "--- PATH_LENGTH_DISTRIBUTION ---\n"
    printf "%-8s %8s\n", "PATH_LEN", "COUNT"
    for (p in path_dist) printf "%-8d %8d\n", p, path_dist[p]
    printf "\nPATH_LENGTH_GT_4: %d\n", high_path+0
}' | sort -t' ' -k1 -n || true

echo ""
echo "--- ALL_DROPS_SAMPLES ---"
grep -E 'RING_DROP|RX_OTHER_ERROR|RELAY_LOOP_BLOCKED|RX_IRQ_STALE|RETRANSMIT_GIVEUP' "$LOGFILE" | head -30 || true

# ─── 19. HIGH HOP PACKETS ───
section "HIGH_HOP_PACKETS"

# List all packets with path length > 4 (more than 4 relay hops)
grep "MH-LoRa:" "$LOGFILE" | awk '{
    for (i=1; i<=NF; i++) {
        if ($i ~ />/) {
            split($i, parts, ">")
            path = parts[1]
            n = gsub(/,/, ",", path)
            path_len = n + 1
            if (path_len > 4) {
                # Reconstruct key info
                mid = "?"
                hop = "?"
                for (j=1; j<=NF; j++) {
                    if ($j ~ /^x[A-Fa-f0-9]{8}$/) { mid = $j; break }
                }
                for (j=1; j<=NF; j++) {
                    if ($j ~ /^H[0-9A-F][0-9A-F]$/) { hop = $j; break }
                }
                type = "?"
                if (/H@R/ || /HG@R/) type = "HB"
                else if (/\*!/) type = "POS"
                else type = "TXT"
                printf "%s %s path_len=%d hop_remain=%s type=%s path=%s\n", $2, mid, path_len, hop, type, $i
            }
            break
        }
    }
}' || true

echo ""
echo "--- PATH_LENGTH_SUMMARY ---"
grep "MH-LoRa:" "$LOGFILE" | awk '{
    for (i=1; i<=NF; i++) {
        if ($i ~ />/) {
            split($i, parts, ">")
            path = parts[1]
            n = gsub(/,/, ",", path)
            path_len = n + 1
            dist[path_len]++
            total++
            break
        }
    }
}
END {
    printf "%-8s %8s %8s\n", "PATH_LEN", "COUNT", "PCT%"
    for (p in dist) printf "%-8d %8d %8.1f\n", p, dist[p], dist[p] * 100.0 / total
    printf "TOTAL: %d\n", total
}' | sort -t' ' -k1 -n || true

# ─── 20. CROSS-LOG CORRELATION ───
if [ $# -ge 2 ] && [ -f "$2" ]; then
    LOGFILE2="$2"
    section "CROSS_CORRELATION"

    echo "LOG1: $LOGFILE"
    echo "LOG2: $LOGFILE2"

    # Extract msg_ids and timestamps from MH-LoRa lines in both logs
    grep "MH-LoRa:" "$LOGFILE" | awk '{
        for (i=1; i<=NF; i++) {
            if ($i ~ /^x[A-Fa-f0-9]{8}$/) { print $i, $2; break }
        }
    }' | sort -u -k1,1 > /tmp/cross_log1_msgids.txt

    grep "MH-LoRa:" "$LOGFILE2" | awk '{
        for (i=1; i<=NF; i++) {
            if ($i ~ /^x[A-Fa-f0-9]{8}$/) { print $i, $2; break }
        }
    }' | sort -u -k1,1 > /tmp/cross_log2_msgids.txt

    LOG1_TOTAL=$(wc -l < /tmp/cross_log1_msgids.txt | tr -d ' ')
    LOG2_TOTAL=$(wc -l < /tmp/cross_log2_msgids.txt | tr -d ' ')

    # Common msg_ids
    COMMON=$(comm -12 <(awk '{print $1}' /tmp/cross_log1_msgids.txt | sort) <(awk '{print $1}' /tmp/cross_log2_msgids.txt | sort) | wc -l | tr -d ' ')
    ONLY_LOG1=$(comm -23 <(awk '{print $1}' /tmp/cross_log1_msgids.txt | sort) <(awk '{print $1}' /tmp/cross_log2_msgids.txt | sort) | wc -l | tr -d ' ')
    ONLY_LOG2=$(comm -13 <(awk '{print $1}' /tmp/cross_log1_msgids.txt | sort) <(awk '{print $1}' /tmp/cross_log2_msgids.txt | sort) | wc -l | tr -d ' ')

    echo "LOG1_UNIQUE_MSGIDS: $LOG1_TOTAL"
    echo "LOG2_UNIQUE_MSGIDS: $LOG2_TOTAL"
    echo "COMMON_MSGIDS: $COMMON"
    echo "ONLY_IN_LOG1: $ONLY_LOG1"
    echo "ONLY_IN_LOG2: $ONLY_LOG2"

    echo ""
    echo "--- RECEPTION_ASYMMETRY ---"
    if [ "$LOG1_TOTAL" -gt 0 ]; then
        echo "LOG1_RECEPTION_RATE: $(echo "scale=1; $COMMON * 100 / $LOG1_TOTAL" | bc)% of LOG1 also heard by LOG2"
    fi
    if [ "$LOG2_TOTAL" -gt 0 ]; then
        echo "LOG2_RECEPTION_RATE: $(echo "scale=1; $COMMON * 100 / $LOG2_TOTAL" | bc)% of LOG2 also heard by LOG1"
    fi

    echo ""
    echo "--- RELAY_DELAY ---"
    # For common msg_ids, compute time difference between first reception in each log
    join -j 1 <(sort -k1,1 /tmp/cross_log1_msgids.txt) <(sort -k1,1 /tmp/cross_log2_msgids.txt) | awk '{
        # $1=msg_id, $2=ts_log1 (HH:MM:SS.mmm), $3=ts_log2 (HH:MM:SS.mmm)
        split($2, t1, ":")
        split(t1[3], s1, ".")
        sec1 = t1[1] * 3600 + t1[2] * 60 + s1[1] + (length(s1) > 1 ? ("0." s1[2]) + 0 : 0)

        split($3, t2, ":")
        split(t2[3], s2, ".")
        sec2 = t2[1] * 3600 + t2[2] * 60 + s2[1] + (length(s2) > 1 ? ("0." s2[2]) + 0 : 0)

        diff = sec2 - sec1
        if (diff < 0) diff = -diff
        sum += diff
        count++
        if (count == 1 || diff > max) max = diff
        if (count == 1 || diff < min) min = diff
        if (diff < 1) d_lt1++
        else if (diff < 5) d_1_5++
        else if (diff < 10) d_5_10++
        else d_gt10++
    }
    END {
        if (count > 0) {
            printf "SAMPLES: %d\n", count
            printf "DELAY_MIN: %.1fs\n", min
            printf "DELAY_AVG: %.1fs\n", sum / count
            printf "DELAY_MAX: %.1fs\n", max
            printf "\n--- DELAY_HISTOGRAM ---\n"
            printf "<1s:   %d\n", d_lt1+0
            printf "1-5s:  %d\n", d_1_5+0
            printf "5-10s: %d\n", d_5_10+0
            printf ">10s:  %d\n", d_gt10+0
        } else {
            print "NO_COMMON_PACKETS"
        }
    }' || true

    echo ""
    echo "--- ONLY_IN_LOG1_SAMPLES ---"
    comm -23 <(awk '{print $1}' /tmp/cross_log1_msgids.txt | sort) <(awk '{print $1}' /tmp/cross_log2_msgids.txt | sort) | head -20 || true

    echo ""
    echo "--- ONLY_IN_LOG2_SAMPLES ---"
    comm -13 <(awk '{print $1}' /tmp/cross_log1_msgids.txt | sort) <(awk '{print $1}' /tmp/cross_log2_msgids.txt | sort) | head -20 || true

    # Cleanup cross-correlation temp files
    rm -f /tmp/cross_log1_msgids.txt /tmp/cross_log2_msgids.txt
fi

# ─── PRIORITY DISTRIBUTION ───
section "PRIORITY_DISTRIBUTION"
if grep -q '\[MC-STAT\]' "$LOGFILE"; then
    echo "--- TX counts per priority (from [MC-STAT] lines) ---"
    grep '\[MC-STAT\]' "$LOGFILE" | tail -20
    echo ""
    echo "--- Latency per priority (from [MC-PRIO] lines) ---"
    grep '\[MC-PRIO\]' "$LOGFILE" | tail -20
    echo ""
    echo "--- Priority drops ---"
    grep 'RING_DROP_PRIO\|RING_DROP_NEW' "$LOGFILE" | wc -l | awk '{printf "  Total priority drops: %d\n", $1}'
    grep 'RING_DROP_PRIO' "$LOGFILE" | grep -oE 'prio=[0-9]' | sort | uniq -c | sort -rn || true
else
    echo "  No [MC-STAT] data found (priority queue not active or no data yet)"
fi

# ─── TRICKLE HEY ───
section "TRICKLE_HEY"
if grep -q '\[MC-TRICKLE\]' "$LOGFILE"; then
    echo "--- Trickle-HEY events ---"
    trickle_send=$(grep -c 'MC-TRICKLE.*SEND' "$LOGFILE" || true)
    trickle_suppress=$(grep -c 'MC-TRICKLE.*SUPPRESS' "$LOGFILE" || true)
    trickle_topo=$(grep -c 'MC-TRICKLE.*TOPO_CHANGE' "$LOGFILE" || true)
    trickle_total=$((trickle_send + trickle_suppress))
    if [ "$trickle_total" -gt 0 ]; then
        suppress_pct=$((trickle_suppress * 100 / trickle_total))
    else
        suppress_pct=0
    fi
    echo "  HEY sent: $trickle_send"
    echo "  HEY suppressed: $trickle_suppress ($suppress_pct%)"
    echo "  Topology changes: $trickle_topo"
    echo ""
    echo "--- Last 10 Trickle events ---"
    grep '\[MC-TRICKLE\]' "$LOGFILE" | tail -10
else
    echo "  No [MC-TRICKLE] data found (Trickle-HEY not active or no data yet)"
fi

# ─── HIGH WATER MARKS ───
section "HIGH_WATER_MARKS"
if grep -q '\[MC-HWM\]' "$LOGFILE"; then
    echo "--- High-Water Marks ---"
    grep '\[MC-HWM\]' "$LOGFILE" | tail -5
else
    echo "  No [MC-HWM] data found"
fi

# ─── DONE ───
section "END"
echo "Analysis complete."

# Cleanup
rm -f /tmp/channel_util.txt
