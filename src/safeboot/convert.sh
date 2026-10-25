# Regenerates ota.h from ota.html. Leading indentation is stripped first: the
# classic-ESP32 safeboot image is within a few hundred bytes of its 704 KB slot
#, and the page is ~9 KB of pure indentation. The arrays are
# const so the 36 KB page stays in flash instead of competing with the 44 KB
# inflate block for DRAM.
t=$(mktemp -d) || exit 1
sed 's/^[[:space:]]*//' ota.html > "$t/ota.html" && (cd "$t" && xxd -i ota.html) | sed 's/^unsigned/const unsigned/' > ota.h
rc=$?
rm -rf "$t"
exit $rc
