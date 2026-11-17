#!/bin/sh
# compare_scenario.sh - Compare two scenario trajectory CSVs (scenario_ref vs scenario_orig).
#
# usage: compare_scenario.sh ref.csv orig.csv [tol=1e-3]
#
# Rows are matched by (frame,obj). Every numeric column must agree within tol.
# Prints the worst absolute difference per column and exits 0 if all rows match, 1 otherwise.

REF=$1
ORIG=$2
TOL=${3:-1e-3}

if [ -z "$REF" ] || [ -z "$ORIG" ]; then
    echo "usage: $0 ref.csv orig.csv [tol]" >&2
    exit 2
fi

awk -F, -v tol="$TOL" '
    FNR == 1 { next }                       # header of each file
    NR == FNR { key = $1 "," $2; for (i = 3; i <= NF; i++) r[key, i] = $i; rows[key] = 1; next }
    {
        key = $1 "," $2
        if (!(key in rows)) { missing++; next }
        seen[key] = 1
        for (i = 3; i <= NF; i++) {
            d = r[key, i] - $i
            if (d < 0) d = -d
            if (d > worst[i]) worst[i] = d
            if (d > tol) bad++
        }
    }
    END {
        split("x y vx_rel vy_rel vx_abs vy_abs mach temp_K", names, " ")
        n = 0; for (k in rows) n++
        m = 0; for (k in seen) m++
        printf "rows in ref: %d, rows matched in orig: %d, unmatched orig rows: %d\n", n, m, missing + 0
        for (i = 3; i <= 10; i++) printf "  %-7s max |diff| = %.3g\n", names[i - 2], worst[i] + 0
        if (bad == 0 && missing == 0 && n == m) { print "SCENARIO: CORRECTE"; exit 0 }
        printf "SCENARIO: %d VALORS FORA DE TOLERANCIA (tol=%s)\n", bad + 0, tol
        exit 1
    }
' "$REF" "$ORIG"
