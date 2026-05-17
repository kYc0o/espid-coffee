#!/usr/bin/env bash
# Verify each commit on the current branch is independently buildable and
# (where applicable) test-green. Useful for checking strict bisectability
# after a rebase / history rewrite.
#
# Usage:
#   scripts/bisect_verify.sh                # verify every commit on HEAD
#   scripts/bisect_verify.sh -n 2           # verify only the last 2 commits
#   scripts/bisect_verify.sh -s <sha>       # verify every commit AFTER <sha>
#                                           # (i.e. <sha>..HEAD; handy for
#                                           # "since last pushed/known-good")
#
# Exactly one of -n / -s may be given; with neither, all commits are checked.
set -e

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

IDF_EXPORT="${IDF_EXPORT:-/Users/facosta/git/espressif/esp-idf-v6.0.1/export.sh}"
SSL_CERT_FILE_DEFAULT="/Library/Frameworks/Python.framework/Versions/3.14/lib/python3.14/site-packages/certifi/cacert.pem"
SSL_CERT_FILE="${SSL_CERT_FILE:-$SSL_CERT_FILE_DEFAULT}"

N=""
SINCE=""

usage() {
    sed -n '2,14p' "$0"
    exit "${1:-0}"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -n) N="$2"; shift 2 ;;
        -s) SINCE="$2"; shift 2 ;;
        -h|--help) usage 0 ;;
        *) echo "Unknown arg: $1" >&2; usage 1 ;;
    esac
done

if [[ -n "$N" && -n "$SINCE" ]]; then
    echo "Error: -n and -s are mutually exclusive" >&2
    exit 1
fi

# Build the list of commits to verify, oldest-first.
if [[ -n "$N" ]]; then
    if ! [[ "$N" =~ ^[0-9]+$ ]] || (( N < 1 )); then
        echo "Error: -n must be a positive integer" >&2
        exit 1
    fi
    RANGE_DESC="last $N commit(s)"
    SHAS=$(git log --reverse --format=%H -n "$N" HEAD)
elif [[ -n "$SINCE" ]]; then
    if ! git rev-parse --verify --quiet "${SINCE}^{commit}" >/dev/null; then
        echo "Error: -s '$SINCE' is not a valid commit" >&2
        exit 1
    fi
    RESOLVED=$(git rev-parse --short "$SINCE")
    RANGE_DESC="commits after $RESOLVED (${SINCE}..HEAD)"
    SHAS=$(git log --reverse --format=%H "${SINCE}..HEAD")
else
    RANGE_DESC="all commits on HEAD"
    SHAS=$(git log --reverse --format=%H HEAD)
fi

if [[ -z "$SHAS" ]]; then
    echo "No commits to verify ($RANGE_DESC)."
    exit 0
fi

COUNT=$(echo "$SHAS" | wc -l | tr -d ' ')
echo "Verifying $COUNT commit(s): $RANGE_DESC"
echo

ORIG=$(git rev-parse --abbrev-ref HEAD)
if [[ "$ORIG" == "HEAD" ]]; then
    ORIG=$(git rev-parse HEAD)
fi

idf_build() {
    env -i HOME="$HOME" PATH="/opt/homebrew/bin:/usr/bin:/bin:/usr/sbin:/sbin" \
        SHELL="$SHELL" TERM="$TERM" SSL_CERT_FILE="$SSL_CERT_FILE" \
        bash -c ". '$IDF_EXPORT' >/dev/null && idf.py build" \
        > /tmp/bisect_idf.log 2>&1
}

host_build_test() {
    rm -rf host_tests/build
    cmake -S host_tests -B host_tests/build -G Ninja > /tmp/bisect_host_cfg.log 2>&1
    cmake --build host_tests/build > /tmp/bisect_host_build.log 2>&1
    ctest --test-dir host_tests/build --output-on-failure > /tmp/bisect_host_test.log 2>&1
}

restore() {
    git checkout -q "$ORIG"
}
trap restore EXIT

for sha in $SHAS; do
    short=$(git rev-parse --short "$sha")
    subj=$(git log -1 --format=%s "$sha")
    echo "=== $short  $subj ==="
    git checkout -q "$sha"
    rm -rf build

    echo -n "  idf.py build: "
    if idf_build; then echo "OK"; else echo "FAIL"; tail -30 /tmp/bisect_idf.log; exit 1; fi

    if [[ -f host_tests/CMakeLists.txt ]]; then
        echo -n "  host cmake+ctest: "
        if host_build_test; then
            passed=$(grep -E "tests passed" /tmp/bisect_host_test.log | tail -1)
            echo "OK ($passed)"
        else
            echo "FAIL"
            tail -30 /tmp/bisect_host_build.log /tmp/bisect_host_test.log
            exit 1
        fi
    else
        echo "  host cmake+ctest: SKIP (no host_tests/)"
    fi
done

echo
echo "=== ALL $COUNT COMMIT(S) GREEN ==="
