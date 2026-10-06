#!/bin/sh
set -eu

ROOT=$(CDPATH= cd -- "$(dirname "$0")/../.." && pwd)
CHECK="$ROOT/scripts/verify-silent-search-logs.sh"
FIXTURES="$ROOT/scripts/tests/fixtures/silent-search"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

run_and_capture() {
    set +e
    "$@" >"$TMP/stdout" 2>"$TMP/stderr"
    status=$?
    set -e
}

run_and_capture "$CHECK"
[ "$status" -eq 2 ] || { printf 'usage must exit 2, got %s\n' "$status" >&2; exit 1; }

run_and_capture "$CHECK" "$TMP/not-found" "$FIXTURES/success-b.log"
[ "$status" -eq 2 ] || { printf 'read errors must exit 2, got %s\n' "$status" >&2; exit 1; }

run_and_capture "$CHECK" "$FIXTURES/success-a.log" "$FIXTURES/missing-ack.log"
[ "$status" -eq 1 ] || { printf 'missing acknowledgement must exit 1, got %s\n' "$status" >&2; exit 1; }

run_and_capture "$CHECK" "$FIXTURES/prohibited-payload.log" "$FIXTURES/success-b.log"
[ "$status" -eq 1 ] || { printf 'prohibited fields must exit 1, got %s\n' "$status" >&2; exit 1; }

run_and_capture "$CHECK" "$FIXTURES/success-a.log" "$FIXTURES/success-b.log"
[ "$status" -eq 0 ] || { printf 'success fixtures must exit 0, got %s: %s\n' "$status" "$(cat "$TMP/stderr")" >&2; exit 1; }
[ "$(cat "$TMP/stdout")" = "silent-search logs valid" ] || {
    printf 'unexpected success output: %s\n' "$(cat "$TMP/stdout")" >&2
    exit 1
}
[ ! -s "$TMP/stderr" ] || { printf 'success wrote stderr\n' >&2; exit 1; }

printf '%s\n' 'test-verify-silent-search-logs: passed'
