#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
script="${script_dir}/db-mcp.sh"
tmpdir="$(mktemp -d "${TMPDIR:-/tmp}/db-mcp-test.XXXXXX")"
trap 'rm -rf "$tmpdir"' EXIT

export OPENCODE_CONFIG="${tmpdir}/opencode.json"
export CODEX_CONFIG="${tmpdir}/config.toml"

cat >"$OPENCODE_CONFIG" <<'JSON'
{
  "mcp": {
    "dbhub-postgres": { "enabled": false },
    "dbhub-sqlserver": { "enabled": false },
    "mongodb": { "enabled": false }
  }
}
JSON

cat >"$CODEX_CONFIG" <<'TOML'
[mcp_servers.dbhub-postgres]
command = "sh"
enabled = false

[mcp_servers.dbhub-sqlserver]
command = "sh"
enabled = false

[mcp_servers.mongodb]
command = "sh"
enabled = false
TOML

run_from_tmp() {
  (cd /private/tmp && "$script" "$@")
}

assert_contains() {
  local expected="$1"
  local actual="$2"
  if [[ "$actual" != *"$expected"* ]]; then
    echo "Expected output to contain: $expected" >&2
    echo "Actual output:" >&2
    echo "$actual" >&2
    exit 1
  fi
}

if run_from_tmp postgres >/tmp/db-mcp-test.out 2>&1; then
  echo "postgres without DSN should fail" >&2
  exit 1
fi
assert_contains "DBHUB_POSTGRES_DSN is required" "$(cat /tmp/db-mcp-test.out)"

export DBHUB_POSTGRES_DSN="postgres://readonly:secret@example.test:5432/app"
postgres_output="$(run_from_tmp postgres)"
assert_contains "dbhub-postgres: enabled" "$postgres_output"
assert_contains "dbhub-sqlserver: disabled" "$postgres_output"
assert_contains "mongodb: disabled" "$postgres_output"

export MDB_MCP_CONNECTION_STRING="mongodb://readonly:secret@example.test/app"
mongo_output="$(run_from_tmp mongo)"
assert_contains "dbhub-postgres: disabled" "$mongo_output"
assert_contains "mongodb: enabled" "$mongo_output"

off_output="$(run_from_tmp off)"
assert_contains "dbhub-postgres: disabled" "$off_output"
assert_contains "dbhub-sqlserver: disabled" "$off_output"
assert_contains "mongodb: disabled" "$off_output"

status_output="$(run_from_tmp status)"
assert_contains "OpenCode:" "$status_output"
assert_contains "Codex:" "$status_output"

echo "db-mcp self-test passed"
