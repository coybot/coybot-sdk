#!/usr/bin/env bash
set -euo pipefail

script_name="${0##*/}"

usage() {
  cat <<EOF
Usage:
  ${script_name} postgres
  ${script_name} sqlserver
  ${script_name} mongo
  ${script_name} off
  ${script_name} status

Required env vars:
  postgres  -> DBHUB_POSTGRES_DSN
  sqlserver -> DBHUB_SQLSERVER_DSN
  mongo     -> MDB_MCP_CONNECTION_STRING

Optional config path overrides:
  OPENCODE_CONFIG defaults to ~/.config/opencode/opencode.json
  CODEX_CONFIG    defaults to ~/.codex/config.toml

Examples:
  export DBHUB_POSTGRES_DSN='postgres://readonly_user:password@host:5432/db?sslmode=require'
  ${script_name} postgres
  opencode --agent review

  export MDB_MCP_CONNECTION_STRING='mongodb+srv://readonly_user:password@cluster/db'
  ${script_name} mongo
  claude
EOF
}

target="${1:-}"
case "$target" in
  postgres|sqlserver|mongo|off|status) ;;
  -h|--help|"")
    usage
    exit 0
    ;;
  *)
    echo "Unknown target: $target" >&2
    usage >&2
    exit 2
    ;;
esac

required_env=""
case "$target" in
  postgres) required_env="DBHUB_POSTGRES_DSN" ;;
  sqlserver) required_env="DBHUB_SQLSERVER_DSN" ;;
  mongo) required_env="MDB_MCP_CONNECTION_STRING" ;;
esac

if [[ -n "$required_env" && -z "${!required_env:-}" ]]; then
  echo "$required_env is required before enabling $target." >&2
  echo "Set it in this shell, then rerun this script." >&2
  exit 1
fi

if ! command -v node >/dev/null 2>&1; then
  echo "node is required but was not found in PATH." >&2
  exit 1
fi

node - "$target" <<'NODE'
const fs = require("fs");
const os = require("os");
const path = require("path");

const target = process.argv[2];
const home = os.homedir();
const servers = ["dbhub-postgres", "dbhub-sqlserver", "mongodb"];
const selectedByTarget = {
  postgres: "dbhub-postgres",
  sqlserver: "dbhub-sqlserver",
  mongo: "mongodb",
};
const selected = selectedByTarget[target] || null;

const opencodePath = expandHome(process.env.OPENCODE_CONFIG || "~/.config/opencode/opencode.json");
const codexPath = expandHome(process.env.CODEX_CONFIG || "~/.codex/config.toml");

function expandHome(value) {
  if (value === "~") return home;
  if (value.startsWith("~/")) return path.join(home, value.slice(2));
  return value;
}

function backupPath(file) {
  const stamp = new Date().toISOString().replace(/[:.]/g, "-");
  return `${file}.bak-${stamp}`;
}

function atomicWrite(file, content) {
  const dir = path.dirname(file);
  const tmp = path.join(dir, `.${path.basename(file)}.${process.pid}.tmp`);
  fs.writeFileSync(tmp, content, { mode: 0o600 });
  fs.renameSync(tmp, file);
}

function writeWithBackup(file, content) {
  fs.copyFileSync(file, backupPath(file));
  atomicWrite(file, content);
}

function readText(file) {
  try {
    return fs.readFileSync(file, "utf8");
  } catch (error) {
    throw new Error(`Cannot read ${file}: ${error.message}`);
  }
}

function setOpenCode() {
  if (!fs.existsSync(opencodePath)) return null;
  const config = JSON.parse(readText(opencodePath));
  config.mcp ||= {};
  for (const name of servers) {
    if (config.mcp[name]) config.mcp[name].enabled = selected === name;
  }
  if (target === "off") {
    for (const name of servers) {
      if (config.mcp[name]) config.mcp[name].enabled = false;
    }
  }
  if (target !== "status") {
    writeWithBackup(opencodePath, JSON.stringify(config, null, 2) + "\n");
  }
  return Object.fromEntries(servers.map((name) => [name, Boolean(config.mcp?.[name]?.enabled)]));
}

function escapeRegExp(value) {
  return value.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
}

function setCodex() {
  if (!fs.existsSync(codexPath)) return null;
  let text = readText(codexPath);
  const statuses = {};

  for (const name of servers) {
    const enable = target !== "off" && selected === name;
    const header = `[mcp_servers.${name}]`;
    const re = new RegExp(`(${escapeRegExp(header)}\\n[\\s\\S]*?)(?=\\n\\[mcp_servers\\.|\\n\\[[^\\n]+\\]|\\n# END database review MCP servers|$)`);
    const match = text.match(re);
    if (!match) {
      statuses[name] = false;
      continue;
    }
    let block = match[1];
    if (/^enabled\s*=/m.test(block)) {
      block = block.replace(/^enabled\s*=.*$/m, `enabled = ${enable}`);
    } else {
      block = block.trimEnd() + `\nenabled = ${enable}\n`;
    }
    text = text.slice(0, match.index) + block + text.slice(match.index + match[1].length);
    statuses[name] = enable;
  }

  if (target !== "status") {
    writeWithBackup(codexPath, text);
  }
  return statuses;
}

let openCode;
let codex;
try {
  openCode = setOpenCode();
  codex = setCodex();
} catch (error) {
  console.error(error.message);
  process.exit(1);
}

function printStatus(label, statuses) {
  if (!statuses) return;
  console.log(`${label}:`);
  for (const name of servers) {
    console.log(`  ${name}: ${statuses[name] ? "enabled" : "disabled"}`);
  }
}

printStatus("OpenCode", openCode);
printStatus("Codex", codex);

if (!openCode && !codex) {
  console.error(`No config files found. Checked ${opencodePath} and ${codexPath}.`);
  process.exit(1);
}

if (target !== "status") {
  console.log("");
  if (target === "off") {
    console.log("Database MCP servers disabled for OpenCode and Codex.");
  } else {
    console.log(`Enabled ${selected} for OpenCode and Codex.`);
    console.log("Claude Code/Desktop already have these MCP servers registered.");
    console.log("Start Claude from this same shell so it inherits the env var.");
  }
}
NODE
