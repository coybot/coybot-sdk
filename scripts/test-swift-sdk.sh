#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PRINT_UDID=false

if [[ "${1:-}" == "--print-udid" ]]; then
  PRINT_UDID=true
  shift
fi

if [[ -z "${SIM_UDID:-}" ]]; then
  SIM_UDID="$(
    xcrun simctl list devices available -j | python3 -c '
import json
import re
import sys

devices_by_runtime = json.load(sys.stdin)["devices"]
for runtime, devices in devices_by_runtime.items():
    match = re.search(r"iOS-(\d+)(?:-(\d+))?", runtime)
    if match is None or int(match.group(1)) < 26:
        continue
    for device in devices:
        if device.get("isAvailable") and "iPhone" in device.get("deviceTypeIdentifier", ""):
            print(device["udid"])
            raise SystemExit(0)
raise SystemExit("No available iOS 26+ iPhone simulator found")
'
  )"
fi

if [[ "$PRINT_UDID" == true ]]; then
  printf '%s\n' "$SIM_UDID"
  exit 0
fi

XCODEBUILD_ARGS=(
  test
  -project "$ROOT_DIR/examples/PhroverOperator/PhroverOperator.xcodeproj"
  -scheme PhroverSDKTests
  -destination "platform=iOS Simulator,id=$SIM_UDID"
)
if [[ " $* " != *" -only-testing:"* ]]; then
  XCODEBUILD_ARGS+=(
    -only-testing:RoverNavTests
    -only-testing:PhroverKitTests
    -only-testing:PhroverCloudTests
  )
fi
XCODEBUILD_ARGS+=("$@")

exec xcodebuild "${XCODEBUILD_ARGS[@]}"
