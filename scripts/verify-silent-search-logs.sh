#!/bin/sh
set -eu

if [ "$#" -ne 2 ]; then
    printf '%s\n' 'usage: verify-silent-search-logs.sh ROVER_A_LOG ROVER_B_LOG' >&2
    exit 2
fi

LOG_A=$1
LOG_B=$2

for file in "$LOG_A" "$LOG_B"; do
    if [ ! -r "$file" ] || [ ! -f "$file" ]; then
        printf 'cannot read log: %s\n' "$file" >&2
        exit 2
    fi
done

if ! awk '
    BEGIN { bad = 0 }
    {
        line = tolower($0)
        if (line ~ /(^|[[:space:]])[^[:space:]=]*(payload|image)[^[:space:]=]*=/ ||
            line ~ /(^|[[:space:]])(pixel_buffer|frame_contents|detections)=/ ||
            line ~ /phrover-cal\|/ || line ~ /roverteamradio|team_radio|team-radio|cloudsession|mqtt|aws[_.-]?iot/) {
            bad = 1
        }
    }
    END { exit bad ? 1 : 0 }
' "$LOG_A" "$LOG_B"; then
    printf '%s\n' 'logs contain prohibited payload, image, cloud, or team-radio data' >&2
    exit 1
fi

context() {
    awk '
        $2 == "silent_search_mission" {
            mission = marker = role = ""
            for (i = 3; i <= NF; i++) {
                split($i, pair, "=")
                if (pair[1] == "mission") mission = pair[2]
                if (pair[1] == "marker") marker = pair[2]
                if (pair[1] == "role") role = pair[2]
            }
            if (mission != "" && marker != "" && role != "") {
                print mission, marker, role
                exit
            }
        }
    ' "$1"
}

set -- $(context "$LOG_A")
if [ "$#" -ne 3 ]; then
    printf '%s\n' 'rover A log has no mission context' >&2
    exit 1
fi
a_mission=$1
a_marker=$2
a_role=$3

set -- $(context "$LOG_B")
if [ "$#" -ne 3 ]; then
    printf '%s\n' 'rover B log has no mission context' >&2
    exit 1
fi
b_mission=$1
b_marker=$2
b_role=$3

if [ "$a_mission" != "$b_mission" ] || [ "$a_marker" != "$b_marker" ] ||
   [ "$a_role" != "a" ] || [ "$b_role" != "b" ]; then
    printf '%s\n' 'mission, marker, or complementary role validation failed' >&2
    exit 1
fi

if ! awk '
    function field(name,    i, pair) {
        for (i = 3; i <= NF; i++) {
            split($i, pair, "=")
            if (pair[1] == name) return pair[2]
        }
        return ""
    }
    $2 == "silent_search_calibration_accepted" { calibrated = 1 }
    $2 == "silent_search_convergence" && field("stage") == "arrived" { converged = 1 }
    $2 == "silent_search_terminal" && field("result") == "success" { terminal = 1 }
    END { exit calibrated && converged && terminal ? 0 : 1 }
' "$LOG_A" || ! awk '
    function field(name,    i, pair) {
        for (i = 3; i <= NF; i++) {
            split($i, pair, "=")
            if (pair[1] == name) return pair[2]
        }
        return ""
    }
    $2 == "silent_search_calibration_accepted" { calibrated = 1 }
    $2 == "silent_search_convergence" && field("stage") == "arrived" { converged = 1 }
    $2 == "silent_search_terminal" && field("result") == "success" { terminal = 1 }
    END { exit calibrated && converged && terminal ? 0 : 1 }
' "$LOG_B"; then
    printf '%s\n' 'calibration, convergence, or terminal result is incomplete' >&2
    exit 1
fi

if ! awk '$2 == "silent_search_target_evidence" || $2 == "silent_search_target_confirmed" { found = 1 }
         END { exit found ? 0 : 1 }' "$LOG_A" "$LOG_B"; then
    printf '%s\n' 'target evidence is missing' >&2
    exit 1
fi

protocol_a=$(
    awk '
        function field(name,    i, pair) {
            for (i = 3; i <= NF; i++) {
                split($i, pair, "=")
                if (pair[1] == name) return pair[2]
            }
            return ""
        }
        $2 == "silent_search_protocol" && field("direction") == "outgoing" && field("outcome") == "accepted" {
            print field("kind"), field("sequence")
        }
    ' "$LOG_A"
)
protocol_b=$(
    awk '
        function field(name,    i, pair) {
            for (i = 3; i <= NF; i++) {
                split($i, pair, "=")
                if (pair[1] == name) return pair[2]
            }
            return ""
        }
        $2 == "silent_search_protocol" && field("direction") == "outgoing" && field("outcome") == "accepted" {
            print field("kind"), field("sequence")
        }
    ' "$LOG_B"
)
expected_a=$(printf '%s\n' 'offer 1' 'searchCommit 2' 'status 3' 'decision 4' 'converge 5')
expected_b=$(printf '%s\n' 'accept 1' 'searchAck 2' 'status 3' 'convergeAck 4')
if [ "$protocol_a" != "$expected_a" ] || [ "$protocol_b" != "$expected_b" ]; then
    printf '%s\n' 'optical protocol order is incomplete or invalid' >&2
    exit 1
fi

printf '%s\n' 'silent-search logs valid'
