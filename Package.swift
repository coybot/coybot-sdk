// swift-tools-version: 6.0
import PackageDescription

// Phrover — the phone-brained WAVE ROVER. Three layered products:
//
//   RoverNav      pure Foundation planning/control core
//   PhroverKit    the brain: perception, nav orchestration, voice, ESP32 comms
//   PhroverCloud  mobile-side cloud client — MQTT telemetry, auth, dialog escalation
//
// PhroverCloud is split out so PhroverKit stays dependency-light: bring your own backend and
// only depend on RoverNav + PhroverKit, or take PhroverCloud for the reference AWS IoT/Cognito
// integration. Only the server side (fleet orchestration, video ingestion, pilot routing) is
// closed — see README.md.
//
// Platform floor: SwiftPM has no per-target platform override, so the package-wide iOS
// minimum is pinned to what PhroverKit's on-device Apple Foundation Model support actually
// requires (iOS 26), even though RoverNav's own code would run on much older iOS. macOS is
// declared only for RoverNav, which is pure Foundation and buildable/testable standalone
// there (`swift build --target RoverNav`) — PhroverKit/PhroverCloud import ARKit, which
// doesn't exist on macOS, so the package as a whole (and its full test suite, including
// RoverNavTests) needs an iOS destination:
//
//   xcodebuild test -scheme coybot-sdk-Package -destination 'platform=iOS Simulator,name=...'
let package = Package(
    name: "coybot-sdk",
    platforms: [.iOS("26.0"), .macOS(.v14)],
    products: [
        .library(name: "RoverNav", targets: ["RoverNav"]),
        .library(name: "PhroverKit", targets: ["PhroverKit"]),
        .library(name: "PhroverCloud", targets: ["PhroverCloud"]),
    ],
    dependencies: [
        .package(url: "https://github.com/aws-amplify/aws-sdk-ios-spm", from: "2.33.0"),
    ],
    targets: [
        .target(name: "RoverNav", path: "swift/Sources/RoverNav"),
        .testTarget(name: "RoverNavTests", dependencies: ["RoverNav"], path: "swift/Tests/RoverNavTests"),

        .target(
            name: "PhroverKit",
            dependencies: ["RoverNav"],
            path: "swift/Sources/PhroverKit",
            resources: [.copy("Resources/RoverYOLO.mlpackage")]
        ),
        .testTarget(name: "PhroverKitTests", dependencies: ["PhroverKit"], path: "swift/Tests/PhroverKitTests"),

        // Deliberately separate from PhroverKitTests: makes real, slow, non-deterministic
        // model calls — the on-device Foundation Model (needs Apple Intelligence ready on
        // the host Mac/device), and, via PhroverCloud's CloudBrain pointed at a local
        // bridge (eco/e2e/harness/live_rover_act_bridge.py), real billed Bedrock calls.
        // eco/e2e/harness/phrover.py's fast gate only passes
        // `-only-testing:RoverNavTests -only-testing:PhroverKitTests`, so this target never
        // runs there — invoke explicitly with `-only-testing:PhroverKitLiveProbes`, or via
        // eco/e2e/run_live_mission.sh for the cloud-brain mission.
        .testTarget(name: "PhroverKitLiveProbes",
                    dependencies: ["PhroverKit", "PhroverCloud"],
                    path: "swift/Tests/PhroverKitLiveProbes"),

        .target(
            name: "PhroverCloud",
            dependencies: [
                "PhroverKit",
                .product(name: "AWSCore", package: "aws-sdk-ios-spm"),
                .product(name: "AWSIoT", package: "aws-sdk-ios-spm"),
            ],
            path: "swift/Sources/PhroverCloud"
        ),

        // Drives the real MissionAgent against the Godot Depot sim (eco/drone/sim/godot/,
        // env_depot.gd + phrover_manager.gd) over its TCP IPC — see
        // eco/rover/sim/depot_client.py for the Python-side counterpart and
        // eco/.claude-plans or the design doc for the op/response shapes. Needs a Godot
        // process already running (GODOT_IPC env, default 127.0.0.1:9999); the harness
        // (eco/rover/sim/depot_harness.py) is responsible for launching it. Separate from
        // PhroverKitLiveProbes: this exercises the sim seams (RoverMotion/RoverPerception/
        // RoverVoice), not the real ARKit/cloud stack.
        .testTarget(name: "PhroverSimTests",
                    dependencies: ["PhroverKit", "PhroverCloud", "RoverNav"],
                    path: "swift/Tests/PhroverSimTests"),
    ]
)
