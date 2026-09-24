import Foundation

/// Connection + tuning defaults for the WAVE ROVER.
public enum RoverConfig {
    /// Default host of the WAVE ROVER ESP32 web server.
    /// - AP mode (rover as its own hotspot): 192.168.4.1
    /// - STA mode (rover joined building WiFi): set to the DHCP address / Bonjour name.
    /// - e2e testing (no chassis): set E2E_ROVER_HOST to a mock ESP32's host:port.
    public static let defaultHost = ProcessInfo.processInfo.environment["E2E_ROVER_HOST"] ?? "192.168.4.1"

    /// The ESP32 firmware exposes JSON control at `GET /js?json=<url-encoded JSON>`.
    public static let jsonCommandPath = "/js"

    /// WAVE ROVER JSON command "T" opcodes we use (see Waveshare sub-controller command set).
    public enum Opcode {
        public static let speedControl = 1     // {"T":1,"L":<m/s>,"R":<m/s>}
        public static let emergencyStop = 0    // {"T":0} — stop all motors
        public static let feedbackFlowOn = 131 // continuous chassis+IMU feedback
        public static let imuQuery = 126       // one-shot IMU read
    }

    // MARK: - Physical parameters (WAVE ROVER)
    public static let wheelBase = 0.13         // m, track width (left↔right)
    public static let maxWheelSpeed = 0.5      // m/s per Waveshare closed-loop range on encoder bases; base rover is open-loop
    /// Minimum in-place turn command that reliably overcomes WAVE ROVER static friction.
    /// The autonomous rotate loop may compute a smaller speed from yaw error, but values
    /// below this can make interpreted voice turns appear to do nothing.
    public static let minimumRotateWheelSpeed = 0.25
    /// Search turns pulse the motors instead of spinning continuously. The pause gives
    /// ARKit pose and Core ML detection a stable camera frame between heading changes.
    public static let scanTurnPulseDuration: TimeInterval = 0.08
    public static let scanTurnSettleDuration: TimeInterval = 0.30
    public static let scanTurnYawTolerance = 7.0 * Double.pi / 180.0
    /// Stand-off distance for a confidently locked visual target.
    public static let visualTargetStopDistance = 0.30
    /// Brake before the desired stand-off to compensate for command latency and chassis coast.
    public static let visualTargetBrakeLeadDistance = 0.10
    /// Begin slowing the final approach early enough to avoid coasting through the stop distance.
    public static let visualTargetSlowdownDistance = 0.60
    /// Final-approach wheel cap; at the 10 Hz command rate this advances about 1.2 cm per tick.
    public static let visualTargetApproachMaxWheelSpeed = 0.12
    /// Only relax the normal obstacle threshold when the projected target is nearby.
    public static let visualTargetApproachDistance = 1.20

    // MARK: - Follow mode
    /// A followed target *is* the forward obstacle, so the clearance stop is relaxed only
    /// while the cone reading agrees with the target's own range to within this margin.
    /// Anything else entering the cone still stops the rover.
    public static let followClearanceMatchTolerance = 0.30
    /// Give up if nothing matching the request is seen within this window.
    public static let followAcquireTimeout: TimeInterval = 8
    /// Hard budget on one follow leg. Following is open-ended by nature; without a ceiling
    /// it runs until the battery decides (see adr/0011-time-budget-rule.md).
    public static let maxFollowDuration: TimeInterval = 300
    /// Scan pulses attempted toward the last known bearing before declaring the target lost.
    public static let followSearchPulses = 12

    // MARK: - Safety
    /// If no successful command round-trip within this window, ObstacleGuard forces a stop.
    /// Keep this above brief planning/logging gaps; individual rover sends still retry and fail fast.
    public static let commsWatchdogTimeout: TimeInterval = 2.0
    /// Drive-command resend period; keeps the base moving and doubles as a heartbeat.
    public static let commandInterval: TimeInterval = 0.1
    /// Total HTTP attempts for a command when the rover WiFi link drops a packet.
    public static let commandRetryAttempts = 3
    /// Short retry delay between transient rover WiFi failures.
    public static let commandRetryBackoff: TimeInterval = 0.15
}
