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
        public static let speedControl = 1     // L/R open-loop wheel commands; both zero stops the chassis
        /// Retained for source compatibility only; stock WAVE ROVER does not
        /// implement T:0 as a chassis stop.
        @available(*, deprecated, message: "Use RoverControl.stop(); WAVE ROVER requires T:1 with L:0 and R:0")
        public static let emergencyStop = 0
        public static let feedbackFlowOn = 131 // continuous chassis+IMU feedback
        public static let imuQuery = 126       // one-shot IMU read
    }

    // MARK: - Physical parameters (WAVE ROVER)
    public static let wheelBase = 0.13         // m, track width (left↔right)
    public static let maxWheelSpeed = 0.5      // m/s per Waveshare closed-loop range on encoder bases; base rover is open-loop
    // User-reported mount: lens approximately 30 cm high and 7–10 cm
    // behind the axle midpoint. Use the provisional midpoint of that range.
    static let cameraMountCalibration = CameraMountCalibration(
        cameraHeight: 0.30,
        forwardOffset: -0.085,
        lateralOffset: 0,
        headingAlignment: 0
    )
    // Conservative box until chassis/holder dimensions are measured. Height
    // sections can then model the narrow upper assembly without shrinking the
    // chassis clearance or averaging away an unseen component.
    static let collisionGeometry = RoverCollisionGeometry(
        length: 0.30,
        width: 0.25,
        minimumCollisionHeight: 0.04,
        maximumCollisionHeight: 0.50,
        lateralSafetyMargin: 0.08
    )
    /// Follow-me depth gate. The WAVE ROVER mount cannot see enough of the near
    /// motion volume to pass `.fullVolume` with the conservative box above, so
    /// follow drops the visibility requirement, as Astral did, until measured
    /// height sections make `.fullVolume` passable. See docs/follow-depth-policy.md.
    static let followDepthPolicy: FollowDepthPolicy = .astralCompatible
    /// Minimum in-place turn command that reliably overcomes WAVE ROVER static friction.
    /// The autonomous rotate loop may compute a smaller speed from yaw error, but values
    /// below this can make interpreted voice turns appear to do nothing.
    public static let minimumRotateWheelSpeed = 0.25
    /// Open-loop wheel output for the short follow readiness translation.
    static let readySignalWheelMagnitude = 0.25
    static let readySignalPulseBudget: TimeInterval = 0.040
    static let readySignalStoppedObservation: TimeInterval = 0.100
    /// Host-time allowance to drain a readiness pulse and confirm its stop.
    static let readySignalStopReserve: TimeInterval = 0.100
    /// Read-only confirmation after the five-second movement cutoff.
    static let readySignalFinalConfirmation: TimeInterval = 0.300
    public static let relativeHeadingUpdateInterval: TimeInterval = 1.0 / 60.0
    public static let relativeHeadingMaximumIntegrationGap: TimeInterval = 0.10
    public static let relativeHeadingMaximumSampleAge: TimeInterval = 0.15
    public static let relativeHeadingAcceptedGravityMagnitude = 0.8...1.2
    /// Search turns pulse the motors instead of spinning continuously. The pause gives
    /// ARKit pose and Core ML detection a stable camera frame between heading changes.
    public static let scanTurnPulseDuration: TimeInterval = 0.02
    public static let scanTurnSettleDuration: TimeInterval = 0.75
    public static let scanTurnYawTolerance = 7.0 * Double.pi / 180.0
    /// Follow-only tuning; legacy and Silent Search scan timing remains independent.
    static let followScanRotationProfile = FollowScanRotationProfile(
        pulseWait: 0.200, settleWait: 0.300, wheelCap: minimumRotateWheelSpeed, yawGain: 0.30,
        angularTolerance: scanTurnYawTolerance)
    public static let scanMinimumProgressYaw = 2.0 * Double.pi / 180.0
    public static let scanFramePollInterval: TimeInterval = 0.05
    public static let scanFrameFreshnessTimeout: TimeInterval = 1.5
    public static let scanDepthRecoveryPollInterval: TimeInterval = 0.05
    public static let scanDepthRecoveryTimeout: TimeInterval = 0.75
    public static let scanPoseJumpTranslation = 0.10
    public static let scanHeadingAgreementTolerance = 15.0 * Double.pi / 180.0
    public static let scanTurnMaxPulseCount = 8
    /// Upper bound for a scan that consumes every pulse and waits for a fresh frame each time.
    public static let maximumScanTurnDuration =
        Double(scanTurnMaxPulseCount)
        * (scanTurnPulseDuration + scanTurnSettleDuration + scanFrameFreshnessTimeout)
    /// Mission recovery must not cancel a scan before the navigation controller's own bound.
    public static let blockedHeadingRecoveryTimeout = maximumScanTurnDuration + 2.0
    public static let navigationTrackingFreshness: TimeInterval = 0.5
    public static let navigationTrackingPollInterval: TimeInterval = 0.05
    public static let navigationTrackingRecoveryTimeout: TimeInterval = 3.0
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
    /// Increase target stand-off gradually when scene-mesh inflation blocks the closest
    /// approach point. This keeps the rover moving toward a visible target without asking
    /// the planner to enter the object's occupied footprint.
    public static let visualTargetStandOffIncrement = 0.15
    /// When the complete route to a visible target is not connected in the local costmap,
    /// advance only this far before collecting a fresh depth frame and planning again.
    public static let visualTargetIncrementalStepDistance = 0.75
    /// Shortest useful incremental move toward a visible target.
    public static let visualTargetMinimumIncrementalStepDistance = 0.30
    /// Reduce a rejected incremental step by this amount until a reachable step is found.
    public static let visualTargetIncrementalStepDecrement = 0.15
    /// Bound visual ray advances while still allowing a target near the edge of the 6 m map.
    public static let visualTargetMaximumIncrementalSteps = 8

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
