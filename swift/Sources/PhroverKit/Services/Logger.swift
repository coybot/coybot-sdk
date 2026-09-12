import Foundation
import OSLog

/// Centralized logging with OSLog. Set `AppLogger.subsystem` once at launch (typically to
/// your app's bundle id) — defaults to the bundled example app's id if you don't.
public enum AppLogger {
    /// Set once at launch, before the first log call — not synchronized for concurrent writes.
    nonisolated(unsafe) public static var subsystem = "bot.coy.phrover"

    public static var auth: Logger { Logger(subsystem: subsystem, category: "Auth") }
    public static var nav: Logger { Logger(subsystem: subsystem, category: "Nav") }
    public static var voice: Logger { Logger(subsystem: subsystem, category: "Voice") }
    public static var ui: Logger { Logger(subsystem: subsystem, category: "UI") }
}
