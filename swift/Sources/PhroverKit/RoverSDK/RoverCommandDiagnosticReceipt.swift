import Foundation

/// Transport acknowledgement is not a measurement of wheel motion or braking.
struct RoverCommandDiagnosticReceipt: Sendable {
    let httpStatus: Int?
    let acknowledged: Bool?
    let acknowledgementUTC: Date?
    let attempts: Int?
    let outcome: String

    static let unknown = Self(httpStatus: nil, acknowledged: nil,
        acknowledgementUTC: nil, attempts: nil, outcome: "unknown")
}

struct RoverCommandDiagnosticResult: Sendable {
    let receipt: RoverCommandDiagnosticReceipt
    let failure: (any Error)?

    func get() throws -> RoverCommandDiagnosticReceipt {
        if let failure { throw failure }
        return receipt
    }
}
