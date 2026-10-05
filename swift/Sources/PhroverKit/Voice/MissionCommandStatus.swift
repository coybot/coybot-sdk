public typealias MissionCommandID = UInt64

public enum MissionCommandStatus: Equatable, Sendable {
    case recognized(id: MissionCommandID, command: String)
    case working(id: MissionCommandID, command: String)
    case succeeded(id: MissionCommandID, command: String, message: String)
    case failed(id: MissionCommandID, command: String, message: String)
    case cancelled(id: MissionCommandID, command: String)

    public var id: MissionCommandID {
        switch self {
        case .recognized(let id, _),
             .working(let id, _),
             .succeeded(let id, _, _),
             .failed(let id, _, _),
             .cancelled(let id, _):
            id
        }
    }

    public var isTerminal: Bool {
        switch self {
        case .recognized, .working: false
        case .succeeded, .failed, .cancelled: true
        }
    }
}
