import RoverNav

public struct SectorPathPolicy: PathAdmissibilityPolicy {
    public let sector: SearchSector
    public let frame: SharedMissionFrame

    public init(sector: SearchSector, frame: SharedMissionFrame) {
        self.sector = sector
        self.frame = frame
    }

    public func evaluate(path: [Vec2]) -> PathAdmissibilityResult {
        for (index, localPoint) in path.enumerated() {
            guard localPoint.x.isFinite,
                  localPoint.y.isFinite,
                  let missionPoint = frame.missionPoint(from: localPoint) else {
                return .rejected(.invalidPoint(pointIndex: index))
            }

            let allowed = switch sector {
            case .west: missionPoint.x < -SilentSearchGeometry.centerBandHalfWidth
            case .east: missionPoint.x > SilentSearchGeometry.centerBandHalfWidth
            }
            guard allowed else {
                return .rejected(.outsideSector(pointIndex: index))
            }
        }
        return .admissible
    }
}
