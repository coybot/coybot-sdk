import CoreGraphics
import CoreImage
import CoreImage.CIFilterBuiltins
import CoreVideo
import Foundation
import ImageIO
import RoverNav
import Vision

public enum OpticalExchangeError: Error, Equatable, Sendable {
    case invalidPayload
    case invalidScale
    case renderingFailed
    case invalidTimestamp
}

public struct OpticalFrame: @unchecked Sendable {
    public let frameID: UInt64
    public let arFrameID: ARFrameID?
    public let monotonicTimestamp: TimeInterval
    fileprivate let image: Image

    fileprivate enum Image {
        case cgImage(CGImage)
        case pixelBuffer(CVPixelBuffer)
    }

    public init(image: CGImage, frameID: UInt64, monotonicTimestamp: TimeInterval) {
        self.image = .cgImage(image)
        self.frameID = frameID
        arFrameID = nil
        self.monotonicTimestamp = monotonicTimestamp
    }

    public init(pixelBuffer: CVPixelBuffer, frameID: UInt64, monotonicTimestamp: TimeInterval) {
        image = .pixelBuffer(pixelBuffer)
        self.frameID = frameID
        arFrameID = nil
        self.monotonicTimestamp = monotonicTimestamp
    }

    public init(image: CGImage, arFrameID: ARFrameID, monotonicTimestamp: TimeInterval) {
        self.image = .cgImage(image)
        self.arFrameID = arFrameID
        frameID = arFrameID.sequence
        self.monotonicTimestamp = monotonicTimestamp
    }

    public init(pixelBuffer: CVPixelBuffer, arFrameID: ARFrameID,
                monotonicTimestamp: TimeInterval) {
        image = .pixelBuffer(pixelBuffer)
        self.arFrameID = arFrameID
        frameID = arFrameID.sequence
        self.monotonicTimestamp = monotonicTimestamp
    }
}

public struct OpticalObservation: Equatable, Sendable {
    public let payload: Data
    public let frameID: UInt64
    public let monotonicTimestamp: TimeInterval
    public let corners: OrientedMarkerCorners

    public init(payload: Data, frameID: UInt64, monotonicTimestamp: TimeInterval,
                corners: OrientedMarkerCorners) {
        self.payload = payload
        self.frameID = frameID
        self.monotonicTimestamp = monotonicTimestamp
        self.corners = corners
    }

    static func canonicalizedCorners(
        _ corners: OrientedMarkerCorners,
        from orientation: CGImagePropertyOrientation
    ) -> OrientedMarkerCorners {
        func point(_ value: Vec2) -> Vec2 {
            switch orientation {
            case .right: value
            case .up: Vec2(value.y, 1 - value.x)
            case .left: Vec2(1 - value.x, 1 - value.y)
            case .down: Vec2(1 - value.y, value.x)
            default: value
            }
        }

        switch orientation {
        case .right:
            return corners
        case .up:
            return OrientedMarkerCorners(
                topLeft: point(corners.bottomLeft), topRight: point(corners.topLeft),
                bottomLeft: point(corners.bottomRight), bottomRight: point(corners.topRight)
            )
        case .left:
            return OrientedMarkerCorners(
                topLeft: point(corners.bottomRight), topRight: point(corners.bottomLeft),
                bottomLeft: point(corners.topRight), bottomRight: point(corners.topLeft)
            )
        case .down:
            return OrientedMarkerCorners(
                topLeft: point(corners.topRight), topRight: point(corners.bottomRight),
                bottomLeft: point(corners.topLeft), bottomRight: point(corners.bottomLeft)
            )
        default:
            return corners
        }
    }
}

public enum OpticalScannerBackend: String, Equatable, Sendable {
    case vision
    case coreImage = "core_image"
}

public struct OpticalScannerBackendDiagnostic: Equatable, Sendable {
    public let backend: OpticalScannerBackend
    public let orientation: CGImagePropertyOrientation?
    public let errorDomain: String
    public let errorCode: Int

    public init(
        backend: OpticalScannerBackend,
        orientation: CGImagePropertyOrientation? = nil,
        errorDomain: String,
        errorCode: Int
    ) {
        self.backend = backend
        self.orientation = orientation
        self.errorDomain = errorDomain
        self.errorCode = errorCode
    }
}

public struct OpticalScanOutcome: Equatable, Sendable {
    public let observations: [OpticalObservation]
    public let diagnostics: [OpticalScannerBackendDiagnostic]

    public init(
        observations: [OpticalObservation],
        diagnostics: [OpticalScannerBackendDiagnostic] = []
    ) {
        self.observations = observations
        self.diagnostics = diagnostics
    }
}

public struct OpticalQRCodeRenderer: Sendable {
    public static let quietZoneModules = 4
    private let context = CIContext(options: [.cacheIntermediates: false])

    public init() {}

    public func render(payload: Data, moduleScale: Int) throws -> CGImage {
        guard !payload.isEmpty, payload.count <= OpticalMessageCodec.maximumPayloadBytes else {
            throw OpticalExchangeError.invalidPayload
        }
        guard moduleScale > 0 else { throw OpticalExchangeError.invalidScale }

        let generator = CIFilter.qrCodeGenerator()
        generator.message = payload
        generator.correctionLevel = "M"
        guard let code = generator.outputImage else { throw OpticalExchangeError.renderingFailed }
        let quiet = CGFloat(Self.quietZoneModules)
        let paddedExtent = code.extent.insetBy(dx: -quiet, dy: -quiet)
        let background = CIImage(color: .white).cropped(to: paddedExtent)
        let padded = code.composited(over: background)
        let scale = CGFloat(moduleScale)
        let scaled = padded.transformed(by: CGAffineTransform(scaleX: scale, y: scale))
        guard let image = context.createCGImage(scaled, from: scaled.extent) else {
            throw OpticalExchangeError.renderingFailed
        }
        return image
    }
}

public final class OpticalQRCodeScanner: @unchecked Sendable {
    private enum CoreImageScannerError: Error {
        case detectorUnavailable
    }

    typealias VisionScanner = @Sendable (
        OpticalFrame, CGImagePropertyOrientation
    ) throws -> [OpticalObservation]
    typealias CoreImageScanner = @Sendable (OpticalFrame) throws -> [OpticalObservation]

    private enum FrameIdentity: Hashable {
        case sequence(UInt64)
        case ar(ARFrameID)
    }

    private var processedFrameIDs = Set<FrameIdentity>()
    private let lock = NSLock()
    private let visionScanner: VisionScanner?
    private let coreImageScanner: CoreImageScanner?

    public init() {
        visionScanner = nil
        coreImageScanner = nil
    }

    init(
        visionScanner: @escaping VisionScanner,
        coreImageScanner: @escaping CoreImageScanner
    ) {
        self.visionScanner = visionScanner
        self.coreImageScanner = coreImageScanner
    }

    public func scan(_ frame: OpticalFrame) throws -> [OpticalObservation] {
        try scanDetailed(frame).observations
    }

    public func scanDetailed(_ frame: OpticalFrame) throws -> OpticalScanOutcome {
        guard frame.monotonicTimestamp.isFinite else { throw OpticalExchangeError.invalidTimestamp }
        lock.lock()
        let identity = frame.arFrameID.map(FrameIdentity.ar) ?? .sequence(frame.frameID)
        let isNewFrame = processedFrameIDs.insert(identity).inserted
        lock.unlock()
        guard isNewFrame else { return OpticalScanOutcome(observations: []) }

        var diagnostics: [OpticalScannerBackendDiagnostic] = []
        for orientation in [CGImagePropertyOrientation.right, .up, .left, .down] {
            do {
                let mapped: [OpticalObservation]
                if let visionScanner {
                    mapped = try visionScanner(frame, orientation)
                } else {
                    let results = try detect(in: frame.image, orientation: orientation)
                    mapped = observations(from: results, frame: frame, orientation: orientation)
                }
                if !mapped.isEmpty {
                    return OpticalScanOutcome(observations: mapped, diagnostics: diagnostics)
                }
            } catch {
                diagnostics.append(Self.diagnostic(
                    for: error, backend: .vision, orientation: orientation
                ))
            }
        }
        do {
            let observations = try coreImageScanner?(frame) ?? detectQRWithCoreImage(in: frame)
            return OpticalScanOutcome(observations: observations, diagnostics: diagnostics)
        } catch {
            diagnostics.append(Self.diagnostic(for: error, backend: .coreImage))
            return OpticalScanOutcome(observations: [], diagnostics: diagnostics)
        }
    }

    private static func diagnostic(
        for error: Error,
        backend: OpticalScannerBackend,
        orientation: CGImagePropertyOrientation? = nil
    ) -> OpticalScannerBackendDiagnostic {
        let error = error as NSError
        return OpticalScannerBackendDiagnostic(
            backend: backend, orientation: orientation,
            errorDomain: error.domain, errorCode: error.code
        )
    }

    private func detect(in image: OpticalFrame.Image,
                        orientation: CGImagePropertyOrientation) throws -> [VNBarcodeObservation] {
        let request = VNDetectBarcodesRequest()
        request.symbologies = [.qr]
        let handler: VNImageRequestHandler
        switch image {
        case let .cgImage(image): handler = VNImageRequestHandler(cgImage: image, orientation: orientation)
        case let .pixelBuffer(buffer): handler = VNImageRequestHandler(cvPixelBuffer: buffer, orientation: orientation)
        }
        try handler.perform([request])
        return request.results ?? []
    }

    nonisolated static func preferredPayload(decodedString: String?, rawData: Data?) -> Data? {
        decodedString.map { Data($0.utf8) } ?? rawData
    }

    private func observations(
        from results: [VNBarcodeObservation],
        frame: OpticalFrame,
        orientation: CGImagePropertyOrientation
    ) -> [OpticalObservation] {
        results.compactMap { observation in
            guard observation.symbology == .qr,
                  let payload = Self.preferredPayload(
                      decodedString: observation.payloadStringValue,
                      rawData: observation.payloadData
                  ) else {
                return nil
            }
            return OpticalObservation(
                payload: payload,
                frameID: frame.frameID,
                monotonicTimestamp: frame.monotonicTimestamp,
                corners: OpticalObservation.canonicalizedCorners(OrientedMarkerCorners(
                    topLeft: Vec2(Double(observation.topLeft.x), Double(observation.topLeft.y)),
                    topRight: Vec2(Double(observation.topRight.x), Double(observation.topRight.y)),
                    bottomLeft: Vec2(Double(observation.bottomLeft.x), Double(observation.bottomLeft.y)),
                    bottomRight: Vec2(Double(observation.bottomRight.x), Double(observation.bottomRight.y))
                ), from: orientation)
            )
        }
    }

    private func detectQRWithCoreImage(in frame: OpticalFrame) throws -> [OpticalObservation] {
        guard let detector = CIDetector(ofType: CIDetectorTypeQRCode, context: nil,
                                        options: [CIDetectorAccuracy: CIDetectorAccuracyHigh]) else {
            throw CoreImageScannerError.detectorUnavailable
        }
        let source: CIImage
        switch frame.image {
        case let .cgImage(image): source = CIImage(cgImage: image)
        case let .pixelBuffer(buffer): source = CIImage(cvPixelBuffer: buffer)
        }
        for orientation in [CGImagePropertyOrientation.up, .right, .left, .down] {
            let image = source.oriented(orientation)
            let extent = image.extent
            let observations = detector.features(in: image).compactMap { feature -> OpticalObservation? in
                guard let qr = feature as? CIQRCodeFeature, let string = qr.messageString else { return nil }
                func normalized(_ point: CGPoint) -> Vec2 {
                    Vec2(Double((point.x - extent.minX) / extent.width),
                         Double((point.y - extent.minY) / extent.height))
                }
                return OpticalObservation(payload: Data(string.utf8), frameID: frame.frameID,
                    monotonicTimestamp: frame.monotonicTimestamp,
                    corners: OpticalObservation.canonicalizedCorners(OrientedMarkerCorners(topLeft: normalized(qr.topLeft),
                        topRight: normalized(qr.topRight), bottomLeft: normalized(qr.bottomLeft),
                        bottomRight: normalized(qr.bottomRight)), from: orientation))
            }
            if !observations.isEmpty { return observations }
        }
        return []
    }
}
