import CoreGraphics
import CoreVideo
import Foundation

struct RGBSample: Equatable, Sendable {
    let red: Float
    let green: Float
    let blue: Float
}

extension Array where Element == ObjectColorEvidence {
    subscript(color: LocalObjectColor) -> Float? {
        first(where: { $0.color == color })?.confidence
    }
}

struct LocalObjectColorAnalyzer: Sendable {
    private static let gridDimension = 8
    private static let insetFraction = 0.20

    func analyze(
        pixelBuffer: CVPixelBuffer,
        normalizedBoundingBox: CGRect
    ) -> [ObjectColorEvidence] {
        guard CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly) == kCVReturnSuccess else {
            return []
        }
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }

        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)
        guard width > 0,
              height > 0,
              let region = Self.sampleRegion(
                  normalizedBoundingBox: normalizedBoundingBox,
                  width: width,
                  height: height
              ) else {
            return []
        }

        let sampleColumns = min(Self.gridDimension, region.width)
        let sampleRows = min(Self.gridDimension, region.height)
        let pixelFormat = CVPixelBufferGetPixelFormatType(pixelBuffer)
        var samples: [RGBSample] = []
        samples.reserveCapacity(sampleColumns * sampleRows)

        for row in 0..<sampleRows {
            let y = region.minY + (2 * row + 1) * region.height / (2 * sampleRows)
            for column in 0..<sampleColumns {
                let x = region.minX + (2 * column + 1) * region.width / (2 * sampleColumns)
                guard let sample = sample(
                    pixelBuffer: pixelBuffer,
                    pixelFormat: pixelFormat,
                    x: min(x, region.maxX - 1),
                    y: min(y, region.maxY - 1)
                ) else {
                    continue
                }
                samples.append(sample)
            }
        }

        return Self.evidence(from: samples)
    }

    static func evidence(from samples: [RGBSample]) -> [ObjectColorEvidence] {
        guard !samples.isEmpty else { return [] }

        var counts: [LocalObjectColor: Int] = [:]
        for sample in samples {
            counts[classify(sample), default: 0] += 1
        }

        let total = Float(samples.count)
        return counts.map { color, count in
            ObjectColorEvidence(color: color, confidence: Float(count) / total)
        }.sorted {
            if $0.confidence != $1.confidence {
                return $0.confidence > $1.confidence
            }
            return $0.color.rawValue < $1.color.rawValue
        }
    }

    private static func classify(_ sample: RGBSample) -> LocalObjectColor {
        let red = min(max(sample.red, 0), 1)
        let green = min(max(sample.green, 0), 1)
        let blue = min(max(sample.blue, 0), 1)
        let maximum = max(red, green, blue)
        let minimum = min(red, green, blue)
        let chroma = maximum - minimum
        let saturation = maximum == 0 ? 0 : chroma / maximum
        let luminance = 0.2126 * red + 0.7152 * green + 0.0722 * blue

        if maximum <= 0.09 || (luminance < 0.12 && saturation < 0.45) {
            return .black
        }
        if saturation < 0.12 {
            return luminance >= 0.82 ? .white : .gray
        }

        let hue = hueDegrees(red: red, green: green, blue: blue, maximum: maximum, chroma: chroma)
        if hue >= 15, hue < 45, maximum < 0.65, luminance < 0.45 {
            return .brown
        }
        switch hue {
        case 0..<15, 345..<360:
            return .red
        case 15..<45:
            return .orange
        case 45..<70:
            return .yellow
        case 70..<165:
            return .green
        case 165..<255:
            return .blue
        default:
            return .purple
        }
    }

    private static func hueDegrees(
        red: Float,
        green: Float,
        blue: Float,
        maximum: Float,
        chroma: Float
    ) -> Float {
        guard chroma > 0 else { return 0 }
        let sector: Float
        if maximum == red {
            sector = (green - blue) / chroma
        } else if maximum == green {
            sector = (blue - red) / chroma + 2
        } else {
            sector = (red - green) / chroma + 4
        }
        let degrees = sector * 60
        return degrees < 0 ? degrees + 360 : degrees
    }

    private func sample(
        pixelBuffer: CVPixelBuffer,
        pixelFormat: OSType,
        x: Int,
        y: Int
    ) -> RGBSample? {
        switch pixelFormat {
        case kCVPixelFormatType_32BGRA:
            guard let address = CVPixelBufferGetBaseAddress(pixelBuffer) else { return nil }
            let bytesPerRow = CVPixelBufferGetBytesPerRow(pixelBuffer)
            let pixel = address.assumingMemoryBound(to: UInt8.self) + y * bytesPerRow + x * 4
            return RGBSample(
                red: Float(pixel[2]) / 255,
                green: Float(pixel[1]) / 255,
                blue: Float(pixel[0]) / 255
            )

        case kCVPixelFormatType_420YpCbCr8BiPlanarFullRange,
             kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange:
            guard CVPixelBufferGetPlaneCount(pixelBuffer) >= 2,
                  let yAddress = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0),
                  let chromaAddress = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1) else {
                return nil
            }
            let yStride = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0)
            let chromaStride = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1)
            let yBase = yAddress.assumingMemoryBound(to: UInt8.self)
            let yValue = yBase[y * yStride + x]
            let chroma = chromaAddress.assumingMemoryBound(to: UInt8.self)
                + (y / 2) * chromaStride
                + (x / 2) * 2
            return Self.rgb(
                y: yValue,
                cb: chroma[0],
                cr: chroma[1],
                videoRange: pixelFormat == kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange
            )

        default:
            return nil
        }
    }

    private static func rgb(y: UInt8, cb: UInt8, cr: UInt8, videoRange: Bool) -> RGBSample {
        let luminance: Float
        let blueDifference: Float
        let redDifference: Float
        if videoRange {
            luminance = (Float(y) - 16) / 219
            blueDifference = (Float(cb) - 128) / 224
            redDifference = (Float(cr) - 128) / 224
        } else {
            luminance = Float(y) / 255
            blueDifference = (Float(cb) - 128) / 255
            redDifference = (Float(cr) - 128) / 255
        }
        return RGBSample(
            red: min(max(luminance + 1.402 * redDifference, 0), 1),
            green: min(max(luminance - 0.344_136 * blueDifference - 0.714_136 * redDifference, 0), 1),
            blue: min(max(luminance + 1.772 * blueDifference, 0), 1)
        )
    }

    private static func sampleRegion(
        normalizedBoundingBox: CGRect,
        width: Int,
        height: Int
    ) -> PixelRegion? {
        let box = normalizedBoundingBox.standardized
        let minX = min(max(box.minX, 0), 1)
        let maxX = min(max(box.maxX, 0), 1)
        let minY = min(max(box.minY, 0), 1)
        let maxY = min(max(box.maxY, 0), 1)
        guard maxX > minX, maxY > minY else { return nil }

        var left = Int(floor(minX * CGFloat(width)))
        var right = Int(ceil(maxX * CGFloat(width)))
        var top = Int(floor((1 - maxY) * CGFloat(height)))
        var bottom = Int(ceil((1 - minY) * CGFloat(height)))
        left = min(max(left, 0), width - 1)
        right = min(max(right, left + 1), width)
        top = min(max(top, 0), height - 1)
        bottom = min(max(bottom, top + 1), height)

        let horizontalInset = Int((Double(right - left) * insetFraction).rounded(.down))
        let verticalInset = Int((Double(bottom - top) * insetFraction).rounded(.down))
        if right - left - 2 * horizontalInset > 0 {
            left += horizontalInset
            right -= horizontalInset
        }
        if bottom - top - 2 * verticalInset > 0 {
            top += verticalInset
            bottom -= verticalInset
        }
        return PixelRegion(minX: left, maxX: right, minY: top, maxY: bottom)
    }
}

private struct PixelRegion {
    let minX: Int
    let maxX: Int
    let minY: Int
    let maxY: Int

    var width: Int { maxX - minX }
    var height: Int { maxY - minY }
}
