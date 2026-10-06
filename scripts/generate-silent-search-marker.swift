#!/usr/bin/env swift

import AppKit
import CoreGraphics
import CoreImage
import CoreImage.CIFilterBuiltins
import Foundation
import PDFKit
import Vision

let markerID = "SILENT_SEARCH_01"
let payload = "PHROVER-CAL|1|\(markerID)"
let pointsPerMillimeter = 72.0 / 25.4
let pageSize = CGSize(width: 297 * pointsPerMillimeter, height: 420 * pointsPerMillimeter)
let qrExtent = 200 * pointsPerMillimeter
let qrOrigin = CGPoint(x: (pageSize.width - qrExtent) / 2, y: 90 * pointsPerMillimeter)
let qrTop = qrOrigin.y + qrExtent
let arrowCenterX = pageSize.width / 2
let arrowBaseY = qrTop + 90

func metadataSubject() -> String {
    [
        "payload=\(payload)",
        "pageWidthPoints=\(pageSize.width)",
        "pageHeightPoints=\(pageSize.height)",
        "qrExtentPoints=\(qrExtent)",
        "qrTopY=\(qrTop)",
        "arrowCenterX=\(arrowCenterX)",
        "arrowBaseY=\(arrowBaseY)",
    ].joined(separator: ";")
}

func generate(at outputURL: URL) throws {
    let filter = CIFilter.qrCodeGenerator()
    filter.message = Data(payload.utf8)
    filter.correctionLevel = "M"
    guard let image = filter.outputImage else {
        throw NSError(domain: "SilentSearchMarker", code: 1, userInfo: [NSLocalizedDescriptionKey: "QR generation failed"])
    }

    let extent = image.extent.integral
    let width = Int(extent.width)
    let height = Int(extent.height)
    var pixels = [UInt8](repeating: 0, count: width * height * 4)
    CIContext(options: [.useSoftwareRenderer: true]).render(
        image,
        toBitmap: &pixels,
        rowBytes: width * 4,
        bounds: extent,
        format: .RGBA8,
        colorSpace: CGColorSpaceCreateDeviceRGB()
    )

    var mediaBox = CGRect(origin: .zero, size: pageSize)
    let metadata: [CFString: Any] = [
        kCGPDFContextTitle: "Phrover Silent Search Calibration Marker",
        kCGPDFContextSubject: metadataSubject(),
        kCGPDFContextCreator: "generate-silent-search-marker.swift",
    ]
    guard let consumer = CGDataConsumer(url: outputURL as CFURL),
          let context = CGContext(consumer: consumer, mediaBox: &mediaBox, metadata as CFDictionary) else {
        throw NSError(domain: "SilentSearchMarker", code: 2, userInfo: [NSLocalizedDescriptionKey: "PDF creation failed"])
    }

    context.beginPDFPage(nil)
    context.setFillColor(CGColor(gray: 1, alpha: 1))
    context.fill(mediaBox)
    context.setFillColor(CGColor(gray: 0, alpha: 1))

    let moduleSize = qrExtent / Double(width)
    for row in 0..<height {
        for column in 0..<width where pixels[(row * width + column) * 4] < 128 {
            context.fill(CGRect(
                x: qrOrigin.x + Double(column) * moduleSize,
                y: qrOrigin.y + Double(row) * moduleSize,
                width: moduleSize,
                height: moduleSize
            ))
        }
    }

    context.setStrokeColor(CGColor(gray: 0, alpha: 1))
    context.setLineWidth(5)
    context.move(to: CGPoint(x: arrowCenterX, y: arrowBaseY))
    context.addLine(to: CGPoint(x: arrowCenterX, y: arrowBaseY + 52))
    context.strokePath()
    context.move(to: CGPoint(x: arrowCenterX, y: arrowBaseY + 68))
    context.addLine(to: CGPoint(x: arrowCenterX - 14, y: arrowBaseY + 44))
    context.addLine(to: CGPoint(x: arrowCenterX + 14, y: arrowBaseY + 44))
    context.closePath()
    context.fillPath()

    context.endPDFPage()
    context.closePDF()
}

func check(_ url: URL) throws {
    guard let document = PDFDocument(url: url), document.pageCount == 1,
          let page = document.page(at: 0) else {
        throw NSError(domain: "SilentSearchMarker", code: 3, userInfo: [NSLocalizedDescriptionKey: "expected a one-page PDF"])
    }
    let bounds = page.bounds(for: .mediaBox)
    guard abs(bounds.width - pageSize.width) < 0.01,
          abs(bounds.height - pageSize.height) < 0.01 else {
        throw NSError(domain: "SilentSearchMarker", code: 4, userInfo: [NSLocalizedDescriptionKey: "unexpected PDF page dimensions"])
    }
    guard let subject = document.documentAttributes?[PDFDocumentAttribute.subjectAttribute] as? String,
          subject == metadataSubject() else {
        throw NSError(domain: "SilentSearchMarker", code: 5, userInfo: [NSLocalizedDescriptionKey: "marker metadata mismatch"])
    }
    let thumbnail = page.thumbnail(of: CGSize(width: 1_684, height: 2_381), for: .mediaBox)
    var proposedRect = CGRect(origin: .zero, size: thumbnail.size)
    guard let image = thumbnail.cgImage(forProposedRect: &proposedRect, context: nil, hints: nil) else {
        throw NSError(domain: "SilentSearchMarker", code: 6, userInfo: [NSLocalizedDescriptionKey: "could not render marker PDF"])
    }
    let barcodeRequest = VNDetectBarcodesRequest()
    barcodeRequest.symbologies = [.qr]
    try VNImageRequestHandler(cgImage: image).perform([barcodeRequest])
    guard barcodeRequest.results?.contains(where: { $0.payloadStringValue == payload }) == true else {
        throw NSError(domain: "SilentSearchMarker", code: 7, userInfo: [NSLocalizedDescriptionKey: "QR payload mismatch"])
    }
    guard abs(qrExtent / pointsPerMillimeter - 200) < 1e-9,
          abs(arrowCenterX - (qrOrigin.x + qrExtent / 2)) < 1e-9,
          arrowBaseY > qrTop else {
        throw NSError(domain: "SilentSearchMarker", code: 8, userInfo: [NSLocalizedDescriptionKey: "marker geometry mismatch"])
    }
    print("silent-search marker valid")
}

do {
    if CommandLine.arguments.count == 3, CommandLine.arguments[1] == "--check" {
        try check(URL(fileURLWithPath: CommandLine.arguments[2]))
    } else if CommandLine.arguments.count <= 2 {
        let path = CommandLine.arguments.count == 2
            ? CommandLine.arguments[1]
            : "docs/assets/silent-search-calibration-marker.pdf"
        try generate(at: URL(fileURLWithPath: path))
    } else {
        throw NSError(domain: "SilentSearchMarker", code: 64, userInfo: [NSLocalizedDescriptionKey: "usage: generate-silent-search-marker.swift [OUTPUT] | --check PDF"])
    }
} catch {
    fputs("generate-silent-search-marker: \(error.localizedDescription)\n", stderr)
    exit(1)
}
