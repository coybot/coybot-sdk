#!/usr/bin/env xcrun swift

import CreateML
import Foundation

private struct Arguments {
    let dataset: URL
    let outputDirectory: URL
    let maxIterations: Int

    init(_ values: [String]) throws {
        var options: [String: String] = [:]
        var index = 1
        while index < values.count {
            guard index + 1 < values.count else {
                throw TrainingError.usage("Missing value for \(values[index])")
            }
            options[values[index]] = values[index + 1]
            index += 2
        }
        guard let datasetPath = options["--dataset"],
              let outputPath = options["--output"] else {
            throw TrainingError.usage("Required: --dataset PATH --output PATH [--max-iterations N]")
        }
        let iterations = Int(options["--max-iterations"] ?? "30") ?? 0
        guard iterations > 0 else {
            throw TrainingError.usage("--max-iterations must be a positive integer")
        }
        dataset = URL(fileURLWithPath: datasetPath, isDirectory: true)
        outputDirectory = URL(fileURLWithPath: outputPath, isDirectory: true)
        maxIterations = iterations
    }
}

private enum TrainingError: Error, CustomStringConvertible {
    case usage(String)
    case missingDataset(URL)

    var description: String {
        switch self {
        case .usage(let message): message
        case .missingDataset(let url): "Missing dataset split or annotations at \(url.path)"
        }
    }
}

private func dataSource(_ split: String, dataset: URL) throws -> MLObjectDetector.DataSource {
    let directory = dataset.appendingPathComponent(split, isDirectory: true)
    let annotations = directory.appendingPathComponent("annotations.json")
    guard FileManager.default.fileExists(atPath: annotations.path) else {
        throw TrainingError.missingDataset(directory)
    }
    return .directoryWithImagesAndJsonAnnotation(at: directory)
}

private func metricsDictionary(_ metrics: MLObjectDetectorMetrics) -> [String: Any] {
    [
        "is_valid": metrics.isValid,
        "error": metrics.error?.localizedDescription as Any,
        "mean_average_precision": [
            "varied_iou": metrics.meanAveragePrecision.variedIoU,
            "iou_50": metrics.meanAveragePrecision.IoU50,
        ],
        "average_precision": [
            "varied_iou": metrics.averagePrecision.variedIoU,
            "iou_50": metrics.averagePrecision.IoU50,
        ],
    ]
}

do {
    let arguments = try Arguments(CommandLine.arguments)
    let fileManager = FileManager.default
    try fileManager.createDirectory(
        at: arguments.outputDirectory,
        withIntermediateDirectories: true
    )

    let trainingData = try dataSource("train", dataset: arguments.dataset)
    let validationData = try dataSource("validation", dataset: arguments.dataset)
    let testData = try dataSource("test", dataset: arguments.dataset)
    let parameters = MLObjectDetector.ModelParameters(
        validation: .dataSource(validationData),
        batchSize: nil,
        maxIterations: arguments.maxIterations,
        algorithm: .transferLearning(.objectPrint(revision: 1))
    )

    print("Training ScreenYOLO with \(arguments.maxIterations) iterations...")
    let detector = try MLObjectDetector(
        trainingData: trainingData,
        parameters: parameters,
        annotationType: .boundingBox(units: .pixel, origin: .topLeft, anchor: .center)
    )
    let testMetrics = detector.evaluation(on: testData)

    let modelURL = arguments.outputDirectory.appendingPathComponent("ScreenYOLO.mlmodel")
    let metadata = MLModelMetadata(
        author: "Coybot Technology Corp",
        shortDescription: "Offline one-class screen detector for Phrover navigation.",
        license: "Training images: Open Images V7, CC BY 2.0; see provenance.json.",
        version: "1"
    )
    try detector.write(to: modelURL, metadata: metadata)

    let report: [String: Any] = [
        "max_iterations": arguments.maxIterations,
        "model": modelURL.lastPathComponent,
        "training": metricsDictionary(detector.trainingMetrics),
        "validation": metricsDictionary(detector.validationMetrics),
        "test": metricsDictionary(testMetrics),
    ]
    let reportData = try JSONSerialization.data(withJSONObject: report, options: [.prettyPrinted, .sortedKeys])
    try reportData.write(to: arguments.outputDirectory.appendingPathComponent("metrics.json"))
    print("Wrote \(modelURL.path)")
    print(String(data: reportData, encoding: .utf8) ?? "")
} catch {
    FileHandle.standardError.write(Data("screen-detector training failed: \(error)\n".utf8))
    exit(1)
}
