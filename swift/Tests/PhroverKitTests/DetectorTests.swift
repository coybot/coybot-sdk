import XCTest
import ImageIO
import CoreML
@testable import PhroverKit

final class DetectorTests: XCTestCase {
    func testModelResourcePrefersCompiledModelBundle() {
        let url = Detector.modelResourceURL(modelName: "RoverYOLO")

        XCTAssertEqual(url?.pathExtension, "mlmodelc")
    }

    func testModelConfigurationAvoidsGPUForBackgroundSafety() {
        XCTAssertEqual(Detector.modelConfiguration().computeUnits, .cpuAndNeuralEngine)
    }
}
