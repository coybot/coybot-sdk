import XCTest
@testable import PhroverKit

final class RuntimeLogWriterTests: XCTestCase {
    func testWriterIsBoundedAndReportsOverflowWithoutBlockingProducer() async {
        let entered = expectation(description: "worker held")
        let dropped = expectation(description: "overflow visible")
        let gate = DispatchSemaphore(value: 0)
        let writer = RuntimeLogWriter(capacity: 1, reportDropped: { count in
            XCTAssertEqual(count, 1)
            dropped.fulfill()
        })
        XCTAssertTrue(writer.submit {
            entered.fulfill()
            _ = gate.wait(timeout: .now() + 3)
        })
        await fulfillment(of: [entered], timeout: 1)
        XCTAssertFalse(writer.submit { XCTFail("Overflow cannot enqueue unbounded work") })
        gate.signal()
        await writer.flush()
        await fulfillment(of: [dropped], timeout: 1)
        XCTAssertTrue(writer.submit {})
        await writer.flush()
    }
    @MainActor
    func testWritingDoesNotOccupyControlThread() async {
        let writer = RuntimeLogWriter()
        let completed = expectation(description: "writer executed")
        XCTAssertTrue(writer.submit {
            XCTAssertFalse(Thread.isMainThread, "Disk/formatting work must not delay control")
            completed.fulfill()
        })
        await fulfillment(of: [completed], timeout: 2)
        await writer.flush()
    }
}
