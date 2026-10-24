import Foundation

final class TestURLProtocol: URLProtocol, @unchecked Sendable {
    struct RecordedRequest: Sendable {
        let url: URL
        let statusCode: Int?
        let predecessorCompletedAtStart: Bool
    }

    private struct MutableRequest {
        let url: URL
        var statusCode: Int?
        let predecessorCompletedAtStart: Bool
    }

    private static let requestLock = NSLock()
    nonisolated(unsafe) private static var configuredDelay: TimeInterval = 0
    nonisolated(unsafe) private static var recordedRequests: [MutableRequest] = []

    nonisolated(unsafe) private static var holdNext = false
    nonisolated(unsafe) private static var heldResponse: (@Sendable () -> Void)?
    nonisolated(unsafe) private static var requestStarted: (@Sendable () -> Void)?

    static func holdNextResponse(onStart: @escaping @Sendable () -> Void) {
        withRequestLock { holdNext = true; requestStarted = onStart }
    }
    static func releaseHeldResponse() {
        let response = withRequestLock { let response = heldResponse; heldResponse = nil; return response }
        response?()
    }

    static var delay: TimeInterval {
        get { withRequestLock { configuredDelay } }
        set { withRequestLock { configuredDelay = newValue } }
    }

    static var requestCount: Int {
        withRequestLock { recordedRequests.count }
    }

    static var requestURLs: [URL] {
        requestRecords.map(\.url)
    }

    static var requestRecords: [RecordedRequest] {
        withRequestLock {
            recordedRequests.map {
                RecordedRequest(url: $0.url, statusCode: $0.statusCode, predecessorCompletedAtStart: $0.predecessorCompletedAtStart)
            }
        }
    }

    static func reset() {
        withRequestLock {
            configuredDelay = 0
            recordedRequests = []
            holdNext = false
            heldResponse = nil
            requestStarted = nil
        }
    }

    override class func canInit(with request: URLRequest) -> Bool { true }
    override class func canonicalRequest(for request: URLRequest) -> URLRequest { request }

    override func startLoading() {
        let request = request
        let (requestIndex, delay) = Self.recordStart(url: request.url!)
        let response: @Sendable () -> Void = { [weak self] in
            guard let self,
                  let response = HTTPURLResponse(
                    url: request.url!,
                    statusCode: 200,
                    httpVersion: nil,
                    headerFields: nil
                  ) else { return }
            Self.recordCompletion(at: requestIndex, statusCode: response.statusCode)
            self.client?.urlProtocol(self, didReceive: response, cacheStoragePolicy: .notAllowed)
            self.client?.urlProtocol(self, didLoad: Data())
            self.client?.urlProtocolDidFinishLoading(self)
        }
        let onStart: (@Sendable () -> Void)? = Self.withRequestLock {
            guard Self.holdNext else { return nil }
            Self.holdNext = false
            Self.heldResponse = response
            let callback = Self.requestStarted
            Self.requestStarted = nil
            return callback
        }
        if let onStart { onStart() }
        else { DispatchQueue.global().asyncAfter(deadline: .now() + delay, execute: response) }
    }

    override func stopLoading() {}

    private static func recordStart(url: URL) -> (index: Int, delay: TimeInterval) {
        withRequestLock {
            let index = recordedRequests.count
            recordedRequests.append(MutableRequest(url: url, statusCode: nil,
                predecessorCompletedAtStart: recordedRequests.allSatisfy { $0.statusCode != nil }))
            return (index, configuredDelay)
        }
    }

    private static func recordCompletion(at index: Int, statusCode: Int) {
        withRequestLock {
            guard recordedRequests.indices.contains(index) else { return }
            recordedRequests[index].statusCode = statusCode
        }
    }

    private static func withRequestLock<T>(_ body: () -> T) -> T {
        requestLock.lock()
        defer { requestLock.unlock() }
        return body()
    }
}
