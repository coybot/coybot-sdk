import Foundation
import FoundationModels
@testable import PhroverKit

/// Which brain a benchmark run puts in the loop, chosen by `BENCH_BRAIN`:
///
///   apple                 Apple's on-device Foundation Model (needs Apple Intelligence)
///   ollama:<model>        any model served by a local Ollama (`OLLAMA_URL`, default
///                         http://127.0.0.1:11434), e.g. `ollama:qwen3.5:9b`
///
/// Every option runs the *same* `OnDeviceBrain` — same instructions, same prompt, same
/// output schema, same decision mapping. Only the model behind it changes, so a
/// difference in the results is a difference in the model, not in the brain around it.
@MainActor
enum BenchBrain {
    static var spec: String? { ProcessInfo.processInfo.environment["BENCH_BRAIN"] }

    static func make() throws -> (name: String, brain: OnDeviceBrain) {
        guard let spec, !spec.isEmpty else {
            throw BenchSkip.notConfigured
        }
        if spec == "apple" {
            let brain = OnDeviceBrain()
            guard brain.isAvailable else { throw BenchSkip.appleUnavailable }
            return ("apple-foundation-model", brain)
        }
        if spec.hasPrefix("ollama:") {
            let model = String(spec.dropFirst("ollama:".count))
            let base = URL(string: ProcessInfo.processInfo.environment["OLLAMA_URL"] ?? "http://127.0.0.1:11434")!
            let brain = OnDeviceBrain(isAvailable: { true },
                                      makeResponder: { OllamaResponder(baseURL: base, model: model) })
            return (model, brain)
        }
        throw BenchSkip.unknown(spec)
    }
}

enum BenchSkip: Error, CustomStringConvertible {
    case notConfigured, appleUnavailable, unknown(String)

    var description: String {
        switch self {
        case .notConfigured: "BENCH_BRAIN not set — run via eco/rover/sim/run_follow_brain_bench.py"
        case .appleUnavailable: "Apple Intelligence is not available on this host"
        case .unknown(let spec): "unknown BENCH_BRAIN \(spec)"
        }
    }
}

/// Per-call model statistics, read back by the benchmark after each decision.
@MainActor
final class BenchStats {
    static let shared = BenchStats()
    var last: [String: Any] = [:]
}

/// `OnDeviceBrainResponder` backed by a local Ollama server. Constrains decoding to
/// `OnDeviceDecision.generationSchema` — the JSON Schema the Foundation Models framework
/// itself derives from the `@Generable` type, guides included — and parses the reply
/// through the same `GeneratedContent` path, so an invalid action is rejected exactly as
/// it would be for the Apple model.
///
/// Apple's framework puts the schema in front of its model; Ollama only constrains the
/// sampler, so the schema is also written into the system prompt to give these models
/// the same information.
///
/// Raw `/api/generate`, not `/api/chat`: Ollama 0.30.9 silently drops the `format`
/// constraint for Qwen 3.5 when `think` is false (it only applies it after a thinking
/// block), and thinking is not affordable in a 12 s decision budget on a phone. The raw
/// prompt is Qwen's own chat format with the thinking block pre-closed — the model
/// card's non-thinking mode — so the constraint applies and no thinking tokens are spent.
@MainActor
struct OllamaResponder: OnDeviceBrainResponder {
    let baseURL: URL
    let model: String

    enum ResponderError: Error { case badResponse(String) }

    func nextAction(prompt: String, context: MissionContext) async throws -> BrainOutput {
        let schemaData = try JSONEncoder().encode(OnDeviceDecision.generationSchema)
        let schema = try JSONSerialization.jsonObject(with: schemaData)
        let system = OnDeviceBrain.instructions
            + "\n\nReply with only a JSON object matching this schema:\n"
            + (String(data: schemaData, encoding: .utf8) ?? "")
        let raw = "<|im_start|>system\n\(system)<|im_end|>\n"
            + "<|im_start|>user\n\(prompt)<|im_end|>\n"
            + "<|im_start|>assistant\n<think>\n\n</think>\n\n"
        let body: [String: Any] = [
            "model": model,
            "stream": false,
            "raw": true,
            "prompt": raw,
            "keep_alive": "30m",
            "format": schema,
            // Qwen's recommended sampling for non-thinking mode.
            "options": ["temperature": 0.7, "top_p": 0.8, "top_k": 20],
        ]
        var request = URLRequest(url: baseURL.appendingPathComponent("api/generate"))
        request.httpMethod = "POST"
        request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        request.httpBody = try JSONSerialization.data(withJSONObject: body)
        request.timeoutInterval = 120

        let (data, _) = try await URLSession.shared.data(for: request)
        guard let reply = try JSONSerialization.jsonObject(with: data) as? [String: Any],
              let text = reply["response"] as? String
        else {
            throw ResponderError.badResponse(String(data: data, encoding: .utf8) ?? "")
        }
        BenchStats.shared.last = [
            "prompt_tokens": reply["prompt_eval_count"] ?? NSNull(),
            "output_tokens": reply["eval_count"] ?? NSNull(),
            "prompt_ms": ((reply["prompt_eval_duration"] as? Double) ?? 0) / 1e6,
            "decode_ms": ((reply["eval_duration"] as? Double) ?? 0) / 1e6,
            "load_ms": ((reply["load_duration"] as? Double) ?? 0) / 1e6,
        ]
        let decision = try OnDeviceDecision(try GeneratedContent(json: text))
        return BrainOutput(decision: OnDeviceBrain.map(decision, context: context),
                           updatedPlan: decision.updatedPlan.isEmpty ? nil : decision.updatedPlan)
    }
}

/// Emits one `BENCH {json}` line — xcodebuild's stdout is the reliable channel out of an
/// iOS Simulator test process (same reasoning as `EventLog`).
func benchEmit(_ record: [String: Any]) {
    guard JSONSerialization.isValidJSONObject(record),
          let data = try? JSONSerialization.data(withJSONObject: record, options: [.sortedKeys]),
          let line = String(data: data, encoding: .utf8)
    else { return }
    print("BENCH \(line)")
}
