import Foundation
import Security

/// Cognito email/password authentication. Construct one with your `PhroverCloudConfig`
/// and own it for the lifetime of your app (e.g. as a single `@State` instance) — this
/// type deliberately isn't a singleton so multiple configs (dev/prod backends) can coexist.
@Observable
@MainActor
public final class AuthService: NSObject {

    public var currentUser: User?
    public var isAuthenticated: Bool { currentUser != nil }
    public var isLoading = false
    public var error: AuthError?

    private var tokens: AuthTokens?
    private let config: PhroverCloudConfig
    private let keychainKey: String

    /// - Parameter keychainKey: Keychain account used to persist the session. Namespace this
    ///   per-app (e.g. your bundle id + ".session") if multiple apps on the same device use
    ///   PhroverCloud against different backends.
    public init(config: PhroverCloudConfig, keychainKey: String = "bot.coy.phrover.session") {
        self.config = config
        self.keychainKey = keychainKey
        super.init()
        loadStoredSession()
    }

    public var idToken: String? { tokens?.idToken }

    // MARK: - Email/Password Sign In

    public func signIn(email: String, password: String) async throws {
        isLoading = true; error = nil
        do {
            let result = try await cognitoInitiateAuth(
                flow: "USER_PASSWORD_AUTH",
                params: ["USERNAME": email, "PASSWORD": password]
            )
            apply(result: result, email: email)
            isLoading = false
        } catch {
            isLoading = false
            let e = AuthError.signInFailed(error.localizedDescription)
            self.error = e
            throw e
        }
    }

    public func signUp(email: String, password: String) async throws {
        isLoading = true; error = nil
        do {
            try await cognitoSignUp(email: email, password: password)
            isLoading = false
        } catch {
            isLoading = false
            let e = AuthError.signInFailed(error.localizedDescription)
            self.error = e
            throw e
        }
    }

    public func confirmSignUp(email: String, code: String) async throws {
        isLoading = true; error = nil
        do {
            try await cognitoConfirmSignUp(email: email, code: code)
            isLoading = false
        } catch {
            isLoading = false
            let e = AuthError.signInFailed(error.localizedDescription)
            self.error = e
            throw e
        }
    }

    public func signOut() {
        tokens = nil; currentUser = nil
        clearSession()
    }

    @discardableResult
    public func refreshTokensIfNeeded(force: Bool = false) async throws -> Bool {
        guard let current = tokens else { throw AuthError.notAuthenticated }
        guard force || current.isExpiredOrExpiringSoon else { return false }
        guard let refreshToken = current.refreshToken else {
            if current.isExpired { throw AuthError.tokenRefreshFailed }
            return false
        }
        do {
            let result = try await cognitoInitiateAuth(
                flow: "REFRESH_TOKEN_AUTH",
                params: ["REFRESH_TOKEN": refreshToken]
            )
            apply(result: result, email: currentUser?.email ?? "", keepRefreshToken: refreshToken)
            return true
        } catch {
            throw AuthError.tokenRefreshFailed
        }
    }

    // MARK: - Cognito API calls

    private struct CognitoAuthResult {
        let idToken: String
        let refreshToken: String?
        let expiresIn: Int
    }

    private func cognitoInitiateAuth(flow: String, params: [String: String]) async throws -> CognitoAuthResult {
        guard let url = URL(string: "https://cognito-idp.\(config.region).amazonaws.com/") else {
            throw AuthError.signInFailed("Invalid Cognito URL")
        }
        let body: [String: Any] = [
            "AuthFlow": flow,
            "ClientId": config.cognitoClientId,
            "AuthParameters": params,
        ]
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("application/x-amz-json-1.1", forHTTPHeaderField: "Content-Type")
        request.setValue("AWSCognitoIdentityProviderService.InitiateAuth", forHTTPHeaderField: "X-Amz-Target")
        request.httpBody = try JSONSerialization.data(withJSONObject: body)

        let (data, _) = try await URLSession.shared.data(for: request)
        return try decodeCognitoAuthResult(from: data)
    }

    private func cognitoSignUp(email: String, password: String) async throws {
        guard let url = URL(string: "https://cognito-idp.\(config.region).amazonaws.com/") else {
            throw AuthError.signInFailed("Invalid Cognito URL")
        }
        let body: [String: Any] = [
            "ClientId": config.cognitoClientId,
            "Username": email,
            "Password": password,
            "UserAttributes": [["Name": "email", "Value": email]],
        ]
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("application/x-amz-json-1.1", forHTTPHeaderField: "Content-Type")
        request.setValue("AWSCognitoIdentityProviderService.SignUp", forHTTPHeaderField: "X-Amz-Target")
        request.httpBody = try JSONSerialization.data(withJSONObject: body)

        let (data, _) = try await URLSession.shared.data(for: request)
        if let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
           let message = json["message"] as? String ?? (json["__type"] as? String) {
            throw AuthError.signInFailed(message)
        }
    }

    private func cognitoConfirmSignUp(email: String, code: String) async throws {
        guard let url = URL(string: "https://cognito-idp.\(config.region).amazonaws.com/") else {
            throw AuthError.signInFailed("Invalid Cognito URL")
        }
        let body: [String: Any] = [
            "ClientId":         config.cognitoClientId,
            "Username":         email,
            "ConfirmationCode": code,
        ]
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("application/x-amz-json-1.1", forHTTPHeaderField: "Content-Type")
        request.setValue("AWSCognitoIdentityProviderService.ConfirmSignUp", forHTTPHeaderField: "X-Amz-Target")
        request.httpBody = try JSONSerialization.data(withJSONObject: body)

        let (data, _) = try await URLSession.shared.data(for: request)
        if let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
           let message = json["message"] as? String ?? (json["__type"] as? String) {
            throw AuthError.signInFailed(message)
        }
    }

    private func decodeCognitoAuthResult(from data: Data) throws -> CognitoAuthResult {
        guard let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any] else {
            throw AuthError.signInFailed("Invalid response")
        }
        if let message = json["message"] as? String { throw AuthError.signInFailed(message) }
        if let type = json["__type"] as? String {
            throw AuthError.signInFailed(json["message"] as? String ?? type)
        }
        guard let auth = json["AuthenticationResult"] as? [String: Any],
              let idToken = auth["IdToken"] as? String else {
            throw AuthError.signInFailed("Missing tokens in response")
        }
        return CognitoAuthResult(
            idToken: idToken,
            refreshToken: auth["RefreshToken"] as? String,
            expiresIn: auth["ExpiresIn"] as? Int ?? 3600
        )
    }

    // MARK: - Helpers

    private func apply(result: CognitoAuthResult, email: String, keepRefreshToken: String? = nil) {
        let expiresAt = Date().addingTimeInterval(TimeInterval(result.expiresIn))
        tokens = AuthTokens(
            idToken: result.idToken,
            refreshToken: result.refreshToken ?? keepRefreshToken,
            expiresAt: expiresAt
        )
        if currentUser == nil || currentUser?.email != email {
            currentUser = User(id: email, email: email)
        }
        saveSession()
    }

    // MARK: - Session Persistence

    private func saveSession() {
        guard let tokens, let user = currentUser else { return }
        if let data = try? JSONEncoder().encode(StoredSession(tokens: tokens, user: user)) {
            KeychainHelper.save(data, forKey: keychainKey)
        }
    }

    private func loadStoredSession() {
        guard let data = KeychainHelper.load(forKey: keychainKey),
              let session = try? JSONDecoder().decode(StoredSession.self, from: data) else { return }
        tokens = session.tokens
        currentUser = session.user
    }

    private func clearSession() { KeychainHelper.delete(forKey: keychainKey) }

    private struct StoredSession: Codable {
        let tokens: AuthTokens
        let user: User
    }
}

// MARK: - Auth Errors

public enum AuthError: LocalizedError {
    case notAuthenticated
    case signInFailed(String)
    case tokenRefreshFailed

    public var errorDescription: String? {
        switch self {
        case .notAuthenticated:      return "Not authenticated"
        case .signInFailed(let msg): return msg
        case .tokenRefreshFailed:    return "Session expired. Please sign in again."
        }
    }
}

// MARK: - Keychain Helper

enum KeychainHelper {
    static func save(_ data: Data, forKey key: String) {
        let query: [String: Any] = [kSecClass as String: kSecClassGenericPassword,
                                    kSecAttrAccount as String: key, kSecValueData as String: data]
        SecItemDelete(query as CFDictionary)
        SecItemAdd(query as CFDictionary, nil)
    }
    static func load(forKey key: String) -> Data? {
        let query: [String: Any] = [kSecClass as String: kSecClassGenericPassword,
                                    kSecAttrAccount as String: key, kSecReturnData as String: true]
        var result: AnyObject?
        SecItemCopyMatching(query as CFDictionary, &result)
        return result as? Data
    }
    static func delete(forKey key: String) {
        let query: [String: Any] = [kSecClass as String: kSecClassGenericPassword,
                                    kSecAttrAccount as String: key]
        SecItemDelete(query as CFDictionary)
    }
}
