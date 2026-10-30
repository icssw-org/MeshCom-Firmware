// Host test for src/web_functions/web_auth.cpp -- nonce login and cookie sessions (WEB-AUTH-01, #1241).
//
//   pio test -e native_web_auth -f test_web_auth   (host only, no hardware)

#include <string>
#include <string.h>
#include <unity.h>

#include "hmac_sha256.h"
#include "web_auth.h"

// deterministic: first byte of call k is 0x30 for k=1; bytes are a mixed 32-bit counter (no repeats in a test)
static uint32_t g_ctr = 0;
static bool fakeRng(uint8_t *out, size_t n)
{
    for (size_t i = 0; i < n; i++)
    {
        uint32_t c = ++g_ctr;
        out[i] = (uint8_t)((c * 37u + 11u) ^ ((c >> 8) * 101u));
    }
    return true;
}

static bool deadRng(uint8_t *out, size_t n)
{
    memset(out, 0, n);
    return false;
}

static const char *PW = "secret";

void setUp(void)
{
    g_ctr = 0;
    webAuthReset();
}
void tearDown(void) {}

static std::string macFor(const char *pw, const std::string &nonce)
{
    uint8_t m[32];
    char hx[65];
    hmacSha256((const uint8_t *)pw, strlen(pw), (const uint8_t *)nonce.c_str(), nonce.size(), m);
    hexLower(m, 32, hx);
    return hx;
}

static std::string nonceAt(uint32_t t)
{
    char n[33];
    TEST_ASSERT_TRUE(webAuthIssueNonce(t, fakeRng, n));
    TEST_ASSERT_EQUAL(32, strlen(n));
    return n;
}

static WebAuthResult loginArg(const std::string &arg, const char *pw, uint32_t t, char *tok)
{
    return webAuthLogin(arg.c_str(), arg.size(), pw, t, fakeRng, tok);
}

static WebAuthResult loginWith(const std::string &nonce, const char *pw, const char *macpw, uint32_t t, char *tok)
{
    return loginArg(nonce + "." + macFor(macpw, nonce), pw, t, tok);
}

static std::string cookieOf(const char *tok) { return std::string("mcs=") + tok; }

static bool valid(const char *tok, uint32_t t)
{
    std::string c = cookieOf(tok);
    return webAuthSessionValid(c.c_str(), c.size(), t);
}

static std::string newSession(uint32_t t)
{
    char tok[33];
    std::string n = nonceAt(t);
    TEST_ASSERT_EQUAL(WA_OK, loginWith(n, PW, PW, t, tok));
    return tok;
}

void test_known_answer(void)
{
    TEST_ASSERT_EQUAL_STRING("024a4cd795dbf64e3f8aee57d9fd5ff8958313b1fcc718b16eaa8e51e2a6ec4a",
                             macFor("secret", "00112233445566778899aabbccddeeff").c_str());
}

void test_nonce_is_hex_from_rng(void)
{
    char n[33];
    TEST_ASSERT_TRUE(webAuthIssueNonce(0, fakeRng, n));
    TEST_ASSERT_EQUAL(32, strlen(n));
    for (int i = 0; i < 32; i++)
        TEST_ASSERT_TRUE((n[i] >= '0' && n[i] <= '9') || (n[i] >= 'a' && n[i] <= 'f'));
    TEST_ASSERT_EQUAL_STRING("30" /* (1*37+11)=48=0x30 */, std::string(n, 2).c_str());
}

void test_valid_login(void)
{
    char tok[33];
    std::string n = nonceAt(1000);
    TEST_ASSERT_EQUAL(WA_OK, loginWith(n, PW, PW, 1500, tok));
    TEST_ASSERT_EQUAL(32, strlen(tok));
    TEST_ASSERT_TRUE(valid(tok, 2000));
}

void test_wrong_password(void)
{
    char tok[33] = "";
    std::string n = nonceAt(0);
    TEST_ASSERT_EQUAL(WA_BADPW, loginWith(n, PW, "wrong", 10, tok));
}

void test_nonce_reuse(void)
{
    char tok[33];
    std::string n = nonceAt(0);
    TEST_ASSERT_EQUAL(WA_OK, loginWith(n, PW, PW, 10, tok));
    TEST_ASSERT_EQUAL(WA_NONCE, loginWith(n, PW, PW, 20, tok));
}

void test_nonce_consumed_by_failed_attempt(void)
{
    char tok[33];
    std::string n = nonceAt(0);
    TEST_ASSERT_EQUAL(WA_BADPW, loginWith(n, PW, "wrong", 10, tok));
    TEST_ASSERT_EQUAL(WA_NONCE, loginWith(n, PW, PW, 20, tok));
}

void test_nonce_expiry_boundary(void)
{
    char tok[33];
    std::string a = nonceAt(1000);
    TEST_ASSERT_EQUAL(WA_OK, loginWith(a, PW, PW, 1000 + 29999, tok));
    std::string b = nonceAt(1000);
    TEST_ASSERT_EQUAL(WA_NONCE, loginWith(b, PW, PW, 1000 + 30000, tok));
}

void test_nonce_expiry_millis_wrap(void)
{
    char tok[33];
    uint32_t t0 = 0xFFFFFFF0u;
    std::string a = nonceAt(t0);
    TEST_ASSERT_EQUAL(WA_OK, loginWith(a, PW, PW, t0 + 20000u, tok)); // wraps past 0
    std::string b = nonceAt(t0);
    TEST_ASSERT_EQUAL(WA_NONCE, loginWith(b, PW, PW, t0 + 30000u, tok));
}

void test_unknown_nonce(void)
{
    char tok[33];
    nonceAt(0);
    std::string n = "ffffffffffffffffffffffffffffffff";
    TEST_ASSERT_EQUAL(WA_NONCE, loginWith(n, PW, PW, 5, tok));
}

void test_malformed_args(void)
{
    char tok[33];
    std::string n = nonceAt(0);
    std::string mac = macFor(PW, n);
    std::string good = n + "." + mac;
    TEST_ASSERT_EQUAL(WA_FORMAT, loginArg("", PW, 1, tok));
    TEST_ASSERT_EQUAL(WA_FORMAT, loginArg(n, PW, 1, tok));
    TEST_ASSERT_EQUAL(WA_FORMAT, loginArg(n + "." + mac.substr(1), PW, 1, tok));
    TEST_ASSERT_EQUAL(WA_FORMAT, loginArg(good + "0", PW, 1, tok));
    TEST_ASSERT_EQUAL(WA_FORMAT, loginArg(good + "&x=1", PW, 1, tok));
    TEST_ASSERT_EQUAL(WA_FORMAT, loginArg(n + "-" + mac, PW, 1, tok));
    std::string up = good;
    for (size_t i = 0; i < up.size(); i++)
        if (up[i] >= 'a' && up[i] <= 'f')
        {
            up[i] = (char)(up[i] - 32);
            break;
        }
    TEST_ASSERT_EQUAL(WA_FORMAT, loginArg(up, PW, 1, tok));
    std::string nonhex = good;
    nonhex[70] = 'g';
    TEST_ASSERT_EQUAL(WA_FORMAT, loginArg(nonhex, PW, 1, tok));
    TEST_ASSERT_EQUAL(WA_FORMAT, webAuthLogin(NULL, 0, PW, 1, fakeRng, tok));
    // length says 97 but embedded NUL position is still checked as non-hex
    std::string withnul = good;
    withnul[40] = '\0';
    TEST_ASSERT_EQUAL(WA_FORMAT, loginArg(withnul, PW, 1, tok));
    // format errors do not consume the nonce and are not throttled
    TEST_ASSERT_EQUAL(WA_OK, loginArg(good, PW, 2, tok));
}

void test_nonce_slot_replacement(void)
{
    char tok[33];
    std::string n[5];
    for (int i = 0; i < 5; i++)
        n[i] = nonceAt(100 + i); // 5th replaces the oldest (n[0])
    TEST_ASSERT_EQUAL(WA_NONCE, loginWith(n[0], PW, PW, 200, tok));
    for (int i = 1; i < 5; i++)
        TEST_ASSERT_EQUAL(WA_OK, loginWith(n[i], PW, PW, 200, tok));
}

void test_cookie_token_parse(void)
{
    const char *t = "00112233445566778899aabbccddeeff";
    char o[33];
    std::string c;
    c = std::string("mcs=") + t;
    TEST_ASSERT_TRUE(webAuthCookieToken(c.c_str(), c.size(), o));
    TEST_ASSERT_EQUAL_STRING(t, o);
    c = std::string("a=b; mcs=") + t + "; c=d";
    TEST_ASSERT_TRUE(webAuthCookieToken(c.c_str(), c.size(), o));
    TEST_ASSERT_EQUAL_STRING(t, o);
    c = std::string("a=b;mcs=") + t;
    TEST_ASSERT_TRUE(webAuthCookieToken(c.c_str(), c.size(), o));
    c = std::string("  mcs=") + t + "  ;x=1";
    TEST_ASSERT_TRUE(webAuthCookieToken(c.c_str(), c.size(), o));
    c = std::string("xmcs=") + t;
    TEST_ASSERT_FALSE(webAuthCookieToken(c.c_str(), c.size(), o));
    c = std::string("a=b; xmcs=") + t + "; mcs=1";
    TEST_ASSERT_FALSE(webAuthCookieToken(c.c_str(), c.size(), o));
    c = std::string("mcs=") + t + "0";
    TEST_ASSERT_FALSE(webAuthCookieToken(c.c_str(), c.size(), o));
    c = std::string("mcs=") + std::string(t).substr(1);
    TEST_ASSERT_FALSE(webAuthCookieToken(c.c_str(), c.size(), o));
    c = "mcs=00112233445566778899AABBCCDDEEFF";
    TEST_ASSERT_FALSE(webAuthCookieToken(c.c_str(), c.size(), o));
    c = "mcs=";
    TEST_ASSERT_FALSE(webAuthCookieToken(c.c_str(), c.size(), o));
    TEST_ASSERT_FALSE(webAuthCookieToken("", 0, o));
    TEST_ASSERT_FALSE(webAuthCookieToken(NULL, 0, o));
    // len bounds the parse
    c = std::string("mcs=") + t;
    TEST_ASSERT_FALSE(webAuthCookieToken(c.c_str(), c.size() - 1, o));
}

void test_session_valid_refresh_expiry(void)
{
    const uint32_t H4 = 4u * 3600u * 1000u;
    std::string t = newSession(1000);
    TEST_ASSERT_TRUE(valid(t.c_str(), 1000 + H4 - 1));          // refreshes
    TEST_ASSERT_TRUE(valid(t.c_str(), 1000 + 2 * (H4 - 1)));    // alive thanks to the refresh
    uint32_t last = 1000 + 2 * (H4 - 1);
    TEST_ASSERT_FALSE(valid(t.c_str(), last + H4));             // exactly 4 h idle: expired
    TEST_ASSERT_FALSE(valid(t.c_str(), last + 1));              // slot was freed
}

void test_session_millis_wrap(void)
{
    const uint32_t H4 = 4u * 3600u * 1000u;
    uint32_t t0 = 0xFFFFFF00u;
    std::string t = newSession(t0);
    TEST_ASSERT_TRUE(valid(t.c_str(), t0 + H4 - 1)); // wrapped
    uint32_t last = t0 + H4 - 1;
    TEST_ASSERT_FALSE(valid(t.c_str(), last + H4));
}

void test_unknown_cookie_invalid(void)
{
    newSession(0);
    TEST_ASSERT_FALSE(valid("ffffffffffffffffffffffffffffffff", 5));
    TEST_ASSERT_FALSE(webAuthSessionValid("a=b", 3, 5));
}

void test_lru_eviction(void)
{
    std::string s[11];
    for (int i = 0; i < 10; i++)
        s[i] = newSession(1000 + i);
    TEST_ASSERT_TRUE(valid(s[0].c_str(), 2000)); // s[1] becomes least recently used
    s[10] = newSession(2001);
    TEST_ASSERT_FALSE(valid(s[1].c_str(), 2002));
    TEST_ASSERT_TRUE(valid(s[0].c_str(), 2002));
    TEST_ASSERT_TRUE(valid(s[10].c_str(), 2002));
    for (int i = 2; i < 10; i++)
        TEST_ASSERT_TRUE(valid(s[i].c_str(), 2002));
}

void test_logout(void)
{
    std::string a = newSession(0), b = newSession(1);
    std::string c = cookieOf(a.c_str());
    webAuthLogout(c.c_str(), c.size());
    TEST_ASSERT_FALSE(valid(a.c_str(), 5));
    TEST_ASSERT_TRUE(valid(b.c_str(), 5));
    webAuthLogout("junk", 4); // harmless
    TEST_ASSERT_TRUE(valid(b.c_str(), 6));
}

void test_reset_drops_sessions_and_nonces(void)
{
    char tok[33];
    std::string s = newSession(0);
    std::string n = nonceAt(1);
    webAuthReset();
    TEST_ASSERT_FALSE(valid(s.c_str(), 2));
    TEST_ASSERT_EQUAL(WA_NONCE, loginWith(n, PW, PW, 2, tok));
}

void test_throttle_trigger_and_release(void)
{
    char tok[33], n[33];
    for (int i = 0; i < 4; i++)
    {
        std::string x = nonceAt(1000 + i);
        TEST_ASSERT_EQUAL(WA_BADPW, loginWith(x, PW, "bad", 1000 + i, tok));
    }
    // 5th failure (an unknown nonce counts too) trips it
    TEST_ASSERT_EQUAL(WA_NONCE, loginWith("ffffffffffffffffffffffffffffffff", PW, PW, 1010, tok));
    TEST_ASSERT_FALSE(webAuthIssueNonce(1011, fakeRng, n));
    std::string good = macFor(PW, "00000000000000000000000000000000");
    TEST_ASSERT_EQUAL(WA_THROTTLED, loginArg("00000000000000000000000000000000." + good, PW, 1012, tok));
    TEST_ASSERT_FALSE(webAuthIssueNonce(1010 + 59999, fakeRng, n));
    TEST_ASSERT_TRUE(webAuthIssueNonce(1010 + 60000, fakeRng, n));
    TEST_ASSERT_EQUAL(WA_OK, loginWith(n, PW, PW, 1010 + 60001, tok));
}

void test_throttle_window_expires(void)
{
    char tok[33];
    for (int i = 0; i < 4; i++)
    {
        std::string x = nonceAt(0);
        TEST_ASSERT_EQUAL(WA_BADPW, loginWith(x, PW, "bad", 0, tok));
    }
    // 5th failure falls outside the 60 s window of the first four: no block
    std::string y = nonceAt(60000);
    TEST_ASSERT_EQUAL(WA_BADPW, loginWith(y, PW, "bad", 60000, tok));
    char n[33];
    TEST_ASSERT_TRUE(webAuthIssueNonce(60001, fakeRng, n));
}

void test_throttle_ignores_format_errors(void)
{
    char tok[33], n[33];
    for (int i = 0; i < 10; i++)
        TEST_ASSERT_EQUAL(WA_FORMAT, loginArg("zzz", PW, 5, tok));
    TEST_ASSERT_TRUE(webAuthIssueNonce(6, fakeRng, n));
}

void test_empty_password_never_authenticates(void)
{
    char tok[33];
    std::string n = nonceAt(0);
    TEST_ASSERT_EQUAL(WA_BADPW, loginWith(n, "", "", 1, tok));
    std::string m = nonceAt(2);
    TEST_ASSERT_EQUAL(WA_BADPW, loginArg(m + "." + macFor("", m), NULL, 3, tok));
}

void test_err_codes(void)
{
    TEST_ASSERT_EQUAL_STRING("format", webAuthErrCode(WA_FORMAT));
    TEST_ASSERT_EQUAL_STRING("nonce", webAuthErrCode(WA_NONCE));
    TEST_ASSERT_EQUAL_STRING("badpw", webAuthErrCode(WA_BADPW));
    TEST_ASSERT_EQUAL_STRING("throttled", webAuthErrCode(WA_THROTTLED));
}

// advisor W1: a truncated constant-time compare must not pass -- only the LAST hex char differs
static std::string flipLast(std::string s)
{
    s[s.size() - 1] = (s[s.size() - 1] == '0') ? '1' : '0';
    return s;
}

void test_mac_last_char_matters(void)
{
    char tok[33];
    std::string n = nonceAt(1);
    TEST_ASSERT_EQUAL(WA_BADPW, loginArg(n + "." + flipLast(macFor(PW, n)), PW, 2, tok));
}

void test_nonce_last_char_matters(void)
{
    char tok[33];
    std::string n = nonceAt(1);
    std::string other = flipLast(n);
    TEST_ASSERT_EQUAL(WA_NONCE, loginArg(other + "." + macFor(PW, other), PW, 2, tok));
    TEST_ASSERT_EQUAL(WA_OK, loginWith(n, PW, PW, 3, tok)); // the real nonce is still unused
}

void test_token_last_char_matters(void)
{
    std::string t = newSession(1);
    TEST_ASSERT_FALSE(valid(flipLast(t).c_str(), 2));
    TEST_ASSERT_TRUE(valid(t.c_str(), 3));
}

// a failing hardware RNG must neither issue a nonce nor create a session
void test_rng_failure_issues_nothing(void)
{
    char n[33];
    char tok[33];
    TEST_ASSERT_FALSE(webAuthIssueNonce(1, deadRng, n));
    std::string good = nonceAt(2);
    std::string arg = good + "." + macFor(PW, good);
    TEST_ASSERT_EQUAL(WA_RNG, webAuthLogin(arg.c_str(), arg.size(), PW, 3, deadRng, tok));
    TEST_ASSERT_EQUAL_STRING("rng", webAuthErrCode(WA_RNG));
    TEST_ASSERT_FALSE(valid("00000000000000000000000000000000", 4));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_known_answer);
    RUN_TEST(test_nonce_is_hex_from_rng);
    RUN_TEST(test_valid_login);
    RUN_TEST(test_wrong_password);
    RUN_TEST(test_nonce_reuse);
    RUN_TEST(test_nonce_consumed_by_failed_attempt);
    RUN_TEST(test_nonce_expiry_boundary);
    RUN_TEST(test_nonce_expiry_millis_wrap);
    RUN_TEST(test_unknown_nonce);
    RUN_TEST(test_malformed_args);
    RUN_TEST(test_nonce_slot_replacement);
    RUN_TEST(test_cookie_token_parse);
    RUN_TEST(test_session_valid_refresh_expiry);
    RUN_TEST(test_session_millis_wrap);
    RUN_TEST(test_unknown_cookie_invalid);
    RUN_TEST(test_lru_eviction);
    RUN_TEST(test_logout);
    RUN_TEST(test_reset_drops_sessions_and_nonces);
    RUN_TEST(test_throttle_trigger_and_release);
    RUN_TEST(test_throttle_window_expires);
    RUN_TEST(test_throttle_ignores_format_errors);
    RUN_TEST(test_empty_password_never_authenticates);
    RUN_TEST(test_err_codes);
    RUN_TEST(test_mac_last_char_matters);
    RUN_TEST(test_nonce_last_char_matters);
    RUN_TEST(test_token_last_char_matters);
    RUN_TEST(test_rng_failure_issues_nothing);
    return UNITY_END();
}
