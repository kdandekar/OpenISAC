// Tests ofdma::encode_weights, the BS's packets-per-pass split across OFDMA users.
// The regression: code_bits is the number of TRANSMITTED bits per 504-bit block, so a higher code rate
// (fewer transmitted bits) must get a larger weight, not a smaller one.
#include "OfdmaEncodeWeights.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace {

int failures = 0;

void expect(const std::vector<std::size_t>& got, const std::vector<std::size_t>& want, const std::string& what) {
    const bool ok = got == want;
    std::cout << (ok ? "PASS " : "FAIL ") << what << " [";
    for (std::size_t i = 0; i < got.size(); ++i) std::cout << (i ? " " : "") << got[i];
    std::cout << "]" << std::endl;
    if (!ok) ++failures;
}

constexpr std::size_t R12 = 1008, R23 = 756, R34 = 672;

}  // namespace

int main() {
    using ofdma::EncodeLoad;
    using ofdma::encode_weights;
    expect(encode_weights({{2, R12, 900}, {2, R12, 900}, {2, R12, 900}, {2, R12, 900}}), {1, 1, 1, 1},
           "equal shares at QPSK 1/2");
    expect(encode_weights({{2, R12, 1800}, {2, R12, 900}}), {2, 1}, "twice the subcarriers, same MCS");
    // QPSK 3/4 carries 1.5x QPSK 1/2 on the same REs. The old formula gave {2, 1}: inverted.
    expect(encode_weights({{2, R12, 900}, {2, R34, 900}}), {1, 2}, "higher code rate gets more packets");
    expect(encode_weights({{2, R12, 900}, {4, R34, 900}}), {1, 3}, "16QAM 3/4 vs QPSK 1/2");
    expect(encode_weights({{2, R12, 900}, {6, R23, 900}}), {1, 4}, "64QAM 2/3 vs QPSK 1/2");
    expect(encode_weights({{4, R12, 900}, {2, R12, 900}}), {2, 1}, "16QAM 1/2 vs QPSK 1/2");
    expect(encode_weights({{2, R12, 0}, {2, R12, 900}}), {1, 1}, "an empty share still gets one");
    const double q12 = ofdma::info_bits_per_frame({2, R12, 1000});
    const double q34 = ofdma::info_bits_per_frame({2, R34, 1000});
    std::cout << ((q12 == 1000.0 && q34 == 1500.0) ? "PASS " : "FAIL ") << "info bits: QPSK 1/2 " << q12
              << ", QPSK 3/4 " << q34 << " per 1000 RE" << std::endl;
    if (!(q12 == 1000.0 && q34 == 1500.0)) ++failures;
    // Deficit round robin with the 2026-11-10 gate's plan (blocks-amc): an always-backlogged encoder.
    // users: 16QAM 3/4 1197 B, 16QAM 1/2 1260 B, 64QAM 1/2 1197 B, QPSK 1/2 630 B; equal REs. Each share
    // carries 6, 4, 6, 4 packets per frame, so the bytes encoded must follow 6x1197 : 4x1260 : 6x1197 : 4x630.
    {
        const std::vector<EncodeLoad> loads{{4, R34, 1000}, {4, R12, 1000}, {6, R12, 1000}, {2, R12, 1000}};
        const std::vector<std::size_t> size{1197, 1260, 1197, 630};
        ofdma::DeficitRoundRobin drr(loads);
        std::vector<double> bytes(4, 0.0), pkts(4, 0.0);
        for (int pass = 0; pass < 20000; ++pass) {
            for (std::size_t u = 0; u < 4; ++u) {
                drr.begin_turn(u);
                while (drr.admit(u, size[u])) { bytes[u] += size[u]; pkts[u] += 1; }
            }
        }
        const double want[4] = {3.0, 2.0, 3.0, 1.0};   // info bits per frame, relative
        bool ok = true;
        for (std::size_t u = 0; u < 4; ++u) {
            const double ratio = (bytes[u] / bytes[3]) / want[u];
            ok = ok && ratio > 0.98 && ratio < 1.02;
        }
        // users 1 and 3 both carry 4 packets per frame: user 1's packets are twice the size and its share
        // twice the bits, so they must get the same number of packets (the gate gave user 3 half).
        ok = ok && pkts[1] / pkts[3] > 0.98 && pkts[1] / pkts[3] < 1.02;
        std::cout << (ok ? "PASS " : "FAIL ") << "DRR bytes follow the shares' info bits (3:2:3:1); packets "
                  << pkts[0] << " " << pkts[1] << " " << pkts[2] << " " << pkts[3] << std::endl;
        if (!ok) ++failures;
        // A user whose queue runs dry banks nothing.
        ofdma::DeficitRoundRobin d2({{2, R12, 1000}, {2, R12, 1000}});
        for (int i = 0; i < 10; ++i) { d2.begin_turn(0); d2.idle(0); }
        d2.begin_turn(0);
        const bool one = d2.admit(0, 1400) && !d2.admit(0, 1400);
        std::cout << (one ? "PASS " : "FAIL ") << "idle user does not bank credit" << std::endl;
        if (!one) ++failures;
    }
    std::cout << (failures ? "FAILED" : "ALL PASS") << std::endl;
    return failures ? 1 : 0;
}
