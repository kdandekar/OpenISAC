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
    std::cout << (failures ? "FAILED" : "ALL PASS") << std::endl;
    return failures ? 1 : 0;
}
