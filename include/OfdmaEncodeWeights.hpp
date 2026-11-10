#ifndef OFDMA_ENCODE_WEIGHTS_HPP
#define OFDMA_ENCODE_WEIGHTS_HPP

/**
 * @file OfdmaEncodeWeights.hpp
 * @brief Encoder fairness across OFDMA users: packets per encoder pass.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace ofdma {

/** Information bits per LDPC(1008,504) mother-code block, at every code rate. */
constexpr double kLdpcInfoBitsPerBlock = 504.0;

/** What one user's share carries per frame. */
struct EncodeLoad {
    double bits_per_symbol;   ///< log2 of the constellation order
    std::size_t code_bits;    ///< transmitted bits per 504-bit block (LdpcRateMatching::code_bits: 1008, 756, 672)
    std::size_t payload_re;   ///< payload resource elements per frame
};

/** Information bits a user's share carries per frame. */
inline double info_bits_per_frame(const EncodeLoad& l) {
    if (l.code_bits == 0) return 0.0;
    return l.bits_per_symbol * static_cast<double>(l.payload_re) *
           kLdpcInfoBitsPerBlock / static_cast<double>(l.code_bits);
}

/**
 * @brief Packets per encoder pass for each user, proportional to its information bits per frame
 *        (packets are the same size for every user), the smallest user getting 1.
 *
 * A higher code rate transmits fewer bits per 504-bit block (code_bits 672 at 3/4), so it carries
 * MORE information per resource element: the weight scales with 504 / code_bits. (Until 2026-11-10
 * BS.cpp scaled with code_bits / 1008, which ranks 3/4 below 1/2; it did not matter while every user
 * ran at 1/2.)
 */
inline std::vector<std::size_t> encode_weights(const std::vector<EncodeLoad>& users) {
    std::vector<std::size_t> w(users.size(), 1);
    double min_bits = 0.0;
    bool any = false;
    for (const auto& u : users) {
        const double b = info_bits_per_frame(u);
        if (b > 0.0) {
            min_bits = any ? std::min(min_bits, b) : b;
            any = true;
        }
    }
    if (!any) return w;
    for (std::size_t i = 0; i < users.size(); ++i) {
        w[i] = static_cast<std::size_t>(std::max(1.0, std::round(info_bits_per_frame(users[i]) / min_bits)));
    }
    return w;
}

}  // namespace ofdma

#endif  // OFDMA_ENCODE_WEIGHTS_HPP
