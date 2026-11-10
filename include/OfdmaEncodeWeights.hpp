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

/**
 * @brief Deficit round robin over OFDMA users, in payload bytes.
 *
 * Each encoder pass adds a user's quantum (proportional to its information bits per frame, the smallest
 * user getting kMinQuantumBytes) to its deficit; the user then encodes queued packets while the next one
 * fits. Packets differ in size per user (the plans size datagrams to each share), so counting packets
 * (encode_weights) starves a user with small packets when the encoder thread is the bottleneck: on the
 * 2026-11-10 simulator gate a QPSK 1/2 user with 630-byte packets got half the packets of a 16QAM 1/2
 * user with 1260-byte packets, although both shares carry 4 packets per frame. Bytes follow the share.
 */
class DeficitRoundRobin {
public:
    static constexpr double kMinQuantumBytes = 1500.0;   // >= one datagram, so every pass can make progress

    explicit DeficitRoundRobin(const std::vector<EncodeLoad>& users) : _quantum(users.size(), kMinQuantumBytes),
                                                                         _deficit(users.size(), 0.0) {
        double min_bits = 0.0;
        bool any = false;
        for (const auto& u : users) {
            const double b = info_bits_per_frame(u);
            if (b > 0.0) {
                min_bits = any ? std::min(min_bits, b) : b;
                any = true;
            }
        }
        if (!any) return;
        for (std::size_t i = 0; i < users.size(); ++i) {
            _quantum[i] = kMinQuantumBytes * std::max(1.0, info_bits_per_frame(users[i]) / min_bits);
        }
    }

    /** Start user u's turn in a pass. */
    void begin_turn(std::size_t u) { _deficit[u] += _quantum[u]; }
    /** May user u encode a packet of this size now? Charges the deficit when it may. */
    bool admit(std::size_t u, std::size_t packet_bytes) {
        const double b = static_cast<double>(packet_bytes);
        if (b > _deficit[u]) return false;
        _deficit[u] -= b;
        return true;
    }
    /** User u's queue is empty: an idle user does not bank credit. */
    void idle(std::size_t u) { _deficit[u] = 0.0; }
    double quantum(std::size_t u) const { return _quantum[u]; }

private:
    std::vector<double> _quantum;
    std::vector<double> _deficit;
};

}  // namespace ofdma

#endif  // OFDMA_ENCODE_WEIGHTS_HPP
