// OFDMA per-user subcarrier masks and payload layouts.
//
//   OfdmaLayoutTest <source dir>
//
// Checks that every allocation scheme gives disjoint, equal, pilot-free masks in
// frequency order, that per-user payload layouts partition the BS union layout,
// that bad explicit masks are rejected, and that the shipped OFDMA example
// configs load. Runs without a radio or a hub.
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include "Common.hpp"

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    std::cout << (ok ? "PASS " : "FAIL ") << what << '\n';
    if (!ok) ++failures;
}

Config base_config(size_t users, const std::string& scheme, size_t cluster = 1) {
    Config cfg = make_default_bs_config();
    cfg.ofdm.fft_size = 1024;
    cfg.ofdm.cp_length = 128;
    cfg.ofdm.num_symbols = 100;
    cfg.ofdm.sync_pos = 1;
    cfg.ofdm.pilot_positions = {571, 631, 692, 752, 812, 872, 933, 993, 29, 89, 150, 210, 270, 330, 391, 451};
    cfg.ofdm.midframe_pilot_symbols = {19, 39, 59, 79, 99};
    cfg.uplink.enabled = false;
    cfg.ofdma.enabled = true;
    cfg.ofdma.num_users = users;
    cfg.ofdma.scheme = scheme;
    cfg.ofdma.cluster_size = cluster;
    return cfg;
}

size_t count(const std::vector<uint8_t>& m) {
    return static_cast<size_t>(std::count(m.begin(), m.end(), 1));
}

// Longest run of owned bins in frequency order (bins n/2.., then 0..).
size_t longest_run_in_frequency_order(const std::vector<uint8_t>& m, const std::vector<uint8_t>& pilot) {
    const size_t n = m.size();
    size_t best = 0, run = 0;
    for (size_t i = 0; i < n; ++i) {
        const size_t k = (i + n / 2) % n;
        if (pilot[k]) continue;                 // pilots interrupt nothing: they are not data
        if (m[k]) { ++run; best = std::max(best, run); } else { run = 0; }
    }
    return best;
}

void test_scheme(const std::string& scheme, size_t users, size_t cluster, size_t expected_run) {
    Config cfg = base_config(users, scheme, cluster);
    const auto masks = build_ofdma_user_subcarrier_masks(cfg);
    const size_t n = cfg.ofdm.fft_size;
    std::vector<uint8_t> pilot(n, 0);
    for (auto p : cfg.ofdm.pilot_positions) pilot[p] = 1;
    const size_t n_data = n - cfg.ofdm.pilot_positions.size();

    check(masks.size() == users, scheme + ": one mask per user");
    std::vector<int> owners(n, 0);
    bool on_pilot = false, equal = true;
    const size_t c0 = count(masks[0]);
    for (const auto& m : masks) {
        if (count(m) != c0) equal = false;
        for (size_t k = 0; k < n; ++k) {
            owners[k] += m[k];
            if (m[k] && pilot[k]) on_pilot = true;
        }
    }
    const bool disjoint = std::all_of(owners.begin(), owners.end(), [](int o) { return o <= 1; });
    const size_t covered = static_cast<size_t>(std::count(owners.begin(), owners.end(), 1));
    check(disjoint, scheme + ": masks are disjoint");
    check(!on_pilot, scheme + ": no pilot subcarrier is owned");
    check(equal, scheme + ": every user owns the same number of subcarriers (" + std::to_string(c0) + ")");
    check(covered + (n_data % (users * cluster)) >= n_data - users * cluster && covered <= n_data,
          scheme + ": " + std::to_string(covered) + " of " + std::to_string(n_data) + " data subcarriers assigned");
    check(longest_run_in_frequency_order(masks[0], pilot) == expected_run,
          scheme + ": user 0's longest frequency-contiguous run is " +
              std::to_string(longest_run_in_frequency_order(masks[0], pilot)) + " (expected " +
              std::to_string(expected_run) + ")");

    // Layouts: the BS union is the sum of the users; each user's payload rank
    // space is exactly its subcarriers times the data symbols.
    const DataResourceGridLayout bs_layout = build_data_resource_grid_layout(cfg);
    size_t sum = 0;
    bool per_user_ok = true;
    for (size_t u = 0; u < users; ++u) {
        const DataResourceGridLayout lu = build_data_resource_grid_layout_for_user(cfg, static_cast<int>(u));
        sum += lu.payload_re_count;
        if (lu.payload_re_count != count(masks[u]) * lu.data_symbol_count) per_user_ok = false;
        for (size_t f = 0; f < lu.non_pilot_re_count; ++f) {
            if (lu.payload_mask[f] && !bs_layout.payload_mask[f]) per_user_ok = false;
        }
    }
    check(sum == bs_layout.payload_re_count,
          scheme + ": per-user payload RE sum to the BS union (" + std::to_string(sum) + ")");
    check(per_user_ok, scheme + ": each user's layout is its subcarriers x data symbols, inside the union");
    check(bs_layout.data_symbol_count == 94, scheme + ": 94 data symbols (100 - sync - 5 mid-frame pilots)");
}

void test_explicit_validation() {
    Config cfg = base_config(2, kOfdmaSchemeExplicit);
    cfg.ofdma.user_subcarriers = {{0, 1, 2}, {3, 4, 5}};
    bool ok = true;
    try { (void)build_ofdma_user_subcarrier_masks(cfg); } catch (...) { ok = false; }
    check(ok, "explicit: valid disjoint lists accepted");

    cfg.ofdma.user_subcarriers = {{0, 1, 2}, {2, 3}};
    bool threw = false;
    try { (void)build_ofdma_user_subcarrier_masks(cfg); } catch (const std::exception&) { threw = true; }
    check(threw, "explicit: overlapping lists rejected");

    cfg.ofdma.user_subcarriers = {{0, 29}, {3}};
    threw = false;
    try { (void)build_ofdma_user_subcarrier_masks(cfg); } catch (const std::exception&) { threw = true; }
    check(threw, "explicit: a pilot bin is rejected");

    cfg.ofdma.user_subcarriers = {{0}, {}};
    threw = false;
    try { (void)build_ofdma_user_subcarrier_masks(cfg); } catch (const std::exception&) { threw = true; }
    check(threw, "explicit: an empty user is rejected");

    cfg.ofdma.user_subcarriers = {{0}, {1}, {2}};
    threw = false;
    try { (void)build_ofdma_user_subcarrier_masks(cfg); } catch (const std::exception&) { threw = true; }
    check(threw, "explicit: wrong number of lists rejected");

    Config bad = base_config(2, "stripes");
    threw = false;
    try { (void)build_ofdma_user_subcarrier_masks(bad); } catch (const std::exception&) { threw = true; }
    check(threw, "unknown scheme rejected");

    Config disabled = base_config(4, kOfdmaSchemeInterleaved);
    disabled.ofdma.enabled = false;
    check(build_ofdma_user_subcarrier_masks(disabled).empty(), "disabled OFDMA yields no masks");
    const DataResourceGridLayout full = build_data_resource_grid_layout(disabled);
    check(full.payload_re_count == 94 * 1008, "disabled OFDMA keeps the full 94 x 1008 payload grid");
}

void test_example_configs(const std::string& src_dir) {
    Config bs = make_default_bs_config();
    const bool bs_ok = load_bs_config_from_yaml(bs, src_dir + "/config/BS_Sim_OFDMA.yaml");
    check(bs_ok && bs.ofdma.enabled && bs.ofdma.num_users == 4 && bs.ofdma.user_id == -1 &&
              bs.ofdma.users.size() == 4 && bs.ofdma.users[3].udp_input_port == 50030 &&
              bs.simulation.ues.size() == 4 && bs.simulation.ues[2].cfo_hz == -2000.0 &&
              bs.simulation.ues[3].comm_multipath_taps.size() == 2,
          "config/BS_Sim_OFDMA.yaml loads: 4 users, 4 simulated UEs");
    Config ue = make_default_ue_config();
    const bool ue_ok = load_ue_config_from_yaml(ue, src_dir + "/config/UE_Sim_OFDMA.yaml");
    check(ue_ok && ue.ofdma.enabled && ue.ofdma.user_id == 0 && ue.simulation.ue_index == 0 &&
              ue.sensing.bistatic_data_mode == "own",
          "config/UE_Sim_OFDMA.yaml loads: user 0, ue_index 0, sensing data mode own");
    if (bs_ok && ue_ok) {
        const DataResourceGridLayout lb = build_data_resource_grid_layout_for_user(bs, 0);
        const DataResourceGridLayout lu = build_data_resource_grid_layout(ue);
        check(lb.payload_mask == lu.payload_mask, "BS user-0 layout equals the UE's own layout");
    }
}

}  // namespace

int main(int argc, char** argv) {
    const std::string src_dir = argc > 1 ? argv[1] : ".";
    // 1008 data subcarriers, 4 users: interleaved -> runs of 1; clusters of 8 -> 8;
    // blocks -> 252 each.
    test_scheme(kOfdmaSchemeInterleaved, 4, 1, 1);
    test_scheme(kOfdmaSchemeClusters, 4, 8, 8);
    test_scheme(kOfdmaSchemeBlocks, 4, 1, 252);
    test_scheme(kOfdmaSchemeInterleaved, 1, 1, 1008);
    test_explicit_validation();
    test_example_configs(src_dir);
    std::cout << (failures == 0 ? "ALL PASS" : std::to_string(failures) + " FAILURES") << '\n';
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
