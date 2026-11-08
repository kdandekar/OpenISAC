// Tests radio::align_device_times_to_next_pps against fake devices whose PPS edges come from the
// wall clock (one edge per second), so each case takes about two seconds.
//
// The two last-PPS behaviours seen on real USRPs on the load edge of set_time_next_pps(0):
//   X310:      time_last_pps() reports the loaded time, 0.
//   USRP2/N210: it latches the time just before the load (the old device time), and reports 0-based
//              values (1.0, 2.0, ...) only from the next edge (openisac-grid E6, 2026-11-07).
// Both must align, and on return time_last_pps() must already be 0-based.
#include "PpsTimeAlignment.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>

namespace {

using Clock = std::chrono::steady_clock;

enum class Latch { Loaded, PreLoad };

class FakeUsrp : public radio::IDevice {
public:
    FakeUsrp(Latch latch, double start_time_s, bool pps_present = true, bool honours_load = true)
        : _latch(latch), _pps(pps_present), _load(honours_load),
          _t0(Clock::now()), _base_s(start_time_s), _last_pps_s(start_time_s - 1.0) {}

    bool supports(radio::Capability cap) const override { return cap == radio::Capability::PpsTimeSync; }
    radio::TimeSpec time_now() const override { std::lock_guard<std::mutex> g(_m); _advance(); return radio::TimeSpec(_now_s()); }
    radio::TimeSpec time_last_pps() const override { std::lock_guard<std::mutex> g(_m); _advance(); return radio::TimeSpec(_last_pps_s); }
    void set_time_next_pps(const radio::TimeSpec& t) override {
        std::lock_guard<std::mutex> g(_m);
        _advance();
        _armed = true;
        _armed_value = t.get_real_secs();
    }
    double master_clock_rate() const override { return 100e6; }
    double get_tx_rate(size_t) const override { return 25e6; }
    radio::TuneResult set_tx_freq(const radio::TuneRequest&, size_t) override { return {}; }
    radio::TuneResult set_rx_freq(const radio::TuneRequest&, size_t) override { return {}; }
    radio::ITxStreamPtr get_tx_stream(const radio::StreamArgs&) override { return nullptr; }
    radio::IRxStreamPtr get_rx_stream(const radio::StreamArgs&) override { return nullptr; }

private:
    // Edges at wall-clock seconds 0.35, 1.35, ... after construction; the device time runs at 1 s/s.
    double _wall() const { return std::chrono::duration<double>(Clock::now() - _t0).count(); }
    double _now_s() const { return _base_s + _wall(); }
    void _advance() const {
        if (!_pps) return;
        const double w = _wall();
        while (_next_edge_w <= w) {
            const double at_edge = _base_s + _next_edge_w;          // device time just before the edge
            if (_armed && _load) {
                _base_s = _armed_value - _next_edge_w;              // device time = armed value at the edge
                _armed = false;
                _last_pps_s = _latch == Latch::Loaded ? _armed_value : at_edge;
            } else {
                _last_pps_s = at_edge;
            }
            _next_edge_w += 1.0;
        }
    }

    Latch _latch;
    bool _pps, _load;
    Clock::time_point _t0;
    mutable std::mutex _m;
    mutable double _base_s, _last_pps_s;
    mutable double _next_edge_w = 0.35;
    mutable bool _armed = false;
    mutable double _armed_value = 0.0;
};

int failures = 0;

void expect(bool ok, const std::string& what) {
    std::cout << (ok ? "PASS " : "FAIL ") << what << std::endl;
    if (!ok) ++failures;
}

bool aligns(std::vector<std::shared_ptr<FakeUsrp>> devs, std::string* error = nullptr) {
    std::vector<radio::PpsTimeAlignmentTarget> targets;
    for (size_t i = 0; i < devs.size(); ++i)
        targets.push_back({"dev" + std::to_string(i), devs[i], "external"});
    try {
        const auto r = radio::align_device_times_to_next_pps(targets);
        for (size_t i = 0; i < devs.size(); ++i) {
            const double last = devs[i]->time_last_pps().get_real_secs();
            const double now = devs[i]->time_now().get_real_secs();
            if (!(last >= 0.0 && last <= now && now < 2.5)) {
                if (error) *error = "after return: last_pps " + std::to_string(last) + ", now " + std::to_string(now);
                return false;
            }
        }
        return r.devices.size() == devs.size();
    } catch (const std::exception& e) {
        if (error) *error = e.what();
        return false;
    }
}

}  // namespace

int main() {
    std::string err;
    expect(aligns({std::make_shared<FakeUsrp>(Latch::Loaded, 1234.5)}, &err), "X310-like single device aligns " + err);
    err.clear();
    expect(aligns({std::make_shared<FakeUsrp>(Latch::PreLoad, 1794032191.4)}, &err),
           "N210-like single device aligns (last_pps latched before the load) " + err);
    err.clear();
    expect(aligns({std::make_shared<FakeUsrp>(Latch::PreLoad, 77.1), std::make_shared<FakeUsrp>(Latch::PreLoad, 912.8)}, &err),
           "two N210-like devices align " + err);
    err.clear();
    expect(aligns({std::make_shared<FakeUsrp>(Latch::Loaded, 5.0), std::make_shared<FakeUsrp>(Latch::PreLoad, 600.0)}, &err),
           "X310 + N210 align " + err);
    err.clear();
    expect(!aligns({std::make_shared<FakeUsrp>(Latch::PreLoad, 10.0, /*pps*/ false)}, &err) &&
               err.find("PPS transition") != std::string::npos,
           "no PPS: fails waiting for the transition (" + err + ")");
    err.clear();
    expect(!aligns({std::make_shared<FakeUsrp>(Latch::PreLoad, 10.0, true, /*honours_load*/ false)}, &err) &&
               err.find("verifying") != std::string::npos,
           "load ignored: fails verification (" + err + ")");
    std::cout << (failures ? "FAILED" : "ALL PASS") << std::endl;
    return failures ? 1 : 0;
}
