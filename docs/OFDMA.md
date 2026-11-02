# OFDMA: several users on one OpenISAC downlink

Branch `ofdma` adds downlink OFDMA to OpenISAC: the base station serves several users at once,
each on its own set of data subcarriers, with the sync symbol, the mid-frame pilot symbols, the
comb pilots and any `sensing_pilot` blocks shared by everyone. Each user has its own UDP payload
input at the BS and its own UE process; a UE decodes only its subcarriers and can restrict its
bistatic sensing reference to them. The channel simulator can serve several UEs at once, each
with its own channel, carrier offset and SNR. Nothing changes when `ofdma.enabled` is false.

## Configuration

The same `ofdma:` section goes into the BS and the UE configs (they must agree):

```yaml
ofdma:
  enabled: true
  num_users: 4
  scheme: interleaved       # interleaved | clusters | blocks | explicit
  cluster_size: 8           # clusters: adjacent data subcarriers per cluster
  user_subcarriers: []      # explicit: one list of FFT-bin indices per user
  mask_file: ""             # explicit: YAML file with a top-level user_subcarriers list
  user_id: 0                # UE only: which user this process is
  users:                    # BS only: one UDP payload input per user
    - { udp_input_port: 50000 }
    - { udp_input_port: 50010 }
    - { udp_input_port: 50020 }
    - { udp_input_port: 50030 }
```

"Data subcarriers" are the non-pilot FFT bins in ascending frequency order (bins
`fft_size/2 .. fft_size-1`, then `0 .. fft_size/2-1`), so `clusters` and `blocks` are contiguous
in frequency. Every user gets the same number of whole clusters; leftover data subcarriers carry
the usual random filler. `explicit` lists are checked for range, pilots and overlap. A user's
mask must leave room for the per-frame Identity SIB (192 QPSK symbols); the BS refuses to start
otherwise. ARQ, measurement mode and eRTM payload injection are not supported together with OFDMA
in this version.

Each BS user entry may also carry its own modulation and LDPC payload rate:

```yaml
  users:
    - { udp_input_port: 50000, modulation: QPSK, code_rate: "1/2" }
    - { udp_input_port: 50010, modulation: 16QAM, code_rate: "3/4" }
```

The defaults are the global `modulation.scheme` and `ldpc.code_rate`; the UE of that user sets those
two global keys to the same values. The Identity SIB and the per-packet control symbols stay QPSK.
The encoder serves users in proportion to the bits their share carries per frame.

Per frame, each user's subcarriers carry that user's Identity SIB followed by whole LDPC packets
from its UDP input, in the same rank order the single-user link uses, so the UE's frame lock,
packet parsing and UDP output are unchanged. Example configs: `config/BS_Sim_OFDMA.yaml`,
`config/UE_Sim_OFDMA.yaml`.

### UE sensing reference

```yaml
sensing:
  bistatic_data_mode: own   # own | all | pilots
```

`all` re-modulates every non-pilot subcarrier by hard decision (the single-user behaviour).
`own` (the default when OFDMA is on) zeroes the reference on other users' subcarriers, so the
bistatic channel estimate `H = rx · conj(tx)` uses the pilots, the sync and mid-frame pilot
symbols and this UE's own data only. `pilots` zeroes all data subcarriers.

## Channel simulator with several UEs

```yaml
simulation:
  ues:
    - { name: ue0, snr_offset_db: 0 }
    - { name: ue1, snr_offset_db: -3, cfo_hz: -2000 }
    - { name: ue2, channel_file: ue2.bin }
    - { name: ue3, comm_multipath_taps: [ { delay_samples: 0, gain_db: -30 }, { delay_samples: 7, gain_db: -36 } ] }
```

With `ues` present the hub creates one communication ring per entry (`rx.comm.<i>`), each with
its own tapped-delay line (or the shared `comm_multipath_taps`), the bistatic scatterers, a
carrier offset (`cfo_hz`, default `simulation.cfo_hz`), a target-SNR offset added to
`target_snr_db` (so the runtime `SNR` command still moves every UE together) and its own AWGN.
`channel_file` replaces the taps by a measured frequency response: `fft_size` little-endian
`complex<float>` values in FFT-bin order at the simulator's subcarrier spacing; its impulse
response is truncated to the cyclic prefix and applied by FFT overlap-save. Each UE sets
`simulation.ue_index` to pick its ring and gets its own RX tune-correction slot in the shared
control block (up to 16 UEs). The uplink is not simulated in multi-UE mode.

Start order and ports are as for the single-UE simulator: hub first, then BS, then the UEs,
each UE in its own directory with its own `UE.yaml` and distinct ZMQ/UDP ports.

Note on time: the hub advances air time only as fast as its slowest consumer reads, so with one or
more UEs attached a simulated link typically runs below real time (about 0.5x with one UE and 0.25x
with four on a 48-core Xeon at 25 MS/s; the stock single-UE 50 MS/s configuration behaves the same).
Nothing is lost, the clock just stretches: measure link throughput in packets per transmitted frame,
or read the hub's "produced N samples (T s of air time)" log lines, not in bits per wall-clock second.

## Tests

`OfdmaLayoutTest` (ctest) checks the masks and layouts for every scheme, the validation of
explicit masks and that the example configs load. End-to-end simulator runs with several UEs,
SNR ladders and measured channels live in the companion repository
[openisac-grid](https://github.com/kdandekar/openisac-grid) (`tests/07_ofdma_sim.sh`,
`tests/py/ofdma_run.py`, `analysis/ofdma_schedule.py`).
