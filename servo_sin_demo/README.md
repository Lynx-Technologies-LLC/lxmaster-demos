# servo_sin_demo — ENI-driven generic sine motion

A sinusoidal CSP profile with DC-sync tuning. Bring-up is **fully generic**: slave identity,
PDO layout, and vendor PreOP SDOs all come from an **ENI** file — there is no vendor C++.

```
+----------------+      +-----------------+      +-----------------------------+
|  eni_gen tool  | ---> |  network.eni.xml| ---> |  servo_sin_demo             |
| (scan + ESI)   |      |  (ETG.2100 ENI) |      |  (validate + sine motion)   |
+----------------+      +-----------------+      +-----------------------------+
```

## End-to-end flow

### 1. Generate an ENI

See `tools/eni_gen/README.md`. Example for a drive on a stock kernel (loose DC sync):

```
sudo lxmstr eni gen \
  --esi-dir path/to/esi \
  --cycle-ns 1000000 \
  --set-sdo 0x2013:6:u16=2 \
  --set-sdo 0x2013:7:u16=6000 \
  --out network.eni.xml
```

The EtherCAT interface is taken from the `LXMSTR_RT_IFACE` env (set by `lxmstr host tune` and
forwarded by `lxmstr eni`).

### 2. Run the sine demo

```
sudo lxmstr run --pty servo_sin_demo
```

The demo takes no command-line arguments. The ENI path (`network.eni.xml` by default) and the
sine profile (`kAmplitudeCounts`, `kFrequencyHz`, `kCycles`) are hardcoded as `constexpr`
constants at the top of `main.cpp` — edit them or place the ENI at the hardcoded path before
running.

The EtherCAT interface comes from the `LXMSTR_RT_IFACE` env (set by `lxmstr host tune`, forwarded
into the RT slice by `lxmstr run`).

Cyclic period and sync mode are taken solely from the ENI. The period is set when you generate
the ENI (`eni_gen --cycle-ns`, as above). The DC tuning knobs (PI divisors, busy-wait, warmup,
OP-entry gate, cooldown) come from the host config (`LXMSTR_*` env in
`/etc/profile.d/lxmstr-config.sh`, with built-in fallbacks); see `scripts/rt/README.md`.

## Build

Requires `LXMSTR_BUILD_APPS=ON` (default), plus `libxml2-dev`.

```
cmake -S . -B build -DLXMSTR_BUILD_APPS=ON
cmake --build build -j
```
