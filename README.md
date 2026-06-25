# LXMASTER demos

Standalone example applications for the [LXMASTER](https://github.com/) real-time
EtherCAT master. These were split out of the main LXMASTER repo so the library can
ship on its own and consumers can build apps against the installed package the
same way an end user would.

Demos in this repo:

| Demo                     | What it shows                                                        |
| ------------------------ | ------------------------------------------------------------------- |
| `do_shift_demo`          | 16-bit digital-output left-shift walk on the first CiA401 I/O module. |
| `servo_sin_demo`         | Cyclic Synchronous Position (CSP) sine profile on every axis.        |
| `servo_sin_vel_demo`     | Cyclic Synchronous Velocity (CSV) sine profile on every axis.        |
| `servo_sin_torque_demo`  | Cyclic Synchronous Torque (CST) sine profile on every axis.          |
| `servo_mode_sweep_demo`  | Sweeps a single axis through CSP / CSV / CST operating modes.        |
| `sine_shift_demo`        | CSP sine on a servo while walking one active bit across a DO module. |

Each demo consumes the library through the single umbrella header and namespace:

```cpp
#include <lxmaster/lxmaster.hpp>

lxmaster::NetworkConfig cfg = lxmaster::NetworkConfig::defaults();
lxmaster::EcNetwork net(cfg);
```

## Building

These demos require an **installed LXMASTER package** that exports the
`lxmaster::` CMake targets (see *LXMASTER package contract* below). Once that is
available:

```bash
cmake -B build
cmake --build build -j
```

If LXMASTER is installed to a non-default prefix, point CMake at it:

```bash
cmake -B build -DCMAKE_PREFIX_PATH=/path/to/lxmaster/prefix
```

Binaries are written to `build/bin/`.

## Running

The demos need a real-time-tuned host and an EtherCAT interface. They take no
command-line arguments: the ENI path and every other knob are hardcoded as
`constexpr` constants at the top of each `main.cpp`. The default ENI path is
`network.eni.xml` (relative to the working directory) — edit the `kEniPath`
constant or place the ENI there before running. With LXMASTER installed, run them
on the isolated RT core via the CLI:

```bash
sudo lxmaster host tune                 # one-time host setup; sets LXMASTER_RT_IFACE
sudo lxmaster run servo_sin_vel_demo
sudo lxmaster run build/bin/sine_shift_demo
```

You can also launch a binary directly, but `lxmaster run` applies the RT CPU/priority
placement the demos expect.

## LXMASTER package (build dependency)

`find_package(lxmaster CONFIG REQUIRED)` resolves to the installed LXMASTER package
(`sudo apt install lxmaster`, or a local `cmake --install`). It provides:

- the merged imported target **`lxmaster::lxmaster`** (the single `liblxmaster.so`, with the
  proprietary LXEC backend compiled in and hidden), with **`lxmaster::ecnet`** kept as a
  back-compat alias - these demos link `lxmaster::ecnet`;
- the installed public headers, with **`<lxmaster/lxmaster.hpp>`** as the advertised single
  entry point (plus the `ecnet/`, `facade/`, `devices/`, `ecmaster/`, `eni/` headers it
  pulls in transitively).

If LXMASTER is installed to a non-default prefix (e.g. a staging dir), pass
`-DCMAKE_PREFIX_PATH=/path/to/prefix` when configuring.
