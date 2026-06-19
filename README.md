# LXMSTR demos

Standalone example applications for the [LXMSTR](https://github.com/) real-time
EtherCAT master. These were split out of the main LXMSTR repo so the library can
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
#include <lxmstr/lxmstr.hpp>

lxmstr::NetworkConfig cfg = lxmstr::NetworkConfig::defaults();
lxmstr::EcNetwork net(cfg);
```

## Building

These demos require an **installed LXMSTR package** that exports the
`lxmstr::` CMake targets (see *LXMSTR package contract* below). Once that is
available:

```bash
cmake -B build
cmake --build build -j
```

If LXMSTR is installed to a non-default prefix, point CMake at it:

```bash
cmake -B build -DCMAKE_PREFIX_PATH=/path/to/lxmstr/prefix
```

Binaries are written to `build/bin/`.

## Running

The demos need a real-time-tuned host and an EtherCAT interface. They take no
command-line arguments: the ENI path and every other knob are hardcoded as
`constexpr` constants at the top of each `main.cpp`. The default ENI path is
`network.eni.xml` (relative to the working directory) — edit the `kEniPath`
constant or place the ENI there before running. With LXMSTR installed, run them
on the isolated RT core via the CLI:

```bash
sudo lxmstr host tune                 # one-time host setup; sets LXMSTR_RT_IFACE
sudo lxmstr run servo_sin_vel_demo
sudo lxmstr run build/bin/sine_shift_demo
```

You can also launch a binary directly, but `lxmstr run` applies the RT CPU/priority
placement the demos expect.

## LXMSTR package (build dependency)

`find_package(lxmstr CONFIG REQUIRED)` resolves to the installed LXMSTR package
(`sudo apt install lxmstr`, or a local `cmake --install`). It provides:

- the merged imported target **`lxmstr::lxmstr`** (the single `liblxmstr.so`, with the
  proprietary LXEC backend compiled in and hidden), with **`lxmstr::ecnet`** kept as a
  back-compat alias - these demos link `lxmstr::ecnet`;
- the installed public headers, with **`<lxmstr/lxmstr.hpp>`** as the advertised single
  entry point (plus the `ecnet/`, `facade/`, `devices/`, `ecmaster/`, `eni/` headers it
  pulls in transitively).

If LXMSTR is installed to a non-default prefix (e.g. a staging dir), pass
`-DCMAKE_PREFIX_PATH=/path/to/prefix` when configuring.
