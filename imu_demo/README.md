# imu_demo

Minimal example of a **brand-new device class** -- one that fits none of the built-in typed contracts
(it is not a drive, an I/O module, or an encoder). It binds a user-defined profile
(`demo::ImuProfile`, a direct subclass of `ecdev::IDeviceProfile`) to an EtherCAT IMU, brings it to
`OPERATIONAL`, prints a few acceleration/angular-rate samples, and shuts down.

It demonstrates the generic-device path end to end:

- **Implement `IDeviceProfile` directly.** `ImuProfile` resolves its mapped TxPDO objects in
  `configurePreOp`, snapshots them each cycle in `readInputs` into lock-free atomics, and exposes its
  own getters. It does **not** implement `asMotion()`/`asIo()`/`asEncoder()`, so there is no typed
  facade for it.
- **Static factory.** `ImuProfile::make` maps `ProfileSelectionInput` to a profile instance; `main()`
  passes it to `makeIdentityProfileFactory`.
- **Register for one run.** `main.cpp` pushes a factory onto `NetworkConfig::extra_profile_factories`,
  so no static registration or whole-archive linking is needed.
- **Reach it through `devices()`.** The application iterates `net.devices()` (every profile-carrying
  slave) and finds the IMU from `GenericDevice::deviceProfile()` without RTTI -- this is an RT system,
  so it guards on the unique `profileName()` and `static_cast`s once at setup (no `dynamic_cast`). It
  calls `configure()` to opt the device into `OPERATIONAL`, then reads samples through the profile.

## Requirements

This demo uses `EcNetwork::devices()`, `lxmaster::GenericDevice`, and `DeviceFacade::deviceProfile()`,
which are part of the LXMASTER profile-extension API. You need an **installed LXMASTER package that
includes those** (rebuild + `cmake --install` from the LXMASTER source tree if your installed package
predates them).

## Build

```bash
cmake -B build
cmake --build build -j
```

If LXMASTER is installed to a non-default prefix, point CMake at it with
`-DCMAKE_PREFIX_PATH=/your/prefix`.

## Run

1. Generate an ENI for your bus (`lxmaster eni gen -h`) and set the path in `kEniPath` in
   [`main.cpp`](main.cpp). Adjust `kImuVendorId`/`kImuProductCode` and the resolved CoE object indices
   to match your IMU's ESI.
2. Ensure `LXMASTER_RT_IFACE` is set (via `lxmaster host setup`).
3. Run on the RT host (e.g. `lxmaster run imu_demo`, or the binary directly with the appropriate
   privileges).

Expected output is one line reporting the bound IMU profile, five sample lines, then `Done.`
