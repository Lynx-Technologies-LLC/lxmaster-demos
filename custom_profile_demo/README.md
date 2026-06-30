# custom_profile_demo

Minimal example of a **custom device profile**. It binds a user-defined profile
(`demo::A6ExtendedProfile`, a subclass of `ecdev::CiA402DriveProfile`) to a StepperOnline A6 servo,
brings the bus up, prints the drive's current actual position once, and shuts down.

It demonstrates the whole custom-profile lifecycle without any motion:

- **Extend, don't reimplement.** `A6ExtendedProfile` subclasses the built-in `CiA402DriveProfile`
  (which is not `final`) and overrides only what it needs, chaining to the base. Because the subclass
  inherits `asMotion()`, the `Axis` facade keeps working unchanged. To expose an extra vendor PDO
  variable you override `readInputs` (and optionally `resolveTopology`) and snapshot your object into
  your own atomics -- see the commented template in [`main.cpp`](main.cpp).
- **Static factory.** `A6ExtendedProfile::make` maps `ProfileSelectionInput` to a profile instance;
  `main()` passes it to `makeIdentityProfileFactory` (same role as `ecdev::makeCiA402DriveProfile`).
- **Register for one run.** `main.cpp` pushes a factory onto `NetworkConfig::extra_profile_factories`,
  which is considered ahead of the built-in device classes, so no static registration or
  whole-archive linking is needed.
- **Reach it from the app.** The application recovers the custom type from `Axis::deviceProfile()`
  without RTTI -- this is an RT system, so it guards on the unique `profileName()` and `static_cast`s
  once at setup (no `dynamic_cast`), then uses its extra API.

## Requirements

This demo uses `CiA402DriveProfile` subclassing and `DeviceFacade::deviceProfile()`, which are part
of the LXMASTER profile-extension API. You need an **installed LXMASTER package that includes those**
(rebuild + `cmake --install` from the LXMASTER source tree if your installed package predates them).

## Build

```bash
cmake -B build
cmake --build build -j
```

If LXMASTER is installed to a non-default prefix, point CMake at it with
`-DCMAKE_PREFIX_PATH=/your/prefix`.

## Run

1. Generate an ENI for your bus (`lxmaster eni gen -h`) and set the path in `kEniPath` in
   [`main.cpp`](main.cpp) (or edit it to read from argv).
2. Ensure `LXMASTER_RT_IFACE` is set (via `lxmaster host setup`).
3. Run on the RT host (e.g. `lxmaster run custom_profile_demo`, or the binary directly with the
   appropriate privileges).

Expected output is one line reporting the bound custom profile and one line with the axis actual
position, then `Done.`
