#pragma once

#include "devices/cia402_drive_profile.hpp"
#include "devices/process_image.hpp"

#include <atomic>
#include <cstdint>

/**
 * Minimal custom device profile for the StepperOnline A6 servo.
 *
 * It extends the built-in `ecdev::CiA402DriveProfile` (which is not `final`) instead of
 * reimplementing the motion contract. Because the subclass inherits `asMotion()`, the `Axis`
 * facade keeps working unchanged -- the application still commands/reads motion through `Axis`.
 *
 * As shipped this subclass only renames the profile (to prove a custom class is in use). To expose
 * an EXTRA vendor PDO variable -- some object the ENI maps beyond the standard CiA402 set -- you
 * would override the lifecycle methods below and CHAIN to the base, e.g.:
 *
 *   void resolveTopology(const ecdev::ProcessImage& image) override {
 *     CiA402DriveProfile::resolveTopology(image);     // resolve the standard CiA402 objects
 *     extra_ref_ = image.resolve(0x2001, 0);          // your vendor object (index:sub)
 *   }
 *
 *   void readInputs(const ecdev::ProcessImage& image, bool wkc, bool op) override {
 *     CiA402DriveProfile::readInputs(image, wkc, op);  // base motion snapshot first
 *     std::int32_t v = 0;
 *     if (wkc && image.readI32(extra_ref_, &v)) extra_.store(v, std::memory_order_release);
 *   }
 *
 *   std::int32_t extraValue() const { return extra_.load(std::memory_order_acquire); }
 *
 * The application reaches this subclass through `axis->deviceProfile()` (see main.cpp).
 */
namespace demo {

class A6ExtendedProfile : public ecdev::CiA402DriveProfile {
public:
  using CiA402DriveProfile::CiA402DriveProfile;  // inherit the (Config) constructor

  const char* profileName() const override { return "a6-custom"; }

  // Extra vendor PDO state would live here (resolved ref + lock-free snapshot):
  // private:
  //   ecdev::PdoEntryRef extra_ref_{};
  //   std::atomic<std::int32_t> extra_{0};
};

}  // namespace demo
