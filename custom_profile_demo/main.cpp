#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include <lxmaster/lxmaster.hpp>

// Direct device-layer headers (installed with the package) for binding a custom profile.
#include "devices/cia402_drive_profile.hpp"
#include "devices/identity_profile.hpp"
#include "devices/profile_registry.hpp"

/**
 * Minimal custom device profile for the StepperOnline A6 servo.
 *
 * It extends the built-in `ecdev::CiA402DriveProfile` (which is not `final`) instead of
 * reimplementing the motion contract. Because the subclass inherits `asMotion()`, the `Axis`
 * facade keeps working unchanged -- the application still commands/reads motion through `Axis`.
 *
 * As shipped this subclass only renames the profile (to prove a custom class is in use). To expose
 * an EXTRA vendor PDO variable -- some object the ENI maps beyond the standard CiA402 set -- override
 * the lifecycle methods below and CHAIN to the base, e.g.:
 *
 *   void readInputs(const ecdev::ProcessImage& image, bool wkc, bool op) override {
 *     CiA402DriveProfile::readInputs(image, wkc, op);  // base motion snapshot first
 *     std::int32_t v = 0;
 *     if (wkc && image.readI32(extra_ref_, &v)) extra_.store(v, std::memory_order_release);
 *   }
 *
 * The application reaches this subclass through `axis->deviceProfile()` (see main() below).
 */
namespace demo {

class A6ExtendedProfile : public ecdev::CiA402DriveProfile {
public:
  using CiA402DriveProfile::CiA402DriveProfile;

  const char* profileName() const override { return "a6-custom"; }

  /** Build from registry selection input (mirrors `ecdev::makeCiA402DriveProfile`). */
  static std::unique_ptr<ecdev::IDeviceProfile> make(const ecdev::ProfileSelectionInput& in) {
    ecdev::CiA402DriveProfile::Config cfg;
    cfg.op_mode = in.op_mode;
    cfg.auto_fault_reset_and_recover = in.auto_fault_reset_and_recover;
    cfg.startup_fault_autoreset = in.startup_fault_autoreset;
    return std::make_unique<A6ExtendedProfile>(cfg);
  }

  // Extra vendor PDO state would live here (resolved ref + lock-free snapshot):
  // private:
  //   ecdev::PdoEntryRef extra_ref_{};
  //   std::atomic<std::int32_t> extra_{0};
};

}  // namespace demo

/**
 * Minimal custom-profile demo.
 *
 * Binds `demo::A6ExtendedProfile` to the StepperOnline A6 servo, brings the bus up, prints the
 * drive's current actual position once, and shuts down. Registration is via
 * `NetworkConfig::extra_profile_factories`; the bound profile is reached through
 * `Axis::deviceProfile()`.
 *
 * Edit kEniPath to point at an ENI generated for your bus (`lxmaster eni gen -h`).
 */
namespace {

constexpr std::uint32_t kA6VendorId = 0x00400000;
constexpr std::uint32_t kA6ProductCode = 0x00000715;
constexpr const char* kEniPath = "/home/user/myenifolder/myenifile.xml";

}  // namespace

int main() {
  lxmaster::NetworkConfig cfg = lxmaster::NetworkConfig::defaults();

  if (cfg.bus.ifname.empty()) {
    std::cerr << "No EtherCAT interface: set LXMASTER_RT_IFACE in /etc/profile.d/lxmaster-config.sh "
                 "(run lxmaster host setup first).\n";
    return 1;
  }

  cfg.eni.eni_path = kEniPath;

  cfg.extra_profile_factories.push_back(ecdev::makeIdentityProfileFactory(
      ecdev::DeviceIdentityMatch{kA6VendorId, kA6ProductCode}, demo::A6ExtendedProfile::make,
      "cia402:a6-custom"));

  lxmaster::EcNetwork net(cfg);

  if (!net.prepare()) {
    std::cerr << "EcNetwork::prepare() failed: " << net.lastError() << "\n";
    return 1;
  }

  std::vector<lxmaster::Axis*> axes = net.axes();
  if (axes.empty()) {
    std::cerr << "ENI produced no motion axes (is the A6 present and mapped in the ENI?).\n";
    return 1;
  }

  lxmaster::Axis* drive = axes.front();
  drive->setDriveMode(lxmaster::DriveOpMode::Csp);
  drive->configure();

  if (!net.start()) {
    std::cerr << "EcNetwork::start() failed: " << net.lastError() << "\n";
    return 1;
  }

  // RTTI-free downcast: this is an RT system, so guard on the unique profileName() and static_cast
  // (done once here at setup, not on any cyclic path).
  ecdev::IDeviceProfile* prof = drive->deviceProfile();
  if (prof != nullptr && std::strcmp(prof->profileName(), "a6-custom") == 0) {
    auto* mine = static_cast<demo::A6ExtendedProfile*>(prof);
    std::cout << "Custom profile in use: " << mine->profileName() << "\n";
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  std::cout << "Axis '" << drive->name() << "' actual position = " << drive->actualPosition()
            << " counts\n";

  net.stop();
  std::cout << "Done.\n";
  return 0;
}
