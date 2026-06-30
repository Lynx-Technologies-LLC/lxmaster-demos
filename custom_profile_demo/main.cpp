#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include <lxmaster/lxmaster.hpp>

// Direct device-layer headers (installed with the package) for binding a custom profile.
#include "devices/identity_profile.hpp"
#include "devices/profile_registry.hpp"

#include "a6_extended_profile.hpp"

/**
 * Minimal custom-profile demo.
 *
 * Binds a user-defined profile (`demo::A6ExtendedProfile`, a subclass of CiA402DriveProfile) to the
 * StepperOnline A6 servo, brings the bus up, prints the drive's current actual position once, and
 * shuts down. It shows the whole custom-profile lifecycle without any motion: registration via
 * `NetworkConfig::extra_profile_factories`, and reaching the bound profile from the application
 * through `Axis::deviceProfile()`.
 *
 * Edit kEniPath to point at an ENI generated for your bus (`lxmaster eni gen -h`).
 */
namespace {

// StepperOnline A6 servo identity (from the ESI). Same identity the in-tree `stepperonline_a6`
// device class uses; an app-supplied factory is considered AHEAD of the built-ins, so this custom
// profile wins the classification for this run.
constexpr std::uint32_t kA6VendorId = 0x00400000;
constexpr std::uint32_t kA6ProductCode = 0x00000715;

constexpr const char* kEniPath = "/home/user/myenifolder/myenifile.xml";

// Profile builder: construct the custom subclass from the registry's selection input, mirroring the
// built-in `ecdev::makeCiA402DriveProfile`.
std::unique_ptr<ecdev::IDeviceProfile> makeA6Extended(const ecdev::ProfileSelectionInput& in) {
  ecdev::CiA402DriveProfile::Config cfg;
  cfg.op_mode = in.op_mode;
  cfg.auto_fault_reset_and_recover = in.auto_fault_reset_and_recover;
  cfg.startup_fault_autoreset = in.startup_fault_autoreset;
  return std::make_unique<demo::A6ExtendedProfile>(cfg);
}

}  // namespace

int main() {
  // Default EtherCAT network parameters (interface from LXMASTER_RT_IFACE).
  lxmaster::NetworkConfig cfg = lxmaster::NetworkConfig::defaults();

  if (cfg.bus.ifname.empty()) {
    std::cerr << "No EtherCAT interface: set LXMASTER_RT_IFACE in /etc/profile.d/lxmaster-config.sh "
                 "(run lxmaster host setup first).\n";
    return 1;
  }

  cfg.eni.eni_path = kEniPath;

  // Register the custom profile for the A6 identity. No static registration / whole-archive needed:
  // app-supplied factories are honoured ahead of the built-ins for this run.
  cfg.extra_profile_factories.push_back(ecdev::makeIdentityProfileFactory(
      ecdev::DeviceIdentityMatch{kA6VendorId, kA6ProductCode}, makeA6Extended, "cia402:a6-custom"));

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
  drive->configure();  // walk this axis up to OP so cyclic data flows

  if (!net.start()) {
    std::cerr << "EcNetwork::start() failed: " << net.lastError() << "\n";
    return 1;
  }

  // Confirm the application is talking to our custom profile (not the built-in CiA402 class).
  if (auto* mine = dynamic_cast<demo::A6ExtendedProfile*>(drive->deviceProfile())) {
    std::cout << "Custom profile in use: " << mine->profileName() << "\n";
  }

  // Let the RT thread publish a couple of cycles so actualPosition() is meaningful.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  std::cout << "Axis '" << drive->name() << "' actual position = " << drive->actualPosition()
            << " counts\n";

  net.stop();
  std::cout << "Done.\n";
  return 0;
}
