#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include <lxmaster/lxmaster.hpp>

// Direct device-layer headers (installed with the package) for a brand-new device class.
#include "devices/device_profile.hpp"
#include "devices/identity_profile.hpp"
#include "devices/process_image.hpp"
#include "devices/profile_registry.hpp"
#include "devices/slave_services.hpp"

/**
 * Custom profile for an EtherCAT IMU -- a device class that fits NONE of the built-in typed
 * contracts (it is not a drive, an I/O module, or an encoder). It therefore implements
 * `ecdev::IDeviceProfile` directly and deliberately does NOT provide `asMotion()`/`asIo()`/
 * `asEncoder()`, so the library exposes it only through the generic `lxmaster::GenericDevice` handle.
 *
 * It maps six 16-bit TxPDO words (acceleration X/Y/Z, angular rate X/Y/Z), resolved once at PreOP
 * and snapshotted each cycle into lock-free atomics that application threads read through the
 * getters. The CoE object indices below are illustrative -- point them at whatever your IMU's ESI
 * actually maps.
 */
namespace demo {

class ImuProfile : public ecdev::IDeviceProfile {
public:
  const char* profileName() const override { return "vendor:imu"; }

  /** Resolve the mapped TxPDO objects once, before cyclic data flows. */
  std::string configurePreOp(ecdev::ISlaveServices& svc, ecdev::ProcessImage& image) override {
    (void)svc;
    accel_ref_[0] = image.resolve(0x6000, 0x01);
    accel_ref_[1] = image.resolve(0x6000, 0x02);
    accel_ref_[2] = image.resolve(0x6000, 0x03);
    gyro_ref_[0] = image.resolve(0x6010, 0x01);
    gyro_ref_[1] = image.resolve(0x6010, 0x02);
    gyro_ref_[2] = image.resolve(0x6010, 0x03);
    return {};  // non-empty string would abort bring-up with that reason
  }

  /** RT cyclic path: copy mapped words into the lock-free snapshot. No SDOs here. */
  void readInputs(const ecdev::ProcessImage& image, bool wkc_valid, bool operational) override {
    (void)operational;
    if (!wkc_valid) {
      return;  // stale buffer this cycle; keep the previous snapshot
    }
    for (int i = 0; i < 3; ++i) {
      std::int16_t v = 0;
      if (image.readI16(accel_ref_[i], &v)) accel_[i].store(v, std::memory_order_release);
      if (image.readI16(gyro_ref_[i], &v)) gyro_[i].store(v, std::memory_order_release);
    }
  }

  std::int16_t accel(int axis) const noexcept {
    return accel_[axis].load(std::memory_order_acquire);
  }
  std::int16_t gyro(int axis) const noexcept { return gyro_[axis].load(std::memory_order_acquire); }

  /** Static factory matching the `ecdev::ProfileCreateFn` signature used by the registry. */
  static std::unique_ptr<ecdev::IDeviceProfile> make(const ecdev::ProfileSelectionInput& in) {
    (void)in;
    return std::make_unique<ImuProfile>();
  }

private:
  ecdev::PdoEntryRef accel_ref_[3]{};
  ecdev::PdoEntryRef gyro_ref_[3]{};
  std::atomic<std::int16_t> accel_[3]{};
  std::atomic<std::int16_t> gyro_[3]{};
};

}  // namespace demo

/**
 * Brand-new-device-class demo.
 *
 * Binds `demo::ImuProfile` to an EtherCAT IMU, brings it to OPERATIONAL through the generic
 * `lxmaster::GenericDevice` handle (no typed Axis/IoModule/Encoder exists for it), prints a few
 * acceleration/rate samples, and shuts down. Registration is via
 * `NetworkConfig::extra_profile_factories`; the bound profile is reached through
 * `GenericDevice::deviceProfile()`.
 *
 * Edit kEniPath / the identity constants to match your bus (`lxmaster eni gen -h`).
 */
namespace {

constexpr std::uint32_t kImuVendorId = 0x00000ABC;
constexpr std::uint32_t kImuProductCode = 0x00010001;
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
      ecdev::DeviceIdentityMatch{kImuVendorId, kImuProductCode}, demo::ImuProfile::make,
      "vendor:imu"));

  lxmaster::EcNetwork net(cfg);

  if (!net.prepare()) {
    std::cerr << "EcNetwork::prepare() failed: " << net.lastError() << "\n";
    return 1;
  }

  // The IMU has no typed facade; find it (and its custom profile) among the generic handles. This is
  // an RT system with no RTTI on the device path, so recover the concrete type without dynamic_cast:
  // guard on the unique profileName() and static_cast, once here at setup.
  lxmaster::GenericDevice* imu_dev = nullptr;
  demo::ImuProfile* imu = nullptr;
  for (lxmaster::GenericDevice* d : net.devices()) {
    ecdev::IDeviceProfile* p = d->deviceProfile();
    if (p != nullptr && std::strcmp(p->profileName(), "vendor:imu") == 0) {
      imu_dev = d;
      imu = static_cast<demo::ImuProfile*>(p);
      break;
    }
  }
  if (imu == nullptr) {
    std::cerr << "No IMU bound: is the device present and mapped in the ENI?\n";
    return 1;
  }

  // configure() is what raises the device's bring-up ceiling to OPERATIONAL.
  imu_dev->configure();

  if (!net.start()) {
    std::cerr << "EcNetwork::start() failed: " << net.lastError() << "\n";
    return 1;
  }

  std::cout << "IMU '" << imu_dev->name() << "' profile=" << imu->profileName() << "\n";
  for (int i = 0; i < 5; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "accel=[" << imu->accel(0) << ", " << imu->accel(1) << ", " << imu->accel(2)
              << "] gyro=[" << imu->gyro(0) << ", " << imu->gyro(1) << ", " << imu->gyro(2) << "]\n";
  }

  net.stop();
  std::cout << "Done.\n";
  return 0;
}
