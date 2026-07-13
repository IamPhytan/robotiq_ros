#ifndef ROBOTIQ_TSF_DEVICE_AUTODETECT_HPP
#define ROBOTIQ_TSF_DEVICE_AUTODETECT_HPP

#include <string>

namespace robotiq_tsf
{

// Default udev-symlink path used by sensor_install.sh.
constexpr const char *kDefaultSensorDevice = "/dev/rq_tsf85_0";

// Search the system for a connected Robotiq tactile sensor. Tries, in order:
//   1. /dev/rq_tsf85_* udev symlinks
//   2. /dev/ttyACM* and /dev/ttyUSB* nodes whose USB descriptor strings
//      match a known sensor manufacturer/product.
// Returns the absolute device path on success, or an empty string if nothing
// matched.
std::string autoDetectSensorDevice();

// Testable core of the above: scan `dev_root` for the candidate device nodes
// and match their USB descriptor strings under `sysfs_tty_root` (the
// /sys/class/tty equivalent).
std::string autoDetectSensorDevice(const std::string &dev_root,
                                   const std::string &sysfs_tty_root);

// True if `device` refers to a path that currently exists on the filesystem.
bool deviceExists(const std::string &device);

// Internals exposed for unit tests.
namespace detail
{
std::string trimWhitespace(const std::string &value);
bool matchesKnownUsbString(const std::string &value);
bool deviceMatchesKnownSensor(const std::string &device_path,
                              const std::string &sysfs_tty_root);
}  // namespace detail

}  // namespace robotiq_tsf

#endif  // ROBOTIQ_TSF_DEVICE_AUTODETECT_HPP
