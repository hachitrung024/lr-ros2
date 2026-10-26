#include <GeographicLib/Geoid.hpp>
#include <limits>
// Tiny C ABI lets Python use the same EGM96 dataset/conversion as MAVROS.
extern "C" double lr_geoid_height(double latitude, double longitude) {
  try {
    static GeographicLib::Geoid geoid("egm96-5", "", true, true);
    return geoid(latitude, longitude);
  } catch (...) {
    return std::numeric_limits<double>::quiet_NaN();
  }
}
