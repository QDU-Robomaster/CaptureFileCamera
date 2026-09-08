#include <cstdint>
#include <string_view>
#include <type_traits>

#include "CaptureFileCamera.hpp"

namespace
{
inline constexpr CameraTypes::FrameLayout kLayout{16U, 8U, 48U,
                                                  CameraTypes::Encoding::BGR8};
using Camera = CaptureFileCamera<kLayout>;
using RuntimeParam = Camera::RuntimeParam;
using FrameGeometry = Camera::FrameGeometry;

inline constexpr FrameGeometry kGeometry{
    16U, 8U, 48U, 4U, 6U, 2U, 3U, CameraTypes::FRAME_GEOMETRY_REVERSE_X, 0U, 0.25F, 0.75F,
};

constexpr bool GeometryMatches(const FrameGeometry& geometry)
{
  return geometry.width == 16U && geometry.height == 8U && geometry.step == 48U &&
         geometry.roi_offset_x_native == 4U && geometry.roi_offset_y_native == 6U &&
         geometry.decimation_x == 2U && geometry.decimation_y == 3U &&
         geometry.flags == CameraTypes::FRAME_GEOMETRY_REVERSE_X &&
         geometry.reserved == 0U && geometry.sample_phase_x_native == 0.25F &&
         geometry.sample_phase_y_native == 0.75F;
}

inline constexpr RuntimeParam kDefault{};
inline constexpr RuntimeParam kCurrent{"current_frames.bin",
                                       "current_frames.csv",
                                       "current_imu.csv",
                                       "current_camera",
                                       "current_image",
                                       "current_imu",
                                       false,
                                       true,
                                       37U,
                                       54321U,
                                       kGeometry};
inline constexpr RuntimeParam kLegacy{"legacy_frames.bin",
                                      "legacy_frames.csv",
                                      "legacy_imu.csv",
                                      "legacy_camera",
                                      "legacy_image",
                                      "legacy_imu",
                                      true,
                                      false,
                                      41U,
                                      kGeometry};

static_assert(std::is_default_constructible_v<RuntimeParam>);
static_assert(
    std::is_constructible_v<RuntimeParam, std::string_view, std::string_view,
                            std::string_view, std::string_view, std::string_view,
                            std::string_view, bool, bool, uint32_t, FrameGeometry>);
static_assert(std::is_constructible_v<RuntimeParam, std::string_view, std::string_view,
                                      std::string_view, std::string_view,
                                      std::string_view, std::string_view, bool, bool,
                                      uint32_t, uint32_t, FrameGeometry>);
static_assert(std::is_constructible_v<RuntimeParam, std::string_view, std::string_view,
                                      std::string_view, std::string_view,
                                      std::string_view, std::string_view, bool, bool,
                                      uint32_t, FrameGeometry, double>);

inline constexpr RuntimeParam kSlow{"slow.bin", "slow.csv", "imu.csv", "camera",
                                    "image",    "imu",      true,      true,
                                    6U,         10000U,     kGeometry, 0.5};
inline constexpr RuntimeParam kFast{"fast.bin", "fast.csv", "imu.csv", "camera",
                                    "image",    "imu",      true,      false,
                                    3U,         kGeometry,  2.0};
static_assert(kSlow.replay_speed == 0.5);
static_assert(kFast.replay_speed == 2.0);
static_assert(kCurrent.replay_speed == 1.0);
static_assert(kLegacy.replay_speed == 1.0);
static_assert(kDefault.replay_speed == 1.0);

static_assert(kDefault.file_path == "capture_frames.bin");
static_assert(kDefault.trigger_period_us == Camera::default_trigger_period_us);
static_assert(kDefault.geometry.width == kLayout.width);
static_assert(kDefault.geometry.height == kLayout.height);
static_assert(kDefault.geometry.step == kLayout.step);

static_assert(kCurrent.file_path == "current_frames.bin");
static_assert(kCurrent.frame_csv_path == "current_frames.csv");
static_assert(kCurrent.imu_csv_path == "current_imu.csv");
static_assert(kCurrent.camera_name == "current_camera");
static_assert(kCurrent.image_topic_name == "current_image");
static_assert(kCurrent.imu_topic_name == "current_imu");
static_assert(!kCurrent.realtime);
static_assert(kCurrent.loop);
static_assert(kCurrent.max_frames == 37U);
static_assert(kCurrent.trigger_period_us == 54321U);
static_assert(GeometryMatches(kCurrent.geometry));

static_assert(kLegacy.file_path == "legacy_frames.bin");
static_assert(kLegacy.frame_csv_path == "legacy_frames.csv");
static_assert(kLegacy.imu_csv_path == "legacy_imu.csv");
static_assert(kLegacy.camera_name == "legacy_camera");
static_assert(kLegacy.image_topic_name == "legacy_image");
static_assert(kLegacy.imu_topic_name == "legacy_imu");
static_assert(kLegacy.realtime);
static_assert(!kLegacy.loop);
static_assert(kLegacy.max_frames == 41U);
static_assert(kLegacy.trigger_period_us == Camera::default_trigger_period_us);
static_assert(GeometryMatches(kLegacy.geometry));
}  // namespace

int main() { return 0; }
