#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string_view>

#include "CaptureFileCameraInput.hpp"

namespace
{
void Expect(bool condition, std::string_view message)
{
  if (!condition)
  {
    std::cerr << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

void ExpectDeadline(uint64_t wall_start_us, uint64_t elapsed_us, double replay_speed,
                    bool expected, uint64_t expected_deadline_us = 0U)
{
  uint64_t deadline_us = 17U;
  const bool valid = CaptureFileCameraDetail::TryReplayDeadlineUs(
      wall_start_us, elapsed_us, replay_speed, deadline_us);
  Expect(valid == expected, "replay deadline validity mismatch");
  if (expected)
  {
    Expect(deadline_us == expected_deadline_us, "replay deadline value mismatch");
  }
  else
  {
    Expect(deadline_us == 17U, "failed replay deadline must preserve output");
  }
}

void TestReplaySlotWaitsForRelease()
{
  int frame = 0;
  uint32_t acquire_count = 0U;
  uint32_t backoff_count = 0U;
  bool running = true;
  int* acquired = CaptureFileCameraDetail::WaitForReplaySlot(
      [&]() -> int*
      {
        ++acquire_count;
        return acquire_count == 4U ? &frame : nullptr;
      },
      [&]() { return running; }, [&]() { ++backoff_count; });

  Expect(acquired == &frame, "replay slot wait must return the released slot");
  Expect(acquire_count == 4U, "replay slot wait must retry the same admission");
  Expect(backoff_count == 3U, "replay slot wait must back off after each miss");
}

void TestReplaySlotWaitStops()
{
  uint32_t acquire_count = 0U;
  uint32_t backoff_count = 0U;
  bool running = true;
  int* acquired = CaptureFileCameraDetail::WaitForReplaySlot(
      [&]() -> int*
      {
        ++acquire_count;
        return nullptr;
      },
      [&]() { return running; },
      [&]()
      {
        ++backoff_count;
        running = false;
      });

  Expect(acquired == nullptr, "stopped replay slot wait must return null");
  Expect(acquire_count == 1U, "stopped replay slot wait must not retry after stop");
  Expect(backoff_count == 1U, "stopped replay slot wait must back off once");
}
}  // namespace

int main()
{
  constexpr uint64_t max = std::numeric_limits<uint64_t>::max();
  ExpectDeadline(0U, 0U, 0.5, true, 0U);
  ExpectDeadline(100U, 10000U, 1.0, true, 10100U);
  ExpectDeadline(100U, 10000U, 1.72, true, 5914U);
  ExpectDeadline(100U, 10000U, 0.5, true, 20100U);
  ExpectDeadline(100U, 10000U, 2.0, true, 5100U);
  ExpectDeadline(100U, 1U, 2.0, true, 101U);
  ExpectDeadline(0U, 1000U, 0.001, true, 1000000U);
  ExpectDeadline(0U, 10000U, 2000.0, true, 5U);
  ExpectDeadline(0U, max, 1.0, true, max);
  ExpectDeadline(1U, max, 1.0, false);
  ExpectDeadline(0U, max, 0.5, false);
  ExpectDeadline(max - 19U, 10U, 0.5, false);
  ExpectDeadline(0U, 1U, 0.0, false);
  ExpectDeadline(0U, 1U, -1.0, false);
  ExpectDeadline(0U, 1U, std::numeric_limits<double>::quiet_NaN(), false);
  ExpectDeadline(0U, 1U, std::numeric_limits<double>::infinity(), false);
  ExpectDeadline(0U, max, std::numeric_limits<double>::denorm_min(), false);
  ExpectDeadline(0U, 0U, std::numeric_limits<double>::denorm_min(), true, 0U);
  TestReplaySlotWaitsForRelease();
  TestReplaySlotWaitStops();

  std::cout << "CaptureFileCamera replay pacing tests passed\n";
  return EXIT_SUCCESS;
}
