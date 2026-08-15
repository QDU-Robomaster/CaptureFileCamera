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

void ExpectParse(const char* text, bool expected, uint32_t expected_rate = 0U)
{
  uint32_t rate = 0U;
  const bool parsed = CaptureFileCameraDetail::ParsePlaybackRateMilli(text, rate);
  Expect(parsed == expected, "playback-rate parse result mismatch");
  if (expected)
  {
    Expect(rate == expected_rate, "playback-rate parsed value mismatch");
  }
}

void ExpectDeadline(uint64_t wall_start_us, uint64_t elapsed_us, uint32_t rate_milli,
                    bool expected, uint64_t expected_deadline_us = 0U)
{
  uint64_t deadline_us = 17U;
  const bool valid = CaptureFileCameraDetail::TryReplayDeadlineUs(
      wall_start_us, elapsed_us, rate_milli, deadline_us);
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
  ExpectParse("1000", true, 1000U);
  ExpectParse("1720", true, 1720U);
  ExpectParse("1000000", true, 1000000U);
  ExpectParse(nullptr, false);
  ExpectParse("", false);
  ExpectParse("0", false);
  ExpectParse("999", false);
  ExpectParse("12x", false);
  ExpectParse("1000001", false);
  ExpectParse("4294967296", false);

  constexpr uint64_t max = std::numeric_limits<uint64_t>::max();
  constexpr uint64_t max_at_1720 = 10724851205645088148ULL;
  ExpectDeadline(0U, 0U, 1720U, true, 0U);
  ExpectDeadline(100U, 10000U, 1000U, true, 10100U);
  ExpectDeadline(100U, 10000U, 1720U, true, 5913U);
  ExpectDeadline(0U, max, 1000U, true, max);
  ExpectDeadline(0U, max, 1720U, true, max_at_1720);
  ExpectDeadline(max - max_at_1720, max, 1720U, true, max);
  ExpectDeadline(max - max_at_1720 + 1U, max, 1720U, false);
  ExpectDeadline(0U, 1U, 999U, false);
  ExpectDeadline(0U, 1U, 1000001U, false);
  TestReplaySlotWaitsForRelease();
  TestReplaySlotWaitStops();

  std::cout << "CaptureFileCamera replay pacing tests passed\n";
  return EXIT_SUCCESS;
}
