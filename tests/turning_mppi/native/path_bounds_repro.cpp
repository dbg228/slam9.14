// Diagnostic only: run the actual installed inline helper under AddressSanitizer.
// This does not instrument or modify the original MPPI baseline libraries.
#include <cstdio>
#include <vector>
#include "nav2_mppi_controller/tools/utils.hpp"

int main() {
  const std::vector<float> distances{0.0f, 0.1f, 0.2f};
  volatile float query = 0.3f;
  const auto index = mppi::utils::findClosestPathPt(distances, query);
  std::printf("index=%zu\n", index);
  return 0;
}
