#include "dds_capture.h"
#include <cassert>

int main() {
  go2_diagnostic::DdsCapture observer;
  assert(observer.lowcmd_count() == 0);
  assert(observer.records().captures().empty());
  return 0;
}
