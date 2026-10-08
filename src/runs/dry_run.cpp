#include "runs/dry_run.h"
#include "navigation/exploration.h"
#include "utils/logger.h"

static const char* TAG = "dryrun";

bool DryRun::run() {
  Exploration ex(_d, _m);
  if (!ex.begin()) { LOGE(TAG, "cannot initialize exploration"); _d.stop(); return false; }
  const uint32_t t0 = millis();
  LOGI(TAG, "Start-to-End exploration; no return-to-start");
  Exploration::Status st = Exploration::Status::CONTINUE;
  for (uint16_t i = 0; i < cfg::MAX_EXPLORE_STEPS && st == Exploration::Status::CONTINUE; ++i) {
    st = ex.step();
    if (_d.aborted()) { st = Exploration::Status::ABORTED; break; }
  }
  _d.stop();
  if (st != Exploration::Status::COMPLETE || _m.finishId < 0) {
    LOGE(TAG, "dry run incomplete (status=%d, finish=%d); map NOT committed", (int)st, _m.finishId);
    _m.print();
    return false;
  }
  if (!_m.validate(true)) { LOGE(TAG, "map failed validation; map NOT committed"); return false; }
  const bool saved = _m.save();
  _m.print();
  LOGI(TAG, "End reached in %lu ms; map %s", (unsigned long)(millis() - t0), saved ? "committed" : "NOT saved");
  return saved;
}
