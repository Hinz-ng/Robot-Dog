// Host test: mit_law.h's estimator is wrap-safe. Drives the rotor across the raw
// count wrap (ENC_CPR) and the 2^32 accumulator wrap, both directions and both
// signs of dir, and checks p stays continuous and v unaffected.
#include "mit_law.h"

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; if (fails < 20) { printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } } while (0)

// Run `steps` loops of `step` counts from an accumulator parked `before` counts
// below the 2^32 wrap; p must equal the commanded travel exactly.
static void run(int32_t step, int8_t dir, uint32_t before, int steps) {
  MitEstimator e;
  uint16_t raw = (uint16_t)(ENC_CPR - 10);         // starts near the raw wrap too
  mitEstPrime(e, raw);
  e.cnt = (uint32_t)0 - before;                    // accumulator just under 2^32
  mitEstZero(e);
  const float dt = 1.0f / 11500.0f;
  for (int k = 1; k <= steps; k++) {
    raw = (uint16_t)(((int32_t)raw + step + (int32_t)ENC_CPR) % (int32_t)ENC_CPR);
    mitEstUpdate(e, raw, dir, dt, 0.001f);
    const int32_t want = step * k;
    CHECK(mitEstDeltaCnt(e) == want, "step %d dir %d k %d: delta %ld want %ld",
          (int)step, (int)dir, k, (long)mitEstDeltaCnt(e), (long)want);
    const float p_want = (float)(dir * want) * ENC_RAD_PER_COUNT / GEAR_RATIO;
    CHECK(fabsf(mitEstP(e, dir) - p_want) <= 1e-6f + 1e-6f * fabsf(p_want), "p %f want %f", mitEstP(e, dir), p_want);
  }
  // velocity converges to step counts per loop, output frame, sign applied
  const float v_want = (float)(dir * step) * ENC_RAD_PER_COUNT / (GEAR_RATIO * dt);
  CHECK(fabsf(e.v_out_rads - v_want) < 1e-3f * fabsf(v_want) + 1e-4f, "v %f want %f", e.v_out_rads, v_want);
}

int main() {
  run(+37, +1, 500, 2000);       // forward across both wraps
  run(-37, +1, 500, 2000);       // backward (accumulator moves away from the wrap, then the raw wraps)
  run(+37, -1, 500, 2000);       // dir = -1 flips p and v
  run(+123, +1, 1, 600);         // accumulator wraps on the first step
  run(-123, -1, (uint32_t)-1, 600);   // parked just ABOVE 0, moving down through it
  printf(fails ? "*** %d FAILURES ***\n" : "ALL PASS (estimator wrap-safe)\n", fails);
  return fails ? 1 : 0;
}
