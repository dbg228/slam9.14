// Equivalent scalar C++ evaluation of controller.GuidedMPPI.rollout.
// No ROS, hardware, randomness, or controller selection in this library.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace {
double clip(double value, double low, double high) {
  return std::min(std::max(value, low), high);
}
double wrap(double value) { return std::atan2(std::sin(value), std::cos(value)); }
struct Location { int index; double offset; };
Location locate(double value, const double *s, int size) {
  if (value <= s[0]) return {0, 0.};
  if (value >= s[size - 1]) return {size - 2, s[size - 1] - s[size - 2]};
  int hi = static_cast<int>(std::upper_bound(s, s + size, value) - s);
  return {hi - 1, value - s[hi - 1]};
}
double interp(Location at, const double *s, const double *values) {
  const int i = at.index;
  // Match numpy.interp: calculate the slope first, then multiply by offset.
  return values[i] + ((values[i + 1] - values[i]) / (s[i + 1] - s[i])) * at.offset;
}
double footprint_clearance(double x, double y, double yaw, const double *obstacles,
                           int obstacle_count, double half_length, double half_width) {
  double distance = std::numeric_limits<double>::infinity();
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (int j = 0; j < obstacle_count; ++j) {
    const double dx = obstacles[3 * j] - x, dy = obstacles[3 * j + 1] - y;
    const double bx = c * dx + s * dy, by = -s * dx + c * dy;
    const double qx = std::abs(bx) - half_length, qy = std::abs(by) - half_width;
    const double candidate = std::hypot(std::max(qx, 0.), std::max(qy, 0.)) +
      std::min(std::max(qx, qy), 0.) - obstacles[3 * j + 2];
    distance = std::min(distance, candidate);
  }
  return distance;
}
}  // namespace

extern "C" int guided_mppi_rollout(
  int count, int steps, int path_count, int delay_steps, int obstacle_count,
  int ideal, int guide_only, const double *path_s, const double *path_x,
  const double *path_y, const double *path_yaw, const double *path_k,
  const double *path_dk, const double *cfg, double initial_progress,
  const double *state, const double *smooth, const double *pending,
  const double *residuals, const double *obstacles,
  double *costs, std::uint8_t *feasible, double *commands, double *min_clearance) {
  if (count < 0 || steps < 1 || path_count < 2 || delay_steps < 0 || obstacle_count < 0) return 1;
  const double dt = cfg[0], speed = cfg[1], vmax = cfg[2], wmax = cfg[3];
  const double accel_v = cfg[4], accel_w = cfg[5], tau_v = cfg[6], tau_w = cfg[7];
  // cfg[8] retains the controller configuration's delay for ABI metadata;
  // preview is determined by the actual already-issued pending queue length.
  const double lateral_gain = cfg[9], heading_gain = cfg[10];
  const double half_length = cfg[11], half_width = cfg[12], required_clearance = cfg[13];
  const double length = path_s[path_count - 1];
  const double alpha_v = 1. - std::exp(-dt / tau_v), alpha_w = 1. - std::exp(-dt / tau_w);
  std::vector<double> delayed_v(steps), delayed_w(steps);
  for (int candidate = 0; candidate < count; ++candidate) {
    double x = state[0], y = state[1], yaw = state[2], v = state[3], w = state[4];
    double sv = smooth[0], sw = smooth[1], progress = initial_progress, total = 0.;
    double minimum = std::numeric_limits<double>::infinity(), e = 0., eh = 0.;
    bool safe = true;
    for (int t = 0; t < steps; ++t) {
      const Location at = locate(progress, path_s, path_count);
      const double rx = interp(at, path_s, path_x), ry = interp(at, path_s, path_y);
      const double tangent = interp(at, path_s, path_yaw), k = interp(at, path_s, path_k);
      e = -std::sin(tangent) * (x - rx) + std::cos(tangent) * (y - ry);
      eh = wrap(yaw - tangent);
      const double preview = progress + (ideal ? 0. : delay_steps * dt) * std::max(v, .05);
      const Location preview_at = locate(preview, path_s, path_count);
      const double kp = interp(preview_at, path_s, path_k), dkp = interp(preview_at, path_s, path_dk);
      double target_v = std::min(speed, std::sqrt(2. * .30 * std::max(length - progress, 0.)));
      target_v = std::min(target_v, wmax / std::max(std::abs(kp), 1e-5));
      double target_w = target_v * kp - lateral_gain * e - heading_gain * eh;
      if (!ideal) target_w += tau_w * (target_v * target_v) * dkp;
      const int offset = (candidate * steps + t) * 2;
      const double rv = residuals[offset], rw = residuals[offset + 1];
      const double uv = guide_only ? 0. : clip(target_v + rv, 0., vmax);
      const double uw = guide_only ? 0. : clip(target_w + rw, -wmax, wmax);
      commands[offset] = uv; commands[offset + 1] = uw;
      if (ideal) { v = uv; w = uw; }
      else {
        sv += clip(uv - sv, -accel_v * dt, accel_v * dt);
        sw += clip(uw - sw, -accel_w * dt, accel_w * dt);
        delayed_v[t] = sv; delayed_w[t] = sw;
        const double dv = t < delay_steps ? pending[t * 2] : delayed_v[t - delay_steps];
        const double dw = t < delay_steps ? pending[t * 2 + 1] : delayed_w[t - delay_steps];
        v += alpha_v * (dv - v); w += alpha_w * (dw - w);
      }
      const double mid = yaw + .5 * w * dt;
      x += v * std::cos(mid) * dt;
      y += v * std::sin(mid) * dt;
      yaw += w * dt;
      progress = clip(progress + v * std::cos(eh) / std::max(.3, 1. - k * e) * dt, 0., length);
      const double dist = footprint_clearance(x, y, yaw, obstacles, obstacle_count, half_length, half_width);
      minimum = std::min(minimum, dist);
      safe = safe && dist >= required_clearance;
      const double curve_weight = 1. + 2. * std::min(std::abs(k), 2.);
      total += dt * (140. * curve_weight * (e * e) + 10. * (eh * eh) +
          12. * ((v - target_v) * (v - target_v)) + .15 * (rv * rv) + .05 * (rw * rw));
      if (obstacle_count) {
        const double close = std::max(.28 - dist, 0.);
        total += dt * 8. * (close * close);
      }
    }
    total += 35. * (e * e) + 3. * (eh * eh) - (obstacle_count ? 8. : 2.) * (progress - initial_progress);
    costs[candidate] = total; feasible[candidate] = safe ? 1 : 0;
    min_clearance[candidate] = minimum;
  }
  return 0;
}
