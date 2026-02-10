"""
Control algorithm for the rocking cradle – energy-optimal variant.

Minimises weight displacement while maintaining the target rocking amplitude.

Key ideas
---------
1. **Continuous amplitude estimate** from the instantaneous energy:
       A_est = sqrt( θ² + (θ̇ / ω₀)² )
   This is smooth every timestep (no waiting for peak detection).

2. **Displacement fraction** set by a simple proportional law on the
   energy deficit, rate-limited for stability.

3. **Phase-locked drive**: weight target proportional to θ̇, giving
   automatic frequency- and phase-lock to the natural oscillation.
"""

from __future__ import annotations

import math
import numpy as np
from config import Config
from physics import CradleState


class SwingController:
    """Energy-optimal resonance driver for the cradle weight."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        # Smoothed amplitude and frequency
        self._amp_est: float = 0.0
        self._omega_est: float = 3.0       # initial guess (rad/s)
        self._theta_dot_peak: float = 0.05
        self._frac: float = 0.0            # current displacement fraction

        # For omega estimation (half-cycle timing)
        self._prev_sign_theta: float = 0.0
        self._last_zero_t: float = 0.0
        self._half_period: float = 0.5     # initial guess

    def reset(self):
        self._amp_est = 0.0
        self._omega_est = 3.0
        self._theta_dot_peak = 0.05
        self._frac = 0.0
        self._prev_sign_theta = 0.0
        self._last_zero_t = 0.0
        self._half_period = 0.5

    # ------------------------------------------------------------------
    # public
    # ------------------------------------------------------------------

    def compute(self, state: CradleState, dt: float) -> float:
        cfg = self.cfg
        cc = cfg.control
        mc = cfg.motor
        wc = cfg.weight

        if not cc.enabled:
            return self._decel_to_stop(state, mc)

        theta = state.theta
        theta_dot = state.theta_dot
        t = state.time

        # ---- frequency estimation (from zero-crossings of theta) ----
        self._update_omega(theta, t, dt)

        # ---- continuous amplitude estimate ----
        omega = self._omega_est
        raw_amp = math.sqrt(theta * theta + (theta_dot / max(omega, 0.5)) ** 2)
        # Smooth with fast-attack, slow-release
        if raw_amp > self._amp_est:
            a = min(8.0 * dt, 1.0)      # fast attack
        else:
            a = min(1.5 * dt, 1.0)      # slow release
        self._amp_est += a * (raw_amp - self._amp_est)

        # ---- peak theta_dot tracker ----
        abs_td = abs(theta_dot)
        decay = max(1.0 - 0.5 * dt, 0.95)
        self._theta_dot_peak = max(self._theta_dot_peak * decay, abs_td)

        # ---- displacement fraction ----
        target_amp = cc.target_amplitude
        if target_amp > 1e-6:
            # Analytical sustain fraction (from energy balance):
            #   frac_ss = b · ω · A / (m · g · half_span)
            sustain = (cfg.surface.damping * omega * max(self._amp_est, 0.005) /
                       max(wc.mass * cfg.sim.gravity * wc.rail_half_span, 1e-6))
            sustain = min(sustain, 0.15)

            amp_error = target_amp - self._amp_est
            norm_err = amp_error / target_amp   # +1 at rest, 0 at target

            if norm_err > 0.02:
                # Below target: drive proportional to deficit
                # Ramp gently: max extra drive = 0.4 at full deficit
                extra = 0.4 * min(norm_err * 2.0, 1.0)
                target_frac = sustain + extra
            elif norm_err < -0.02:
                # Above target: gently reduce (negative = absorb energy)
                target_frac = sustain * 0.3 + 0.15 * norm_err
                target_frac = max(target_frac, -0.08)
            else:
                # At target: sustain only
                target_frac = sustain

            target_frac = max(-0.08, min(0.5, target_frac))
        else:
            target_frac = 0.0

        # Rate-limit fraction changes (prevents oscillation)
        max_rate = 0.3 * dt    # max change per second = 0.3
        delta = target_frac - self._frac
        delta = max(-max_rate, min(max_rate, delta))
        self._frac += delta
        self._frac = max(-0.08, min(0.5, self._frac))

        effective_frac = cc.gain * self._frac

        # ---- target position: proportional to θ̇ ----
        td_norm = theta_dot / max(self._theta_dot_peak, 1e-4)
        td_norm = max(-1.0, min(1.0, td_norm))

        s_target = effective_frac * td_norm * wc.rail_half_span

        # ---- motor command ----
        return self._position_controller(
            state.s, state.s_dot, s_target, mc, wc)

    # ------------------------------------------------------------------
    # frequency estimator
    # ------------------------------------------------------------------

    def _update_omega(self, theta: float, t: float, dt: float):
        """Estimate natural frequency from theta zero-crossings."""
        sign_th = 1.0 if theta > 0.001 else (-1.0 if theta < -0.001 else 0.0)
        if sign_th != 0 and sign_th != self._prev_sign_theta and t > 0.1:
            half_p = t - self._last_zero_t
            if 0.05 < half_p < 5.0:
                a = 0.3
                self._half_period = a * half_p + (1 - a) * self._half_period
                self._omega_est = math.pi / max(self._half_period, 0.05)
            self._last_zero_t = t
        if sign_th != 0:
            self._prev_sign_theta = sign_th

    # ------------------------------------------------------------------
    # properties
    # ------------------------------------------------------------------

    @property
    def peak_amplitude(self) -> float:
        return self._amp_est

    @property
    def displacement_fraction(self) -> float:
        return self._frac

    # ------------------------------------------------------------------
    # PD position controller
    # ------------------------------------------------------------------

    @staticmethod
    def _position_controller(s, s_dot, s_target, mc, wc) -> float:
        error = s_target - s
        a_max = mc.max_acceleration
        v_max = mc.max_velocity

        kp = 60.0
        kd = 10.0
        s_ddot = kp * error - kd * s_dot
        s_ddot = float(np.clip(s_ddot, -a_max, a_max))

        if s_dot >= v_max and s_ddot > 0:
            s_ddot = 0.0
        elif s_dot <= -v_max and s_ddot < 0:
            s_ddot = 0.0
        if s >= wc.rail_half_span * 0.99 and s_ddot > 0:
            s_ddot = min(s_ddot, 0.0)
        elif s <= -wc.rail_half_span * 0.99 and s_ddot < 0:
            s_ddot = max(s_ddot, 0.0)

        return s_ddot

    @staticmethod
    def _decel_to_stop(state: CradleState, mc) -> float:
        s_ddot = -40.0 * state.s - 10.0 * state.s_dot
        return float(np.clip(s_ddot, -mc.max_acceleration, mc.max_acceleration))
