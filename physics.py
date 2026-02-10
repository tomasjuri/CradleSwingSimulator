"""
Physics engine for the rocking cradle simulation.

Conventions
-----------
- **theta > 0  =  tilt RIGHT** (clockwise viewed from the front).
- **Body frame**: origin at the lowest point of the rocker (equilibrium
  contact point).  +x right, +y up (into the cradle body).
- **Rocker surface** in body frame:  y(x) = R * |x / W|^p   (≥ 0,
  curves *upward* from the contact point).
- **Body-to-world rotation** for a CW tilt by theta:
      R_cw(θ) = [[ cos θ,  sin θ],
                 [-sin θ,  cos θ]]
  i.e.  world = origin + R_cw @ body_vec
- **Torque sign**:  positive torque  =  CW  =  direction of increasing θ.
  tau_gravity_CW = + M g r_x   where r_x is the world-frame horizontal
  distance from the contact point to the mass.
- **Rolling constraint**: the arc length along the rocker curve from x=0
  to the contact point equals the horizontal displacement of the contact
  on the flat ground.

The rocker profile in the body frame is:
    y(x) = R * |x / W|^p
where R = depth, W = half-width, p = shape exponent
(p = 2 → parabolic / circular,  p → 1 → V-shape).
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass
from config import Config


# ---------------------------------------------------------------------------
# Rocker profile helpers
# ---------------------------------------------------------------------------

def rocker_y(x: float | np.ndarray, R: float, W: float, p: float):
    """Height of the rocker curve at position x (body frame, ≥ 0)."""
    return R * np.abs(x / W) ** p


def rocker_dy(x: float, R: float, W: float, p: float) -> float:
    """dy/dx of the rocker curve."""
    if abs(x) < 1e-15:
        return 0.0
    return R * p / W * np.abs(x / W) ** (p - 1) * np.sign(x)


def rocker_ddy(x: float, R: float, W: float, p: float) -> float:
    """d²y/dx² of the rocker curve."""
    if p >= 2.0:
        if abs(x) < 1e-15:
            return R * p * (p - 1) / (W * W)
        return R * p * (p - 1) / (W * W) * np.abs(x / W) ** (p - 2)
    else:
        eps = 1e-6 * W
        xc = max(abs(x), eps)
        return R * p * (p - 1) / (W * W) * (xc / W) ** (p - 2)


def _find_contact_x(theta: float, R: float, W: float, p: float) -> float:
    """
    Find x on the rocker curve whose tangent slope equals tan(theta).

    For y = R |x/W|^p the slope is  dy/dx = (Rp/W)|x/W|^{p-1} sign(x),
    so for theta > 0 (tilt right) the contact is at positive x:
        x_c = W * (W tan θ / (R p))^{1/(p-1)}
    """
    tan_th = np.tan(np.clip(theta, -1.2, 1.2))   # safety clamp
    if abs(tan_th) < 1e-14:
        return 0.0

    sign = np.sign(tan_th)
    abs_tan = abs(tan_th)
    coeff = R * p / W
    if coeff < 1e-15:
        return 0.0

    ratio = abs_tan / coeff

    if abs(p - 1.0) < 1e-9:
        # V-shape: slope is constant ±R/W; contact stays near vertex.
        return sign * 1e-6 * W

    exponent = 1.0 / (p - 1.0)
    xc = W * ratio ** exponent
    xc = min(xc, W * 0.999)
    return sign * xc


def _arc_length(x_end: float, R: float, W: float, p: float,
                n_steps: int = 40) -> float:
    """Arc length of the rocker curve from x = 0 to x = x_end (trapezoidal)."""
    if abs(x_end) < 1e-15:
        return 0.0
    xe = abs(x_end)
    xs = np.linspace(0.0, xe, n_steps + 1)
    # vectorised slope (all positive since xe > 0)
    slopes = R * p / W * (xs / W) ** (p - 1)
    slopes[0] = 0.0  # avoid 0^(neg) for p < 1
    integrand = np.sqrt(1.0 + slopes ** 2)
    return float(np.trapezoid(integrand, xs))


# ---------------------------------------------------------------------------
# CW rotation helper
# ---------------------------------------------------------------------------

def _rot_cw(bx: float, by: float, cos_t: float, sin_t: float):
    """Apply R_cw(θ) = [[cos, sin],[-sin, cos]] to (bx, by)."""
    return (bx * cos_t + by * sin_t,
            -bx * sin_t + by * cos_t)


# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------

@dataclass
class CradleState:
    """Full simulation state."""
    theta: float = 0.0
    theta_dot: float = 0.0
    s: float = 0.0            # weight position on rail (body frame, m)
    s_dot: float = 0.0        # weight velocity on rail (body frame, m/s)
    time: float = 0.0

    # Derived / cached ---------------------------------------------------
    contact_x: float = 0.0
    contact_world_x: float = 0.0
    cradle_origin_world: tuple = (0.0, 0.0)
    weight_world: tuple = (0.0, 0.0)
    total_cm_world: tuple = (0.0, 0.0)
    motor_accel: float = 0.0

    def copy(self) -> "CradleState":
        return CradleState(
            theta=self.theta, theta_dot=self.theta_dot,
            s=self.s, s_dot=self.s_dot, time=self.time,
            contact_x=self.contact_x,
            contact_world_x=self.contact_world_x,
            cradle_origin_world=self.cradle_origin_world,
            weight_world=self.weight_world,
            total_cm_world=self.total_cm_world,
            motor_accel=self.motor_accel,
        )


# ---------------------------------------------------------------------------
# Physics engine
# ---------------------------------------------------------------------------

class PhysicsEngine:
    """Integrates the rocking-cradle equations of motion."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.state = CradleState()
        self._update_derived()

    # -- public helpers ----------------------------------------------------

    def reset(self, theta0: float = 0.0, theta_dot0: float = 0.0):
        self.state = CradleState(theta=theta0, theta_dot=theta_dot0)
        self._update_derived()

    def get_rocker_curve_points(self, n: int = 80) -> np.ndarray:
        """(n, 2) rocker curve in *body* frame (y ≥ 0, curves upward)."""
        R, W, p = self.cfg.rocker.R, self.cfg.rocker.W, self.cfg.rocker.p
        xs = np.linspace(-W, W, n)
        ys = rocker_y(xs, R, W, p)       # positive – curves upward
        return np.column_stack([xs, ys])

    # -- body ↔ world transforms ------------------------------------------

    def _body_to_world(self, bx: float, by: float, theta: float,
                       ox: float, oy: float):
        """Body-frame → world using CW rotation."""
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        rx, ry = _rot_cw(bx, by, cos_t, sin_t)
        return ox + rx, oy + ry

    def body_to_world(self, bx: float, by: float):
        """Transform using current state."""
        ox, oy = self.state.cradle_origin_world
        return self._body_to_world(bx, by, self.state.theta, ox, oy)

    # -- contact geometry --------------------------------------------------

    def _compute_contact(self, theta: float):
        """
        Returns (xc, ground_x, origin_wx, origin_wy).

        xc        – x on the rocker curve at the contact point (body frame)
        ground_x  – world x of the contact point on the flat ground
        origin_w  – world position of the body-frame origin
        """
        R = self.cfg.rocker.R
        W = self.cfg.rocker.W
        p = self.cfg.rocker.p

        xc = _find_contact_x(theta, R, W, p)
        yc = rocker_y(xc, R, W, p)          # positive (above origin)

        arc = _arc_length(xc, R, W, p)
        ground_x = np.sign(theta) * arc if abs(theta) > 1e-15 else 0.0

        # Contact in body frame is (xc, yc).
        # In world frame the contact sits on the ground: (ground_x, 0).
        # world_contact = origin + R_cw @ (xc, yc)
        # ⇒  origin = (ground_x, 0) − R_cw @ (xc, yc)
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        rcx, rcy = _rot_cw(xc, yc, cos_t, sin_t)

        origin_wx = ground_x - rcx
        origin_wy = 0.0 - rcy

        return xc, ground_x, origin_wx, origin_wy

    # -- derived quantities ------------------------------------------------

    def _update_derived(self):
        st = self.state
        cfg = self.cfg

        xc, gx, ox, oy = self._compute_contact(st.theta)
        st.contact_x = xc
        st.contact_world_x = gx
        st.cradle_origin_world = (ox, oy)

        # Weight world position  (rail at height rail_height in body frame)
        st.weight_world = self._body_to_world(
            st.s, cfg.weight.rail_height, st.theta, ox, oy)

        # Cradle CoM world position  (body frame: (0, cm_height))
        cm_cradle = self._body_to_world(
            0.0, cfg.cradle.cm_height, st.theta, ox, oy)

        M = cfg.cradle.mass
        m = cfg.weight.mass
        total = M + m
        st.total_cm_world = (
            (M * cm_cradle[0] + m * st.weight_world[0]) / total,
            (M * cm_cradle[1] + m * st.weight_world[1]) / total,
        )

    # -- equations of motion -----------------------------------------------

    def _compute_theta_ddot(self, theta: float, theta_dot: float,
                            s: float, s_dot: float, s_ddot: float) -> float:
        """
        Angular acceleration about the instantaneous contact point.

        Torque convention: positive = CW = direction of increasing θ.
        """
        cfg = self.cfg
        g = cfg.sim.gravity
        M = cfg.cradle.mass
        m = cfg.weight.mass

        xc, ground_x, ox, oy = self._compute_contact(theta)
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)

        # Vector from contact point to each mass, in world frame.
        def rel_world(bx, by):
            rx, ry = _rot_cw(bx - xc, by - rocker_y(xc, cfg.rocker.R,
                              cfg.rocker.W, cfg.rocker.p),
                              cos_t, sin_t)
            return rx, ry

        # Cradle CoM:  body (0, cm_height)
        rcx, rcy = rel_world(0.0, cfg.cradle.cm_height)
        # Weight:      body (s, rail_height)
        rwx, rwy = rel_world(s, cfg.weight.rail_height)

        # --- Gravitational torques (CW positive) -------------------------
        # Gravity F = (0, -mg).  Standard CCW torque = rx*(-mg).
        # CW torque = −CCW = +mg*rx.
        tau_grav_cradle = M * g * rcx
        tau_grav_weight = m * g * rwx

        # --- Moment of inertia about contact ------------------------------
        a = cfg.cradle.body_width
        b = cfg.cradle.body_height
        I_cm_cradle = M / 4.0 * (a ** 2 + b ** 2)
        I_cradle = I_cm_cradle + M * (rcx ** 2 + rcy ** 2)
        I_weight = m * (rwx ** 2 + rwy ** 2)
        I_total = I_cradle + I_weight
        if I_total < 1e-12:
            return 0.0

        # --- Coriolis coupling (weight moving on rail) --------------------
        # dI/dt from weight changing position → angular-momentum coupling
        tau_coriolis = -2.0 * m * s * s_dot * theta_dot

        # --- Motor reaction torque ----------------------------------------
        # The motor pushes the weight along the rail; the reaction on the
        # cradle acts in the opposite direction.  The torque about the
        # contact is  F_reaction × perpendicular distance.
        # Perpendicular distance from contact to rail (body frame):
        yc_body = rocker_y(xc, cfg.rocker.R, cfg.rocker.W, cfg.rocker.p)
        perp = cfg.weight.rail_height - yc_body
        tau_motor = -m * s_ddot * perp

        # --- Damping ------------------------------------------------------
        tau_damp = -cfg.surface.damping * theta_dot

        # --- Total --------------------------------------------------------
        tau = (tau_grav_cradle + tau_grav_weight +
               tau_coriolis + tau_motor + tau_damp)

        return tau / I_total

    # -- integration -------------------------------------------------------

    def step(self, s_ddot_cmd: float, dt: float | None = None):
        """Advance one timestep (RK4).  s_ddot_cmd is the motor command."""
        if dt is None:
            dt = self.cfg.sim.dt

        cfg = self.cfg
        st = self.state

        a_max = cfg.motor.max_acceleration
        v_max = cfg.motor.max_velocity
        s_half = cfg.weight.rail_half_span
        max_th = cfg.sim.max_theta

        s_ddot = float(np.clip(s_ddot_cmd, -a_max, a_max))

        y = np.array([st.theta, st.theta_dot, st.s, st.s_dot])

        def derivs(yc):
            th, thd, sc, scd = yc

            # Rail limits
            scd_c = float(np.clip(scd, -v_max, v_max))
            sd = s_ddot
            if sc >= s_half and scd_c >= 0:
                scd_c = 0.0
                sd = min(sd, 0.0)
            elif sc <= -s_half and scd_c <= 0:
                scd_c = 0.0
                sd = max(sd, 0.0)

            # Theta limits  (elastic wall)
            if th >= max_th and thd > 0:
                thd = 0.0
            elif th <= -max_th and thd < 0:
                thd = 0.0

            th_dd = self._compute_theta_ddot(th, thd, sc, scd_c, sd)

            return np.array([thd, th_dd, scd_c, sd])

        k1 = derivs(y)
        k2 = derivs(y + 0.5 * dt * k1)
        k3 = derivs(y + 0.5 * dt * k2)
        k4 = derivs(y + dt * k3)
        y_new = y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        # Hard clamps
        y_new[0] = float(np.clip(y_new[0], -max_th, max_th))
        y_new[2] = float(np.clip(y_new[2], -s_half, s_half))
        y_new[3] = float(np.clip(y_new[3], -v_max, v_max))

        # Zero velocity at walls
        if y_new[0] >= max_th and y_new[1] > 0:
            y_new[1] = 0.0
        elif y_new[0] <= -max_th and y_new[1] < 0:
            y_new[1] = 0.0
        if y_new[2] >= s_half and y_new[3] > 0:
            y_new[3] = 0.0
        elif y_new[2] <= -s_half and y_new[3] < 0:
            y_new[3] = 0.0

        st.theta, st.theta_dot, st.s, st.s_dot = y_new
        st.time += dt
        st.motor_accel = s_ddot

        self._update_derived()
