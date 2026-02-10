"""
2D side-view renderer for the rocking cradle simulation using pygame.

Draws the cradle body (egg shape), rocker legs, weight, rail, ground,
contact point marker, centre-of-mass marker, and tilt angle indicator.

Body-frame conventions (matching physics.py):
  - origin at the lowest point of the rocker (equilibrium contact)
  - +x right, +y up
  - rocker surface: y = R |x/W|^p  (curves upward from origin)
  - body_to_world uses CW rotation for positive theta (tilt right)
"""

from __future__ import annotations

import math
import numpy as np
import pygame
from config import Config
from physics import PhysicsEngine, rocker_y


# Colour palette
COL_BG = (30, 30, 35)
COL_GROUND = (90, 85, 75)
COL_GROUND_LINE = (140, 130, 115)
COL_BODY_FILL = (70, 100, 160)
COL_BODY_OUTLINE = (120, 160, 220)
COL_ROCKER = (180, 140, 60)
COL_ROCKER_FILL = (110, 90, 40)
COL_RAIL = (160, 160, 160)
COL_WEIGHT = (220, 60, 60)
COL_CONTACT = (255, 220, 50)
COL_CM = (50, 220, 100)
COL_ANGLE_ARC = (200, 200, 200)
COL_GHOST = (60, 60, 65)
COL_TEXT = (200, 200, 200)


class CradleRenderer:
    """Renders the cradle simulation in a pygame surface region."""

    def __init__(self, cfg: Config, engine: PhysicsEngine):
        self.cfg = cfg
        self.engine = engine
        self._build_body_shape()

    # -------------------------------------------------------------------
    # body shape (body-frame polygon)
    # -------------------------------------------------------------------

    def _build_body_shape(self):
        """Build the cradle body as an egg / ellipse in body-frame coords.

        The bottom of the egg sits at  y = R  (top of the rocker curve at
        the widest point), so the rocker legs are visible between the body
        and the ground.
        """
        cfg = self.cfg.cradle
        n = 60
        angles = np.linspace(0, 2 * np.pi, n, endpoint=False)
        w = cfg.body_width
        h = cfg.body_height
        # Egg modifier: top half is slightly narrower
        xs = w * np.cos(angles) * (1.0 - 0.15 * np.sin(angles))
        ys = h * np.sin(angles)

        # Position the egg so its bottom sits at y = rocker.R (top of
        # the rocker curve at the edges).  This leaves the rocker legs
        # visible between the body and the ground.
        R_rocker = self.cfg.rocker.R
        bottom = ys.min()
        ys = ys - bottom + R_rocker  # shift: bottom → R_rocker

        self._body_points = list(zip(xs.tolist(), ys.tolist()))

    # -------------------------------------------------------------------
    # coordinate helpers
    # -------------------------------------------------------------------

    def _world_to_screen(self, wx: float, wy: float,
                         view_rect: pygame.Rect) -> tuple[int, int]:
        ppm = self.cfg.display.pixels_per_meter
        cx = view_rect.centerx
        gy = view_rect.top + int(view_rect.height * 0.65)
        sx = cx + int(wx * ppm)
        sy = gy - int(wy * ppm)
        return sx, sy

    def _body_to_screen(self, bx: float, by: float,
                        view_rect: pygame.Rect) -> tuple[int, int]:
        wx, wy = self.engine.body_to_world(bx, by)
        return self._world_to_screen(wx, wy, view_rect)

    # -------------------------------------------------------------------
    # drawing
    # -------------------------------------------------------------------

    def draw(self, surface: pygame.Surface, view_rect: pygame.Rect):
        clip_prev = surface.get_clip()
        surface.set_clip(view_rect)

        surface.fill(COL_BG, view_rect)

        self._draw_ground(surface, view_rect)
        self._draw_ghost_outline(surface, view_rect)
        self._draw_rocker(surface, view_rect)
        self._draw_body(surface, view_rect)
        self._draw_rail(surface, view_rect)
        self._draw_weight(surface, view_rect)
        self._draw_contact_point(surface, view_rect)
        self._draw_cm_marker(surface, view_rect)
        self._draw_angle_indicator(surface, view_rect)
        self._draw_info_text(surface, view_rect)

        surface.set_clip(clip_prev)

    # -- ground --

    def _draw_ground(self, surface: pygame.Surface, vr: pygame.Rect):
        gy = vr.top + int(vr.height * 0.65)
        ground_rect = pygame.Rect(vr.left, gy, vr.width, vr.bottom - gy)
        surface.fill(COL_GROUND, ground_rect)
        pygame.draw.line(surface, COL_GROUND_LINE,
                         (vr.left, gy), (vr.right, gy), 2)
        for i in range(0, vr.width, 30):
            x = vr.left + i
            pygame.draw.line(surface, COL_GROUND_LINE,
                             (x, gy + 4), (x + 8, gy + 4), 1)

    # -- ghost outline (equilibrium position) --

    def _draw_ghost_outline(self, surface: pygame.Surface, vr: pygame.Rect):
        ppm = self.cfg.display.pixels_per_meter
        cx = vr.centerx
        gy = vr.top + int(vr.height * 0.65)
        # At θ=0 the body-frame origin sits on the ground at (cx, gy).
        points = []
        for bx, by in self._body_points:
            sx = cx + int(bx * ppm)
            sy = gy - int(by * ppm)
            points.append((sx, sy))
        if len(points) >= 3:
            pygame.draw.polygon(surface, COL_GHOST, points, 1)

    # -- rocker legs --

    def _draw_rocker(self, surface: pygame.Surface, vr: pygame.Rect):
        """Draw the rocker curve and connecting legs to the body."""
        # The rocker curve (the ground-touching runner)
        pts = self.engine.get_rocker_curve_points(80)
        screen_pts = []
        for bx, by in pts:
            sx, sy = self._body_to_screen(bx, by, vr)
            screen_pts.append((sx, sy))

        # Also draw "leg" lines from the body bottom corners down to the
        # rocker curve ends, to make the structure visible.
        R = self.cfg.rocker.R
        W = self.cfg.rocker.W

        # Left and right bottom of the body ≈ (±body_width*0.8, R)
        bw = self.cfg.cradle.body_width * 0.8
        leg_top_L = self._body_to_screen(-bw, R, vr)
        leg_top_R = self._body_to_screen(bw, R, vr)
        leg_bot_L = self._body_to_screen(-W, rocker_y(-W, R, W, self.cfg.rocker.p), vr)
        leg_bot_R = self._body_to_screen(W, rocker_y(W, R, W, self.cfg.rocker.p), vr)

        # Draw legs (lines from body to rocker ends)
        pygame.draw.line(surface, COL_ROCKER, leg_top_L, leg_bot_L, 2)
        pygame.draw.line(surface, COL_ROCKER, leg_top_R, leg_bot_R, 2)

        # Draw the rocker curve itself
        if len(screen_pts) >= 2:
            pygame.draw.lines(surface, COL_ROCKER, False, screen_pts, 3)

    # -- cradle body --

    def _draw_body(self, surface: pygame.Surface, vr: pygame.Rect):
        screen_pts = []
        for bx, by in self._body_points:
            sx, sy = self._body_to_screen(bx, by, vr)
            screen_pts.append((sx, sy))
        if len(screen_pts) >= 3:
            pygame.draw.polygon(surface, COL_BODY_FILL, screen_pts)
            pygame.draw.polygon(surface, COL_BODY_OUTLINE, screen_pts, 2)

    # -- rail --

    def _draw_rail(self, surface: pygame.Surface, vr: pygame.Rect):
        half_span = self.cfg.weight.rail_half_span
        rh = self.cfg.weight.rail_height
        # Rail height is in body frame (above rocker origin).
        # Shift up to account for body offset.
        p1 = self._body_to_screen(-half_span, rh, vr)
        p2 = self._body_to_screen(half_span, rh, vr)
        _draw_dashed_line(surface, COL_RAIL, p1, p2, dash_len=8, gap_len=4)
        for pt in (p1, p2):
            pygame.draw.circle(surface, COL_RAIL, pt, 3)

    # -- weight --

    def _draw_weight(self, surface: pygame.Surface, vr: pygame.Rect):
        st = self.engine.state
        wx, wy = st.weight_world
        sx, sy = self._world_to_screen(wx, wy, vr)
        pygame.draw.circle(surface, COL_WEIGHT, (sx, sy), 10)
        pygame.draw.circle(surface, (255, 255, 255), (sx, sy), 10, 2)

    # -- contact point --

    def _draw_contact_point(self, surface: pygame.Surface, vr: pygame.Rect):
        st = self.engine.state
        sx, sy = self._world_to_screen(st.contact_world_x, 0.0, vr)
        pygame.draw.circle(surface, COL_CONTACT, (sx, sy), 5)

    # -- centre of mass --

    def _draw_cm_marker(self, surface: pygame.Surface, vr: pygame.Rect):
        st = self.engine.state
        cx, cy = st.total_cm_world
        sx, sy = self._world_to_screen(cx, cy, vr)
        size = 6
        pygame.draw.line(surface, COL_CM, (sx - size, sy), (sx + size, sy), 2)
        pygame.draw.line(surface, COL_CM, (sx, sy - size), (sx, sy + size), 2)

    # -- angle indicator --

    def _draw_angle_indicator(self, surface: pygame.Surface, vr: pygame.Rect):
        st = self.engine.state
        if abs(st.theta) < 0.001:
            return
        cx_s, cy_s = self._world_to_screen(st.contact_world_x, 0.0, vr)
        radius = 40
        # Vertical reference
        pygame.draw.line(surface, COL_ANGLE_ARC,
                         (cx_s, cy_s), (cx_s, cy_s - radius), 1)
        # Tilted line
        angle = st.theta
        ex = cx_s + int(radius * math.sin(angle))
        ey = cy_s - int(radius * math.cos(angle))
        pygame.draw.line(surface, COL_ANGLE_ARC,
                         (cx_s, cy_s), (ex, ey), 1)
        # Arc
        rect = pygame.Rect(cx_s - radius, cy_s - radius,
                           2 * radius, 2 * radius)
        start_angle = math.pi / 2 - angle
        end_angle = math.pi / 2
        if angle > 0:
            pygame.draw.arc(surface, COL_ANGLE_ARC, rect,
                            start_angle, end_angle, 1)
        else:
            pygame.draw.arc(surface, COL_ANGLE_ARC, rect,
                            end_angle, start_angle, 1)

    # -- info text --

    def _draw_info_text(self, surface: pygame.Surface, vr: pygame.Rect):
        st = self.engine.state
        font = pygame.font.SysFont("monospace", 14)
        lines = [
            f"\u03b8 = {math.degrees(st.theta):+.2f}\u00b0",
            f"\u03b8\u0307 = {st.theta_dot:+.3f} rad/s",
            f"s = {st.s * 100:+.1f} cm",
            f"t = {st.time:.2f} s",
        ]
        x = vr.left + 10
        y = vr.top + 10
        for line in lines:
            txt = font.render(line, True, COL_TEXT)
            surface.blit(txt, (x, y))
            y += 18


# -----------------------------------------------------------------------
# utility
# -----------------------------------------------------------------------

def _draw_dashed_line(surface, colour, start, end,
                      dash_len=6, gap_len=4, width=1):
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)
    if length < 1:
        return
    ux, uy = dx / length, dy / length
    pos = 0.0
    while pos < length:
        seg_end = min(pos + dash_len, length)
        sx = int(x1 + ux * pos)
        sy = int(y1 + uy * pos)
        ex = int(x1 + ux * seg_end)
        ey = int(y1 + uy * seg_end)
        pygame.draw.line(surface, colour, (sx, sy), (ex, ey), width)
        pos += dash_len + gap_len
