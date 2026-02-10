"""
Live rolling graphs drawn directly with pygame.

Displays three time-series graphs stacked vertically:
  1. Tilt angle θ (degrees)
  2. Weight position s (cm)
  3. Motor acceleration (m/s²)
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import pygame
from config import Config


# Colours
COL_BG = (25, 25, 30)
COL_GRID = (55, 55, 60)
COL_AXIS = (100, 100, 105)
COL_LABEL = (180, 180, 185)
COL_ZERO = (70, 70, 75)

# Per-graph trace colours
COL_THETA = (100, 200, 255)
COL_WEIGHT_POS = (255, 120, 80)
COL_MOTOR_ACC = (120, 255, 120)


@dataclass
class GraphChannel:
    """One data channel to graph."""
    name: str
    unit: str
    colour: tuple
    y_range: float            # symmetric range: [-y_range, +y_range]
    data: deque = field(default_factory=lambda: deque(maxlen=600))

    def push(self, value: float):
        self.data.append(value)


class LiveGraphs:
    """Manages and renders live rolling graphs in a pygame rect."""

    def __init__(self, cfg: Config, max_samples: int = 600):
        self.cfg = cfg
        self.max_samples = max_samples

        self.channels: list[GraphChannel] = [
            GraphChannel("Tilt angle", "\u00b0", COL_THETA, 20.0,
                         deque(maxlen=max_samples)),
            GraphChannel("Weight pos", "cm", COL_WEIGHT_POS, 20.0,
                         deque(maxlen=max_samples)),
            GraphChannel("Motor accel", "m/s\u00b2", COL_MOTOR_ACC, 3.0,
                         deque(maxlen=max_samples)),
            GraphChannel("Disp frac", "", (200, 180, 80), 0.5,
                         deque(maxlen=max_samples)),
        ]

        self._font: pygame.font.Font | None = None

    def _get_font(self) -> pygame.font.Font:
        if self._font is None:
            self._font = pygame.font.SysFont("monospace", 12)
        return self._font

    def push(self, theta_deg: float, weight_pos_cm: float,
             motor_accel: float, disp_frac: float = 0.0):
        """Add one data point to all channels."""
        self.channels[0].push(theta_deg)
        self.channels[1].push(weight_pos_cm)
        self.channels[2].push(motor_accel)
        self.channels[3].push(disp_frac)

    def update_ranges(self):
        """Auto-adjust y-ranges to fit the data with some margin."""
        for ch in self.channels:
            if len(ch.data) > 10:
                peak = max(abs(v) for v in ch.data)
                # Round up to a nice number with 20% margin
                desired = peak * 1.3
                if desired < 0.1:
                    desired = 0.1
                # Smooth transition
                ch.y_range = ch.y_range * 0.95 + desired * 0.05

    def draw(self, surface: pygame.Surface, rect: pygame.Rect):
        """Draw all graph channels stacked vertically in the given rect."""
        surface.fill(COL_BG, rect)
        font = self._get_font()

        n_channels = len(self.channels)
        margin_top = 4
        margin_bottom = 2
        spacing = 6
        available = rect.height - margin_top - margin_bottom - spacing * (n_channels - 1)
        graph_h = max(available // n_channels, 30)

        for i, ch in enumerate(self.channels):
            y_top = rect.top + margin_top + i * (graph_h + spacing)
            graph_rect = pygame.Rect(rect.left + 4, y_top,
                                     rect.width - 8, graph_h)
            self._draw_channel(surface, graph_rect, ch, font)

    def _draw_channel(self, surface: pygame.Surface,
                      rect: pygame.Rect, ch: GraphChannel,
                      font: pygame.font.Font):
        """Draw a single graph channel."""
        # Background
        pygame.draw.rect(surface, (35, 35, 40), rect)
        pygame.draw.rect(surface, COL_GRID, rect, 1)

        cx = rect.left
        cy = rect.centery
        w = rect.width
        h = rect.height

        # Zero line
        pygame.draw.line(surface, COL_ZERO,
                         (rect.left, cy), (rect.right, cy), 1)

        # Grid lines (quarter marks)
        for frac in (0.25, 0.75):
            yg = rect.top + int(h * frac)
            pygame.draw.line(surface, COL_GRID,
                             (rect.left, yg), (rect.right, yg), 1)

        # Label
        label_text = f"{ch.name} [{ch.unit}]  ±{ch.y_range:.1f}"
        lbl = font.render(label_text, True, COL_LABEL)
        surface.blit(lbl, (rect.left + 4, rect.top + 2))

        # Y-range labels
        top_val = f"+{ch.y_range:.1f}"
        bot_val = f"-{ch.y_range:.1f}"
        tv = font.render(top_val, True, COL_AXIS)
        bv = font.render(bot_val, True, COL_AXIS)
        surface.blit(tv, (rect.right - tv.get_width() - 4, rect.top + 2))
        surface.blit(bv, (rect.right - bv.get_width() - 4,
                          rect.bottom - bv.get_height() - 2))

        # Plot data
        data = ch.data
        n = len(data)
        if n < 2:
            return

        points = []
        for j in range(n):
            px = rect.right - (n - 1 - j) * (w / self.max_samples)
            # Map value to y pixel
            val = data[j]
            normalized = val / ch.y_range if ch.y_range > 0 else 0.0
            normalized = max(-1.0, min(1.0, normalized))
            py = cy - int(normalized * (h * 0.45))
            points.append((int(px), py))

        if len(points) >= 2:
            pygame.draw.lines(surface, ch.colour, False, points, 2)
