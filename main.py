#!/usr/bin/env python3
"""
KolebkaSim – Rocking cradle simulation with motorised internal weight.

Run:  python main.py
"""

from __future__ import annotations

import math
import sys

import numpy as np
import pygame

from config import Config
from physics import PhysicsEngine
from controller import SwingController
from renderer import CradleRenderer
from graphs import LiveGraphs
from ui_widgets import Slider, Button

# -----------------------------------------------------------------------
# slider definitions  (label, config_path, min, max, fmt)
# -----------------------------------------------------------------------
SLIDER_DEFS: list[tuple[str, str, float, float, str]] = [
    ("Cradle mass kg",          "cradle.mass",              1.0, 20.0, ".1f"),
    ("CoM height m",            "cradle.cm_height",         0.02, 0.40, ".2f"),
    ("Rocker depth R",          "rocker.R",                 0.01, 0.20, ".3f"),
    ("Rocker width W",          "rocker.W",                 0.05, 0.40, ".2f"),
    ("Shape exp p",             "rocker.p",                 1.0,  2.0,  ".2f"),
    ("Weight mass kg",          "weight.mass",              0.2,  5.0,  ".1f"),
    ("Rail height m",           "weight.rail_height",       0.01, 0.30, ".2f"),
    ("Rail half-span m",        "weight.rail_half_span",    0.05, 0.30, ".2f"),
    ("Motor max accel",         "motor.max_acceleration",   0.5, 10.0, ".1f"),
    ("Motor max vel",           "motor.max_velocity",       0.1,  1.0, ".2f"),
    ("Surface damping",         "surface.damping",          0.01, 1.0, ".2f"),
    ("Target amplitude rad",    "control.target_amplitude", 0.02, 0.5, ".2f"),
    ("Control gain",            "control.gain",             0.0,  1.0, ".2f"),
    ("Sim speed",               "sim.speed",                0.1,  4.0, ".1f"),
]


def _get_cfg_attr(cfg: Config, path: str):
    parts = path.split(".")
    obj = cfg
    for p in parts:
        obj = getattr(obj, p)
    return obj


def _set_cfg_attr(cfg: Config, path: str, value: float):
    parts = path.split(".")
    obj = cfg
    for p in parts[:-1]:
        obj = getattr(obj, p)
    setattr(obj, parts[-1], value)


# -----------------------------------------------------------------------
# main application
# -----------------------------------------------------------------------

class App:
    def __init__(self):
        pygame.init()

        self.cfg = Config()
        self.engine = PhysicsEngine(self.cfg)
        self.controller = SwingController(self.cfg)
        self.renderer = CradleRenderer(self.cfg, self.engine)
        self.graphs = LiveGraphs(self.cfg, max_samples=600)

        dc = self.cfg.display
        self.screen = pygame.display.set_mode((dc.window_width, dc.window_height))
        pygame.display.set_caption("KolebkaSim – Rocking Cradle Simulation")

        self.font = pygame.font.SysFont("monospace", 13)

        # Layout regions
        self.sim_rect = pygame.Rect(0, 0, dc.sim_view_width, dc.window_height)
        panel_w = dc.window_width - dc.sim_view_width
        self.panel_rect = pygame.Rect(dc.sim_view_width, 0,
                                      panel_w, dc.window_height)

        self._build_ui()

        self.clock = pygame.time.Clock()
        self.running = True
        self.paused = False
        self._graph_sample_counter = 0

        # Give a tiny initial push so it starts rocking
        self.engine.reset(theta0=0.02)

    # -------------------------------------------------------------------
    # UI construction
    # -------------------------------------------------------------------

    def _build_ui(self):
        px = self.panel_rect.left + 6
        pw = self.panel_rect.width - 12
        y = 8
        slider_h = 32
        spacing = 2

        self.sliders: list[tuple[str, Slider]] = []

        for label_text, cfg_path, lo, hi, fmt in SLIDER_DEFS:
            val = _get_cfg_attr(self.cfg, cfg_path)
            rect = pygame.Rect(px, y, pw, slider_h)
            sl = Slider(rect, label_text, lo, hi, val, fmt)
            self.sliders.append((cfg_path, sl))
            y += slider_h + spacing

        y += 8

        # Buttons row
        btn_w = (pw - 10) // 4
        self.btn_pause = Button(pygame.Rect(px, y, btn_w, 28), "Pause")
        self.btn_reset = Button(pygame.Rect(px + btn_w + 4, y, btn_w, 28), "Reset")
        self.btn_push = Button(pygame.Rect(px + 2 * (btn_w + 4), y, btn_w, 28), "Push")
        self.btn_ctrl = Button(pygame.Rect(px + 3 * (btn_w + 4), y, btn_w, 28), "Ctrl:ON")
        self.buttons = [self.btn_pause, self.btn_reset, self.btn_push, self.btn_ctrl]
        y += 36

        # Graphs fill the rest
        self.graph_rect = pygame.Rect(
            self.panel_rect.left, y,
            self.panel_rect.width, self.panel_rect.bottom - y)

    # -------------------------------------------------------------------
    # main loop
    # -------------------------------------------------------------------

    def run(self):
        while self.running:
            dt_frame = self.clock.tick(self.cfg.display.fps) / 1000.0
            self._handle_events()

            if not self.paused:
                self._step_simulation(dt_frame)

            self._draw()
            pygame.display.flip()

        pygame.quit()

    # -------------------------------------------------------------------
    # events
    # -------------------------------------------------------------------

    def _handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                return

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self._toggle_pause()
                elif event.key == pygame.K_r:
                    self._reset()
                elif event.key == pygame.K_p:
                    self._push()

            # Sliders
            for cfg_path, sl in self.sliders:
                if sl.handle_event(event):
                    _set_cfg_attr(self.cfg, cfg_path, sl.value)
                    self.renderer._build_body_shape()

            # Buttons
            if self.btn_pause.handle_event(event):
                self._toggle_pause()
            if self.btn_reset.handle_event(event):
                self._reset()
            if self.btn_push.handle_event(event):
                self._push()
            if self.btn_ctrl.handle_event(event):
                self.cfg.control.enabled = not self.cfg.control.enabled
                self.btn_ctrl.text = ("Ctrl:ON" if self.cfg.control.enabled
                                      else "Ctrl:OFF")

    def _toggle_pause(self):
        self.paused = not self.paused
        self.btn_pause.text = "Resume" if self.paused else "Pause"

    def _reset(self):
        self.engine.reset(theta0=0.02)
        self.controller.reset()
        for ch in self.graphs.channels:
            ch.data.clear()
        self._graph_sample_counter = 0

    def _push(self):
        self.engine.state.theta_dot += 0.5

    # -------------------------------------------------------------------
    # simulation step
    # -------------------------------------------------------------------

    def _step_simulation(self, dt_frame: float):
        cfg = self.cfg
        sim_dt = cfg.sim.dt
        sim_time = dt_frame * cfg.sim.speed
        n_steps = max(1, int(sim_time / sim_dt))
        # Cap sub-steps to avoid freezing on slow frames
        n_steps = min(n_steps, 2000)

        graph_interval = max(1, int(1.0 / (60.0 * sim_dt)))

        for _ in range(n_steps):
            s_ddot = self.controller.compute(self.engine.state, sim_dt)
            self.engine.step(s_ddot, sim_dt)

            self._graph_sample_counter += 1
            if self._graph_sample_counter >= graph_interval:
                self._graph_sample_counter = 0
                st = self.engine.state
                self.graphs.push(
                    math.degrees(st.theta),
                    st.s * 100.0,
                    st.motor_accel,
                    self.controller.displacement_fraction,
                )

        self.graphs.update_ranges()

    # -------------------------------------------------------------------
    # rendering
    # -------------------------------------------------------------------

    def _draw(self):
        self.screen.fill((30, 30, 35))

        # Left: cradle visualisation
        self.renderer.draw(self.screen, self.sim_rect)

        # Right panel background
        pygame.draw.rect(self.screen, (40, 40, 45), self.panel_rect)
        pygame.draw.line(self.screen, (80, 80, 85),
                         (self.panel_rect.left, 0),
                         (self.panel_rect.left, self.panel_rect.bottom), 2)

        # Sliders
        for _, sl in self.sliders:
            sl.draw(self.screen, self.font)

        # Buttons
        for btn in self.buttons:
            btn.draw(self.screen, self.font)

        # Controller info
        ctrl = self.controller
        info = (f"Peak: {math.degrees(ctrl.peak_amplitude):.1f}°  "
                f"Disp: {ctrl.displacement_fraction:.2f}")
        txt = self.font.render(info, True, (180, 180, 185))
        self.screen.blit(txt, (self.graph_rect.left + 8, self.graph_rect.top - 16))

        # Graphs
        self.graphs.draw(self.screen, self.graph_rect)


# -----------------------------------------------------------------------
# entry point
# -----------------------------------------------------------------------

def main():
    app = App()
    app.run()


if __name__ == "__main__":
    main()
