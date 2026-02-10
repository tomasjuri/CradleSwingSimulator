"""
Lightweight UI widgets (sliders, buttons) drawn with pure pygame.
No external GUI library needed.
"""

from __future__ import annotations
import pygame

# Colours
C_PANEL_BG = (40, 40, 45)
C_SLIDER_TRACK = (65, 65, 70)
C_SLIDER_FILL = (80, 140, 220)
C_SLIDER_KNOB = (200, 200, 210)
C_SLIDER_KNOB_ACTIVE = (255, 255, 255)
C_LABEL = (185, 185, 190)
C_VALUE = (140, 190, 255)
C_BTN_BG = (60, 65, 75)
C_BTN_HOVER = (80, 90, 110)
C_BTN_TEXT = (210, 210, 215)
C_BTN_BORDER = (100, 105, 115)


class Slider:
    """Horizontal slider with label and value display."""

    def __init__(self, rect: pygame.Rect, label: str,
                 lo: float, hi: float, value: float,
                 fmt: str = ".2f"):
        self.rect = rect          # full widget area (label + slider)
        self.label = label
        self.lo = lo
        self.hi = hi
        self.value = value
        self.fmt = fmt
        self._dragging = False

        # Sub-rects
        self._label_h = 16
        track_y = rect.y + self._label_h + 2
        track_h = rect.height - self._label_h - 4
        self._track_rect = pygame.Rect(rect.x + 4, track_y,
                                       rect.width - 8, track_h)

    @property
    def _knob_x(self) -> int:
        frac = (self.value - self.lo) / max(self.hi - self.lo, 1e-9)
        return int(self._track_rect.x + frac * self._track_rect.width)

    def handle_event(self, event: pygame.event.Event) -> bool:
        """Returns True if the value changed."""
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            # Check if click is on the track area (with some vertical margin)
            expanded = self._track_rect.inflate(0, 10)
            if expanded.collidepoint(event.pos):
                self._dragging = True
                return self._update_from_mouse(event.pos[0])
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            if self._dragging:
                self._dragging = False
        elif event.type == pygame.MOUSEMOTION:
            if self._dragging:
                return self._update_from_mouse(event.pos[0])
        return False

    def _update_from_mouse(self, mx: int) -> bool:
        tr = self._track_rect
        frac = (mx - tr.x) / max(tr.width, 1)
        frac = max(0.0, min(1.0, frac))
        new_val = self.lo + frac * (self.hi - self.lo)
        if new_val != self.value:
            self.value = new_val
            return True
        return False

    def draw(self, surface: pygame.Surface, font: pygame.font.Font):
        # Label + value
        val_str = f"{self.value:{self.fmt}}"
        lbl_surf = font.render(f"{self.label}: {val_str}", True, C_LABEL)
        surface.blit(lbl_surf, (self.rect.x + 4, self.rect.y))

        tr = self._track_rect
        # Track background
        pygame.draw.rect(surface, C_SLIDER_TRACK, tr, border_radius=3)
        # Filled portion
        frac = (self.value - self.lo) / max(self.hi - self.lo, 1e-9)
        fill_w = int(frac * tr.width)
        if fill_w > 0:
            fill_rect = pygame.Rect(tr.x, tr.y, fill_w, tr.height)
            pygame.draw.rect(surface, C_SLIDER_FILL, fill_rect, border_radius=3)
        # Knob
        kx = self._knob_x
        ky = tr.centery
        colour = C_SLIDER_KNOB_ACTIVE if self._dragging else C_SLIDER_KNOB
        pygame.draw.circle(surface, colour, (kx, ky), 6)


class Button:
    """Simple clickable button."""

    def __init__(self, rect: pygame.Rect, text: str):
        self.rect = rect
        self.text = text
        self._hovered = False

    def handle_event(self, event: pygame.event.Event) -> bool:
        """Returns True if the button was clicked."""
        if event.type == pygame.MOUSEMOTION:
            self._hovered = self.rect.collidepoint(event.pos)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                return True
        return False

    def draw(self, surface: pygame.Surface, font: pygame.font.Font):
        bg = C_BTN_HOVER if self._hovered else C_BTN_BG
        pygame.draw.rect(surface, bg, self.rect, border_radius=4)
        pygame.draw.rect(surface, C_BTN_BORDER, self.rect, 1, border_radius=4)
        txt = font.render(self.text, True, C_BTN_TEXT)
        tx = self.rect.centerx - txt.get_width() // 2
        ty = self.rect.centery - txt.get_height() // 2
        surface.blit(txt, (tx, ty))
