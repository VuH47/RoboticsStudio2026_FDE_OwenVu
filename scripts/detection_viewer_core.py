# #!/usr/bin/env python3
# """Pure OpenCV layout helpers for the combined Parrot + Husky detection window.

# No ROS imports: this module can be tested with synthetic frames. The ROS node
# (detection_viewer.py) only feeds it images and status text.

# Layout (default 480x360 panes):

#     +------------------------+------------------------+
#     | PARROT  aerial detect  | HUSKY  visual confirm  |  <- title bars
#     |  (letterboxed frame)   |  (letterboxed frame)   |
#     +------------------------+------------------------+
#     | status line 1 (Parrot detection)                |  <- status bar
#     | status line 2 (Husky confirm)                   |
#     +-------------------------------------------------+
# """

# from __future__ import annotations

# from dataclasses import dataclass
# from typing import List, Optional, Sequence, Tuple

# import cv2
# import numpy as np

# BG = (24, 24, 24)
# TITLE_BG = (48, 48, 48)
# WHITE = (235, 235, 235)
# GREY = (150, 150, 150)
# GREEN = (0, 220, 0)
# AMBER = (0, 180, 255)
# RED = (0, 0, 220)

# TITLE_H = 28
# STATUS_LINE_H = 24
# STATUS_PAD = 8


# @dataclass(frozen=True)
# class DashboardConfig:
#     pane_w: int = 480
#     pane_h: int = 360
#     stale_s: float = 3.0  # frame older than this is shown as STALE
#     status_lines: int = 2


# def fit_pane(image: Optional[np.ndarray], width: int, height: int) -> np.ndarray:
#     """Letterbox ``image`` into a width x height BGR canvas (aspect kept)."""
#     canvas = np.full((height, width, 3), BG, dtype=np.uint8)
#     if image is None or image.size == 0:
#         return canvas
#     if image.ndim == 2:
#         image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
#     h, w = image.shape[:2]
#     image = np.ascontiguousarray(image[:, :, :3])
#     if h < 2 or w < 2:
#         image = cv2.copyMakeBorder(image, 0, max(0, 2 - h), 0, max(0, 2 - w),
#                                    cv2.BORDER_REPLICATE)
#         h, w = image.shape[:2]
#     scale = min(width / float(w), height / float(h))
#     new_w = max(1, int(round(w * scale)))
#     new_h = max(1, int(round(h * scale)))
#     interp = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
#     resized = cv2.resize(image, (new_w, new_h), interpolation=interp)
#     x0 = (width - new_w) // 2
#     y0 = (height - new_h) // 2
#     canvas[y0:y0 + new_h, x0:x0 + new_w] = resized
#     return canvas


# def draw_centered_text(canvas: np.ndarray, text: str, colour=GREY, scale: float = 0.7) -> None:
#     (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 2)
#     x = max(4, (canvas.shape[1] - tw) // 2)
#     y = (canvas.shape[0] + th) // 2
#     cv2.putText(canvas, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, colour, 2)


# def classify_frame(has_frame: bool, age_s: Optional[float], stale_s: float) -> str:
#     """'no_signal' | 'stale' | 'live'."""
#     if not has_frame or age_s is None:
#         return 'no_signal'
#     return 'stale' if age_s > stale_s else 'live'


# def make_pane(image: Optional[np.ndarray], age_s: Optional[float], title: str,
#               cfg: DashboardConfig) -> np.ndarray:
#     """One robot pane: title bar plus letterboxed frame (or NO SIGNAL / STALE)."""
#     state = classify_frame(image is not None, age_s, cfg.stale_s)
#     body = fit_pane(image if state != 'no_signal' else None, cfg.pane_w, cfg.pane_h)
#     if state == 'no_signal':
#         draw_centered_text(body, 'NO SIGNAL', GREY)
#         tag, tag_col = 'no signal', GREY
#     elif state == 'stale':
#         # Dim a frozen frame so it is not mistaken for live video.
#         body = (body * 0.45).astype(np.uint8)
#         draw_centered_text(body, 'STALE  %.0fs' % age_s, AMBER)
#         tag, tag_col = 'stale %.0fs' % age_s, AMBER
#     else:
#         tag, tag_col = 'live', GREEN

#     bar = np.full((TITLE_H, cfg.pane_w, 3), TITLE_BG, dtype=np.uint8)
#     cv2.putText(bar, title, (10, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 1)
#     (tw, _), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
#     cv2.putText(bar, tag, (cfg.pane_w - tw - 10, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.45, tag_col, 1)
#     return np.vstack([bar, body])


# def make_status_bar(lines: Sequence[Tuple[str, Tuple[int, int, int]]], width: int,
#                     min_lines: int = 2) -> np.ndarray:
#     """Status strip. ``lines`` is a sequence of (text, BGR colour)."""
#     count = max(len(lines), min_lines)
#     height = count * STATUS_LINE_H + STATUS_PAD
#     bar = np.full((height, width, 3), (16, 16, 16), dtype=np.uint8)
#     for i, (text, colour) in enumerate(lines):
#         y = STATUS_PAD // 2 + (i + 1) * STATUS_LINE_H - 8
#         cv2.putText(bar, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1)
#     return bar


# def compose_dashboard(
#     parrot: Optional[np.ndarray],
#     husky: Optional[np.ndarray],
#     parrot_age_s: Optional[float],
#     husky_age_s: Optional[float],
#     status_lines: Sequence[Tuple[str, Tuple[int, int, int]]],
#     cfg: DashboardConfig = DashboardConfig(),
# ) -> np.ndarray:
#     """Build the single combined window image (BGR uint8)."""
#     left = make_pane(parrot, parrot_age_s, 'PARROT  aerial detection', cfg)
#     right = make_pane(husky, husky_age_s, 'HUSKY  visual confirm', cfg)
#     divider = np.full((left.shape[0], 2, 3), (80, 80, 80), dtype=np.uint8)
#     top = np.hstack([left, divider, right])
#     bar = make_status_bar(status_lines, top.shape[1], cfg.status_lines)
#     return np.vstack([top, bar])


# def status_colour(status: str) -> Tuple[int, int, int]:
#     s = (status or '').strip().lower()
#     if s in ('confirmed',):
#         return GREEN
#     if s in ('candidate', 'inconclusive'):
#         return AMBER
#     if s in ('not_confirmed', 'none'):
#         return RED
#     return WHITE


# def parrot_status_line(status: str, pose: Optional[Tuple[float, float, float]],
#                        frame_id: str = '') -> Tuple[str, Tuple[int, int, int]]:
#     text = 'PARROT detection: %s' % (status or 'waiting').upper()
#     if pose is not None:
#         text += '   pose (%.1f, %.1f, %.1f)' % pose
#         if frame_id:
#             text += ' [%s]' % frame_id
#     return text, status_colour(status)


# def husky_status_line(result: Optional[dict]) -> Tuple[str, Tuple[int, int, int]]:
#     if not result:
#         return 'HUSKY confirm: standby (no request yet)', GREY
#     status = str(result.get('status', '?'))
#     text = 'HUSKY confirm: %s   %s   %s/%s positive frames' % (
#         status, result.get('request_id', ''),
#         result.get('positive_frames', '?'), result.get('valid_frames', '?'))
#     reason = result.get('reason')
#     if reason:
#         text += '   (%s)' % reason
#     return text, status_colour(status)

#!/usr/bin/env python3
"""OpenCV dashboard helpers for the combined Parrot + Husky detection window.

No ROS imports: this module can be tested with synthetic frames. The ROS node
(detection_viewer.py) only feeds it images, status values and events.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np

Colour = Tuple[int, int, int]
FONT = cv2.FONT_HERSHEY_DUPLEX
AA = cv2.LINE_AA

# Palette (BGR)
BG = (24, 24, 24)
HEADER_BG = (36, 30, 28)
TITLE_BG = (52, 48, 46)
CARD_BG = (34, 34, 34)
WHITE = (240, 240, 240)
GREY = (150, 150, 150)
DIM = (110, 110, 110)
GREEN = (80, 200, 80)
AMBER = (0, 190, 255)
RED = (60, 60, 230)
BLUE = (230, 160, 60)      # Parrot accent
ORANGE = (60, 140, 255)    # Husky accent

# Layout
HEADER_H = 34
TITLE_H = 34
CARD_H = 62
CARD_GAP = 6
PAD = 8
LOG_LINE_H = 20
LOG_MAX = 5


@dataclass(frozen=True)
class DashboardConfig:
    pane_w: int = 480
    pane_h: int = 360
    stale_s: float = 3.0   # frame older than this is shown as STALE
    status_lines: int = 2  # kept for compatibility, unused
    title: str = 'FDE ICE DETECTION'


# ---------------------------------------------------------------- text utils
def text(img: np.ndarray, s: str, org: Tuple[int, int], scale: float = 0.55,
         col: Colour = WHITE, thick: int = 1) -> None:
    cv2.putText(img, s, org, FONT, scale, col, thick, AA)


def text_size(s: str, scale: float, thick: int = 1) -> Tuple[int, int]:
    (w, h), _ = cv2.getTextSize(s, FONT, scale, thick)
    return w, h


def fit_text(s: str, max_w: int, scale: float, thick: int = 1) -> str:
    """Truncate with an ellipsis so text never overflows its box."""
    if text_size(s, scale, thick)[0] <= max_w:
        return s
    while len(s) > 1 and text_size(s + '...', scale, thick)[0] > max_w:
        s = s[:-1]
    return s + '...'


def badge(img: np.ndarray, label: str, right_x: int, cy: int, col: Colour,
          scale: float = 0.45) -> int:
    """Filled pill badge anchored by its right edge. Returns its left x."""
    tw, th = text_size(label, scale)
    w, h = tw + 16, th + 10
    x0, y0 = right_x - w, cy - h // 2
    cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), col, -1)
    text(img, label, (x0 + 8, y0 + h - 6), scale, (20, 20, 20))
    return x0


# --------------------------------------------------------------- camera panes
def fit_pane(image: Optional[np.ndarray], width: int, height: int) -> np.ndarray:
    """Letterbox ``image`` into a width x height BGR canvas (aspect kept)."""
    canvas = np.full((height, width, 3), BG, dtype=np.uint8)
    if image is None or image.size == 0:
        return canvas
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    image = np.ascontiguousarray(image[:, :, :3])
    h, w = image.shape[:2]
    if h < 2 or w < 2:
        image = cv2.copyMakeBorder(image, 0, max(0, 2 - h), 0, max(0, 2 - w),
                                   cv2.BORDER_REPLICATE)
        h, w = image.shape[:2]
    scale = min(width / float(w), height / float(h))
    nw = max(1, int(round(w * scale)))
    nh = max(1, int(round(h * scale)))
    interp = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(image, (nw, nh), interpolation=interp)
    x0, y0 = (width - nw) // 2, (height - nh) // 2
    canvas[y0:y0 + nh, x0:x0 + nw] = resized
    return canvas


def draw_centered_text(canvas: np.ndarray, s: str, colour: Colour = GREY,
                       scale: float = 0.9) -> None:
    tw, th = text_size(s, scale, 2)
    x = max(4, (canvas.shape[1] - tw) // 2)
    y = (canvas.shape[0] + th) // 2
    text(canvas, s, (x, y), scale, colour, 2)


def classify_frame(has_frame: bool, age_s: Optional[float], stale_s: float) -> str:
    """'no_signal' | 'stale' | 'live'."""
    if not has_frame or age_s is None:
        return 'no_signal'
    return 'stale' if age_s > stale_s else 'live'


def make_pane(image: Optional[np.ndarray], age_s: Optional[float], title: str,
              cfg: DashboardConfig, accent: Colour = BLUE) -> np.ndarray:
    """One robot pane: accent title bar with state badge plus letterboxed frame."""
    state = classify_frame(image is not None, age_s, cfg.stale_s)
    body = fit_pane(image if state != 'no_signal' else None, cfg.pane_w, cfg.pane_h)
    if state == 'no_signal':
        draw_centered_text(body, 'NO SIGNAL', GREY)
        tag, col = 'NO SIGNAL', GREY
    elif state == 'stale':
        body = (body * 0.45).astype(np.uint8)  # dim so it is not read as live
        draw_centered_text(body, 'STALE  %.0fs' % age_s, AMBER)
        tag, col = 'STALE %.0fs' % age_s, AMBER
    else:
        tag, col = 'LIVE', GREEN

    bar = np.full((TITLE_H, cfg.pane_w, 3), TITLE_BG, dtype=np.uint8)
    cv2.rectangle(bar, (0, 0), (5, TITLE_H), accent, -1)
    text(bar, title, (16, 23), 0.6, WHITE)
    badge(bar, tag, cfg.pane_w - 8, TITLE_H // 2, col)
    pane = np.vstack([bar, body])
    cv2.rectangle(pane, (0, 0), (pane.shape[1] - 1, pane.shape[0] - 1),
                  (70, 70, 70), 1)
    return pane


# ----------------------------------------------------------------- status UI
def status_colour(status: str) -> Colour:
    s = (status or '').strip().lower()
    if s == 'confirmed':
        return GREEN
    if s == 'active':
        return ORANGE
    if s == 'ready':
        return BLUE
    if s in ('candidate', 'inconclusive', 'tracking', 'seen'):
        return AMBER
    if s in ('not_confirmed', 'none', 'missed'):
        return RED
    return GREY


def make_card(width: int, title: str, badge_text: str, col: Colour,
              detail: str = '', accent: Colour = GREY) -> np.ndarray:
    card = np.full((CARD_H, width, 3), CARD_BG, dtype=np.uint8)
    cv2.rectangle(card, (0, 0), (6, CARD_H), col, -1)
    text(card, title, (18, 24), 0.6, accent)
    badge(card, badge_text, width - 12, 20, col, 0.5)
    text(card, fit_text(detail, width - 30, 0.5), (18, 50), 0.5, GREY)
    return card


def parrot_card(status: str, pose: Optional[Tuple[float, float, float]],
                frame_id: str, width: int) -> np.ndarray:
    detail = 'Waiting for detection'
    if pose is not None:
        detail = 'Pose  x %.1f   y %.1f   z %.1f' % pose
        if frame_id:
            detail += '   [%s]' % frame_id
    return make_card(width, 'PARROT  detection', (status or 'WAITING').upper(),
                     status_colour(status), detail, BLUE)


def husky_card(result: Optional[dict], width: int) -> np.ndarray:
    if not result:
        return make_card(width, 'HUSKY  confirmation', 'STANDBY', GREY,
                         'No request yet', ORANGE)
    status = str(result.get('status', '?'))
    detail = '%s   %s/%s positive frames' % (
        result.get('request_id', ''),
        result.get('positive_frames', '?'), result.get('valid_frames', '?'))
    if result.get('reason'):
        detail += '   (%s)' % result['reason']
    return make_card(width, 'HUSKY  confirmation', status.upper(),
                     status_colour(status), detail, ORANGE)


def make_header(width: int, cfg: DashboardConfig) -> np.ndarray:
    bar = np.full((HEADER_H, width, 3), HEADER_BG, dtype=np.uint8)
    text(bar, cfg.title, (12, 24), 0.7, WHITE)
    s = 'Parrot aerial  >  Husky ground confirm'
    tw, _ = text_size(s, 0.5)
    text(bar, s, (width - tw - 12, 23), 0.5, GREY)
    return bar


def make_event_log(width: int,
                   events: Sequence[Tuple[str, str, Colour]]) -> np.ndarray:
    """events: sequence of (timestamp_str, text, BGR colour), oldest first."""
    h = LOG_MAX * LOG_LINE_H + 30
    panel = np.full((h, width, 3), CARD_BG, dtype=np.uint8)
    cv2.rectangle(panel, (0, 0), (6, h), (90, 90, 90), -1)
    text(panel, 'EVENT LOG', (18, 20), 0.55, GREY)
    shown = list(events)[-LOG_MAX:][::-1]  # newest first
    if not shown:
        text(panel, 'No events yet', (18, 46), 0.5, DIM)
    for i, (ts, msg, col) in enumerate(shown):
        y = 44 + i * LOG_LINE_H
        text(panel, ts, (18, y), 0.5, DIM)
        text(panel, fit_text(msg, width - 110, 0.5), (96, y), 0.5,
             col if i == 0 else GREY)
    return panel


def make_targets_panel(width: int,
                       data: Optional[dict] = None) -> np.ndarray:
    """Target queue and patrol/Husky progress panel, matching event log height."""
    h = LOG_MAX * LOG_LINE_H + 30
    panel = np.full((h, width, 3), CARD_BG, dtype=np.uint8)
    cv2.rectangle(panel, (0, 0), (6, h), (90, 90, 90), -1)

    if not data:
        text(panel, 'TARGET QUEUE', (18, 20), 0.55, GREY)
        text(panel, 'No target feed', (18, 46), 0.5, DIM)
        return panel

    p_info = data.get('parrot', {})
    p_idx = p_info.get('patrol_index', 0)
    p_tot = p_info.get('patrol_total', 0)
    h_info = data.get('husky', {})
    h_phase = h_info.get('phase', 'STANDBY')
    h_act = h_info.get('active') or ''

    stale = data.get('stale', False)
    title_col = AMBER if stale else GREY
    hdr_left = 'TARGET QUEUE (STALE)' if stale else 'TARGET QUEUE'
    text(panel, hdr_left, (18, 20), 0.55, title_col)

    hdr_right = f"Patrol {p_idx}/{p_tot} | Husky: {h_phase}" + (f" {h_act}" if h_act else "")
    tw, _ = text_size(hdr_right, 0.45)
    text(panel, fit_text(hdr_right, width - 150, 0.45),
         (max(140, width - tw - 12), 20), 0.45, DIM)

    targets = data.get('targets', [])
    if not targets:
        text(panel, 'No targets detected yet', (18, 46), 0.5, DIM)
        return panel

    for i, tr in enumerate(targets[:LOG_MAX]):
        y = 44 + i * LOG_LINE_H
        tid = str(tr.get('id', '?'))
        span = str(tr.get('span') or '')
        state = str(tr.get('state', '')).upper()
        hits = f"{tr.get('n_hits', 0)} hits"
        dist = f"{tr.get('dist_m', 0.0):.1f}m"
        col = status_colour(tr.get('state', ''))

        text(panel, tid, (18, y), 0.5, WHITE)
        text(panel, fit_text(span, 70, 0.45), (60, y), 0.45, GREY)
        text(panel, fit_text(state, 90, 0.45), (145, y), 0.45, col)
        text(panel, hits, (250, y), 0.45, DIM)
        text(panel, dist, (320, y), 0.45, DIM)

    return panel


# ------------------------------------------------------------------ composer
def compose_dashboard(
    parrot: Optional[np.ndarray],
    husky: Optional[np.ndarray],
    parrot_age_s: Optional[float],
    husky_age_s: Optional[float],
    parrot_status: str = '',
    parrot_pose: Optional[Tuple[float, float, float]] = None,
    parrot_frame_id: str = '',
    husky_result: Optional[dict] = None,
    events: Sequence[Tuple[str, str, Colour]] = (),
    targets: Optional[dict] = None,
    cfg: DashboardConfig = DashboardConfig(),
) -> np.ndarray:
    """Build the single combined window image (BGR uint8)."""
    left = make_pane(parrot, parrot_age_s, 'PARROT  aerial detection', cfg, BLUE)
    right = make_pane(husky, husky_age_s, 'HUSKY  visual confirm', cfg, ORANGE)
    gap = np.full((left.shape[0], 6, 3), BG, dtype=np.uint8)
    top = np.hstack([left, gap, right])
    W = top.shape[1]
    half = (W - CARD_GAP) // 2
    cards = np.hstack([
        parrot_card(parrot_status, parrot_pose, parrot_frame_id, half),
        np.full((CARD_H, W - 2 * half, 3), BG, dtype=np.uint8),
        husky_card(husky_result, half),
    ])
    pad = np.full((PAD, W, 3), BG, dtype=np.uint8)
    h_log = LOG_MAX * LOG_LINE_H + 30
    bottom = np.hstack([
        make_event_log(half, events),
        np.full((h_log, W - 2 * half, 3), BG, dtype=np.uint8),
        make_targets_panel(half, targets),
    ])
    return np.vstack([make_header(W, cfg), top, pad, cards, pad,
                      bottom, pad])

