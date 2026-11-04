#!/usr/bin/env python3
"""Offline tests for the combined detection dashboard (no ROS needed).

Run:  python scripts/test_detection_viewer.py      (also pytest-compatible)
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'scripts'))
from detection_viewer_core import (  # noqa: E402
    AMBER, BLUE, GREEN, GREY, ORANGE, RED, TITLE_H, DashboardConfig, classify_frame,
    compose_dashboard, fit_pane, fit_text, husky_card, make_event_log,
    make_pane, make_targets_panel, parrot_card, status_colour,
)

CFG = DashboardConfig(pane_w=480, pane_h=360, stale_s=3.0)


def frame(w, h, bgr=(10, 200, 10)):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = bgr
    return img


def test_fit_pane_keeps_size_for_any_aspect():
    for w, h in ((320, 240), (480, 360), (640, 480), (200, 600), (1, 1)):
        out = fit_pane(frame(w, h), 480, 360)
        assert out.shape == (360, 480, 3) and out.dtype == np.uint8


def test_fit_pane_none_and_gray():
    assert fit_pane(None, 100, 80).shape == (80, 100, 3)
    gray = np.zeros((40, 60), dtype=np.uint8)
    assert fit_pane(gray, 100, 80).shape == (80, 100, 3)


def test_letterbox_centres_wide_image():
    out = fit_pane(frame(400, 100, (0, 0, 255)), 400, 400)
    assert out[200, 200].tolist() == [0, 0, 255]  # image in the middle
    assert out[5, 200].tolist() != [0, 0, 255]  # bar on top is background


def test_classify_frame():
    assert classify_frame(False, None, 3.0) == 'no_signal'
    assert classify_frame(True, 1.0, 3.0) == 'live'
    assert classify_frame(True, 5.0, 3.0) == 'stale'


def test_status_colour_mapping():
    assert status_colour('high') != GREEN
    assert status_colour('high') != status_colour('confirmed')
    assert status_colour('confirmed') == GREEN
    assert status_colour('CONFIRMED') == GREEN
    assert status_colour('active') == ORANGE
    assert status_colour('ready') == BLUE
    assert status_colour('candidate') == AMBER
    assert status_colour('inconclusive') == AMBER
    assert status_colour('tracking') == AMBER
    assert status_colour('not_confirmed') == RED
    assert status_colour('missed') == RED
    assert status_colour('') == GREY and status_colour('bogus') == GREY


def test_pane_height_and_state_change():
    live = make_pane(frame(480, 360, (255, 255, 255)), 0.1, 'T', CFG)
    stale = make_pane(frame(480, 360, (255, 255, 255)), 9.0, 'T', CFG)
    none = make_pane(None, None, 'T', CFG)
    for p in (live, stale, none):
        assert p.shape == (TITLE_H + 360, 480, 3)
    # sample a body pixel away from the centred text
    y, x = TITLE_H + 20, 20
    assert stale[y, x].mean() < live[y, x].mean()


def test_make_targets_panel_empty_and_populated():
    empty = make_targets_panel(480, None)
    populated = make_targets_panel(480, {
        'parrot': {'patrol_index': 1, 'patrol_total': 3},
        'husky': {'phase': 'CONFIRM', 'active': 'T01'},
        'targets': [
            {'id': 'T01', 'span': 'span_01', 'state': 'active', 'n_hits': 5, 'dist_m': 12.3},
            {'id': 'T02', 'span': 'span_02', 'state': 'ready', 'n_hits': 3, 'dist_m': 25.1},
        ],
    })
    stale = make_targets_panel(480, {'stale': True, 'targets': []})
    assert empty.shape == (130, 480, 3)
    assert populated.shape == (130, 480, 3)
    assert stale.shape == (130, 480, 3)
    assert not np.array_equal(empty, populated)


def test_dashboard_shape_is_constant_and_live_differs():
    none = compose_dashboard(None, None, None, None, cfg=CFG)
    live = compose_dashboard(
        frame(320, 240), frame(480, 360), 0.2, 0.1,
        parrot_status='confirmed', parrot_pose=(1.0, 2.0, 3.0),
        parrot_frame_id='husky1_odom',
        husky_result={'status': 'CONFIRMED', 'request_id': 'a',
                      'positive_frames': 4, 'valid_frames': 5},
        events=[('12:00:00', 'Parrot: confirmed', GREEN)],
        targets={'parrot': {'patrol_index': 2, 'patrol_total': 4},
                 'husky': {'phase': 'CONFIRM', 'active': 'T01'},
                 'targets': [{'id': 'T01', 'span': 'span_01', 'state': 'active',
                              'n_hits': 4, 'dist_m': 10.0}]},
        cfg=CFG)
    assert live.shape == none.shape and live.dtype == np.uint8
    assert live.shape[1] == 480 * 2 + 6
    assert not np.array_equal(live, none)


def test_cards_render_for_every_state():
    w = 480
    cards = [parrot_card('', None, '', w),
             parrot_card('confirmed', (35.8, -30.7, 7.6), 'husky1_odom', w),
             husky_card(None, w),
             husky_card({'status': 'NOT_CONFIRMED', 'request_id': 'r1',
                         'positive_frames': 0, 'valid_frames': 5,
                         'reason': 'no_ice'}, w)]
    assert len({c.shape for c in cards}) == 1
    # a confirmed card and a standby card must not look the same
    assert not np.array_equal(cards[1], cards[0])
    assert not np.array_equal(cards[3], cards[2])


def test_husky_card_tolerates_missing_fields():
    assert husky_card({'status': 'CONFIRMED'}, 300).shape[1] == 300
    assert husky_card({}, 300).shape[1] == 300


def test_husky_card_priority_differs_from_plain_confirm():
    plain = husky_card({
        'status': 'CONFIRMED', 'request_id': 'a',
        'positive_frames': 4, 'valid_frames': 5,
    }, 480)
    high = husky_card({
        'status': 'CONFIRMED', 'priority': 'HIGH',
        'priority_reason': (
            'mid-span ice moves the cable centre of gravity off the towers'),
    }, 480)
    assert plain.shape == high.shape
    assert not np.array_equal(plain, high)


def test_target_row_shows_priority_without_changing_shape():
    base = {
        'parrot': {'patrol_index': 1, 'patrol_total': 3},
        'husky': {'phase': 'CONFIRM', 'active': 'T01',
                  'corridor': 'rock gate at x=32, passing south'},
        'targets': [
            {'id': 'T01', 'span': 'span_02', 'state': 'confirmed',
             'n_hits': 5, 'dist_m': 12.3},
        ],
    }
    plain = make_targets_panel(480, base)
    ranked = make_targets_panel(480, {
        **base,
        'targets': [{
            **base['targets'][0],
            'priority': 'HIGH',
            'priority_reason': 'mid-span ice moves the cable centre of gravity',
        }],
    })
    assert plain.shape == ranked.shape == (130, 480, 3)
    assert not np.array_equal(plain, ranked)


def test_fit_text_never_overflows():
    from detection_viewer_core import text_size
    long = 'x' * 200
    out = fit_text(long, 150, 0.5)
    assert out.endswith('...') and text_size(out, 0.5)[0] <= 150 + 20
    assert fit_text('ok', 150, 0.5) == 'ok'


def test_event_log_empty_and_overfull():
    empty = make_event_log(400, [])
    many = make_event_log(400, [('t%d' % i, 'event %d' % i, GREEN)
                                for i in range(50)])
    assert empty.shape == many.shape  # fixed height however many events
    assert not np.array_equal(empty, many)


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for t in tests:
        t()
        print('ok  ', t.__name__)
    print('%d tests passed' % len(tests))
