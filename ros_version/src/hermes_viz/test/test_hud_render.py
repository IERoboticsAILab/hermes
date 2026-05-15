from hermes_viz.hud.hud import HudState, render_hud


def _base_state(**overrides) -> HudState:
    base = dict(
        deadman=True,
        latency_p50=12.0,
        latency_p95=33.0,
        rate_left=48.0,
        rate_right=47.0,
        gesture="FORM_V",
        command_id="gesture.formation.v",
        motors=(0, 0, 0, 0, 0, 0),
        trails_on=True,
        trail_n=200,
    )
    base.update(overrides)
    return HudState(**base)


def test_render_hud_live_shows_LIVE_badge():
    html = render_hud(_base_state(deadman=True))
    assert "LIVE" in html
    assert "STOPPED" not in html


def test_render_hud_deadman_off_shows_STOPPED():
    html = render_hud(_base_state(deadman=False))
    assert "STOPPED" in html


def test_render_hud_renders_latency_values():
    html = render_hud(_base_state(latency_p50=12.345, latency_p95=33.0))
    assert "12" in html  # p50 rendered as int ms
    assert "33" in html  # p95 rendered as int ms


def test_render_hud_latency_unavailable_when_p50_none():
    html = render_hud(_base_state(latency_p50=None, latency_p95=None))
    assert "unavailable" in html.lower()
    # Must not silently display a wrong number.


def test_render_hud_packet_rates_present():
    html = render_hud(_base_state(rate_left=48.5, rate_right=47.2))
    assert "48" in html
    assert "47" in html


def test_render_hud_gesture_and_command_id():
    html = render_hud(_base_state(gesture="FIST", command_id="gesture.fist"))
    assert "FIST" in html
    assert "gesture.fist" in html


def test_render_hud_no_gesture_shows_dash():
    html = render_hud(_base_state(gesture=None, command_id=""))
    assert "—" in html or "-" in html  # em dash or hyphen; either acceptable


def test_render_hud_motor_dots_count_is_six():
    html = render_hud(_base_state(motors=(255, 0, 128, 64, 0, 200)))
    # Count motor-dot divs — template should emit exactly 6.
    assert html.count("motor-dot") == 6 or html.count("border-radius:50%") == 6


def test_render_hud_trails_status_line():
    html_on = render_hud(_base_state(trails_on=True, trail_n=150))
    html_off = render_hud(_base_state(trails_on=False, trail_n=200))
    assert "150" in html_on
    assert "off" in html_off.lower() or "disabled" in html_off.lower()
