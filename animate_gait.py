"""Top-down view of the offline model. Run: python animate_gait.py

Pause and drag the time slider to inspect a support triangle. Coordinates are
body-relative, so stance feet move backward as the assumed body moves forward.
The model's center_mass is an assumed point, not a measured center of mass.
"""

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon
from matplotlib.widgets import Button, Slider

import leg_kinematics as model


STANCE = "#167d9a"
SWING = "#e89024"
OUTSIDE = "#c74343"


def sample_frame(t):
    """Evaluate the existing model at one time; no new gait equations."""
    feet = {}
    stance = []
    for leg, offset in model.phase_offsets.items():
        x, z, in_stance = model.foot_target(
            t + offset * model.cycle_duration,
            model.cycle_duration,
            model.stance_fraction,
        )
        position = model.foot_in_body_frame(leg, x, z)
        feet[leg] = (position, in_stance)
        if in_stance:
            stance.append(position[:2])
    return feet, stance


def create_view():
    """Create plot artists once; draw(t) moves them to the sampled positions."""
    fig, ax = plt.subplots(figsize=(9, 8))
    fig.subplots_adjust(bottom=0.26, top=0.86)
    fig.suptitle("Where is the body center relative to the support triangle?", fontsize=14)
    ax.set_title("Top view | body frame | HAA = 0", fontsize=11, color="#555555")
    # Plot y horizontally on a reversed axis, and x vertically: forward is up.
    ax.set_aspect("equal")
    ax.set_xlim(0.32, -0.32)
    ax.set_ylim(-0.34, 0.34)
    ax.set_xlabel("Body y (m)  /  positive toward robot-left")
    ax.set_ylabel("Body x (m)  /  positive forward")
    ax.grid(alpha=0.18)
    ax.axhline(0, color="#bbbbbb", lw=0.7)
    ax.axvline(0, color="#bbbbbb", lw=0.7)

    hip_order = ("FL", "FR", "RR", "RL")
    hips = [model.haa_positions[leg] for leg in hip_order]
    ax.add_patch(Polygon([(p[1], p[0]) for p in hips], closed=True,
                         facecolor="#eeeeee", edgecolor="#999999", zorder=1))
    ax.text(0, 0.12, "HAA rectangle", ha="center", color="#666666", fontsize=9)

    triangle = Polygon([(0, 0)] * 3, closed=True, facecolor=STANCE,
                       edgecolor=STANCE, alpha=0.22, lw=2, zorder=2)
    ax.add_patch(triangle)
    markers, labels = {}, {}
    for leg, hip in model.hfe_positions.items():
        haa = model.haa_positions[leg]
        ax.plot([haa[1], hip[1]], [haa[0], hip[0]], color="#999999", lw=2)
        ax.plot(hip[1], hip[0], "+", color="#555555", ms=8)
        markers[leg], = ax.plot([], [], "o", ms=10, zorder=4)
        labels[leg] = ax.annotate(leg, (0, 0), xytext=(10, 10),
                                  textcoords="offset points", fontsize=10, zorder=5)

    com, = ax.plot([model.center_mass[1]], [model.center_mass[0]],
                   marker="*", ms=18, color=OUTSIDE, zorder=6)
    status = fig.text(0.5, 0.16, "", ha="center", fontsize=11)
    fig.text(0.5, 0.12, "Planned contacts; center of mass assumed at model.center_mass.",
             ha="center", fontsize=9, color="#555555")
    ax.legend(handles=[
        Line2D([], [], marker="o", ls="", color=STANCE, label="Scheduled stance"),
        Line2D([], [], marker="o", ls="", color=SWING, label="Swing"),
        Line2D([], [], marker="*", ls="", color=OUTSIDE, ms=12, label="Assumed COM"),
    ], loc="lower center", fontsize=9, framealpha=0.95)

    def draw(t):
        feet, stance = sample_frame(t)
        for leg, (position, in_stance) in feet.items():
            x, y, _ = position
            markers[leg].set_data([y], [x])
            markers[leg].set_color(STANCE if in_stance else SWING)
            labels[leg].xy = (y, x)

        triangle.set_visible(len(stance) == 3)
        if len(stance) == 3:
            triangle.set_xy([(y, x) for x, y in stance])
            a, b, c = stance
            margin_m = model.support_margin(a, b, c, model.center_mass)
            inside = margin_m > 0
            # Display tolerance in metres only; the model keeps its raw result.
            on_boundary = abs(margin_m) <= 1e-12
            label = "ON EDGE" if on_boundary else "INSIDE" if inside else "OUTSIDE"
            com.set_color(SWING if on_boundary else STANCE if inside else OUTSIDE)
            display_margin = 0.0 if on_boundary else margin_m * 1000
            margin_label = f"Support margin: {display_margin:+.2f} mm"
        else:
            label = f"triangle test needs 3 stance feet (got {len(stance)})"
            margin_label = "Support margin: unavailable"
            com.set_color("#777777")
        swing = ", ".join(leg for leg, (_, contact) in feet.items() if not contact)
        status.set_text(f"t = {t:.2f} s   |   Swing: {swing or 'none'}   |   COM: {label}\n"
                        f"{margin_label}")
        return triangle, com, status, *markers.values(), *labels.values()

    draw(0)
    return fig, draw


def main():
    fig, draw = create_view()
    slider = Slider(fig.add_axes((0.20, 0.065, 0.58, 0.025)),
                    "Time (s)", 0, model.cycle_duration,
                    valinit=0, valstep=model.dt, valfmt="%.2f")
    button = Button(fig.add_axes((0.81, 0.052, 0.12, 0.05)), "Pause")
    paused = False

    def seek(t):
        draw(t)
        fig.canvas.draw_idle()

    def toggle(_event):
        nonlocal paused
        paused = not paused
        button.label.set_text("Play" if paused else "Pause")
        fig.canvas.draw_idle()

    def tick(_frame):
        if not paused:
            t = slider.val + model.dt
            if t >= model.cycle_duration - 1e-12:
                t = 0.0
            slider.set_val(t)

    slider.on_changed(seek)
    button.on_clicked(toggle)
    # Keep the animation alive for as long as the window is open.
    animation = FuncAnimation(fig, tick, init_func=lambda: draw(0),
                              interval=model.dt * 1000, cache_frame_data=False)
    plt.show()
    return animation


if __name__ == "__main__":
    main()
