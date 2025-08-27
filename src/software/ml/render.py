from software.ml.reward_functions import (
    goal_reward,
    distance_reward,
    possession_reward,
    face_ball_orientation_reward,
    dribble_reward,
    kick_reward,
    ball_toward_goal_reward,
)
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np


def render_simulator(sim_state, ssl_geometry, action=None, is_blue=False):
    """Render simulator state with SSL geometry data and reward panel"""
    if not sim_state or not ssl_geometry:
        return np.zeros((800, 1200, 3), dtype=np.uint8)

    fig, (ax, reward_ax) = plt.subplots(
        1, 2, figsize=(10, 4), dpi=200, gridspec_kw={"width_ratios": [3, 1]}
    )

    # Draw field lines using SSL geometry data
    field = ssl_geometry.field
    field_length = field.field_length / 1000  # convert mm to m
    field_width = field.field_width / 1000
    goal_width = field.goal_width / 1000
    goal_depth = field.goal_depth / 1000

    # Field boundary
    field_rect = patches.Rectangle(
        (-field_length / 2, -field_width / 2),
        field_length,
        field_width,
        linewidth=2,
        edgecolor="white",
        facecolor="green",
        alpha=0.3,
    )
    ax.add_patch(field_rect)

    # Center circle
    center_circle = patches.Circle(
        (0, 0),
        field.center_circle_radius / 1000,  # convert mm to m
        linewidth=2,
        edgecolor="white",
        facecolor="none",
    )
    ax.add_patch(center_circle)

    # Left goal area
    left_goal_area = patches.Rectangle(
        (-field_length / 2, -goal_width / 2),
        goal_depth,
        goal_width,
        linewidth=2,
        edgecolor="white",
        facecolor="none",
    )
    ax.add_patch(left_goal_area)

    # Right goal area
    right_goal_area = patches.Rectangle(
        (field_length / 2 - goal_depth, -goal_width / 2),
        goal_depth,
        goal_width,
        linewidth=2,
        edgecolor="white",
        facecolor="none",
    )
    ax.add_patch(right_goal_area)

    # Goals
    left_goal = patches.Rectangle(
        (-field_length / 2 - goal_depth, -goal_width / 2),
        goal_depth,
        goal_width,
        linewidth=2,
        edgecolor="white",
        facecolor="none",
    )
    ax.add_patch(left_goal)

    right_goal = patches.Rectangle(
        (field_length / 2, -goal_width / 2),
        goal_depth,
        goal_width,
        linewidth=2,
        edgecolor="white",
        facecolor="none",
    )
    ax.add_patch(right_goal)

    # Draw robots and ball
    robot_radius = 0.09  # 180mm diameter = 90mm radius

    # Yellow robots (friendly)
    for robot in sim_state.yellow_robots:
        circle = patches.Circle(
            (robot.p_x, robot.p_y),
            robot_radius,
            facecolor="yellow",
            edgecolor="black",
        )
        ax.add_patch(circle)
        # Orientation line
        end_x = robot.p_x + robot_radius * np.cos(robot.r_z)
        end_y = robot.p_y + robot_radius * np.sin(robot.r_z)
        ax.plot([robot.p_x, end_x], [robot.p_y, end_y], "k-", linewidth=1)

        # Red dot if robot can kick
        if robot.can_kick_ball:
            kick_dot = patches.Circle(
                (robot.p_x, robot.p_y),
                robot_radius * 0.2,
                facecolor="red",
                edgecolor="darkred",
            )
            ax.add_patch(kick_dot)

        # Velocity line from action
        if action is not None:
            vel_scale = 2 * robot_radius
            vel_x = action[0] * vel_scale  # VELOCITY_X
            vel_y = action[1] * vel_scale  # VELOCITY_Y
            ax.plot(
                [robot.p_x, robot.p_x + vel_x],
                [robot.p_y, robot.p_y + vel_y],
                "r-",
                linewidth=1.5,
            )

    # Blue robots (enemy)
    for robot in sim_state.blue_robots:
        circle = patches.Circle(
            (robot.p_x, robot.p_y),
            robot_radius,
            facecolor="blue",
            edgecolor="black",
        )
        ax.add_patch(circle)
        # Orientation line
        end_x = robot.p_x + robot_radius * np.cos(robot.r_z)
        end_y = robot.p_y + robot_radius * np.sin(robot.r_z)
        ax.plot([robot.p_x, end_x], [robot.p_y, end_y], "k-", linewidth=1)

    # Ball
    if sim_state.ball:
        circle = patches.Circle(
            (sim_state.ball.p_x, sim_state.ball.p_y),
            0.0215,
            facecolor="orange",
            edgecolor="black",
        )
        ax.add_patch(circle)

    ax.set_xlim(-6, 6)
    ax.set_ylim(-4, 4)
    ax.set_aspect("equal")
    ax.set_facecolor("darkgreen")
    ax.axis("off")

    # Reward panel
    reward_ax.set_xlim(0, 1)
    reward_ax.set_ylim(0, 1)
    reward_ax.axis("off")
    reward_ax.set_facecolor("black")

    # Calculate rewards
    rewards = {
        "Goal": goal_reward(sim_state, ssl_geometry, is_blue),
        "Distance": distance_reward(sim_state, is_blue),
        "Possession": possession_reward(sim_state, is_blue),
        "Face Ball": face_ball_orientation_reward(sim_state, is_blue),
        "Dribble": dribble_reward(sim_state, action, is_blue)
        if action is not None
        else 0.0,
        "Kick": kick_reward(sim_state, action, is_blue) if action is not None else 0.0,
        "Ball toward goal": ball_toward_goal_reward(sim_state, ssl_geometry, is_blue),
        "dribbler": action[4] if action is not None else 0.0,
        "kick": action[3] if action is not None else 0.0,
    }

    # Display rewards
    y_pos = 0.9
    for name, value in rewards.items():
        reward_ax.text(
            0.05,
            y_pos,
            f"{name}: {value:.3f}",
            color="black",
            fontsize=6,
            transform=reward_ax.transAxes,
        )
        y_pos -= 0.07

    fig.canvas.draw()
    buf = np.frombuffer(fig.canvas.tostring_argb(), dtype=np.uint8)
    buf = buf.reshape(fig.canvas.get_width_height()[::-1] + (4,))
    buf = buf[:, :, 1:]
    plt.close(fig)
    return buf
