import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def get_enemy_goal_area(ssl_geometry_data):
    """Compute enemy goal area rectangle from SSL geometry data"""
    if not ssl_geometry_data or not ssl_geometry_data.field:
        return 6.0, 6.18, -0.9, 0.9  # fallback values

    field = ssl_geometry_data.field
    field_length = field.field_length / 1000  # convert mm to m
    goal_width = field.goal_width / 1000
    goal_depth = field.goal_depth / 1000

    # Enemy goal is at positive X end
    x_min = field_length / 2
    x_max = field_length / 2 + goal_depth
    y_min = -goal_width / 2
    y_max = goal_width / 2

    return x_min, x_max, y_min, y_max


def transform_to_robot_frame(x, y, theta, reference_robot):
    """Transform (x, y, theta) into the frame of reference_robot"""
    # Translate to robot origin
    dx = x - reference_robot.p_x
    dy = y - reference_robot.p_y

    # Rotate by negative robot orientation
    cos_theta = np.cos(-reference_robot.r_z)
    sin_theta = np.sin(-reference_robot.r_z)

    rel_x = dx * cos_theta - dy * sin_theta
    rel_y = dx * sin_theta + dy * cos_theta
    rel_theta = theta - reference_robot.r_z

    return rel_x, rel_y, rel_theta


def render_simulator(sim_state, ssl_geometry):
    """Render simulator state with SSL geometry data"""
    if not sim_state or not ssl_geometry:
        return np.zeros((800, 1200, 3), dtype=np.uint8)

    fig, ax = plt.subplots(figsize=(6, 4), dpi=200)

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

    fig.canvas.draw()
    buf = np.frombuffer(fig.canvas.tostring_argb(), dtype=np.uint8)
    buf = buf.reshape(fig.canvas.get_width_height()[::-1] + (4,))
    buf = buf[:, :, 1:]
    plt.close(fig)
    return buf
