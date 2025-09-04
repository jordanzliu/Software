from software.ml.utils import create_observation, ObservationIndex
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

    # Robot-centric observation panel
    reward_ax.set_xlim(-3, 3)
    reward_ax.set_ylim(-3, 3)
    reward_ax.set_aspect("equal")
    reward_ax.set_facecolor("darkgreen")
    reward_ax.set_title("Robot View", color="white", fontsize=8)

    # Render robot-centric world
    if sim_state and sim_state.yellow_robots:
        obs = create_observation(sim_state, ssl_geometry, 0, is_blue)
        
        # Friendly robot at origin (0,0)
        robot_circle = patches.Circle((0, 0), 0.09, facecolor="yellow", edgecolor="black")
        reward_ax.add_patch(robot_circle)
        
        # Enemy robot
        enemy_x = obs[ObservationIndex.ENEMY_ROBOT_REL_X].item()
        enemy_y = obs[ObservationIndex.ENEMY_ROBOT_REL_Y].item()
        enemy_circle = patches.Circle((enemy_x, enemy_y), 0.09, facecolor="blue", edgecolor="black")
        reward_ax.add_patch(enemy_circle)
        
        # Ball
        ball_x = obs[ObservationIndex.BALL_REL_X].item()
        ball_y = obs[ObservationIndex.BALL_REL_Y].item()
        ball_circle = patches.Circle((ball_x, ball_y), 0.0215, facecolor="orange", edgecolor="black")
        reward_ax.add_patch(ball_circle)
        
        # Goal posts
        goal_top_x = obs[ObservationIndex.GOAL_TOP_REL_X].item()
        goal_top_y = obs[ObservationIndex.GOAL_TOP_REL_Y].item()
        goal_bottom_x = obs[ObservationIndex.GOAL_BOTTOM_REL_X].item()
        goal_bottom_y = obs[ObservationIndex.GOAL_BOTTOM_REL_Y].item()
        reward_ax.plot([goal_top_x, goal_bottom_x], [goal_top_y, goal_bottom_y], "w-", linewidth=3)
        
        # Ball velocity vector
        ball_vx = obs[ObservationIndex.BALL_REL_VX].item()
        ball_vy = obs[ObservationIndex.BALL_REL_VY].item()
        reward_ax.arrow(ball_x, ball_y, ball_vx*0.1, ball_vy*0.1, head_width=0.05, head_length=0.05, fc="red", ec="red")

    fig.canvas.draw()
    buf = np.frombuffer(fig.canvas.tostring_argb(), dtype=np.uint8)
    buf = buf.reshape(fig.canvas.get_width_height()[::-1] + (4,))
    buf = buf[:, :, 1:]
    plt.close(fig)
    return buf
