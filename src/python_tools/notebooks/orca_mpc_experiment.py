# ---
# jupyter:
#   jupytext:
#     formats: ipynb,py:light
#     text_representation:
#       extension: .py
#       format_name: light
#       format_version: '1.5'
#       jupytext_version: 1.11.2
#   kernelspec:
#     display_name: Python 3 (ipykernel)
#     language: python
#     name: python3
# ---

# +
from proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from proto.ssl_wrapper_and_velocity_info_pb2 import SSLWrapperAndVelocityInfo
from python_tools.proto_log import ProtoLog
import ipywidgets
from IPython.display import display
from software.python_bindings import world, passing, pass_generator
from extlibs.rvo2.src.RVOPybind import Vector2, RVOSimulator
import numpy as np

wrapper_proto_log = ProtoLog(
    "/home/ubuntu/share/test_data_with_velocity/03012022_002255/SensorFusion_SSLWrapperAndVelocityInfo",
    SSLWrapperAndVelocityInfo,
)


# +
from bokeh.plotting import figure
from bokeh.io import output_notebook, show, push_notebook
from python_tools.plotting.plot_ssl_wrapper import (
    SSLWrapperPlotter,
    MM_PER_M,
    ROBOT_MAX_RADIUS,
)

output_notebook()

fig = figure(plot_width=1000, plot_height=900, match_aspect=True)
fig.background_fill_color = "lightgrey"

ssl_wrapper_plotter = SSLWrapperPlotter(fig)

fig.legend.click_policy = "hide"

heatmap_grid_size = 0.05


def plot_ssl_wrapper_at_idx(idx):
    ssl_wrapper_plotter.plot_ssl_wrapper(wrapper_proto_log[idx].ssl_wrapper)

    the_world = world.World(
        wrapper_proto_log[idx].ssl_wrapper.SerializeToString(), dict()
    )

    push_notebook()


show(fig, notebook_handle=True)

slider = ipywidgets.IntSlider(min=0, max=len(wrapper_proto_log) - 1)
ipywidgets.interact(plot_ssl_wrapper_at_idx, idx=slider)
# -

TICK_REACHABILITY_DISTANCE = 1.0
TICK_DURATION = 0.1
MAX_SPEED = 16


# +
def toRvoVector(protoVector):
    return Vector2(protoVector.x_component_meters, protoVector.y_component_meters)


def addRobotToRVOSimulator(sim, ssl_robot, velocity):
    return sim.addAgent(
        Vector2(ssl_robot.x, ssl_robot.y),
        neighborDist=TICK_REACHABILITY_DISTANCE,
        maxNeighbors=11,
        timeHorizon=TICK_DURATION * 2,
        timeHorizonObst=TICK_DURATION * 2,
        radius=ROBOT_MAX_RADIUS,
        maxSpeed=MAX_SPEED,
        velocity=toRvoVector(velocity),
    )


def addAllRobotsToRVOSimulator(sim, ssl_wrapper_and_velocity_info):
    yellow_robot_velocities = (
        ssl_wrapper_and_velocity_info.velocity_info.yellow_robot_velocities
    )
    yellow_id_to_rvoid_map = dict()
    for robot in ssl_wrapper_and_velocity_info.ssl_wrapper.detection.robots_yellow:
        yellow_id_to_rvoid_map[robot.robot_id] = addRobotToRVOSimulator(
            sim, robot, yellow_robot_velocities[robot.robot_id]
        )

    blue_robot_velocities = (
        ssl_wrapper_and_velocity_info.velocity_info.blue_robot_velocities
    )
    blue_id_to_rvoid_map = dict()
    for robot in ssl_wrapper_and_velocity_info.ssl_wrapper.detection.robots_blue:
        blue_id_to_rvoid_map[robot.robot_id] = addRobotToRVOSimulator(
            sim, robot, blue_robot_velocities[robot.robot_id]
        )

    return (yellow_id_to_rvoid_map, blue_id_to_rvoid_map)


ssl_robot = wrapper_proto_log[0].ssl_wrapper.detection.robots_yellow[0]
velocity = wrapper_proto_log[0].velocity_info.yellow_robot_velocities[
    ssl_robot.robot_id
]
rvoSimulator = RVOSimulator(
    timeStep=0.2,
    neighborDist=TICK_REACHABILITY_DISTANCE,
    maxNeighbors=11,
    timeHorizon=TICK_DURATION * 2,
    timeHorizonObst=TICK_DURATION * 2,
    radius=ROBOT_MAX_RADIUS,
    maxSpeed=MAX_SPEED,
)
yellow_id_mapping, blue_id_mapping = addAllRobotsToRVOSimulator(
    rvoSimulator, wrapper_proto_log[500]
)
# -
rvoSimulator.doStep()
rvoSimulator.getAgentORCALines(3)
