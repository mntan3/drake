import matplotlib.pyplot as plt

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import ConnectDrakeVisualizer, SceneGraph
from pydrake.lcm import DrakeLcm
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.drawing import plot_graphviz, plot_system_graphviz
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource

file_name = FindResourceOrThrow(
    "drake/examples/multibody/cart_pole/cart_pole.sdf")
builder = DiagramBuilder()
scene_graph = builder.AddSystem(SceneGraph())
cart_pole = builder.AddSystem(MultibodyPlant(0.0))
cart_pole.RegisterAsSourceForSceneGraph(scene_graph)
Parser(plant=cart_pole).AddModelFromFile(file_name)

plt.figure()

cart_pole.Finalize()
val = builder.AddSystem(ConstantVectorSource([0]))

builder.Connect(
    scene_graph.get_query_output_port(),
    cart_pole.get_geometry_query_input_port())
builder.Connect(
    cart_pole.get_geometry_poses_output_port(),
    scene_graph.get_source_pose_port(cart_pole.get_source_id()))
builder.Connect(val.get_output_port(0),
                cart_pole.get_actuation_input_port())

ConnectDrakeVisualizer(builder=builder, scene_graph=scene_graph, lcm=DrakeLcm())

diagram = builder.Build()

simulator = Simulator(diagram)
simulator.Initialize()
simulator.AdvanceTo(4)
