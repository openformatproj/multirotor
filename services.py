from jinja2 import Environment, FileSystemLoader
import math
import numpy as np
import re
import xml.dom.minidom

class ItemList(list):

    def __init__(self, size):
        super().__init__([None] * (size + 1))

    def __getitem__(self, index):
        return super().__getitem__(index - 1)
    
    def __setitem__(self, index, value):
        super().__setitem__(index - 1, value)

def get_inertia_matrix(mass, size):

    inertia_matrix = np.zeros((3, 3))
    c = (mass*(size**2))/6.0
    inertia_matrix[0, 0] = c
    inertia_matrix[1, 1] = c
    inertia_matrix[2, 2] = c
    return inertia_matrix

def get_propellers_positions(propellers, main_radius):

    # Generate the propellers equally spaced centers on the circle with radius main_radius
    positions = ItemList(propellers)
    for i in range(1, propellers + 1):
        angle = 2 * math.pi * (i-1) / propellers
        x = main_radius * math.cos(angle)
        y = main_radius * math.sin(angle)
        x = 0.0 if abs(x) < 1e-9 else x
        y = 0.0 if abs(y) < 1e-9 else y
        positions[i] = [x, y, 0.0]
    return positions

def generate_urdf_model(engine, conf):

    if conf.UPDATE_URDF_MODEL:

        # Load the Jinja2 template
        with open(conf.URDF_TEMPLATE) as file:
            file_content = file.read()
        template = Environment(loader=FileSystemLoader(conf.BASE_DIRECTORY)).from_string(file_content)

        # Define the context for the template rendering
        context = {
            'frame_link_name': 'frame',
            'mass': conf.FRAME_MASS,
            'size': conf.FRAME_SIZE,
            'inertia_matrix': get_inertia_matrix(conf.FRAME_MASS, conf.FRAME_SIZE),
            'propellers': conf.PROPELLERS
        }

        propellers_positions = get_propellers_positions(conf.PROPELLERS, conf.MAIN_RADIUS)
        xyz = ItemList(conf.PROPELLERS)
        for i in range(1, conf.PROPELLERS + 1):
            xyz[i] = f"{propellers_positions[i][0]} {propellers_positions[i][1]} {propellers_positions[i][2]}"
        context['xyz'] = xyz

        # Render the template with the context
        file_content = template.render(context)

        # Write the rendered content to the output file
        dom = xml.dom.minidom.parseString(file_content)
        file_content = '\n'.join([line for line in dom.toprettyxml().split('\n') if line.strip()])

    if engine == 'MuJoCo':

        if conf.UPDATE_URDF_MODEL:

            # --- Patching the URDF for MuJoCo ---
            # 1. Inject <mujoco> block for environment settings (lights, options).
            mjcf_header = f"""
            <mujoco>
                <option timestep="{conf.TIME_STEP}" gravity="0 0 {conf.G}"/>
                <visual>
                    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
                    <rgba haze="0.15 0.25 0.35 1"/>
                    <global azimuth="120" elevation="-20"/>
                </visual>
            </mujoco>
        """
            file_content = re.sub(r'(<robot[^>]*>)', r'\1' + mjcf_header, file_content, count=1)

            # 2. Inject a 'world_root' link containing the floor and axes, and connect it to 'frame'.
            #    This effectively adds the environment to the robot model.
            world_env = f"""
            <link name="world_root">
                <visual>
                    <origin xyz="0 0 -1.0"/>
                    <geometry><box size="10 10 0.1"/></geometry>
                    <material name="floor"><color rgba="1 1 1 1"/></material>
                </visual>
                <collision>
                    <origin xyz="0 0 -1.0"/>
                    <geometry><box size="10 10 0.1"/></geometry>
                </collision>
            </link>
            <link name="world_axis_x">
                <inertial>
                    <mass value="0.001"/>
                    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
                </inertial>
                <visual>
                    <geometry><cylinder radius="0.03" length="0.5"/></geometry>
                    <material name="axis_x"><color rgba="1 0 0 1"/></material>
                </visual>
                <collision>
                    <geometry><cylinder radius="0.001" length="0.5"/></geometry>
                </collision>
            </link>
            <joint name="joint_axis_x" type="fixed">
                <parent link="world_root"/>
                <child link="world_axis_x"/>
                <origin xyz="0.50 0 0" rpy="0 1.5708 0"/>
            </joint>
            <link name="world_axis_y">
                <inertial>
                    <mass value="0.001"/>
                    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
                </inertial>
                <visual>
                    <geometry><cylinder radius="0.03" length="0.5"/></geometry>
                    <material name="axis_y"><color rgba="0 1 0 1"/></material>
                </visual>
                <collision>
                    <geometry><cylinder radius="0.001" length="0.5"/></geometry>
                </collision>
            </link>
            <joint name="joint_axis_y" type="fixed">
                <parent link="world_root"/>
                <child link="world_axis_y"/>
                <origin xyz="0 0.50 0" rpy="1.5708 0 0"/>
            </joint>
            <link name="world_axis_z">
                <inertial>
                    <mass value="0.001"/>
                    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
                </inertial>
                <visual>
                    <geometry><cylinder radius="0.03" length="0.5"/></geometry>
                    <material name="axis_z"><color rgba="0 0 1 1"/></material>
                </visual>
                <collision>
                    <geometry><cylinder radius="0.001" length="0.5"/></geometry>
                </collision>
            </link>
            <joint name="joint_axis_z" type="fixed">
                <parent link="world_root"/>
                <child link="world_axis_z"/>
                <origin xyz="0 0 0.50"/>
            </joint>
            <link name="world_origin">
                <inertial>
                    <mass value="0.001"/>
                    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
                </inertial>
                <visual>
                    <geometry><sphere radius="0.03"/></geometry>
                    <material name="origin_white"><color rgba="1 1 1 1"/></material>
                </visual>
                <collision>
                    <geometry><sphere radius="0.001"/></geometry>
                </collision>
            </link>
            <joint name="joint_origin" type="fixed">
                <parent link="world_root"/>
                <child link="world_origin"/>
                <origin xyz="0 0 0"/>
            </joint>
            <joint name="root_joint" type="floating">
                <parent link="world_root"/>
                <child link="frame"/>
                <dynamics damping="{conf.DAMPING_FACTOR}" friction="0"/>
            </joint>
        """
            file_content = file_content.replace('</robot>', world_env + '</robot>')

    urdf_model_path = conf.URDF_MODEL.format(engine=engine.lower())

    if conf.UPDATE_URDF_MODEL:
        with open(urdf_model_path, 'w') as file:
            file.write(file_content)

    return urdf_model_path