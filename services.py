from jinja2 import Environment, FileSystemLoader
import math
import numpy as np
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

def generate_urdf_model(base_directory, urdf_model, urdf_template, mass, size, propellers, main_radius):

    # Load the Jinja2 template
    with open(urdf_template) as file:
        file_content = file.read()
    template = Environment(loader=FileSystemLoader(base_directory)).from_string(file_content)

    # Define the context for the template rendering
    context = {
        'frame_link_name': 'frame',
        'mass': mass,
        'size': size,
        'inertia_matrix': get_inertia_matrix(mass, size),
        'propellers': propellers
    }

    propellers_positions = get_propellers_positions(propellers, main_radius)
    xyz = ItemList(propellers)
    for i in range(1, propellers + 1):
        xyz[i] = f"{propellers_positions[i][0]} {propellers_positions[i][1]} {propellers_positions[i][2]}"
    context['xyz'] = xyz

    # Render the template with the context
    file_content = template.render(context)

    # Write the rendered content to the output file
    dom = xml.dom.minidom.parseString(file_content)
    file_content = '\n'.join([line for line in dom.toprettyxml().split('\n') if line.strip()])
    with open(urdf_model, 'w') as file:
        file.write(file_content)