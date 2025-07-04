import numpy as np 
from stl import mesh  # pip install numpy-stl
# Using an existing closed stl file: 
your_mesh = mesh.Mesh.from_file('/home/cindy/test_ws/src/line-following-AGV-gazebo-sim/agv_description/meshes/wheels/caster_left.stl') 
volume, cog, inertia = your_mesh.get_mass_properties() 
print("Volume = {0}".format(volume)) 
print("Position of the center of gravity (COG) = {0}".format(cog)) 
print("Inertia matrix at expressed at the COG = {0}".format(inertia[0,:])) 
print(" {0}".format(inertia[1,:])) 
print(" {0}".format(inertia[2,:]))
print(inertia)
