from ikpy.chain import Chain
from ikpy.utils import plot
import numpy as np
import matplotlib.pyplot as plt
from ikpy.link import OriginLink

# Load full robot chain from the correct base
full_chain = Chain.from_urdf_file(
    "/home/shan/mywork/RBY1/rby1_ws/test_codes/ikp/model.urdf",
    base_elements=["base"]
)

# Find indices for torso and right arm
start_link = "link_torso_5"
end_link = "link_right_arm_6"

link_names = [link.name for link in full_chain.links]

print("Links parsed by IKPy:")
for i, link in enumerate(full_chain.links):
    print(f"{i}: {link.name}")

start_index = link_names.index(start_link)
end_index = link_names.index(end_link)

# Slice right arm links (inclusive)
right_arm_links = full_chain.links[start_index:end_index + 1]

# Build a clean chain for right arm
right_arm_chain = Chain(name="right_arm", links=[OriginLink()] + right_arm_links[1:])

# Set target position relative to torso_5
target_position = [0.2, -0.1, 0.3]  # Example position

# Solve IK
angles = right_arm_chain.inverse_kinematics(target_position)

print("Right arm joint angles (rad):", angles)

# Try plotting
try:
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    right_arm_chain.plot(angles, ax)
    plt.title("Right Arm IK Solution")
    plt.show()
except Exception as e:
    print("3D plot failed:", e)
