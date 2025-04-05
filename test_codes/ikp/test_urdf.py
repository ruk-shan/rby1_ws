from ikpy.chain import Chain
import numpy as np

# Load full chain from URDF
full_chain = Chain.from_urdf_file("model.urdf")

# Print all links to help us identify the indices
print("All link names:")
for i, link in enumerate(full_chain.links):
    print(f"{i}: {link.name}")

# Manually slice the right arm chain
# Replace these indices with the correct ones for your right arm
# (based on printed list)
start_index = full_chain.links.index(next(l for l in full_chain.links if l.name == "right_arm_0"))
end_index = full_chain.links.index(next(l for l in full_chain.links if l.name == "link_right_arm_6")) + 1

# Build the sub-chain
right_arm_links = [full_chain.links[0]] + full_chain.links[start_index:end_index]  # 0 = base link
active_mask = [False] + [True] * (len(right_arm_links) - 1)

# Create the right arm chain
from ikpy.link import OriginLink
right_arm_chain = Chain(name="right_arm", links=right_arm_links, active_links_mask=active_mask)

# Target position for IK
target_position = [0.4, -0.2, 0.6]

# Compute IK
ik_angles = right_arm_chain.inverse_kinematics(target_position)

# Print joint values
print("\nJoint angles (radians):")
for link, angle in zip(right_arm_chain.links[1:], ik_angles[1:]):
    print(f"{link.name}: {angle:.4f} rad")

# Compute FK
fk_pose = right_arm_chain.forward_kinematics(ik_angles)
print("\nEnd effector pose from FK:")
print(fk_pose)
