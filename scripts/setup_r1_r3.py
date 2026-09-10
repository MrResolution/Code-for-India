import bpy

def setup_leg_joint(mesh_name, link_name, parent_link_name, joint_name, axis, is_vertical=True):
    mesh = bpy.data.objects.get(mesh_name)
    parent = bpy.data.objects.get(parent_link_name)
    if not mesh:
        print(f"Error: {mesh_name} not found in scene!")
        return

    # 1. Create or retrieve link armature
    link = bpy.data.objects.get(link_name)
    if not link:
        arm_data = bpy.data.armatures.new(link_name)
        link = bpy.data.objects.new(link_name, arm_data)
        bpy.context.collection.objects.link(link)
        link.location = mesh.location
        link.rotation_euler = mesh.rotation_euler
        
        # Add reference bone
        bpy.context.view_layer.objects.active = link
        bpy.ops.object.mode_set(mode='EDIT')
        bone = arm_data.edit_bones.new('Bone')
        bone.head = (0, 0, 0)
        bone.tail = (0, 0, 1.0) if is_vertical else (0, 0.5, 0)
        bpy.ops.object.mode_set(mode='OBJECT')

    # 2. Parent mesh to link armature
    mesh.parent = link
    mesh.matrix_parent_inverse = link.matrix_world.inverted()

    # 3. Parent link armature to parent link
    if parent and link.parent != parent:
        link.parent = parent
        link.matrix_parent_inverse = parent.matrix_world.inverted()

    # 4. Phobos Custom Properties
    link["geometry/type"] = "mesh"
    link["joint/type"] = "revolute"
    link["joint/name"] = joint_name
    link["joint/axis"] = axis
    link["joint/limits/lower"] = -1.5708
    link["joint/limits/upper"] = 1.5708
    link["joint/limits/effort"] = 2.5
    link["joint/limits/velocity"] = 6.0
    print(f"✅ Successfully configured: {link_name} -> {joint_name} (Parent: {parent_link_name})")

# Check chassis root link name (base_link or Internal-Frame-v121_link)
chassis_link = "base_link" if "base_link" in bpy.data.objects else "Internal-Frame-v121_link"

# 1. R1 (Blue Hip) -> Attached to Chassis (vertical yaw axis)
setup_leg_joint("R1-v117", "R1-v117_link", chassis_link, "joint_r1_hip", [0, 0, 1], is_vertical=True)

# 2. R3 (Orange Foot) -> Attached to R1 (horizontal pitch axis)
setup_leg_joint("R3-v117", "R3-v117_link", "R1-v117_link", "joint_r3_foot", [0, 1, 0], is_vertical=False)

print("🎯 R1 and R3 leg configuration complete!")
