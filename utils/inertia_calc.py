# <xacro:macro name="cylinder_inertia" params="m r h">
# 	<inertia ixx="${m * (3 * r * r + h * h) / 12}" ixy="0" ixz="0"
# 		iyy="${m * (3 * r * r + h * h) / 12}" iyz="0"
# 		izz="${m * r * r / 2}"
# 	/>
# </xacro:macro>

# <xacro:macro name="box_inertia" params="m x y z">
# 	<inertia ixx="${m * (y * y + z * z) / 12}" ixy="0" ixz="0"
# 		iyy="${m * (x * x + z * z) / 12}" iyz="0"
# 		izz="${m * (y * y + x * x) / 12}"
# 	/>
# </xacro:macro>

# <xacro:macro name="sphere_inertia" params="m r">
# 	<inertia ixx="${2 * m * r * r / 5}" ixy="0" ixz="0"
# 		iyy="${2 * m * r * r / 5}" iyz="0"
# 		izz="${2 * m * r * r / 5}"
# 	/>
# </xacro:macro>

def print_result(ixx, iyy, izz):

    print(f"ixx : {ixx}")
    print(f"iyy : {iyy}")
    print(f"izz : {izz}")

def cylinder_inertia(m, r, h):
    ixx = m * (3 * r * r + h * h) / 12
    iyy = m * (3 * r * r + h * h) / 12
    izz = m * r * r / 2

    print_result(ixx, iyy, izz)

def box_inertia(m, x, y, z):
    ixx = m * (y * y + z * z) / 12
    iyy = m * (x * x + z * z) / 12
    izz = m * (y * y + x * x) / 12

    print_result(ixx, iyy, izz)

def sphere_inertia(m, r):
    ixx = 2 * m * r * r / 5
    iyy = 2 * m * r * r / 5
    izz = 2 * m * r * r / 5 

    print_result(ixx, iyy, izz)

sphere_inertia(0.01, 0.03)