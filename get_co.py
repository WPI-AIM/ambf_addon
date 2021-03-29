import mathutils
import math

def rot_matrix_from_vecs(v1, v2):
    out = mathutils.Matrix.Identity(3)
    vcross = v1.cross(v2)
    vdot = v1.dot(v2)
    rot_angle = v1.angle(v2)
    if abs(rot_angle) < 0.001:
        return out
    if abs(rot_angle) > 3.14:
        # Since the vectors are almost opposite, we need to define a rotation order
        nx = mathutils.Vector([1, 0, 0])
        temp_ang = v1.angle(nx)
        if 0.001 < abs(temp_ang) < 3.14:
            axis = v1.cross(nx)
            out = out.Rotation(rot_angle, 3, axis)
        else:
            ny = mathutils.Vector([0, 1, 0])
            axis = v1.cross(ny)
            out = out.Rotation(rot_angle, 3, axis)
    else:
        skew_v = skew_mat(vcross)
        out = mathutils.Matrix.Identity(3) + skew_v + skew_v @ skew_v * ((1 - vdot) / (vec_norm(vcross) ** 2))
    return out

def get_axis_of_ambf_constraint(joint_obj_handle):
   if joint_obj_handle.ambf_constraint_axis == 'X':
       joint_axis = mathutils.Vector([1, 0, 0])
   elif joint_obj_handle.ambf_constraint_axis == 'Y':
       joint_axis = mathutils.Vector([0, 1, 0])
   elif joint_obj_handle.ambf_constraint_axis == 'Z':
       joint_axis = mathutils.Vector([0, 0, 1])
   else:
       print("ERROR! JOINT AXES NOT UNDERSTOOD")
   return joint_axis

def vec_norm(v):
   return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)

def skew_mat(v):
   m = mathutils.Matrix.Identity(3)
   m.Identity(3)
   m[0][0] = 0
   m[0][1] = -v.z
   m[0][2] = v.y
   m[1][0] = v.z
   m[1][1] = 0
   m[1][2] = -v.x
   m[2][0] = -v.y
   m[2][1] = v.x
   m[2][2] = 0
   return m

def compute_body_pivot_and_axis(parent, child, constraint_axis):
   # Since the rotation matrix is carrying the scale, separate out just
   # the rotation component
   # Transform of Parent in World
   t_p_w = parent.matrix_world.copy().to_euler().to_matrix().to_4x4()
   t_p_w.translation = parent.matrix_world.copy().translation
   # Since the rotation matrix is carrying the scale, separate out just
   # the rotation component
   # Transform of Child in World
   t_c_w = child.matrix_world.copy().to_euler().to_matrix().to_4x4()
   t_c_w.translation = child.matrix_world.copy().translation
   # Copy over the transform to invert it
   t_w_p = t_p_w.copy()
   t_w_p.invert()
   # Transform of Child in Parent
   # t_c_p = t_w_p * t_c_w
   t_c_p = t_w_p @ t_c_w
   pivot = t_c_p.translation
   constraint_axis.resize_4d()
   constraint_axis[3] = 0.0
   # The third col of rotation matrix is the z axis of child in parent
   axis = mathutils.Vector((t_c_p @ constraint_axis)[0:3])
   return pivot, axis

def get_co(joint_obj_handle):
   parent_obj_handle = joint_obj_handle.ambf_constraint_parent
   child_obj_handle = joint_obj_handle.ambf_constraint_child
   constraint_axis = get_axis_of_ambf_constraint(joint_obj_handle)
   parent_pivot, parent_axis = compute_body_pivot_and_axis(
       parent_obj_handle, joint_obj_handle, constraint_axis)
   child_pivot, child_axis = compute_body_pivot_and_axis(
       child_obj_handle, joint_obj_handle, constraint_axis)
   r_c_p_ambf = rot_matrix_from_vecs(child_axis, parent_axis)
   r_p_c_ambf = r_c_p_ambf.to_3x3().copy()
   r_p_c_ambf.invert()
   t_p_w = parent_obj_handle.matrix_world.copy()
   r_w_p = t_p_w.to_3x3().copy()
   r_w_p.invert()
   r_c_w = child_obj_handle.matrix_world.to_3x3().copy()
   r_c_p_blender = r_w_p @ r_c_w
   r_angular_offset = r_p_c_ambf @ r_c_p_blender
   offset_axis_angle = r_angular_offset.to_quaternion().to_axis_angle()
   print("CHILD AXIS ", child_axis)
   print("PARENT AXIS ", parent_axis)
   print("OFFSET AXIS ", offset_axis_angle[0])
   print("CHILD OFFSET ANGLE ", offset_axis_angle[1])
   print("DOT(ch_ax, off_ax ", child_axis.dot(offset_axis_angle[0]))
   print("CROSS(ch_ax, off_ax ", child_axis.cross(offset_axis_angle[0]))
   print("**********")