# Author: Adnan Munawar
# Email: amunawar@wpi.edu
# Lab: aimlab.wpi.edu

bl_info = {
    "name": "Asynchronous Multi-Body Framework (AMBF) Config Creator",
    "author": "Adnan Munawar",
    "version": (0, 1),
    "blender": (2, 8, 0),
    "location": "View3D > Add > Mesh > AMBF",
    "description": "Helps Generate AMBF Config File and Saves both High and Low Resolution(Collision) Meshes",
    "warning": "",
    "wiki_url": "https://github.com/WPI-AIM/ambf_addon",
    "category": "AMBF",
    }

import bpy
import math
import yaml
import os
import sys
from pathlib import Path
import mathutils
from enum import Enum
from collections import OrderedDict, Counter
from datetime import datetime
from bpy.props import BoolProperty, FloatProperty, FloatVectorProperty, BoolVectorProperty
from bpy.props import StringProperty, IntProperty, PointerProperty, EnumProperty, CollectionProperty
from bpy.types import Scene, Operator, Panel, Object, PropertyGroup


# Body Template for the some commonly used of afBody's data
class BodyTemplate:
    def __init__(self):
        self._adf_data = OrderedDict()
        self._adf_data['name'] = ""
        self._adf_data['mesh'] = ""
        self._adf_data['collision mesh type'] = ""
        self._adf_data['mass'] = 0.0
        self._adf_data['inertia'] = {'ix': 0.0, 'iy': 0.0, 'iz': 0.0}
        self._adf_data['collision margin'] = 0.001
        self._adf_data['scale'] = 1.0
        self._adf_data['location'] = get_pose_ordered_dict()
        self._adf_data['inertial offset'] = get_pose_ordered_dict()
        self._adf_data['passive'] = False

        # self._adf_data['controller'] = {'linear': {'P': 1000, 'I': 0, 'D': 1},
        #                                  'angular': {'P': 1000, 'I': 0, 'D': 1}}
        self._adf_data['color'] = 'random'


# Body Template for the some commonly used of afBody's data
class GhostObjectTemplate:
    def __init__(self):
        self._adf_data = OrderedDict()
        self._adf_data['name'] = ""
        self._adf_data['mesh'] = ""
        self._adf_data['collision mesh type'] = ""
        self._adf_data['collision margin'] = 0.001
        self._adf_data['scale'] = 1.0
        self._adf_data['location'] = get_pose_ordered_dict()
        self._adf_data['passive'] = False

        self._adf_data['color'] = 'random'


# Joint Template for the some commonly used of afJoint's data
class JointTemplate:
    def __init__(self):
        self._adf_data = OrderedDict()
        self._adf_data['name'] = ''
        self._adf_data['parent'] = ''
        self._adf_data['child'] = ''
        self._adf_data['parent axis'] = get_xyz_ordered_dict()
        self._adf_data['parent pivot'] = get_xyz_ordered_dict()
        self._adf_data['child axis'] = get_xyz_ordered_dict()
        self._adf_data['child pivot'] = get_xyz_ordered_dict()
        self._adf_data['joint limits'] = {'low': -1.2, 'high': 1.2}
        self._adf_data['enable feedback'] = False
        self._adf_data['passive'] = False

        cont_dict = OrderedDict()
        cont_dict['P'] = 1000
        cont_dict['I'] = 0
        cont_dict['D'] = 1
        self._adf_data['controller'] = cont_dict
        self._adf_data['controller output type'] = 'VELOCITY'


# Global Variables
class CommonConfig:
    namespace = ''
    num_collision_groups = 20
    # Some properties don't exist in Blender are supported in AMBF. If an AMBF file is loaded
    # and then resaved, we can capture the extra properties of bodies and joints and take
    # them into consideration before re saving the AMBF File so we don't reset those values
    loaded_body_map = {}
    loaded_joint_map = {}
    collision_shape_material = None
    collision_shape_material_name = 'collision_shape_material'
    collision_shape_material_color = mathutils.Vector((0.8, 0.775, 0.0, 0.4)) # Pick a random color


# https://stackoverflow.com/questions/31605131/dumping-a-dictionary-to-a-yaml-file-while-preserving-order/31609484
def represent_dictionary_order(self, dict_data):
    return self.represent_mapping('tag:yaml.org,2002:map', dict_data.items())


def ambf_round(val):
    return round(val, bpy.context.scene.ambf_precision)


def setup_yaml():
    yaml.add_representer(OrderedDict, represent_dictionary_order)


# Enum Class for Mesh Type
class MeshType(Enum):
    meshSTL = 0
    meshOBJ = 1
    mesh3DS = 2
    meshPLY = 3


def get_extension(val):
    if val == MeshType.meshSTL.value:
        extension = '.STL'
    elif val == MeshType.meshOBJ.value:
        extension = '.OBJ'
    elif val == MeshType.mesh3DS.value:
        extension = '.3DS'
    elif val == MeshType.meshPLY.value:
        extension = '.PLY'
    else:
        extension = None

    return extension


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


def vec_norm(v):
    return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)


def round_vec(v):
    for i in range(0, 3):
        v[i] = ambf_round(v[i])
    return v


# https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d/897677#897677
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


# Get rotation matrix to represent rotation between two vectors
# Brute force implementation
def get_rot_mat_from_vecs(vecA, vecB):
    # Angle between two axis
    angle = vecA.angle(vecB)
    # Axis of rotation between child's joints axis and constraint_axis
    if abs(angle) <= 0.1:
        # Doesn't matter which axis we chose, the rot mat is going to be identity
        # as angle is almost 0
        axis = mathutils.Vector([0, 1, 0])
    elif abs(angle) >= 3.13:
        # This is a more involved case, find out the orthogonal vector to vecA
        nx = mathutils.Vector([1, 0, 0])
        temp_ang = vecA.angle(nx)
        if 0.1 < abs(temp_ang) < 3.13:
            axis = vecA.cross(nx)
        else:
            ny = mathutils.Vector([0, 1, 0])
            axis = vecA.cross(ny)
    else:
        axis = vecA.cross(vecB)

    mat = mathutils.Matrix()
    # Rotation matrix representing the above angular offset
    rot_mat = mat.Rotation(angle, 4, axis)
    return rot_mat, angle


def ensure_collision_shape_material():
    # Create a use a material only for the first instance
    if bpy.data.materials.find(CommonConfig.collision_shape_material_name) == -1:
        CommonConfig.collision_shape_material = bpy.data.materials.new(CommonConfig.collision_shape_material_name)
    else:
        CommonConfig.collision_shape_material = bpy.data.materials[CommonConfig.collision_shape_material_name]

    CommonConfig.collision_shape_material.diffuse_color = CommonConfig.collision_shape_material_color


def update_global_namespace(context):
    CommonConfig.namespace = context.scene.ambf_namespace
    if CommonConfig.namespace[-1] != '/':
        print('WARNING, MULTI-BODY NAMESPACE SHOULD END WITH \'/\'')
        CommonConfig.namespace += '/'
        context.scene.ambf_namespace += CommonConfig.namespace


def set_global_namespace(context, namespace):
    CommonConfig.namespace = namespace
    if CommonConfig.namespace[-1] != '/':
        print('WARNING, MULTI-BODY NAMESPACE SHOULD END WITH \'/\'')
        CommonConfig.namespace += '/'
    context.scene.ambf_namespace = CommonConfig.namespace


def get_body_namespace(fullname):
    last_occurance = fullname.rfind('/')
    _body_namespace = ''
    if last_occurance >= 0:
        # This means that the name contains a namespace
        _body_namespace = fullname[0:last_occurance+1]
    return _body_namespace


def remove_namespace_prefix(full_name):
    last_occurance = full_name.rfind('/')
    if last_occurance > 0:
        # Body name contains a namespace
        _name = full_name[last_occurance+1:]
    else:
        # Body name doesn't have a namespace
        _name = full_name
    return _name


def replace_dot_from_object_names(char_subs ='_'):
    for obj_handle in bpy.data.objects:
        obj_handle.name = obj_handle.name.replace('.', char_subs)


def compare_body_namespace_with_global(fullname):
    last_occurance = fullname.rfind('/')
    _is_namespace_same = False
    _body_namespace = ''
    _name = ''
    if last_occurance >= 0:
        # This means that the name contains a namespace
        _body_namespace = fullname[0:last_occurance+1]
        _name = fullname[last_occurance+1:]

        if CommonConfig.namespace == _body_namespace:
            # The CommonConfig namespace is the same as the body namespace
            _is_namespace_same = True
        else:
            # The CommonConfig namespace is different form body namespace
            _is_namespace_same = False

    else:
        # The body's name does not contain and namespace
        _is_namespace_same = False

    # print("FULLNAME: %s, BODY: %s, NAMESPACE: %s NAMESPACE_MATCHED: %d" %
    # (fullname, _name, _body_namespace, _is_namespace_same))
    return _is_namespace_same


def add_namespace_prefix(name):
    return CommonConfig.namespace + name


def get_grand_parent(body):
    grand_parent = body
    while grand_parent.parent is not None:
        grand_parent = grand_parent.parent
    return grand_parent


def downward_tree_pass(body, _heirarichal_bodies_list, _added_bodies_list):
    if body is None or _added_bodies_list[body] is True:
        return

    else:
        # print('DOWNWARD TREE PASS: ', body.name)
        _heirarichal_bodies_list.append(body)
        _added_bodies_list[body] = True

        for child in body.children:
            downward_tree_pass(child, _heirarichal_bodies_list, _added_bodies_list)


def populate_heirarchial_tree():
    # Create a dict with {body, added_flag} elements
    # The added_flag is to check if the body has already
    # been added
    _added_bodies_list = {}
    _heirarchial_bodies_list = []

    for obj_handle in bpy.data.objects:
        _added_bodies_list[obj_handle] = False

    for obj_handle in bpy.data.objects:
        grand_parent = get_grand_parent(obj_handle)
        # print('CALLING DOWNWARD TREE PASS FOR: ', grand_parent.name)
        downward_tree_pass(grand_parent, _heirarchial_bodies_list, _added_bodies_list)

    for body in _heirarchial_bodies_list:
        print(body.name, "--->",)

    return _heirarchial_bodies_list


# Courtesy: https://stackoverflow.com/questions/5914627/prepend-line-to-beginning-of-a-file
def prepend_comment_to_file(filename, comment):
    temp_filename = filename + '.tmp'
    with open(filename,'r') as f:
        with open(temp_filename, 'w') as f2:
            f2.write(comment)
            f2.write(f.read())
    os.rename(temp_filename, filename)


def select_object(obj_handle, select=True):
    obj_handle.select_set(select)


def select_objects(obj_handles, select=True):
    for obj_handle in obj_handles:
        select_object(obj_handle, select)


def select_all_objects(select):
    # First deselect all objects
    for obj_handle in bpy.data.objects:
        select_object(obj_handle, select)


def hide_object(object, hide):
    if object:
        # object.hide = hide
        object.hide_set(hide)


def is_object_hidden(object):
    if object:
        # hidden = object.hide
        hidden = object.hide_get()
    else:
        raise ValueError
    return hidden


def get_active_object():
    active_obj_handle = bpy.context.active_object
    return active_obj_handle


def set_active_object(active_object):
   # bpy.context.scene.objects.active = active_object
    bpy.context.view_layer.objects.active = active_object


def get_selected_objects():
    return bpy.context.selected_objects


def make_obj1_parent_of_obj2(obj1, obj2):
    select_all_objects(False)
    if obj2.parent is None:
        select_object(obj2)
        select_object(obj1)
        set_active_object(obj1)
        bpy.ops.object.parent_set(keep_transform=True)


def get_xyz_ordered_dict():
    xyz = OrderedDict()
    xyz['x'] = 0
    xyz['y'] = 0
    xyz['z'] = 0
    return xyz


def get_rpy_ordered_dict():
    rpy = OrderedDict()
    rpy['r'] = 0
    rpy['p'] = 0
    rpy['y'] = 0
    return rpy


def get_pose_ordered_dict():
    pose = OrderedDict()
    pose['position'] = get_xyz_ordered_dict()
    pose['orientation'] = get_rpy_ordered_dict()
    return pose


# For shapes such as Cylinder, Cone and Ellipse, this function returns
# the major axis by comparing the dimensions of the bounding box
def get_major_axis(dims):
    d = dims
    axis = {0: 'x', 1: 'y', 2: 'z'}
    sum_diff = [abs(d[0] - d[1]) + abs(d[0] - d[2]),
                abs(d[1] - d[0]) + abs(d[1] - d[2]),
                abs(d[2] - d[0]) + abs(d[2] - d[1])]
    # If the bounds are equal, choose the z axis
    if sum_diff[0] == sum_diff[1] and sum_diff[1] == sum_diff[2]:
        axis_idx = 2
    else:
        axis_idx = sum_diff.index(max(sum_diff))

    return axis[axis_idx], axis_idx


def get_axis_str(axis_idx):
    axis_str = None
    if axis_idx == 0:
        axis_str = 'X'
    elif axis_idx == 1:
        axis_str = 'Y'
    elif axis_idx == 2:
        axis_str = 'Z'

    return axis_str


def get_axis_idx(axis_str):
    axis_idx = None
    if axis_str == 'X':
        axis_idx = 0
    elif axis_str == 'Y':
        axis_idx = 1
    elif axis_str == 'Z':
        axis_idx = 2

    return axis_idx


def get_axis_vec_from_str(axis_str):
    axis_str = axis_str.upper()
    axis_vec = mathutils.Vector((0, 0, 0))
    if axis_str == 'X':
        axis_vec[0] = 1.0
    elif axis_str == 'Y':
        axis_vec[1] = 1.0
    elif axis_str == 'Z':
        axis_vec[2] = 1.0
    return axis_vec


# For shapes such as Cylinder, Cone and Ellipse, this function returns
# the median axis (not-major and non-minor or the middle axis) by comparing
# the dimensions of the bounding box
def get_median_axis(dims):
    axis = {0: 'x', 1: 'y', 2: 'z'}
    maj_ax, maj_ax_idx = get_major_axis(dims)
    min_ax, min_ax_idx = get_minor_axis(dims)
    med_axis_idx = [1, 1, 1]
    med_axis_idx[maj_ax_idx] = 0
    med_axis_idx[min_ax_idx] = 0
    axis_idx = med_axis_idx.index(max(med_axis_idx))

    return axis[axis_idx], axis_idx


# For shapes such as Cylinder, Cone and Ellipse, this function returns
# the minor axis by comparing the dimensions of the bounding box
def get_minor_axis(dims):
    d = dims
    axis = {0: 'x', 1: 'y', 2: 'z'}
    sum_diff = [abs(d[0] - d[1]) + abs(d[0] - d[2]),
                abs(d[1] - d[0]) + abs(d[1] - d[2]),
                abs(d[2] - d[0]) + abs(d[2] - d[1])]
    max_idx = sum_diff.index(max(sum_diff))
    min_idx = sum_diff.index(min(sum_diff))
    sort_idx = [1, 1, 1]
    sort_idx[max_idx] = 0
    sort_idx[min_idx] = 0
    median_idx = sort_idx.index(max(sort_idx))
    return axis[median_idx], median_idx


# Courtesy of:
# https://blender.stackexchange.com/questions/62040/get-center-of-geometry-of-an-object
def compute_local_com(obj_handle):
    vcos = [v.co for v in obj_handle.data.vertices]
    find_center = lambda l: ( max(l) + min(l)) / 2
    x, y, z = [[v[i] for v in vcos] for i in range(3)]
    center = [find_center(axis) for axis in [x, y, z]]
    for i in range(0, 3):
        center[i] = center[i] * obj_handle.scale[i]
    return center


def estimate_joint_controller_gain(obj_handle):
    if obj_handle.ambf_object_type == 'CONSTRAINT':
        parent_obj_handle = obj_handle.ambf_object_parent
        child_obj_handle = obj_handle.ambf_object_child
        if parent_obj_handle and child_obj_handle:
            T_p_w = parent_obj_handle.matrix_world.copy()
            T_c_w = child_obj_handle.matrix_world.copy()
            T_j_w = obj_handle.matrix_world.copy()
            T_p_j = T_j_w.inverted() @ T_p_w
            T_c_j = T_j_w.inverted() @ T_c_w
            N_j = get_axis_vec_from_str(obj_handle.ambf_constraint_axis)
            P_pcom = mathutils.Vector(compute_local_com(parent_obj_handle))
            P_pcom_j = T_p_j @ P_pcom
            P_ccom = mathutils.Vector(compute_local_com(child_obj_handle))
            P_ccom_j = T_c_j @ P_ccom
            mass_p = parent_obj_handle.ambf_rigid_body_mass
            mass_c = child_obj_handle.ambf_rigid_body_mass
            if obj_handle.ambf_constraint_type == 'REVOLUTE':
                if P_pcom_j.length > 0.001:
                    theta_pj = N_j.angle(P_pcom_j)
                    d_pn = P_pcom_j.length * math.sin(theta_pj)
                else:
                    d_pn = 0.0
                if P_ccom_j.length > 0.001:
                    theta_cj = N_j.angle(P_ccom_j)
                    d_cn = P_ccom_j.length * math.sin(theta_cj)
                else:
                    d_cn = 0.0
                # The gains should be scaled according the sum of these
                # distances
                holding_torque = (mass_p * mass_p * d_pn + mass_c * mass_c * d_cn)
                print('Holding Torque:, ', holding_torque)
                if holding_torque == 0.0:
                    holding_torque = 0.001
                # Lets define at what error do we want to apply the holding torque
                error_deg = 1.0
                Kp = holding_torque / ((error_deg / 180.0) * math.pi)
                Ki = Kp / 100.0
                Kd = Kp / 100.0
                print('Kp:, ', Kp)
                print('Kd:, ', Kd)
                obj_handle.ambf_constraint_controller_p_gain = Kp
                obj_handle.ambf_constraint_controller_i_gain = Ki
                obj_handle.ambf_constraint_controller_d_gain = Kd
            elif obj_handle.ambf_constraint_type == 'PRISMATIC':
                holding_effort = (mass_p + mass_c)
                error_m = 0.001
                Kp = holding_effort / error_m
                Ki = Kp / 100.0
                Kd = Kp / 100.0
                print('Kp:, ', Kp)
                print('Kd:, ', Kd)
                obj_handle.ambf_constraint_controller_p_gain = Kp
                obj_handle.ambf_constraint_controller_i_gain = Ki
                obj_handle.ambf_constraint_controller_d_gain = Kd


def inertia_of_mesh(obj_handle, mass=None):
    if mass == None:
        mass = obj_handle.ambf_rigid_body_mass
    num_vertices = len(obj_handle.data.vertices)
    dm = mass / num_vertices
    I = mathutils.Vector((0, 0, 0))
    # Tripple Summation or Integral
    for v in obj_handle.data.vertices:
        I[0] = I[0] + dm * (v.co[1]*v.co[1] + v.co[2] * v.co[2])
        I[1] = I[1] + dm * (v.co[0]*v.co[0] + v.co[2] * v.co[2])
        I[2] = I[2] + dm * (v.co[0]*v.co[0] + v.co[1] * v.co[1])
    return I


def inertia_of_box(mass, lx, ly, lz):
    I = mathutils.Vector((0, 0, 0))
    I[0] = (1.0 / 12.0) * mass * (ly*ly + lz*lz)
    I[1] = (1.0 / 12.0) * mass * (lx*lx + lz*lz)
    I[2] = (1.0 / 12.0) * mass * (lx*lx + ly*ly)
    return I


def inertia_of_sphere(mass, r):
    I = mathutils.Vector((0, 0, 0))
    r2 = r * r
    I[0] = (2.0 / 5.0) * mass * r2
    I[1] = (2.0 / 5.0) * mass * r2
    I[2] = (2.0 / 5.0) * mass * r2
    return I


def inertia_of_cylinder(mass, r, h, axis):
    I = mathutils.Vector((0, 0, 0))
    r2 = r * r
    h2 = h * h
    I[axis] = (1/2) * mass * r2
    I[(axis + 1) % 3] = (1/4) * mass * r2 + (1/12) * mass * h2
    I[(axis + 2) % 3] = (1/4) * mass * r2 + (1/12) * mass * h2
    return I


def inertia_of_cone(mass, r, h, axis):
    I = mathutils.Vector((0, 0, 0))
    r2 = r * r
    h2 = h * h
    I[axis] = (3/10) * mass * r2
    I[(axis + 1) % 3] = (3/20) * mass * r2 + (3/5) * mass * h2
    I[(axis + 2) % 3] = (3/20) * mass * r2 + (3/5) * mass * h2
    return I


def inertia_of_capsule(mass, r, h_total, axis):
    I = mathutils.Vector((0, 0, 0))
    h = h_total - (r * 2) # Get the length of main cylinder
    if h <= 0.001:
        # This means that this obj_handle shape is essentially a sphere, not a capsule
        I = inertia_of_sphere(mass, r)
    else:
        r2 = r * r
        h2 = h * h
        # Factor the mass: (mass of hemisphere) / (mass of cylinder)
        mass_factor = (2 * r) / (3 * h)
        m_hs = mass * mass_factor
        m_cy = mass * (1 - mass_factor)
        I[axis] = (1/2) * m_cy * r2 + (4 / 5) * m_hs * r2
        I[(axis + 1) % 3] = (1 / 12) * m_cy * (h2 + (3 * r2)) + 2 * m_hs * ((2 * r2 / 5) + (h2 / 2) + ((3/8) * h * r))
        I[(axis + 2) % 3] = (1 / 12) * m_cy * (h2 + (3 * r2)) + 2 * m_hs * ((2 * r2 / 5) + (h2 / 2) + ((3/8) * h * r))
    return I


def calculate_principal_inertia(obj_handle):
    # Calculate Ixx, Iyy and Izz
    mass = obj_handle.ambf_rigid_body_mass
    # For now, handle the calculation of the compound shape inertia as the convex hull's inertia
    if obj_handle.ambf_collision_type in ['MESH', 'COMPOUND_SHAPE']:
        I = inertia_of_mesh(obj_handle)

    elif obj_handle.ambf_collision_type == 'SINGULAR_SHAPE':
        prop_group = obj_handle.ambf_collision_shape_prop_collection.items()[0]
        coll_shape_obj_handle = prop_group[1]

        lx = coll_shape_obj_handle.ambf_collision_shape_xyz_dims[0]
        ly = coll_shape_obj_handle.ambf_collision_shape_xyz_dims[1]
        lz = coll_shape_obj_handle.ambf_collision_shape_xyz_dims[2]
        radius = coll_shape_obj_handle.ambf_collision_shape_radius
        height = coll_shape_obj_handle.ambf_collision_shape_height
        axis = get_axis_idx(coll_shape_obj_handle.ambf_collision_shape_axis.upper())

        if coll_shape_obj_handle.ambf_collision_shape == 'BOX':
            I = inertia_of_box(mass, lx, ly, lz)
        elif coll_shape_obj_handle.ambf_collision_shape == 'SPHERE':
            I = inertia_of_sphere(mass, radius)
        elif coll_shape_obj_handle.ambf_collision_shape == 'CYLINDER':
            I = inertia_of_cylinder(mass, radius, height, axis)
        elif coll_shape_obj_handle.ambf_collision_shape == 'CONE':
            I = inertia_of_cone(mass, radius, height, axis)
        elif coll_shape_obj_handle.ambf_collision_shape == 'CAPSULE':
            I = inertia_of_capsule(mass, radius, height, axis)
    else:
        print('ERROR!, Not an understood shape or mesh')
        return

    # Parallel Axis Theorem
    off = obj_handle.ambf_rigid_body_linear_inertial_offset
    I[0] = I[0] + mass * (off[1] ** 2 + off[2] ** 2)
    I[1] = I[1] + mass * (off[0] ** 2 + off[2] ** 2)
    I[2] = I[2] + mass * (off[0] ** 2 + off[1] ** 2)
    ix = ambf_round(I[0])
    iy = ambf_round(I[1])
    iz = ambf_round(I[2])
    print(ix, iy, iz)
    return I


def create_capsule(height, radius, axis='Z'):
    if axis.upper() == 'X':
        axis_vec = mathutils.Vector((1.0, 0.0, 0.0))
        rot_axis_angle = mathutils.Vector((0.0, math.pi/2.0, 0.0))
    elif axis.upper() == 'Y':
        axis_vec = mathutils.Vector((0.0, 1.0, 0.0))
        rot_axis_angle = mathutils.Vector((math.pi/2.0, 0.0, 0.0))
    elif axis.upper() == 'Z':
        axis_vec = mathutils.Vector((0.0, 0.0, 1.0))
        rot_axis_angle = mathutils.Vector((0.0, 0.0, 0.0))
    else:
        raise ValueError
    caps_dist = height/2.0 - radius
    trunk_length = height - (2 * radius)
    bpy.ops.mesh.primitive_uv_sphere_add(radius=radius)
    sphere1 = get_active_object()
    sphere1.matrix_world.translation = sphere1.matrix_world.translation + axis_vec * caps_dist
    bpy.ops.mesh.primitive_uv_sphere_add(radius=radius)
    sphere2 = get_active_object()
    sphere2.matrix_world.translation = sphere2.matrix_world.translation - axis_vec * caps_dist
    bpy.ops.mesh.primitive_cylinder_add(radius=radius, depth=trunk_length, rotation=rot_axis_angle)
    cylinder = get_active_object()
    select_object(sphere1)
    select_object(sphere2)
    set_active_object(cylinder)
    bpy.ops.object.join()
    

def add_collision_shape_property(obj_handle, shape_type=None):
    obj_handle.ambf_collision_shape_prop_collection.add()
    cnt = len(obj_handle.ambf_collision_shape_prop_collection.items())
    prop_tuple = obj_handle.ambf_collision_shape_prop_collection.items()[cnt - 1]

    if shape_type is not None:
        prop_tuple[1].ambf_collision_shape = shape_type

    collision_shape_create_visual(obj_handle, prop_tuple[1])
    return prop_tuple[1]


def remove_collision_shape_property(obj_handle, idx=None):
    cnt = len(obj_handle.ambf_collision_shape_prop_collection.items())
    if idx is None:
        idx = cnt - 1

    if idx < 0 or idx >= cnt:
        print('ERROR! Object ', obj_handle.name, ' has only ', cnt, ' collision props')
        print('ERROR! Cannot remove at Idx: ', idx)
        return

    shape_prop = obj_handle.ambf_collision_shape_prop_collection.items()[cnt - 1][1]
    coll_shape_obj_handle = shape_prop.ambf_collision_shape_pointer
    bpy.data.objects.remove(coll_shape_obj_handle)
    obj_handle.ambf_collision_shape_prop_collection.remove(cnt - 1)


def estimate_collision_shape_geometry(obj_handle):
    if obj_handle.ambf_object_type == 'RIGID_BODY':

        if len(obj_handle.ambf_collision_shape_prop_collection.items()) == 0:
            add_collision_shape_property(obj_handle)
        # Don't bother if the shape is a compound shape for now. Let the
        # user calculate the geometries.
        if obj_handle.ambf_collision_type in ['MESH', 'SINGULAR_SHAPE']:
            dims = obj_handle.dimensions.copy()
            prop_group = obj_handle.ambf_collision_shape_prop_collection.items()[0][1]
            # Now we need to find out the geometry of the shape
            if prop_group.ambf_collision_shape == 'BOX':
                prop_group.ambf_collision_shape_disable_update_cbs = True
                prop_group.ambf_collision_shape_xyz_dims[0] = dims[0]
                prop_group.ambf_collision_shape_xyz_dims[1] = dims[1]
                prop_group.ambf_collision_shape_xyz_dims[2] = dims[2]
                prop_group.ambf_collision_shape_disable_update_cbs = False
                collision_shape_update_dimensions(prop_group)
            elif prop_group.ambf_collision_shape == 'SPHERE':
                prop_group.ambf_collision_shape_radius = max(dims) / 2
            elif prop_group.ambf_collision_shape in ['CYLINDER', 'CONE', 'CAPSULE']:
                major_ax_char, major_ax_idx = get_major_axis(dims)
                median_ax_char, median_ax_idx = get_median_axis(dims)
                prop_group.ambf_collision_shape_radius = dims[median_ax_idx] / 2.0
                prop_group.ambf_collision_shape_height = dims[major_ax_idx]
                prop_group.ambf_collision_shape_axis = major_ax_char.upper()
                

def collision_shape_update_dimensions(shape_prop):
    if shape_prop.ambf_collision_shape_disable_update_cbs:
        return

    coll_shape_obj_handle = shape_prop.ambf_collision_shape_pointer
    if coll_shape_obj_handle is None:
        return

    height = shape_prop.ambf_collision_shape_height
    radius = shape_prop.ambf_collision_shape_radius

    lx = shape_prop.ambf_collision_shape_xyz_dims[0]
    ly = shape_prop.ambf_collision_shape_xyz_dims[1]
    lz = shape_prop.ambf_collision_shape_xyz_dims[2]
    
    lx = ambf_round(lx)
    ly = ambf_round(ly)
    lz = ambf_round(lz)

    dim_old = coll_shape_obj_handle.dimensions.copy()
    scale_old = coll_shape_obj_handle.scale.copy()

    if shape_prop.ambf_collision_shape == 'BOX':
        coll_shape_obj_handle.scale[0] = scale_old[0] * lx / dim_old[0]
        coll_shape_obj_handle.scale[1] = scale_old[1] * ly / dim_old[1]
        coll_shape_obj_handle.scale[2] = scale_old[2] * lz / dim_old[2]

    elif shape_prop.ambf_collision_shape in ['CONE', 'CYLINDER', 'CAPSULE', 'SPHERE']:
        dir_axis = get_axis_idx(shape_prop.ambf_collision_shape_axis.upper())

        height_old = coll_shape_obj_handle.dimensions[dir_axis]
        radius_old = coll_shape_obj_handle.dimensions[(dir_axis + 1) % 3]

        if shape_prop.ambf_collision_shape == 'SPHERE':
            coll_shape_obj_handle.scale = scale_old * radius / radius_old * 2
        else: # For Cylinder, Cone and Capsule
            coll_shape_obj_handle.scale[dir_axis] = scale_old[dir_axis] * height / height_old
            coll_shape_obj_handle.scale[(dir_axis + 1) % 3] = scale_old[(dir_axis + 1) % 3] * radius / radius_old * 2
            coll_shape_obj_handle.scale[(dir_axis + 2) % 3] = scale_old[(dir_axis + 2) % 3] * radius / radius_old * 2


def collision_shape_update_local_offset(obj_handle, shape_prop):
    if shape_prop.ambf_collision_shape_disable_update_cbs:
        return

    coll_shape_obj_handle = shape_prop.ambf_collision_shape_pointer
    if coll_shape_obj_handle is None:
        return
    scale_old = coll_shape_obj_handle.scale.copy()
    T_p_w = obj_handle.matrix_world.copy()
    coll_shape_obj_handle.matrix_world = T_p_w

    euler_rot = mathutils.Euler((shape_prop.ambf_collision_shape_angular_offset[0],
                                 shape_prop.ambf_collision_shape_angular_offset[1],
                                 shape_prop.ambf_collision_shape_angular_offset[2]), 'ZYX')

    # Shape Offset in Inertial Frame
    R_c_p = euler_rot.to_matrix()
    T_c_p = R_c_p.to_4x4()
    T_c_p.translation.x = shape_prop.ambf_collision_shape_linear_offset[0]
    T_c_p.translation.y = shape_prop.ambf_collision_shape_linear_offset[1]
    T_c_p.translation.z = shape_prop.ambf_collision_shape_linear_offset[2]

    coll_shape_obj_handle.matrix_world = T_p_w @ T_c_p
    coll_shape_obj_handle.scale = scale_old
    

def set_3d_cursor_location(location):
    bpy.context.scene.cursor.location = location


def collision_shape_create_visual(obj_handle, shape_prop_group):
    cur_active_obj_handle = get_active_object()
    set_3d_cursor_location([0, 0, 0])
    select_all_objects(False)
    if shape_prop_group.ambf_collision_shape_pointer is None:
        height = shape_prop_group.ambf_collision_shape_height
        radius = shape_prop_group.ambf_collision_shape_radius

        lx = shape_prop_group.ambf_collision_shape_xyz_dims[0]
        ly = shape_prop_group.ambf_collision_shape_xyz_dims[1]
        lz = shape_prop_group.ambf_collision_shape_xyz_dims[2]
            
        if shape_prop_group.ambf_collision_shape == 'BOX':
            bpy.ops.mesh.primitive_cube_add(size=1.0)
            coll_shape_obj_handle = get_active_object()
            coll_shape_obj_handle.scale[0] = lx
            coll_shape_obj_handle.scale[1] = ly
            coll_shape_obj_handle.scale[2] = lz

        elif shape_prop_group.ambf_collision_shape == 'SPHERE':
            bpy.ops.mesh.primitive_uv_sphere_add(radius=radius)
            
        elif shape_prop_group.ambf_collision_shape in ['CONE', 'CYLINDER', 'CAPSULE']:
            if shape_prop_group.ambf_collision_shape_axis == 'X':
                dir_axis = 0
                rot_axis = mathutils.Vector((0, 1, 0))  # Choose y axis for rot
                rot_angle = math.pi/2
            elif shape_prop_group.ambf_collision_shape_axis == 'Y':
                dir_axis = 1
                rot_axis = mathutils.Vector((1, 0, 0))  # Choose y axis for rot
                rot_angle = -math.pi/2
            else:
                dir_axis = 2
                rot_axis = mathutils.Vector((0, 0, 1))
                rot_angle = 0
                
            rpy_rot = rot_axis * rot_angle

            if shape_prop_group.ambf_collision_shape == 'CONE':
                bpy.ops.mesh.primitive_cone_add(rotation=rpy_rot, radius1=radius, depth=height)

            elif shape_prop_group.ambf_collision_shape == 'CYLINDER':
                bpy.ops.mesh.primitive_cylinder_add(rotation=rpy_rot, radius=radius, depth=height)

            elif shape_prop_group.ambf_collision_shape == 'CAPSULE':
                # There is no primitive for capsule in Blender, so we
                # have to use a workaround using the sphere
                create_capsule(height=height, radius=radius, axis=shape_prop_group.ambf_collision_shape_axis)

            else:
                print("FAIL! Shouldn't Get Here")
                
        coll_shape_obj_handle = get_active_object()
        coll_shape_obj_handle.ambf_object_type = 'COLLISION_SHAPE'
        bpy.ops.object.transform_apply(scale=True, rotation=True)
        make_obj1_parent_of_obj2(obj_handle, coll_shape_obj_handle)
        shape_prop_group.ambf_collision_shape_pointer = coll_shape_obj_handle

        # Update the collision shape transform
        collision_shape_update_local_offset(obj_handle, shape_prop_group)

        ensure_collision_shape_material()

        # coll_shape_obj_handle.draw_type = 'WIRE'
        coll_shape_obj_handle.hide_select = True
        coll_shape_obj_handle.show_transparent = True
        coll_shape_obj_handle.data.materials.append(CommonConfig.collision_shape_material)
        hide_object(coll_shape_obj_handle, not obj_handle.ambf_collision_show_shapes_per_object)

        set_active_object(cur_active_obj_handle)

    else:
        coll_shape_obj_handle = shape_prop_group.ambf_collision_shape_pointer

    shape_number = 1
    for prop_tuple in obj_handle.ambf_collision_shape_prop_collection.items():
        if prop_tuple[1] == shape_prop_group:
            break
        shape_number = shape_number + 1

    coll_shape_obj_handle.name = obj_handle.name + '_coll_shape_' + str(shape_number)

    return coll_shape_obj_handle


def collision_shape_show_update_cb(self, context):
    for obj_handle in bpy.data.objects:
        if obj_handle.ambf_collision_type in ['SINGULAR_SHAPE', 'COMPOUND_SHAPE']:
            for prop_tuple in obj_handle.ambf_collision_shape_prop_collection.items():
                shape_prop_group = prop_tuple[1]
                coll_shape_obj = shape_prop_group.ambf_collision_shape_pointer
                if coll_shape_obj is None:
                    collision_shape_create_visual(obj_handle, shape_prop_group)
                    coll_shape_obj = shape_prop_group.ambf_collision_shape_pointer
                hide_object(coll_shape_obj, not context.scene.ambf_show_collision_shapes)


def collision_shape_dims_update_cb(self, context):
    obj_handle = context.object
    for prop_tuple in obj_handle.ambf_collision_shape_prop_collection.items():
        collision_shape_update_dimensions(prop_tuple[1])


def collision_shape_axis_update_cb(self, context):
    collision_shape_type_update_cb(self, context)


def collision_shape_type_update_cb(self, context):
    obj_handle = context.object
    for prop_tuple in obj_handle.ambf_collision_shape_prop_collection.items():
        shape_prop_group = prop_tuple[1]
        if shape_prop_group.ambf_collision_shape_pointer:
            bpy.data.objects.remove(shape_prop_group.ambf_collision_shape_pointer)

    if obj_handle.ambf_collision_type in ['SINGULAR_SHAPE', 'COMPOUND_SHAPE']:
        for prop_tuple in obj_handle.ambf_collision_shape_prop_collection.items():
            collision_shape_create_visual(obj_handle, prop_tuple[1])


def collision_shape_offset_update_cb(self, context):
    obj_handle = context.object
    for prop_tuple in obj_handle.ambf_collision_shape_prop_collection.items():
        collision_shape_update_local_offset(obj_handle, prop_tuple[1])


def rigid_body_collision_type_update_cb(self, context):
    if len(context.object.ambf_collision_shape_prop_collection.items()) == 0:
        add_collision_shape_property(context.object)


def collision_shape_show_per_object_update_cb(self, context):
    obj_handle = context.object
    if obj_handle.ambf_collision_type in ['SINGULAR_SHAPE', 'COMPOUND_SHAPE']:
        for prop_tuple in obj_handle.ambf_collision_shape_prop_collection.items():
            shape_prop_group = prop_tuple[1]
            coll_shape_obj = shape_prop_group.ambf_collision_shape_pointer
            if coll_shape_obj is None:
                collision_shape_create_visual(obj_handle, shape_prop_group)
                coll_shape_obj = shape_prop_group.ambf_collision_shape_pointer
            hide_object(coll_shape_obj, not obj_handle.ambf_collision_show_shapes_per_object)


def draw_collision_shape_prop(context, prop, box):
    sbox = box.box()
    col = sbox.column()
    col.prop(prop, 'ambf_collision_shape')
    col.scale_y = 1.5

    if prop.ambf_collision_shape in ['CYLINDER', 'CONE', 'CAPSULE']:
        row = sbox.row()
        split = row.split()

        col = split.column()
        col.prop(prop, 'ambf_collision_shape_axis')

        col = split.column()
        col.prop(prop, 'ambf_collision_shape_radius')

        col = split.column()
        col.prop(prop, 'ambf_collision_shape_height')

    elif prop.ambf_collision_shape == 'SPHERE':
        row = sbox.row()
        row.prop(prop, 'ambf_collision_shape_radius')

    elif prop.ambf_collision_shape == 'BOX':
        col = sbox.column()
        col.prop(prop, 'ambf_collision_shape_xyz_dims')

    if context.object.ambf_collision_type == 'SINGULAR_SHAPE':
        sbox.separator()
        col = sbox.column()
        col.operator("ambf.estimate_shape_offset_per_object")

    sbox.separator()
    col = sbox.column()
    col = col.split(factor=0.5)
    col.alignment = 'EXPAND'
    col.prop(prop, 'ambf_collision_shape_linear_offset')

    col = col.column()
    col.alignment = 'EXPAND'
    col.prop(prop, 'ambf_collision_shape_angular_offset')


class AMBF_OT_generate_ambf_file(Operator):
    """Tooltip"""
    bl_idname = "ambf.add_generate_ambf_file"
    bl_label = "Write AMBF Description File (ADF)"
    bl_description = "This generated the AMBF Config file in the location and filename specified in the field" \
                     " above"

    def __init__(self):
        self._body_names_list = []
        self._joint_names_list = []
        self.body_name_prefix = 'BODY '
        self.joint_name_prefix = 'JOINT '
        self._adf = None
        self._context = None

    def execute(self, context):
        self._context = context
        self.generate_adf()
        return {'FINISHED'}

    # This joint adds the body prefix str if set to all the bodies in the AMBF
    def add_body_prefix_str(self, urdf_body_str):
        return self.body_name_prefix + urdf_body_str

    # This method add the joint prefix if set to all the joints in AMBF
    def add_joint_prefix_str(self, urdf_joint_str):
        return self.joint_name_prefix + urdf_joint_str

    def generate_body_data_from_ambf_rigid_body(self, adf_data, obj_handle):

        if obj_handle.ambf_object_type != 'RIGID_BODY':
            return

        # The object is unlinked from the scene. Don't write it
        if self._context.scene.objects.get(obj_handle.name) is None:
            return

        if is_object_hidden(obj_handle) is True:
            return

        body = BodyTemplate()
        body_data = body._adf_data

        if not compare_body_namespace_with_global(obj_handle.name):
            if get_body_namespace(obj_handle.name) != '':
                body_data['namespace'] = get_body_namespace(obj_handle.name)

        obj_handle_name = remove_namespace_prefix(obj_handle.name)

        body_yaml_name = self.add_body_prefix_str(obj_handle_name)
        output_mesh = bpy.context.scene.ambf_meshes_save_type
        body_data['name'] = obj_handle_name
        
        body_data['passive'] = obj_handle.ambf_rigid_body_passive
        if obj_handle.ambf_rigid_body_passive:
            body_data['publish children names'] = False
            body_data['publish joint names'] = False
            body_data['publish joint positions'] = False
        else:
            body_data['publish children names'] = obj_handle.ambf_rigid_body_publish_children_names
            body_data['publish joint names'] = obj_handle.ambf_rigid_body_publish_joint_names
            body_data['publish joint positions'] = obj_handle.ambf_rigid_body_publish_joint_positions

        world_pos = obj_handle.matrix_world.translation
        world_rot = obj_handle.matrix_world.to_euler()
        body_pos = body_data['location']['position']
        body_rot = body_data['location']['orientation']
        body_pos['x'] = ambf_round(world_pos.x)
        body_pos['y'] = ambf_round(world_pos.y)
        body_pos['z'] = ambf_round(world_pos.z)
        body_rot['r'] = ambf_round(world_rot[0])
        body_rot['p'] = ambf_round(world_rot[1])
        body_rot['y'] = ambf_round(world_rot[2])

        if obj_handle.type == 'EMPTY':
            body_data['mesh'] = ''
            if obj_handle_name in ['world', 'World', 'WORLD']:
                body_data['mass'] = 0
            else:
                if obj_handle.ambf_rigid_body_is_static:
                    body_data['mass'] = 0.0
                else:
                    body_data['mass'] = obj_handle.ambf_rigid_body_mass
                    if obj_handle.ambf_rigid_body_specify_inertia:
                        body_data['inertia'] = {'ix': ambf_round(obj_handle.ambf_rigid_body_inertia_x),
                                                'iy': ambf_round(obj_handle.ambf_rigid_body_inertia_y),
                                                'iz': ambf_round(obj_handle.ambf_rigid_body_inertia_z)}
                    else:
                        body_data['inertia'] = {'ix': 0.01, 'iy': 0.01, 'iz': 0.01}

        elif obj_handle.type == 'MESH':

            if obj_handle.ambf_rigid_body_is_static:
                body_data['mass'] = 0.0
            else:
                body_data['mass'] = ambf_round(obj_handle.ambf_rigid_body_mass)
                if obj_handle.ambf_rigid_body_specify_inertia:
                    body_data['inertia'] = {'ix': ambf_round(obj_handle.ambf_rigid_body_inertia_x),
                                            'iy': ambf_round(obj_handle.ambf_rigid_body_inertia_y),
                                            'iz': ambf_round(obj_handle.ambf_rigid_body_inertia_z)}
                else:
                    # We can delete the inertia as it will be estimated in AMBF
                    del body_data['inertia']

            body_data['friction'] = {'static': ambf_round(obj_handle.ambf_rigid_body_static_friction),
                                     'rolling': ambf_round(obj_handle.ambf_rigid_body_rolling_friction)}

            body_data['restitution'] = ambf_round(obj_handle.ambf_rigid_body_restitution)

            body_data['damping'] = {'linear': ambf_round(obj_handle.ambf_rigid_body_linear_damping),
                                    'angular': ambf_round(obj_handle.ambf_rigid_body_angular_damping)}

            body_data['visible'] = obj_handle.ambf_object_visible

            body_data['collision groups'] = [idx for idx, chk in enumerate(obj_handle.ambf_collision_groups) if chk == True]

            if obj_handle.ambf_collision_margin_enable is True:
                body_data['collision margin'] = ambf_round(obj_handle.ambf_collision_margin)

            if obj_handle.ambf_collision_type == 'MESH':
                body_data['collision mesh type'] = obj_handle.ambf_collision_mesh_type
            elif obj_handle.ambf_collision_type == 'SINGULAR_SHAPE':
                shape_prop_group = obj_handle.ambf_collision_shape_prop_collection.items()[0][1]
                body_data['collision shape'] = shape_prop_group.ambf_collision_shape
                bcg = OrderedDict()
                dims = obj_handle.dimensions.copy()
                # Now we need to find out the geometry of the shape
                if shape_prop_group.ambf_collision_shape == 'BOX':
                    bcg = get_xyz_ordered_dict()
                    bcg['x'] = ambf_round(shape_prop_group.ambf_collision_shape_xyz_dims[0])
                    bcg['y'] = ambf_round(shape_prop_group.ambf_collision_shape_xyz_dims[1])
                    bcg['z'] = ambf_round(shape_prop_group.ambf_collision_shape_xyz_dims[2])
                elif shape_prop_group.ambf_collision_shape == 'SPHERE':
                    bcg = {'radius': ambf_round(shape_prop_group.ambf_collision_shape_radius)}
                elif shape_prop_group.ambf_collision_shape in ['CONE', 'CYLINDER', 'CAPSULE']:
                    bcg = {'radius': ambf_round(shape_prop_group.ambf_collision_shape_radius),
                           'height': ambf_round(shape_prop_group.ambf_collision_shape_height),
                           'axis': shape_prop_group.ambf_collision_shape_axis}
                body_data['collision geometry'] = bcg

                offset = get_pose_ordered_dict()
                offset['position']['x'] = ambf_round(shape_prop_group.ambf_collision_shape_linear_offset[0])
                offset['position']['y'] = ambf_round(shape_prop_group.ambf_collision_shape_linear_offset[1])
                offset['position']['z'] = ambf_round(shape_prop_group.ambf_collision_shape_linear_offset[2])
                offset['orientation']['r'] = ambf_round(shape_prop_group.ambf_collision_shape_angular_offset[0])
                offset['orientation']['p'] = ambf_round(shape_prop_group.ambf_collision_shape_angular_offset[1])
                offset['orientation']['y'] = ambf_round(shape_prop_group.ambf_collision_shape_angular_offset[2])
                body_data['collision offset'] = offset
            elif obj_handle.ambf_collision_type == 'COMPOUND_SHAPE':
                if 'collision shape' in body_data:
                    del body_data['collision shape']
                compound_shape = []
                shape_count = 0
                for prop_tuple in obj_handle.ambf_collision_shape_prop_collection.items():
                    shape_prop_group = prop_tuple[1]
                    bcg = OrderedDict()
                    bcg['name'] = str(shape_count + 1)
                    bcg['shape'] = shape_prop_group.ambf_collision_shape
                    bcg['geometry'] = OrderedDict()
                    # Now we need to find out the geometry of the shape
                    if shape_prop_group.ambf_collision_shape == 'BOX':
                        bcg['geometry'] = {'x': ambf_round(shape_prop_group.ambf_collision_shape_xyz_dims[0]),
                                           'y': ambf_round(shape_prop_group.ambf_collision_shape_xyz_dims[1]),
                                           'z': ambf_round(shape_prop_group.ambf_collision_shape_xyz_dims[2])}
                    elif shape_prop_group.ambf_collision_shape == 'SPHERE':
                        bcg['geometry'] = {'radius': ambf_round(shape_prop_group.ambf_collision_shape_radius)}
                    elif shape_prop_group.ambf_collision_shape in ['CONE', 'CYLINDER', 'CAPSULE']:
                        geometry = dict({'radius': 0, 'height': 0, 'axis': 'Z'})
                        geometry['radius'] = ambf_round(shape_prop_group.ambf_collision_shape_radius)
                        geometry['height'] = ambf_round(shape_prop_group.ambf_collision_shape_height)
                        geometry['axis'] = shape_prop_group.ambf_collision_shape_axis
                        bcg['geometry'] = geometry

                    offset = get_pose_ordered_dict()
                    offset['position']['x'] = shape_prop_group.ambf_collision_shape_linear_offset[0]
                    offset['position']['y'] = shape_prop_group.ambf_collision_shape_linear_offset[1]
                    offset['position']['z'] = shape_prop_group.ambf_collision_shape_linear_offset[2]
                    offset['orientation']['r'] = shape_prop_group.ambf_collision_shape_angular_offset[0]
                    offset['orientation']['p'] = shape_prop_group.ambf_collision_shape_angular_offset[1]
                    offset['orientation']['y'] = shape_prop_group.ambf_collision_shape_angular_offset[2]
                    bcg['offset'] = offset
                    compound_shape.append(bcg)
                    shape_count = shape_count + 1

                body_data['compound collision shape'] = compound_shape

            body_data['mesh'] = obj_handle_name + '.' + output_mesh
            xyz_inertial_off = get_xyz_ordered_dict()
            xyz_inertial_off['x'] = ambf_round(obj_handle.ambf_rigid_body_linear_inertial_offset[0])
            xyz_inertial_off['y'] = ambf_round(obj_handle.ambf_rigid_body_linear_inertial_offset[1])
            xyz_inertial_off['z'] = ambf_round(obj_handle.ambf_rigid_body_linear_inertial_offset[2])

            body_data['inertial offset']['position'] = xyz_inertial_off

            if obj_handle.data.materials:
                del body_data['color']
                body_data['color components'] = OrderedDict()
                body_data['color components'] = {'diffuse': {'r': 1.0, 'g': 1.0, 'b': 1.0},
                                                 'specular': {'r': 1.0, 'g': 1.0, 'b': 1.0},
                                                 'ambient': {'level': 0.5},
                                                 'transparency': 1.0}

                mat = obj_handle.data.materials[0]
                body_data['color components']['diffuse']['r'] = ambf_round(mat.diffuse_color[0])
                body_data['color components']['diffuse']['g'] = ambf_round(mat.diffuse_color[1])
                body_data['color components']['diffuse']['b'] = ambf_round(mat.diffuse_color[2])

                spec_r = mat.diffuse_color[0] * mat.specular_intensity
                spec_g = mat.diffuse_color[1] * mat.specular_intensity
                spec_b = mat.diffuse_color[1] * mat.specular_intensity

                body_data['color components']['specular']['r'] = ambf_round(spec_r)
                body_data['color components']['specular']['g'] = ambf_round(spec_g)
                body_data['color components']['specular']['b'] = ambf_round(spec_b)

                body_data['color components']['ambient']['level'] = 1.0

                body_data['color components']['transparency'] = ambf_round(mat.diffuse_color[3])

            # Set the body controller data from the controller props
            if obj_handle.ambf_rigid_body_enable_controllers is True:
                _controller_gains = OrderedDict()
                _lin_gains = OrderedDict()
                _ang_gains = OrderedDict()
                _lin_gains['P'] = ambf_round(obj_handle.ambf_rigid_body_linear_controller_p_gain)
                _lin_gains['I'] = ambf_round(obj_handle.ambf_rigid_body_linear_controller_i_gain)
                _lin_gains['D'] = ambf_round(obj_handle.ambf_rigid_body_linear_controller_d_gain)

                _ang_gains['P'] = ambf_round(obj_handle.ambf_rigid_body_angular_controller_p_gain)
                _ang_gains['I'] = ambf_round(obj_handle.ambf_rigid_body_angular_controller_i_gain)
                _ang_gains['D'] = ambf_round(obj_handle.ambf_rigid_body_angular_controller_d_gain)

                _controller_gains['linear'] = _lin_gains
                _controller_gains['angular'] = _ang_gains
                body_data['controller'] = _controller_gains
                body_data['controller output type'] = obj_handle.ambf_rigid_body_controller_output_type
            else:
                if 'controller' in body_data:
                    del body_data['controller']

        adf_data[body_yaml_name] = body_data
        self._body_names_list.append(body_yaml_name)

    def generate_joint_data_from_ambf_constraint(self, adf_data, joint_obj_handle):

        if joint_obj_handle.ambf_object_type != 'CONSTRAINT':
            return

        if is_object_hidden(joint_obj_handle) is True:
            return

        _valid_constraint = True
        if joint_obj_handle.ambf_object_parent:
            if is_object_hidden(joint_obj_handle.ambf_object_parent) is True:
                _valid_constraint = False
        else:
            _valid_constraint = False

        if joint_obj_handle.ambf_object_child:
            if is_object_hidden(joint_obj_handle.ambf_object_child) is True:
                _valid_constraint = False
        else:
            _valid_constraint = False

        if not _valid_constraint:
            print('ERROR! CONSTRAINT: ', joint_obj_handle.name, ' IS NOT A VALID CONSTRAINT, SKIPPING')
            return

        joint_template = JointTemplate()
        joint_data = joint_template._adf_data
        parent_obj_handle = joint_obj_handle.ambf_object_parent
        child_obj_handle = joint_obj_handle.ambf_object_child
        parent_obj_handle_name = remove_namespace_prefix(parent_obj_handle.name)
        child_obj_handle_name = remove_namespace_prefix(child_obj_handle.name)
        obj_handle_name = remove_namespace_prefix(joint_obj_handle.name)

        if joint_obj_handle.ambf_constraint_name == '':
            joint_data['name'] = parent_obj_handle_name + "-" + child_obj_handle_name
        else:
            joint_data['name'] = joint_obj_handle.ambf_constraint_name

        joint_data['parent'] = self.add_body_prefix_str(parent_obj_handle_name)
        joint_data['child'] = self.add_body_prefix_str(child_obj_handle_name)
        constraint_axis = self.get_axis_of_ambf_constraint(joint_obj_handle)
        parent_pivot, parent_axis = self.compute_body_pivot_and_axis(
            parent_obj_handle, joint_obj_handle, constraint_axis)
        child_pivot, child_axis = self.compute_body_pivot_and_axis(
            child_obj_handle, joint_obj_handle, constraint_axis)

        parent_pivot_data = joint_data["parent pivot"]
        parent_axis_data = joint_data["parent axis"]
        parent_pivot_data['x'] = ambf_round(parent_pivot.x)
        parent_pivot_data['y'] = ambf_round(parent_pivot.y)
        parent_pivot_data['z'] = ambf_round(parent_pivot.z)
        parent_axis_data['x'] = ambf_round(parent_axis.x)
        parent_axis_data['y'] = ambf_round(parent_axis.y)
        parent_axis_data['z'] = ambf_round(parent_axis.z)

        child_pivot_data = joint_data["child pivot"]
        child_axis_data = joint_data["child axis"]
        child_pivot_data['x'] = ambf_round(child_pivot.x)
        child_pivot_data['y'] = ambf_round(child_pivot.y)
        child_pivot_data['z'] = ambf_round(child_pivot.z)
        child_axis_data['x'] = ambf_round(child_axis.x)
        child_axis_data['y'] = ambf_round(child_axis.y)
        child_axis_data['z'] = ambf_round(child_axis.z)

        # This method assigns joint limits, joint_type, joint damping and stiffness for spring joints
        self.assign_joint_params_from_ambf_constraint(joint_obj_handle, joint_data)
        # The use of pivot and axis does not fully define the connection and relative
        # transform between two bodies it is very likely that we need an additional offset
        # of the child body as in most of the cases of URDF's For this purpose, we calculate
        # the offset as follows
        r_c_p_ambf = rot_matrix_from_vecs(child_axis, parent_axis)
        r_p_c_ambf = r_c_p_ambf.to_3x3().copy()
        r_p_c_ambf.invert()
        t_p_w = parent_obj_handle.matrix_world.copy()
        r_w_p = t_p_w.to_3x3().copy()
        r_w_p.invert()
        r_c_w = child_obj_handle.matrix_world.to_3x3().copy()
        r_c_p_blender = r_w_p @ r_c_w
        r_angular_offset = r_p_c_ambf @ r_c_p_blender
        child_offset_axis_angle = r_angular_offset.to_quaternion().to_axis_angle()

        if abs(child_offset_axis_angle[1]) > 0.0:
            offset_angle = ambf_round(child_offset_axis_angle[1])

            if abs(1.0 - child_axis.dot(child_offset_axis_angle[0])) < 0.1:
                joint_data['offset'] = offset_angle
                # print ': SAME DIRECTION'
            elif abs(1.0 + child_axis.dot(child_offset_axis_angle[0])) < 0.1:
                joint_data['offset'] = -offset_angle
                # print ': OPPOSITE DIRECTION'
            else:
                print('ERROR: CALCULATION OF CHILD OFFSET: (', sys._getframe().f_code.co_name, ') (', joint_data['name'], ') SHOULD\'NT GET HERE: ', offset_angle)
                print("CHILD AXIS ", child_axis)
                print("PARENT AXIS ", parent_axis)
                print("CHILD OFFSET AXIS ", child_offset_axis_angle[0])
                print("DOT(ch_axis, ch_off_axis ", child_axis.dot(child_offset_axis_angle[0]))
                print("CHILD OFFSET ANGLE ", child_offset_axis_angle[1])

        # Should also do the same for joint
        joint_axis = mathutils.Vector(constraint_axis[0:3])
        r_j_p_ambf = rot_matrix_from_vecs(joint_axis, parent_axis)
        r_p_j_ambf = r_j_p_ambf.to_3x3().copy()
        r_p_j_ambf.invert()
        t_p_w = parent_obj_handle.matrix_world.copy()
        r_w_p = t_p_w.to_3x3().copy()
        r_w_p.invert()
        r_j_w_blender = joint_obj_handle.matrix_world.to_3x3().copy()
        r_j_p_blender = r_w_p @ r_j_w_blender
        r_angular_offset = r_p_j_ambf @ r_j_p_blender
        joint_offset_axis_angle = r_angular_offset.to_quaternion().to_axis_angle()

        if abs(joint_offset_axis_angle[1]) > 0.0:
            joint_offset_angle = ambf_round(joint_offset_axis_angle[1])

            if abs(1.0 - joint_axis.dot(joint_offset_axis_angle[0])) < 0.1:
                joint_data['joint offset'] = joint_offset_angle
                # print ': SAME DIRECTION'
            elif abs(1.0 + joint_axis.dot(joint_offset_axis_angle[0])) < 0.1:
                joint_data['joint offset'] = -joint_offset_angle
                # print ': OPPOSITE DIRECTION'
            else:
                print('ERROR: CALCULATION OF PARENT OFFSET: (', sys._getframe().f_code.co_name, ') (', joint_data['name'], ') SHOULD\'NT GET HERE: ', joint_offset_angle)
                print("JOINT AXIS ", joint_axis)
                print("PARENT AXIS ", parent_axis)
                print("JOINT OFFSET AXIS ", joint_offset_axis_angle[0])
                print("DOT(jnt_axis, pa_off_axis ", joint_axis.dot(joint_offset_axis_angle[0]))
                print("JOINT OFFSET ANGLE ", joint_offset_axis_angle[1])



        joint_yaml_name = self.add_joint_prefix_str(joint_data['name'])
        adf_data[joint_yaml_name] = joint_data
        self._joint_names_list.append(joint_yaml_name)

    # Get the joints axis as a vector
    def get_axis_of_ambf_constraint(self, joint_obj_handle):
        joint_axis = mathutils.Vector([0, 0, 1])
        # If the constraint is multi DOF, ignore checking for the axis and always return n_z
        if joint_obj_handle.ambf_constraint_type in ['CONE_TWIST', 'SIX_DOF', 'SIX_DOF_SPRING']:
            return mathutils.Vector([0, 0, 1])
        else:
            if joint_obj_handle.ambf_constraint_axis == 'X':
                joint_axis = mathutils.Vector([1, 0, 0])
            elif joint_obj_handle.ambf_constraint_axis == 'Y':
                joint_axis = mathutils.Vector([0, 1, 0])
            elif joint_obj_handle.ambf_constraint_axis == 'Z':
                joint_axis = mathutils.Vector([0, 0, 1])
            else:
                print("ERROR! JOINT AXES NOT UNDERSTOOD")

        return joint_axis

    # Since changing the scale of the bodies directly impacts the rotation matrix, we have
    # to take that into account while calculating offset of child from parent using
    # transform manipulation
    def compute_body_pivot_and_axis(self, parent, child, constraint_axis):
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

    # Assign the joint parameters that include joint limits, type, damping and joint stiffness for spring joints
    def assign_joint_params_from_ambf_constraint(self, joint_obj_handle, joint_data):
        if joint_obj_handle.ambf_constraint_type == 'REVOLUTE':
            joint_data['type'] = 'revolute'
        elif joint_obj_handle.ambf_constraint_type == 'PRISMATIC':
            joint_data['type'] = 'prismatic'
        elif joint_obj_handle.ambf_constraint_type == 'LINEAR_SPRING':
            joint_data['type'] = 'linear spring'
        elif joint_obj_handle.ambf_constraint_type == 'TORSION_SPRING':
            joint_data['type'] = 'angular spring'
        elif joint_obj_handle.ambf_constraint_type == 'FIXED':
            joint_data['type'] = 'fixed'
        elif joint_obj_handle.ambf_constraint_type == 'P2P':
            joint_data['type'] = 'p2p'
        elif joint_obj_handle.ambf_constraint_type == 'CONE_TWIST':
            joint_data['type'] = 'cone twist'
        elif joint_obj_handle.ambf_constraint_type == 'SIX_DOF':
            joint_data['type'] = 'six dof'
        elif joint_obj_handle.ambf_constraint_type == 'SIX_DOF_SPRING':
            joint_data['type'] = 'six dof spring'

        if joint_obj_handle.ambf_constraint_type in ['REVOLUTE', 'TORSION_SPRING']:
            if joint_obj_handle.ambf_constraint_limits_enable:
                joint_data['joint limits'] = {'low': ambf_round(math.radians(joint_obj_handle.ambf_constraint_limits_lower)),
                                        'high': ambf_round(math.radians(joint_obj_handle.ambf_constraint_limits_higher))}
            else:
                del joint_data['joint limits']

            joint_data['max motor impulse'] = joint_obj_handle.ambf_constraint_max_motor_impulse

        elif joint_obj_handle.ambf_constraint_type in ['PRISMATIC', 'LINEAR_SPRING']:
            if joint_obj_handle.ambf_constraint_limits_enable:
                joint_data['joint limits'] = {'low': ambf_round(joint_obj_handle.ambf_constraint_limits_lower),
                                        'high': ambf_round(joint_obj_handle.ambf_constraint_limits_higher)}
            else:
                del joint_data['joint limits']

            joint_data['max motor impulse'] = joint_obj_handle.ambf_constraint_max_motor_impulse

        elif joint_obj_handle.ambf_constraint_type == 'CONE_TWIST':
            lims = {'x', ambf_round(math.radians(joint_obj_handle.ambf_constraint_cone_twist_limits[0])),
                    'y', ambf_round(math.radians(joint_obj_handle.ambf_constraint_cone_twist_limits[1])),
                    'z', ambf_round(math.radians(joint_obj_handle.ambf_constraint_cone_twist_limits[2]))}
            joint_data['joint limits'] = lims

        elif joint_obj_handle.ambf_constraint_type in ['SIX_DOF', 'SIX_DOF_SPRING']:
            ang_lims_low = {'x': ambf_round(math.radians(joint_obj_handle.ambf_constraint_six_dof_limits_low_angular[0])),
                            'y': ambf_round(math.radians(joint_obj_handle.ambf_constraint_six_dof_limits_low_angular[1])),
                            'z': ambf_round(math.radians(joint_obj_handle.ambf_constraint_six_dof_limits_low_angular[2]))}

            ang_lims_hig = {'x': ambf_round(math.radians(joint_obj_handle.ambf_constraint_six_dof_limits_high_angular[0])),
                            'y': ambf_round(math.radians(joint_obj_handle.ambf_constraint_six_dof_limits_high_angular[1])),
                            'z': ambf_round(math.radians(joint_obj_handle.ambf_constraint_six_dof_limits_high_angular[2]))}

            lin_lims_low = {'x': ambf_round(joint_obj_handle.ambf_constraint_six_dof_limits_low_linear[0]),
                            'y': ambf_round(joint_obj_handle.ambf_constraint_six_dof_limits_low_linear[1]),
                            'z': ambf_round(joint_obj_handle.ambf_constraint_six_dof_limits_low_linear[2])}

            lin_lims_hig = {'x': ambf_round(joint_obj_handle.ambf_constraint_six_dof_limits_high_linear[0]),
                            'y': ambf_round(joint_obj_handle.ambf_constraint_six_dof_limits_high_linear[1]),
                            'z': ambf_round(joint_obj_handle.ambf_constraint_six_dof_limits_high_linear[2])}

            joint_data['joint limits'] = {'angular': {'low': ang_lims_low, 'high': ang_lims_hig},
                                          'linear': {'low': lin_lims_low, 'high': lin_lims_hig}}

        if joint_obj_handle.ambf_constraint_type in ['LINEAR_SPRING', 'TORSION_SPRING']:
            joint_data['stiffness'] = ambf_round(joint_obj_handle.ambf_constraint_stiffness)
            joint_data['equilibrium point'] = ambf_round(math.radians(joint_obj_handle.ambf_constraint_equilibrium_point))

        elif joint_obj_handle.ambf_constraint_type == 'SIX_DOF_SPRING':
            ang_stiffness = {'x': ambf_round(joint_obj_handle.ambf_constraint_six_dof_stiffness_angular[0]),
                             'y': ambf_round(joint_obj_handle.ambf_constraint_six_dof_stiffness_angular[1]),
                             'z': ambf_round(joint_obj_handle.ambf_constraint_six_dof_stiffness_angular[2])}

            lin_stiffness = {'x': ambf_round(joint_obj_handle.ambf_constraint_six_dof_stiffness_linear[0]),
                             'y': ambf_round(joint_obj_handle.ambf_constraint_six_dof_stiffness_linear[1]),
                             'z': ambf_round(joint_obj_handle.ambf_constraint_six_dof_stiffness_linear[2])}

            joint_data['stiffness'] = {'angular': ang_stiffness, 'linear': lin_stiffness}

            ang_equilib = {'x': ambf_round(math.radians(joint_obj_handle.ambf_constraint_six_dof_equilibrium_angular[0])),
                           'y': ambf_round(math.radians(joint_obj_handle.ambf_constraint_six_dof_equilibrium_angular[1])),
                           'z': ambf_round(math.radians(joint_obj_handle.ambf_constraint_six_dof_equilibrium_angular[2]))}

            lin_equilib = {'x': ambf_round(joint_obj_handle.ambf_constraint_six_dof_equilibrium_linear[0]),
                           'y': ambf_round(joint_obj_handle.ambf_constraint_six_dof_equilibrium_linear[1]),
                           'z': ambf_round(joint_obj_handle.ambf_constraint_six_dof_equilibrium_linear[2])}

            joint_data['equilibrium point'] = {'angular': ang_equilib, 'linear': lin_equilib}

        else:
            if 'stiffness' in joint_data:
                del joint_data['stiffness']

        if joint_obj_handle.ambf_constraint_type in ['REVOLUTE', 'TORSION_SPRING', 'PRISMATIC', 'LINEAR_SPRING', 'CONE_TWIST']:
            joint_data['damping'] = ambf_round(joint_obj_handle.ambf_constraint_damping)

        elif joint_obj_handle.ambf_constraint_type == 'SIX_DOF_SPRING':
            ang_damping = {'x': ambf_round(joint_obj_handle.ambf_constraint_six_dof_damping_angular[0]),
                           'y': ambf_round(joint_obj_handle.ambf_constraint_six_dof_damping_angular[1]),
                           'z': ambf_round(joint_obj_handle.ambf_constraint_six_dof_damping_angular[2])}
            lin_damping = {'x': ambf_round(joint_obj_handle.ambf_constraint_six_dof_damping_linear[0]),
                           'y': ambf_round(joint_obj_handle.ambf_constraint_six_dof_damping_linear[1]),
                           'z': ambf_round(joint_obj_handle.ambf_constraint_six_dof_damping_linear[2])}
            joint_data['damping'] = {'angular': ang_damping, 'linear': lin_damping}



        # Set the joint controller gains data from the joint controller props
        if joint_obj_handle.ambf_constraint_enable_controller_gains:
            _gains = OrderedDict()
            joint_data['controller']['P'] = ambf_round(joint_obj_handle.ambf_constraint_controller_p_gain)
            joint_data['controller']['I'] = ambf_round(joint_obj_handle.ambf_constraint_controller_i_gain)
            joint_data['controller']['D'] = ambf_round(joint_obj_handle.ambf_constraint_controller_d_gain)
            joint_data['controller output type'] = joint_obj_handle.ambf_constraint_controller_output_type
        else:
            del joint_data['controller']
            
        joint_data['enable feedback'] = joint_obj_handle.ambf_constraint_enable_feedback

        joint_data['passive'] = joint_obj_handle.ambf_constraint_passive

    def generate_adf(self):
        num_objs = len(bpy.data.objects)
        save_to = bpy.path.abspath(self._context.scene.ambf_adf_path)
        filename = os.path.basename(save_to)
        save_dir = os.path.dirname(save_to)
        if not filename:
            filename = 'default.yaml'
        output_filename = os.path.join(save_dir, filename)
        # if a file exists by that name, save a backup
        if os.path.isfile(output_filename):
            os.rename(output_filename, output_filename + '.old')
        output_file = open(output_filename, 'w')
        print('Output filename is: ', output_filename)

        # For inorder processing, set the bodies and joints tag at the top of the map
        self._adf = OrderedDict()
        
        self._adf['bodies'] = []
        self._adf['joints'] = []
        print('SAVE PATH', bpy.path.abspath(save_dir))
        print('AMBF CONFIG PATH', bpy.path.abspath(self._context.scene.ambf_meshes_path))
        rel_mesh_path = os.path.relpath(bpy.path.abspath(self._context.scene.ambf_meshes_path), bpy.path.abspath(save_dir))

        self._adf['high resolution path'] = rel_mesh_path + '/high_res/'
        self._adf['low resolution path'] = rel_mesh_path + '/low_res/'

        self._adf['ignore inter-collision'] = self._context.scene.ambf_ignore_inter_collision

        update_global_namespace(self._context)

        if CommonConfig.namespace is not "":
            self._adf['namespace'] = CommonConfig.namespace

        # We want in-order processing, so make sure to
        # add bodies to ambf in a hierarchial fashion.

        _heirarichal_objects_list = populate_heirarchial_tree()

        for obj_handle in _heirarichal_objects_list:
            self.generate_body_data_from_ambf_rigid_body(self._adf, obj_handle)

        for obj_handle in _heirarichal_objects_list:
            self.generate_joint_data_from_ambf_constraint(self._adf, obj_handle)

        # Now populate the bodies and joints tag
        self._adf['bodies'] = self._body_names_list
        self._adf['joints'] = self._joint_names_list
        
        yaml.dump(self._adf, output_file)

        header_str = "# AMBF Version: %s\n" \
                     "# Generated By: ambf_addon for Blender %s\n" \
                     "# Link: %s\n" \
                     "# Generated on: %s\n"\
                     % (str(bl_info['version']).replace(', ', '.'),
                        str(bl_info['blender']).replace(', ', '.'),
                        bl_info['wiki_url'],
                        datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
        prepend_comment_to_file(output_filename, header_str)


class AMBF_OT_save_meshes(Operator):
    bl_idname = "ambf.save_meshes"
    bl_label = "Save Meshes"
    bl_description = "This saves the meshes in base folder specifed in the field above. Two folders" \
                     " are created in the base folder named, \"high_res\" and \"low_res\" to store the" \
                     " high-res and low-res meshes separately"

    def execute(self, context):
        replace_dot_from_object_names()
        self.save_meshes(context)
        return {'FINISHED'}

    # This recursive function is specialized to deal with
    # tree based hierarchy. In such cases we have to ensure
    # to move the parent to origin first and then its children successively
    # otherwise moving any parent after its child has been moved with move the
    # child as well
    def set_to_origin(self, p_obj_handle, obj_name_mat_list):
        if p_obj_handle.children is None:
            return
        obj_name_mat_list.append([p_obj_handle.name, p_obj_handle.matrix_world.copy()])
        # Since setting the world transform clears the embedded scale
        # of the object, we need to re-scale the obj_handle after putting it to origin
        scale_mat = mathutils.Matrix()
        scale_mat = scale_mat.Scale(p_obj_handle.matrix_world.median_scale, 4)
        p_obj_handle.matrix_world.identity()
        p_obj_handle.matrix_world = scale_mat
        for c_obj_handle in p_obj_handle.children:
            self.set_to_origin(c_obj_handle, obj_name_mat_list)

    # Since Blender exports meshes w.r.t world transform and not the
    # the local mesh transform, we explicitly push each obj_handle to origin
    # and remember its world transform for putting it back later on
    def set_all_meshes_to_origin(self):
        obj_name_mat_list = []
        for p_obj_handle in bpy.data.objects:
            if p_obj_handle.parent is None:
                self.set_to_origin(p_obj_handle, obj_name_mat_list)
        return obj_name_mat_list

    # This recursive function works in similar fashion to the
    # set_to_origin function, but uses the know default transform
    # to set the tree back to default in a hierarchial fashion
    def reset_back_to_default(self, p_obj_handle, obj_name_mat_list):
        if p_obj_handle.children is None:
            return
        for item in obj_name_mat_list:
            if p_obj_handle.name == item[0]:
                p_obj_handle.matrix_world = item[1]
        for c_obj_handle in p_obj_handle.children:
            self.reset_back_to_default(c_obj_handle, obj_name_mat_list)

    def reset_meshes_to_original_position(self, obj_name_mat_list):
        for p_obj_handle in bpy.data.objects:
            if p_obj_handle.parent is None:
                self.reset_back_to_default(p_obj_handle, obj_name_mat_list)

    def save_body_textures(self, context, obj_handle, high_res_path):
        if obj_handle.type == 'MESH':
            # First save the texture(s) if any
            # Store current render settings
            _settings = context.scene.render.image_settings

            # Change render settings to our target format
            _settings.file_format = 'PNG'

            if context.scene.ambf_save_textures:
                for mat in obj_handle.data.materials:
                    if mat.node_tree:
                        for node in mat.node_tree.nodes:
                            if node.type == 'TEX_IMAGE':
                                im = node.image
                                _filename = im.name_full
                                _filename_wo_ext = _filename.split('.')[0]
                                print("Texture Filename ", _filename)
                                _save_as = os.path.join(high_res_path, _filename_wo_ext + '.png')
                                im.filepath_raw = _save_as
                                im.save_render(_save_as)

    def save_body_meshes(self, context, obj_handle, mesh_type, high_res_path, low_res_path):
        if not obj_handle.ambf_object_type == 'RIGID_BODY':
            # Only Save Meshes if the object type is ambf rigid body
            return

        select_object(obj_handle)
        obj_handle_name = remove_namespace_prefix(obj_handle.name)

        if obj_handle.type == 'MESH':
            # First save the texture(s) if any
            # Store current render settings
            _settings = context.scene.render.image_settings

            # Change render settings to our target format
            _settings.file_format = 'PNG'

            if context.scene.ambf_save_textures:
                for mat in obj_handle.data.materials:
                    if mat.node_tree:
                        for node in mat.node_tree.nodes:
                            if node.type == 'TEX_IMAGE':
                                im = node.image
                                _filename = im.name_full
                                _filename_wo_ext = _filename.split('.')[0]
                                print("Texture Filename ", _filename)
                                _save_as = os.path.join(high_res_path, _filename_wo_ext + '.png')
                                im.filepath_raw = _save_as
                                im.save_render(_save_as)

            if mesh_type == 'STL':
                obj_name = obj_handle_name + '.STL'
                filename_high_res = os.path.join(high_res_path, obj_name)
                filename_low_res = os.path.join(low_res_path, obj_name)
                if context.scene.ambf_save_high_res:
                    bpy.ops.export_mesh.stl(filepath=filename_high_res, use_selection=True, use_mesh_modifiers=False)
                if context.scene.ambf_save_low_res:
                    bpy.ops.export_mesh.stl(filepath=filename_low_res, use_selection=True, use_mesh_modifiers=True)
            elif mesh_type == 'OBJ':
                obj_name = obj_handle_name + '.OBJ'
                filename_high_res = os.path.join(high_res_path, obj_name)
                filename_low_res = os.path.join(low_res_path, obj_name)
                if context.scene.ambf_save_high_res:
                    bpy.ops.export_scene.obj(filepath=filename_high_res, axis_up='Z', axis_forward='Y',
                                             use_selection=True, use_mesh_modifiers=False)
                if context.scene.ambf_save_low_res:
                    bpy.ops.export_scene.obj(filepath=filename_low_res, axis_up='Z', axis_forward='Y',
                                             use_selection=True, use_mesh_modifiers=True)
            elif mesh_type == '3DS':
                obj_name = obj_handle_name + '.3DS'
                filename_high_res = os.path.join(high_res_path, obj_name)
                filename_low_res = os.path.join(low_res_path, obj_name)
                # 3DS doesn't support supressing modifiers, so we explicitly
                # toggle them to save as high res and low res meshes
                # STILL BUGGY
                for mod in obj_handle.modifiers:
                    mod.show_viewport = True
                if context.scene.ambf_save_high_res:
                    bpy.ops.export_scene.autodesk_3ds(filepath=filename_low_res, use_selection=True)

                for mod in obj_handle.modifiers:
                    mod.show_viewport = True
                if context.scene.ambf_save_low_res:
                    bpy.ops.export_scene.autodesk_3ds(filepath=filename_high_res, use_selection=True)
            elif mesh_type == 'PLY':
                # .PLY export has a bug in which it only saves the mesh that is
                # active in context of view. Hence we explicitly select this object
                # as active in the scene on top of being selected
                obj_name = obj_handle_name + '.PLY'
                filename_high_res = os.path.join(high_res_path, obj_name)
                filename_low_res = os.path.join(low_res_path, obj_name)
                set_active_object(obj_handle)

                if context.scene.ambf_save_high_res:
                    bpy.ops.export_mesh.ply(filepath=filename_high_res, use_mesh_modifiers=False)
                if context.scene.ambf_save_low_res:
                    bpy.ops.export_mesh.ply(filepath=filename_low_res, use_mesh_modifiers=True)
                # Make sure to deselect the mesh
                set_active_object(None)
            else:
                raise Exception('Mesh Format Not Specified/Understood')

        select_object(obj_handle, False)

    def save_meshes(self, context):
        # Get the list of currently selected objects
        selected_objects = get_selected_objects()

        # Now deselect all objects
        select_all_objects(False)

        save_path = bpy.path.abspath(context.scene.ambf_meshes_path)
        high_res_path = os.path.join(save_path, 'high_res/')
        low_res_path = os.path.join(save_path, 'low_res/')
        os.makedirs(high_res_path, exist_ok=True)
        os.makedirs(low_res_path, exist_ok=True)
        mesh_type = bpy.context.scene.ambf_meshes_save_type

        mesh_name_mat_list = self.set_all_meshes_to_origin()
        if context.scene.ambf_save_selection_only:
            for obj_handle in selected_objects:
                self.save_body_textures(context, obj_handle, high_res_path)
                self.save_body_meshes(context, obj_handle, mesh_type, high_res_path, low_res_path)
        else:
            for obj_handle in bpy.data.objects:
                self.save_body_textures(context, obj_handle, high_res_path)
                self.save_body_meshes(context, obj_handle, mesh_type, high_res_path, low_res_path)
        self.reset_meshes_to_original_position(mesh_name_mat_list)

        # Now reselect the objects that we selected prior to saving meshes
        select_objects(selected_objects, True)


class AMBF_OT_generate_low_res_mesh_modifiers(Operator):
    bl_idname = "ambf.generate_low_res_mesh_modifiers"
    bl_label = "Generate Low-Res Meshes"
    bl_description = "This creates the low-res modifiers for higher speed collision computation" \
                     " . For now, the mesh decimation modifiers are being used but they shall be" \
                     " replaced with other methods"

    def execute(self, context):
        # First off, remove any existing Modifiers:
        bpy.ops.ambf.remove_low_res_mesh_modifiers()

        # Now deselect all objects
        for obj_handle in bpy.data.objects:
            select_object(obj_handle, False)

        vertices_max = context.scene.ambf_mesh_max_vertices
        # Select each obj_handle iteratively and generate its low-res mesh
        for obj_handle in bpy.data.objects:
            if obj_handle.type == 'MESH' and is_object_hidden(obj_handle) is False:
                decimate_mod = obj_handle.modifiers.new('decimate_mod', 'DECIMATE')
                if len(obj_handle.data.vertices) > vertices_max:
                    reduction_ratio = vertices_max / len(obj_handle.data.vertices)
                    decimate_mod.use_symmetry = False
                    decimate_mod.use_collapse_triangulate = True
                    decimate_mod.ratio = reduction_ratio
                    decimate_mod.show_viewport = True
        return {'FINISHED'}


class AMBF_OT_create_joint(Operator):
    bl_idname = "ambf.create_joint"
    bl_label = "Create Joint"
    bl_description = "This creates an empty object that can be used to create closed loop mechanisms. Make" \
                     " sure to set the rigid body constraint (RBC) for this empty mesh and ideally parent this empty" \
                     " object with the parent body of its RBC"

    def execute(self, context):
        select_all_objects(False)
        bpy.ops.object.empty_add(type='PLAIN_AXES')
        active_obj_handle = get_active_object()
        active_obj_handle.name = 'joint'
        return {'FINISHED'}


class AMBF_OT_remove_low_res_mesh_modifiers(Operator):
    bl_idname = "ambf.remove_low_res_mesh_modifiers"
    bl_label = "Remove All Modifiers"
    bl_description = "This removes all the mesh modifiers generated for meshes in the current scene"

    def execute(self, context):
        for obj_handle in bpy.data.objects:
            for mod in obj_handle.modifiers:
                obj_handle.modifiers.remove(mod)
        return {'FINISHED'}


class AMBF_OT_toggle_low_res_mesh_modifiers_visibility(Operator):
    bl_idname = "ambf.toggle_low_res_mesh_modifiers_visibility"
    bl_label = "Toggle Modifiers Visibility"
    bl_description = "This hides all the mesh modifiers generated for meshes in the current scene"

    def execute(self, context):
        for obj_handle in bpy.data.objects:
            for mod in obj_handle.modifiers:
                mod.show_viewport = not mod.show_viewport
        return {'FINISHED'}


class AMBF_OT_estimate_inertial_offsets(Operator):
    bl_idname = "ambf.estimate_inertial_offsets"
    bl_label = "Estimate Inertial Offsets"
    bl_description = "Automatically Estimate the Inertial Offsets for the Bodies"

    def execute(self, context):
        for obj_handle in bpy.data.objects:
            if obj_handle.ambf_object_type == 'RIGID_BODY' and obj_handle.type == 'MESH':
                local_com = compute_local_com(obj_handle)
                obj_handle.ambf_rigid_body_linear_inertial_offset[0] = local_com[0]
                obj_handle.ambf_rigid_body_linear_inertial_offset[1] = local_com[1]
                obj_handle.ambf_rigid_body_linear_inertial_offset[2] = local_com[2]
        return {'FINISHED'}


class AMBF_OT_estimate_shape_offsets(Operator):
    bl_idname = "ambf.estimate_shape_offsets"
    bl_label = "Estimate Shape Offsets"
    bl_description = "Automatically Estimate the Shape Offsets for the Bodies (ONLY FOR SINGULAR SHAPES)"

    def execute(self, context):
        cur_active_obj = get_active_object()
        for obj_handle in bpy.data.objects:
            if obj_handle.ambf_object_type == 'RIGID_BODY' and obj_handle.type == 'MESH':
                if obj_handle.ambf_collision_type == 'SINGULAR_SHAPE':
                    set_active_object(obj_handle)
                    prop_group = obj_handle.ambf_collision_shape_prop_collection.items()[0][1]
                    local_com = compute_local_com(obj_handle)
                    prop_group.ambf_collision_shape_linear_offset[0] = local_com[0]
                    prop_group.ambf_collision_shape_linear_offset[1] = local_com[1]
                    prop_group.ambf_collision_shape_linear_offset[2] = local_com[2]
        set_active_object(cur_active_obj)
        return {'FINISHED'}


class AMBF_OT_estimate_collision_shapes_geometry(Operator):
    bl_idname = "ambf.estimate_collision_shapes_geometry"
    bl_label = "Estimate Collision Shapes Geometry"
    bl_description = "Estimate Collision Shapes Geometry"

    def execute(self, context):
        for obj_handle in bpy.data.objects:
            estimate_collision_shape_geometry(obj_handle)
        return {'FINISHED'}


class AMBF_OT_estimate_inertias(Operator):
    bl_idname = "ambf.estimate_inertias"
    bl_label = "Estimate Body Inertias"
    bl_description = "Estimate Body Inertias"

    def execute(self, context):
        for obj_handle in bpy.data.objects:
            if obj_handle.ambf_object_type == 'RIGID_BODY':
                if not obj_handle.ambf_rigid_body_is_static:
                    I = calculate_principal_inertia(obj_handle)
                    obj_handle.ambf_rigid_body_inertia_x = I[0]
                    obj_handle.ambf_rigid_body_inertia_y = I[1]
                    obj_handle.ambf_rigid_body_inertia_z = I[2]
                    obj_handle.ambf_rigid_body_specify_inertia = True
        return {'FINISHED'}


class AMBF_OT_estimate_joint_controller_gains(Operator):
    bl_idname = "ambf.estimate_joint_controller_gains"
    bl_label = "Estimate Joint Controller Gains"
    bl_description = "Estimate Joint Controller Gains"

    def execute(self, context):
        for obj_handle in bpy.data.objects:
                estimate_joint_controller_gain(obj_handle)
                if obj_handle.ambf_object_type == 'CONSTRAINT':
                    obj_handle.ambf_constraint_enable_controller_gains = True
        return {'FINISHED'}


class AMBF_OT_auto_rename_joints(Operator):
    bl_idname = "ambf.auto_rename_joints"
    bl_label = "Automatically Rename Joints"
    bl_description = "Automatically Rename Joints as Parent-Child name"

    def execute(self, context):
        for obj_handle in bpy.data.objects:
            if obj_handle.ambf_object_type == 'CONSTRAINT':
                parent = obj_handle.ambf_object_parent
                child = obj_handle.ambf_object_child
                if parent and child:
                    obj_handle.ambf_constraint_name = remove_namespace_prefix(parent.name) + '-' + remove_namespace_prefix(child.name)
                pass
        return {'FINISHED'}


class AMBF_OT_estimate_inertial_offset_per_object(Operator):
    bl_idname = "ambf.estimate_inertial_offset_per_object"
    bl_label = "Estimate Inertial Offset"
    bl_description = "Automatically Estimate the Inertial Offsets for the Bodies"

    def execute(self, context):
        obj_handle = context.object
        if obj_handle.ambf_object_type == 'RIGID_BODY' and obj_handle.type == 'MESH':
            local_com = compute_local_com(obj_handle)
            obj_handle.ambf_rigid_body_linear_inertial_offset[0] = local_com[0]
            obj_handle.ambf_rigid_body_linear_inertial_offset[1] = local_com[1]
            obj_handle.ambf_rigid_body_linear_inertial_offset[2] = local_com[2]
            pass
        return {'FINISHED'}


class AMBF_OT_estimate_shape_offset_per_object(Operator):
    bl_idname = "ambf.estimate_shape_offset_per_object"
    bl_label = "Estimate Shape Offset Per Object"
    bl_description = "Automatically Estimate the Shape Offset for the Body (SINGULAR SHAPE ONLY)"

    def execute(self, context):
        obj_handle = context.object
        if obj_handle.ambf_object_type == 'RIGID_BODY' and obj_handle.type == 'MESH':
            if obj_handle.ambf_collision_type == 'SINGULAR_SHAPE':
                prop_group = obj_handle.ambf_collision_shape_prop_collection.items()[0][1]
                local_com = compute_local_com(obj_handle)
                prop_group.ambf_collision_shape_linear_offset[0] = local_com[0]
                prop_group.ambf_collision_shape_linear_offset[1] = local_com[1]
                prop_group.ambf_collision_shape_linear_offset[2] = local_com[2]
        return {'FINISHED'}


class AMBF_OT_estimate_collision_shape_geometry_per_object(Operator):
    bl_idname = "ambf.estimate_collision_shape_geometry_per_object"
    bl_label = "Estimate Collision Shape Geometry"
    bl_description = "Estimate Collision Shape Geometry"

    def execute(self, context):
        estimate_collision_shape_geometry(context.object)
        return {'FINISHED'}


class AMBF_OT_estimate_inertia_per_object(Operator):
    bl_idname = "ambf.estimate_inertia_per_object"
    bl_label = "Estimate Body Inertia"
    bl_description = "Estimate Body Inertia"

    def execute(self, context):
        obj_handle = context.object
        if obj_handle.ambf_object_type == 'RIGID_BODY':
            if not obj_handle.ambf_rigid_body_is_static:
                I = calculate_principal_inertia(obj_handle)
                obj_handle.ambf_rigid_body_inertia_x = I[0]
                obj_handle.ambf_rigid_body_inertia_y = I[1]
                obj_handle.ambf_rigid_body_inertia_z = I[2]
                obj_handle.ambf_rigid_body_specify_inertia = True
        return {'FINISHED'}


class AMBF_OT_estimate_joint_controller_gain_per_object(Operator):
    bl_idname = "ambf.estimate_joint_controller_gain_per_object"
    bl_label = "Estimate Joint Controller Gains Per Object"
    bl_description = "Estimate Joint Controller Gains"

    def execute(self, context):
        obj_handle = context.object
        estimate_joint_controller_gain(obj_handle)
        return {'FINISHED'}


class AMBF_OT_auto_rename_joint_per_object(Operator):
    bl_idname = "ambf.auto_rename_joint_per_object"
    bl_label = "Automatically Rename Joint"
    bl_description = "Automatically Rename Joint as Parent-Child name"

    def execute(self, context):
        obj_handle = context.object
        if obj_handle.ambf_object_type == 'CONSTRAINT':
            parent = obj_handle.ambf_object_parent
            child = obj_handle.ambf_object_child
            if parent and child:
                obj_handle.ambf_constraint_name = remove_namespace_prefix(parent.name) + '-' + remove_namespace_prefix(child.name)
            pass
        return {'FINISHED'}


class AMBF_OT_remove_object_namespaces(Operator):
    bl_idname = "ambf.remove_object_namespaces"
    bl_label = "Remove Object Namespaces"
    bl_description = "This removes any current object namespaces"

    def execute(self, context):
        for obj_handle in bpy.data.objects:
            obj_handle.name = obj_handle.name.split('/')[-1]
        return {'FINISHED'}


class AMBF_OT_load_ambf_file(Operator):
    bl_idname = "ambf.load_ambf_file"
    bl_label = "Load AMBF Description File (ADF)"
    bl_description = "This loads an AMBF from the specified config file"

    def __init__(self):
        self._adf_data = None
        self._joint_additional_offset = {}
        # A dict for body name as defined in YAML File and the Name Blender gives
        # the body
        self._blender_remapped_body_names = {}
        self._high_res_path = ''
        self._low_res_path = ''
        self._context = None
        self._yaml_filepath = ''

    def get_qualified_path(self, path):
        filepath = Path(path)

        if filepath.is_absolute():
            return path
        else:
            ambf_filepath = Path(self._yaml_filepath)
            path = str(ambf_filepath.parent.joinpath(filepath))
            return path

    def load_mesh(self, body_data, body_name):

        af_name = body_data['name']

        if 'high resolution path' in body_data:
            body_high_res_path = self.get_qualified_path(body_data['high resolution path'])
        else:
            body_high_res_path = self._high_res_path
        # If body name is world. Check if a world body has already
        # been defined, and if it has been, ignore adding another world body
        if af_name in ['world', 'World', 'WORLD']:
            for temp_obj_handle in bpy.data.objects:
                if temp_obj_handle.type in ['MESH', 'EMPTY']:
                    if temp_obj_handle.name in ['world', 'World', 'WORLD']:
                        self._blender_remapped_body_names[body_name] = temp_obj_handle.name
                        return
        body_mesh_name = body_data['mesh']

        mesh_filepath = Path(os.path.join(body_high_res_path, body_mesh_name))

        if mesh_filepath.suffix in ['.stl', '.STL']:
            bpy.ops.import_mesh.stl(filepath=str(mesh_filepath.resolve()))

        elif mesh_filepath.suffix in ['.obj', '.OBJ']:
            _manually_select_obj_handle = True
            bpy.ops.import_scene.obj(filepath=str(mesh_filepath.resolve()), axis_up='Z', axis_forward='Y')
            # Hack, .3ds and .obj imports do not make the imported obj_handle active. A hack is
            # to capture the selected objects in this case.
            set_active_object(self._context.selected_objects[0])

        elif mesh_filepath.suffix in ['.dae', '.DAE']:
            bpy.ops.wm.collada_import(filepath=str(mesh_filepath.resolve()))
            # If we are importing .dae meshes, they can import stuff other than meshes, such as cameras etc.
            # We should remove these extra things and only keep the meshes
            for temp_obj_handle in self._context.selected_objects:
                if temp_obj_handle.type == 'MESH':
                    obj_handle = temp_obj_handle
                    # set_active_object(obj_handle)
                else:
                    bpy.data.objects.remove(temp_obj_handle)

            so = bpy.context.selected_objects
            if len(so) > 1:
                set_active_object(so[0])
                bpy.ops.object.join()
                so[0].name = af_name
                obj_handle = get_active_object()

                # The lines below are essential in joint the multiple meshes
                # defined in the .dae into one mesh, secondly, making sure that
                # the origin of the mesh is what it is supposed to be as
                # using the join() function call alters the mesh origin
                trans_o = obj_handle.matrix_world.copy()
                obj_handle.matrix_world.identity()
                obj_handle.data.transform(trans_o)

                # Kind of a hack, blender is spawning the collada file
                # a 90 deg offset along the axis axis, this is to correct that
                # Maybe this will not be needed in future versions of blender
                r_x = mathutils.Matrix.Rotation(-pi/2, 4, 'X')
                obj_handle.data.transform(r_x)
            else:
                set_active_object(so[0])

        elif mesh_filepath.suffix in ['.3ds', '.3DS']:
            _manually_select_obj_handle = True
            bpy.ops.import_scene.autodesk_3ds(filepath=str(mesh_filepath.resolve()))
            # Hack, .3ds and .obj imports do not make the imported obj_handle active. A hack is
            # to capture the selected objects in this case.
            set_active_object(self._context.selected_objects[0])

        elif mesh_filepath.suffix == '':
            bpy.ops.object.empty_add(type='PLAIN_AXES')

        return get_active_object()

    def load_material(self, body_data, obj_handle):

        af_name = body_data['name']

        if 'color rgba' in body_data:
            mat = bpy.data.materials.new(name=af_name + 'mat')
            mat.diffuse_color[0] = body_data['color rgba']['r']
            mat.diffuse_color[1] = body_data['color rgba']['g']
            mat.diffuse_color[2] = body_data['color rgba']['b']
            mat.diffuse_color[3] = body_data['color rgba']['a']
            obj_handle.data.materials.append(mat)

        elif 'color components' in body_data:
            mat = bpy.data.materials.new(name=af_name + 'mat')
            mat.diffuse_color[0] = body_data['color components']['diffuse']['r']
            mat.diffuse_color[1] = body_data['color components']['diffuse']['g']
            mat.diffuse_color[2] = body_data['color components']['diffuse']['b']
            mat.diffuse_color[3] = body_data['color components']['transparency']

            # In Blender 2.8, specular is a float unlike 2.79 where it was an RGB
            intensity = 0
            try:
                intensity = body_data['color components']['specular']['r'] / mat.diffuse_color[0]
            except:
                try:
                    intensity = body_data['color components']['specular']['r'] / mat.diffuse_color[1]
                except:
                    try:
                        intensity = body_data['color components']['specular']['r'] / mat.diffuse_color[2]
                    except:
                        intensity = 0.0
                        
            mat.specular_intensity = intensity

#            mat.ambient = body_data['color components']['ambient']['level']
#            mat.use_transparency = True
#            mat.transparency_method = 'Z_TRANSPARENCY'
#            mat.alpha = body_data['color components']['transparency']
            obj_handle.data.materials.append(mat)

    def load_ambf_rigid_body(self, body_data, obj_handle):
        if obj_handle.type in ['EMPTY', 'MESH']:
            obj_handle.ambf_rigid_body_mass = body_data['mass']
            obj_handle.ambf_object_type = 'RIGID_BODY'

            if 'visible' in body_data:
                obj_handle.ambf_object_visible = body_data['visible']

            if 'inertia' in body_data:
                obj_handle.ambf_rigid_body_specify_inertia = True
                obj_handle.ambf_rigid_body_inertia_x = body_data['inertia']['ix']
                obj_handle.ambf_rigid_body_inertia_y = body_data['inertia']['iy']
                obj_handle.ambf_rigid_body_inertia_z = body_data['inertia']['iz']

            if body_data['mass'] == 0.0:
                obj_handle.ambf_rigid_body_is_static = True

            if 'inertial offset' in body_data:
                obj_handle.ambf_rigid_body_linear_inertial_offset[0] = body_data['inertial offset']['position']['x']
                obj_handle.ambf_rigid_body_linear_inertial_offset[1] = body_data['inertial offset']['position']['y']
                obj_handle.ambf_rigid_body_linear_inertial_offset[2] = body_data['inertial offset']['position']['z']

                obj_handle.ambf_rigid_body_angular_inertial_offset[0] = body_data['inertial offset']['orientation']['r']
                obj_handle.ambf_rigid_body_angular_inertial_offset[1] = body_data['inertial offset']['orientation']['p']
                obj_handle.ambf_rigid_body_angular_inertial_offset[2] = body_data['inertial offset']['orientation']['y']

            # Finally add the rigid body data if defined
            if 'friction' in body_data:
                if 'static' in body_data['friction']:
                    obj_handle.ambf_rigid_body_static_friction = body_data['friction']['static']
                if 'rolling' in body_data['friction']:
                    obj_handle.ambf_rigid_body_rolling_friction = body_data['friction']['rolling']

            if 'damping' in body_data:
                if 'linear' in body_data['damping']:
                    obj_handle.ambf_rigid_body_linear_damping = body_data['damping']['linear']
                if 'angular' in body_data['damping']:
                    obj_handle.ambf_rigid_body_angular_damping = body_data['damping']['angular']

            if 'restitution' in body_data:
                obj_handle.ambf_rigid_body_restitution = body_data['restitution']

            if 'collision margin' in body_data:
                obj_handle.ambf_collision_margin = body_data['collision margin']
                obj_handle.ambf_collision_margin_enable = True

            if 'collision shape' in body_data:
                obj_handle.ambf_collision_shape_prop_collection.add()
                ocs = obj_handle.ambf_collision_shape_prop_collection.items()[0][1]
                ocs.ambf_collision_shape = body_data['collision shape']
                if ocs.ambf_collision_shape == 'BOX':
                    ocs.ambf_collision_shape_xyz_dims[0] = body_data['collision geometry']['x']
                    ocs.ambf_collision_shape_xyz_dims[1] = body_data['collision geometry']['y']
                    ocs.ambf_collision_shape_xyz_dims[2] = body_data['collision geometry']['z']
                elif ocs.ambf_collision_shape == 'SPHERE':
                    ocs.ambf_collision_shape_radius = body_data['collision geometry']['radius']
                elif ocs.ambf_collision_shape in ['CYLINDER', 'CONE', 'CAPSULE']:
                    ocs.ambf_collision_shape_radius = body_data['collision geometry']['radius']
                    ocs.ambf_collision_shape_height = body_data['collision geometry']['height']
                    ocs.ambf_collision_shape_axis = str.upper(body_data['collision geometry']['axis'])

                if 'collision offset' in body_data:
                    cso = body_data['collision offset']
                    ocs.ambf_collision_shape_linear_offset[0] = cso['position']['x']
                    ocs.ambf_collision_shape_linear_offset[1] = cso['position']['y']
                    ocs.ambf_collision_shape_linear_offset[2] = cso['position']['z']
                    ocs.ambf_collision_shape_angular_offset[0] = cso['orientation']['r']
                    ocs.ambf_collision_shape_angular_offset[1] = cso['orientation']['p']
                    ocs.ambf_collision_shape_angular_offset[2] = cso['orientation']['y']
                else:
                    # This is for legacy ADF, if the shape offset is
                    # not defined set the shape offset equal to the inertial offset
                    ocs.ambf_collision_shape_linear_offset[0] = obj_handle.ambf_rigid_body_linear_inertial_offset[0]
                    ocs.ambf_collision_shape_linear_offset[1] = obj_handle.ambf_rigid_body_linear_inertial_offset[1]
                    ocs.ambf_collision_shape_linear_offset[2] = obj_handle.ambf_rigid_body_linear_inertial_offset[2]

                    ocs.ambf_collision_shape_angular_offset[0] = obj_handle.ambf_rigid_body_angular_inertial_offset[0]
                    ocs.ambf_collision_shape_angular_offset[1] = obj_handle.ambf_rigid_body_angular_inertial_offset[1]
                    ocs.ambf_collision_shape_angular_offset[2] = obj_handle.ambf_rigid_body_angular_inertial_offset[2]
                
                obj_handle.ambf_collision_type = 'SINGULAR_SHAPE'
            elif 'compound collision shape' in body_data:
                shape_count = 0
                for shape_item in body_data['compound collision shape']:
                    obj_handle.ambf_collision_shape_prop_collection.add()
                    ocs = obj_handle.ambf_collision_shape_prop_collection.items()[shape_count][1]
                    ocs.ambf_collision_shape = shape_item['shape']
                    shape_count = shape_count + 1
                    if ocs.ambf_collision_shape == 'BOX':
                        ocs.ambf_collision_shape_xyz_dims[0] = shape_item['geometry']['x']
                        ocs.ambf_collision_shape_xyz_dims[1] = shape_item['geometry']['y']
                        ocs.ambf_collision_shape_xyz_dims[2] = shape_item['geometry']['z']
                    elif ocs.ambf_collision_shape == 'SPHERE':
                        ocs.ambf_collision_shape_radius = shape_item['geometry']['radius']
                    elif ocs.ambf_collision_shape in ['CYLINDER', 'CONE', 'CAPSULE']:
                        ocs.ambf_collision_shape_radius = shape_item['geometry']['radius']
                        ocs.ambf_collision_shape_height = shape_item['geometry']['height']
                        ocs.ambf_collision_shape_axis = str.upper(shape_item['geometry']['axis'])

                    ocs.ambf_collision_shape_linear_offset[0] = shape_item['offset']['position']['x']
                    ocs.ambf_collision_shape_linear_offset[1] = shape_item['offset']['position']['y']
                    ocs.ambf_collision_shape_linear_offset[2] = shape_item['offset']['position']['z']

                    ocs.ambf_collision_shape_angular_offset[0] = shape_item['offset']['orientation']['r']
                    ocs.ambf_collision_shape_angular_offset[1] = shape_item['offset']['orientation']['p']
                    ocs.ambf_collision_shape_angular_offset[2] = shape_item['offset']['orientation']['y']
                    
                obj_handle.ambf_collision_type = 'COMPOUND_SHAPE'
            else:
                # Since the shape is neither a single or a compound shape, it is a mesh based collision.
                # Now figure out what type of collision mesh is used. (CONCAVE_MESH, CONVEX_MESH or CONVEX_HULL)
                if 'collision mesh type' in body_data:
                    obj_handle.ambf_collision_mesh_type = body_data['collision mesh type']
                else:
                    # For backward compatibility, the default collision mesh type used to be CONCAVE_MESH
                    obj_handle.ambf_collision_mesh_type = 'CONCAVE_MESH'

            if 'collision groups' in body_data:
                col_groups = body_data['collision groups']
                # First clear existing collision group of 0
                obj_handle.ambf_collision_groups[0] = False
                for group in col_groups:
                    if 0 <= group < 20:
                        obj_handle.ambf_collision_groups[group] = True
                    else:
                        print('WARNING, Collision Group Outside [0-20]')
                        
            if 'passive' in body_data:
                obj_handle.ambf_rigid_body_passive = body_data['passive']

            # If Body Controller Defined. Set the P and D gains for linera and angular controller prop fields
            if 'controller' in body_data:
                obj_handle.ambf_rigid_body_linear_controller_p_gain = body_data['controller']['linear']['P']
                if 'I' in body_data['controller']['linear']:
                    obj_handle.ambf_rigid_body_linear_controller_i_gain = body_data['controller']['linear']['I']
                obj_handle.ambf_rigid_body_linear_controller_d_gain = body_data['controller']['linear']['D']
                obj_handle.ambf_rigid_body_angular_controller_p_gain = body_data['controller']['angular']['P']
                if 'I' in body_data['controller']['angular']:
                    obj_handle.ambf_rigid_body_angular_controller_i_gain = body_data['controller']['angular']['I']
                obj_handle.ambf_rigid_body_angular_controller_d_gain = body_data['controller']['angular']['D']
                obj_handle.ambf_rigid_body_enable_controllers = True

                if 'controller output type' in body_data:
                    obj_handle.ambf_rigid_body_controller_output_type = body_data['controller output type']
                else:
                    obj_handle.ambf_rigid_body_controller_output_type = 'FORCE'

            # Now lets add a collision shape for each collision_property_group
            for prop_tuple in obj_handle.ambf_collision_shape_prop_collection.items():
                shape_prop_group = prop_tuple[1]
                collision_shape_create_visual(obj_handle, shape_prop_group)

    def load_object_name(self, adf_data, obj_handle):

        af_name = adf_data['name']

        if 'namespace' in adf_data:
            _body_namespace = adf_data['namespace']
            obj_handle.name = _body_namespace + af_name
        else:
            obj_handle.name = add_namespace_prefix(af_name)

    def load_body_location(self, body_data, obj_handle):

        bpy.ops.object.transform_apply(scale=True)

        body_location_xyz = {'x': 0, 'y': 0, 'z': 0}
        body_location_rpy = {'r': 0, 'p': 0, 'y': 0}

        if 'location' in body_data:
            if 'position' in body_data['location']:
                body_location_xyz = body_data['location']['position']
            if 'orientation' in body_data['location']:
                body_location_rpy = body_data['location']['orientation']

        obj_handle.matrix_world.translation[0] = body_location_xyz['x']
        obj_handle.matrix_world.translation[1] = body_location_xyz['y']
        obj_handle.matrix_world.translation[2] = body_location_xyz['z']
        obj_handle.rotation_euler = (body_location_rpy['r'],
                                     body_location_rpy['p'],
                                     body_location_rpy['y'])

    def load_body(self, body_name):
        body_data = self._adf_data[body_name]

        obj_handle = self.load_mesh(body_data, body_name)

        self.load_object_name(body_data, obj_handle)

        self.load_ambf_rigid_body(body_data, obj_handle)

        self.load_body_location(body_data, obj_handle)

        self.load_material(body_data, obj_handle)

        self._blender_remapped_body_names[body_name] = obj_handle.name
        CommonConfig.loaded_body_map[obj_handle] = body_data

    def load_ghost(self, ghost_name):
        ghost_data = self._adf_data[ghost_name]

        obj_handle = self.load_mesh(ghost_data, ghost_name)

        self.load_object_name(ghost_data, obj_handle)

        self.load_ambf_rigid_body(ghost_data, obj_handle)

        self.load_body_location(ghost_data, obj_handle)

        self.load_material(ghost_data, obj_handle)

        self._blender_remapped_body_names[ghost_name] = obj_handle.name
        CommonConfig.loaded_body_map[obj_handle] = ghost_data

    def get_ambf_joint_type(self, joint_data):
        joint_type = 'FIXED'
        if 'type' in joint_data:
            if joint_data['type'] in ['hinge', 'revolute', 'continuous']:
                joint_type = 'REVOLUTE'
            elif joint_data['type'] in ['prismatic', 'slider']:
                joint_type = 'PRISMATIC'
            elif joint_data['type'] in ['spring', 'linear spring']:
                joint_type = 'LINEAR_SPRING'
            elif joint_data['type'] in ['angular spring', 'torsional spring', 'torsion spring']:
                joint_type = 'TORSION_SPRING'
            elif joint_data['type'] in ['p2p', 'point2point']:
                joint_type = 'P2P'
            elif joint_data['type'] in ['fixed', 'FIXED']:
                joint_type = 'FIXED'
            elif joint_data['type'] in ['cone twist']:
                joint_type = 'CONE_TWIST'
            elif joint_data['type'] in ['six dof']:
                joint_type = 'SIX_DOF'
            elif joint_data['type'] in ['six dof spring']:
                joint_type = 'SIX_DOF_SPRING'

        return joint_type
    
    def set_default_ambf_constraint_axis(self, joint_obj_handle):
        if joint_obj_handle.ambf_object_type == 'CONSTRAINT':
            if joint_obj_handle.ambf_constraint_type in ['REVOLUTE', 'TORSION_SPRING', 'CONE_TWIST', 'SIX_DOF', 'SIX_DOF_SPRING']:
                joint_obj_handle.ambf_constraint_axis = 'Z'
            elif joint_obj_handle.ambf_constraint_type in ['PRISMATIC', 'LINEAR_SPRING']:
                joint_obj_handle.ambf_constraint_axis = 'X'

    def get_parent_and_child_object_handles(self, joint_data):
        parent_body_name = joint_data['parent']
        child_body_name = joint_data['child']

        parent_obj_handle = bpy.data.objects[self._blender_remapped_body_names[parent_body_name]]
        child_obj_handle = bpy.data.objects[self._blender_remapped_body_names[child_body_name]]

        return parent_obj_handle, child_obj_handle

    def get_parent_pivot_and_axis_data(self, joint_data):
        parent_pivot_data = {'x': 0, 'y': 0, 'z': 0}
        parent_axis_data = {'x': 0, 'y': 0, 'z': 1}

        if 'parent pivot' in joint_data:
            parent_pivot_data = joint_data['parent pivot']
        if 'parent axis' in joint_data:
            parent_axis_data = joint_data['parent axis']

        return parent_pivot_data, parent_axis_data

    def get_child_pivot_and_axis_data(self, joint_data):
        child_pivot_data = {'x': 0, 'y': 0, 'z': 0}
        child_axis_data = {'x': 0, 'y': 0, 'z': 1}

        if 'child pivot' in joint_data:
            child_pivot_data = joint_data['child pivot']
        if 'child axis' in joint_data:
            child_axis_data = joint_data['child axis']

        return child_pivot_data, child_axis_data

    def get_standard_pivot_and_axis_data(self, joint_data):
        pivot_data = {'x': 0, 'y': 0, 'z': 0}
        axis_data = {'x': 0, 'y': 0, 'z': 1}
        if joint_data['type'] in ['hinge', 'continuous', 'revolute', 'fixed']:
            axis_data = {'x': 0, 'y': 0, 'z': 1}
        elif joint_data['type'] in ['prismatic', 'slider']:
            axis_data = {'x': 1, 'y': 0, 'z': 0}
        elif joint_data['type'] in ['spring', 'linear spring']:
            axis_data = {'x': 1, 'y': 0, 'z': 0}
        elif joint_data['type'] in ['angular spring', 'torsional spring', 'torsion spring']:
            axis_data = {'x': 0, 'y': 0, 'z': 1}
        elif joint_data['type'] in ['p2p', 'point2point']:
            axis_data = {'x': 0, 'y': 0, 'z': 1}
        elif joint_data['type'] in ['cone twist', 'six dof', 'six dof spring']:
            axis_data = {'x': 0, 'y': 0, 'z': 1}
        else:
            print('ERROR, (', sys._getframe().f_code.co_name, ') ( Joint Type', joint_data['type'], 'Not Understood')

        return pivot_data, axis_data

    def get_joint_offset_angle(self, joint_data):
        # To fully define a child body's connection and pose in a parent body, just the joint pivots
        # and joint axis are not sufficient. We also need the joint offset which correctly defines
        # the initial pose of the child body in the parent body.
        parent_offset_angle = 0.0
        if 'joint offset' in joint_data:
            parent_offset_angle = joint_data['joint offset']

        return parent_offset_angle

    def get_child_offset_angle(self, joint_data):
        # To fully define a child body's connection and pose in a parent body, just the joint pivots
        # and joint axis are not sufficient. We also need the joint offset which correctly defines
        # the initial pose of the child body in the parent body.
        child_offset_angle = 0.0
        if 'offset' in joint_data:
            child_offset_angle = joint_data['offset']
        elif 'child offset' in joint_data:
            child_offset_angle = joint_data['child offset']

        return child_offset_angle

    def get_joint_in_world_transform(self, joint_data):
        parent_obj_handle, child_obj_handle = self.get_parent_and_child_object_handles(joint_data)
        parent_pivot_data, parent_axis_data = self.get_parent_pivot_and_axis_data(joint_data)
        standard_pivot_data, standard_axis_data = self.get_standard_pivot_and_axis_data(joint_data)

        # Transformation matrix representing parent in world frame
        T_p_w = parent_obj_handle.matrix_world.copy()
        # Parent's Joint Axis in parent's frame
        parent_axis = mathutils.Vector([parent_axis_data['x'],
                                        parent_axis_data['y'],
                                        parent_axis_data['z']])
        # Transformation of joint in parent frame
        P_j_p = mathutils.Matrix()
        P_j_p.translation = mathutils.Vector([parent_pivot_data['x'],
                                              parent_pivot_data['y'],
                                              parent_pivot_data['z']])

        joint_axis = mathutils.Vector([standard_axis_data['x'],
                                       standard_axis_data['y'],
                                       standard_axis_data['z']])

        # Rotation matrix representing child frame in parent frame
        R_j_p, r_j_p_angle = get_rot_mat_from_vecs(joint_axis, parent_axis)

        # Offset along constraint axis
        R_j_offset_rot = mathutils.Matrix().Rotation(self.get_joint_offset_angle(joint_data), 4, parent_axis)

        # Axis Alignment Offset resulting from adjusting the child bodies. If the child bodies are not
        # adjusted, this will be an identity matrix

        # Transformation of child in parents frame
        T_j_w = T_p_w @ P_j_p @ R_j_offset_rot @ R_j_p

        return T_j_w

    def get_child_in_world_transform(self, joint_data):
        parent_obj_handle, child_obj_handle = self.get_parent_and_child_object_handles(joint_data)
        parent_pivot_data, parent_axis_data = self.get_parent_pivot_and_axis_data(joint_data)
        child_pivot_data, child_axis_data = self.get_child_pivot_and_axis_data(joint_data)

        # Transformation matrix representing parent in world frame
        T_p_w = parent_obj_handle.matrix_world.copy()
        # Parent's Joint Axis in parent's frame
        parent_axis = mathutils.Vector([parent_axis_data['x'],
                                        parent_axis_data['y'],
                                        parent_axis_data['z']])
        # Transformation of joint in parent frame
        P_j_p = mathutils.Matrix()
        # P_j_p = P_j_p * r_j_p
        P_j_p.translation = mathutils.Vector([parent_pivot_data['x'],
                                              parent_pivot_data['y'],
                                              parent_pivot_data['z']])
        child_axis = mathutils.Vector([child_axis_data['x'],
                                       child_axis_data['y'],
                                       child_axis_data['z']])
        # Rotation matrix representing child frame in parent frame
        R_c_p, r_c_p_angle = get_rot_mat_from_vecs(child_axis, parent_axis)
        # print ('r_c_p')
        # print(r_c_p)
        # Transformation of joint in child frame
        P_j_c = mathutils.Matrix()
        # p_j_c *= r_j_c
        # If the child bodies have been adjusted. This pivot data will be all zeros
        P_j_c.translation = mathutils.Vector([child_pivot_data['x'],
                                              child_pivot_data['y'],
                                              child_pivot_data['z']])
        # print(p_j_c)
        # Transformation of child in joints frame
        P_c_j = P_j_c.copy()
        P_c_j.invert()
        # Offset along constraint axis
        R_j_offset_rot = mathutils.Matrix().Rotation(self.get_child_offset_angle(joint_data), 4, parent_axis)

        # Axis Alignment Offset resulting from adjusting the child bodies. If the child bodies are not
        # adjusted, this will be an identity matrix

        # Transformation of child in parents frame
        T_c_w = T_p_w @ P_j_p @ R_j_offset_rot @ R_c_p @ P_c_j

        return T_c_w

    def get_ambf_joint_handle(self, joint_data):
        bpy.ops.object.empty_add(type='PLAIN_AXES')
        joint_obj_handle = get_active_object()
        joint_name = str(joint_data['name'])

        joint_obj_handle.name = joint_name
        joint_obj_handle.scale = 0.1 * joint_obj_handle.scale
        bpy.ops.object.transform_apply(scale=True)

        return joint_obj_handle

    def create_ambf_constraint(self, joint_obj_handle, joint_type, parent_obj_handle, child_obj_handle):
        set_active_object(joint_obj_handle)
        select_object(joint_obj_handle)

        joint_obj_handle.ambf_object_type = 'CONSTRAINT'
        joint_obj_handle.ambf_constraint_type = joint_type

        joint_obj_handle.ambf_object_parent = parent_obj_handle
        joint_obj_handle.ambf_object_child = child_obj_handle

    def set_ambf_constraint_params(self, joint_obj_handle, joint_data):
        limits_defined = False
        joint_obj_handle.ambf_constraint_name = joint_data['name']
        joint_type = self.get_ambf_joint_type(joint_data)
        if 'joint limits' in joint_data:
            limits_defined = True
            if joint_type in ['REVOLUTE', 'TORSION_SPRING']:
                if 'low' in joint_data['joint limits'] and 'high' in joint_data['joint limits']:
                    joint_obj_handle.ambf_constraint_limits_lower = math.degrees(joint_data['joint limits']['low'])
                    joint_obj_handle.ambf_constraint_limits_higher = math.degrees(joint_data['joint limits']['high'])
            elif joint_type in ['PRISMATIC', 'LINEAR_SPRING']:
                    joint_obj_handle.ambf_constraint_limits_lower = joint_data['joint limits']['low']
                    joint_obj_handle.ambf_constraint_limits_higher = joint_data['joint limits']['high']
            elif joint_type == 'CONE_TWIST':
                joint_obj_handle.ambf_constraint_cone_twist_limits[0] = math.degrees(joint_data['joint limits']["x"])
                joint_obj_handle.ambf_constraint_cone_twist_limits[1] = math.degrees(joint_data['joint limits']["y"])
                joint_obj_handle.ambf_constraint_cone_twist_limits[2] = math.degrees(joint_data['joint limits']["z"])
            elif joint_type in ['SIX_DOF', 'SIX_DOF_SPRING']:
                joint_obj_handle.ambf_constraint_six_dof_limits_low_angular[0] = math.degrees(joint_data['joint limits']["angular"]["low"]["x"])
                joint_obj_handle.ambf_constraint_six_dof_limits_low_angular[1] = math.degrees(joint_data['joint limits']["angular"]["low"]["y"])
                joint_obj_handle.ambf_constraint_six_dof_limits_low_angular[2] = math.degrees(joint_data['joint limits']["angular"]["low"]["z"])

                joint_obj_handle.ambf_constraint_six_dof_limits_high_angular[0] = math.degrees(joint_data['joint limits']["angular"]["high"]["x"])
                joint_obj_handle.ambf_constraint_six_dof_limits_high_angular[1] = math.degrees(joint_data['joint limits']["angular"]["high"]["y"])
                joint_obj_handle.ambf_constraint_six_dof_limits_high_angular[2] = math.degrees(joint_data['joint limits']["angular"]["high"]["z"])

                joint_obj_handle.ambf_constraint_six_dof_limits_low_linear[0] = joint_data['joint limits']["linear"]["low"]["x"]
                joint_obj_handle.ambf_constraint_six_dof_limits_low_linear[1] = joint_data['joint limits']["linear"]["low"]["y"]
                joint_obj_handle.ambf_constraint_six_dof_limits_low_linear[2] = joint_data['joint limits']["linear"]["low"]["z"]

                joint_obj_handle.ambf_constraint_six_dof_limits_high_linear[0] = joint_data['joint limits']["linear"]["high"]["x"]
                joint_obj_handle.ambf_constraint_six_dof_limits_high_linear[1] = joint_data['joint limits']["linear"]["high"]["y"]
                joint_obj_handle.ambf_constraint_six_dof_limits_high_linear[2] = joint_data['joint limits']["linear"]["high"]["z"]

                if joint_type == 'SIX_DOF_SPRING':
                    joint_obj_handle.ambf_constraint_six_dof_stiffness_angular[0] = joint_data['stiffness']["angular"]["x"]
                    joint_obj_handle.ambf_constraint_six_dof_stiffness_angular[1] = joint_data['stiffness']["angular"]["y"]
                    joint_obj_handle.ambf_constraint_six_dof_stiffness_angular[2] = joint_data['stiffness']["angular"]["z"]

                    joint_obj_handle.ambf_constraint_six_dof_stiffness_linear[0] = joint_data['stiffness']["linear"]["x"]
                    joint_obj_handle.ambf_constraint_six_dof_stiffness_linear[1] = joint_data['stiffness']["linear"]["y"]
                    joint_obj_handle.ambf_constraint_six_dof_stiffness_linear[2] = joint_data['stiffness']["linear"]["z"]

                    joint_obj_handle.ambf_constraint_six_dof_damping_angular[0] = joint_data['damping']["angular"]["x"]
                    joint_obj_handle.ambf_constraint_six_dof_damping_angular[1] = joint_data['damping']["angular"]["y"]
                    joint_obj_handle.ambf_constraint_six_dof_damping_angular[2] = joint_data['damping']["angular"]["z"]

                    joint_obj_handle.ambf_constraint_six_dof_damping_linear[0] = joint_data['damping']["linear"]["x"]
                    joint_obj_handle.ambf_constraint_six_dof_damping_linear[1] = joint_data['damping']["linear"]["y"]
                    joint_obj_handle.ambf_constraint_six_dof_damping_linear[2] = joint_data['damping']["linear"]["z"]

                    joint_obj_handle.ambf_constraint_six_dof_equilibrium_angular[0] = math.degrees(joint_data['equilibrium point']["angular"]["x"])
                    joint_obj_handle.ambf_constraint_six_dof_equilibrium_angular[1] = math.degrees(joint_data['equilibrium point']["angular"]["y"])
                    joint_obj_handle.ambf_constraint_six_dof_equilibrium_angular[2] = math.degrees(joint_data['equilibrium point']["angular"]["z"])

                    joint_obj_handle.ambf_constraint_six_dof_equilibrium_linear[0] = joint_data['equilibrium point']["linear"]["x"]
                    joint_obj_handle.ambf_constraint_six_dof_equilibrium_linear[1] = joint_data['equilibrium point']["linear"]["y"]
                    joint_obj_handle.ambf_constraint_six_dof_equilibrium_linear[2] = joint_data['equilibrium point']["linear"]["z"]


                
        self.set_default_ambf_constraint_axis(joint_obj_handle)

        if not limits_defined:
            joint_obj_handle.ambf_constraint_limits_enable = False

        if joint_type != 'SIX_DOF_SPRING':
            if 'damping' in joint_data:
                joint_obj_handle.ambf_constraint_damping = joint_data['damping']

            if 'stiffness' in joint_data:
                if joint_type in ['LINEAR_SPRING', 'TORSION_SPRING']:
                    joint_obj_handle.ambf_constraint_stiffness = joint_data['stiffness']
                    if 'equilibrium point' in joint_data:
                        joint_obj_handle.ambf_constraint_equilibrium_point = math.degrees(joint_data['equilibrium point'])
                    else:
                        # Set the eq point midway between lower and upper limit
                        joint_obj_handle.ambf_constraint_equilibrium_point = (joint_obj_handle.ambf_constraint_limits_lower
                                                                              + joint_obj_handle.ambf_constraint_limits_higher) / 2.0

                
        if 'enable feedback' in joint_data:
                joint_obj_handle.ambf_constraint_enable_feedback = joint_data['enable feedback']

        if 'passive' in joint_data:
                joint_obj_handle.ambf_constraint_passive = joint_data['passive']

        # If joint controller is defined. Set the corresponding values in the joint properties
        if 'controller' in joint_data:
            if joint_type in ['REVOLUTE', 'PRISMATIC']:
                joint_obj_handle.ambf_constraint_enable_controller_gains = True
                joint_obj_handle.ambf_constraint_controller_p_gain = joint_data["controller"]["P"]
                joint_obj_handle.ambf_constraint_controller_i_gain = joint_data["controller"]["I"]
                joint_obj_handle.ambf_constraint_controller_d_gain = joint_data["controller"]["D"]
                if 'controller output type' in joint_data:
                    joint_obj_handle.ambf_constraint_controller_output_type = joint_data['controller output type']
                else:
                    joint_obj_handle.ambf_constraint_controller_output_type = 'FORCE'
        else:
            joint_obj_handle.ambf_constraint_controller_p_gain = 10
            joint_obj_handle.ambf_constraint_controller_i_gain = 0
            joint_obj_handle.ambf_constraint_controller_d_gain = 1.0
            joint_obj_handle.ambf_constraint_controller_output_type = 'VELOCITY'

        if 'max motor impulse' in joint_data:
            if joint_type in ['REVOLUTE', 'PRISMATIC']:
                joint_obj_handle.ambf_constraint_max_motor_impulse = joint_data["max motor impulse"]

    def load_ambf_joint(self, joint_name):
        joint_data = self._adf_data[joint_name]
        select_all_objects(False)
        set_active_object(None)
        # Set joint type to blender appropriate name

        parent_obj_handle, child_obj_handle = self.get_parent_and_child_object_handles(joint_data)
        joint_obj_handle = self.get_ambf_joint_handle(joint_data)

        T_j_w = self.get_joint_in_world_transform(joint_data)
        joint_obj_handle.matrix_world = T_j_w

        T_c_w = self.get_child_in_world_transform(joint_data)
        # Set the child body the pose calculated above
        # If the child_obj already has a parent, no need to set its transform again
        if child_obj_handle.parent is None:
            child_obj_handle.matrix_world = T_c_w

        make_obj1_parent_of_obj2(obj1=parent_obj_handle, obj2=joint_obj_handle)
        make_obj1_parent_of_obj2(obj1=joint_obj_handle, obj2=child_obj_handle)

        self.create_ambf_constraint(joint_obj_handle, self.get_ambf_joint_type(joint_data), parent_obj_handle,
                                       child_obj_handle)

        self.set_ambf_constraint_params(joint_obj_handle, joint_data)

        CommonConfig.loaded_joint_map[child_obj_handle.rigid_body_constraint] = joint_data

    def execute(self, context):
        self._yaml_filepath = str(bpy.path.abspath(context.scene['ambf_load_adf_filepath']))
        print(self._yaml_filepath)
        yaml_file = open(self._yaml_filepath)
        
        # Check YAML version
        ver = [int(x, 10) for x in yaml.__version__.split('.')]
        if ver[0] >= 5:
            self._adf_data = yaml.load(yaml_file, Loader=yaml.FullLoader)
        else:
            self._adf_data = yaml.load(yaml_file)
        self._context = context

        bodies_list = self._adf_data['bodies']
        joints_list = self._adf_data['joints']

        if 'namespace' in self._adf_data:
            set_global_namespace(context, self._adf_data['namespace'])
        else:
            set_global_namespace(context, '/ambf/env/')

        # num_bodies = len(bodies_list)
        # print('Number of Bodies Specified = ', num_bodies)

        self._high_res_path = self.get_qualified_path(self._adf_data['high resolution path'])
        # print(self._high_res_path)
        for body_name in bodies_list:
            self.load_body(body_name)

        for joint_name in joints_list:
            self.load_ambf_joint(joint_name)

        # Set the model ignore collision flag
        context.scene.ambf_ignore_inter_collision = self._adf_data['ignore inter-collision']

        # print('Printing Blender Remapped Body Names')
        # print(self._blender_remapped_body_names)
        return {'FINISHED'}


class AMBF_OT_cleanup_all(Operator):
    """Add Rigid Body Properties"""
    bl_label = "CLEAN UP ALL"
    bl_idname = "ambf.ambf_cleanup_all"

    def execute(self, context):
        for o in bpy.data.objects:
            bpy.data.objects.remove(o)
        return {'FINISHED'}


class AMBF_OT_hide_all_joints(Operator):
    """Add Joint Properties"""
    bl_label = "HIDE ALL JOINTS (TOGGLE)"
    bl_idname = "ambf.ambf_hide_all_joints"

    def execute(self, context):
        for o in bpy.data.objects:
            if o.ambf_object_type == 'CONSTRAINT':
                hidden = is_object_hidden(o)
                hide_object(o, not hidden)
        return {'FINISHED'}


class AMBF_OT_hide_passive_joints(Operator):
    """Add Joint Properties"""
    bl_label = "HIDE PASSIVE JOINTS (TOGGLE)"
    bl_idname = "ambf.ambf_hide_passive_joints"

    def execute(self, context):
        for o in bpy.data.objects:
            if o.ambf_object_type == 'CONSTRAINT':
                if o.ambf_constraint_passive:
                    hidden = is_object_hidden(o)
                    hide_object(o, not hidden)
        return {'FINISHED'}


class AMBF_OT_ambf_rigid_body_cleanup(Operator):
    """Add Rigid Body Properties"""
    bl_label = "AMBF RIGID BODY CLEANUP"
    bl_idname = "ambf.ambf_rigid_body_cleanup"

    def execute(self, context):
        for o in bpy.data.objects:
            if o.ambf_object_type == 'RIGID_BODY':
                bpy.data.objects.remove(o)
        return {'FINISHED'}


class AMBF_OT_ambf_constraint_cleanup(Operator):
    """Add Rigid Body Properties"""
    bl_label = "AMBF CONSTRAINT CLEANUP"
    bl_idname = "ambf.ambf_constraint_cleanup"

    def execute(self, context):
        for o in bpy.data.objects:
            if o.ambf_object_type == 'CONSTRAINT':
                bpy.data.objects.remove(o)
        return {'FINISHED'}


class AMBF_OT_ambf_collision_shape_cleanup(Operator):
    """Add Rigid Body Properties"""
    bl_label = "AMBF COLLISION SHAPE CLEANUP"
    bl_idname = "ambf.ambf_collision_shape_cleanup"

    def execute(self, context):
        for o in bpy.data.objects:
            if o.ambf_object_type == 'COLLISION_SHAPE':
                bpy.data.objects.remove(o)
        return {'FINISHED'}


class AMBF_OT_ambf_collision_shape_add(Operator):
    """Add Rigid Body Properties"""
    bl_label = "ADD COLLISION SHAPE"
    bl_idname = "ambf.ambf_collision_shape_add"

    def execute(self, context):
        add_collision_shape_property(context.object)
        return {'FINISHED'}


class AMBF_OT_ambf_collision_shape_remove(Operator):
    """Add Rigid Body Properties"""
    bl_label = "REMOVE COLLISION SHAPE"
    bl_idname = "ambf.ambf_collision_shape_remove"

    def execute(self, context):
        cnt = len(context.object.ambf_collision_shape_prop_collection.items())
        if cnt > 1:
            remove_collision_shape_property(context.object, idx=cnt - 1)
        else:
            print('WARNING, CANNOT HAVE LESS THAN 1 COLLISION SHAPE FOR COMPOUND COLLISION')

        return {'FINISHED'}

class AMBF_OT_ambf_rigid_body_activate(Operator):
    """Add Rigid Body Properties"""
    bl_label = "AMBF RIGID BODY ACTIVATE"
    bl_idname = "ambf.ambf_rigid_body_activate"

    def execute(self, context):
        if context.object.ambf_object_type != 'RIGID_BODY':
            context.object.ambf_object_type = 'RIGID_BODY'
            cnt = len(context.object.ambf_collision_shape_prop_collection.items())
            if cnt == 0:
                add_collision_shape_property(context.object)
        else:
            context.object.ambf_object_type = 'NONE'

        return {'FINISHED'}


class AMBF_OT_ambf_ghost_object_activate(Operator):
    """Add GHOST OBJECT Properties"""
    bl_label = "AMBF GHOST OBJECT ACTIVATE"
    bl_idname = "ambf.ambf_ghost_object_activate"

    def execute(self, context):
        if context.object.ambf_object_type != 'GHOST_OBJECT':
            context.object.ambf_object_type = 'GHOST_OBJECT'
            cnt = len(context.object.ambf_collision_shape_prop_collection.items())
            if cnt == 0:
                add_collision_shape_property(context.object)
        else:
            context.object.ambf_object_type = 'NONE'

        return {'FINISHED'}


class AMBF_OT_ambf_constraint_activate(Operator):
    """Add Rigid Body Properties"""
    bl_label = "AMBF CONSTRAINT ACTIVATE"
    bl_idname = "ambf.ambf_constraint_activate"

    def execute(self, context):
        if context.object.ambf_object_type != 'CONSTRAINT':
            context.object.ambf_object_type = 'CONSTRAINT'
        else:
            context.object.ambf_object_type = 'NONE'

        return {'FINISHED'}


class AMBF_PT_main_panel(Panel):
    """Creates a Panel in the Tool Shelf"""
    bl_label = "LOAD, CREATE AND SAVE ADFs"
    bl_idname = "AMBF_PT_main_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "AMBF"

    setup_yaml()

    def draw(self, context):

        # Sanity check, if there are any objects
        # that have been unlinked from the scene. Delete them
        for o in bpy.data.objects:
            if o.ambf_object_type in ['RIGID_BODY', 'CONSTRAINT', 'COLLISION_SHAPE']:
                if context.scene.objects.get(o.name) is None:
                    bpy.data.objects.remove(o)

        layout = self.layout
        
        col = layout.column()
        col.prop(context.scene, 'ambf_enable_forced_cleanup')
        
        box = layout.box()
        box.enabled = context.scene.ambf_enable_forced_cleanup
        box.label(text='WARNING! CLEAN UP ALL OBJECTS')

        col = box.column()
        col.operator("ambf.ambf_cleanup_all")

        col = box.column()
        col.operator("ambf.ambf_rigid_body_cleanup")

        col = box.column()
        col.operator("ambf.ambf_constraint_cleanup")

        col = box.column()
        col.operator("ambf.ambf_collision_shape_cleanup")
        
        col = box.column()
        col.operator("ambf.ambf_hide_passive_joints")

        col = box.column()
        col.operator("ambf.ambf_hide_all_joints")
        
        box = layout.box()
        row = box.row()
        # Load AMBF File Into Blender
        row.alignment = 'CENTER'
        row.label(text="LOAD ADF:", icon='IMPORT')

        # Load
        col = box.column()
        col.alignment = 'CENTER'
        col.prop(context.scene, 'ambf_load_adf_filepath')
        
        col = box.column()
        col.alignment = 'CENTER'
        col.operator("ambf.load_ambf_file")
        
        ### SEPERATOR
        layout.separator()

        box = layout.box()
        
        row = box.row()
        row.alignment = 'CENTER'
        row.label(text='CREATE ADF:', icon='EXPORT')
        
        # Panel Label
        sbox = box.box()
        row = sbox.row()
        row.label(text="A. ONLY FOR CONVEX HULL COLL.")

        # Mesh Reduction Ratio Properties
        row = sbox.row(align=True)
        row.alignment = 'LEFT'
        split = row.split(factor=0.7)
        row = split.row()
        row.label(text='Coll Mesh Max Verts: ')
        row = split.row()
        row.prop(context.scene, 'ambf_mesh_max_vertices')
        
        # Low Res Mesh Modifier Button
        col = sbox.column()
        col.alignment = 'CENTER'
        col.operator("ambf.generate_low_res_mesh_modifiers")

        sbox = box.box()
        row = sbox.row()
        row.label(text="B. OPTIONAL (ALL BODIES)")
        
        # Column for creating joint
        col = sbox.column()
        col.operator('ambf.estimate_collision_shapes_geometry')

        col = sbox.column()
        col.operator("ambf.estimate_shape_offsets")

        col = sbox.column()
        col.operator("ambf.estimate_inertial_offsets")
        
        col = sbox.column()
        col.operator("ambf.estimate_inertias")

        col = sbox.column()
        col.operator("ambf.estimate_joint_controller_gains")

        col = sbox.column()
        col.prop(context.scene, "ambf_show_collision_shapes", toggle=True)

        # Panel Label
        sbox = box.box()
        row = sbox.row()
        row.label(text="C. SAVE MESHES + TEXTURES")

        # Meshes Save Location
        col = sbox.column()
        col.prop(context.scene, 'ambf_meshes_path')

        # Select the Mesh-Type for saving the meshes
        col = sbox.column()
        col.alignment = 'CENTER'
        col.prop(context.scene, 'ambf_meshes_save_type')

        row = sbox.row()
        row.prop(context.scene, 'ambf_save_high_res')

        row = row.row()
        row.prop(context.scene, 'ambf_save_low_res')

        row = row.row()
        row.prop(context.scene, 'ambf_save_textures')

        col = sbox.column()
        col.prop(context.scene, 'ambf_save_selection_only')

        # Meshes Save Button
        col = sbox.column()
        col.alignment = 'CENTER'
        col.operator("ambf.save_meshes")

        # Panel Label
        sbox = box.box()
        row = sbox.row()
        row.label(text="D. SAVE ADF")
        
        # Ignore Inter Collision Button
        col = sbox.column()
        col.alignment = 'CENTER'
        col.prop(context.scene, "ambf_ignore_inter_collision")

        # Ignore Inter Collision Button
        col = sbox.column()
        col.alignment = 'CENTER'
        col.prop(context.scene, "ambf_precision")
        
        # AMBF Namespace
        col = sbox.column()
        col.alignment = 'CENTER'
        col.prop(context.scene, 'ambf_namespace', text='Global NS')
        
        # Config File Save Location
        col = sbox.column()
        col.prop(context.scene, 'ambf_adf_path', text='Save As')

        col = sbox.column()
        col.alignment = 'CENTER'
        col.operator("ambf.add_generate_ambf_file")
        
        ### SEPERATOR
        layout.separator()

        box = layout.box()
        row = box.row()
        row.alignment = 'CENTER'
        row.label(text="OPTIONAL HELPERS:", icon='OUTLINER_DATA_ARMATURE')

        col = box.column()
        col.alignment = 'CENTER'
        col.operator("ambf.remove_object_namespaces")

        # Add Optional Button to Remove All Modifiers
        col = box.column()
        col.alignment = 'CENTER'
        col.operator("ambf.remove_low_res_mesh_modifiers")

        # Add Optional Button to Toggle the Visibility of Low-Res Modifiers
        col = box.column()
        col.alignment = 'CENTER'
        col.operator("ambf.toggle_low_res_mesh_modifiers_visibility")
        
        col = box.column()
        col.operator("ambf.auto_rename_joints")
        
        row = box.row()
        row.scale_y = 1.5
        row.operator("ambf.create_joint")
        
        
        ### SEPERATOR
        layout.separator()

        box = layout.box()
        
        row = box.row()
        row.alignment = 'CENTER'
        row.label(text="LEGACY:", icon='CONSOLE')


class AMBF_PT_ambf_rigid_body(Panel):
    """Add Rigid Body Properties"""
    bl_label = "AMBF RIGID BODY PROPERTIES"
    bl_idname = "AMBF_PT_ambf_rigid_body"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "physics"
    
    @classmethod
    def poll(self, context):
        active = False
        active_obj_handle = get_active_object()
        if active_obj_handle: # Check if an obj_handle is active
            if active_obj_handle.type in ['EMPTY', 'MESH']:
                active = True
                
        return active
    
    def draw(self, context):
        layout = self.layout
        
        col = layout.row()
        col.alignment = 'EXPAND'
        col.scale_y = 2
        col.operator('ambf.ambf_rigid_body_activate', text='Enable AMBF Rigid Body', icon='RNA_ADD')

        if context.object.ambf_object_type == 'RIGID_BODY':
            layout.separator() 
            layout.separator()

            col = layout.column()
            col.prop(context.object, 'ambf_object_visible', toggle=True)
            col.scale_y = 2.0

            col = layout.column()
            col.enabled = False
            col.prop(context.object, 'ambf_rigid_body_namespace')
            
            box = layout.box()

            row = box.row()
            row.prop(context.object, 'ambf_rigid_body_is_static', toggle=True)
            
            col = row.row()
            col.enabled = not context.object.ambf_rigid_body_is_static
            col.alignment = 'EXPAND'
            col.prop(context.object, 'ambf_rigid_body_mass')
            
            row = box.row()
            split = row.split()
            row = split.row()
            row.enabled = not context.object.ambf_rigid_body_is_static
            col = row.column()
            col.scale_y = 1.5
            col.operator("ambf.estimate_inertia_per_object")
            col = col.column()
            col.scale_y = 1.5
            col.prop(context.object, 'ambf_rigid_body_specify_inertia', toggle=True)
            
            row = split.row()
            row.enabled = context.object.ambf_rigid_body_specify_inertia and not context.object.ambf_rigid_body_is_static
            col = row.column()
            col.prop(context.object, 'ambf_rigid_body_inertia_x')
            
            col = col.column()
            col.prop(context.object, 'ambf_rigid_body_inertia_y')
            
            col = col.column()
            col.prop(context.object, 'ambf_rigid_body_inertia_z')
            
            # Inertial Offsets
            box.separator()
            col = box.column()
            col.operator("ambf.estimate_inertial_offset_per_object")
            
            col = box.column()
            col = col.split(factor=0.5)
            col.alignment = 'EXPAND'
            col.prop(context.object, 'ambf_rigid_body_linear_inertial_offset')
            
            col = col.column()
            col.enabled = False
            col.alignment = 'EXPAND'
            col.prop(context.object, 'ambf_rigid_body_angular_inertial_offset')
            
            layout.separator()

            box = layout.box()
            row = box.row()
            row.prop(context.object, 'ambf_collision_type')

            if context.object.ambf_collision_type == 'MESH':

                col = box.column()
                col.prop(context.object, 'ambf_collision_mesh_type')

            elif context.object.ambf_collision_type == 'SINGULAR_SHAPE':

                col = box.column()
                col.operator('ambf.estimate_collision_shape_geometry_per_object')
                propgroup = context.object.ambf_collision_shape_prop_collection.items()[0][1]
                draw_collision_shape_prop(context, propgroup, box)
                
            elif context.object.ambf_collision_type == 'COMPOUND_SHAPE':
                
                cnt = len(context.object.ambf_collision_shape_prop_collection.items())
                for i in range(cnt):
                    propgroup = context.object.ambf_collision_shape_prop_collection.items()[i][1]
                    draw_collision_shape_prop(context, propgroup, box)
                row = box.row()
                row.operator('ambf.ambf_collision_shape_add',  text='ADD SHAPE')
                row = row.column()
                row.operator('ambf.ambf_collision_shape_remove', text='REMOVE SHAPE')
                if cnt == 1:
                    row.enabled = False
            
            box.separator()
            row = box.row()
            row.prop(context.object, 'ambf_collision_margin_enable', toggle=True)
            
            row = row.row()
            row.enabled = context.object.ambf_collision_margin_enable
            row.prop(context.object, 'ambf_collision_margin')
            
            row = box.column()
            row.alignment = 'EXPAND'
            row.prop(context.object, 'ambf_collision_groups', toggle=True)

            col = box.column()
            col.prop(context.object, 'ambf_collision_show_shapes_per_object', toggle=True)
            col.scale_y = 1.5
            
            layout.separator()
            
            box = layout.box()
            
            row = box.row()
            row.prop(context.object, 'ambf_rigid_body_static_friction')

            row = box.row()
            row.prop(context.object, 'ambf_rigid_body_rolling_friction')
            
            box.separator()
            
            row = box.row()
            row.prop(context.object, 'ambf_rigid_body_linear_damping')
            
            row = box.row()
            row.prop(context.object, 'ambf_rigid_body_angular_damping')
            
            box.separator()
            
            row = box.row()
            row.prop(context.object, 'ambf_rigid_body_restitution')
            
            layout.separator()
            
            # Rigid Body Controller Properties
            box = layout.box()
            row = box.row()
            row.alignment = 'CENTER'
            row.prop(context.object, 'ambf_rigid_body_enable_controllers', toggle=True)
            row.scale_y=2
        
            col = box.column()
            col.label(text='Linear Gains')
            
            col = box.column()
            col.enabled = context.object.ambf_rigid_body_enable_controllers
            row = col.row()
            row.prop(context.object, 'ambf_rigid_body_linear_controller_p_gain', text='P')
        
            row = row.row()
            row.prop(context.object, 'ambf_rigid_body_linear_controller_i_gain', text='I')

            row = row.row()
            row.prop(context.object, 'ambf_rigid_body_linear_controller_d_gain', text='D')
            
            col = box.column()
            col.label(text='Angular Gains')
            
            col = box.column()
            col.enabled = context.object.ambf_rigid_body_enable_controllers
            row = col.row()
            row.prop(context.object, 'ambf_rigid_body_angular_controller_p_gain', text='P')
        
            row = row.row()
            row.prop(context.object, 'ambf_rigid_body_angular_controller_i_gain', text='I')

            row = row.row()
            row.prop(context.object, 'ambf_rigid_body_angular_controller_d_gain', text='D')

            row = col.row()
            row.prop(context.object, 'ambf_rigid_body_controller_output_type')
            
            layout.separator()
            
            # Publish various children properties
            box = layout.box()
            
            col = box.column()
            col.prop(context.object, 'ambf_rigid_body_passive')
            
            col = box.column()
            col.prop(context.object, 'ambf_rigid_body_publish_children_names')
            col.enabled = not context.object.ambf_rigid_body_passive

            col = box.column()
            col.prop(context.object, 'ambf_rigid_body_publish_joint_names')
            col.enabled = not context.object.ambf_rigid_body_passive

            col = box.column()
            col.prop(context.object, 'ambf_rigid_body_publish_joint_positions')
            col.enabled = not context.object.ambf_rigid_body_passive


class AMBF_PT_ambf_ghost_object(Panel):
    """Add Ghost Object Properties"""
    bl_label = "AMBF GHOST OBJECT PROPERTIES"
    bl_idname = "AMBF_PT_ambf_ghost_object"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "physics"

    @classmethod
    def poll(self, context):
        active = False
        active_obj_handle = get_active_object()
        if active_obj_handle:  # Check if an obj_handle is active
            if active_obj_handle.type == 'MESH':
                active = True

        return active

    def draw(self, context):
        layout = self.layout

        col = layout.row()
        col.alignment = 'EXPAND'
        col.scale_y = 2
        col.operator('ambf.ambf_ghost_object_activate', text='Enable AMBF Ghost Object', icon='RNA_ADD')

        if context.object.ambf_object_type == 'GHOST_OBJECT':
            layout.separator()
            layout.separator()

            col = layout.column()
            col.enabled = False
            col.prop(context.object, 'ambf_rigid_body_namespace')

            layout.separator()

            col = layout.column()
            col.prop_search(context.object, "ambf_object_parent", context.scene, "objects")

            box = layout.box()
            row = box.row()
            row.prop(context.object, 'ambf_collision_type')

            if context.object.ambf_collision_type == 'MESH':

                col = box.column()
                col.prop(context.object, 'ambf_collision_mesh_type')

            elif context.object.ambf_collision_type == 'SINGULAR_SHAPE':

                col = box.column()
                col.operator('ambf.estimate_collision_shape_geometry_per_object')
                propgroup = context.object.ambf_collision_shape_prop_collection.items()[0][1]
                draw_collision_shape_prop(context, propgroup, box)

            elif context.object.ambf_collision_type == 'COMPOUND_SHAPE':

                cnt = len(context.object.ambf_collision_shape_prop_collection.items())
                for i in range(cnt):
                    propgroup = context.object.ambf_collision_shape_prop_collection.items()[i][1]
                    draw_collision_shape_prop(context, propgroup, box)
                row = box.row()
                row.operator('ambf.ambf_collision_shape_add', text='ADD SHAPE')
                row = row.column()
                row.operator('ambf.ambf_collision_shape_remove', text='REMOVE SHAPE')
                if cnt == 1:
                    row.enabled = False

            box.separator()
            row = box.row()
            row.prop(context.object, 'ambf_collision_margin_enable', toggle=True)

            row = row.row()
            row.enabled = context.object.ambf_collision_margin_enable
            row.prop(context.object, 'ambf_collision_margin')

            row = box.column()
            row.alignment = 'EXPAND'
            row.prop(context.object, 'ambf_collision_groups', toggle=True)

            col = box.column()
            col.prop(context.object, 'ambf_collision_show_shapes_per_object', toggle=True)
            col.scale_y = 1.5

            layout.separator()

            # Publish various children properties
            box = layout.box()

            col = box.column()
            col.prop(context.object, 'ambf_rigid_body_passive')
    

class AMBF_PT_ambf_constraint(Panel):
    """Add Rigid Body Properties"""
    bl_label = "AMBF CONSTRAINT PROPERTIES"
    bl_idname = "AMBF_PT_ambf_constraint"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context= "physics"
    
    @classmethod
    def poll(self, context):
        active = False
        active_obj_handle = get_active_object()
        if active_obj_handle: # Check if an obj_handle is active
            if active_obj_handle.type in ['EMPTY']:
                active = True
                          
        return active
    
    def draw(self, context):
        
        layout = self.layout
        
        row = layout.row()
        row.alignment = 'EXPAND'
        row.operator('ambf.ambf_constraint_activate', text='Enable AMBF Constraint', icon='FORCE_HARMONIC')
        row.scale_y = 2
        
        if context.object.ambf_object_type == 'CONSTRAINT':
            layout.separator()
            col = layout.column()
            col.operator('ambf.auto_rename_joint_per_object')
            
            col = layout.column()
            col.alignment = 'CENTER'
            col.prop(context.object, 'ambf_constraint_name')
            
            col = layout.column()
            col.prop(context.object, 'ambf_constraint_type')
            
            col = layout.column()
            col.prop_search(context.object, "ambf_object_parent", context.scene, "objects")
            
            col = layout.column()
            col.prop_search(context.object, "ambf_object_child", context.scene, "objects")

            # If the parent or child have been deleted from the scene, they might still be
            # present but unlinked. In that case, clear the corresponding parent or child handle
            if context.object.ambf_object_parent:
                if context.scene.objects.get(context.object.ambf_object_parent.name) is None:
                    context.object.ambf_object_parent = None

            if context.object.ambf_object_child:
                if context.scene.objects.get(context.object.ambf_object_child.name) is None:
                    context.object.ambf_object_child = None

            
            layout.separator()
            
            if context.object.ambf_constraint_type in ['PRISMATIC', 'REVOLUTE', 'LINEAR_SPRING', 'TORSION_SPRING']:
                row = layout.row()
                row.alignment = 'EXPAND'
                row.prop(context.object, 'ambf_constraint_axis')

                row = layout.row()
                row.prop(context.object, 'ambf_constraint_damping')
                row.scale_y=1.5

                if context.object.ambf_constraint_type in ['LINEAR_SPRING', 'TORSION_SPRING']:
                    row = layout.row()
                    row.prop(context.object, 'ambf_constraint_stiffness')
                    row.scale_y=1.5
                    row = layout.row()
                    row.prop(context.object, 'ambf_constraint_equilibrium_point')
                    row.scale_y=1.5
                    
                layout.separator()
                
                split = layout.split(factor=0.3)
                row = split.column()
                row.alignment = 'CENTER'
                row.prop(context.object, 'ambf_constraint_limits_enable', toggle=True)
                row.scale_y=2
                
                if context.object.ambf_constraint_type in ['REVOLUTE', 'TORSION_SPRING']:
                    units = '(Degrees)'
                    
                elif context.object.ambf_constraint_type in ['PRISMATIC', 'LINEAR_SPRING']:
                    units = '(Meters)'
                
                row = split.column()
                row.enabled = context.object.ambf_constraint_limits_enable
                r1 = row.split(factor=0.8)
                r1.prop(context.object, 'ambf_constraint_limits_lower', text='Low')
                r2 = r1.row()
                r2.label(text=units)
                
                row = row.column()
                row.enabled = context.object.ambf_constraint_limits_enable
                r1 = row.split(factor=0.8)
                r1.prop(context.object, 'ambf_constraint_limits_higher', text='High')
                r2 = r1.row()
                r2.label(text=units)
 
                if context.object.ambf_constraint_type in ['PRISMATIC', 'REVOLUTE']:
                    layout.separator()
                    
                    if context.object.ambf_constraint_enable_controller_gains and not context.object.ambf_constraint_passive:
                        enable_gain_setting = True
                    else:
                        enable_gain_setting = False
                    
                    col = layout.column()
                    split = col.split(factor=0.3)
                    c1 = split.column()
                    c1.alignment = 'CENTER'
                    c1.prop(context.object, 'ambf_constraint_enable_controller_gains',
                             toggle=True,
                             text='Enable Gains')
                    c1.scale_y=3

                    s2 = split.split(factor=0.3)
                    c2 = s2.column()
                    c2.operator('ambf.estimate_joint_controller_gain_per_object', text='Estimate')
                    c2.scale_y=3
                    c2.enabled = enable_gain_setting
        
                    c3 = s2.column()
                    c3.enabled = enable_gain_setting
                    c3.prop(context.object, 'ambf_constraint_controller_p_gain', text='P')
        
                    r3 = c3.row()
                    r3.prop(context.object, 'ambf_constraint_controller_i_gain', text='I')

                    r3 = c3.row()
                    r3.prop(context.object, 'ambf_constraint_controller_d_gain', text='D')

                    row = layout.row()
                    row.prop(context.object, 'ambf_constraint_controller_output_type')
                    row.enabled = enable_gain_setting

                    layout.separator()

                    col = layout.column()
                    col.scale_y = 2.0
                    col.prop(context.object, 'ambf_constraint_max_motor_impulse')
            elif context.object.ambf_constraint_type in ['CONE_TWIST']:
                row = layout.row()
                row.label(text="Limits")
                box = layout.box()
                col = box.column()
                col.prop(context.object, 'ambf_constraint_cone_twist_limits')
                row = layout.row()
                row.prop(context.object, 'ambf_constraint_damping')
                row.scale_y = 1.5
            elif context.object.ambf_constraint_type in ['SIX_DOF', 'SIX_DOF_SPRING']:

                row = layout.row()
                row.label(text="Limits")
                box = layout.box()

                col = box.column()
                split = col.split(factor=0.5)
                col = split.column()
                col.prop(context.object, 'ambf_constraint_six_dof_limits_low_angular')
                col = split.column()
                col.prop(context.object, 'ambf_constraint_six_dof_limits_high_angular')

                col = box.column()
                split = col.split(factor=0.5)
                col = split.column()
                col.prop(context.object, 'ambf_constraint_six_dof_limits_low_linear')
                col = split.column()
                col.prop(context.object, 'ambf_constraint_six_dof_limits_high_linear')

                if context.object.ambf_constraint_type == 'SIX_DOF_SPRING':

                    row = layout.row()
                    row.label(text="Angular (Stiffness / Damping)")
                    box = layout.box()
                    col = box.column()
                    split = col.split(factor=0.5)
                    col = split.column()
                    col.prop(context.object, 'ambf_constraint_six_dof_stiffness_angular')
                    col = split.column()
                    col.prop(context.object, 'ambf_constraint_six_dof_damping_angular')


                    row = layout.row()
                    row.label(text="Linear (Stiffness / Damping)")
                    box = layout.box()
                    col = box.column()
                    split = col.split(factor=0.5)
                    col = split.column()
                    col.prop(context.object, 'ambf_constraint_six_dof_stiffness_linear')
                    col = split.column()
                    col.prop(context.object, 'ambf_constraint_six_dof_damping_linear')


                    row = layout.row()
                    row.label(text="Equilibrium Point")
                    box = layout.box()
                    col = box.column()
                    col.prop(context.object, 'ambf_constraint_six_dof_equilibrium_angular')
                    col = box.column()
                    col.prop(context.object, 'ambf_constraint_six_dof_equilibrium_linear')

            layout.separator()

            col = layout.column()
            col.prop(context.object, 'ambf_constraint_enable_feedback')

            col = layout.column()
            col.prop(context.object, 'ambf_constraint_passive')


class AMBF_PG_CollisionShapePropGroup(PropertyGroup):
    ambf_collision_shape_radius: FloatProperty \
            (
            name='Radius',
            default=1.0,
            update=collision_shape_dims_update_cb,
            min=0.0001
        )

    ambf_collision_shape_height: FloatProperty \
            (
            name='Height',
            default=1.0,
            update=collision_shape_dims_update_cb,
            min=0.0001
        )

    ambf_collision_shape_xyz_dims: FloatVectorProperty \
            (
            name='Dimension (XYZ)',
            default=(1.0, 1.0, 1.0),
            min=0.0001,
            options={'PROPORTIONAL'},
            update=collision_shape_dims_update_cb,
            subtype='XYZ',
        )

    ambf_collision_shape_disable_update_cbs: BoolProperty(default=False)

    ambf_collision_shape_pointer: PointerProperty(name="Collision Shape", type=Object)

    ambf_collision_shape: EnumProperty \
            (
            items=
            [
                ('CONE', 'Cone', '', 'MESH_CONE', 0),
                ('CYLINDER', 'Cylinder', '', 'MESH_CYLINDER', 1),
                ('CAPSULE', 'Capsule', '', 'MESH_CAPSULE', 2),
                ('SPHERE', 'Sphere', '', 'MESH_UVSPHERE', 3),
                ('BOX', 'Box', '', 'MESH_CUBE', 4),
            ],
            name="Collision Shape",
            update=collision_shape_type_update_cb,
            default="BOX"
        )

    ambf_collision_shape_axis: EnumProperty \
            (
            name='Shape Axis',
            items=
            [
                ('X', 'X', '', '', 0),
                ('Y', 'Y', '', '', 1),
                ('Z', 'Z', '', '', 2),
            ],
            default='Z',
            update=collision_shape_axis_update_cb,
            description='The direction the collision shape is aligned. Use for Cone, Cylinder and Capsule'
        )

    ambf_collision_shape_linear_offset: FloatVectorProperty \
            (
            name='Linear Shape Offset',
            default=(0.0, 0.0, 0.0),
            options={'PROPORTIONAL'},
            update=collision_shape_offset_update_cb,
            subtype='XYZ',
        )

    ambf_collision_shape_angular_offset: FloatVectorProperty \
            (
            name='Angular Shape Offset',
            default=(0.0, 0.0, 0.0),
            options={'PROPORTIONAL'},
            update=collision_shape_offset_update_cb,
            subtype='EULER',
        )


custom_classes = (AMBF_OT_toggle_low_res_mesh_modifiers_visibility,
                  AMBF_PG_CollisionShapePropGroup,
                  AMBF_OT_cleanup_all,
                  AMBF_OT_ambf_rigid_body_cleanup,
                  AMBF_OT_ambf_constraint_cleanup,
                  AMBF_OT_ambf_collision_shape_cleanup,
                  AMBF_OT_hide_passive_joints,
                  AMBF_OT_hide_all_joints,
                  AMBF_OT_remove_low_res_mesh_modifiers,
                  AMBF_OT_generate_low_res_mesh_modifiers,
                  AMBF_OT_generate_ambf_file,
                  AMBF_OT_save_meshes,
                  AMBF_OT_load_ambf_file,
                  AMBF_OT_create_joint,
                  AMBF_OT_remove_object_namespaces,
                  AMBF_OT_estimate_inertial_offsets,
                  AMBF_OT_estimate_shape_offsets,
                  AMBF_OT_ambf_collision_shape_add,
                  AMBF_OT_ambf_collision_shape_remove,
                  AMBF_OT_estimate_collision_shapes_geometry,
                  AMBF_OT_estimate_inertias,
                  AMBF_OT_estimate_joint_controller_gains,
                  AMBF_OT_auto_rename_joints,
                  AMBF_OT_estimate_inertial_offset_per_object,
                  AMBF_OT_estimate_shape_offset_per_object,
                  AMBF_OT_estimate_collision_shape_geometry_per_object,
                  AMBF_OT_estimate_inertia_per_object,
                  AMBF_OT_estimate_joint_controller_gain_per_object,
                  AMBF_OT_auto_rename_joint_per_object,
                  AMBF_OT_ambf_rigid_body_activate,
                  AMBF_OT_ambf_ghost_object_activate,
                  AMBF_OT_ambf_constraint_activate,
                  AMBF_PT_main_panel,
                  AMBF_PT_ambf_rigid_body,
                  AMBF_PT_ambf_ghost_object,
                  AMBF_PT_ambf_constraint)


def register():
    from bpy.utils import register_class
    for cls in custom_classes:
        register_class(cls)
        
    Object.ambf_object_type = EnumProperty \
            (
            name="Object Type",
            items=
            [
                ('NONE', 'None', '', '', 0),
                ('RIGID_BODY', 'RIGID_BODY', '', '', 1),
                ('GHOST_OBJECT', 'GHOST_OBJECT', '', '', 2),
                ('CONSTRAINT', 'CONSTRAINT', '', '', 3),
                ('COLLISION_SHAPE', 'COLLISION_SHAPE', '', '', 4),
            ],
            default='NONE'
        )

    Object.ambf_rigid_body_namespace = StringProperty(name="Namespace", default="")

    Object.ambf_rigid_body_mass = FloatProperty(name="mass", default=1.0, min=0.0001)

    Object.ambf_rigid_body_inertia_x = FloatProperty(name='Ix', default=1.0, min=0.0)

    Object.ambf_rigid_body_inertia_y = FloatProperty(name='Iy', default=1.0, min=0.0)

    Object.ambf_rigid_body_inertia_z = FloatProperty(name='Iz', default=1.0, min=0.0)

    Object.ambf_rigid_body_static_friction = FloatProperty(name="Static Friction", default=0.5, min=0.0, max=10.0)

    Object.ambf_rigid_body_rolling_friction = FloatProperty(name="Rolling Friction", default=0.0, min=0.0, max=1.0)

    Object.ambf_rigid_body_restitution = FloatProperty(name="Restitution", default=0.1, min=0.0, max=1.0)

    Object.ambf_rigid_body_linear_damping = FloatProperty(name="Linear Damping", default=0.04, min=0.0, max=1.0)

    Object.ambf_rigid_body_angular_damping = FloatProperty(name="Angular Damping", default=0.1, min=0.0, max=1.0)

    Object.ambf_rigid_body_enable_controllers = BoolProperty(name="Enable Controllers", default=False)

    Object.ambf_rigid_body_linear_controller_p_gain = FloatProperty(name="Proportional Gain (P)", default=10, min=0)

    Object.ambf_rigid_body_linear_controller_i_gain = FloatProperty(name="Integral Gain (I)", default=0, min=0)

    Object.ambf_rigid_body_linear_controller_d_gain = FloatProperty(name="Damping Gain (D)", default=1, min=0)

    Object.ambf_rigid_body_angular_controller_p_gain = FloatProperty(name="Proportional Gain (P)", default=10, min=0)

    Object.ambf_rigid_body_angular_controller_i_gain = FloatProperty(name="Integral Gain (I)", default=0, min=0)

    Object.ambf_rigid_body_angular_controller_d_gain = FloatProperty(name="Damping Gain (D)", default=1, min=0)

    Object.ambf_rigid_body_passive = BoolProperty(name="Is Passive?", default=False,
                                                  description="If passive. this body will not be spawned as an AMBF communication object")

    Object.ambf_rigid_body_is_static = BoolProperty \
            (
            name="Static",
            default=False,
            description="Is this object dynamic or static (mass = 0.0 Kg)"
        )

    Object.ambf_rigid_body_specify_inertia = BoolProperty \
            (
            name="Specify Inertia",
            default=False,
            description="If not set explicitly, it is calculated automatically by AMBF"
        )

    Object.ambf_rigid_body_linear_inertial_offset = FloatVectorProperty \
            (
            name='Linear Inertial Offset',
            default=(0.0, 0.0, 0.0),
            options={'PROPORTIONAL'},
            update=collision_shape_offset_update_cb,
            subtype='XYZ',
        )

    Object.ambf_rigid_body_angular_inertial_offset = FloatVectorProperty \
            (
            name='Angular Inertial Offset',
            default=(0.0, 0.0, 0.0),
            options={'PROPORTIONAL'},
            subtype='EULER',
        )

    Object.ambf_rigid_body_controller_output_type = EnumProperty \
            (
            items=
            [
                ('VELOCITY', 'VELOCITY', '', '', 0),
                ('FORCE', 'FORCE', '', '', 1),
            ],
            name="Controller Output Type",
            default='VELOCITY',
            description='The output of the controller fed to the simulation. Better to use (VELOCITY) with P <= 10, D <= 1'
        )

    Object.ambf_rigid_body_publish_children_names = BoolProperty \
            (
            name="Publish Children Names",
            default=False
        )

    Object.ambf_rigid_body_publish_joint_names = BoolProperty \
            (
            name="Publish Joint Names",
            default=False
        )

    Object.ambf_rigid_body_publish_joint_positions = BoolProperty \
            (
            name="Publish Joint Positions",
            default=False
        )

    Object.ambf_object_visible = BoolProperty(name="Visible", default=True, description='Show this object in AMBF')

    Object.ambf_collision_margin_enable = BoolProperty(name="Enable Collision Margin", default=False)

    Object.ambf_collision_show_shapes_per_object = BoolProperty(name="Show Collision Shapes", default=False, update=collision_shape_show_per_object_update_cb)

    Object.ambf_collision_margin = FloatProperty(name="Margin", default=0.001, min=-0.1, max=1.0)

    Object.ambf_collision_type = EnumProperty \
            (
            name='Collision Type',
            items=
            [
                ('MESH', 'MESH', '', 'MESH_ICOSPHERE', 0),
                ('SINGULAR_SHAPE', 'Singular Shape', '', 'MESH_CUBE', 1),
                ('COMPOUND_SHAPE', 'Compound Shape', '', 'OUTLINER_OB_GROUP_INSTANCE', 2),
            ],
            default='MESH',
            update=rigid_body_collision_type_update_cb,
            description='Choose between a singular or a compound collision that consists of multiple shapes'
        )

    Object.ambf_collision_mesh_type = EnumProperty \
            (
            name='Collision Mesh Type',
            items=
            [
                ('CONCAVE_MESH', 'Concave Mesh', '', 'MESH_ICOSPHERE', 0),
                ('CONVEX_MESH', 'Convex Mesh', '', 'MESH_CUBE', 1),
                ('CONVEX_HULL', 'Convex Hull', '', 'OUTLINER_OB_GROUP_INSTANCE', 2),
            ],
            default='CONVEX_HULL',
            description='Choose between the type of the collision mesh. Avoid Concave Meshes if you can.'
        )

    Object.ambf_collision_groups = BoolVectorProperty \
            (
            name='Collision Groups',
            size=20,
            default=(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
            options={'PROPORTIONAL'},
            subtype='LAYER'
        )
    
    Object.ambf_constraint_type = EnumProperty \
            (
            items=
            [
                ('FIXED', 'Fixed', '', '', 0),
                ('REVOLUTE', 'Revolute', '', '', 1),
                ('PRISMATIC', 'Prismatic', '', '', 2),
                ('LINEAR_SPRING', 'Linear Spring', '', '', 3),
                ('TORSION_SPRING', 'Torsion Spring', '', '', 4),
                ('P2P', 'p2p', '', '', 5),
                ('CONE_TWIST', 'Cone Twist', '', '', 6),
                ('SIX_DOF', 'Six DOF', '', '', 7),
                ('SIX_DOF_SPRING', 'Six DOF Spring', '', '', 8),
            ],
            name="Type",
            default='REVOLUTE'
        )

    Object.ambf_object_parent = PointerProperty(name="Parent", type=Object)

    Object.ambf_object_child = PointerProperty(name="Child", type=Object)

    Object.ambf_constraint_name = StringProperty(name="Name", default="")

    Object.ambf_constraint_enable_controller_gains = BoolProperty(name="Enable Controller Gains", default=False)

    Object.ambf_constraint_controller_p_gain = FloatProperty(name="Proportional Gain (P)", default=10, min=0)

    Object.ambf_constraint_controller_i_gain = FloatProperty(name="Integral Gain (I)", default=0, min=0)

    Object.ambf_constraint_controller_d_gain = FloatProperty(name="Damping Gain (D)", default=1, min=0)

    Object.ambf_constraint_damping = FloatProperty(name="Joint Damping", default=0.7, min=0.0)

    Object.ambf_constraint_stiffness = FloatProperty(name="Joint Stiffness", default=10.0, min=0.0)

    Object.ambf_constraint_equilibrium_point = FloatProperty(name="Equilibrium Point", default=0.0)

    Object.ambf_constraint_limits_enable = BoolProperty(name="Enable Limits", default=True)

    Object.ambf_constraint_passive = BoolProperty(name="Is Passive?", default=False)

    Object.ambf_constraint_enable_feedback = BoolProperty(name="Enable Feedback", default=False)

    Object.ambf_constraint_limits_lower = FloatProperty(name="Low", default=-60, min=-359, max=359)

    Object.ambf_constraint_limits_higher = FloatProperty(name="High", default=60, min=-359, max=359)

    Object.ambf_constraint_max_motor_impulse = FloatProperty(name="Max Motor Impulse", default=0.05, min=0.0)

    Object.ambf_constraint_axis = EnumProperty \
            (
            name='Axis',
            items=
            [
                ('X', 'X', '', '', 0),
                ('Y', 'Y', '', '', 1),
                ('Z', 'Z', '', '', 2),
            ],
            default='Z'
        )

    Object.ambf_constraint_cone_twist_limits = FloatVectorProperty \
            (
            name='Cone Twist Swing Limits (Degrees)',
            default=(60, 60, 60),
            min=0.0,
            options={'PROPORTIONAL'},
            subtype='XYZ',
        )

    Object.ambf_constraint_six_dof_limits_low_angular = FloatVectorProperty \
            (
            name='Low: Limits Angular (Degrees)',
            default=(-60, -60, -60),
            options={'PROPORTIONAL'},
            subtype='XYZ',
        )

    Object.ambf_constraint_six_dof_limits_high_angular = FloatVectorProperty \
            (
            name='High: Limits Angular (Degrees)',
            default=(60, 60, 60),
            options={'PROPORTIONAL'},
            subtype='XYZ',
        )

    Object.ambf_constraint_six_dof_limits_low_linear = FloatVectorProperty \
            (
            name='Low: Limits Linear',
            default=(-1, -1, -1),
            options={'PROPORTIONAL'},
            subtype='XYZ',
        )

    Object.ambf_constraint_six_dof_limits_high_linear = FloatVectorProperty \
            (
            name='High: Limits Linear',
            default=(1, 1, 1),
            options={'PROPORTIONAL'},
            subtype='XYZ',
        )

    Object.ambf_constraint_six_dof_stiffness_angular = FloatVectorProperty \
            (
            name='Stiffness Angular',
            default=(1., 1., 1.),
            min=0.0,
            options={'PROPORTIONAL'},
            subtype='XYZ',
        )

    Object.ambf_constraint_six_dof_stiffness_linear = FloatVectorProperty \
            (
            name='Stiffness Linear',
            default=(1., 1., 1.),
            min=0.0,
            options={'PROPORTIONAL'},
            subtype='XYZ',
        )

    Object.ambf_constraint_six_dof_damping_angular = FloatVectorProperty \
            (
            name='Damping Angular',
            default=(1., 1., 1.),
            min=0.0,
            options={'PROPORTIONAL'},
            subtype='XYZ',
        )

    Object.ambf_constraint_six_dof_damping_linear = FloatVectorProperty \
            (
            name='Damping Linear',
            default=(1., 1., 1.),
            min=0.0,
            options={'PROPORTIONAL'},
            subtype='XYZ',
        )

    Object.ambf_constraint_six_dof_equilibrium_angular = FloatVectorProperty \
            (
            name='Equilibrium Angular (Degrees)',
            default=(0., 0., 0.),
            options={'PROPORTIONAL'},
            subtype='XYZ',
        )

    Object.ambf_constraint_six_dof_equilibrium_linear = FloatVectorProperty \
            (
            name='Equilibrium Linear',
            default=(0., 0., 0.),
            options={'PROPORTIONAL'},
            subtype='XYZ',
        )

    Object.ambf_constraint_controller_output_type = EnumProperty \
            (
            items=
            [
                ('VELOCITY', 'VELOCITY', '', '', 0),
                ('FORCE', 'FORCE', '', '', 1),
            ],
            name="Controller Output Type",
            default='VELOCITY',
            description='The output of the controller fed to the simulation. Better to use (VELOCITY) with P <= 10, D <= 1'
        )

    Scene.ambf_adf_path = StringProperty \
            (
            name="Config (Save To)",
            default="",
            description="Define the root path of the project",
            subtype='FILE_PATH'
        )

    Scene.ambf_meshes_path = StringProperty \
            (
            name="Meshes (Save To)",
            default="",
            description="Define the path to save to mesh files",
            subtype='DIR_PATH'
        )

    Scene.ambf_meshes_save_type = EnumProperty \
            (
            items=
            [
                ('STL', 'STL', 'STL'),
                ('OBJ', 'OBJ', 'OBJ'),
                ('3DS', '3DS', '3DS'),
                ('PLY', 'PLY', 'PLY')
            ],
            name="Mesh Type",
            default='STL'
        )

    Scene.ambf_mesh_max_vertices = IntProperty \
            (
            name="",
            default=150,
            description="The maximum number of vertices the low resolution collision mesh is allowed to have",
        )

    Scene.ambf_ignore_inter_collision = BoolProperty \
            (
            name="Ignore Inter-Collision",
            default=True,
            description="Ignore collision between all the bodies in the scene (default = True)",
        )

    Scene.ambf_precision = IntProperty \
            (
            name="Precision",
            default=5,
            description="Precision of scalar and vector variables to be written",
        )

    Scene.ambf_load_adf_filepath = StringProperty \
            (
            name="AMBF Config",
            default="",
            description="Load AMBF YAML FILE",
            subtype='FILE_PATH'
        )

    Scene.ambf_namespace = StringProperty \
            (
            name="AMBF Namespace",
            default="/ambf/env/",
            description="The namespace for all bodies in this scene"
        )

    Scene.ambf_show_collision_shapes = BoolProperty \
            (
            name="Show Collision Shapes",
            default=False,
            update=collision_shape_show_update_cb
        )

    Scene.ambf_enable_forced_cleanup = BoolProperty \
            (
            name="Enable Forced Cleanup",
            default=False
        )

    Scene.ambf_save_high_res = BoolProperty \
            (
            name="High Res",
            default=True,
            description="Save High Res Meshes for Visual."
        )

    Scene.ambf_save_low_res = BoolProperty \
            (
            name="Low Res",
            default=True,
            description="Save Low Res Meshes for Collision. Used only if the collision is set to MESH type"
        )

    Scene.ambf_save_textures = BoolProperty \
            (
            name="Textures",
            default=True,
            description="Save textures if defined for a body"
        )

    Scene.ambf_save_selection_only = BoolProperty \
            (
            name="Selection Only",
            default=False,
            description="Only save the meshes + textures for the selected object"
        )

    Object.ambf_collision_shape_prop_collection = CollectionProperty(type=AMBF_PG_CollisionShapePropGroup)

def unregister():
    from bpy.utils import unregister_class
    for cls in reversed(custom_classes):
        unregister_class(cls)


if __name__ == "__main__":
    register()
    #unregister()
