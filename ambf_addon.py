# Author: Adnan Munawar
# Email: amunawar@wpi.edu
# Lab: aimlab.wpi.edu

# Editor: Vanessa Wang
# Email: swang368@jh.edu
# Lab: LCSR, Johns Hopkins University

bl_info = {
    "name": "Asynchronous Multi-Body Framework (AMBF) Config Creator",
    "author": "Adnan Munawar, Shiyue Vanessa Wang",
    "version": (0, 1),
    "blender": (4, 0, 0),
    "location": "View3D > Add > Mesh > AMBF",
    "description": "Helps Generate AMBF Config File and Saves both High and Low Resolution(Collision) Meshes",
    "warning": "",
    # "wiki_url": "https://github.com/WPI-AIM/ambf_addon",
    "wiki_url": "https://github.com/capricieuxV/ambf__blender_addon",
    "category": "AMBF",
    }

import bpy
import bmesh
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
        self._adf_data['collision mesh'] = ""
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
# No collision with this object
class GhostObjectTemplate:
    def __init__(self):
        self._adf_data = OrderedDict()
        self._adf_data['name'] = ""
        # self._adf_data['namespace'] = ""
        self._adf_data['parent'] = ""
        self._adf_data['mesh'] = ""
        self._adf_data['shape'] = ""
        # self._adf_data['geometry'] = "*shape_geometry" # Default to whatever is defined as pointer
        self._adf_data['geometry'] = get_xyz_ordered_dict()
        self._adf_data['location'] = get_pose_ordered_dict()
        self._adf_data['location']['position'] = get_xyz_ordered_dict()  
        self._adf_data['location']['orientation'] = get_rpy_ordered_dict() 
        self._adf_data['scale'] = 1.0
        self._adf_data['passive'] = False
        
        # self._adf_data['high resolution path'] = ""
        # self._adf_data['low resolution path'] = ""
        
        self._adf_data['collision mesh type'] = "CONVEX_HULL" # Default to convex hull
        self._adf_data['collision margin'] = 0.001
        self._adf_data['collision group'] = []
        self._adf_data['collision shape'] = "BOX" # Default to box
        self._adf_data['collision geometry'] = get_xyz_ordered_dict()
        # self._adf_data['collision geometry'] = "*box_geometry" # Default to box
        # self._adf_data['compound shape'] = []
        # self._adf_data['collision offset'] = get_xyz_ordered_dict() 
        # self._adf_data['compound collision shape'] = []
        # self._adf_data['publish frequency'] = 0.0

        self._adf_data['color components'] = {'ambient': {'level': 1.0}, 'diffuse': {'r': 0.5, 'g': 0.5, 'b':0.5}, 'specular': {'r': 0.5, 'g': 0.05, 'b':0.05}, 'transparency': 1.0}
        # self._adf_data['color'] = 'random'
        # self._adf_data['color rgba'] = {'r': 1.0, 'g': 1.0, 'b': 1.0, 'a': 1.0}
        
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

class CameraTemplate:
    def __init__(self):
        self._adf_data = OrderedDict()
        self._adf_data['name'] = ''
        self._adf_data['location'] = get_pose_ordered_dict()
        self._adf_data['look at'] = get_pose_ordered_dict()
        self._adf_data['up'] = get_pose_ordered_dict()
        self._adf_data['clipping plane'] = {'near': 1.0, 'far': 10.0}
        self._adf_data['field view angle'] = 1.0
        self._adf_data['monitor'] = 0
        self._adf_data['publish image'] = False
        self._adf_data['publish image interval'] = 1
        self._adf_data['publish image resolution'] = {'width': 640, 'height': 480}
        self._adf_data['publish depth'] = False
        self._adf_data['publish depth interval'] = 1
        self._adf_data['publish depth resolution'] = {'width': 640, 'height': 480}
        # self._adf_data['orthographic view width'] = 0.0
        # self._adf_data['stereo'] = {'mode': "", 'eye separation': 0.0, 'focal length': 0.0}
        # self._adf_data['controlling devices'] = []
        # self._adf_data['publish depth noise'] = {'mean': 0.0, 'std_dev': 0.0, 'bias': 0.0}
        # self._adf_data['preprocessing shaders'] = []
        # self._adf_data['depth compute shaders'] = []
        # self._adf_data['multipass'] = False
        # self._adf_data['mouse control multipliers'] = {'pan': 0.0, 'rotate': 0.0, 'scroll': 0.0, 'arcball': 0.0}

class LightTemplate:
    def __init__(self):
        self._adf_data = OrderedDict()
        self._adf_data['namespace'] = ''
        self._adf_data['name'] = ''
        self._adf_data['location'] = get_pose_ordered_dict()
        self._adf_data['type'] = 'SPOT'
        self._adf_data['direction'] = get_xyz_ordered_dict()
        self._adf_data['spot exponent'] = 1.0
        self._adf_data['shadow quality'] = 1.0
        self._adf_data['cutoff angle'] = 1.0
        # self._adf_data['color'] = ''
        # self._adf_data['intensity'] = 0.0
        # self._adf_data['spot'] = {'angle': 0.0, 'blend': 0.0}
        # self._adf_data['distance'] = 0.0
        # self._adf_data['decay'] = 0.0
        self._adf_data['attenuation'] = {'constant': 1.0, 'linear': 0.0, 'quadratic': 0.0}

class SensorTemplate:
    def __init__(self, sensor_type="Proximity", publish_frequency=None):
        self._adf_data = OrderedDict()
        self._adf_data['name'] = ''
        self._adf_data['type'] = sensor_type
        self._adf_data['parent'] = ''
        self._adf_data['location'] = get_pose_ordered_dict()
        self._adf_data['location']['position'] = get_xyz_ordered_dict()  
        self._adf_data['location']['orientation'] = get_rpy_ordered_dict() 
        if publish_frequency is not None:
            self._adf_data['publish frequency'] = publish_frequency
        self._adf_data['visible'] = False
        self._adf_data['visible size'] = 1.0
        
        if sensor_type == "Proximity":
            self._adf_data['range'] = 0.1
            self._adf_data['array'] = []  

        elif sensor_type == "Resistance":
            self._adf_data.update({
                'friction': {'static': 0.0, 'damping': 0.0, 'dynamic': 0.0, 'variable': False},
                'contact area': 0.0,
                'contact stiffness': 0.0,
                'contact damping': 0.0
            })

        elif sensor_type == "Contact":
            self._adf_data.update({
                'distance threshold': 0.0,
                'process contact details': False
            })

class ActuatorTemplate:
    def __init__(self):
        self._adf_data = OrderedDict()
        self._adf_data['name'] = ''
        # self._adf_data['namespace'] = ''
        self._adf_data['type'] = 'Constraint'
        self._adf_data['parent'] = ''
        self._adf_data['location'] = get_pose_ordered_dict()  
        self._adf_data['location']['position'] = get_xyz_ordered_dict()  
        self._adf_data['location']['orientation'] = get_rpy_ordered_dict() 
        self._adf_data['visible'] = False
        self._adf_data['visible size'] = 1.0
        # self._adf_data['publish frequency'] = 0.0
        # self._adf_data['max impulse'] = 0.0
        # self._adf_data['tau'] = 0.0

class SoftBodyTemplate:
    def __init__(self):
        self._adf_data = OrderedDict()
        self._adf_data['name'] = ''
        self._adf_data['namespace'] = ''

        self._adf_data['location'] = OrderedDict()
        self._adf_data['location']['position'] = get_xyz_ordered_dict()
        self._adf_data['location']['orientation'] = get_rpy_ordered_dict()

        self._adf_data['inertial offset'] = OrderedDict()
        self._adf_data['inertial offset']['position'] = get_xyz_ordered_dict()
        self._adf_data['inertial offset']['orientation'] = get_rpy_ordered_dict()

        self._adf_data['color components'] = {
            'ambient': {'level': 1.0},
            'diffuse': {'r': 0.5, 'g': 0.5, 'b': 0.5},
            'specular': {'r': 0.5, 'g': 0.5, 'b': 0.5}
        }

        self._adf_data['mesh'] = ''
        self._adf_data['scale'] = 1.0
        self._adf_data['mass'] = 1.0

        self._adf_data['config'] = OrderedDict()
        self._adf_data['randomize constraints'] = False

# Global Variables
class CommonConfig:
    namespace = ''
    num_collision_groups = 20
    # Some properties don't exist in Blender are supported in AMBF. If an AMBF file is loaded
    # and then resaved, we can capture the extra properties of bodies and joints and take
    # them into consideration before re saving the AMBF File so we don't reset those values
    loaded_body_map = {}
    loaded_joint_map = {}
    loaded_camera_map = {}
    loaded_light_map = {}
    loaded_sensor_map = {}
    loaded_actuator_map = {}
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


def set_view_transform_orientation_to_local():
    # bpy.context.scene.transform_orientation_slots[0].type = 'LOCAL'
    pass


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

def get_namespace(fullname):
    # print("Handling get_namespace...")
    last_occurance = fullname.rfind('/')
    _namespace = ''
    if last_occurance >= 0:
        # This means that the name contains a namespace
        _namespace = fullname[0:last_occurance+1]
    return _namespace

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

def compare_namespace_with_global(fullname):
    # print("Handling compare_namespace_with_global...")
    last_occurance = fullname.rfind('/')
    _is_namespace_same = False
    _namespace = ''
    _name = ''
    if last_occurance >= 0:
        # This means that the name contains a namespace
        _namespace = fullname[0:last_occurance+1]
        _name = fullname[last_occurance+1:]

        if CommonConfig.namespace == _namespace:
            # The CommonConfig namespace is the same as the object namespace
            _is_namespace_same = True
        else:
            # The CommonConfig namespace is different form object namespace
            _is_namespace_same = False

    else:
        # The object's name does not contain and namespace
        _is_namespace_same = False

    # print("FULLNAME: %s, OBJ: %s, NAMESPACE: %s NAMESPACE_MATCHED: %d" %
    # (fullname, _name, _namespace, _is_namespace_same))
    return _is_namespace_same


def add_namespace_prefix(name):
    return CommonConfig.namespace + name

def normalize_name(name):
    return name.replace(' ', '').lower()

def get_grand_parent(body):
    grand_parent = body
    while grand_parent.parent is not None:
        # print('GRAND PARENT: ', grand_parent.name)
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
        # print('ADDING OBJ: ', obj_handle.name, 'TO HEIRARCHIAL LIST')
        _added_bodies_list[obj_handle] = False

    for body in _heirarchial_bodies_list:
            print(body.name, "--->",)

    for obj_handle in bpy.data.objects:
        grand_parent = get_grand_parent(obj_handle)
        # print('CALLING DOWNWARD TREE PASS FOR: ', grand_parent.name)
        downward_tree_pass(grand_parent, _heirarchial_bodies_list, _added_bodies_list)

    return _heirarchial_bodies_list


# Courtesy: https://stackoverflow.com/questions/5914627/prepend-line-to-beginning-of-a-file
def prepend_comment_to_file(filename, comment):
    temp_filename = filename + '.tmp'
    with open(filename,'r') as f:
        with open(temp_filename, 'w') as f2:
            f2.write(comment)
            f2.write(f.read())
    os.rename(temp_filename, filename)

def update_selected_collections(self, context):
    selected_collections = self.selected_collections.split(',')
    for collection in bpy.data.collections:
        if collection.name in selected_collections:
            collection.hide_viewport = False
        else:
            collection.hide_viewport = True

# Hook to ensure new objects are added to the active collection
def ensure_active_collection(scene):
    if scene.active_collection_name:
        active_collection = bpy.data.collections.get(scene.active_collection_name)
        if active_collection:
            for obj in bpy.context.selected_objects:
                for col in obj.users_collection:
                    col.objects.unlink(obj)
                active_collection.objects.link(obj)

bpy.types.Scene.selected_collections = bpy.props.StringProperty(
    name="Selected Collections",
    description="Comma-separated list of selected collection names",
    default="",
    update=update_selected_collections
)

def find_object_by_normalized_name(normalized_name):
    for obj in bpy.data.objects:
        name = obj.name.split('/')[-1].replace(' ', '').lower()
        if normalize_name(name) == normalized_name:
            return obj
    return None

def select_object(obj_handle, select=True):
    # print(obj_handle.name)
    obj_handle.select_set(select)

def select_objects(obj_handles, select=True):
    # print('SELECTING OBJECTS: ', len(obj_handles))
    for obj_handle in obj_handles:
        select_object(obj_handle, select)

def select_all_objects(select):
    # print('SELECTING ALL OBJECTS: ', len(bpy.data.objects))
    # First deselect all objects
    for obj_handle in bpy.data.objects:
        select_object(obj_handle, select)

def hide_object(object, hide):
    # print('HIDING OBJECT: ', object.name)
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
            mass_p = parent_obj_handle.ambf_body_mass
            mass_c = child_obj_handle.ambf_body_mass
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
        mass = obj_handle.ambf_body_mass
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
    mass = obj_handle.ambf_body_mass
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
    off = obj_handle.ambf_body_linear_inertial_offset
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


def _resolve_blender_operator(op_path):
    op_handle = bpy.ops
    for attr in op_path.split('.'):
        if not hasattr(op_handle, attr):
            return None
        op_handle = getattr(op_handle, attr)
    return op_handle


def _call_first_available_operator(candidates, error_message):
    for op_path, kwargs in candidates:
        op_handle = _resolve_blender_operator(op_path)
        if op_handle is not None:
            return op_handle(**kwargs)
    raise RuntimeError(error_message)


def load_blender_mesh(context, mesh_filepath, name):
    result = True
    if mesh_filepath.suffix in ['.stl', '.STL']:
        path = str(mesh_filepath.resolve())
        _call_first_available_operator(
            [
                ("wm.stl_import", {"filepath": path}),
                ("import_mesh.stl", {"filepath": path}),
            ],
            "No STL import operator available in Blender."
        )

    elif mesh_filepath.suffix in ['.obj', '.OBJ']:
        print('Importing OBJ: ', mesh_filepath)
        _manually_select_obj_handle = True
        path = str(mesh_filepath.resolve())
        _call_first_available_operator(
            [
                ("wm.obj_import", {"filepath": path, "up_axis": 'Z', "forward_axis": 'Y'}),
                ("import_scene.obj", {"filepath": path, "axis_up": 'Z', "axis_forward": 'Y'}),
            ],
            "No OBJ import operator available in Blender."
        )
        # Hack, .3ds and .obj imports do not make the imported obj_handle active. A hack is
        # to capture the selected objects in this case.
        set_active_object(context.selected_objects[0])

    elif mesh_filepath.suffix in ['.dae', '.DAE']:
        bpy.ops.wm.collada_import(filepath=str(mesh_filepath.resolve()))
        # If we are importing .dae meshes, they can import stuff other than meshes, such as cameras etc.
        # We should remove these extra things and only keep the meshes
        for temp_obj_handle in context.selected_objects:
            if temp_obj_handle.type == 'MESH':
                obj_handle = temp_obj_handle
                # set_active_object(obj_handle)
            else:
                bpy.data.objects.remove(temp_obj_handle)

        so = bpy.context.selected_objects
        if len(so) > 1:
            set_active_object(so[0])
            bpy.ops.object.join()
            so[0].name = name
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
            r_x = mathutils.Matrix.Rotation(-pi / 2, 4, 'X')
            obj_handle.data.transform(r_x)
        else:
            set_active_object(so[0])

    elif mesh_filepath.suffix in ['.3ds', '.3DS']:
        _manually_select_obj_handle = True
        path = str(mesh_filepath.resolve())
        _call_first_available_operator(
            [
                ("import_scene.autodesk_3ds", {"filepath": path}),
                ("import_scene.max3ds", {"filepath": path}),
                ("wm.max3ds_import", {"filepath": path}),
            ],
            "No 3DS import operator available in Blender."
        )
        # Hack, .3ds and .obj imports do not make the imported obj_handle active. A hack is
        # to capture the selected objects in this case.
        set_active_object(context.selected_objects[0])

    elif mesh_filepath.suffix == '':
        bpy.ops.object.empty_add(type='PLAIN_AXES')

    else:
        # We failed, mark as false
        result = False

    return result

# TODO: save to mesh not working for all except .stl
def save_blender_mesh(obj_handle, mesh_filepath, mesh_type, use_mesh_modifiers):
    print('\nHandle Save Blender Mesh: ', obj_handle.name, 'TO: ', mesh_filepath)
    hide_state = is_object_hidden(obj_handle)
    hide_object(obj_handle, False)
    select_object(obj_handle, True)

    mesh_filepath = mesh_filepath + '.' + mesh_type
    if mesh_type == 'STL':
        _call_first_available_operator(
            [
                ("wm.stl_export", {
                    "filepath": mesh_filepath,
                    "export_selected_objects": True,
                    "apply_modifiers": use_mesh_modifiers,
                }),
                ("export_mesh.stl", {
                    "filepath": mesh_filepath,
                    "use_selection": True,
                    "use_mesh_modifiers": use_mesh_modifiers,
                }),
            ],
            "No STL export operator available in Blender."
        )
    elif mesh_type == 'OBJ':
        _call_first_available_operator(
            [
                ("wm.obj_export", {
                    "filepath": mesh_filepath,
                    "up_axis": 'Z',
                    "forward_axis": 'Y',
                    "export_selected_objects": True,
                    "apply_modifiers": use_mesh_modifiers,
                }),
                ("export_scene.obj", {
                    "filepath": mesh_filepath,
                    "axis_up": 'Z',
                    "axis_forward": 'Y',
                    "use_selection": True,
                    "use_mesh_modifiers": use_mesh_modifiers,
                }),
            ],
            "No OBJ export operator available in Blender."
        )
    elif mesh_type == '3DS':
        # 3DS doesn't support suppressing modifiers, so we explicitly
        # toggle them to save as high res and low res meshes
        # STILL BUGGY
        for mod in obj_handle.modifiers:
            mod.show_viewport = True

        _call_first_available_operator(
            [
                ("export_scene.autodesk_3ds", {"filepath": mesh_filepath, "use_selection": True}),
                ("export_scene.max3ds", {"filepath": mesh_filepath, "use_selection": True}),
                ("wm.max3ds_export", {"filepath": mesh_filepath, "export_selected_objects": True}),
            ],
            "No 3DS export operator available in Blender."
        )

    elif mesh_type == 'PLY':
        # .PLY export has a bug in which it only saves the mesh that is
        # active in context of view. Hence we explicitly select this object
        # as active in the scene on top of being selected
        set_active_object(obj_handle)
        _call_first_available_operator(
            [
                ("wm.ply_export", {
                    "filepath": mesh_filepath,
                    "export_selected_objects": True,
                    "apply_modifiers": use_mesh_modifiers,
                }),
                ("export_mesh.ply", {
                    "filepath": mesh_filepath,
                    "use_mesh_modifiers": use_mesh_modifiers,
                }),
            ],
            "No PLY export operator available in Blender."
        )
        set_active_object(None)
    else:
        raise Exception('High Res Mesh Format Not Specified/Understood')

    select_object(obj_handle, False)
    hide_object(obj_handle, hide_state)
    

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
    if obj_handle.ambf_object_type == 'RIGID_BODY' or obj_handle.ambf_object_type == 'GHOST_OBJECT' or obj_handle.ambf_object_type == 'SOFT_BODY':

        if len(obj_handle.ambf_collision_shape_prop_collection.items()) == 0:
            add_collision_shape_property(obj_handle)
        # Don't bother if the shape is a compound shape for now. Let the
        # user calculate the geometries.
        if obj_handle.ambf_collision_type in ['MESH', 'SINGULAR_SHAPE']:
            dims = obj_handle.dimensions.copy()
            prop_group = obj_handle.ambf_collision_shape_prop_collection.items()[0][1]
            print('Estimating Collision Shape Geometry for: ', obj_handle.ambf_collision_shape_prop_collection.items())
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


def ambf_light_type_update_cb(self, context):
    if self.type == 'LIGHT':
        blender_type_map = {'POINT': 'POINT', 'SPOT': 'SPOT', 'DIRECTIONAL': 'SUN'}
        self.data.type = blender_type_map.get(self.ambf_light_type, 'SPOT')


def ambf_camera_clip_end_update_cb(self, context):
    if self.type == 'CAMERA':
        self.data.clip_end = self.ambf_camera_clip_end


def ambf_camera_clip_start_update_cb(self, context):
    if self.type == 'CAMERA':
        self.data.clip_start = self.ambf_camera_clip_start


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
    bl_idname = "ambf.generate_ambf_file"
    bl_label = "Write AMBF Description File (ADF)"
    bl_description = "This generated the AMBF Config file in the location and filename specified in the field" \
                     " above"

    def __init__(self, *args, **kwargs):
        pass 
    
    def execute(self, context):
        import types

        class _ADFGenHelper:
            pass

        helper = _ADFGenHelper()

        helper._body_names_list = []
        helper._ghost_object_names_list = []
        helper._soft_body_names_list = []
        helper._joint_names_list = []
        helper._camera_names_list = []
        helper._light_names_list = []
        helper._sensor_names_list = []
        helper._actuator_names_list = []

        helper.body_name_prefix = 'BODY '
        helper.joint_name_prefix = 'JOINT '
        helper.camera_name_prefix = ''
        helper.light_name_prefix = ''
        helper.sensor_name_prefix = 'SENSOR '
        helper.actuator_name_prefix = 'ACTUATOR '

        helper._adf = None

        method_names = [
            'add_body_prefix_str',
            'add_joint_prefix_str',
            'add_camera_prefix_str',
            'add_light_prefix_str',
            'add_sensor_prefix_str',
            'add_actuator_prefix_str',
            'generate_body_data_from_ambf_rigid_body',
            'generate_body_data_from_ambf_ghost_object',
            'generate_body_data_from_ambf_soft_body',
            'generate_joint_data_from_ambf_constraint',
            'generate_camera_data_from_ambf_camera',
            'generate_light_data_from_ambf_light',
            'generate_sensor_data_from_ambf_sensor',
            'generate_actuator_data_from_ambf_actuator',
            'get_axis_of_ambf_constraint',
            'compute_body_pivot_and_axis',
            'assign_joint_params_from_ambf_constraint',
        ]

        for name in method_names:
            setattr(helper, name, types.MethodType(getattr(AMBF_OT_generate_ambf_file, name), helper))

        AMBF_OT_generate_ambf_file.generate_adf(helper, context)
        return {'FINISHED'}

    # This joint adds the body prefix str if set to all the bodies(rigid, ghost, soft) in the AMBF
    def add_body_prefix_str(self, urdf_body_str):
        return self.body_name_prefix + urdf_body_str

    # This method add the joint prefix if set to all the joints in AMBF
    def add_joint_prefix_str(self, urdf_joint_str):
        return self.joint_name_prefix + urdf_joint_str
    
    # This method add the camera prefix if set to all the cameras in AMBF
    def add_camera_prefix_str(self, urdf_camera_str):
        return self.camera_name_prefix + urdf_camera_str
    
    # This method add the light prefix if set to all the lights in AMBF
    def add_light_prefix_str(self, urdf_light_str):
        return self.light_name_prefix + urdf_light_str
    
    # This method add the sensor prefix if set to all the sensors in AMBF
    def add_sensor_prefix_str(self, urdf_sensor_str):
        return self.sensor_name_prefix + urdf_sensor_str
    
    # This method add the actuator prefix if set to all the actuators in AMBF
    def add_actuator_prefix_str(self, urdf_actuator_str):
        return self.actuator_name_prefix + urdf_actuator_str

    def generate_body_data_from_ambf_ghost_object(self, adf_data, body_obj_handle):
        if body_obj_handle.ambf_object_type != 'GHOST_OBJECT':
            return

        # The object is unlinked from the scene. Don't write it
        if bpy.context.scene.objects.get(body_obj_handle.name) is None:
            return

        if is_object_hidden(body_obj_handle):
            return
        
        print('Generating Ghost Object: ', body_obj_handle.name)
        ghost = GhostObjectTemplate()
        ghost_data = ghost._adf_data

        if not compare_namespace_with_global(body_obj_handle.name):
            if get_namespace(body_obj_handle.name) != '':
                ghost_data['namespace'] = get_namespace(body_obj_handle.name)

        body_obj_handle_name = remove_namespace_prefix(body_obj_handle.name)
        ghost_yaml_name = self.add_body_prefix_str(body_obj_handle_name)
        output_mesh = bpy.context.scene.ambf_meshes_save_type
        ghost_data['name'] = body_obj_handle_name
        if body_obj_handle.ambf_object_parent:
            ghost_data['parent'] = body_obj_handle.ambf_object_parent
        ghost_pose = body_obj_handle.matrix_world
        ghost_data['location'] = {'position': {'x': ambf_round(ghost_pose.translation.x),
                                              'y': ambf_round(ghost_pose.translation.y),
                                              'z': ambf_round(ghost_pose.translation.z)},
                                'orientation': {'r': ambf_round(ghost_pose.to_euler().x),
                                                'p': ambf_round(ghost_pose.to_euler().y),
                                                'y': ambf_round(ghost_pose.to_euler().z)}}

        # Populate geometry and collision properties from the object handle
        # if body_obj_handle.type == 'MESH':

        ghost_data['mesh'] = body_obj_handle_name + '.' + output_mesh
        ghost_data['shape'] = body_obj_handle.ambf_ghost_shape
        ghost_data['geometry'] = {'x': body_obj_handle.dimensions[0], 'y': body_obj_handle.dimensions[1], 'z': body_obj_handle.dimensions[2]}
        if body_obj_handle.ambf_collision_margin_enable is True:
            ghost_data['collision margin'] = ambf_round(body_obj_handle.ambf_collision_margin)
        # ghost_data['collision shape'] = body_obj_handle.ambf_collision_shape
        # ghost_data['collision geometry'] = body_obj_handle.ambf_ghost_collision_geometry
        # ghost_data['collision group'] = body_obj_handle.ambf_collision_groups

        ghost_data['collision mesh type'] = body_obj_handle.ambf_collision_mesh_type
        ghost_data['collision group'] = [idx for idx, chk in enumerate(body_obj_handle.ambf_collision_groups) if chk == True]
        ghost_data['collision geometry'] = {
            'x': body_obj_handle.ambf_ghost_collision_geometry[0],
            'y': body_obj_handle.ambf_ghost_collision_geometry[1],
            'z': body_obj_handle.ambf_ghost_collision_geometry[2]
        } if body_obj_handle.ambf_ghost_collision_geometry else {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # TODO: handle the case ambf_collision_shape is various shapes
        
        ghost_data['scale'] = body_obj_handle.ambf_scale
        ghost_data['passive'] = body_obj_handle.ambf_body_passive
        ghost_data['transparency'] = body_obj_handle.ambf_body_transparency

        # Populate color components
        ghost_data['color components'] = {
            'ambient': {
                'level': body_obj_handle.ambf_object_ambient_level,
            },
            'diffuse': {
                'r': body_obj_handle.ambf_object_diffuse_color[0],
                'g': body_obj_handle.ambf_object_diffuse_color[1],
                'b': body_obj_handle.ambf_object_diffuse_color[2]
            },
            'specular': {
                'r': body_obj_handle.ambf_object_specular_color[0],
                'g': body_obj_handle.ambf_object_specular_color[1],
                'b': body_obj_handle.ambf_object_specular_color[2]
            },
            'transparency': body_obj_handle.ambf_body_transparency
        }

        adf_data[ghost_yaml_name] = ghost_data
        self._ghost_object_names_list.append(ghost_yaml_name)
    
    def generate_body_data_from_ambf_soft_body(self, adf_data, body_obj_handle):
        if body_obj_handle.ambf_object_type != 'SOFT_BODY':
            return

        # The object is unlinked from the scene. Don't write it
        if bpy.context.scene.objects.get(body_obj_handle.name) is None:
            return

        if is_object_hidden(body_obj_handle):
            return

        print('Generating Soft Body: ', body_obj_handle.name)

        soft = SoftBodyTemplate()
        soft_data = soft._adf_data

        # if not compare_namespace_with_global(body_obj_handle.name):
        #     if get_namespace(body_obj_handle.name) != '':
        #         soft_data['namespace'] = get_namespace(body_obj_handle.name)

        body_obj_handle_name = remove_namespace_prefix(body_obj_handle.name)
        soft_yaml_name = self.add_body_prefix_str(body_obj_handle_name)
        output_mesh = bpy.context.scene.ambf_meshes_save_type

        soft_data['name'] = body_obj_handle_name
        soft_data['mesh'] = body_obj_handle_name + '.' + output_mesh
        soft_data['mass'] = body_obj_handle.ambf_body_mass

        if body_obj_handle.ambf_collision_margin_enable:
            soft_data['collision margin'] = ambf_round(body_obj_handle.ambf_collision_margin)

        soft_data['scale'] = body_obj_handle.ambf_scale

        soft_pose = body_obj_handle.matrix_world
        soft_data['location'] = {
            'position': {
                'x': ambf_round(soft_pose.translation.x),
                'y': ambf_round(soft_pose.translation.y),
                'z': ambf_round(soft_pose.translation.z),
            },
            'orientation': {
                'r': ambf_round(soft_pose.to_euler().x),
                'p': ambf_round(soft_pose.to_euler().y),
                'y': ambf_round(soft_pose.to_euler().z),
            },
        }

        soft_data['inertial offset'] = {
            'position': {
                'x': ambf_round(body_obj_handle.ambf_body_linear_inertial_offset[0]),
                'y': ambf_round(body_obj_handle.ambf_body_linear_inertial_offset[1]),
                'z': ambf_round(body_obj_handle.ambf_body_linear_inertial_offset[2]),
            },
            'orientation': {
                'r': ambf_round(body_obj_handle.ambf_body_angular_inertial_offset[0]),
                'p': ambf_round(body_obj_handle.ambf_body_angular_inertial_offset[1]),
                'y': ambf_round(body_obj_handle.ambf_body_angular_inertial_offset[2]),
            },
        }

        soft_data['color components'] = {
            'ambient': {'level': body_obj_handle.ambf_object_ambient_level},
            'diffuse': {
                'r': body_obj_handle.ambf_object_diffuse_color[0],
                'g': body_obj_handle.ambf_object_diffuse_color[1],
                'b': body_obj_handle.ambf_object_diffuse_color[2],
            },
            'specular': {
                'r': body_obj_handle.ambf_object_specular_color[0],
                'g': body_obj_handle.ambf_object_specular_color[1],
                'b': body_obj_handle.ambf_object_specular_color[2],
            },
            'transparency': body_obj_handle.ambf_body_transparency,
        }

        # Check and add enabled configuration properties
        def add_if_enabled(config_key, enable_attr, value_attr):
            # print('Checking: ', config_key, enable_attr, value_attr)
            if getattr(body_obj_handle.ambf_soft_body_properties, enable_attr):
                # print('Adding: ', config_key, value_attr)
                value = getattr(body_obj_handle.ambf_soft_body_properties, value_attr)
                # print('Adding: ', config_key, value)
                soft_data['config'][config_key] = value

        properties = [
        ('kLST', 'ambf_soft_body_enable_linear_stiffness', 'ambf_soft_body_linear_stiffness'),
        ('kAST', 'ambf_soft_body_enable_angular_stiffness', 'ambf_soft_body_angular_stiffness'),
        ('kVST', 'ambf_soft_body_enable_volume_stiffness', 'ambf_soft_body_volume_stiffness'),
        ('kVCF', 'ambf_soft_body_enable_damping', 'ambf_soft_body_velocity_damping'),
        ('kDP', 'ambf_soft_body_enable_drag', 'ambf_soft_body_drag_coefficient'),
        ('kDG', 'ambf_soft_body_enable_friction', 'ambf_soft_body_dynamic_friction'),
        ('kLF', 'ambf_soft_body_enable_aerodynamics', 'ambf_soft_body_lift_coefficient'),
        ('kPR', 'ambf_soft_body_enable_pressure', 'ambf_soft_body_pressure_coefficient'),
        ('kVC', 'ambf_soft_body_enable_volume_conservation', 'ambf_soft_body_volume_conservation'),
        ('kDF', 'ambf_soft_body_enable_deformation_friction', 'ambf_soft_body_deformation_friction'),
        ('kMT', 'ambf_soft_body_enable_pose_matching', 'ambf_soft_body_pose_matching'),
        ('kCHR', 'ambf_soft_body_enable_collision_hardness', 'ambf_soft_body_collision_hardness'),
        ('kKHR', 'ambf_soft_body_enable_kinetic_hardness', 'ambf_soft_body_kinetic_hardness'),
        ('kSHR', 'ambf_soft_body_enable_shear_hardness', 'ambf_soft_body_shear_hardness'),
        ('kAHR', 'ambf_soft_body_enable_anchor_hardness', 'ambf_soft_body_anchor_hardness'),
        ('kSRHR_CL', 'ambf_soft_body_enable_srhr_cl_stiffness', 'ambf_soft_body_srhr_cl_stiffness'),
        ('kSKHR_CL', 'ambf_soft_body_enable_skhr_cl_stiffness', 'ambf_soft_body_skhr_cl_stiffness'),
        ('kSSHR_CL', 'ambf_soft_body_enable_sshr_cl_stiffness', 'ambf_soft_body_sshr_cl_stiffness'),
        ('kSR_SPLT_CL', 'ambf_soft_body_enable_sr_splt_cl_stiffness', 'ambf_soft_body_sr_splt_cl_stiffness'),
        ('kSK_SPLT_CL', 'ambf_soft_body_enable_sk_splt_cl_stiffness', 'ambf_soft_body_sk_splt_cl_stiffness'),
        ('kSS_SPLT_CL', 'ambf_soft_body_enable_ss_splt_cl_stiffness', 'ambf_soft_body_ss_splt_cl_stiffness'),
        ('maxvolume', 'ambf_soft_body_enable_max_volume', 'ambf_soft_body_max_volume'),
        ('timescale', 'ambf_soft_body_enable_timescale', 'ambf_soft_body_timescale'),
        ('viterations', 'ambf_soft_body_enable_velocity_iterations', 'ambf_soft_body_velocity_iterations'),
        ('piterations', 'ambf_soft_body_enable_position_iterations', 'ambf_soft_body_position_iterations'),
        ('diterations', 'ambf_soft_body_enable_deformation_iterations', 'ambf_soft_body_deformation_iterations'),
        ('citerations', 'ambf_soft_body_enable_collision_iterations', 'ambf_soft_body_collision_iterations'),
        ('flags', 'ambf_soft_body_enable_flags', 'ambf_soft_body_flags'),
        ('bending constraints', 'ambf_soft_body_enable_bending_constraint', 'ambf_soft_body_bending_constraint'),
        ('cutting', 'ambf_soft_body_enable_cutting_enabled', 'ambf_soft_body_cutting_enabled'),
        ('clusters', 'ambf_soft_body_enable_clusters', 'ambf_soft_body_clusters'),
    ]

        for config_key, enable_attr, value_attr in properties:
            add_if_enabled(config_key, enable_attr, value_attr)

        if body_obj_handle.ambf_soft_body_properties.ambf_soft_body_enable_fixed_nodes:
            soft_data['config']['fixed nodes'] = [
                node.node_index for node in body_obj_handle.ambf_soft_body_properties.ambf_soft_body_fixed_nodes
            ]
        
        soft_data['randomize constraints'] = body_obj_handle.ambf_soft_body_properties.ambf_soft_body_randomize_constraints
        
        adf_data[soft_yaml_name] = soft_data
        self._soft_body_names_list.append(soft_yaml_name)

    def generate_body_data_from_ambf_rigid_body(self, adf_data, body_obj_handle):

        if body_obj_handle.ambf_object_type != 'RIGID_BODY':
            return

        # The object is unlinked from the scene. Don't write it
        if bpy.context.scene.objects.get(body_obj_handle.name) is None:
            return

        if is_object_hidden(body_obj_handle) is True:
            return

        print('Generating Rigid Body: ', body_obj_handle.name)
        body = BodyTemplate()
        body_data = body._adf_data

        if not compare_namespace_with_global(body_obj_handle.name):
            if get_namespace(body_obj_handle.name) != '':
                body_data['namespace'] = get_namespace(body_obj_handle.name)

        body_obj_handle_name = remove_namespace_prefix(body_obj_handle.name)

        body_yaml_name = self.add_body_prefix_str(body_obj_handle_name)
        output_mesh = bpy.context.scene.ambf_meshes_save_type
        body_data['name'] = body_obj_handle_name
        
        body_data['passive'] = body_obj_handle.ambf_body_passive
        if body_obj_handle.ambf_body_passive:
            body_data['publish children names'] = False
            body_data['publish joint names'] = False
            body_data['publish joint positions'] = False
        else:
            body_data['publish children names'] = body_obj_handle.ambf_rigid_body_publish_children_names
            body_data['publish joint names'] = body_obj_handle.ambf_rigid_body_publish_joint_names
            body_data['publish joint positions'] = body_obj_handle.ambf_rigid_body_publish_joint_positions

        world_pos = body_obj_handle.matrix_world.translation
        world_rot = body_obj_handle.matrix_world.to_euler()
        body_pos = body_data['location']['position']
        body_rot = body_data['location']['orientation']
        body_pos['x'] = ambf_round(world_pos.x)
        body_pos['y'] = ambf_round(world_pos.y)
        body_pos['z'] = ambf_round(world_pos.z)
        body_rot['r'] = ambf_round(world_rot[0])
        body_rot['p'] = ambf_round(world_rot[1])
        body_rot['y'] = ambf_round(world_rot[2])

        if body_obj_handle.type == 'EMPTY':
            body_data['mesh'] = ''
            if body_obj_handle_name in ['world', 'World', 'WORLD']:
                body_data['mass'] = 0
            else:
                if body_obj_handle.ambf_rigid_body_is_static:
                    body_data['mass'] = 0.0
                else:
                    body_data['mass'] = body_obj_handle.ambf_body_mass
                    if body_obj_handle.ambf_rigid_body_specify_inertia:
                        body_data['inertia'] = {'ix': ambf_round(body_obj_handle.ambf_rigid_body_inertia_x),
                                                'iy': ambf_round(body_obj_handle.ambf_rigid_body_inertia_y),
                                                'iz': ambf_round(body_obj_handle.ambf_rigid_body_inertia_z)}
                    else:
                        body_data['inertia'] = {'ix': 0.01, 'iy': 0.01, 'iz': 0.01}

        elif body_obj_handle.type == 'MESH':

            if body_obj_handle.ambf_rigid_body_is_static:
                body_data['mass'] = 0.0
            else:
                body_data['mass'] = ambf_round(body_obj_handle.ambf_body_mass)
                if body_obj_handle.ambf_rigid_body_specify_inertia:
                    body_data['inertia'] = {'ix': ambf_round(body_obj_handle.ambf_rigid_body_inertia_x),
                                            'iy': ambf_round(body_obj_handle.ambf_rigid_body_inertia_y),
                                            'iz': ambf_round(body_obj_handle.ambf_rigid_body_inertia_z)}
                else:
                    # We can delete the inertia as it will be estimated in AMBF
                    del body_data['inertia']

            if body_obj_handle.ambf_object_override_gravity:
                body_data['gravity'] = {'x': body_obj_handle.ambf_object_gravity[0],
                                        'y': body_obj_handle.ambf_object_gravity[1],
                                        'z': body_obj_handle.ambf_object_gravity[2]}

            body_data['friction'] = {'static': ambf_round(body_obj_handle.ambf_rigid_body_static_friction),
                                     'rolling': ambf_round(body_obj_handle.ambf_rigid_body_rolling_friction)}

            body_data['restitution'] = ambf_round(body_obj_handle.ambf_rigid_body_restitution)

            body_data['damping'] = {'linear': ambf_round(body_obj_handle.ambf_rigid_body_linear_damping),
                                    'angular': ambf_round(body_obj_handle.ambf_rigid_body_angular_damping)}

            body_data['visible'] = body_obj_handle.ambf_object_visible

            body_data['collision groups'] = [idx for idx, chk in enumerate(body_obj_handle.ambf_collision_groups) if chk == True]

            if body_obj_handle.ambf_collision_margin_enable is True:
                body_data['collision margin'] = ambf_round(body_obj_handle.ambf_collision_margin)

            if body_obj_handle.ambf_collision_type == 'MESH':
                body_data['collision mesh type'] = body_obj_handle.ambf_collision_mesh_type
                if body_obj_handle.ambf_use_separate_collision_mesh:
                    if body_obj_handle.ambf_collision_mesh:
                        body_data['collision mesh'] = \
                            remove_namespace_prefix(body_obj_handle.ambf_collision_mesh.name + '.' + output_mesh)
                    else:
                        raise Exception("ERROR! For object ", body_obj_handle.name, " \" Use Separate Collision Mesh\" is \"True\" but \"Collision Mesh\" not specified.")
                else:
                    del body_data['collision mesh']
            else:
                del body_data['collision mesh']
                del body_data['collision mesh type']

                if body_obj_handle.ambf_collision_type == 'SINGULAR_SHAPE':
                    shape_prop_group = body_obj_handle.ambf_collision_shape_prop_collection.items()[0][1]
                    body_data['collision shape'] = shape_prop_group.ambf_collision_shape
                    bcg = OrderedDict()
                    dims = body_obj_handle.dimensions.copy()
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
                elif body_obj_handle.ambf_collision_type == 'COMPOUND_SHAPE':
                    if 'collision shape' in body_data:
                        del body_data['collision shape']
                    compound_shape = []
                    shape_count = 0
                    for prop_tuple in body_obj_handle.ambf_collision_shape_prop_collection.items():
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

            body_data['mesh'] = body_obj_handle_name + '.' + output_mesh
            xyz_inertial_off = get_xyz_ordered_dict()
            xyz_inertial_off['x'] = ambf_round(body_obj_handle.ambf_body_linear_inertial_offset[0])
            xyz_inertial_off['y'] = ambf_round(body_obj_handle.ambf_body_linear_inertial_offset[1])
            xyz_inertial_off['z'] = ambf_round(body_obj_handle.ambf_body_linear_inertial_offset[2])

            body_data['inertial offset']['position'] = xyz_inertial_off

            if body_obj_handle.data.materials:
                del body_data['color']
                if output_mesh == 'OBJ':# If saving as OBJ, ignore material info.
                    body_data['use material'] = False
                else:
                    body_data['use material'] = True

                body_data['color components'] = OrderedDict()
                body_data['color components'] = {'diffuse': {'r': 1.0, 'g': 1.0, 'b': 1.0},
                                                 'specular': {'r': 1.0, 'g': 1.0, 'b': 1.0},
                                                 'ambient': {'level': 0.5},
                                                 'transparency': 1.0}

                mat = body_obj_handle.data.materials[0]
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
            if body_obj_handle.ambf_rigid_body_enable_controllers is True:
                _controller_gains = OrderedDict()
                _lin_gains = OrderedDict()
                _ang_gains = OrderedDict()
                _lin_gains['P'] = ambf_round(body_obj_handle.ambf_rigid_body_linear_controller_p_gain)
                _lin_gains['I'] = ambf_round(body_obj_handle.ambf_rigid_body_linear_controller_i_gain)
                _lin_gains['D'] = ambf_round(body_obj_handle.ambf_rigid_body_linear_controller_d_gain)

                _ang_gains['P'] = ambf_round(body_obj_handle.ambf_rigid_body_angular_controller_p_gain)
                _ang_gains['I'] = ambf_round(body_obj_handle.ambf_rigid_body_angular_controller_i_gain)
                _ang_gains['D'] = ambf_round(body_obj_handle.ambf_rigid_body_angular_controller_d_gain)

                _controller_gains['linear'] = _lin_gains
                _controller_gains['angular'] = _ang_gains
                body_data['controller'] = _controller_gains
                body_data['controller output type'] = body_obj_handle.ambf_rigid_body_controller_output_type
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
    
    def generate_actuator_data_from_ambf_actuator(self, adf_data, actuator_obj_handle):
        if actuator_obj_handle.ambf_object_type != 'ACTUATOR':
            return

        # The object is unlinked from the scene. Don't write it
        if bpy.context.scene.objects.get(actuator_obj_handle.name) is None:
            return
        
        if is_object_hidden(actuator_obj_handle) is True:
            return
        
        print("Generating Actuator: ", actuator_obj_handle.name)

        actuator_template = ActuatorTemplate()
        actuator_data = actuator_template._adf_data  # Correct reference to actuator_data

        if not compare_namespace_with_global(actuator_obj_handle.name):
            if get_namespace(actuator_obj_handle.name) != '':
                actuator_data['namespace'] = get_namespace(actuator_obj_handle.name)
        
        actuator_obj_handle_name = remove_namespace_prefix(actuator_obj_handle.name)
        actuarator_yaml_name = self.add_actuator_prefix_str(actuator_obj_handle_name)
        actuator_data['name'] = actuator_obj_handle_name

        # Ensure `location` and its sub-keys are properly set up before assigning values
        if 'location' not in actuator_data:
            actuator_data['location'] = OrderedDict({
                'position': OrderedDict(),
                'orientation': OrderedDict()
            })

        # Populate position and orientation under location
        world_pos = actuator_obj_handle.matrix_world.translation
        world_rot = actuator_obj_handle.matrix_world.to_euler()
        actuator_data['location']['position']['x'] = ambf_round(world_pos.x)
        actuator_data['location']['position']['y'] = ambf_round(world_pos.y)
        actuator_data['location']['position']['z'] = ambf_round(world_pos.z)
        actuator_data['location']['orientation']['r'] = ambf_round(world_rot[0])
        actuator_data['location']['orientation']['p'] = ambf_round(world_rot[1])
        actuator_data['location']['orientation']['y'] = ambf_round(world_rot[2])

        # Set additional properties
        if actuator_obj_handle.ambf_object_parent is None:
            actuator_data['parent'] = ''
        else:
            parent_obj_handle = actuator_obj_handle.ambf_object_parent
            parent_obj_handle_name = remove_namespace_prefix(parent_obj_handle.name)
            actuator_data['parent'] = self.add_body_prefix_str(parent_obj_handle_name)
            
        actuator_data['visible'] = actuator_obj_handle.ambf_object_visible
        actuator_data['visible size'] = actuator_obj_handle.ambf_object_visible_size

        # Add to final data and list
        self._actuator_names_list.append(actuarator_yaml_name)
        adf_data[actuarator_yaml_name] = actuator_data

    def generate_sensor_data_from_ambf_sensor(self, adf_data, sensor_obj_handle):
        if sensor_obj_handle.ambf_object_type != 'SENSOR':
            return

        # Check if the object is unlinked or hidden in the scene
        if bpy.context.scene.objects.get(sensor_obj_handle.name) is None or is_object_hidden(sensor_obj_handle):
            return

        print("Generating Sensor: ", sensor_obj_handle.name)

        # Create a SensorTemplate instance with the sensor type and frequency
        sensor_obj = sensor_obj_handle.ambf_sensor_properties
        sensor_type = sensor_obj.ambf_sensor_type
        # TODO: Deal with sensor frequency in the future
        # freq = sensor_obj.ambf_sensor_frequency 
        freq = None
        sensor_template = SensorTemplate(sensor_type, freq)

        sensor_data = sensor_template._adf_data

        # Set namespace if available
        if not compare_namespace_with_global(sensor_obj_handle.name):
            namespace = get_namespace(sensor_obj_handle.name)
            if namespace:
                sensor_data['namespace'] = namespace

        # Set the sensor's name and prepare for YAML structure
        sensor_obj_handle_name = remove_namespace_prefix(sensor_obj_handle.name)
        sensor_yaml_name = self.add_sensor_prefix_str(sensor_obj_handle_name)
        sensor_data['name'] = sensor_obj_handle_name
        
        # sensor_data['parent'] = sensor_obj_handle.ambf_object_parent #TODO: Fix parent not set issue

        # Define location with position and orientation
        world_pos = sensor_obj_handle.matrix_world.translation
        world_rot = sensor_obj_handle.matrix_world.to_euler()
        sensor_data['location'] = {
            'position': {
                'x': ambf_round(world_pos.x),
                'y': ambf_round(world_pos.y),
                'z': ambf_round(world_pos.z)
            },
            'orientation': {
                'r': ambf_round(world_rot[0]),
                'p': ambf_round(world_rot[1]),
                'y': ambf_round(world_rot[2])
            }
        }

        # Set common attributes
        sensor_data['visible'] = sensor_obj_handle.ambf_object_visible
        sensor_data['visible size'] = sensor_obj_handle.ambf_object_visible_size

        # Add specific attributes based on the sensor type
        if sensor_type == 'Proximity':
            sensor_data['range'] = sensor_obj.ambf_sensor_range
            sensor_data['array'] = [
                {
                    'offset': {'x': ambf_round(item.offset[0]), 'y': ambf_round(item.offset[1]), 'z': ambf_round(item.offset[2])},
                    'direction': {'x': ambf_round(item.direction[0]), 'y': ambf_round(item.direction[1]), 'z': ambf_round(item.direction[2])}
                }
                for item in sensor_obj.ambf_sensor_array
            ]

        elif sensor_type == 'Resistance':
            sensor_data['friction'] = {
                'static': sensor_obj.ambf_sensor_friction_static,
                'damping': sensor_obj.ambf_sensor_friction_damping,
                'dynamic': sensor_obj.ambf_sensor_friction_dynamic,
                'variable': sensor_obj.ambf_sensor_friction_variable
            }
            sensor_data['contact area'] = sensor_obj.ambf_sensor_contact_area
            sensor_data['contact stiffness'] = sensor_obj.ambf_sensor_contact_stiffness
            sensor_data['contact damping'] = sensor_obj.ambf_sensor_contact_damping

        elif sensor_type == 'Contact':
            sensor_data['distance threshold'] = sensor_obj.ambf_sensor_distance_threshold
            sensor_data['process contact details'] = sensor_obj.ambf_sensor_process_contact_details

        # Add the sensor to the list and dictionary for the ADF structure
        self._sensor_names_list.append(sensor_yaml_name)
        adf_data[sensor_yaml_name] = sensor_data
    
    def generate_camera_data_from_ambf_camera(self, adf_data, camera_obj_handle):
        if camera_obj_handle.ambf_object_type != 'CAMERA':
            return
        
        # The object is unlinked from the scene. Don't write it
        if bpy.context.scene.objects.get(camera_obj_handle.name) is None:
            return

        if is_object_hidden(camera_obj_handle) is True:
            return

        print(f"Generating Camera {camera_obj_handle.name}")
        camera_template = CameraTemplate()
        camera_data = camera_template._adf_data

        if not compare_namespace_with_global(camera_obj_handle.name):
            if get_namespace(camera_obj_handle.name) != '':
                camera_data['namespace'] = get_namespace(camera_obj_handle.name)
        
        camera_obj_handle_name = remove_namespace_prefix(camera_obj_handle.name)
        camera_data['name'] = camera_obj_handle_name

        # Get camera properties
        world_pos = camera_obj_handle.matrix_world.translation
        world_rot = camera_obj_handle.matrix_world.to_euler()

        # Convert the Euler rotation to a rotation matrix
        rot_matrix = world_rot.to_matrix().to_4x4()
        # print(rot_matrix)

        # Define the default forward, right, and up vectors
        track_axis = camera_obj_handle.constraints.data.track_axis
        # print(f"Track Axis: {track_axis}")

        coef = 1
        if "NEG" in track_axis:
            coef = -1
        
        if "X" in track_axis:
            forward0 = coef * mathutils.Vector((1, 0, 0))
        elif "Y" in track_axis:
            forward0 = coef * mathutils.Vector((0, 1, 0))
        elif "Z" in track_axis:
            forward0 = coef * mathutils.Vector((0, 0, 1))

        up_axis = camera_obj_handle.constraints.data.up_axis  
        # print(f"Up Axis: {up_axis}")

        if "X" in up_axis:
            up0 = coef * mathutils.Vector((1, 0, 0))
        elif "Y" in up_axis:
            up0 = coef * mathutils.Vector((0, 1, 0))
        elif "Z" in up_axis:
            up0 = coef * mathutils.Vector((0, 0, 1))     

        print(forward0, up0)

        right0 = forward0.cross(up0)

        # Transform these directions by the rotation matrix
        forward = rot_matrix @ forward0
        right = rot_matrix @ right0
        up = rot_matrix @ up0        

        look_at = world_pos + forward
        right = rot_matrix @ right0
        up = forward.cross(right).normalized()

        camera_data['location'] = {'x': ambf_round(world_pos.x), 'y': ambf_round(world_pos.y), 'z': ambf_round(world_pos.z)}
        camera_data['look at'] = {'x': ambf_round(look_at[0]), 'y': ambf_round(look_at[1]), 'z': ambf_round(look_at[2])}
        camera_data['up'] = {'x': ambf_round(up[0]), 'y': ambf_round(up[1]), 'z': ambf_round(up[2])}

        camera_data['field view angle'] = camera_obj_handle.data.angle
        camera_data['clipping plane']['near'] = camera_obj_handle.data.clip_start
        camera_data['clipping plane']['far'] = camera_obj_handle.data.clip_end
        camera_data['monitor'] = camera_obj_handle.ambf_camera_monitor
        camera_data['publish image'] = camera_obj_handle.ambf_camera_publish_image
        camera_data['publish image interval'] = camera_obj_handle.ambf_camera_publish_image_interval
        camera_data['publish image resolution'] = {
            'width': camera_obj_handle.ambf_camera_publish_image_width,
            'height': camera_obj_handle.ambf_camera_publish_image_height
        }
        camera_data['publish depth'] = camera_obj_handle.ambf_camera_publish_depth
        camera_data['publish depth interval'] = camera_obj_handle.ambf_camera_publish_depth_interval
        camera_data['publish depth resolution'] = {
            'width': camera_obj_handle.ambf_camera_publish_depth_width,
            'height': camera_obj_handle.ambf_camera_publish_depth_height
        }

        if camera_obj_handle.ambf_camera_enable_intrinsics:
            camera_data['camera intrinsics'] = {
                'fx': ambf_round(camera_obj_handle.ambf_camera_intrinsics_fx),
                'fy': ambf_round(camera_obj_handle.ambf_camera_intrinsics_fy),
                'cx': ambf_round(camera_obj_handle.ambf_camera_intrinsics_cx),
                'cy': ambf_round(camera_obj_handle.ambf_camera_intrinsics_cy),
                's':  ambf_round(camera_obj_handle.ambf_camera_intrinsics_s)
            }

        if camera_obj_handle.ambf_camera_enable_projection_matrix:
            mat = camera_obj_handle.ambf_camera_projection_matrix
            camera_data['projection matrix'] = [
                [ambf_round(mat[r * 4 + c]) for c in range(4)]
                for r in range(4)
            ]

        camera_yaml_name = self.add_camera_prefix_str(camera_data['name'])
        adf_data[camera_yaml_name] = camera_data
        self._camera_names_list.append(camera_yaml_name)

    def generate_light_data_from_ambf_light(self, adf_data, light_obj_handle):
        if light_obj_handle.ambf_object_type != 'LIGHT':
            return
        
        # The object is unlinked from the scene. Don't write it
        if bpy.context.scene.objects.get(light_obj_handle.name) is None:
            return

        if is_object_hidden(light_obj_handle) is True:
            return

        print(f"Generating Light {light_obj_handle.name}")
        light_template = LightTemplate()
        light_data = light_template._adf_data

        if not compare_namespace_with_global(light_obj_handle.name):
            if get_namespace(light_obj_handle.name) == '':
                light_data['namespace'] = light_obj_handle.ambf_light_namespace
            else:
                light_data['namespace'] = get_namespace(light_obj_handle.name)

        light_obj_handle_name = remove_namespace_prefix(light_obj_handle.name)
        light_data['name'] = light_obj_handle_name
        light_data['type'] = light_obj_handle.ambf_light_type

        # Get light position and orientation
        world_pos = light_obj_handle.matrix_world.translation
        world_rot = light_obj_handle.matrix_world.to_euler()

        light_data['location'] = {'x': ambf_round(world_pos.x), 'y': ambf_round(world_pos.y), 'z': ambf_round(world_pos.z)}
        light_data['direction'] = {'x': ambf_round(world_rot[0]), 'y': ambf_round(world_rot[1]), 'z': ambf_round(world_rot[2])}

        light_data['shadow quality'] = light_obj_handle.ambf_light_shadow_quality
        if light_obj_handle.ambf_light_type == 'SPOT':
            light_data['spot exponent'] = light_obj_handle.ambf_light_spot_exponent
            light_data['cutoff angle'] = light_obj_handle.ambf_light_cutoff_angle
        light_data['attenuation'] = {
            'constant':  ambf_round(light_obj_handle.ambf_light_attenuation_constant),
            'linear':    ambf_round(light_obj_handle.ambf_light_attenuation_linear),
            'quadratic': ambf_round(light_obj_handle.ambf_light_attenuation_quadratic)
        }
        if light_obj_handle.ambf_light_parent:
            light_data['parent'] = light_obj_handle.ambf_light_parent

        light_yaml_name = self.add_light_prefix_str(light_data['name'])
        adf_data[light_yaml_name] = light_data
        self._light_names_list.append(light_yaml_name)

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

    
    def generate_adf(self, context):
        helper = self

        print('\n######### GENERATE ADF #########')

        num_objs_unhidden = len([obj for obj in context.scene.objects if not obj.hide_get()])
        print('Number of unhidden objects in the scene: ', num_objs_unhidden)

        save_to = bpy.path.abspath(context.scene.ambf_adf_path)
        filename = os.path.basename(save_to)
        save_dir = os.path.dirname(save_to)
        if not filename:
            filename = 'default.yaml'
        output_filename = os.path.join(save_dir, filename)

        os.makedirs(save_dir, exist_ok=True)

        if os.path.isfile(output_filename):
            os.rename(output_filename, output_filename + '.old')

        output_file = open(output_filename, 'w')
        print('Output filename is: ', output_filename)

        self._adf = OrderedDict()

        self._adf['bodies'] = []
        self._adf['ghost objects'] = []
        self._adf['soft bodies'] = []
        self._adf['joints'] = []
        self._adf['cameras'] = []
        self._adf['lights'] = []
        self._adf['sensors'] = []
        self._adf['actuators'] = []

        mesh_root_abs = bpy.path.abspath(context.scene.ambf_meshes_path)
        save_dir_abs = bpy.path.abspath(save_dir)

        if not mesh_root_abs:
            mesh_root_abs = save_dir_abs

        os.makedirs(mesh_root_abs, exist_ok=True)
        os.makedirs(os.path.join(mesh_root_abs, "high_res"), exist_ok=True)
        os.makedirs(os.path.join(mesh_root_abs, "low_res"), exist_ok=True)

        rel_mesh_path = os.path.relpath(mesh_root_abs, save_dir_abs)
        if rel_mesh_path == ".":
            rel_mesh_path = ""

        self._adf['high resolution path'] = (rel_mesh_path + "/high_res/") if rel_mesh_path else "high_res/"
        self._adf['low resolution path'] = (rel_mesh_path + "/low_res/") if rel_mesh_path else "low_res/"

        self._adf['ignore inter-collision'] = context.scene.ambf_ignore_inter_collision

        if context.scene.ambf_model_override_gravity:
            self._adf['gravity'] = {'x': context.scene.ambf_model_gravity[0],
                                'y': context.scene.ambf_model_gravity[1],
                                'z': context.scene.ambf_model_gravity[2]}

        update_global_namespace(context)

        if CommonConfig.namespace != "":
            self._adf['namespace'] = CommonConfig.namespace

        _heirarichal_objects_list = populate_heirarchial_tree()

        for obj_handle in _heirarichal_objects_list:
            self.generate_body_data_from_ambf_rigid_body(self._adf, obj_handle)
            self.generate_body_data_from_ambf_ghost_object(self._adf, obj_handle)
            self.generate_body_data_from_ambf_soft_body(self._adf, obj_handle)
            self.generate_joint_data_from_ambf_constraint(self._adf, obj_handle)
            self.generate_camera_data_from_ambf_camera(self._adf, obj_handle)
            self.generate_light_data_from_ambf_light(self._adf, obj_handle)
            self.generate_sensor_data_from_ambf_sensor(self._adf, obj_handle)
            self.generate_actuator_data_from_ambf_actuator(self._adf, obj_handle)

        self._adf['bodies'] = self._body_names_list
        self._adf['ghost objects'] = self._ghost_object_names_list
        self._adf['soft bodies'] = self._soft_body_names_list
        self._adf['joints'] = self._joint_names_list
        self._adf['cameras'] = self._camera_names_list
        self._adf['lights'] = self._light_names_list
        self._adf['sensors'] = self._sensor_names_list
        self._adf['actuators'] = self._actuator_names_list

        yaml.dump(self._adf, output_file)

        header_str = "# AMBF Version: %s\n" \
                    "# Generated By: ambf_addon for Blender %s\n" \
                    "# Author: %s\n" \
                    "# Generated on: %s\n" \
                    % (str(bl_info['version']).replace(', ', '.'),
                        str(bl_info['blender']).replace(', ', '.'),
                        bl_info['author'],
                        datetime.now().strftime('%Y-%m-%d %H:%M:%S'))

        prepend_comment_to_file(output_filename, header_str)

        # Auto-link saved file to the launch field for quick access
        context.scene.ambf_launch_adf_path = output_filename


class AMBF_OT_save_meshes(Operator):#
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
    def set_to_origin(self, p_obj_handle, original_obj_poses_map):
        if p_obj_handle.children is None:
            return
        original_obj_poses_map[p_obj_handle.name] = p_obj_handle.matrix_world.copy()
        # Since setting the world transform clears the embedded scale
        # of the object, we need to re-scale the obj_handle after putting it to origin
        scale_mat = mathutils.Matrix()
        scale_mat = scale_mat.Scale(p_obj_handle.matrix_world.median_scale, 4)
        p_obj_handle.matrix_world.identity()
        p_obj_handle.matrix_world = scale_mat
        for c_obj_handle in p_obj_handle.children:
            self.set_to_origin(c_obj_handle, original_obj_poses_map)

    # Since Blender exports meshes w.r.t world transform and not the
    # the local mesh transform, we explicitly push each obj_handle to origin
    # and remember its world transform for putting it back later on
    def set_all_meshes_to_origin(self):
        original_obj_poses_map = dict()
        for p_obj_handle in bpy.data.objects:
            if p_obj_handle.parent is None:
                self.set_to_origin(p_obj_handle, original_obj_poses_map)
        return original_obj_poses_map

    # This recursive function works in similar fashion to the
    # set_to_origin function, but uses the know default transform
    # to set the tree back to default in a hierarchial fashion
    def reset_back_to_default(self, p_obj_handle, original_obj_poses_map):
        if p_obj_handle.children is None:
            return
        for key in original_obj_poses_map:
            if p_obj_handle.name == key:
                original_pose = original_obj_poses_map[key]
                p_obj_handle.matrix_world = original_pose
                p_obj_handle.rotation_euler = original_pose.to_euler().copy()
        for c_obj_handle in p_obj_handle.children:
            self.reset_back_to_default(c_obj_handle, original_obj_poses_map)

    def reset_meshes_to_original_position(self, original_obj_poses_map):
        for p_obj_handle in bpy.data.objects:
            if p_obj_handle.parent is None:
                self.reset_back_to_default(p_obj_handle, original_obj_poses_map)

    def save_body_textures(self, context, obj_handle, high_res_path):
        print("Saving Textures for Object: ", obj_handle.name)
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
        if obj_handle.ambf_object_type == 'RIGID_BODY' or obj_handle.ambf_object_type == 'SOFT_BODY' or obj_handle.ambf_object_type == 'GHOST_OBJECT':
            obj_handle_name = remove_namespace_prefix(obj_handle.name)

            if obj_handle.type == 'MESH':
                print("Saving Mesh for Body: ", obj_handle.name)
                # SAVE HIGH RES / VISUAL MESHES FIRST
                if context.scene.ambf_save_high_res:
                    print("Saving High Res Mesh for Object: ", obj_handle.name)
                    filename_high_res = os.path.join(high_res_path, obj_handle_name)
                    save_blender_mesh(obj_handle, filename_high_res, mesh_type, False)

                # NOW SAVE LOW RES MESHES
                if context.scene.ambf_save_low_res:
                    print("Saving Low Res Mesh for Object: ", obj_handle.name)
                    if obj_handle.ambf_use_separate_collision_mesh:
                        if obj_handle.ambf_collision_mesh:
                            coll_mesh_name = remove_namespace_prefix(obj_handle.ambf_collision_mesh.name)
                            filename_low_res = os.path.join(low_res_path, coll_mesh_name)
                            save_blender_mesh(obj_handle.ambf_collision_mesh, filename_low_res, mesh_type, True)
                        else:
                            print("ERROR! NO SEPARATE COLLISION MESH SET FOR OBJECT: ", obj_handle.name)
                    else:
                        filename_low_res = os.path.join(low_res_path, obj_handle_name)
                        save_blender_mesh(obj_handle, filename_low_res, mesh_type, True)
        else:
            print("Object Type is not Body, Skipping Mesh Save")
            return

    def save_meshes(self, context):
        print("\nSaving Meshes......")
        # Get the list of currently selected objects
        if context.scene.ambf_save_selection_only:
            objects_to_save = get_selected_objects()
            print("Saving Selected Objects Only", objects_to_save)
        else:
            print("Saving All Objects")
            objects_to_save = bpy.data.objects

        # Now deselect all objects
        select_all_objects(False)
        print("###### Done Deselecting Objects ######")

        save_path = bpy.path.abspath(context.scene.ambf_meshes_path)
        high_res_path = os.path.join(save_path, 'high_res/')
        low_res_path = os.path.join(save_path, 'low_res/')
        os.makedirs(high_res_path, exist_ok=True)
        os.makedirs(low_res_path, exist_ok=True)
        mesh_type = bpy.context.scene.ambf_meshes_save_type
        
        print(f"Save to: {save_path} with mesh type: {mesh_type}")

        original_obj_poses_map = self.set_all_meshes_to_origin()
        for obj_handle in objects_to_save:
            self.save_body_textures(context, obj_handle, high_res_path)
            self.save_body_meshes(context, obj_handle, mesh_type, high_res_path, low_res_path)
        self.reset_meshes_to_original_position(original_obj_poses_map)

        # Now reselect the objects that we selected prior to saving meshes
        select_objects(objects_to_save, True)
        print("###### Done Reselecting Saving Objects ######")


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
        active_obj_handle.ambf_object_type = 'CONSTRAINT'
        return {'FINISHED CREATING JOINT'}
    
class AMBF_OT_create_sensor(Operator):
    bl_idname = "ambf.create_sensor"
    bl_label = "Create Sensor"
    bl_description = "This creates an empty object that can be used to create closed loop mechanisms. Make" \
                     " sure to set the rigid body constraint (RBC) for this empty mesh and ideally parent this empty" \
                     " object with the parent body of its RBC"

    def execute(self, context):
        select_all_objects(False)
        bpy.ops.object.empty_add(type='SPHERE')
        active_obj_handle = get_active_object()
        active_obj_handle.name = 'sensor'
        active_obj_handle.ambf_object_type = 'SENSOR'
        return {'FINISHED'}
    
class AMBF_OT_add_sensor_array_item(Operator):
    bl_idname = "ambf.add_sensor_array_item"
    bl_label = "Add Sensor Array Item"

    def execute(self, context):
        obj = context.object

        # Check if the object has the 'ambf_sensor_array' collection property
        if not hasattr(obj.ambf_sensor_properties, 'ambf_sensor_array'):
            self.report({'ERROR'}, "The object doesn't have an ambf_sensor_array property.")
            return {'CANCELLED'}

        print("Adding Sensor Array Item")
        new_item = obj.ambf_sensor_properties.ambf_sensor_array.add()
        new_item.offset = (0.0, 0.0, 0.0)
        new_item.direction = (0.0, -1.0, 0.0)

        return {'FINISHED'}
    
class AMBF_OT_remove_sensor_array_item(Operator):
    bl_idname = "ambf.remove_sensor_array_item"
    bl_label = "Remove Sensor Array Item"

    index: IntProperty(default=0)

    def execute(self, context):
        obj = context.object

        # Check if the object has the 'ambf_sensor_array' collection property
        if not hasattr(obj, 'ambf_sensor_array'):
            self.report({'ERROR'}, "The object doesn't have an ambf_sensor_array property.")
            return {'CANCELLED'}

        obj.ambf_sensor_properties.ambf_sensor_array.remove(self.index)

        return {'FINISHED'}
    
class AMBF_OT_create_actuator(Operator):
    bl_idname = "ambf.create_actuator"
    bl_label = "Create Actuator"
    bl_description = (
        "This creates an empty object that can be used to create closed loop mechanisms. Make"
        " sure to set the rigid body constraint (RBC) for this empty mesh and ideally parent this empty"
        " object with the parent body of its RBC"
    )

    def execute(self, context):
        select_all_objects(False)
        bpy.ops.object.empty_add(type='SPHERE')
        active_obj_handle = get_active_object()
        active_obj_handle.name = 'actuator'
        active_obj_handle.ambf_object_type = 'ACTUATOR'
        return {'FINISHED'} 


class AMBF_OT_create_camera(Operator):
    bl_idname = "ambf.create_camera"
    bl_label = "Create Camera"
    bl_description = "Creates a new Blender camera object and marks it as an AMBF camera"

    def execute(self, context):
        select_all_objects(False)
        bpy.ops.object.camera_add()
        active_obj_handle = get_active_object()
        active_obj_handle.name = 'camera'
        active_obj_handle.ambf_object_type = 'CAMERA'
        return {'FINISHED'}


class AMBF_OT_create_light(Operator):
    bl_idname = "ambf.create_light"
    bl_label = "Create Light"
    bl_description = "Creates a new Blender spot light object and marks it as an AMBF light"

    def execute(self, context):
        select_all_objects(False)
        bpy.ops.object.light_add(type='SPOT')
        active_obj_handle = get_active_object()
        active_obj_handle.name = 'light'
        active_obj_handle.ambf_object_type = 'LIGHT'
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

# TODO: decide if soft bodies should be supported
class AMBF_OT_estimate_inertial_offsets(Operator):
    bl_idname = "ambf.estimate_inertial_offsets"
    bl_label = "Estimate Inertial Offsets"
    bl_description = "Automatically Estimate the Inertial Offsets for the Bodies"

    def execute(self, context):
        for obj_handle in bpy.data.objects:
            if obj_handle.ambf_object_type == 'RIGID_BODY' and obj_handle.type == 'MESH':
                local_com = compute_local_com(obj_handle)
                obj_handle.ambf_body_linear_inertial_offset[0] = local_com[0]
                obj_handle.ambf_body_linear_inertial_offset[1] = local_com[1]
                obj_handle.ambf_body_linear_inertial_offset[2] = local_com[2]
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

# TODO: decide if soft bodies should be supported
class AMBF_OT_estimate_inertial_offset_per_object(Operator):
    bl_idname = "ambf.estimate_inertial_offset_per_object"
    bl_label = "Estimate Inertial Offset"
    bl_description = "Automatically Estimate the Inertial Offsets for the Bodies"

    def execute(self, context):
        obj_handle = context.object
        if obj_handle.ambf_object_type == 'RIGID_BODY' and obj_handle.type == 'MESH':
            local_com = compute_local_com(obj_handle)
            obj_handle.ambf_body_linear_inertial_offset[0] = local_com[0]
            obj_handle.ambf_body_linear_inertial_offset[1] = local_com[1]
            obj_handle.ambf_body_linear_inertial_offset[2] = local_com[2]
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

class AMBF_OT_launch_ambf_simulator(Operator):
    bl_idname = "ambf.launch_simulator"
    bl_label = "Launch in AMBF Simulator"
    bl_description = "Open the selected ADF file directly in the AMBF simulator"

    def execute(self, context):
        import subprocess, os

        adf_path = bpy.path.abspath(context.scene.ambf_launch_adf_path)
        simulator_path = bpy.path.abspath(context.scene.ambf_simulator_path)

        if not adf_path:
            self.report({'ERROR'}, "No ADF file selected. Set the 'ADF to Launch' path first.")
            return {'CANCELLED'}

        if not os.path.isfile(adf_path):
            self.report({'ERROR'}, f"ADF file not found: {adf_path}")
            return {'CANCELLED'}

        if not simulator_path:
            self.report({'ERROR'}, "No simulator path set. Fill in the Simulator Path field.")
            return {'CANCELLED'}

        if not os.path.isfile(simulator_path):
            self.report({'ERROR'}, f"Simulator not found at: {simulator_path}")
            return {'CANCELLED'}

        try:
            subprocess.Popen([simulator_path, '--a', adf_path])
            self.report({'INFO'}, f"Launched AMBF simulator with: {adf_path}")
        except Exception as e:
            self.report({'ERROR'}, f"Failed to launch simulator: {e}")
            return {'CANCELLED'}

        return {'FINISHED'}


class AMBF_OT_load_ambf_file(Operator):
    bl_idname = "ambf.load_ambf_file"
    bl_label = "Load AMBF Description File (ADF)"
    bl_description = "This loads an AMBF from the specified config file"


    


    # def __init__(self, *args, **kwargs):
    #     self._adf_data = None
    #     self._joint_additional_offset = {}
    #     self._blender_remapped_body_names = {}
    #     self.loaded_cameras = {}
    #     self._high_res_path = ''
    #     self._low_res_path = ''
    #     bpy.context = None
    #     self._yaml_filepath = ''

    def get_qualified_path(self, path):
        filepath = Path(path)

        if filepath.is_absolute():
            return path
        else:
            ambf_filepath = Path(self._yaml_filepath)
            path = str(ambf_filepath.parent.joinpath(filepath))
            return path

    def load_ambf_mesh(self, body_data, body_id):
        print('Loading Mesh for Body: ', body_data['name'])

        body_name = body_data['name']

        try:
            body_high_res_path = self.get_qualified_path(body_data['high resolution path'])
        except:
            body_high_res_path = self._high_res_path

        try:
            body_low_res_path = self.get_qualified_path(body_data['low resolution path'])
        except:
            body_low_res_path = self._low_res_path

        # If body name is world. Check if a world body has already
        # been defined, and if it has been, ignore adding another world body
        if body_name in ['world', 'World', 'WORLD']:
            for temp_obj_handle in bpy.data.objects:
                if temp_obj_handle.type in ['MESH', 'EMPTY']:
                    if temp_obj_handle.name in ['world', 'World', 'WORLD']:
                        self._blender_remapped_body_names[body_id] = temp_obj_handle.name
                        return

        coll_mesh_obj = None
        # If a collision mesh is specified, load it as well
        if 'collision mesh' in body_data:
            collision_mesh_name = body_data['collision mesh']
            collision_mesh_filepath = Path(os.path.join(body_low_res_path, collision_mesh_name))
            load_blender_mesh(bpy.context, collision_mesh_filepath, collision_mesh_name)
            coll_mesh_obj = get_active_object()
            self._blender_remapped_body_names[collision_mesh_name] = coll_mesh_obj.name

        body_mesh_name = body_data['mesh']

        mesh_filepath = Path(os.path.join(body_high_res_path, body_mesh_name))

        load_blender_mesh(bpy.context, mesh_filepath, body_name)

        print('Loaded Mesh: ', body_name)
        mesh_obj = get_active_object()
        print('Mesh Object: ', mesh_obj)

        if coll_mesh_obj:
            print('Collision Mesh Object: ', coll_mesh_obj)
            make_obj1_parent_of_obj2(mesh_obj, coll_mesh_obj)
        return mesh_obj

    def load_ambf_camera(self, camera_data):
        camera_name = camera_data['name']
        
        # Create a new camera data block
        camera = bpy.data.cameras.new(name=camera_name)
        camera.angle = camera_data['field view angle']
        camera.clip_start = camera_data['clipping plane']['near']
        camera.clip_end = camera_data['clipping plane']['far']

        # Create a new object with the camera data
        camera_object = bpy.data.objects.new(camera_name, camera)
        camera_object.ambf_object_type = 'CAMERA'
        
        # Set the camera's location
        location_vec = mathutils.Vector((
            camera_data['location']['x'], 
            camera_data['location']['y'], 
            camera_data['location']['z']
        ))

        camera_object.location = location_vec
        
        # Calculate the forward, right, and up vectors in world coordinates
        look_at_vec = mathutils.Vector((
            camera_data['look at']['x'], 
            camera_data['look at']['y'], 
            camera_data['look at']['z']
        ))

        up_world = mathutils.Vector((
            camera_data['up']['x'], 
            camera_data['up']['y'], 
            camera_data['up']['z']
        )).normalized()
        
        # Forward vector from the camera to the look-at point
        forward_world = (look_at_vec - location_vec).normalized()
        
        # Right vector as the cross product of forward and up
        right_world = forward_world.cross(up_world).normalized()
        
        # Recompute the up vector as cross product of right and forward to ensure orthogonality
        up_world = right_world.cross(forward_world).normalized()
        
        # Retrieve track_axis and up_axis from camera_data (you need to store these during generation)
        track_axis = 'NEG_Z'
        up_axis = 'NEG_Y'
        
        coef = -1 if "NEG" in track_axis else 1
        
        # Define the default forward0 vector based on track_axis
        if "X" in track_axis:
            forward0 = coef * mathutils.Vector((1, 0, 0))
        elif "Y" in track_axis:
            forward0 = coef * mathutils.Vector((0, 1, 0))
        elif "Z" in track_axis:
            forward0 = coef * mathutils.Vector((0, 0, 1))
        else:
            forward0 = coef * mathutils.Vector((0, 0, 1)) 
        
        # Define the default up0 vector based on up_axis
        if "X" in up_axis:
            up0 = mathutils.Vector((1, 0, 0))
        elif "Y" in up_axis:
            up0 = mathutils.Vector((0, 1, 0))
        elif "Z" in up_axis:
            up0 = mathutils.Vector((0, 0, 1))
        else:
            up0 = mathutils.Vector((0, 1, 0))  
        
        # Compute the local right0 vector
        right0 = forward0.cross(up0).normalized()
        
        # Build the local axes matrix (columns are right0, up0, forward0)
        M_local = mathutils.Matrix((right0, up0, forward0)).transposed()
        
        # Build the world axes matrix (columns are right_world, up_world, forward_world)
        M_world = mathutils.Matrix((right_world, up_world, forward_world)).transposed()
        
        # Compute the rotation matrix that aligns the local axes to the world axes
        rot_matrix = M_world @ M_local.inverted() #TODO: This is not correct, need to fix
        
        # Apply the rotation matrix to the camera object
        camera_object.matrix_world = mathutils.Matrix.Translation(location_vec) @ rot_matrix.to_4x4()
        
        # Link the camera object to the current scene
        scene = bpy.context.scene
        scene.collection.objects.link(camera_object)
        
        # Optionally set the camera as the active camera for the scene
        scene.camera = camera_object

        # Load AMBF-specific camera properties
        camera_object.ambf_camera_clip_start = camera_data['clipping plane']['near']
        camera_object.ambf_camera_clip_end = camera_data['clipping plane']['far']
        camera_object.ambf_camera_monitor = camera_data.get('monitor', 0)
        camera_object.ambf_camera_publish_image = camera_data.get('publish image', False)
        camera_object.ambf_camera_publish_image_interval = camera_data.get('publish image interval', 1)
        img_res = camera_data.get('publish image resolution', {})
        camera_object.ambf_camera_publish_image_width = img_res.get('width', 640)
        camera_object.ambf_camera_publish_image_height = img_res.get('height', 480)
        camera_object.ambf_camera_publish_depth = camera_data.get('publish depth', False)
        camera_object.ambf_camera_publish_depth_interval = camera_data.get('publish depth interval', 1)
        depth_res = camera_data.get('publish depth resolution', {})
        camera_object.ambf_camera_publish_depth_width = depth_res.get('width', 640)
        camera_object.ambf_camera_publish_depth_height = depth_res.get('height', 480)

        if 'camera intrinsics' in camera_data:
            intr = camera_data['camera intrinsics']
            camera_object.ambf_camera_enable_intrinsics = True
            camera_object.ambf_camera_intrinsics_fx = float(intr.get('fx', 500.0))
            camera_object.ambf_camera_intrinsics_fy = float(intr.get('fy', 500.0))
            camera_object.ambf_camera_intrinsics_cx = float(intr.get('cx', 320.0))
            camera_object.ambf_camera_intrinsics_cy = float(intr.get('cy', 240.0))
            camera_object.ambf_camera_intrinsics_s  = float(intr.get('s', 0.0))

        if 'projection matrix' in camera_data:
            flat = [camera_data['projection matrix'][r][c]
                    for r in range(4) for c in range(4)]
            camera_object.ambf_camera_enable_projection_matrix = True
            camera_object.ambf_camera_projection_matrix = flat

        return camera_object

    def load_ambf_light(self, light_data):
        light_name = light_data['name']

        # Determine Blender light type from AMBF type
        ambf_light_type = light_data.get('type', 'SPOT').upper()
        blender_type_map = {'POINT': 'POINT', 'SPOT': 'SPOT', 'DIRECTIONAL': 'SUN'}
        blender_light_type = blender_type_map.get(ambf_light_type, 'SPOT')

        # Create a new light data block
        light = bpy.data.lights.new(name=light_name, type=blender_light_type)
                
        # Create a new light object with the light data
        light_object = bpy.data.objects.new(light_name, light)
        light_object.ambf_object_type = 'LIGHT'

        light_object.ambf_light_cutoff_angle = light_data.get('cutoff angle', 1.7)
        light_object.ambf_light_spot_exponent = light_data.get('spot exponent', 1.0)
        light_object.ambf_light_shadow_quality = light_data.get('shadow quality', 1)
        light_object.ambf_light_type = 'SPOT'

        if 'attenuation' in light_data:
            atten = light_data['attenuation']
            light_object.ambf_light_attenuation_constant  = float(atten.get('constant',  1.0))
            light_object.ambf_light_attenuation_linear    = float(atten.get('linear',    0.0))
            light_object.ambf_light_attenuation_quadratic = float(atten.get('quadratic', 0.0))

        if 'parent' in light_data and light_data['parent']:
            light_object.ambf_light_parent = light_data['parent']

        # Set the position and rotation of the light
        light_object.location = (
            light_data['location']['x'],
            light_data['location']['y'],
            light_data['location']['z']
        )

        # Set rotation from direction in `light_data`
        light_object.rotation_euler = (
            light_data['direction']['x'],
            light_data['direction']['y'],
            light_data['direction']['z']
        )

        # Set the namespace if provided
        if 'namespace' in light_data:
            light_object.ambf_light_namespace = light_data['namespace']

        # Link the light object to the current scene
        scene = bpy.context.scene
        scene.collection.objects.link(light_object)

        return light_object

    def load_ambf_actuator(self, actuator_data):
        actuator_name = actuator_data['name']
        
        actuator = bpy.data.objects.new(name=actuator_name, object_data=None)
        actuator.empty_display_type = 'SPHERE' 
        actuator.ambf_object_type = 'ACTUATOR'

        actuator.location = (
            actuator_data['location']['position']['x'],
            actuator_data['location']['position']['y'],
            actuator_data['location']['position']['z']
        )

        actuator.rotation_euler = (
            actuator_data['location']['orientation']['r'],
            actuator_data['location']['orientation']['p'],
            actuator_data['location']['orientation']['y']
        )

        # if 'namespace' in actuator_data:
        #     actuator.ambf_actuator_namespace = actuator_data['namespace']
        if 'parent' in actuator_data:
            parent_obj_handle = bpy.data.objects[actuator_data['parent']]
            actuator.ambf_object_parent = parent_obj_handle
        if 'visible' in actuator_data:
            actuator.ambf_object_visible = actuator_data['visible']
        if 'visible size' in actuator_data:
            actuator.ambf_object_visible_size = actuator_data['visible size']

        scene = bpy.context.scene
        scene.collection.objects.link(actuator)

        return actuator
    
    def load_ambf_sensor(self, sensor_data):
        sensor_name = sensor_data['name']
        
        # Create a new sensor object
        sensor = bpy.data.objects.new(name=sensor_name, object_data=None)
        sensor.empty_display_type = 'SPHERE' 
        sensor.ambf_object_type = 'SENSOR'

        # Set location and rotation
        sensor.location = (
            sensor_data['location']['position']['x'],
            sensor_data['location']['position']['y'],
            sensor_data['location']['position']['z']
        )

        sensor.rotation_euler = (
            sensor_data['location']['orientation']['r'],
            sensor_data['location']['orientation']['p'],
            sensor_data['location']['orientation']['y']
        )

        if 'parent' in sensor_data:
            parent_obj_handle = bpy.data.objects[sensor_data['parent']]
            sensor.ambf_object_parent = parent_obj_handle
        if 'visible' in sensor_data:
            sensor.ambf_object_visible = sensor_data['visible']
        if 'visible size' in sensor_data:
            sensor.ambf_object_visible_size = sensor_data['visible size']

        # Check sensor type and load relevant properties
        sensor_type = sensor.ambf_sensor_properties.ambf_sensor_type
        if sensor_type == 'Proximity':
            if 'range' in sensor_data:
                sensor.ambf_sensor_properties.ambf_sensor_range = sensor_data['range']
            
            # Populate the array data
            if 'array' in sensor_data:
                # Clear any existing items in the array
                sensor.ambf_sensor_properties.ambf_sensor_array.clear()
                for item in sensor_data['array']:
                    new_item = sensor.ambf_sensor_properties.ambf_sensor_array.add()
                    new_item.offset = (item['offset']['x'], item['offset']['y'], item['offset']['z'])
                    new_item.direction = (item['direction']['x'], item['direction']['y'], item['direction']['z'])

        elif sensor_type == 'Resistance':
            if 'friction' in sensor_data:
                if 'damping' in sensor_data['friction']:
                    sensor.ambf_sensor_properties.ambf_sensor_friction_damping = sensor_data['friction']['damping']
                if 'static' in sensor_data['friction']:
                    sensor.ambf_sensor_properties.ambf_sensor_friction_static = sensor_data['friction']['static']
                if 'dynamic' in sensor_data['friction']:
                    sensor.ambf_sensor_properties.ambf_sensor_friction_dynamic = sensor_data['friction']['dynamic']
                if 'variable' in sensor_data['friction']:
                    sensor.ambf_sensor_properties.ambf_sensor_friction_variable = sensor_data['friction']['variable']
            if 'contact area' in sensor_data:
                sensor.ambf_sensor_properties.ambf_sensor_contact_area = sensor_data['contact area']
            if 'contact stiffness' in sensor_data:
                sensor.ambf_sensor_properties.ambf_sensor_contact_stiffness = sensor_data['contact stiffness']
            if 'contact damping' in sensor_data:
                sensor.ambf_sensor_properties.ambf_sensor_contact_damping = sensor_data['contact damping']

        elif sensor_type == 'Contact':
            if 'distance threshold' in sensor_data:
                sensor.ambf_sensor_properties.ambf_sensor_distance_threshold = sensor_data['distance threshold']
            if 'process contact details' in sensor_data:
                sensor.ambf_sensor_properties.ambf_sensor_process_contact_details = sensor_data['process contact details']

        # Link the sensor object to the current scene
        scene = bpy.context.scene
        scene.collection.objects.link(sensor)

        return sensor

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

    def load_ambf_ghost_body(self, body_data, obj_handle):
        obj_handle.ambf_object_type = 'GHOST_OBJECT'

        if 'name' in body_data:
            obj_handle.name = body_data['name']

        if 'parent' in body_data and body_data['parent']:
            obj_handle.ambf_object_parent = body_data['parent']

        if 'shape' in body_data:
            obj_handle.ambf_ghost_shape = body_data['shape']

        if 'geometry' in body_data:
            geometry = body_data['geometry']
            obj_handle.dimensions[0] = geometry.get('x', 1.0)
            obj_handle.dimensions[1] = geometry.get('y', 1.0)
            obj_handle.dimensions[2] = geometry.get('z', 1.0)

        if 'location' in body_data:
            location = body_data['location']
            if 'position' in location:
                position = location['position']
                obj_handle.location.x = position.get('x', 0.0)
                obj_handle.location.y = position.get('y', 0.0)
                obj_handle.location.z = position.get('z', 0.0)
            if 'orientation' in location:
                orientation = location['orientation']
                obj_handle.rotation_euler.x = orientation.get('r', 0.0)
                obj_handle.rotation_euler.y = orientation.get('p', 0.0)
                obj_handle.rotation_euler.z = orientation.get('y', 0.0)

        if 'scale' in body_data:
            obj_handle.ambf_scale = body_data['scale']

        if 'passive' in body_data:
            obj_handle.ambf_body_passive = body_data['passive']

        if 'transparency' in body_data:
            obj_handle.ambf_body_transparency = body_data['transparency']

        if 'color components' in body_data:
            color_components = body_data['color components']
            if 'ambient' in color_components:
                ambient = color_components['ambient']
                obj_handle.ambf_object_ambient_level = ambient.get('level', 1.0)
            if 'diffuse' in color_components:
                diffuse = color_components['diffuse']
                obj_handle.ambf_object_diffuse_color[0] = diffuse.get('r', 0.5)
                obj_handle.ambf_object_diffuse_color[1] = diffuse.get('g', 0.5)
                obj_handle.ambf_object_diffuse_color[2] = diffuse.get('b', 0.5)
            if 'specular' in color_components:
                specular = color_components['specular']
                obj_handle.ambf_object_specular_color[0] = specular.get('r', 0.5)
                obj_handle.ambf_object_specular_color[1] = specular.get('g', 0.5)
                obj_handle.ambf_object_specular_color[2] = specular.get('b', 0.5)

        if 'collision mesh type' in body_data:
            obj_handle.ambf_collision_mesh_type = body_data['collision mesh type']
        if 'collision margin' in body_data:
            obj_handle.ambf_collision_margin = body_data['collision margin']
            obj_handle.ambf_collision_margin_enable = True 
        if 'collision shape' in body_data:
            obj_handle.ambf_collision_shape = body_data['collision shape']
            obj_handle.ambf_collision_type = 'SINGULAR_SHAPE'  

        if 'collision geometry' in body_data:
            collision_geometry = body_data['collision geometry']
            obj_handle.ambf_ghost_collision_geometry[0] = collision_geometry.get('x', 1.0)
            obj_handle.ambf_ghost_collision_geometry[1] = collision_geometry.get('y', 1.0)
            obj_handle.ambf_ghost_collision_geometry[2] = collision_geometry.get('z', 1.0)

        if 'collision group' in body_data:
            collision_group = body_data['collision group']
            for i in range(len(obj_handle.ambf_collision_groups)):
                obj_handle.ambf_collision_groups[i] = False
            for group in collision_group:
                if 0 <= group < len(obj_handle.ambf_collision_groups):
                    obj_handle.ambf_collision_groups[group] = True
                else:
                    print('WARNING: Collision group {} is out of bounds.'.format(group))

        if not obj_handle.ambf_collision_shape_prop_collection:
            add_collision_shape_property(obj_handle)

        ocs = obj_handle.ambf_collision_shape_prop_collection.items()[0][1]
        ocs.ambf_collision_shape = obj_handle.ambf_collision_shape

        if ocs.ambf_collision_shape == 'BOX':
            if 'collision geometry' in body_data:
                ocs.ambf_collision_shape_xyz_dims[0] = collision_geometry.get('x', 1.0)
                ocs.ambf_collision_shape_xyz_dims[1] = collision_geometry.get('y', 1.0)
                ocs.ambf_collision_shape_xyz_dims[2] = collision_geometry.get('z', 1.0)
        elif ocs.ambf_collision_shape == 'SPHERE':
            if 'collision geometry' in body_data:
                ocs.ambf_collision_shape_radius = collision_geometry.get('radius', 0.5)
        elif ocs.ambf_collision_shape in ['CYLINDER', 'CONE', 'CAPSULE']:
            if 'collision geometry' in body_data:
                ocs.ambf_collision_shape_radius = collision_geometry.get('radius', 0.5)
                ocs.ambf_collision_shape_height = collision_geometry.get('height', 1.0)
                ocs.ambf_collision_shape_axis = collision_geometry.get('axis', 'Z').upper()

        if 'collision offset' in body_data:
            cso = body_data['collision offset']
            if 'position' in cso:
                position_offset = cso['position']
                ocs.ambf_collision_shape_linear_offset[0] = position_offset.get('x', 0.0)
                ocs.ambf_collision_shape_linear_offset[1] = position_offset.get('y', 0.0)
                ocs.ambf_collision_shape_linear_offset[2] = position_offset.get('z', 0.0)
            if 'orientation' in cso:
                orientation_offset = cso['orientation']
                ocs.ambf_collision_shape_angular_offset[0] = orientation_offset.get('r', 0.0)
                ocs.ambf_collision_shape_angular_offset[1] = orientation_offset.get('p', 0.0)
                ocs.ambf_collision_shape_angular_offset[2] = orientation_offset.get('y', 0.0)
        else:
            ocs.ambf_collision_shape_linear_offset[0] = 0.0
            ocs.ambf_collision_shape_linear_offset[1] = 0.0
            ocs.ambf_collision_shape_linear_offset[2] = 0.0
            ocs.ambf_collision_shape_angular_offset[0] = 0.0
            ocs.ambf_collision_shape_angular_offset[1] = 0.0
            ocs.ambf_collision_shape_angular_offset[2] = 0.0

        collision_shape_create_visual(obj_handle, ocs)
        
    def load_ambf_rigid_body(self, body_data, obj_handle):
        if obj_handle.type in ['EMPTY', 'MESH']:
            obj_handle.ambf_body_mass = body_data['mass']

            if 'gravity' in body_data:
                obj_handle.ambf_object_override_gravity = True
                obj_handle.ambf_object_gravity[0] = body_data['gravity']['x']
                obj_handle.ambf_object_gravity[1] = body_data['gravity']['y']
                obj_handle.ambf_object_gravity[2] = body_data['gravity']['z']

            else:
                obj_handle.ambf_object_override_gravity = False

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
                obj_handle.ambf_body_linear_inertial_offset[0] = body_data['inertial offset']['position']['x']
                obj_handle.ambf_body_linear_inertial_offset[1] = body_data['inertial offset']['position']['y']
                obj_handle.ambf_body_linear_inertial_offset[2] = body_data['inertial offset']['position']['z']

                obj_handle.ambf_body_angular_inertial_offset[0] = body_data['inertial offset']['orientation']['r']
                obj_handle.ambf_body_angular_inertial_offset[1] = body_data['inertial offset']['orientation']['p']
                obj_handle.ambf_body_angular_inertial_offset[2] = body_data['inertial offset']['orientation']['y']

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
                    ocs.ambf_collision_shape_linear_offset[0] = obj_handle.ambf_body_linear_inertial_offset[0]
                    ocs.ambf_collision_shape_linear_offset[1] = obj_handle.ambf_body_linear_inertial_offset[1]
                    ocs.ambf_collision_shape_linear_offset[2] = obj_handle.ambf_body_linear_inertial_offset[2]

                    ocs.ambf_collision_shape_angular_offset[0] = obj_handle.ambf_body_angular_inertial_offset[0]
                    ocs.ambf_collision_shape_angular_offset[1] = obj_handle.ambf_body_angular_inertial_offset[1]
                    ocs.ambf_collision_shape_angular_offset[2] = obj_handle.ambf_body_angular_inertial_offset[2]
                
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
                # If a separate collision mesh was specified, enable it
                if 'collision mesh' in body_data:
                    obj_handle.ambf_use_separate_collision_mesh = True
                    obj_handle.ambf_collision_mesh = \
                        bpy.data.objects[self._blender_remapped_body_names[body_data['collision mesh']]]

                # Since the shape is neither a single or a compound shape, it is a mesh based collision.
                # Now figure out what type of collision mesh is used. (CONCAVE_MESH, CONVEX_MESH or CONVEX_HULL)
                if 'collision mesh type' in body_data:
                    if body_data['collision mesh type'] == '':
                        obj_handle.ambf_collision_mesh_type = 'CONCAVE_MESH'
                    else:
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
                obj_handle.ambf_body_passive = body_data['passive']

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

    def load_ambf_soft_body(self, body_data, obj_handle):
        print(f"Loading Soft Body: {body_data.get('name')}")

        # Set basic object properties
        obj_handle.ambf_object_type = 'SOFT_BODY'
        if 'name' in body_data:
            obj_handle.name = body_data['name']
        
        if 'mass' in body_data:
            obj_handle.ambf_body_mass = body_data['mass']
        
        if 'scale' in body_data:
            obj_handle.ambf_scale = body_data['scale']

        if 'location' in body_data:
            location = body_data['location']
            if 'position' in location:
                position = location['position']
                obj_handle.location.x = position.get('x', 0.0)
                obj_handle.location.y = position.get('y', 0.0)
                obj_handle.location.z = position.get('z', 0.0)
            if 'orientation' in location:
                orientation = location['orientation']
                obj_handle.rotation_euler.x = orientation.get('r', 0.0)
                obj_handle.rotation_euler.y = orientation.get('p', 0.0)
                obj_handle.rotation_euler.z = orientation.get('y', 0.0)

        # Set inertial offsets
        if 'inertial offset' in body_data:
            inertial_offset = body_data['inertial offset']
            if 'position' in inertial_offset:
                position_offset = inertial_offset['position']
                obj_handle.ambf_body_linear_inertial_offset[0] = position_offset.get('x', 0.0)
                obj_handle.ambf_body_linear_inertial_offset[1] = position_offset.get('y', 0.0)
                obj_handle.ambf_body_linear_inertial_offset[2] = position_offset.get('z', 0.0)
            if 'orientation' in inertial_offset:
                orientation_offset = inertial_offset['orientation']
                obj_handle.ambf_body_angular_inertial_offset[0] = orientation_offset.get('r', 0.0)
                obj_handle.ambf_body_angular_inertial_offset[1] = orientation_offset.get('p', 0.0)
                obj_handle.ambf_body_angular_inertial_offset[2] = orientation_offset.get('y', 0.0)

        if 'color components' in body_data:
            color_components = body_data['color components']
            if 'ambient' in color_components:
                ambient = color_components['ambient']
                obj_handle.ambf_object_ambient_level = ambient.get('level', 1.0)
            if 'diffuse' in color_components:
                diffuse = color_components['diffuse']
                obj_handle.ambf_object_diffuse_color[0] = diffuse.get('r', 0.5)
                obj_handle.ambf_object_diffuse_color[1] = diffuse.get('g', 0.5)
                obj_handle.ambf_object_diffuse_color[2] = diffuse.get('b', 0.5)
            if 'specular' in color_components:
                specular = color_components['specular']
                obj_handle.ambf_object_specular_color[0] = specular.get('r', 0.5)
                obj_handle.ambf_object_specular_color[1] = specular.get('g', 0.5)
                obj_handle.ambf_object_specular_color[2] = specular.get('b', 0.5)

        # Set soft body configuration
        config = body_data.get('config', {})
        for config_key, enable_attr, value_attr in [
            ('kLST', 'ambf_soft_body_enable_linear_stiffness', 'ambf_soft_body_linear_stiffness'),
            ('kAST', 'ambf_soft_body_enable_angular_stiffness', 'ambf_soft_body_angular_stiffness'),
            ('kVST', 'ambf_soft_body_enable_volume_stiffness', 'ambf_soft_body_volume_stiffness'),
            ('kVCF', 'ambf_soft_body_enable_damping', 'ambf_soft_body_velocity_damping'),
            ('kDP', 'ambf_soft_body_enable_drag', 'ambf_soft_body_drag_coefficient'),
            ('kDG', 'ambf_soft_body_enable_friction', 'ambf_soft_body_dynamic_friction'),
            ('kLF', 'ambf_soft_body_enable_aerodynamics', 'ambf_soft_body_lift_coefficient'),
            ('kPR', 'ambf_soft_body_enable_pressure', 'ambf_soft_body_pressure_coefficient'),
            ('kVC', 'ambf_soft_body_enable_volume_conservation', 'ambf_soft_body_volume_conservation'),
            ('kDF', 'ambf_soft_body_enable_deformation_friction', 'ambf_soft_body_deformation_friction'),
            ('kMT', 'ambf_soft_body_enable_pose_matching', 'ambf_soft_body_pose_matching'),
            ('kCHR', 'ambf_soft_body_enable_collision_hardness', 'ambf_soft_body_collision_hardness'),
            ('kKHR', 'ambf_soft_body_enable_kinetic_hardness', 'ambf_soft_body_kinetic_hardness'),
            ('kSHR', 'ambf_soft_body_enable_shear_hardness', 'ambf_soft_body_shear_hardness'),
            ('kAHR', 'ambf_soft_body_enable_anchor_hardness', 'ambf_soft_body_anchor_hardness'),
            ('kSRHR_CL', 'ambf_soft_body_enable_srhr_cl_stiffness', 'ambf_soft_body_srhr_cl_stiffness'),
            ('kSKHR_CL', 'ambf_soft_body_enable_skhr_cl_stiffness', 'ambf_soft_body_skhr_cl_stiffness'),
            ('kSSHR_CL', 'ambf_soft_body_enable_sshr_cl_stiffness', 'ambf_soft_body_sshr_cl_stiffness'),
            ('kSR_SPLT_CL', 'ambf_soft_body_enable_sr_splt_cl_stiffness', 'ambf_soft_body_sr_splt_cl_stiffness'),
            ('kSK_SPLT_CL', 'ambf_soft_body_enable_sk_splt_cl_stiffness', 'ambf_soft_body_sk_splt_cl_stiffness'),
            ('kSS_SPLT_CL', 'ambf_soft_body_enable_ss_splt_cl_stiffness', 'ambf_soft_body_ss_splt_cl_stiffness'),
            ('maxvolume', 'ambf_soft_body_enable_max_volume', 'ambf_soft_body_max_volume'),
            ('timescale', 'ambf_soft_body_enable_timescale', 'ambf_soft_body_timescale'),
            ('viterations', 'ambf_soft_body_enable_velocity_iterations', 'ambf_soft_body_velocity_iterations'),
            ('piterations', 'ambf_soft_body_enable_position_iterations', 'ambf_soft_body_position_iterations'),
            ('diterations', 'ambf_soft_body_enable_deformation_iterations', 'ambf_soft_body_deformation_iterations'),
            ('citerations', 'ambf_soft_body_enable_collision_iterations', 'ambf_soft_body_collision_iterations'),
            ('flags', 'ambf_soft_body_enable_flags', 'ambf_soft_body_flags'),
            ('bending constraints', 'ambf_soft_body_enable_bending_constraint', 'ambf_soft_body_bending_constraint'),
            ('cutting', 'ambf_soft_body_enable_cutting_enabled', 'ambf_soft_body_cutting_enabled'),
            ('clusters', 'ambf_soft_body_enable_clusters', 'ambf_soft_body_clusters'),
        ]:
            if config_key in config:
                setattr(obj_handle.ambf_soft_body_properties, enable_attr, True)
                setattr(obj_handle.ambf_soft_body_properties, value_attr, config[config_key])

        # Handle fixed nodes
        if 'fixed nodes' in config:
            obj_handle.ambf_soft_body_properties.ambf_soft_body_enable_fixed_nodes = True
            obj_handle.ambf_soft_body_properties.ambf_soft_body_fixed_nodes.clear()
            for node_index in config['fixed nodes']:
                node_item = obj_handle.ambf_soft_body_properties.ambf_soft_body_fixed_nodes.add()
                node_item.node_index = node_index

        # Randomize constraints
        obj_handle.ambf_soft_body_properties.ambf_soft_body_randomize_constraints = config.get('randomize constraints', False)

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

    def load_body(self, body_id):
        body_data = self._adf_data[body_id]
        obj_handle = self.load_ambf_mesh(body_data, body_id)
        self.load_object_name(body_data, obj_handle)
        self.load_ambf_rigid_body(body_data, obj_handle)
        self.load_body_location(body_data, obj_handle)
        self.load_material(body_data, obj_handle)
        self._blender_remapped_body_names[body_id] = obj_handle.name
        CommonConfig.loaded_body_map[obj_handle] = body_data

    def load_soft_body(self, body_id):
        body_data = self._adf_data[body_id]
        obj_handle = self.load_ambf_mesh(body_data, body_id)
        self.load_object_name(body_data, obj_handle)
        self.load_ambf_soft_body(body_data, obj_handle)
        self.load_body_location(body_data, obj_handle)
        self.load_material(body_data, obj_handle)
        self._blender_remapped_body_names[body_id] = obj_handle.name
        CommonConfig.loaded_body_map[obj_handle] = body_data

    def load_ghost(self, ghost_name):
        ghost_data = self._adf_data[ghost_name]
        obj_handle = self.load_ambf_mesh(ghost_data, ghost_name)
        self.load_object_name(ghost_data, obj_handle)
        self.load_ambf_ghost_body(ghost_data, obj_handle)
        self.load_body_location(ghost_data, obj_handle)
        self.load_material(ghost_data, obj_handle)
        self._blender_remapped_body_names[ghost_name] = obj_handle.name
        CommonConfig.loaded_body_map[obj_handle] = ghost_data

    def load_camera(self, camera_name):
        camera_data = self._adf_data[camera_name]
        cam_handle = self.load_ambf_camera(camera_data)
        CommonConfig.loaded_camera_map[cam_handle] = camera_data

    def load_light(self, light_name):
        light_data = self._adf_data[light_name]
        light_handle = self.load_ambf_light(light_data)
        CommonConfig.loaded_light_map[light_handle] = light_data

    def load_actuator(self, actuator_name):
        actuator_data = self._adf_data[actuator_name]
        obj_handle = self.load_ambf_actuator(actuator_data)
        CommonConfig.loaded_body_map[obj_handle] = actuator_data
    
    def load_sensor(self, sensor_name):
        sensor_data = self._adf_data[sensor_name]
        obj_handle = self.load_ambf_sensor(sensor_data)
        CommonConfig.loaded_body_map[obj_handle] = sensor_data

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
        bpy.ops.object.transform_apply(scale=True, rotation=False, location=False, properties=False)

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
        joint_obj_handle.rotation_euler = T_j_w.to_euler()

        T_c_w = self.get_child_in_world_transform(joint_data)
        # Set the child body the pose calculated above
        # If the child_obj already has a parent, no need to set its transform again
        if child_obj_handle.parent is None:
            child_obj_handle.matrix_world = T_c_w
            child_obj_handle.rotation_euler = T_c_w.to_euler()

        # make_obj1_parent_of_obj2(obj1=parent_obj_handle, obj2=joint_obj_handle)
        # make_obj1_parent_of_obj2(obj1=joint_obj_handle, obj2=child_obj_handle)

        self.create_ambf_constraint(joint_obj_handle, self.get_ambf_joint_type(joint_data), parent_obj_handle,
                                       child_obj_handle)

        self.set_ambf_constraint_params(joint_obj_handle, joint_data)

        CommonConfig.loaded_joint_map[child_obj_handle.rigid_body_constraint] = joint_data

    def execute(self, context):
        # added
        cls = self.__class__
        cls._adf_data = None
        cls._joint_additional_offset = {}
        cls._blender_remapped_body_names = {}
        cls.loaded_cameras = {}
        cls._high_res_path = ''
        cls._low_res_path = ''
        cls._yaml_filepath = ''

        yaml_path = str(bpy.path.abspath(context.scene.ambf_load_adf_filepath)) # Changed
        cls._yaml_filepath = yaml_path
        # check if the file path is valid
        if not yaml_path:
            self.report({'ERROR'}, "Select an ADF file path first.")
            return {'CANCELLED'}
        
        import os
        if not os.path.isfile(yaml_path):
            self.report({'ERROR'}, f"ADF file not found: {yaml_path}")
            return {'CANCELLED'}
        print('Loading ADF File: ', yaml_path)
        yaml_file = open(yaml_path)
        
        # Check YAML version
        ver = [int(x, 10) for x in yaml.__version__.split('.')]
        if ver[0] >= 5:
            cls._adf_data = yaml.load(yaml_file, Loader=yaml.FullLoader)
        else:
            cls._adf_data = yaml.load(yaml_file)

        adf = cls._adf_data
        # print(self._adf_data)

        try:
            bodies_list = adf.get('bodies', [])
        except:
            bodies_list = []

        try:
            soft_bodies_list = adf.get('soft bodies', [])
        except:
            soft_bodies_list = []

        try :
            ghost_list = adf.get('ghost objects', [])
        except:
            ghost_list = []

        try:
            joints_list = adf.get('joints', [])
        except:
            joints_list = []
        
        try:
            camera_list = adf.get('cameras', [])
        except:
            camera_list = []

        try:
            lighting_list = adf.get('lights', [])
        except:
            lighting_list = []

        try:
            sensors_list = adf.get('sensors', [])
        except:
            sensors_list = []
        
        try:
            actuators_list = adf.get('actuators', [])
        except:
            actuators_list = []

        if 'namespace' in adf:
            set_global_namespace(context, adf['namespace'])
        else:
            set_global_namespace(context, '/ambf/env/')

        # num_bodies = len(bodies_list)
        # print('Number of Bodies Specified = ', num_bodies)

        try:
            cls._high_res_path = self.get_qualified_path(adf['high resolution path'])
        except:
            cls._high_res_path = ''

        try:
            cls._low_res_path = self.get_qualified_path(adf['low resolution path'])
        except:
            cls._low_res_path = ''

        # print(self._high_res_path)
        for body_id in bodies_list:
            print('Loading Body: ', body_id)
            self.load_body(body_id)
        
        for body_id in soft_bodies_list:
            print('Loading Soft Body: ', body_id)
            self.load_soft_body(body_id)
        
        for ghost_name in ghost_list:
            print('Loading Ghost: ', ghost_name)
            self.load_ghost(ghost_name)

        for joint_name in joints_list:
            print('Loading Joint: ', joint_name)
            self.load_ambf_joint(joint_name)

        for camera_name in camera_list:
            print('Loading Camera: ', camera_name)
            self.load_camera(camera_name)
        
        for light_name in lighting_list:
            print('Loading Light: ', light_name)
            self.load_light(light_name)
        
        for sensor_name in sensors_list:
            print('Loading Sensor: ', sensor_name)
            self.load_sensor(sensor_name)
        
        for actuator_name in actuators_list:
            print('Loading Actuator: ', actuator_name)
            self.load_actuator(actuator_name)

        # Set the model ignore collision flag
        try:
            context.scene.ambf_ignore_inter_collision = adf.get('ignore inter-collision', True)
        except:
            context.scene.ambf_ignore_inter_collision = True

        if 'gravity' in adf:
            context.scene.ambf_model_override_gravity = True
            try:
                context.scene.ambf_model_gravity[0] = adf['gravity']['x']
                context.scene.ambf_model_gravity[1] = adf['gravity']['y']
                context.scene.ambf_model_gravity[2] = adf['gravity']['z']
            except:
                print('ERROR! CANNOT READ MODEL GRAVITY FROM ADF')

        # print('Printing Blender Remapped Body Names')
        # print(self._blender_remapped_body_names)
        return {'FINISHED'}
    
"""******************* CREATOR ADDON *******************"""

'''
WORKSPACE OPERATIONS
'''
class CleanWorkspace(bpy.types.Operator):
    """Clean all objects not in any scene"""
    bl_idname = "object.clean_workspace"
    bl_label = "Clean Workspace"

    def execute(self, context):
        # Get all objects that are part of the current scene
        scene_objects = set(context.scene.objects)
        # Get all objects in the data
        all_objects = set(bpy.data.objects)
        # Calculate the difference
        orphan_objects = all_objects - scene_objects

        # Remove objects that are not part of any scene
        for obj in orphan_objects:
            # Data-blocks users counter doesn't update in the undo/redo stack, hence obj.users > 0 condition is added
            if obj.users == 0:
                bpy.data.objects.remove(obj, do_unlink=True)

        return {'FINISHED'}

class OBJECT_PT_Workspace_Main_Panel(bpy.types.Panel):
    """Panel for Workspace Operations"""
    bl_label = "Workspace Operations"
    bl_idname = "OBJECT_PT_workspace_main_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Helper"
    
    def draw(self, context):
        layout = self.layout
        layout.label(text="Perform Workspace Operations Here")

        col = layout.column()
        col.separator()
        col.operator("object.clean_workspace", text="Clean Workspace")


'''
GEOMETRY OPERATIONS
'''
class OBJECT_PT_Geometry_Main_Panel(bpy.types.Panel):
    """Panel for Geometry Operations"""
    bl_label = "Geometry Operations"
    bl_idname = "OBJECT_PT_geometry_main_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Helper"
    
    def draw(self, context):
        layout = self.layout
        layout.label(text="Perform Geometry Operations Here")

        row = layout.row()
        row.alignment = 'CENTER'
        row.operator("ed.undo", text="", icon='LOOP_BACK')
        row.separator()
        row.operator("ed.redo", text="", icon='LOOP_FORWARDS')

        col = layout.column()
        col.operator("mesh.add_cube", text="Add Cube")
        col.operator("mesh.add_sphere", text="Add Sphere")
        col.operator("mesh.add_cylinder", text="Add Cylinder")

        col = layout.column()
        col.separator()
        col.operator("object.select_all", text="Select All").action = 'SELECT'
        col.operator("object.select_all", text="Deselect All").action = 'DESELECT'
        
        col = layout.column()
        # add layout divider
        col.separator()
        col.operator("object.delete_all_objects", text="Delete All")
        col.operator("object.delete_selected_objects", text="Delete Selected")

class AddCube(bpy.types.Operator):
    """Add a new cube to the scene"""
    bl_idname = "mesh.add_cube"
    bl_label = "Add Cube"

    def execute(self, context):
        bpy.ops.mesh.primitive_cube_add()
        return {'FINISHED'}

class AddSphere(bpy.types.Operator):
    """Add a new sphere to the scene"""
    bl_idname = "mesh.add_sphere"
    bl_label = "Add Sphere"

    def execute(self, context):
        bpy.ops.mesh.primitive_uv_sphere_add()
        return {'FINISHED'}

class AddCylinder(bpy.types.Operator):
    """Add a new cylinder to the scene"""
    bl_idname = "mesh.add_cylinder"
    bl_label = "Add Cylinder"

    def execute(self, context):
        bpy.ops.mesh.primitive_cylinder_add()
        return {'FINISHED'}

class DeleteAllObjects(bpy.types.Operator):
    """Delete all objects in the scene"""
    bl_idname = "object.delete_all_objects"
    bl_label = "Delete All Objects"

    def execute(self, context):
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=False)
        return {'FINISHED'}
    
class DeleteSelectedObjects(bpy.types.Operator):
    """Delete selected objects in the scene"""
    bl_idname = "object.delete_selected_objects"
    bl_label = "Delete Selected Objects"

    def execute(self, context):
        bpy.ops.object.delete(use_global=False)
        return {'FINISHED'}

'''
CAMERA OPERATIONS
'''
class SetActiveCamera(bpy.types.Operator):
    """Set a camera as the active camera"""
    bl_idname = "scene.set_active_camera"
    bl_label = "Set Active Camera"

    camera_name: bpy.props.StringProperty() 

    def execute(self, context):
        camera = bpy.data.objects.get(self.camera_name)
        if camera and camera.type == 'CAMERA':
            context.scene.camera = camera
            self.report({'INFO'}, f"Active camera set to: {camera.name}")
        else:
            self.report({'ERROR'}, "No camera found with the given name")
        return {'FINISHED'}

class OBJECT_PT_Camera_Panel(bpy.types.Panel):
    """Panel for Camera Operations"""
    bl_label = "Camera Operations"
    bl_idname = "OBJECT_PT_camera_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Helper"

    def draw(self, context):
        layout = self.layout
        scene = context.scene

        layout.label(text="Set Active Camera")
        # Iterate through all cameras in the scene and create a button for each
        for obj in scene.objects:
            if obj.type == 'CAMERA':
                op = layout.operator("scene.set_active_camera", text=obj.name)
                op.camera_name = obj.name  # Pass the camera name to the operator

"""****************************************************************"""


"""********************** ROS Bridge *******************************"""

class SelectObjectForVelocityControl(Operator):
    bl_idname = "wm.select_object_velocity_control"
    bl_label = "Select Object for Velocity Control"
    object_name: bpy.props.StringProperty()

    def execute(self, context):
        context.scene.selected_object_velocity_control = self.object_name
        return {'FINISHED'}
    
class ROS_PT_AngularVelocityPanel(Panel):
    bl_idname = "ROS_PT_AngularVelocityPanel"
    bl_label = "Angular Velocity Panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "ROS Service"

    def draw(self, context):
        layout = self.layout
        col = layout.column()
        col.label(text='Angular Velocity Control')

        col.prop_search(context.scene, "selected_object_velocity_control", bpy.data, "objects", text="Select Object")

        row = col.row()
        row.prop(context.scene, "angular_velocity_x", text="X")
        row.prop(context.scene, "angular_velocity_y", text="Y")
        row.prop(context.scene, "angular_velocity_z", text="Z")

        col.operator("wm.send_angular_velocity", text="Send Angular Velocity")

class ROS_PT_LinearVelocityPanel(Panel):
    bl_idname = "ROS_PT_LinearVelocityPanel"
    bl_label = "Linear Velocity Panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "ROS Service"

    def draw(self, context):
        layout = self.layout
        col = layout.column()
        col.label(text='Linear Velocity Control')
        
        col.prop_search(context.scene, "selected_object_velocity_control", bpy.data, "objects", text="Select Object")

        row = col.row()
        row.prop(context.scene, "linear_velocity_x", text="X")
        row.prop(context.scene, "linear_velocity_y", text="Y")
        row.prop(context.scene, "linear_velocity_z", text="Z")

        col.operator("wm.send_linear_velocity", text="Send Linear Velocity")

class SendAngularVelocity(Operator):
    bl_idname = "wm.send_angular_velocity"
    bl_label = "Send Angular Velocity"

    def execute(self, context):
        obj_name = context.scene.selected_object_velocity_control
        x = context.scene.angular_velocity_x
        y = context.scene.angular_velocity_y
        z = context.scene.angular_velocity_z
        if obj_name:
            handler = bpy.context.blend_data.objects[obj_name]._client.get_obj_handle(obj_name) 
            handler.set_angular_velocity(x, y, z)
        return {'FINISHED'}
    
class SendLinearVelocity(Operator):
    bl_idname = "wm.send_linear_velocity"
    bl_label = "Send Linear Velocity"

    def execute(self, context):
        obj_name = context.scene.selected_object_velocity_control
        x = context.scene.linear_velocity_x
        y = context.scene.linear_velocity_y
        z = context.scene.linear_velocity_z
        if obj_name:
            handler = bpy.context.blend_data.objects[obj_name]._client.get_obj_handle(obj_name) 
            handler.set_linear_velocity(x, y, z)

        return {'FINISHED'}

class ServiceROS(Operator):
    bl_idname = "wm.ros_service"
    bl_label = "ROS Background Service"
    _timer = None
    _client = None
    is_running = False

    def start_ambf_client(self):
        from ambf_client import Client
        self._client = Client()
        self._client.connect()
        print('--> Connected to AMBF Client')

    def modal(self, context, event):
        if event.type == 'TIMER' and self.is_running:
            self.update_objects(context)
        if context.scene.stop_service:
            return self.cancel(context)
        return {'PASS_THROUGH'}

    def execute(self, context):
        if self.is_running:
            self.report({'WARNING'}, "Service already running")
            return {'CANCELLED'}
        self.start_ambf_client()
        self._timer = context.window_manager.event_timer_add(0.1, window=context.window)
        context.window_manager.modal_handler_add(self)
        self.is_running = True
        context.scene.stop_service = False
        return {'RUNNING_MODAL'}
    
    def find_object_in_collections(object_name):
        """Find an object by its name in all collections."""
        for collection in bpy.data.collections:
            for obj in collection.objects:
                if obj.name == object_name:
                    return obj
        return None

    def update_objects(self, context):
        obj_names = self._client.get_obj_names()
        print(f"Found {len(obj_names)} objects in AMBF")
        
        for ambf_name in obj_names:
            normalized_name = normalize_name(ambf_name.split('/')[-1])
            obj = find_object_by_normalized_name(normalized_name)
            print(f"Looking for object matching: {normalized_name}")
            handle = self._client.get_obj_handle(ambf_name)
            
            if handle and obj:
                pose = handle.get_pose()
                obj.location = mathutils.Vector(pose[:3])
                obj.rotation_euler = mathutils.Euler(pose[3:], 'XYZ')
                print(f"Updated {obj.name} to location {pose[:3]} and rotation {pose[3:]}")
            else:
                if not handle:
                    print(f"Failed to get AMBF handle for {ambf_name}")
                if not obj:
                    print(f"No matching object found in Blender for {normalized_name}")

    def cancel(self, context):
        if self._timer:
            context.window_manager.event_timer_remove(self._timer)
        self.is_running = False
        if self._client:
            self._client.clean_up()
        print("--> ROS Service Stopped")
        return {'CANCELLED'}

    def __del__(self):
        self.cancel(bpy.context)

class StopServiceROS(Operator):
    bl_idname = "wm.stop_ros_service"
    bl_label = "Stop ROS Background Service"

    def execute(self, context):
        context.scene.stop_service = True
        return {'FINISHED'}

class ROS_PT_Service_Panel(Panel):
    bl_idname = "ROS_PT_Service_Panel"
    bl_label = "ROS Background Service Panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "ROS Service"

    def draw(self, context):
        layout = self.layout
        col = layout.column()
        col.label(text='ROS Service Control')
        col.operator("wm.ros_service", text="Start ROS Service")
        col.operator("wm.stop_ros_service", text="Stop ROS Service", icon='CANCEL')
"""****************************************************************"""

class AMBF_OT_cleanup_all(Operator):
    """Add Rigid Body Properties"""
    bl_label = "CLEAN UP ALL"
    bl_idname = "ambf.ambf_cleanup_all"

    def execute(self, context):
        for o in bpy.data.objects:
            bpy.data.objects.remove(o)
        return {'FINISHED'}
    
class CollectionSelectorPanel(bpy.types.Panel):
    bl_label = "Collection Manager"
    bl_idname = "OBJECT_PT_collection_selector"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Collection Manager"
    
    def draw(self, context):
        layout = self.layout
        scene = context.scene

        # Active Collection Box at the Top
        box = layout.box()
        row = box.row()
        row.label(text="Active Collection:", icon='OUTLINER_OB_GROUP_INSTANCE')
        row = box.row()
        row.label(text=f"{scene.active_collection_name}" if scene.active_collection_name else "None")

        layout.separator()

        # Collection Selection
        box = layout.box()
        box.label(text="Select Collections to Show:", icon='OUTLINER_COLLECTION')

        selected_collections = scene.selected_collections.split(',')
        active_collection = scene.active_collection_name
        
        for collection in bpy.data.collections:
            row = box.row(align=True)
            is_selected = collection.name in selected_collections
            is_active = collection.name == active_collection

            if is_active:
                row.alert = True
            elif not is_selected:
                row.enabled = False

            row.prop(collection, "hide_viewport", text=collection.name, toggle=True)
            op = row.operator("view3d.activate_collection", text="", icon='RESTRICT_SELECT_OFF' if is_active else 'RADIOBUT_OFF')
            op.collection_name = collection.name
            op = row.operator("view3d.delete_collection", text="", icon='X')
            op.collection_name = collection.name

        layout.separator()

        # Add New Collection
        box = layout.box()
        box.label(text="Add New Collection:", icon='ADD')
        row = box.row(align=True)
        row.prop(scene, "new_collection_name", text="")
        row.operator("view3d.add_collection", text="", icon='ADD')

class ToggleCollectionSelectionOperator(bpy.types.Operator):
    bl_idname = "view3d.toggle_collection_selection"
    bl_label = "Toggle Collection Selection"
    
    collection_name: bpy.props.StringProperty()
    
    def execute(self, context):
        scene = context.scene
        selected_collections = scene.selected_collections.split(',')
        
        if self.collection_name in selected_collections:
            selected_collections.remove(self.collection_name)
        else:
            selected_collections.append(self.collection_name)
        
        scene.selected_collections = ','.join(selected_collections)
        return {'FINISHED'}
    
class ActivateCollectionOperator(bpy.types.Operator):
    bl_idname = "view3d.activate_collection"
    bl_label = "Activate Collection"
    
    collection_name: bpy.props.StringProperty()
    
    def execute(self, context):
        scene = context.scene
        context.scene.active_collection_name = self.collection_name
        
        # Automatically select the collection checkbox when activated
        selected_collections = scene.selected_collections.split(',')
        if self.collection_name not in selected_collections:
            selected_collections.append(self.collection_name)
            scene.selected_collections = ','.join(selected_collections)
        
        # Ensure the collection is shown by updating the selected collections
        update_selected_collections(scene, context)
        
        self.report({'INFO'}, f"Collection '{self.collection_name}' Activated and Shown")
        return {'FINISHED'}

class DeleteCollectionOperator(bpy.types.Operator):
    bl_idname = "view3d.delete_collection"
    bl_label = "Delete Collection"
    
    collection_name: bpy.props.StringProperty()
    
    def execute(self, context):
        collection = bpy.data.collections.get(self.collection_name)
        if collection:
            bpy.data.collections.remove(collection)
            # Update selected collections after deletion
            selected_collections = context.scene.selected_collections.split(',')
            if self.collection_name in selected_collections:
                selected_collections.remove(self.collection_name)
                context.scene.selected_collections = ','.join(selected_collections)
            # Clear active collection if it's the one being deleted
            if context.scene.active_collection_name == self.collection_name:
                context.scene.active_collection_name = ""
            self.report({'INFO'}, f"Collection '{self.collection_name}' Deleted")
        else:
            self.report({'WARNING'}, f"Collection '{self.collection_name}' not found")
        return {'FINISHED'}

class AddCollectionOperator(bpy.types.Operator):
    bl_idname = "view3d.add_collection"
    bl_label = "Add Collection"
    
    def execute(self, context):
        scene = context.scene
        collection_name = scene.new_collection_name.strip()
        if collection_name:
            new_collection = bpy.data.collections.new(name=collection_name)
            bpy.context.scene.collection.children.link(new_collection)
            scene.selected_collections = ','.join(scene.selected_collections.split(',') + [collection_name])
            self.report({'INFO'}, f"Collection '{collection_name}' Added")
        else:
            self.report({'WARNING'}, "Collection name cannot be empty")
        return {'FINISHED'}
    
class AMBF_OT_select_all(Operator):
    """Add Rigid Body Properties"""
    bl_label = "SELECT ALL"
    bl_idname = "ambf.ambf_select_all"

    def execute(self, context):
        select_all_objects(True)
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

class AMBF_OT_ambf_move_collision_mesh_to_body_origin(Operator):
    bl_idname = "ambf.ambf_move_collision_mesh_to_body_origin"
    bl_label = "Move collision mesh to body origin"
    bl_description = "Move the collision mesh to the correct location (i.e. body's origin, this is where AMBF will consider this mesh)"

    def execute(self, context):
        collision_mesh_obj = context.object.ambf_collision_mesh
        mesh_obj = get_active_object()
        if collision_mesh_obj is not None:
            collision_mesh_obj.matrix_world = mesh_obj.matrix_world.copy()
            make_obj1_parent_of_obj2(collision_mesh_obj, mesh_obj)
            select_object(collision_mesh_obj, False)
            select_object(mesh_obj)
            set_active_object(mesh_obj)
        return {'FINISHED'}

class AMBF_OT_ambf_collision_mesh_use_current_location(Operator):
    bl_idname = "ambf.ambf_collision_mesh_use_current_location"
    bl_label = "Use current location of collision mesh"
    bl_description = "Use the current location of the collision mesh (i.e. set its origin to be at the origin of the AMBF object without moving it)"

    def execute(self, context):
        collision_mesh_obj = context.object.ambf_collision_mesh
        mesh_obj = get_active_object()
        if collision_mesh_obj is not None:
            T_rb_w = mesh_obj.matrix_world.copy()
            T_c_w = collision_mesh_obj.matrix_world.copy()
            T_c_rb = T_rb_w.inverted() @ T_c_w
            collision_mesh_obj.matrix_world = T_rb_w
            collision_mesh_obj.data.vertices.data.transform(T_c_rb)
        return {'FINISHED'}
    
"""******************* START OBJECT OT *******************"""
class OBJECT_OT_ClearFidexNodes(Operator):
    bl_idname = "object.clear_fixed_nodes"
    bl_label = "Clear Fixed Nodes"
    bl_description = "Clear all fixed nodes"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        obj = context.object
        obj.ambf_soft_body_properties.ambf_soft_body_fixed_nodes.clear()
        self.report({'INFO'}, "Cleared all fixed nodes.")
        return {'FINISHED'}
    
class OBJECT_OT_AddFixedNodesFromSelection(Operator):
    bl_idname = "object.add_fixed_nodes_from_selection"
    bl_label = "Add Fixed Nodes from Selection"
    bl_description = "Add selected vertices as fixed nodes"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        obj = context.object
        if obj.mode != 'EDIT':
            self.report({'WARNING'}, "You need to be in Edit Mode to select vertices.")
            return {'CANCELLED'}
        
        # Access the bmesh to get selected vertices
        bm = bmesh.from_edit_mesh(obj.data)
        selected_verts = [v.index for v in bm.verts if v.select]
        
        if not selected_verts:
            self.report({'WARNING'}, "No vertices selected.")
            return {'CANCELLED'}
        
        # Add selected vertex indices to the collection property, avoiding duplicates
        existing_indices = {item.node_index for item in obj.ambf_soft_body_properties.ambf_soft_body_fixed_nodes}
        new_indices = [index for index in selected_verts if index not in existing_indices]
        
        for index in new_indices:
            item = obj.ambf_soft_body_properties.ambf_soft_body_fixed_nodes.add()
            item.node_index = index
        
        self.report({'INFO'}, f"Added {len(new_indices)} fixed nodes.")
        return {'FINISHED'}

class OBJECT_OT_RemoveFixedNode(bpy.types.Operator):
    bl_idname = "object.remove_fixed_node"
    bl_label = "Remove Fixed Node"
    bl_description = "Remove this fixed node"
    
    index: bpy.props.IntProperty()
    
    def execute(self, context):
        obj = context.object
        collection = obj.ambf_soft_body_properties.ambf_soft_body_fixed_nodes
        idx_to_remove = None
        for idx, item in enumerate(collection):
            if item.node_index == self.index:
                idx_to_remove = idx
                break
        if idx_to_remove is not None:
            collection.remove(idx_to_remove)
            self.report({'INFO'}, f"Removed fixed node {self.index}")
            return {'FINISHED'}
        else:
            self.report({'WARNING'}, "Fixed node not found.")
            return {'CANCELLED'}

class AMBF_OT_ambf_camera_activate(Operator):
    """Add Camera Properties"""
    bl_label = "AMBF CAMERA ACTIVATE"
    bl_idname = "ambf.ambf_camera_activate"

    def execute(self, context):
        if context.object.ambf_object_type != 'CAMERA':
            context.object.ambf_object_type = 'CAMERA'
        else:
            context.object.ambf_object_type = 'NONE'

        return {'FINISHED'}

class AMBF_OT_ambf_light_activate(Operator):
    """Add Light Properties"""
    bl_label = "AMBF LIGHT ACTIVATE"
    bl_idname = "ambf.ambf_light_activate"

    def execute(self, context):
        if context.object.ambf_object_type != 'LIGHT':
            context.object.ambf_object_type = 'LIGHT'
        else:
            context.object.ambf_object_type = 'NONE'

        return {'FINISHED'}

class AMBF_OT_ambf_actuator_activate(Operator):
    """Add Actuator Properties"""
    bl_label = "AMBF ACTUATOR ACTIVATE"
    bl_idname = "ambf.ambf_actuator_activate"

    def execute(self, context):
        # print ('ACTUATOR ACTIVATE')
        if context.object.ambf_object_type != 'ACTUATOR':
            context.object.ambf_object_type = 'ACTUATOR'
        else:
            context.object.ambf_object_type = 'NONE'

        return {'FINISHED'}
    
class AMBF_OT_ambf_sensor_activate(Operator):
    """Add Sensor Properties"""
    bl_label = "AMBF SENSOR ACTIVATE"
    bl_idname = "ambf.ambf_sensor_activate"

    def execute(self, context):
        # print ('SENSOR ACTIVATE')
        if context.object.ambf_object_type != 'SENSOR':
            context.object.ambf_object_type = 'SENSOR'
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
    
class AMBF_OT_ambf_soft_body_activate(Operator):
    """Add Soft Body Properties"""
    bl_label = "AMBF SOFT BODY ACTIVATE"
    bl_idname = "ambf.ambf_soft_body_activate"

    def execute(self, context):
        if context.object.ambf_object_type != 'SOFT_BODY':
            context.object.ambf_object_type = 'SOFT_BODY'
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

class OBJECT_PT_DebuggerPanel(bpy.types.Panel):
    """Creates a Panel in the Object properties window"""
    bl_label = "Debugger Panel"
    bl_idname = "OBJECT_PT_debugger"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "object"

    def draw(self, context):
        layout = self.layout
        obj = context.object

        if obj:
            # Display the name and type of the current object
            row = layout.row()
            row.label(text="Active Object:", icon='OBJECT_DATA')
            
            row = layout.row()
            row.label(text="Name: " + obj.name)

            row = layout.row()
            row.label(text="Type: " + obj.type)
        else:
            row = layout.row()
            row.label(text="No active object")

class AMBF_PT_main_panel(Panel):
    """Creates a Panel in the Tool Shelf"""
    bl_label = "IMPORT, MAKE AND EXPORT ADFs"
    bl_idname = "AMBF_PT_main_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "AMBF"

    ambf_object_types = ['RIGID_BODY','GHOST_OBJECT', 'SOFT_BODY',  'CONSTRAINT', 'COLLISION_SHAPE', 'CAMERA', 'LIGHT', 'SENSOR', 'ACTUATOR']

    setup_yaml()
    # set_view_transform_orientation_to_local()
   # bpy.context.scene.transform_orientation_slots[0].type = 'LOCAL' (2/23/26)

    def draw(self, context):

        # print('\n######### Drawing addon main Panel #########')

        # Sanity check, if there are any objects
        # that have been unlinked from the scene. Delete them
        for o in bpy.data.objects:
            if o.ambf_object_type in self.ambf_object_types:
                if context.scene.objects.get(o.name) is None:
                    print('Object: ', o.name, ' is not in the scene')
                # if bpy.data.objects[o.name] is None:
                    bpy.data.objects.remove(o)

        layout = self.layout
        
        col = layout.column()
        col.prop(context.scene, 'ambf_enable_forced_cleanup')
        
        box = layout.box()
        box.enabled = context.scene.ambf_enable_forced_cleanup
        box.label(text='!!CAUTION !! FORCED CLEANUP!!')

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

        scene = context.scene
        
        sbox = box.box()
        row = sbox.row()
        row.label(text="Select Collections to Operate:")
        
        selected_collections = scene.selected_collections.split(',')
        
        for collection in bpy.data.collections:
            row = sbox.row()
            is_selected = collection.name in selected_collections
            icon = 'CHECKBOX_HLT' if is_selected else 'CHECKBOX_DEHLT'
            op = row.operator("view3d.toggle_collection_selection", text=collection.name, icon=icon)
            op.collection_name = collection.name
        
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

        col = sbox.column()
        col.alignment = 'CENTER'
        col.prop(context.scene, "ambf_model_override_gravity")

        col = sbox.column()
        col.alignment = 'CENTER'
        col.prop(context.scene, "ambf_model_gravity")
        col.enabled = context.scene.ambf_model_override_gravity

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
        col.operator("ambf.generate_ambf_file")

        ### LAUNCH IN SIMULATOR
        layout.separator()
        box = layout.box()
        row = box.row()
        row.alignment = 'CENTER'
        row.label(text="LAUNCH IN AMBF SIMULATOR:", icon='PLAY')

        col = box.column()
        col.prop(context.scene, 'ambf_launch_adf_path', text='ADF to Launch')

        col = box.column()
        col.prop(context.scene, 'ambf_simulator_path', text='Simulator Path')

        col = box.column()
        col.scale_y = 2
        col.operator("ambf.launch_simulator", icon='PLAY')

        ### SEPERATOR
        layout.separator()

        box = layout.box()
        row = box.row()
        row.alignment = 'CENTER'
        row.label(text="OPTIONAL HELPERS:", icon='OUTLINER_DATA_ARMATURE')

        col = box.column()
        col.operator("ambf.ambf_select_all")

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

        row = box.row()
        row.scale_y = 1.5
        row.operator("ambf.create_sensor")

        row = box.row()
        row.scale_y = 1.5
        row.operator("ambf.create_actuator")

        row = box.row()
        row.scale_y = 1.5
        row.operator("ambf.create_camera")

        row = box.row()
        row.scale_y = 1.5
        row.operator("ambf.create_light")

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
            if active_obj_handle.type == 'MESH':
                active = True
                
        return active
    
    def draw(self, context):
        layout = self.layout
        
        col = layout.row()
        col.alignment = 'EXPAND'
        col.scale_y = 2
        col.operator('ambf.ambf_rigid_body_activate', text='Enable AMBF Rigid Body', icon='ADD')

        if context.object.ambf_object_type == 'RIGID_BODY':
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
            col.prop(context.object, 'ambf_body_mass')

            col = box.row()
            col.prop(context.object, 'ambf_object_override_gravity')
            col.alignment = 'EXPAND'

            col = box.row()
            col.prop(context.object, 'ambf_object_gravity')
            col.alignment = 'EXPAND'
            col.enabled = context.object.ambf_object_override_gravity
            
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
            col.prop(context.object, 'ambf_body_linear_inertial_offset')
            
            col = col.column()
            col.enabled = False
            col.alignment = 'EXPAND'
            col.prop(context.object, 'ambf_body_angular_inertial_offset')
            
            layout.separator()

            box = layout.box()
            row = box.row()
            row.prop(context.object, 'ambf_collision_type')

            if context.object.ambf_collision_type == 'MESH':

                col = box.column()
                col.prop(context.object, 'ambf_collision_mesh_type')

                col.separator()

                col = box.column()
                col.prop(context.object, 'ambf_use_separate_collision_mesh')

                col = box.column()
                col.prop(context.object, 'ambf_collision_mesh')
                col.enabled = context.object.ambf_use_separate_collision_mesh

                col = box.column()
                col.operator('ambf.ambf_move_collision_mesh_to_body_origin')
                col.enabled = context.object.ambf_use_separate_collision_mesh

                col = box.column()
                col.operator('ambf.ambf_collision_mesh_use_current_location')
                col.enabled = context.object.ambf_use_separate_collision_mesh

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
            col.prop(context.object, 'ambf_body_passive')
            
            col = box.column()
            col.prop(context.object, 'ambf_rigid_body_publish_children_names')
            col.enabled = not context.object.ambf_body_passive

            col = box.column()
            col.prop(context.object, 'ambf_rigid_body_publish_joint_names')
            col.enabled = not context.object.ambf_body_passive

            col = box.column()
            col.prop(context.object, 'ambf_rigid_body_publish_joint_positions')
            col.enabled = not context.object.ambf_body_passive

class AMBF_PT_ambf_soft_body(bpy.types.Panel):
    """Add Soft Body Properties"""
    bl_label = "AMBF SOFT BODY PROPERTIES"
    bl_idname = "AMBF_PT_ambf_soft_body"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "physics"
    
    @classmethod
    def poll(cls, context):
        active_obj_handle = context.object
        return active_obj_handle and active_obj_handle.type == 'MESH'
    
    def draw(self, context):
        layout = self.layout
        obj = context.object
        
        col = layout.row()
        col.alignment = 'EXPAND'
        col.scale_y = 2
        col.operator('ambf.ambf_soft_body_activate', text='Enable AMBF Soft Body', icon='ADD')
        
        if obj.ambf_object_type == 'SOFT_BODY':
            layout.separator()
            # Basic Properties
            box = layout.box()
            box.label(text="Basic Properties")
            box.prop(obj, 'name')
            box.prop(obj, 'ambf_body_mass')
            box.prop(obj, 'ambf_scale')

            # Color components
            col = box.column()
            col.label(text="Color Components:")
            row = col.row()
            row.prop(obj, 'ambf_object_ambient_level', text="Ambient Level")
            row.prop(obj, 'ambf_body_transparency', text="Transparency")
            row = col.row()
            row.prop(obj, 'ambf_object_diffuse_color', text="Diffuse Color")
            row = col.row()
            row.prop(obj, 'ambf_object_specular_color', text="Specular Color")

            # Namespace
            row = layout.row()
            row.prop(obj, 'ambf_soft_body_namespace')

            row = box.row()
            row.prop(obj, 'ambf_collision_margin_enable', text="Enable Collision Margin")
            if obj.ambf_collision_margin_enable:
                box.prop(obj, 'ambf_collision_margin')

            # Inertial Offsets
            box = layout.box()
            row = box.row()
            row.label(text="Inertial Offsets")
            col = box.column()
            col = col.split(factor=0.5)
            col.alignment = 'EXPAND'
            col.prop(obj, 'ambf_body_linear_inertial_offset')
            col = col.column()
            col.alignment = 'EXPAND'
            col.prop(obj, 'ambf_body_angular_inertial_offset')

            # # Stiffness Properties
            # box = layout.box()
            # box.label(text="Stiffness Properties")
            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_linear_stiffness', text="Enable Linear Stiffness")
            # if obj.ambf_soft_body_enable_linear_stiffness:
            #     box.prop(obj, 'ambf_soft_body_linear_stiffness')

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_angular_stiffness', text="Enable Angular Stiffness")
            # if obj.ambf_soft_body_enable_angular_stiffness:
            #     box.prop(obj, 'ambf_soft_body_angular_stiffness')
            
            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_volume_stiffness', text="Enable Volume Stiffness")
            # if obj.ambf_soft_body_enable_volume_stiffness:
            #     box.prop(obj, 'ambf_soft_body_volume_stiffness')

            # # Damping and Drag Properties
            # box = layout.box()
            # box.label(text="Damping and Drag Properties")

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_damping', text="Enable Damping")
            # if obj.ambf_soft_body_enable_damping:
            #     box.prop(obj, 'ambf_soft_body_velocity_damping')
            
            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_drag', text="Enable Drag")
            # if obj.ambf_soft_body_enable_drag:
            #     box.prop(obj, 'ambf_soft_body_drag_coefficient')

            # # Friction Properties
            # box = layout.box()
            # box.label(text="Friction Properties")

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_friction', text="Enable Friction")
            # if obj.ambf_soft_body_enable_friction:
            #     box.prop(obj, 'ambf_soft_body_dynamic_friction')

            # # Aerodynamics and Pressure Properties
            # box = layout.box()
            # box.label(text="Aerodynamic and Pressure Properties")

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_aerodynamics', text="Enable Aerodynamics")
            # if obj.ambf_soft_body_enable_aerodynamics:
            #     box.prop(obj, 'ambf_soft_body_lift_coefficient')

            # # Pressure and Volume Conservation Properties
            # box = layout.box()
            # box.label(text="Pressure and Volume Conservation Properties")

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_pressure', text="Enable Pressure")
            # if obj.ambf_soft_body_enable_pressure:
            #     box.prop(obj, 'ambf_soft_body_pressure_coefficient')

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_volume_conservation', text="Enable Volume Conservation")
            # if obj.ambf_soft_body_enable_volume_conservation:
            #     box.prop(obj, 'ambf_soft_body_volume_conservation')

            # # Hardness Properties
            # box = layout.box()
            # box.label(text="Hardness Properties")

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_collision_hardness', text="Enable Collision Hardness")
            # if obj.ambf_soft_body_enable_collision_hardness:
            #     box.prop(obj, 'ambf_soft_body_collision_hardness')

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_kinetic_hardness', text="Enable Kinetic Hardness")
            # if obj.ambf_soft_body_enable_kinetic_hardness:
            #     box.prop(obj, 'ambf_soft_body_kinetic_hardness')

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_shear_hardness', text="Enable Shear Hardness")
            # if obj.ambf_soft_body_enable_shear_hardness:
            #     box.prop(obj, 'ambf_soft_body_shear_hardness')

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_deformation_friction', text="Enable Deformation Friction")
            # if obj.ambf_soft_body_enable_deformation_friction:
            #     box.prop(obj, 'ambf_soft_body_deformation_friction')

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_pose_matching', text="Enable Pose Matching")
            # if obj.ambf_soft_body_enable_pose_matching:
            #     box.prop(obj, 'ambf_soft_body_pose_matching')

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_anchor_hardness', text="Enable Anchor Hardness")
            # if obj.ambf_soft_body_enable_anchor_hardness:
            #     box.prop(obj, 'ambf_soft_body_anchor_hardness')

            # # Cluster-Related Stiffness Properties
            # box = layout.box()
            # box.label(text="Cluster-Related Stiffness Properties")

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_srhr_cl_stiffness', text="Enable SRHR_CL Stiffness")
            # if obj.ambf_soft_body_enable_srhr_cl_stiffness:
            #     box.prop(obj, 'ambf_soft_body_srhr_cl_stiffness')

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_skhr_cl_stiffness', text="Enable SKHR_CL Stiffness")
            # if obj.ambf_soft_body_enable_skhr_cl_stiffness:
            #     box.prop(obj, 'ambf_soft_body_skhr_cl_stiffness')

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_ssHR_cl_stiffness', text="Enable SSHR_CL Stiffness")
            # if obj.ambf_soft_body_enable_ssHR_cl_stiffness:
            #     box.prop(obj, 'ambf_soft_body_ssHR_cl_stiffness')

            # # Cluster Split Stiffness Properties
            # box = layout.box()
            # box.label(text="Cluster Split Stiffness Properties")

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_sr_splt_cl_stiffness', text="Enable SR_SPLT_CL Stiffness")
            # if obj.ambf_soft_body_enable_sr_splt_cl_stiffness:
            #     box.prop(obj, 'ambf_soft_body_sr_splt_cl_stiffness')

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_sk_splt_cl_stiffness', text="Enable SK_SPLT_CL Stiffness")
            # if obj.ambf_soft_body_enable_sk_splt_cl_stiffness:
            #     box.prop(obj, 'ambf_soft_body_sk_splt_cl_stiffness')

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_ss_splt_cl_stiffness', text="Enable SS_SPLT_CL Stiffness")
            # if obj.ambf_soft_body_enable_ss_splt_cl_stiffness:
            #     box.prop(obj, 'ambf_soft_body_ss_splt_cl_stiffness')

            # # Maximum Volume and Timescale Properties
            # box = layout.box()
            # box.label(text="Volume and Timescale Properties")

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_max_volume', text="Enable Max Volume")
            # if obj.ambf_soft_body_enable_max_volume:
            #     box.prop(obj, 'ambf_soft_body_max_volume')

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_timescale', text="Enable Timescale")
            # if obj.ambf_soft_body_enable_timescale:
            #     box.prop(obj, 'ambf_soft_body_timescale')

            # # Iterations Properties
            # box = layout.box()
            # box.label(text="Iterations Properties")

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_velocity_iterations', text="Enable Velocity Iterations")
            # if obj.ambf_soft_body_enable_velocity_iterations:
            #     box.prop(obj, 'ambf_soft_body_velocity_iterations')

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_position_iterations', text="Enable Position Iterations")
            # if obj.ambf_soft_body_enable_position_iterations:
            #     box.prop(obj, 'ambf_soft_body_position_iterations')

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_deformation_iterations', text="Enable Deformation Iterations")
            # if obj.ambf_soft_body_enable_deformation_iterations:
            #     box.prop(obj, 'ambf_soft_body_deformation_iterations')

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_collision_iterations', text="Enable Collision Iterations")
            # if obj.ambf_soft_body_enable_collision_iterations:
            #     box.prop(obj, 'ambf_soft_body_collision_iterations')

            # # Flags, Bending, and Cutting Properties
            # box = layout.box()
            # box.label(text="Flags, Bending, and Cutting Properties")

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_flags', text="Enable Flags")
            # if obj.ambf_soft_body_enable_flags:
            #     box.prop(obj, 'ambf_soft_body_flags')

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_bending_constraint', text="Enable Bending Constraint")
            # if obj.ambf_soft_body_enable_bending_constraint:
            #     box.prop(obj, 'ambf_soft_body_bending_constraint')

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_cutting_enabled', text="Enable Cutting")
            # if obj.ambf_soft_body_enable_cutting_enabled:
            #     box.prop(obj, 'ambf_soft_body_cutting_enabled')

            # row = box.row()
            # row.prop(obj, 'ambf_soft_body_enable_clusters', text="Enable Clusters")
            # if obj.ambf_soft_body_enable_clusters:
            #     box.prop(obj, 'ambf_soft_body_clusters')

            prop_groups = [
                ("Stiffness Properties", [
                    ('ambf_soft_body_enable_linear_stiffness', 'ambf_soft_body_linear_stiffness', "Linear Stiffness"),
                    ('ambf_soft_body_enable_angular_stiffness', 'ambf_soft_body_angular_stiffness', "Angular Stiffness"),
                    ('ambf_soft_body_enable_volume_stiffness', 'ambf_soft_body_volume_stiffness', "Volume Stiffness")
                ]),
                ("Damping and Drag Properties", [
                    ('ambf_soft_body_enable_damping', 'ambf_soft_body_velocity_damping', "Velocity Damping"),
                    ('ambf_soft_body_enable_drag', 'ambf_soft_body_drag_coefficient', "Drag Coefficient")
                ]),
                ("Friction Properties", [
                    ('ambf_soft_body_enable_friction', 'ambf_soft_body_dynamic_friction', "Dynamic Friction")
                ]),
                ("Aerodynamic Properties", [
                    ('ambf_soft_body_enable_aerodynamics', 'ambf_soft_body_lift_coefficient', "Lift Coefficient")
                ]),
                ("Pressure and Volume Conservation Properties", [
                    ('ambf_soft_body_enable_pressure', 'ambf_soft_body_pressure_coefficient', "Pressure Coefficient"),
                    ('ambf_soft_body_enable_volume_conservation', 'ambf_soft_body_volume_conservation', "Volume Conservation")
                ]),
                ("Hardness Properties", [
                    ('ambf_soft_body_enable_collision_hardness', 'ambf_soft_body_collision_hardness', "Collision Hardness"),
                    ('ambf_soft_body_enable_kinetic_hardness', 'ambf_soft_body_kinetic_hardness', "Kinetic Hardness"),
                    ('ambf_soft_body_enable_shear_hardness', 'ambf_soft_body_shear_hardness', "Shear Hardness"),
                    ('ambf_soft_body_enable_deformation_friction', 'ambf_soft_body_deformation_friction', "Deformation Friction"),
                    ('ambf_soft_body_enable_pose_matching', 'ambf_soft_body_pose_matching', "Pose Matching"),
                    ('ambf_soft_body_enable_anchor_hardness', 'ambf_soft_body_anchor_hardness', "Anchor Hardness")
                ]),
                ("Cluster-Related Stiffness Properties", [
                    ('ambf_soft_body_enable_srhr_cl_stiffness', 'ambf_soft_body_srhr_cl_stiffness', "SRHR_CL Stiffness"),
                    ('ambf_soft_body_enable_skhr_cl_stiffness', 'ambf_soft_body_skhr_cl_stiffness', "SKHR_CL Stiffness"),
                    ('ambf_soft_body_enable_sshr_cl_stiffness', 'ambf_soft_body_sshr_cl_stiffness', "SSHR_CL Stiffness")
                ]),
                ("Cluster Split Stiffness Properties", [
                    ('ambf_soft_body_enable_sr_splt_cl_stiffness', 'ambf_soft_body_sr_splt_cl_stiffness', "SR_SPLT_CL Stiffness"),
                    ('ambf_soft_body_enable_sk_splt_cl_stiffness', 'ambf_soft_body_sk_splt_cl_stiffness', "SK_SPLT_CL Stiffness"),
                    ('ambf_soft_body_enable_ss_splt_cl_stiffness', 'ambf_soft_body_ss_splt_cl_stiffness', "SS_SPLT_CL Stiffness")
                ]),
                ("Volume and Timescale Properties", [
                    ('ambf_soft_body_enable_max_volume', 'ambf_soft_body_max_volume', "Maximum Volume"),
                    ('ambf_soft_body_enable_timescale', 'ambf_soft_body_timescale', "Timescale")
                ]),
                ("Iterations Properties", [
                    ('ambf_soft_body_enable_velocity_iterations', 'ambf_soft_body_velocity_iterations', "Velocity Iterations"),
                    ('ambf_soft_body_enable_position_iterations', 'ambf_soft_body_position_iterations', "Position Iterations"),
                    ('ambf_soft_body_enable_deformation_iterations', 'ambf_soft_body_deformation_iterations', "Deformation Iterations"),
                    ('ambf_soft_body_enable_collision_iterations', 'ambf_soft_body_collision_iterations', "Collision Iterations")
                ]),
                ("Flags, Bending, and Cutting Properties", [
                    ('ambf_soft_body_enable_flags', 'ambf_soft_body_flags', "Flags"),
                    ('ambf_soft_body_enable_bending_constraint', 'ambf_soft_body_bending_constraint', "Bending Constraint"),
                    ('ambf_soft_body_enable_cutting_enabled', 'ambf_soft_body_cutting_enabled', "Cutting Enabled")
                ])
            ]

            for group_name, properties in prop_groups:
                box = layout.box()
                box.label(text=group_name)
                for toggle_attr, value_attr, label in properties:
                    self._draw_toggle_and_value(box, obj.ambf_soft_body_properties, toggle_attr, value_attr, label)

            row = box.row()
            row.prop(obj.ambf_soft_body_properties, 'ambf_soft_body_enable_fixed_nodes', text="Enable Fixed Nodes")
            if obj.ambf_soft_body_properties.ambf_soft_body_enable_fixed_nodes:
                # Instructions for the user
                row = layout.row()
                row.label(text="Select vertices in Edit Mode and click:")

                # Button to add fixed nodes from selection
                row = layout.row()
                row.operator("object.add_fixed_nodes_from_selection", text="Add Fixed Nodes from Selection")
                row.operator("object.clear_fixed_nodes", text="", icon='TRASH')

                # Display the list of fixed nodes
                if obj.ambf_soft_body_properties.ambf_soft_body_fixed_nodes:
                    box = layout.box()
                    box.label(text="Fixed Nodes:")
                    for item in obj.ambf_soft_body_properties.ambf_soft_body_fixed_nodes:
                        row = box.row()
                        row.label(text=f"Index: {item.node_index}")
                        op = row.operator("object.remove_fixed_node", text="", icon='X')
                        op.index = item.node_index
                else:
                    row = layout.row()
                    row.label(text="No fixed nodes added.")

            # Randomize Constraints
            box = layout.box()
            box.label(text="Randomize Constraints")
            row = box.row()
            row.prop(obj.ambf_soft_body_properties, 'ambf_soft_body_randomize_constraints', text="Randomize Constraints")
            
    def _draw_toggle_and_value(self, box, soft_body, toggle_prop, value_prop, label):
        """Helper function to draw a toggle and its corresponding value."""
        row = box.row()
        row.prop(soft_body, toggle_prop, text=f"Enable {label}")
        if getattr(soft_body, toggle_prop):
            box.prop(soft_body, value_prop, text=label)

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
        if active_obj_handle:  
            if active_obj_handle.type == 'MESH':
                active = True
        return active

    def draw(self, context):
        layout = self.layout

        col = layout.row()
        col.alignment = 'EXPAND'
        col.scale_y = 2
        col.operator('ambf.ambf_ghost_object_activate', text='Enable AMBF Ghost Object', icon='ADD')

        if context.object.ambf_object_type == 'GHOST_OBJECT':
            layout.separator()
            col = layout.column()
            col.prop(context.object, 'ambf_object_visible', toggle=True)
            col.scale_y = 2.0

            col = layout.column()
            col.enabled = False
            col.prop(context.object, 'ambf_rigid_body_namespace')

            layout.separator()

            col = layout.column()
            col.prop_search(context.object, "ambf_object_parent", context.scene, "objects")

            layout.separator()

            # Color components
            box = layout.box()
            col = box.column()
            col.label(text="Color Components:")
            row = col.row()
            row.prop(context.object, 'ambf_object_ambient_level', text="Ambient Level")
            row.prop(context.object, 'ambf_body_transparency', text="Transparency")
            row = col.row()
            row.prop(context.object, 'ambf_object_diffuse_color', text="Diffuse Color")
            row = col.row()
            row.prop(context.object, 'ambf_object_specular_color', text="Specular Color")

            # Collision properties
            box = layout.box()
            row = box.row()
            row.prop(context.object, 'ambf_collision_shape', text="Collision Shape")
            row = box.row()
            row.prop(context.object, 'ambf_ghost_collision_geometry', text="Collision Geometry")            

            if context.object.ambf_collision_type == 'MESH':
                col = box.column()
                col.prop(context.object, 'ambf_collision_mesh_type', text="Collision Mesh Type")
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
            row.prop(context.object, 'ambf_collision_margin', text="Collision Margin")

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

            col.prop(context.object, 'ambf_scale', text="Scale")
            col.prop(context.object, 'ambf_body_passive')

class AMBF_PT_ambf_constraint(Panel):
    """Add Body Properties"""
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
        else:
            layout.separator()
            col = layout.column()
            
            layout.separator()

            col = layout.column()
            col.prop(context.object, 'ambf_constraint_enable_feedback')

            col = layout.column()
            col.prop(context.object, 'ambf_constraint_passive')

class AMBF_PT_ambf_camera(Panel):
    """Add Camera Properties"""
    bl_label = "AMBF CAMERA PROPERTIES"
    bl_idname = "AMBF_PT_ambf_camera"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "physics"

    @classmethod
    def poll(self, context):
        active = False
        active_obj_handle = get_active_object()
        if active_obj_handle:
            if active_obj_handle.type in ['CAMERA']:
                active = True
        return active
    
    def draw(self, context):
        layout = self.layout

        row = layout.row()
        row.alignment = 'EXPAND'
        row.operator('ambf.ambf_camera_activate', text='Enable AMBF Camera', icon='CAMERA_DATA')
        row.scale_y = 2

        if context.object.ambf_object_type == 'CAMERA':
            layout.separator()

            col = layout.column()
            col.prop(context.object, 'name')

            layout.separator()
            box = layout.box()
            box.label(text="Optics")
            col = box.column()
            col.prop(context.object.data, 'angle', text="Field of View")
            col = box.column()
            col.prop(context.object, 'ambf_camera_clip_start', text="Near Plane")
            col = box.column()
            col.prop(context.object, 'ambf_camera_clip_end', text="Far Plane")

            layout.separator()
            box = layout.box()
            row = box.row()
            row.prop(context.object, 'ambf_camera_enable_intrinsics',
                     text="Option 1: Camera Intrinsics", toggle=True)
            if context.object.ambf_camera_enable_intrinsics:
                col = box.column()
                row = col.row()
                row.prop(context.object, 'ambf_camera_intrinsics_fx')
                row.prop(context.object, 'ambf_camera_intrinsics_fy')
                row = col.row()
                row.prop(context.object, 'ambf_camera_intrinsics_cx')
                row.prop(context.object, 'ambf_camera_intrinsics_cy')
                col.prop(context.object, 'ambf_camera_intrinsics_s')

            layout.separator()
            box = layout.box()
            row = box.row()
            row.prop(context.object, 'ambf_camera_enable_projection_matrix',
                     text="Option 2: Projection Matrix", toggle=True)
            if context.object.ambf_camera_enable_projection_matrix:
                col = box.column()
                mat = context.object.ambf_camera_projection_matrix
                for r in range(4):
                    row = col.row(align=True)
                    for c in range(4):
                        row.prop(context.object, 'ambf_camera_projection_matrix',
                                 index=r * 4 + c, text="")

            layout.separator()
            box = layout.box()
            box.label(text="Display")
            col = box.column()
            col.prop(context.object, 'ambf_camera_monitor', text="Monitor Number")

            layout.separator()
            box = layout.box()
            box.label(text="Publish Image (ROS)")
            row = box.row()
            row.prop(context.object, 'ambf_camera_publish_image', text="Enable")
            col = box.column()
            col.enabled = context.object.ambf_camera_publish_image
            col.prop(context.object, 'ambf_camera_publish_image_interval', text="Interval (steps)")
            row = col.row()
            row.prop(context.object, 'ambf_camera_publish_image_width', text="Width")
            row.prop(context.object, 'ambf_camera_publish_image_height', text="Height")

            layout.separator()
            box = layout.box()
            box.label(text="Publish Depth (ROS)")
            row = box.row()
            row.prop(context.object, 'ambf_camera_publish_depth', text="Enable")
            col = box.column()
            col.enabled = context.object.ambf_camera_publish_depth
            col.prop(context.object, 'ambf_camera_publish_depth_interval', text="Interval (steps)")
            row = col.row()
            row.prop(context.object, 'ambf_camera_publish_depth_width', text="Width")
            row.prop(context.object, 'ambf_camera_publish_depth_height', text="Height")

class AMBF_PT_ambf_light(Panel):
    """Add Light Properties"""
    bl_label = "AMBF LIGHT PROPERTIES"
    bl_idname = "AMBF_PT_ambf_light"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "physics"

    @classmethod
    def poll(self, context):
        active = False
        active_obj_handle = get_active_object()
        if active_obj_handle:
            if active_obj_handle.type == 'LIGHT':
                active = True
        return active
    
    def draw(self, context):
        layout = self.layout

        row = layout.row()
        row.alignment = 'EXPAND'
        row.operator('ambf.ambf_light_activate', text='Enable AMBF Light', icon='LIGHT')
        row.scale_y = 2

        if context.object.ambf_object_type == 'LIGHT':
            row = layout.row()
            row.prop(context.object, 'ambf_light_namespace')

            row = layout.row()
            row.prop(context.object, 'name')

            layout.separator()
            box = layout.box()
            box.label(text="Light Type")
            row = box.row()
            row.label(text="Spotlight", icon='LIGHT_SPOT')

            layout.separator()
            box = layout.box()
            box.label(text="Light Properties")
            col = box.column()
            col.prop(context.object, 'ambf_light_shadow_quality', text="Shadow Quality")

            is_spot = context.object.ambf_light_type == 'SPOT'
            col = box.column()
            col.enabled = is_spot
            col.prop(context.object, 'ambf_light_spot_exponent', text="Spot Exponent")
            col = box.column()
            col.enabled = is_spot
            col.prop(context.object, 'ambf_light_cutoff_angle', text="Cutoff Angle")

            layout.separator()
            box = layout.box()
            box.label(text="Attenuation")
            col = box.column()
            row = col.row(align=True)
            row.prop(context.object, 'ambf_light_attenuation_constant',  text="Constant")
            row.prop(context.object, 'ambf_light_attenuation_linear',    text="Linear")
            row.prop(context.object, 'ambf_light_attenuation_quadratic', text="Quadratic")

            layout.separator()
            box = layout.box()
            box.label(text="Attachment")
            row = box.row()
            row.prop_search(context.object, 'ambf_light_parent', context.scene, 'objects', text="Parent Body")


class AMBF_PT_ambf_actuator(Panel):
    """Add Actuator Properties"""
    bl_label = "AMBF ACTUATOR PROPERTIES"
    bl_idname = "AMBF_PT_ambf_actuator"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "physics"

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
        row.operator('ambf.ambf_actuator_activate', text='Enable AMBF Actuator', icon='FORCE_HARMONIC')
        row.scale_y = 2

        if context.object.ambf_object_type == 'ACTUATOR':
            # Name property
            row = layout.row()
            row.prop(context.object, 'name')

            # Parent property
            row = layout.row()
            row.prop_search(context.object, 'ambf_object_parent', context.scene, 'objects')

            # Visible boolean property
            row = layout.row()
            row.prop(context.object, 'ambf_object_visible', toggle=False)

            # Visible size property
            row = layout.row()
            row.alignment = 'EXPAND'
            row.prop(context.object, 'ambf_object_visible_size')

class AMBF_PT_ambf_sensor(Panel):
    """Add Sensor Properties"""
    bl_label = "AMBF SENSOR PROPERTIES"
    bl_idname = "AMBF_PT_ambf_sensor"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "physics"

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
        obj = context.object
        sensor = obj.ambf_sensor_properties

        # Sensor Activation Button
        row = layout.row()
        row.alignment = 'EXPAND'
        row.operator('ambf.ambf_sensor_activate', text='Enable AMBF Sensor', icon='FORCE_HARMONIC')
        row.scale_y = 2

        # Display properties only if the object is a sensor
        if obj.ambf_object_type == 'SENSOR':
            # Sensor Name Property
            row = layout.row()
            row.prop(obj, 'name', text="Sensor Name")

            # Parent Property
            row = layout.row()
            row.prop_search(obj, 'ambf_object_parent', context.scene, 'objects', text="Parent Object")

            # Sensor Type Dropdown
            row = layout.row()
            row.prop(sensor, 'ambf_sensor_type', text="Sensor Type")

            # Visible Boolean Property
            row = layout.row()
            row.prop(obj, 'ambf_object_visible', text="Visible")

            # Visible Size Property
            row = layout.row()
            row.alignment = 'EXPAND'
            row.prop(obj, 'ambf_object_visible_size', text="Visible Size")
            
            # Conditional properties based on sensor type
            sensor_type = sensor.ambf_sensor_type

            if sensor.ambf_sensor_type == 'Proximity':
                # Range Property
                row = layout.row()
                row.prop(sensor, 'ambf_sensor_range', text="Range")

                # Sensor Array Items
                col = layout.column()
                col.label(text="Sensor Array Items:")
                col.operator('ambf.add_sensor_array_item', text="Add Array Item")

                for i, item in enumerate(sensor.ambf_sensor_array):
                    box = col.box()
                    row = box.row()
                    row.label(text=f"Item {i+1}")
                    
                    # Remove button
                    remove_op = row.operator('ambf.remove_sensor_array_item', text="", icon='X')
                    remove_op.index = i  # Pass the index of the item to remove

                    # Offset and Direction properties
                    box.prop(item, "offset", text="Offset")
                    box.prop(item, "direction", text="Direction")

            elif sensor_type == 'Resistance':
                # Friction Properties
                row = layout.row()
                row.label(text="Friction Properties")
                col = layout.column(align=True)
                col.prop(sensor, 'ambf_sensor_friction_static', text="Static Friction")
                col.prop(sensor, 'ambf_sensor_friction_damping', text="Damping Friction")
                col.prop(sensor, 'ambf_sensor_friction_dynamic', text="Dynamic Friction")
                col.prop(sensor, 'ambf_sensor_friction_variable', text="Variable Friction")

                # Contact Area, Stiffness, Damping
                row = layout.row()
                row.prop(sensor, 'ambf_sensor_contact_area', text="Contact Area")
                row = layout.row()
                row.prop(sensor, 'ambf_sensor_contact_stiffness', text="Contact Stiffness")
                row = layout.row()
                row.prop(sensor, 'ambf_sensor_contact_damping', text="Contact Damping")

            elif sensor_type == 'Contact':
                # Distance Threshold
                row = layout.row()
                row.prop(sensor, 'ambf_sensor_distance_threshold', text="Distance Threshold")

                # Process Contact Details
                row = layout.row()
                row.prop(sensor, 'ambf_sensor_process_contact_details', text="Process Contact Details")
            
            # TODO: Implement this in the future
            # # Enable Frequency Checkbox
            # row = layout.row()
            # row.prop(obj, 'ambf_sensor_enable_frequency', text="Enable Frequency")

            # # Only display Publish Frequency Property if enabled
            # if obj.ambf_sensor_enable_frequency:
            #     row = layout.row()
            #     row.prop(obj, 'ambf_sensor_frequency', text="Publish Frequency")

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
    
class GhostObjectProperties(PropertyGroup):
    bl_idname = "OBJECT_PT_ghost_object_properties"
    bl_label = "Ghost Object Properties"
    Object.ambf_ghost_scale = FloatProperty(name="Scale", default=1.0, min=0.1)
    Object.ambf_ghost_shape = EnumProperty(
        name="Shape",
        items=[
            ('BOX', "Box", "", 0),
            ('SPHERE', "Sphere", "", 1),
            ('CAPSULE', "Capsule", "", 2),
            ('CYLINDER', "Cylinder", "", 3),
            ('CONE', "Cone", "", 4),
        ],
        default='BOX'
    )
    # ambf_collision_geometry: StringProperty(name="Collision Geometry", default="*box_geometry")
    Object.ambf_ghost_collision_geometry = FloatVectorProperty(name="Collision Geometry", size=3, default=(0.0, 0.0, 0.0))

class FixedNodeItem(PropertyGroup):
    node_index: IntProperty(name="Fixed Node Index")

class SoftBodyProperties(PropertyGroup):
    ambf_soft_body_namespace: StringProperty(
        name="Namespace",
        default="/ambf/env/"
    )

    # Stiffness properties
    ambf_soft_body_enable_linear_stiffness: BoolProperty(
        name="Enable Linear Stiffness", default=False
    )
    ambf_soft_body_linear_stiffness: FloatProperty(
        name="kLST (Linear Stiffness)", default=0.05, min=-float('inf'), max=float('inf')
    )
    ambf_soft_body_enable_angular_stiffness: BoolProperty(
        name="Enable Angular Stiffness", default=False
    )
    ambf_soft_body_angular_stiffness: FloatProperty(
        name="kAST (Angular Stiffness)", default=0.05, min=-float('inf'), max=float('inf')
    )
    ambf_soft_body_enable_volume_stiffness: BoolProperty(
        name="Enable Volume Stiffness", default=False
    )
    ambf_soft_body_volume_stiffness: FloatProperty(
        name="kVST (Volume Stiffness)", default=0.05, min=-float('inf'), max=float('inf')
    )

    # Damping and drag properties
    ambf_soft_body_enable_damping: BoolProperty(
        name="Enable Damping", default=False
    )
    ambf_soft_body_velocity_damping: FloatProperty(
        name="kVCF (Velocity Damping Coefficient)", default=0.05, min=-float('inf'), max=float('inf')
    )
    ambf_soft_body_enable_drag: BoolProperty(
        name="Enable Drag", default=False
    )
    ambf_soft_body_drag_coefficient: FloatProperty(
        name="kDP (Drag Coefficient)", default=0.05, min=-float('inf'), max=float('inf')
    )

    # Friction properties
    ambf_soft_body_enable_friction: BoolProperty(
        name="Enable Friction", default=False
    )
    ambf_soft_body_dynamic_friction: FloatProperty(
        name="kDG (Dynamic Friction Coefficient)", default=0.05, min=-float('inf'), max=float('inf')
    )

    # Aerodynamic properties
    ambf_soft_body_enable_aerodynamics: BoolProperty(
        name="Enable Aerodynamics", default=False
    )
    ambf_soft_body_lift_coefficient: FloatProperty(
        name="kLF (Lift Coefficient)", default=0.05, min=-float('inf'), max=float('inf')
    )

    # Pressure and volume conservation properties
    ambf_soft_body_enable_pressure: BoolProperty(
        name="Enable Pressure", default=False
    )
    ambf_soft_body_pressure_coefficient: FloatProperty(
        name="kPR (Pressure Coefficient)", default=0.05, min=-float('inf'), max=float('inf')
    )
    ambf_soft_body_enable_volume_conservation: BoolProperty(
        name="Enable Volume Conservation", default=False
    )
    ambf_soft_body_volume_conservation: FloatProperty(
        name="kVC (Volume Conservation Coefficient)", default=0.05, min=-float('inf'), max=float('inf')
    )

    # Hardness properties
    ambf_soft_body_enable_collision_hardness: BoolProperty(
        name="Enable Collision Hardness", default=False
    )
    ambf_soft_body_collision_hardness: FloatProperty(
        name="kCHR (Collision Hardening Coefficient)", default=0.05, min=-float('inf'), max=float('inf')
    )
    ambf_soft_body_enable_kinetic_hardness: BoolProperty(
        name="Enable Kinetic Hardness", default=False
    )
    ambf_soft_body_kinetic_hardness: FloatProperty(
        name="kKHR (Kinetic Hardening Coefficient)", default=0.05, min=-float('inf'), max=float('inf')
    )
    ambf_soft_body_enable_shear_hardness: BoolProperty(
        name="Enable Shear Hardness", default=False
    )
    ambf_soft_body_shear_hardness: FloatProperty(
        name="kSHR (Shear Hardening Coefficient)", default=0.05, min=-float('inf'), max=float('inf')
    )
    ambf_soft_body_enable_deformation_friction: BoolProperty(
        name="Enable Deformation Friction", default=False
    )
    ambf_soft_body_deformation_friction: FloatProperty(
        name="kDF (Deformation Friction)", default=0.05, min=-float('inf'), max=float('inf')
    )
    ambf_soft_body_enable_pose_matching: BoolProperty(
        name="Enable Pose Matching", default=False
    )
    ambf_soft_body_pose_matching: FloatProperty(
        name="kMT (Pose Matching)", default=0.0, min=-float('inf'), max=float('inf')
    )
    ambf_soft_body_enable_anchor_hardness: BoolProperty(
        name="Enable Anchor Hardness", default=False
    )
    ambf_soft_body_anchor_hardness: FloatProperty(
        name="kAHR (Anchor Hardness)", default=0.05, min=-float('inf'), max=float('inf')
    )

    # Cluster-related stiffness properties
    ambf_soft_body_enable_srhr_cl_stiffness: BoolProperty(
        name="Enable SRHR_CL Stiffness", default=False
    )
    ambf_soft_body_srhr_cl_stiffness: FloatProperty(
        name="kSRHR_CL", default=0.05, min=-float('inf'), max=float('inf')
    )
    ambf_soft_body_enable_skhr_cl_stiffness: BoolProperty(
        name="Enable SKHR_CL Stiffness", default=False
    )
    ambf_soft_body_skhr_cl_stiffness: FloatProperty(
        name="kSKHR_CL", default=0.05, min=-float('inf'), max=float('inf')
    )
    ambf_soft_body_enable_sshr_cl_stiffness: BoolProperty(
        name="Enable SSHR_CL Stiffness", default=False
    )
    ambf_soft_body_sshr_cl_stiffness: FloatProperty(
        name="kSSHR_CL", default=0.05, min=-float('inf'), max=float('inf')
    )

    # Cluster Split Stiffness Properties
    ambf_soft_body_enable_sr_splt_cl_stiffness: BoolProperty(
        name="Enable SR_SPLT_CL Stiffness", default=False
    )
    ambf_soft_body_sr_splt_cl_stiffness: FloatProperty(
        name="kSR_SPLT_CL", default=0.05, min=-float('inf'), max=float('inf')
    )
    ambf_soft_body_enable_sk_splt_cl_stiffness: BoolProperty(
        name="Enable SK_SPLT_CL Stiffness", default=False
    )
    ambf_soft_body_sk_splt_cl_stiffness: FloatProperty(
        name="kSK_SPLT_CL", default=0.05, min=-float('inf'), max=float('inf')
    )
    ambf_soft_body_enable_ss_splt_cl_stiffness: BoolProperty(
        name="Enable SS_SPLT_CL Stiffness", default=False
    )
    ambf_soft_body_ss_splt_cl_stiffness: FloatProperty(
        name="kSS_SPLT_CL", default=0.05, min=-float('inf'), max=float('inf')
    )

    # Maximum volume property
    ambf_soft_body_enable_max_volume: BoolProperty(
        name="Enable Maximum Volume", default=False
    )
    ambf_soft_body_max_volume: FloatProperty(
        name="Maximum Volume", default=0.75, min=-float('inf'), max=float('inf')
    )

    # Timescale property
    ambf_soft_body_enable_timescale: BoolProperty(
        name="Enable Timescale", default=False
    )
    ambf_soft_body_timescale: FloatProperty(
        name="Timescale", default=1.0, min=-float('inf'), max=float('inf')
    )

    # Iterations properties
    ambf_soft_body_enable_velocity_iterations: BoolProperty(
        name="Enable Velocity Iterations", default=False
    )
    ambf_soft_body_velocity_iterations: IntProperty(
        name="Velocity Iterations", default=10, min=0, max=1000000
    )
    ambf_soft_body_enable_position_iterations: BoolProperty(
        name="Enable Position Iterations", default=False
    )
    ambf_soft_body_position_iterations: IntProperty(
        name="Position Iterations", default=10, min=0, max=1000000
    )
    ambf_soft_body_enable_deformation_iterations: BoolProperty(
        name="Enable Deformation Iterations", default=False
    )
    ambf_soft_body_deformation_iterations: IntProperty(
        name="Deformation Iterations", default=10, min=0, max=1000000
    )
    ambf_soft_body_enable_collision_iterations: BoolProperty(
        name="Enable Collision Iterations", default=False
    )
    ambf_soft_body_collision_iterations: IntProperty(
        name="Collision Iterations", default=10, min=0, max=1000000
    )

    # Flags, bending, and cutting properties
    ambf_soft_body_enable_flags: BoolProperty(
        name="Enable Flags", default=False
    )
    ambf_soft_body_flags: IntProperty(
        name="Flags", default=0, min=0, max=1000000
    )
    ambf_soft_body_enable_bending_constraint: BoolProperty(
        name="Enable Bending Constraint", default=False
    )
    ambf_soft_body_bending_constraint: IntProperty(
        name="Bending Constraint", default=0, min=0, max=1000000
    )
    ambf_soft_body_enable_cutting_enabled: BoolProperty(
        name="Enable Cutting", default=False
    )
    ambf_soft_body_cutting_enabled: BoolProperty(
        name="Cutting Capability", default=False
    )
    ambf_soft_body_enable_clusters: BoolProperty(
        name="Enable Clusters", default=False
    )
    ambf_soft_body_clusters: IntProperty(
        name="Clusters", default=0, min=0, max=1000000
    )

    # Fixed nodes
    ambf_soft_body_enable_fixed_nodes: BoolProperty(
        name="Enable Fixed Nodes", default=False
    )
    ambf_soft_body_fixed_nodes: CollectionProperty(
        type=FixedNodeItem,
        name="Fixed Nodes",
        description="List of Fixed Node Indices"
    )
    
    ambf_soft_body_randomize_constraints: BoolProperty(
        name="Randomize Constraints", default=False
    )


class RigidBodyProperties(PropertyGroup):
    Object.ambf_rigid_body_namespace = StringProperty(name="Namespace", default="")

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

# TODO: Fix type expression
class SensorArrayItem(PropertyGroup):
    bl_idname = "OBJECT_PT_sensor_array_item"
    bl_label = "Sensor Array Item"
    
    offset: FloatVectorProperty(
        name="Offset",
        size=3,
        subtype='XYZ',
        default=(0.0, 0.0, 0.0)
    )
    direction: FloatVectorProperty(
        name="Direction",
        size=3,
        subtype='XYZ',
        default=(0.0, -1.0, 0.0)
    )

# TODO: Fix type expression
class SensorProperties(PropertyGroup):
    bl_idname = "OBJECT_PT_sensor_properties"
    bl_label = "Sensor Properties"

    ambf_sensor_type: EnumProperty(
        items=[
            ('Proximity', 'Proximity', '', '', 0),
            ('Resistance', 'Resistance', '', '', 1),
            ('Contact', 'Contact', '', '', 2),
        ],
        name="Type",
        default='Proximity'
    )
    
    ambf_sensor_enable_frequency: BoolProperty(
        name="Enable Frequency",
        default=False
    )

    ambf_sensor_frequency: FloatProperty(
        name="Frequency", 
        default=1.0, 
        min=0.0
    )
    
    ambf_object_visible: BoolProperty(
        name="Visible",
        default=False
    )
    
    ambf_object_visible_size: FloatProperty(
        name="Visible Size",
        default=1.0,
        min=0.0
    )

    # Proximity Sensor Properties
    ambf_sensor_range: FloatProperty(
        name="Range",
        default=0.1,
        min=0.0
    )
    
    # Sensor Array Collection for Proximity Sensor
    ambf_sensor_array: CollectionProperty(
        type=SensorArrayItem,
        name="Array"
    )

    # Resistance Sensor Properties
    ambf_sensor_friction_static: FloatProperty(
        name="Static Friction",
        default=0.0,
        min=0.0
    )

    ambf_sensor_friction_damping: FloatProperty(
        name="Damping Friction",
        default=0.0,
        min=0.0
    )

    ambf_sensor_friction_dynamic: FloatProperty(
        name="Dynamic Friction",
        default=0.0,
        min=0.0
    )

    ambf_sensor_friction_variable: BoolProperty(
        name="Variable Friction",
        default=False
    )

    ambf_sensor_contact_area: FloatProperty(
        name="Contact Area",
        default=0.0,
        min=0.0
    )

    ambf_sensor_contact_stiffness: FloatProperty(
        name="Contact Stiffness",
        default=0.0,
        min=0.0
    )

    ambf_sensor_contact_damping: FloatProperty(
        name="Contact Damping",
        default=0.0,
        min=0.0
    )

    # Contact Sensor Properties
    ambf_sensor_distance_threshold: FloatProperty(
        name="Distance Threshold",
        default=0.0,
        min=0.0
    )

    ambf_sensor_process_contact_details: BoolProperty(
        name="Process Contact Details",
        default=False
    )

custom_classes = (
        AMBF_OT_toggle_low_res_mesh_modifiers_visibility,
        AMBF_PG_CollisionShapePropGroup,

        # Workspace Operators
        AMBF_OT_cleanup_all,
        AMBF_OT_ambf_rigid_body_cleanup,
        AMBF_OT_ambf_constraint_cleanup,
        AMBF_OT_ambf_collision_shape_cleanup,
        AMBF_OT_select_all,
        AMBF_OT_hide_passive_joints,
        AMBF_OT_hide_all_joints,

        # Modelling Helpers
        # TODO: naming with OBJECT_OT_ prefix, OBJECT_PT_ prefix
        OBJECT_PT_Geometry_Main_Panel,
        OBJECT_PT_Camera_Panel,
        OBJECT_PT_Workspace_Main_Panel,
        AddCube,
        AddSphere,
        AddCylinder,
        DeleteAllObjects,
        DeleteSelectedObjects,
        SetActiveCamera,
        CleanWorkspace,

        AMBF_OT_remove_low_res_mesh_modifiers,
        AMBF_OT_generate_low_res_mesh_modifiers,
        AMBF_OT_generate_ambf_file,
        AMBF_OT_save_meshes,
        AMBF_OT_load_ambf_file,
        AMBF_OT_launch_ambf_simulator,

        AMBF_OT_create_joint,
        AMBF_OT_create_sensor,
        AMBF_OT_add_sensor_array_item,
        AMBF_OT_remove_sensor_array_item,
        AMBF_OT_create_actuator,

        AMBF_OT_remove_object_namespaces,
        AMBF_OT_estimate_inertial_offsets,
        AMBF_OT_estimate_shape_offsets,
        AMBF_OT_ambf_collision_shape_add,
        AMBF_OT_ambf_collision_shape_remove,
        AMBF_OT_ambf_move_collision_mesh_to_body_origin,
        AMBF_OT_ambf_collision_mesh_use_current_location,
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

        OBJECT_OT_ClearFidexNodes,
        OBJECT_OT_AddFixedNodesFromSelection,
        OBJECT_OT_RemoveFixedNode,

        AMBF_OT_ambf_rigid_body_activate,
        AMBF_OT_ambf_ghost_object_activate,
        AMBF_OT_ambf_soft_body_activate,
        AMBF_OT_ambf_constraint_activate,
        AMBF_OT_create_camera,
        AMBF_OT_create_light,
        AMBF_OT_ambf_camera_activate,
        AMBF_OT_ambf_light_activate,
        AMBF_OT_ambf_sensor_activate,
        AMBF_OT_ambf_actuator_activate, 

        OBJECT_PT_DebuggerPanel,

        AMBF_PT_main_panel,
        AMBF_PT_ambf_rigid_body,
        AMBF_PT_ambf_ghost_object,
        AMBF_PT_ambf_soft_body,
        AMBF_PT_ambf_constraint,
        AMBF_PT_ambf_actuator,
        AMBF_PT_ambf_sensor,
        AMBF_PT_ambf_camera,
        AMBF_PT_ambf_light,

        # Collection Operators
        # TODO: naming with COLLECTION_OT_ prefix, COLLECTION_PT_ prefix
        CollectionSelectorPanel,
        ToggleCollectionSelectionOperator,
        ActivateCollectionOperator,
        DeleteCollectionOperator,
        AddCollectionOperator,

        # ROS
        # TODO: naming with ROS_OT_ prefix, ROS_PT_ prefix
        ROS_PT_Service_Panel,
        ServiceROS,
        StopServiceROS,
        ROS_PT_LinearVelocityPanel,
        SendLinearVelocity,
        ROS_PT_AngularVelocityPanel,
        SendAngularVelocity,

        # AMBF Obj Properties
        RigidBodyProperties,
        GhostObjectProperties,
        FixedNodeItem,
        SoftBodyProperties,
        SensorArrayItem,
        SensorProperties
)

def register():

    from bpy.utils import register_class
    for cls in custom_classes:
        register_class(cls)

    # TODO: organize each property group
    
    ''' COLLECTION PROPERTIES'''
    Scene.selected_collections = bpy.props.StringProperty(
        name="Selected Collections",
        description="Comma-separated list of selected collection names",
        default="",
        update=update_selected_collections
    )

    Scene.new_collection_name = bpy.props.StringProperty(
    name="New Collection Name",
    description="Name of the new collection",
    default="New Collection"
    )

    Scene.active_collection_name = bpy.props.StringProperty(
    name="Active Collection Name",
    description="Name of the active collection",
    default=""
    )
    
    bpy.app.handlers.depsgraph_update_post.append(ensure_active_collection)
            
    Object.ambf_object_type = EnumProperty \
            (
            name="Object Type",
            items=
            [
                ('NONE', 'None', '', '', 0),
                ('RIGID_BODY', 'RIGID_BODY', '', '', 1),
                ('GHOST_OBJECT', 'GHOST_OBJECT', '', '', 2),
                ('SOFT_BODY', 'SOFT_BODY', '', '', 3),
                ('CONSTRAINT', 'CONSTRAINT', '', '', 4),
                ('COLLISION_SHAPE', 'COLLISION_SHAPE', '', '', 5),
                ('CAMERA', 'CAMERA', '', '', 6),
                ('LIGHT', 'LIGHT', '', '', 7), 
                ('SENSOR', 'SENSOR', '', '', 8),
                ('ACTUATOR', 'ACTUATOR', '', '', 9),
            ],
            default='NONE'
        )
    
    ''' LIGHT PROPERTIES'''
    Object.ambf_light_namespace = StringProperty(name="Namespace", default="")

    Object.ambf_light_spot_exponent = bpy.props.FloatProperty(
        name="Spot Exponent",
        description="Custom spot exponent for AMBF lights",
        default=1.0,
        min=0.0,
        max=128.0
    )

    Object.ambf_light_shadow_quality = bpy.props.IntProperty(
        name="Shadow Quality",
        description="Custom shadow quality for AMBF lights",
        default=1,
        min=0,
        max=10
    )

    Object.ambf_light_cutoff_angle = bpy.props.FloatProperty(
        name="Cutoff Angle",
        description="Custom cutoff angle for AMBF lights",
        default=1.7,
        min=0.0,
        max=3.14159  # Maximum value is pi radians (~180 degrees)
    )

    Object.ambf_light_constant_attenuation = bpy.props.FloatProperty(
        name="Constant Attenuation",
        description="Custom constant attenuation for AMBF lights",
        default=1.0,
        min=0.0,
        max=10.0
    )

    Object.ambf_light_type = EnumProperty(
        name="Light Type",
        description="AMBF light type",
        items=[
            ('SPOT', 'Spotlight', 'Directional spot light', '', 0),
        ],
        default='SPOT',
        update=ambf_light_type_update_cb
    )

    Object.ambf_light_parent = StringProperty(
        name="Parent Body",
        description="Name of the parent AMBF body this light is attached to",
        default=""
    )

    Object.ambf_light_attenuation_constant = FloatProperty(
        name="Constant",
        description="Constant attenuation factor",
        default=1.0, min=0.0, max=10.0, precision=3
    )
    Object.ambf_light_attenuation_linear = FloatProperty(
        name="Linear",
        description="Linear attenuation factor",
        default=0.0, min=0.0, max=10.0, precision=3
    )
    Object.ambf_light_attenuation_quadratic = FloatProperty(
        name="Quadratic",
        description="Quadratic attenuation factor",
        default=0.0, min=0.0, max=10.0, precision=4
    )

    ''' CAMERA PROPERTIES '''
    Object.ambf_camera_clip_start = FloatProperty(
        name="Near Plane",
        description="Near clipping plane distance",
        default=0.1,
        min=0.001,
        step=1,
        precision=3,
        update=ambf_camera_clip_start_update_cb
    )

    Object.ambf_camera_clip_end = FloatProperty(
        name="Far Plane",
        description="Far clipping plane distance",
        default=1000.0,
        min=0.001,
        step=100,
        precision=1,
        update=ambf_camera_clip_end_update_cb
    )

    Object.ambf_camera_monitor = bpy.props.IntProperty(
        name="Monitor",
        description="Display monitor index (0-indexed) for this camera window",
        default=0,
        min=0,
        max=10
    )

    Object.ambf_camera_publish_image = BoolProperty(
        name="Publish Image",
        description="Publish camera image over ROS",
        default=False
    )

    Object.ambf_camera_publish_image_interval = bpy.props.IntProperty(
        name="Publish Image Interval",
        description="Publish every N simulation steps",
        default=1,
        min=1
    )

    Object.ambf_camera_publish_image_width = bpy.props.IntProperty(
        name="Image Width",
        description="Published image width in pixels",
        default=640,
        min=1
    )

    Object.ambf_camera_publish_image_height = bpy.props.IntProperty(
        name="Image Height",
        description="Published image height in pixels",
        default=480,
        min=1
    )

    Object.ambf_camera_publish_depth = BoolProperty(
        name="Publish Depth",
        description="Publish depth map over ROS",
        default=False
    )

    Object.ambf_camera_publish_depth_interval = bpy.props.IntProperty(
        name="Publish Depth Interval",
        description="Publish depth every N simulation steps",
        default=1,
        min=1
    )

    Object.ambf_camera_publish_depth_width = bpy.props.IntProperty(
        name="Depth Width",
        description="Published depth image width in pixels",
        default=640,
        min=1
    )

    Object.ambf_camera_publish_depth_height = bpy.props.IntProperty(
        name="Depth Height",
        description="Published depth image height in pixels",
        default=480,
        min=1
    )

    ''' CAMERA INTRINSICS '''
    Object.ambf_camera_enable_intrinsics = BoolProperty(
        name="Enable Camera Intrinsics",
        description="Override camera projection with intrinsic parameters",
        default=False
    )
    Object.ambf_camera_intrinsics_fx = FloatProperty(
        name="fx", description="Focal length X (pixels)", default=500.0, min=0.0, precision=3
    )
    Object.ambf_camera_intrinsics_fy = FloatProperty(
        name="fy", description="Focal length Y (pixels)", default=500.0, min=0.0, precision=3
    )
    Object.ambf_camera_intrinsics_cx = FloatProperty(
        name="cx", description="Principal point X (pixels)", default=320.0, min=0.0, precision=3
    )
    Object.ambf_camera_intrinsics_cy = FloatProperty(
        name="cy", description="Principal point Y (pixels)", default=240.0, min=0.0, precision=3
    )
    Object.ambf_camera_intrinsics_s = FloatProperty(
        name="s", description="Skew coefficient", default=0.0, precision=4
    )

    ''' CAMERA PROJECTION MATRIX '''
    Object.ambf_camera_enable_projection_matrix = BoolProperty(
        name="Enable Projection Matrix",
        description="Override camera projection with a custom 4x4 matrix",
        default=False
    )
    Object.ambf_camera_projection_matrix = FloatVectorProperty(
        name="Projection Matrix",
        description="Custom 4x4 camera projection matrix (row-major)",
        size=16,
        default=(
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        ),
        precision=4
    )

    ''' GHOST OBJECT PROPERTIES'''
    Object.ambf_ghost_object_properties = PointerProperty(type=GhostObjectProperties)

    ''' SOFT BODY PROPERTIES'''
    Object.ambf_soft_body_properties = PointerProperty(type=SoftBodyProperties)
    
    ''' RIGID BODY PROPERTIES'''
    Object.ambf_rigid_body_properties = PointerProperty(type=RigidBodyProperties)
    
    ''' COMMON OBJECT PROPERTIES'''
    Object.ambf_body_angular_inertial_offset = FloatVectorProperty \
            (
            name='Angular Inertial Offset',
            default=(0.0, 0.0, 0.0),
            options={'PROPORTIONAL'},
            subtype='EULER',
        )
    
    Object.ambf_body_linear_inertial_offset = FloatVectorProperty \
            (
            name='Linear Inertial Offset',
            default=(0.0, 0.0, 0.0),
            options={'PROPORTIONAL'},
            update=collision_shape_offset_update_cb,
            subtype='XYZ',
        )
    
    Object.ambf_body_mass = FloatProperty(name="mass", default=1.0, min=0.0001)

    Object.ambf_body_passive = BoolProperty(name="Is Passive?", default=False,
                                                  description="If passive. this body will not be spawned as an AMBF communication object")
    
    Object.ambf_body_transparency = FloatProperty(name="Transparency", default=1.0, min=0.0, max=1.0)

    Object.ambf_scale = FloatProperty(name="Scale", default=1.0, min=0.0001)
    
    # Ambient color component
    Object.ambf_object_ambient_level = FloatProperty(name="Ambient Level", min=0, max=1, default=1.0) # Level = 1.0

    # Diffuse color component
    Object.ambf_object_diffuse_color  = FloatVectorProperty(
        name="Diffuse Color",
        subtype="COLOR",
        size=3,
        min=0,
        max=1,
        default=(0.5, 0.5, 0.5) # r=0.5, g=0.5, b=0.5
    )

    # Specular color component
    Object.ambf_object_specular_color = FloatVectorProperty(
        name="Specular Color",
        subtype="COLOR",
        size=3,
        min=0,
        max=1,
        default=(0.5, 0.05, 0.05)  # r=0.5, g=0.05, b=0.05
    )

    Object.ambf_object_override_gravity = BoolProperty \
            (
            name="Override Object Gravity",
            default=False,
            description="Override this objects gravity (default = False)",
        )

    Object.ambf_object_gravity = FloatVectorProperty \
            (
            name='Object Gravity',
            default=(0.0, 0.0, -9.8),
            options={'PROPORTIONAL'},
            subtype='XYZ',
        )

    Object.ambf_object_visible = BoolProperty(name="Visible", default=True, description='Show this object in AMBF')
    
    Object.ambf_object_visible_size = FloatProperty(name="Visible Size", default=0.005, min=0.0)

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
    
    Object.ambf_use_separate_collision_mesh = BoolProperty(name='Use Separate Collision Mesh', default=False,
                                                           description='Use a separate mesh for collision')

    Object.ambf_collision_mesh = PointerProperty(name="Collision Mesh", type=Object,
                                                 description='A separate mesh to be used for collision detection')

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
    
    ''' CONSTRAINT PROPERTIES'''
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
    
    ''' ACTUATOR PROPERTIES'''
    #TODO: Add Actuator Properties

    ''' SENSOR PROPERTIES'''
    Object.ambf_sensor_properties = bpy.props.PointerProperty(type=SensorProperties)
    
    ''' SCENARIO PROPERTIES'''
    Scene.ambf_adf_path = StringProperty \
            (
            name="Config (Save To)",
            default="",
            description="Define the root path of the project",
            subtype='FILE_PATH'
        )

    Scene.ambf_simulator_path = StringProperty(
        name="Simulator Path",
        description="Full path to the ambf_simulator executable",
        default="/home/yoheen/ros_ambf_ws/install/AMBF/bin/ambf_simulator",
        subtype='FILE_PATH'
    )

    Scene.ambf_launch_adf_path = StringProperty(
        name="ADF to Launch",
        description="ADF file to open in the AMBF simulator. Auto-filled when you save an ADF.",
        default="",
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

    Scene.ambf_model_override_gravity = BoolProperty \
            (
            name="Override Model Gravity",
            default=False,
            description="Override this models gravity (default = False)",
        )

    Scene.ambf_model_gravity = FloatVectorProperty\
            (
            name='Model Gravity',
            default=(0.0, 0.0, -9.8),
            options={'PROPORTIONAL'},
            subtype='XYZ',
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
    
    """ ROS PROPERTIES """
    Scene.stop_service = bpy.props.BoolProperty(default=False)
    Scene.selected_object_velocity_control = bpy.props.StringProperty()

    Scene.angular_velocity_x = bpy.props.FloatProperty(default=0.0)
    Scene.angular_velocity_y = bpy.props.FloatProperty(default=0.0)
    Scene.angular_velocity_z = bpy.props.FloatProperty(default=0.0)

    Scene.linear_velocity_x = bpy.props.FloatProperty(default=0.0)
    Scene.linear_velocity_y = bpy.props.FloatProperty(default=0.0)
    Scene.linear_velocity_z = bpy.props.FloatProperty(default=0.0)

    Object.ambf_collision_shape_prop_collection = CollectionProperty(type=AMBF_PG_CollisionShapePropGroup)

def unregister():
    from bpy.utils import unregister_class
    for cls in reversed(custom_classes):
        unregister_class(cls)

    ''' COLLECTION PROPERTIES '''
    del bpy.types.Scene.selected_collections
    del bpy.types.Scene.new_collection_name
    del bpy.types.Scene.active_collection_name

    ''' LIGHT PROPERTIES '''
    del bpy.types.Object.ambf_light_namespace
    del bpy.types.Object.ambf_light_spot_exponent
    del bpy.types.Object.ambf_light_shadow_quality
    del bpy.types.Object.ambf_light_cutoff_angle
    del bpy.types.Object.ambf_light_constant_attenuation
    del bpy.types.Object.ambf_light_type
    del bpy.types.Object.ambf_light_parent
    del bpy.types.Object.ambf_light_attenuation_constant
    del bpy.types.Object.ambf_light_attenuation_linear
    del bpy.types.Object.ambf_light_attenuation_quadratic

    ''' CAMERA PROPERTIES '''
    del bpy.types.Object.ambf_camera_clip_start
    del bpy.types.Object.ambf_camera_clip_end
    del bpy.types.Object.ambf_camera_monitor
    del bpy.types.Object.ambf_camera_publish_image
    del bpy.types.Object.ambf_camera_publish_image_interval
    del bpy.types.Object.ambf_camera_publish_image_width
    del bpy.types.Object.ambf_camera_publish_image_height
    del bpy.types.Object.ambf_camera_publish_depth
    del bpy.types.Object.ambf_camera_publish_depth_interval
    del bpy.types.Object.ambf_camera_publish_depth_width
    del bpy.types.Object.ambf_camera_publish_depth_height
    del bpy.types.Object.ambf_camera_enable_intrinsics
    del bpy.types.Object.ambf_camera_intrinsics_fx
    del bpy.types.Object.ambf_camera_intrinsics_fy
    del bpy.types.Object.ambf_camera_intrinsics_cx
    del bpy.types.Object.ambf_camera_intrinsics_cy
    del bpy.types.Object.ambf_camera_intrinsics_s
    del bpy.types.Object.ambf_camera_enable_projection_matrix
    del bpy.types.Object.ambf_camera_projection_matrix

    ''' GHOST OBJECT PROPERTIES '''
    del bpy.types.Object.ambf_ghost_object_properties

    ''' SOFT BODY PROPERTIES '''
    del bpy.types.Object.ambf_soft_body_properties

    ''' RIGID BODY PROPERTIES '''
    del bpy.types.Object.ambf_rigid_body_properties

    ''' COMMON OBJECT PROPERTIES '''
    del bpy.types.Object.ambf_body_mass
    del bpy.types.Object.ambf_body_passive
    del bpy.types.Object.ambf_scale
    del bpy.types.Object.ambf_body_transparency
    del bpy.types.Object.ambf_object_override_gravity
    del bpy.types.Object.ambf_object_gravity
    del bpy.types.Object.ambf_object_visible
    del bpy.types.Object.ambf_object_visible_size
    del bpy.types.Object.ambf_collision_margin_enable
    del bpy.types.Object.ambf_collision_show_shapes_per_object
    del bpy.types.Object.ambf_collision_margin
    del bpy.types.Object.ambf_collision_type
    del bpy.types.Object.ambf_use_separate_collision_mesh
    del bpy.types.Object.ambf_collision_mesh
    del bpy.types.Object.ambf_collision_mesh_type
    del bpy.types.Object.ambf_collision_groups

    ''' CONSTRAINT PROPERTIES '''
    del bpy.types.Object.ambf_constraint_type
    del bpy.types.Object.ambf_object_parent
    del bpy.types.Object.ambf_object_child
    del bpy.types.Object.ambf_constraint_name
    del bpy.types.Object.ambf_constraint_enable_controller_gains
    del bpy.types.Object.ambf_constraint_controller_p_gain
    del bpy.types.Object.ambf_constraint_controller_i_gain
    del bpy.types.Object.ambf_constraint_controller_d_gain
    del bpy.types.Object.ambf_constraint_damping
    del bpy.types.Object.ambf_constraint_stiffness
    del bpy.types.Object.ambf_constraint_equilibrium_point
    del bpy.types.Object.ambf_constraint_limits_enable
    del bpy.types.Object.ambf_constraint_passive
    del bpy.types.Object.ambf_constraint_enable_feedback
    del bpy.types.Object.ambf_constraint_limits_lower
    del bpy.types.Object.ambf_constraint_limits_higher
    del bpy.types.Object.ambf_constraint_max_motor_impulse
    del bpy.types.Object.ambf_constraint_axis
    del bpy.types.Object.ambf_constraint_cone_twist_limits
    del bpy.types.Object.ambf_constraint_six_dof_limits_low_angular
    del bpy.types.Object.ambf_constraint_six_dof_limits_high_angular
    del bpy.types.Object.ambf_constraint_six_dof_limits_low_linear
    del bpy.types.Object.ambf_constraint_six_dof_limits_high_linear
    del bpy.types.Object.ambf_constraint_six_dof_stiffness_angular
    del bpy.types.Object.ambf_constraint_six_dof_stiffness_linear
    del bpy.types.Object.ambf_constraint_six_dof_damping_angular
    del bpy.types.Object.ambf_constraint_six_dof_damping_linear
    del bpy.types.Object.ambf_constraint_six_dof_equilibrium_angular
    del bpy.types.Object.ambf_constraint_six_dof_equilibrium_linear
    del bpy.types.Object.ambf_constraint_controller_output_type

    ''' SENSOR PROPERTIES '''
    del bpy.types.Object.ambf_sensor_properties

    ''' SCENARIO PROPERTIES '''
    del bpy.types.Scene.ambf_adf_path
    del bpy.types.Scene.ambf_simulator_path
    del bpy.types.Scene.ambf_launch_adf_path
    del bpy.types.Scene.ambf_meshes_path
    del bpy.types.Scene.ambf_meshes_save_type
    del bpy.types.Scene.ambf_mesh_max_vertices
    del bpy.types.Scene.ambf_model_override_gravity
    del bpy.types.Scene.ambf_model_gravity
    del bpy.types.Scene.ambf_ignore_inter_collision
    del bpy.types.Scene.ambf_precision
    del bpy.types.Scene.ambf_load_adf_filepath
    del bpy.types.Scene.ambf_namespace
    del bpy.types.Scene.ambf_show_collision_shapes
    del bpy.types.Scene.ambf_enable_forced_cleanup
    del bpy.types.Scene.ambf_save_high_res
    del bpy.types.Scene.ambf_save_low_res
    del bpy.types.Scene.ambf_save_textures
    del bpy.types.Scene.ambf_save_selection_only

    ''' ROS PROPERTIES '''
    del bpy.types.Scene.stop_service
    del bpy.types.Scene.selected_object_velocity_control
    del bpy.types.Scene.linear_velocity_x
    del bpy.types.Scene.linear_velocity_y
    del bpy.types.Scene.linear_velocity_z
    del bpy.types.Scene.angular_velocity_x
    del bpy.types.Scene.angular_velocity_y
    del bpy.types.Scene.angular_velocity_z
    del bpy.types.Object.ambf_collision_shape_prop_collection

    bpy.app.handlers.depsgraph_update_post.remove(ensure_active_collection)

if __name__ == "__main__":
    print("\n\n########## STARTING ##########")
    register()
    #unregister()


""" ********** TODO ********** 
- Make each property group only responsible for the object type;
Fix object property problem that (Object.blablabla) will be shared between all objects
"""
