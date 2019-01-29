# Author: Adnan Munawar
# Email: amunawar@wpi.edu
# Lab: aimlab.wpi.edu
bl_info = {
    "name": "AF Multi-Body Config Creator",
    "author": "Adnan Munawar",
    "version": (0, 1),
    "blender": (2, 79, 0),
    "location": "View3D > Add > Mesh > AF Multi-Body",
    "description": "Helps Generate AF Multi-Body Config File and Saves both High and Low Resolution(Collision) Meshes",
    "warning": "",
    "wiki_url": "https://github.com/adnanmunawar/af_multibody_config",
    "category": "AF Multi-Body",
    }

import bpy
import math
import yaml
import os
from pathlib import Path
import mathutils
from enum import Enum
from collections import OrderedDict
from datetime import datetime


# https://stackoverflow.com/questions/31605131/dumping-a-dictionary-to-a-yaml-file-while-preserving-order/31609484
def represent_dictionary_order(self, dict_data):
    return self.represent_mapping('tag:yaml.org,2002:map', dict_data.items())


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
        v[i] = round(v[i], 3)
    return v


# https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d/897677#897677
def rot_matrix_from_vecs(v1, v2):
    out = mathutils.Matrix.Identity(3)
    vcross = v1.cross(v2)
    vdot = v1.dot(v2)
    rot_angle = v1.angle(v2)
    if 1.0 - vdot < 0.1:
        return out
    elif 1.0 + vdot < 0.1:
        # This is a more involved case, find out the orthogonal vector to vecA
        nx = mathutils.Vector([1, 0, 0])
        temp_ang = v1.angle(nx)
        if 0.1 < abs(temp_ang) < 3.13:
            axis = v1.cross(nx)
            out = out.Rotation(rot_angle, 3, axis)
        else:
            ny = mathutils.Vector([0, 1, 0])
            axis = v1.cross(ny)
            out = out.Rotation(rot_angle, 3, axis)
    else:
        skew_v = skew_mat(vcross)
        out = mathutils.Matrix.Identity(3) + skew_v + skew_v * skew_v * ((1 - vdot) / (vec_norm(vcross) ** 2))
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


# Body Template for the some commonly used of afBody's data
class BodyTemplate:
    def __init__(self):
        self._afmb_data = OrderedDict()
        self._afmb_data['name'] = ""
        self._afmb_data['mesh'] = ""
        self._afmb_data['mass'] = 0.0
        self._afmb_data['inertia'] = {'ix': 0.0, 'iy': 0.0, 'iz': 0.0}
        self._afmb_data['scale'] = 1.0
        self._afmb_data['location'] = {'position': {'x': 0, 'y': 0, 'z': 0},
                                       'orientation': {'r': 0, 'p': 0, 'y': 0}}
        self._afmb_data['inertial offset'] = {'position': {'x': 0, 'y': 0, 'z': 0},
                                              'orientation': {'r': 0, 'p': 0, 'y': 0}}
        self._afmb_data['color'] = 'random'
        # Transform of Child Rel to Joint, which in inverse of t_c_j
        self.t_j_c = mathutils.Matrix()


# Joint Template for the some commonly used of afJoint's data
class JointTemplate:
    def __init__(self):
        self._afmb_data = OrderedDict()
        self._afmb_data['name'] = ''
        self._afmb_data['parent'] = ''
        self._afmb_data['child'] = ''
        self._afmb_data['parent axis'] = {'x': 0, 'y': 0.0, 'z': 1.0}
        self._afmb_data['parent pivot'] = {'x': 0, 'y': 0.0, 'z': 0}
        self._afmb_data['child axis'] = {'x': 0, 'y': 0.0, 'z': 1.0}
        self._afmb_data['child pivot'] = {'x': 0, 'y': 0.0, 'z': 0}
        self._afmb_data['joint limits'] = {'low': -1.2, 'high': 1.2}
        self._afmb_data['max motor impulse'] = 0.01


class GenerateAFMB(bpy.types.Operator):
    """Tooltip"""
    bl_idname = "myops.add_create_af_yaml"
    bl_label = "Write Multi-Body AF Config"

    def __init__(self):
        self._body_names_list = []
        self._joint_names_list = []
        self.body_name_prefix = 'BODY '
        self.joint_name_prefix = 'JOINT '
        self._afmb_yaml = None

    def execute(self, context):
        self.generate_afmb_yaml(context)
        return {'FINISHED'}

    def get_body_prefixed_name(self, urdf_body_str):
        return self.body_name_prefix + urdf_body_str

    def get_joint_prefixed_name(self, urdf_joint_str):
        return self.joint_name_prefix + urdf_joint_str

    # Courtesy of:
    # https://blender.stackexchange.com/questions/62040/get-center-of-geometry-of-an-object
    def compute_local_com(self, obj):
        vcos = [ v.co for v in obj.data.vertices ]
        find_center = lambda l: ( max(l) + min(l)) / 2
        x, y, z = [[v[i] for v in vcos] for i in range(3)]
        center = [find_center(axis) for axis in [x, y, z]]
        for i in range(0, 3):
            center[i] = center[i] * obj.scale[i]
        return center

    def generate_body_data(self, afmb_yaml, obj_handle):
        if obj_handle.hide is True:
            return
        body = BodyTemplate()
        body_data = body._afmb_data
        body_yaml_name = self.get_body_prefixed_name(obj_handle.name)
        output_mesh = bpy.context.scene['mesh_output_type']
        body_data['name'] = obj_handle.name
        world_pos = obj_handle.matrix_world.translation
        world_rot = obj_handle.matrix_world.to_euler()
        body_pos = body_data['location']['position']
        body_rot = body_data['location']['orientation']
        body_pos['x'] = round(world_pos.x, 3)
        body_pos['y'] = round(world_pos.y, 3)
        body_pos['z'] = round(world_pos.z, 3)
        body_rot['r'] = round(world_rot[0], 3)
        body_rot['p'] = round(world_rot[1], 3)
        body_rot['y'] = round(world_rot[2], 3)
        if obj_handle.type == 'EMPTY':
            body_data['mesh'] = ''

            if obj_handle.name in ['world', 'World', 'WORLD']:
                body_data['mass'] = 0

            else:
                body_data['mass'] = 0.1
                body_data['inertia'] = {'ix': 0.01, 'iy': 0.01, 'iz': 0.01}

        elif obj_handle.type == 'MESH':

            if obj_handle.rigid_body:
                body_data['mass'] = round(obj_handle.rigid_body.mass, 3)

            del body_data['inertia']
            body_data['mesh'] = obj_handle.name + get_extension(output_mesh)
            body_com = self.compute_local_com(obj_handle)
            body_d_pos = body_data['inertial offset']['position']
            body_d_pos['x'] = round(body_com[0], 3)
            body_d_pos['y'] = round(body_com[1], 3)
            body_d_pos['z'] = round(body_com[2], 3)

            if obj_handle.data.materials:

                del body_data['color']
                body_data['color rgba'] = {'r': 1.0, 'g': 1.0, 'b': 1.0, 'a': 1.0}
                body_data['color rgba']['r'] = round(obj_handle.data.materials[0].diffuse_color[0], 4)
                body_data['color rgba']['g'] = round(obj_handle.data.materials[0].diffuse_color[1], 4)
                body_data['color rgba']['b'] = round(obj_handle.data.materials[0].diffuse_color[2], 4)
                body_data['color rgba']['a'] = round(obj_handle.data.materials[0].alpha, 4)

        afmb_yaml[body_yaml_name] = body_data
        self._body_names_list.append(body_yaml_name)

    def generate_joint_data(self, afmb_yaml, obj_handle):
        if obj_handle.rigid_body_constraint:
            if obj_handle.rigid_body_constraint.object1:
                if obj_handle.rigid_body_constraint.object1.hide is True:
                    return
            if obj_handle.rigid_body_constraint.object2:
                if obj_handle.rigid_body_constraint.object2.hide is True:
                    return

            if obj_handle.rigid_body_constraint.type in ['FIXED', 'HINGE', 'SLIDER']:
                constraint = obj_handle.rigid_body_constraint
                joint_template = JointTemplate()
                joint_data = joint_template._afmb_data
                if constraint.object1:
                    parent_obj_handle = constraint.object1
                    child_obj_handle = constraint.object2
                    joint_data['name'] = parent_obj_handle.name + "-" + child_obj_handle.name
                    joint_data['parent'] = self.get_body_prefixed_name(parent_obj_handle.name)
                    joint_data['child'] = self.get_body_prefixed_name(child_obj_handle.name)
                    parent_body_data = self._afmb_yaml[self.get_body_prefixed_name(parent_obj_handle.name)]
                    child_body_data = self._afmb_yaml[self.get_body_prefixed_name(child_obj_handle.name)]
                    parent_pivot, parent_axis = self.compute_parent_pivot_and_axis(
                        parent_obj_handle, child_obj_handle, obj_handle.rigid_body_constraint.type)
                    child_pivot = mathutils.Vector([0, 0, 0])
                    child_axis = mathutils.Vector([0, 0, 0])
                    if obj_handle.rigid_body_constraint.type == 'HINGE':
                        child_axis = mathutils.Vector([0, 0, 1])
                        if constraint.use_limit_ang_z:
                            joint_data['type'] = 'revolute'
                            higher_limit = constraint.limit_ang_z_upper
                            lower_limit = constraint.limit_ang_z_lower
                        else:
                            joint_data['type'] = 'continuous'
                    elif obj_handle.rigid_body_constraint.type == 'SLIDER':
                        joint_data['type'] = 'prismatic'
                        child_axis = mathutils.Vector([1, 0, 0])
                        higher_limit = constraint.limit_lin_x_upper
                        lower_limit = constraint.limit_lin_x_lower
                    elif obj_handle.rigid_body_constraint.type == 'FIXED':
                        joint_data['type'] = 'fixed'
                        child_axis = mathutils.Vector([0, 0, 1])

                    parent_pivot_data = joint_data["parent pivot"]
                    parent_axis_data = joint_data["parent axis"]
                    parent_pivot_data['x'] = round(parent_pivot.x, 3)
                    parent_pivot_data['y'] = round(parent_pivot.y, 3)
                    parent_pivot_data['z'] = round(parent_pivot.z, 3)
                    parent_axis_data['x'] = round(parent_axis.x, 3)
                    parent_axis_data['y'] = round(parent_axis.y, 3)
                    parent_axis_data['z'] = round(parent_axis.z, 3)
                    child_pivot_data = joint_data["child pivot"]
                    child_axis_data = joint_data["child axis"]
                    child_pivot_data['x'] = child_pivot.x
                    child_pivot_data['y'] = child_pivot.y
                    child_pivot_data['z'] = child_pivot.z
                    child_axis_data['x'] = child_axis.x
                    child_axis_data['y'] = child_axis.y
                    child_axis_data['z'] = child_axis.z

                    if joint_data['type'] == 'fixed':
                        del joint_data["joint limits"]

                    elif joint_data['type'] == 'continuous':
                        del joint_data["joint limits"]['low']
                        del joint_data["joint limits"]['high']

                    else:
                        joint_limit_data = joint_data["joint limits"]
                        joint_limit_data['low'] = round(lower_limit, 3)
                        joint_limit_data['high'] = round(higher_limit, 3)

                    # The use of pivot and axis does not fully define the connection and relative
                    # transform between two bodies it is very likely that we need an additional offset
                    # of the child body as in most of the cases of URDF's For this purpose, we calculate
                    # the offset as follows
                    r_c_p_afmb = rot_matrix_from_vecs(child_axis, parent_axis)
                    r_p_c_afmb = r_c_p_afmb.to_3x3().copy()
                    r_p_c_afmb.invert()

                    t_p_w = parent_obj_handle.matrix_world.copy()
                    r_w_p = t_p_w.to_3x3().copy()
                    r_w_p.invert()
                    r_c_w = child_obj_handle.matrix_world.to_3x3().copy()
                    r_c_p_blender = r_w_p * r_c_w

                    r_angular_offset = r_p_c_afmb * r_c_p_blender

                    offset_axis_angle = r_angular_offset.to_quaternion().to_axis_angle()

                    if abs(offset_axis_angle[1]) > 0.01:
                        # print '*****************************'
                        # print joint_data['name']
                        # print 'Joint Axis, '
                        # print '\t', joint.axis
                        # print 'Offset Axis'
                        # print '\t', offset_axis_angle[1]
                        offset_angle = round(offset_axis_angle[1], 3)
                        # offset_angle = round(offset_angle, 3)
                        # print 'Offset Angle: \t', offset_angle
                        print('OFFSET ANGLE', offset_axis_angle[1])
                        print('CHILD AXIS', child_axis)
                        print('OFFSET AXIS', offset_axis_angle)
                        print('DOT PRODUCT', parent_axis.dot(offset_axis_angle[0]))

                        if abs(1.0 - child_axis.dot(offset_axis_angle[0])) < 0.1:
                            joint_data['offset'] = offset_angle
                            # print ': SAME DIRECTION'
                        elif abs(1.0 + child_axis.dot(offset_axis_angle[0])) < 0.1:
                            joint_data['offset'] = -offset_angle
                            # print ': OPPOSITE DIRECTION'
                        else:
                            print('ERROR: SHOULD\'NT GET HERE')

                    joint_yaml_name = self.get_joint_prefixed_name(joint_data['name'])
                    afmb_yaml[joint_yaml_name] = joint_data
                    self._joint_names_list.append(joint_yaml_name)

    # Since changing the scale of the bodies directly impacts the rotation matrix, we have
    # to take that into account while calculating offset of child from parent using
    # transform manipulation
    def compute_parent_pivot_and_axis(self, parent, child, joint_type):
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
        t_c_p = t_w_p * t_c_w
        parent_pivot = t_c_p.translation
        if joint_type == 'HINGE':
            col_num = 2
        elif joint_type == 'SLIDER':
            col_num = 0
        elif joint_type == 'FIXED':
            col_num = 2
        # The third col of rotation matrix is the z axes of child in parent
        parent_axis = mathutils.Vector(t_c_p.col[col_num][0:3])
        return parent_pivot, parent_axis

    def generate_afmb_yaml(self, context):
        num_objs = len(bpy.data.objects)
        save_to = bpy.path.abspath(context.scene.afmb_yaml_conf_path)
        file_name = os.path.basename(save_to)
        save_path = os.path.dirname(save_to)
        if not file_name:
            file_name = 'default.yaml'
        output_file_name = os.path.join(save_path, file_name)
        output_file = open(output_file_name, 'w')
        print('Output filename is: ', output_file_name)

        # For inorder processing, set the bodies and joints tag at the top of the map
        self._afmb_yaml = OrderedDict()
        
        self._afmb_yaml['bodies'] = []
        self._afmb_yaml['joints'] = []
        print('SAVE PATH', bpy.path.abspath(save_path))
        print('AFMB CONFIG PATH', bpy.path.abspath(context.scene.afmb_yaml_mesh_path))
        rel_mesh_path = os.path.relpath(bpy.path.abspath(context.scene.afmb_yaml_mesh_path), bpy.path.abspath(save_path))

        self._afmb_yaml['high resolution path'] = rel_mesh_path + '/high_res/'
        self._afmb_yaml['low resolution path'] = rel_mesh_path + '/low_res/'

        self._afmb_yaml['ignore inter-collision'] = str(context.scene.ignore_inter_collision)

        for obj in bpy.data.objects:
            self.generate_body_data(self._afmb_yaml, obj)

        for obj in bpy.data.objects:
            self.generate_joint_data(self._afmb_yaml, obj)

        # Now populate the bodies and joints tag
        self._afmb_yaml['bodies'] = self._body_names_list
        self._afmb_yaml['joints'] = self._joint_names_list
        
        yaml.dump(self._afmb_yaml, output_file)

        header_str = "# AFMB Version: %s\n" \
                     "# Generated By: afmb_addon for Blender %s\n" \
                     "# Link: %s\n" \
                     "# Generated on: %s\n"\
                     % (str(bl_info['version']).replace(', ', '.'),
                        str(bl_info['blender']).replace(', ', '.'),
                        bl_info['wiki_url'],
                        datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
        prepend_comment_to_file(output_file_name, header_str)


# Courtesy: https://stackoverflow.com/questions/5914627/prepend-line-to-beginning-of-a-file
def prepend_comment_to_file(filename, comment):
    with open(filename,'r') as f:
        with open('tempfile.txt', 'w') as f2:
            f2.write(comment)
            f2.write(f.read())
    os.rename('tempfile.txt', filename)


def select_all_objects(select=True):
    # First deselect all objects
    for obj in bpy.data.objects:
        obj.select = select


class SaveMeshes(bpy.types.Operator):
    bl_idname = "myops.add_save_af_meshes"
    bl_label = "Save Meshes"

    def execute(self, context):
        self.save_meshes(context)
        return {'FINISHED'}

    # This recursive function is specialized to deal with
    # tree based hierarchy. In such cases we have to ensure
    # to move the parent to origin first and then its children successively
    # otherwise moving any parent after its child has been moved with move the
    # child as well
    def set_to_origin(self, p_obj, obj_name_mat_list):
        if p_obj.children is None:
            return
        obj_name_mat_list.append([p_obj.name, p_obj.matrix_world.copy()])
        # Since setting the world transform clears the embedded scale
        # of the object, we need to re-scale the object after putting it to origin
        scale_mat = mathutils.Matrix()
        scale_mat = scale_mat.Scale(p_obj.matrix_world.median_scale, 4)
        p_obj.matrix_world.identity()
        p_obj.matrix_world = scale_mat
        for c_obj in p_obj.children:
            self.set_to_origin(c_obj, obj_name_mat_list)

    # Since Blender exports meshes w.r.t world transform and not the
    # the local mesh transform, we explicitly push each object to origin
    # and remember its world transform for putting it back later on
    def set_all_meshes_to_origin(self):
        obj_name_mat_list = []
        for p_obj in bpy.data.objects:
            if p_obj.parent is None:
                self.set_to_origin(p_obj, obj_name_mat_list)
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

    def save_meshes(self, context):
        # First deselect all objects
        select_all_objects(False)

        save_path = bpy.path.abspath(context.scene.afmb_yaml_mesh_path)
        high_res_path = os.path.join(save_path, 'high_res/')
        low_res_path = os.path.join(save_path, 'low_res/')
        os.makedirs(high_res_path, exist_ok=True)
        os.makedirs(low_res_path, exist_ok=True)
        mesh_type = bpy.context.scene['mesh_output_type']

        mesh_name_mat_list = self.set_all_meshes_to_origin()

        for obj_handle in bpy.data.objects:
            # Mesh Type is .stl
            obj_handle.select = True
            if obj_handle.type == 'MESH':
                if mesh_type == MeshType.meshSTL.value:
                    obj_name = obj_handle.name + '.STL'
                    filename_high_res = os.path.join(high_res_path, obj_name)
                    filename_low_res = os.path.join(low_res_path, obj_name)
                    bpy.ops.export_mesh.stl(filepath=filename_high_res, use_selection=True, use_mesh_modifiers=False)
                    bpy.ops.export_mesh.stl(filepath=filename_low_res, use_selection=True, use_mesh_modifiers=True)
                elif mesh_type == MeshType.meshOBJ.value:
                    obj_name = obj_handle.name + '.OBJ'
                    filename_high_res = os.path.join(high_res_path, obj_name)
                    filename_low_res = os.path.join(low_res_path, obj_name)
                    bpy.ops.export_scene.obj(filepath=filename_high_res, use_selection=True, use_mesh_modifiers=False)
                    bpy.ops.export_scene.obj(filepath=filename_low_res, use_selection=True, use_mesh_modifiers=True)
                elif mesh_type == MeshType.mesh3DS.value:
                    obj_name = obj_handle.name + '.3DS'
                    filename_high_res = os.path.join(high_res_path, obj_name)
                    filename_low_res = os.path.join(low_res_path, obj_name)
                    # 3DS doesn't support supressing modifiers, so we explicitly
                    # toggle them to save as high res and low res meshes
                    # STILL BUGGY
                    for mod in obj_handle.modifiers:
                        mod.show_viewport = True
                    bpy.ops.export_scene.autodesk_3ds(filepath=filename_low_res, use_selection=True)
                    for mod in obj_handle.modifiers:
                        mod.show_viewport = True
                    bpy.ops.export_scene.autodesk_3ds(filepath=filename_high_res, use_selection=True)
                elif mesh_type == MeshType.meshPLY.value:
                    # .PLY export has a bug in which it only saves the mesh that is
                    # active in context of view. Hence we explicitly select this object
                    # as active in the scene on top of being selected
                    obj_name = obj_handle.name + '.PLY'
                    filename_high_res = os.path.join(high_res_path, obj_name)
                    filename_low_res = os.path.join(low_res_path, obj_name)
                    bpy.context.scene.objects.active = obj_handle
                    bpy.ops.export_mesh.ply(filepath=filename_high_res, use_mesh_modifiers=False)
                    bpy.ops.export_mesh.ply(filepath=filename_low_res, use_mesh_modifiers=True)
                    # Make sure to deselect the mesh
                    bpy.context.scene.objects.active = None

            obj_handle.select = False
        self.reset_meshes_to_original_position(mesh_name_mat_list)


class GenerateLowResMeshModifiers(bpy.types.Operator):
    bl_idname = "myops.generate_low_res_mesh_modifiers"
    bl_label = "Generate Low-Res Meshes"

    def execute(self, context):
        # First off, remove any existing Modifiers:
        bpy.ops.myops.remove_modifiers()

        # Now deselect all objects
        for obj in bpy.data.objects:
            obj.select = False

        vertices_max = context.scene.mesh_max_vertices
        # Select each object iteratively and generate its low-res mesh
        for obj in bpy.data.objects:
            if obj.type == 'MESH' and obj.hide is False:
                decimate_mod = obj.modifiers.new('decimate_mod', 'DECIMATE')
                if len(obj.data.vertices) > vertices_max:
                    reduction_ratio = vertices_max / len(obj.data.vertices)
                    decimate_mod.use_symmetry = False
                    decimate_mod.use_collapse_triangulate = True
                    decimate_mod.ratio = reduction_ratio
                    decimate_mod.show_viewport = True
        return {'FINISHED'}


class RemoveModifiers(bpy.types.Operator):
    bl_idname = "myops.remove_modifiers"
    bl_label = "Remove All Modifiers"

    def execute(self, context):
        for obj in bpy.data.objects:
            for mod in obj.modifiers:
                obj.modifiers.remove(mod)
        return {'FINISHED'}


class ToggleModifiersVisibility(bpy.types.Operator):
    bl_idname = "myops.toggle_modifiers_visibility"
    bl_label = "Toggle Modifiers Visibility"

    def execute(self, context):
        for obj in bpy.data.objects:
            for mod in obj.modifiers:
                mod.show_viewport = not mod.show_viewport
        return {'FINISHED'}


class LoadAFMB(bpy.types.Operator):
    bl_idname = "myops.load_afmb_yaml_config"
    bl_label = "Load AFMB YAML Config"

    def __init__(self):
        self._afmb = None
        # A dict of T_c_j frames for each body
        self._body_t_j_c = {}
        self._joint_additional_offset = {}
        # A dict for body name as defined in YAML File and the Name Blender gives
        # the body
        self._blender_remapped_body_names = {}
        self._high_res_path = ''
        self._low_res_path = ''
        self._context = None

    def get_qualified_path(self, path):
        filepath = Path(path)

        if filepath.is_absolute():
            return path
        else:
            afmb_filepath = Path(self._yaml_filepath)
            path = str(afmb_filepath.parent.joinpath(filepath))
            return path

    def load_body(self, body_name):
        body = self._afmb[body_name]
        # print(body['name'])
        if 'high resolution path' in body:
            body_high_res_path = self.get_qualified_path(body['high resolution path'])
        else:
            body_high_res_path = self._high_res_path
        af_name = body['name']
        # If body name is world. Check if a world body has already
        # been defined, and if it has been, ignore adding another world body
        if af_name in ['world', 'World', 'WORLD']:
            for temp_obj_handle in bpy.data.objects:
                if temp_obj_handle.type in ['MESH', 'EMPTY']:
                    if temp_obj_handle.name in ['world', 'World', 'WORLD']:
                        self._blender_remapped_body_names[body_name] = temp_obj_handle.name
                        self._body_t_j_c[body_name] = mathutils.Matrix()
                        return
        body_mesh_name = body['mesh']
        body_mass = body['mass']
        self._body_t_j_c[body_name] = mathutils.Matrix()

        body_location_xyz = {'x': 0, 'y': 0, 'z': 0}
        body_location_rpy = {'r': 0, 'p': 0, 'y': 0}

        if 'location' in body:
            if 'position' in body['location']:
                body_location_xyz = body['location']['position']
            if 'orientation' in body['location']:
                body_location_rpy = body['location']['orientation']

        mesh_filepath = Path(os.path.join(body_high_res_path, body_mesh_name))
        _is_empty_object = False

        if mesh_filepath.suffix in ['.stl', '.STL']:
            bpy.ops.import_mesh.stl(filepath=str(mesh_filepath.resolve()))

        elif mesh_filepath.suffix in ['.obj', '.OBJ']:
            bpy.ops.import_scene.obj(filepath=str(mesh_filepath.resolve()))

        elif mesh_filepath.suffix in ['.dae', '.DAE']:
            bpy.ops.wm.collada_import(filepath=str(mesh_filepath.resolve()))
            # If we are importing .dae meshes, they can import stuff other than meshes, such as cameras etc.
            # We should remove these extra things and only keep the meshes
            for temp_obj_handle in self._context.selected_objects:
                if temp_obj_handle.type == 'MESH':
                    obj_handle = temp_obj_handle
                    self._context.scene.objects.active = obj_handle
                else:
                    bpy.data.objects.remove(temp_obj_handle)

        elif mesh_filepath.suffix == '':
            bpy.ops.object.empty_add(type='PLAIN_AXES')
            _is_empty_object = True

        obj_handle = self._context.active_object
        bpy.ops.object.transform_apply(scale=True)
        obj_handle.name = af_name
        self._blender_remapped_body_names[body_name] = obj_handle.name

        if not _is_empty_object:
            if 'color rgba' in body:
                mat = bpy.data.materials.new(name=body_name + 'mat')
                mat.diffuse_color[0] = body['color rgba']['r']
                mat.diffuse_color[1] = body['color rgba']['g']
                mat.diffuse_color[2] = body['color rgba']['b']
                mat.use_transparency = True
                mat.transparency_method = 'Z_TRANSPARENCY'
                mat.alpha = body['color rgba']['a']
                obj_handle.data.materials.append(mat)

            bpy.ops.rigidbody.object_add()
            obj_handle.rigid_body.mass = body_mass

        obj_handle.matrix_world.translation[0] = body_location_xyz['x']
        obj_handle.matrix_world.translation[1] = body_location_xyz['y']
        obj_handle.matrix_world.translation[2] = body_location_xyz['z']
        obj_handle.rotation_euler = (body_location_rpy['r'],
                                     body_location_rpy['p'],
                                     body_location_rpy['y'])

    # print('Remapped Body Names: ', self._blender_remapped_body_names)

    def load_joint(self, joint_name):
        joint = self._afmb[joint_name]
        select_all_objects(False)
        self._context.scene.objects.active = None
        parent_body_name = joint['parent']
        child_body_name = joint['child']
        parent_body_data = self._afmb[parent_body_name]
        child_body_data = self._afmb[child_body_name]
        # Set joint type to blender appropriate name
        joint_type = 'HINGE'
        if 'type' in joint:
            if joint['type'] in ['hinge', 'revolute', 'continuous']:
                joint_type = 'HINGE'
            elif joint['type'] in ['prismatic', 'slider']:
                joint_type = 'SLIDER'
            elif joint['type'] in ['fixed', 'FIXED']:
                joint_type = 'FIXED'
        parent_obj_handle = bpy.data.objects[self._blender_remapped_body_names[parent_body_name]]
        child_obj_handle = bpy.data.objects[self._blender_remapped_body_names[child_body_name]]
        if 'parent pivot' in joint:
            parent_pivot_data = joint['parent pivot']
            parent_axis_data = joint['parent axis']
            if 'child pivot' in joint:
                child_pivot_data = joint['child pivot']
                child_axis_data = joint['child axis']
                # To fully define a child body's connection and pose in a parent body, just the joint pivots
                # and joint axes are not sufficient. We also need the joint offset which correctly defines
                # the initial pose of the child body in the parent body.
                offset_angle = 0.0
                if not self._context.scene.ignore_afmb_joint_offsets:
                    if 'offset' in joint:
                        offset_angle = joint['offset']
                # Transformation matrix representing parent in world frame
                t_p_w = parent_obj_handle.matrix_world.copy()
                # Parent's Joint Axis in parent's frame
                parent_axis = mathutils.Vector([parent_axis_data['x'], parent_axis_data['y'], parent_axis_data['z']])
                # Transformation of joint in parent frame
                P_j_p = mathutils.Matrix()
                # P_j_p = P_j_p * r_j_p
                P_j_p.translation = mathutils.Vector(
                    [parent_pivot_data['x'], parent_pivot_data['y'], parent_pivot_data['z']])
                child_axis = mathutils.Vector([child_axis_data['x'], child_axis_data['y'], child_axis_data['z']])
                # Rotation matrix representing child frame in parent frame
                r_c_p, r_c_p_angle = get_rot_mat_from_vecs(child_axis, parent_axis)
                # print ('r_c_p')
                # print(r_c_p)
                # Transformation of joint in child frame
                p_j_c = mathutils.Matrix()
                # p_j_c *= r_j_c
                p_j_c.translation = mathutils.Vector(
                    [child_pivot_data['x'], child_pivot_data['y'], child_pivot_data['z']])
                # print(p_j_c)
                # Transformation of child in joints frame
                P_c_j = p_j_c.copy()
                P_c_j.invert()
                # Offset along constraint axis
                t_c_offset_rot = mathutils.Matrix().Rotation(offset_angle, 4, parent_axis)
                # Transformation of child in parents frame
                t_c_p = t_p_w * P_j_p * t_c_offset_rot * r_c_p * P_c_j
                # Set the child body the pose calculated above
                child_obj_handle.matrix_world = t_c_p
                child_obj_handle.select = True
                parent_obj_handle.select = True
                self._context.scene.objects.active = parent_obj_handle
                bpy.ops.object.parent_set(keep_transform=True)
                self._context.scene.objects.active = child_obj_handle
                child_obj_handle.select = True
                bpy.ops.rigidbody.constraint_add(type=joint_type)
                child_obj_handle.rigid_body_constraint.object1 \
                    = bpy.data.objects[self._blender_remapped_body_names[parent_body_name]]
                child_obj_handle.rigid_body_constraint.object2 \
                    = bpy.data.objects[self._blender_remapped_body_names[child_body_name]]

                if 'joint limits' in joint:
                    if joint['type'] == 'revolute':
                        child_obj_handle.rigid_body_constraint.limit_ang_z_upper \
                            = joint['joint limits']['high'] + offset_angle
                        child_obj_handle.rigid_body_constraint.limit_ang_z_lower \
                            = joint['joint limits']['low'] + offset_angle
                        child_obj_handle.rigid_body_constraint.use_limit_ang_z = True
                    elif joint['type'] == 'prismatic':
                        child_obj_handle.rigid_body_constraint.limit_lin_x_upper = joint['joint limits']['high']
                        child_obj_handle.rigid_body_constraint.limit_lin_x_lower = joint['joint limits']['low']
                        child_obj_handle.rigid_body_constraint.use_limit_lin_x = True
                    elif joint['type'] == 'continuous':
                        # Do nothing, not enable the limits
                        pass

    def adjust_body_pivots_and_axes(self):
        for joint_name in self._afmb['joints']:
            joint = self._afmb[joint_name]
            if 'child pivot' in joint:
                child_body_name = joint['child']
                child_pivot_data = joint['child pivot']
                child_axis_data = joint['child axis']
                parent_axis_data = joint['parent axis']

                child_obj_handle = bpy.data.objects[self._blender_remapped_body_names[child_body_name]]

                joint_type = 'HINGE'
                if 'type' in joint:
                    if joint['type'] in ['hinge', 'revolute', 'continuous']:
                        joint_type = 'HINGE'
                    elif joint['type'] in ['prismatic', 'slider']:
                        joint_type = 'SLIDER'
                    elif joint['type'] in ['fixed', 'FIXED']:
                        joint_type = 'FIXED'

                # Universal Constraint Axis
                constraint_axis = mathutils.Vector([0, 0, 1])
                # If check is enabled, set the appropriate constraint axis
                if joint_type == 'HINGE':
                    constraint_axis = mathutils.Vector([0, 0, 1])
                elif joint_type == 'SLIDER':
                    constraint_axis = mathutils.Vector([1, 0, 0])
                elif joint_type == 'FIXED':
                    constraint_axis = mathutils.Vector([0, 0, 1])

                # Child's Joint Axis in child's frame
                child_axis = mathutils.Vector([child_axis_data['x'], child_axis_data['y'], child_axis_data['z']])

                # To keep the joint limits intact, we set the constraint axis as
                # negative if the joint axis is negative
                # if any(child_axis[i] < 0.0 for i in range(0, 3)):
                #     constraint_axis = -constraint_axis

                r_j_c, d_angle = get_rot_mat_from_vecs(constraint_axis, child_axis)

                # Transformation of joint in child frame
                p_j_c = mathutils.Matrix()
                p_j_c.translation = mathutils.Vector(
                    [child_pivot_data['x'], child_pivot_data['y'], child_pivot_data['z']])
                # Now apply the rotation based on the axes deflection from constraint_axis
                t_j_c = r_j_c
                t_j_c.translation = p_j_c.translation
                t_c_j = t_j_c.copy()
                t_c_j.invert()
                child_axis_data['x'] = constraint_axis[0]
                child_axis_data['y'] = constraint_axis[1]
                child_axis_data['z'] = constraint_axis[2]

                if child_obj_handle.type != 'EMPTY':
                    child_obj_handle.data.transform(t_c_j)
                self._body_t_j_c[joint['child']] = t_c_j

                # Implementing the Alignment Offset Correction Algorithm (AO)

                # Parent's Joint Axis in parent's frame
                parent_axis = mathutils.Vector([parent_axis_data['x'], parent_axis_data['y'], parent_axis_data['z']])

                r_caxis_p, r_cnew_p_angle = get_rot_mat_from_vecs(constraint_axis, parent_axis)
                r_cnew_p = r_caxis_p * t_c_j
                r_c_p, r_c_p_angle = get_rot_mat_from_vecs(child_axis, parent_axis)
                r_p_cnew = r_cnew_p.copy()
                r_p_cnew.invert()
                delta_r = r_p_cnew * r_c_p
                print('Joint Name: ', joint_name)
                # print('Delta R: ')
                d_axis_angle = delta_r.to_quaternion().to_axis_angle()
                d_axis = round_vec(d_axis_angle[0])
                d_angle = d_axis_angle[1]
                # Sanity Check: The axis angle should be along the the direction of child axis
                # Throw warning if its not
                v_diff = d_axis.cross(child_axis)
                if v_diff.length > 0.1 and abs(d_angle) > 0.1:
                    print('*** WARNING: AXIS ALIGNMENT LOGIC ERROR')
                print(d_axis, ' : ', d_angle)
                if any(d_axis[i] < 0.0 for i in range(0, 3)):
                    d_angle = - d_angle

                if abs(d_angle) > 0.1:
                    r_ao = mathutils.Matrix().Rotation(d_angle, 4, constraint_axis)
                    child_obj_handle.data.transform(r_ao)
                    self._body_t_j_c[joint['child']] = r_ao * t_c_j
                # end of AO algorithm

            # Finally assign joints and set correct positions

    def load_joint_with_adjusted_bodies(self, joint_name):
        joint = self._afmb[joint_name]
        select_all_objects(False)
        self._context.scene.objects.active = None
        parent_body_name = joint['parent']
        child_body_name = joint['child']
        parent_body_data = self._afmb[parent_body_name]
        child_body_data = self._afmb[child_body_name]
         # Set joint type to blender appropriate name
        joint_type = 'HINGE'
        if 'type' in joint:
            if joint['type'] in ['hinge', 'revolute', 'continuous']:
                joint_type = 'HINGE'
            elif joint['type'] in ['prismatic', 'slider']:
                joint_type = 'SLIDER'
            elif joint['type'] in ['fixed', 'FIXED']:
                joint_type = 'FIXED'
        parent_obj_handle = bpy.data.objects[self._blender_remapped_body_names[parent_body_name]]
        child_obj_handle = bpy.data.objects[self._blender_remapped_body_names[child_body_name]]
        # print('JOINT:', joint_name)
        # print('\tParent: ', parent_obj_handle.name)
        # print('\tChild: ', child_obj_handle.name)
        if 'parent pivot' in joint:
            parent_pivot_data = joint['parent pivot']
            parent_axis_data = joint['parent axis']
            if 'child pivot' in joint:
                child_pivot_data = joint['child pivot']
                child_axis_data = joint['child axis']
                # To fully define a child body's connection and pose in a parent body, just the joint pivots
                # and joint axes are not sufficient. We also need the joint offset which correctly defines
                # the initial pose of the child body in the parent body.
                offset_angle = 0.0
                if not self._context.scene.ignore_afmb_joint_offsets:
                    if 'offset' in joint:
                        offset_angle = joint['offset']
                # Latest release of blender (2.79) only supports joints along child's z axis
                # which requires a workaround for this limitation. We rotate the joint frame by the
                # angular offset of parent's joint axis, w.r.t to parent's z axis.
                # Transformation matrix representing parent in world frame
                t_p_w = parent_obj_handle.matrix_world.copy()

                # Parent's Joint Axis in parent's frame
                parent_axis = mathutils.Vector([parent_axis_data['x'], parent_axis_data['y'], parent_axis_data['z']])

                # Child's Joint Axis in Child's frame
                child_axis = mathutils.Vector([child_axis_data['x'], child_axis_data['y'], child_axis_data['z']])

                # Transformation of joint in parent frame
                p_j_p = mathutils.Matrix()
                # P_j_p = P_j_p * r_j_p
                p_j_p.translation = mathutils.Vector(
                    [parent_pivot_data['x'], parent_pivot_data['y'], parent_pivot_data['z']])
                # Now we need to transform the child since we could have potentially moved
                # the origin of the mesh in last loop's iterations
                # Child's Joint Axis in child's frame

                # Rotation matrix representing child frame in parent frame
                r_c_p, r_c_p_angle = get_rot_mat_from_vecs(child_axis, parent_axis)
                # print ('r_c_p')
                # print(r_c_p)

                # Offset along constraint axis
                t_c_offset_rot = mathutils.Matrix().Rotation(offset_angle, 4, parent_axis)

                t_p_w_off = self._body_t_j_c[joint['parent']]

                # Transformation of child in parents frame
                t_c_p = t_p_w * t_p_w_off * p_j_p * t_c_offset_rot * r_c_p
                # Set the child body the pose calculated above
                child_obj_handle.matrix_world = t_c_p
                child_obj_handle.select = True
                parent_obj_handle.select = True
                self._context.scene.objects.active = parent_obj_handle
                bpy.ops.object.parent_set(keep_transform=True)
                self._context.scene.objects.active = child_obj_handle
                child_obj_handle.select = True
                bpy.ops.rigidbody.constraint_add(type=joint_type)
                child_obj_handle.rigid_body_constraint.object1 \
                    = bpy.data.objects[self._blender_remapped_body_names[parent_body_name]]
                child_obj_handle.rigid_body_constraint.object2 \
                    = bpy.data.objects[self._blender_remapped_body_names[child_body_name]]
                if 'joint limits' in joint:
                    if joint['type'] == 'revolute':
                        child_obj_handle.rigid_body_constraint.limit_ang_z_upper \
                            = joint['joint limits']['high']
                        child_obj_handle.rigid_body_constraint.limit_ang_z_lower \
                            = joint['joint limits']['low']
                        child_obj_handle.rigid_body_constraint.use_limit_ang_z = True
                    elif joint['type'] == 'prismatic':
                        child_obj_handle.rigid_body_constraint.limit_lin_x_upper = \
                            joint['joint limits']['high']
                        child_obj_handle.rigid_body_constraint.limit_lin_x_lower = \
                            joint['joint limits']['low']
                        child_obj_handle.rigid_body_constraint.use_limit_lin_x = True

    def execute(self, context):
        print('HOWDY PARTNER')
        self._yaml_filepath = str(bpy.path.abspath(context.scene['external_afmb_yaml_filepath']))
        print(self._yaml_filepath)
        yaml_file = open(self._yaml_filepath)
        self._afmb = yaml.load(yaml_file)
        self._context = context

        bodies_list = self._afmb['bodies']
        joints_list = self._afmb['joints']

        num_bodies = len(bodies_list)
        # print('Number of Bodies Specified = ', num_bodies)
        self._high_res_path = self.get_qualified_path(self._afmb['high resolution path'])
        print(self._high_res_path)
        for body_name in bodies_list:
            self.load_body(body_name)

        if context.scene.adjust_joint_pivots is True:
            self.adjust_body_pivots_and_axes()

        for joint_name in joints_list:
            if context.scene.adjust_joint_pivots is True:
                self.load_joint_with_adjusted_bodies(joint_name)
            else:
                self.load_joint(joint_name)

        # print('Printing Blender Remapped Body Names')
        # print(self._blender_remapped_body_names)
        return {'FINISHED'}


class CreateAFMBPanel(bpy.types.Panel):
    """Creates a Panel in the Tool Shelf"""
    bl_label = "AF FILE CREATION"
    bl_idname = "OBJECT_PT_afmb_yaml"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = "AF Multi-Body"

    bpy.types.Scene.afmb_yaml_conf_path = bpy.props.StringProperty \
      (
        name="Config (Save To)",
        default="",
        description="Define the root path of the project",
        subtype='FILE_PATH'
      )

    bpy.types.Scene.afmb_yaml_mesh_path = bpy.props.StringProperty \
      (
        name="Meshes (Save To)",
        default="",
        description = "Define the path to save to mesh files",
        subtype='DIR_PATH'
      )

    bpy.types.Scene.mesh_output_type = bpy.props.EnumProperty \
        (
            items=[('STL', 'STL', 'STL'),('OBJ', 'OBJ', 'OBJ'),('3DS', '3DS', '3DS'),('PLY', 'PLY', 'PLY')],
            name="Mesh Type"
        )

    # bpy.context.scene['mesh_output_type'] = 0

    bpy.types.Scene.mesh_max_vertices = bpy.props.IntProperty \
        (
            name="",
            default=150,
            description="The maximum number of vertices the low resolution collision mesh is allowed to have",
        )

    bpy.types.Scene.adjust_joint_pivots = bpy.props.BoolProperty \
        (
            name="Adjust Child Pivots",
            default=True,
            description="If the child axis is offset from the joint axis, correct for this offset, keep this to "
                        "default (True) unless you want to debug the model or something advanced",
        )

    bpy.types.Scene.ignore_afmb_joint_offsets = bpy.props.BoolProperty \
        (
            name="Ignore Offsets",
            default=False,
            description="Ignore the joint offsets from afmb yaml file, keep this to default (False) "
                        "unless you want to debug the model or something advanced",
        )

    bpy.types.Scene.external_afmb_yaml_filepath = bpy.props.StringProperty \
        (
            name="AFMB Config",
            default="",
            description="Load AFMB YAML FILE",
            subtype='FILE_PATH'
        )

    bpy.types.Scene.ignore_inter_collision = bpy.props.BoolProperty \
            (
            name="Ignore Inter-Collision",
            default=True,
            description="Ignore collision between all the bodies in the scene (default = True)",
        )

    setup_yaml()

    def draw(self, context):
        layout = self.layout

        # Panel Label
        layout.label(text="Step 1: GENERATE LOW-RES COLLISION MESH MODIFIERS")

        # Mesh Reduction Ratio Properties
        row = layout.row(align=True)
        row.alignment = 'LEFT'
        split = row.split(percentage=0.7)
        row = split.row()
        row.label('Coll Mesh Max Verts: ')
        row = split.row()
        row.prop(context.scene, 'mesh_max_vertices')

        # Low Res Mesh Modifier Button
        col = layout.column()
        col.alignment = 'CENTER'
        col.operator("myops.generate_low_res_mesh_modifiers")

        # Panel Label
        layout.label(text="Step 2: SELECT LOCATION AND SAVE MESHES")

        # Meshes Save Location
        layout.column().prop(context.scene, 'afmb_yaml_mesh_path')

        # Select the Mesh-Type for saving the meshes
        col = layout.column()
        col.alignment = 'CENTER'
        col.prop(context.scene, 'mesh_output_type')

        # Ignore Inter Collision Button
        col = layout.column()
        col.alignment = 'CENTER'
        col.prop(context.scene, "ignore_inter_collision")

        # Meshes Save Button
        col = layout.column()
        col.alignment = 'CENTER'
        col.operator("myops.add_save_af_meshes")

        layout.label(text="Step 3: GENERATE AF MULTI-BODY CONFIG")

        # Config File Save Location
        col = layout.column()
        col.prop(context.scene, 'afmb_yaml_conf_path')
        # Config File Save Button
        col = layout.column()
        col.alignment = 'CENTER'
        col.operator("myops.add_create_af_yaml")

        layout.label(text="Optional :")

        # Add Optional Button to Remove All Modifiers
        col = layout.column()
        col.alignment = 'CENTER'
        col.operator("myops.remove_modifiers")

        # Add Optional Button to Toggle the Visibility of Low-Res Modifiers
        col = layout.column()
        col.alignment = 'CENTER'
        col.operator("myops.toggle_modifiers_visibility")

        # Load AFMB File Into Blender
        layout.label(text="LOAD AFMB FILE :")

        # Load
        col = layout.column()
        col.alignment = 'CENTER'
        col.prop(context.scene, 'adjust_joint_pivots')

        # Load
        col = layout.column()
        col.alignment = 'CENTER'
        col.prop(context.scene, 'ignore_afmb_joint_offsets')

        # Load
        col = layout.column()
        col.alignment = 'CENTER'
        col.prop(context.scene, 'external_afmb_yaml_filepath')
        
        col = layout.column()
        col.alignment = 'CENTER'
        col.operator("myops.load_afmb_yaml_config")


def register():
    bpy.utils.register_class(ToggleModifiersVisibility)
    bpy.utils.register_class(RemoveModifiers)
    bpy.utils.register_class(GenerateLowResMeshModifiers)
    bpy.utils.register_class(GenerateAFMB)
    bpy.utils.register_class(SaveMeshes)
    bpy.utils.register_class(LoadAFMB)
    bpy.utils.register_class(CreateAFMBPanel)


def unregister():
    bpy.utils.unregister_class(ToggleModifiersVisibility)
    bpy.utils.unregister_class(RemoveModifiers)
    bpy.utils.unregister_class(GenerateLowResMeshModifiers)
    bpy.utils.unregister_class(GenerateAFMB)
    bpy.utils.unregister_class(SaveMeshes)
    bpy.utils.unregister_class(LoadAFMB)
    bpy.utils.unregister_class(CreateAFMBPanel)


if __name__ == "__main__":
    register()
