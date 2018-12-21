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
import yaml
import os
from pathlib import Path
import mathutils
from enum import Enum
from collections import OrderedDict

from collections import OrderedDict


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


# Body Template for the some commonly used of afBody's data
class BodyTemplate:
    def __init__(self):
        self.afmb_data = {'name': "",
                          'mesh': "",
                          'mass': 0.0,
                          'scale': 1.0,
                          'location': {
                             'position': {'x':0, 'y':0, 'z':0},
                             'orientation': {'r':0, 'p':0, 'y':0}},
                          'inertial offset': {
                             'position': {'x': 0, 'y': 0, 'z': 0},
                             'orientation': {'r': 0, 'p': 0, 'y': 0}},
                          'color': 'random'}


# Joint Template for the some commonly used of afJoint's data
class JointTemplate:
    def __init__(self):
        self.afmb_data = {'name': '',
                          'parent': '',
                          'child': '',
                          'parent axis': {'x': 0, 'y': 0.0, 'z': 1.0},
                          'parent pivot': {'x': 0, 'y': 0.0, 'z': 0},
                          'child axis': {'x': 0, 'y': 0.0, 'z': 1.0},
                          'child pivot': {'x': 0, 'y': 0.0, 'z': 0},
                          'joint limits': {'low': -1.2, 'high': 1.2},
                          'enable motor': 0,
                          'max motor impulse': 0}


class CreateAFYAML(bpy.types.Operator):
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
        return center

    def load_body_data(self, afmb_yaml, obj):
        if obj.hide is True:
            return
        body = BodyTemplate()
        body_data = body.afmb_data
        body_yaml_name = self.get_body_prefixed_name(obj.name)
        output_mesh = bpy.context.scene['mesh_output_type']
        extension = get_extension(output_mesh)
        body_data['name'] = obj.name
        body_data['mesh'] = obj.name + extension
        local_pos = obj.matrix_local.translation
        local_rot = obj.matrix_local.to_euler()
        body_pos = body_data['location']['position']
        body_rot = body_data['location']['orientation']
        body_pos['x'] = round(local_pos.x, 3)
        body_pos['y'] = round(local_pos.y, 3)
        body_pos['z'] = round(local_pos.z, 3)
        body_rot['r'] = round(local_rot[0], 3)
        body_rot['p'] = round(local_rot[1], 3)
        body_rot['y'] = round(local_rot[2], 3)
        if obj.rigid_body:
            body_data['mass'] = round(obj.rigid_body.mass, 3)
        body_com = self.compute_local_com(obj)
        body_d_pos = body_data['inertial offset']['position']
        body_d_pos['x'] = round(body_com[0], 3)
        body_d_pos['y'] = round(body_com[1], 3)
        body_d_pos['z'] = round(body_com[2], 3)

        afmb_yaml[body_yaml_name] = body_data
        self._body_names_list.append(body_yaml_name)

    def load_joint_data(self, afmb_yaml, obj):
        if obj.rigid_body_constraint:
            if obj.rigid_body_constraint.object1:
                if obj.rigid_body_constraint.object1.hide is True:
                    return
            if obj.rigid_body_constraint.object2:
                if obj.rigid_body_constraint.object2.hide is True:
                    return
            if obj.rigid_body_constraint.type == 'FIXED':
                fixed_constraint = obj.rigid_body_constraint
                if fixed_constraint.object1 is None or fixed_constraint.object2 is None:
                    if fixed_constraint.object1:
                        afmb_yaml[self.get_body_prefixed_name(fixed_constraint.object1.name)]['mass'] = 0.0
                    elif fixed_constraint.object2 is None:
                        afmb_yaml[self.get_body_prefixed_name(fixed_constraint.object2.name)]['mass'] = 0.0

            if obj.rigid_body_constraint.type == 'HINGE':
                hinge = obj.rigid_body_constraint
                joint = JointTemplate()
                joint_data = joint.afmb_data
                if hinge.object1:
                    parent = hinge.object1
                    child = hinge.object2
                    joint_data['name'] = parent.name + "-" + child.name
                    joint_data['parent'] = self.get_body_prefixed_name(parent.name)
                    joint_data['child'] = self.get_body_prefixed_name(child.name)
                    parent_pivot, parent_axis = self.compute_parent_pivot_and_axis(parent, child)
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
                    child_pivot_data['x'] = 0.0
                    child_pivot_data['y'] = 0.0
                    child_pivot_data['z'] = 0.0
                    child_axis_data['x'] = 0.0
                    child_axis_data['y'] = 0.0
                    child_axis_data['z'] = 1.0
                    joint_limit_data = joint_data["joint limits"]
                    joint_limit_data['low'] = round(hinge.limit_ang_z_lower, 3)
                    joint_limit_data['high'] = round(hinge.limit_ang_z_upper, 3)

                    joint_yaml_name = self.get_joint_prefixed_name(joint_data['name'])
                    afmb_yaml[joint_yaml_name] = joint_data
                    self._joint_names_list.append(joint_yaml_name)

    def compute_parent_pivot_and_axis(self, parent, child):
        # Transform of Parent in World
        t_p_w = parent.matrix_world.copy()
        # Transform of Child in World
        t_c_w = child.matrix_world.copy()

        # Copy over the transform to invert it
        t_w_p = t_p_w.copy()
        t_w_p.invert()
        # Transform of Child in Parent
        # t_c_p = t_w_p * t_c_w
        t_c_p = t_w_p * t_c_w
        parent_pivot = t_c_p.translation
        # The third col of rotation matrix is the z axes of child in parent
        parent_axis = t_c_p.col[2]
        return parent_pivot, parent_axis

    def generate_afmb_yaml(self, context):
        num_objs = len(bpy.data.objects)
        save_to = context.scene.afmb_yaml_conf_path
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
        self._afmb_yaml['high resolution path'] = os.path.join(context.scene.afmb_yaml_mesh_path, 'high_res/')
        self._afmb_yaml['low resolution path'] = os.path.join(context.scene.afmb_yaml_mesh_path, 'low_res/')

        for obj in bpy.data.objects:
            self.load_body_data(self._afmb_yaml, obj)

        for obj in bpy.data.objects:
            self.load_joint_data(self._afmb_yaml, obj)

        # Now populate the bodies and joints tag
        self._afmb_yaml['bodies'] = self._body_names_list
        self._afmb_yaml['joints'] = self._joint_names_list

        yaml.dump(self._afmb_yaml, output_file)


def select_all_objects(select = True):
    # First deselect all objects
    for obj in bpy.data.objects:
        obj.select = select


class SaveAFMeshes(bpy.types.Operator):
    bl_idname = "myops.add_save_af_meshes"
    bl_label = "Save Meshes"

    def execute(self, context):
        self.save_meshes(context)
        return {'FINISHED'}

    def save_meshes(self, context):
        # First deselect all objects
        select_all_objects(False)

        save_path = context.scene.afmb_yaml_mesh_path
        high_res_path = os.path.join(save_path, 'high_res/')
        low_res_path = os.path.join(save_path, 'low_res/')
        os.makedirs(high_res_path, exist_ok= True)
        os.makedirs(low_res_path, exist_ok= True)
        mesh_type = bpy.context.scene['mesh_output_type']
        # Select each object iteratively, save it, and then deselect it
        # Since Blender exports meshes w.r.t world transform and not the
        # the local mesh transform, we explicity push each object to origin,
        # export it as a mesh, and then place it back to its original transform
        for obj in bpy.data.objects:
            obj.select = True
            obj_trans = obj.matrix_world.copy()
            obj.matrix_world.translation = mathutils.Vector([0, 0, 0])
            obj.rotation_euler = mathutils.Euler([0, 0, 0])
            # Mesh Type is .stl
            if mesh_type == MeshType.meshSTL.value:
                obj_name = obj.name + '.STL'
                filename_high_res = os.path.join(high_res_path, obj_name)
                filename_low_res = os.path.join(low_res_path, obj_name)
                bpy.ops.export_mesh.stl(filepath=filename_high_res, use_selection=True, use_mesh_modifiers=False)
                bpy.ops.export_mesh.stl(filepath=filename_low_res, use_selection=True, use_mesh_modifiers=True)
            elif mesh_type == MeshType.meshOBJ.value:
                obj_name = obj.name + '.OBJ'
                filename_high_res = os.path.join(high_res_path, obj_name)
                filename_low_res = os.path.join(low_res_path, obj_name)
                bpy.ops.export_scene.obj(filepath=filename_high_res, use_selection=True, use_mesh_modifiers=False)
                bpy.ops.export_scene.obj(filepath=filename_low_res, use_selection=True, use_mesh_modifiers=True)
            elif mesh_type == MeshType.mesh3DS.value:
                obj_name = obj.name + '.3DS'
                filename_high_res = os.path.join(high_res_path, obj_name)
                filename_low_res = os.path.join(low_res_path, obj_name)
                # 3DS doesn't support supressing modifiers, so we explicitly
                # toggle them to save as high res and low res meshes
                # STILL BUGGY
                for mod in obj.modifiers:
                    mod.show_viewport = True
                bpy.ops.export_scene.autodesk_3ds(filepath=filename_low_res, use_selection=True)
                for mod in obj.modifiers:
                    mod.show_viewport = True
                bpy.ops.export_scene.autodesk_3ds(filepath=filename_high_res, use_selection=True)
            elif mesh_type == MeshType.meshPLY.value:
                # .PLY export has a bug in which it only saves the mesh that is
                # active in context of view. Hence we explicitly select this object
                # as active in the scene on top of being selected
                obj_name = obj.name + '.PLY'
                filename_high_res = os.path.join(high_res_path, obj_name)
                filename_low_res = os.path.join(low_res_path, obj_name)
                bpy.context.scene.objects.active = obj
                bpy.ops.export_mesh.ply(filepath=filename_high_res, use_mesh_modifiers=False)
                bpy.ops.export_mesh.ply(filepath=filename_low_res, use_mesh_modifiers=True)
                # Make sure to deselect the mesh
                bpy.context.scene.objects.active = None

            obj.matrix_world = obj_trans
            obj.select = False


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
            decimate_mod = obj.modifiers.new('decimate_mod', 'DECIMATE')
            if len(obj.data.vertices) > vertices_max:
                reduction_ratio = vertices_max / len(obj.data.vertices)
                decimate_mod.use_symmetry = False
                decimate_mod.use_collapse_triangulate = True
                decimate_mod.ratio = reduction_ratio
                decimate_mod.show_viewport = True
        return {'FINISHED'}


class CreateAFYAMLPanel(bpy.types.Panel):
    """Creates a Panel in the Tool Shelf"""
    bl_label = "AF FILE CREATION"
    bl_idname = "OBJECT_PT_afmb_yaml"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = "AF Multi-Body"

    bpy.types.Scene.afmb_yaml_conf_path = bpy.props.StringProperty \
      (
        name = "Config (Save To)",
        default = "",
        description = "Define the root path of the project",
        subtype = 'FILE_PATH'
      )

    bpy.types.Scene.afmb_yaml_mesh_path = bpy.props.StringProperty \
      (
        name = "Meshes (Save To)",
        default = "",
        description = "Define the path to save to mesh files",
        subtype = 'DIR_PATH'
      )

    bpy.types.Scene.mesh_output_type = bpy.props.EnumProperty \
        (
            items = [('STL', 'STL', 'STL'),('OBJ', 'OBJ', 'OBJ'),('3DS', '3DS', '3DS'),('PLY', 'PLY', 'PLY')],
            name = "Mesh Type"
        )

    # bpy.context.scene['mesh_output_type'] = 0

    bpy.types.Scene.mesh_max_vertices = bpy.props.IntProperty \
        (
            name = "",
            default = 150,
            description = "The maximum number of vertices the low resolution collision mesh is allowed to have",
        )

    bpy.types.Scene.external_afmb_yaml_filepath = bpy.props.StringProperty \
        (
            name="AMBF Config",
            default="",
            description="Load AFMB YAML FILE",
            subtype='FILE_PATH'
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
        col= layout.column()
        col.alignment = 'CENTER'
        col.prop(context.scene, 'mesh_output_type')

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
        col.prop(context.scene, 'external_afmb_yaml_filepath')
        
        col = layout.column()
        col.alignment = 'CENTER'
        col.operator("myops.load_afmb_yaml_config")


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
    
class LoadAFMBYAML(bpy.types.Operator):
    bl_idname = "myops.load_afmb_yaml_config"
    bl_label = "Load AFMB YAML Config"
    
    def __init__(self):
        self._afmb = None
        # A dict for body name as defined in YAML File and the Name Blender gives
        # the body
        self._blender_remapped_body_names = {}
        
    def execute(self, context):
        print('HOWDY PARTNER')
        yaml_file = open(context.scene['external_afmb_yaml_filepath'])
        self._afmb = yaml.load(yaml_file)
        
        bodies_list = self._afmb['bodies']
        joints_list = self._afmb['joints']
        
        num_bodies = len(bodies_list)
        print('Number of Bodies Specified = ', num_bodies)
        high_res_path = self._afmb['high resolution path']
        for body_name in bodies_list:
            body = self._afmb[body_name]
            print(body['name'])
            body_high_res_path = ''
            if 'high resolution path' in body:
                body_high_res_path = body['high resolution path']
            else:
                body_high_res_path = high_res_path
            body_name = body['name']
            body_mesh_name = body['mesh']
            body_mass = body['mass']
            body_location_xyz = body['location']['position']
            body_location_rpy = body['location']['orientation']
            mesh_filepath = Path(os.path.join(body_high_res_path, body_mesh_name))

            if mesh_filepath.suffix in ['.stl', '.STL']:
                bpy.ops.import_mesh.stl(filepath=str(mesh_filepath.resolve()))

            if mesh_filepath.suffix in ['.obj', '.OBJ']:
                bpy.ops.import_scene.obj(filepath=str(mesh_filepath.resolve()))

            obj_handle = context.selected_objects[0]
            self._blender_remapped_body_names[body_name] = obj_handle.name

            bpy.ops.rigidbody.object_add()
            if body_mass == 0.0:
                context.scene.objects.active = obj_handle
                bpy.ops.rigidbody.constraint_add(type='FIXED')
                obj_handle.rigid_body_constraint.object2 = obj_handle
            obj_handle.rigid_body.mass = body_mass
            obj_handle.matrix_world.translation[0] = body_location_xyz['x']
            obj_handle.matrix_world.translation[1] = body_location_xyz['y']
            obj_handle.matrix_world.translation[2] = body_location_xyz['z']
            obj_handle.rotation_euler = (body_location_rpy['r'],
                                         body_location_rpy['p'],
                                         body_location_rpy['y'])

        for joint_name in joints_list:
            joint = self._afmb[joint_name]
            select_all_objects(False)
            parent_body_name = joint['parent']
            child_body_name = joint['child']
            parent_body = self._afmb[parent_body_name]
            child_body = self._afmb[child_body_name]
            if 'type' in joint:
                joint_type = joint['type']
            else:
                joint_type = 'HINGE'

            child_obj_handle = bpy.data.objects[self._blender_remapped_body_names[child_body['name']]]
            context.scene.objects.active = child_obj_handle
            child_obj_handle.select = True
            bpy.ops.rigidbody.constraint_add(type=joint_type)
            child_obj_handle.rigid_body_constraint.object1 = bpy.data.objects[self._blender_remapped_body_names[parent_body['name']]]
            child_obj_handle.rigid_body_constraint.object2 = bpy.data.objects[self._blender_remapped_body_names[child_body['name']]]

        print('Printing Blender Remapped Body Names')
        print(self._blender_remapped_body_names)    
        return {'FINISHED'}


def register():
    bpy.utils.register_class(ToggleModifiersVisibility)
    bpy.utils.register_class(RemoveModifiers)
    bpy.utils.register_class(GenerateLowResMeshModifiers)
    bpy.utils.register_class(CreateAFYAML)
    bpy.utils.register_class(SaveAFMeshes)
    bpy.utils.register_class(LoadAFMBYAML)
    bpy.utils.register_class(CreateAFYAMLPanel)


def unregister():
    bpy.utils.unregister_class(ToggleModifiersVisibility)
    bpy.utils.unregister_class(RemoveModifiers)
    bpy.utils.unregister_class(GenerateLowResMeshModifiers)
    bpy.utils.unregister_class(CreateAFYAML)
    bpy.utils.unregister_class(SaveAFMeshes)
    bpy.utils.unregister_class(LoadAFMBYAML)
    bpy.utils.unregister_class(CreateAFYAMLPanel)


if __name__ == "__main__":
    register()
