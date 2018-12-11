bl_info = {
    "name": "AF Multi-Body Config Creator",
    "author": "Adnan Munawar",
    "version": (0, 1),
    "blender": (2, 75, 0),
    "location": "View3D > Add > Mesh > New Object",
    "description": "Helps Generate AF Multi-Body Config File and Saves both High and Low Resolution(Collision) Meshes",
    "warning": "",
    "wiki_url": "",
    "category": "Add Mesh",
    }

import bpy
import yaml
import os
import mathutils
from enum import Enum

# Enum Class for Mesh Type
class MeshType(Enum):
    meshSTL = 0
    meshOBJ = 1
    mesh3DS = 2
    meshPLY = 3

def getExtension(val):
    if val == MeshType.meshSTL.value:
        extension = '.STL'
    elif val == MeshType.meshOBJ.value:
        extension = '.OBJ'
    elif val == MeshType.mesh3DS.value:
        extension = '.3DS'
    elif val == MeshType.meshPLY.value:
        extension = '.PLY'

    return extension

# Body Template for the some commonly used of afBody's data
class BodyTemplate():
    def __init__(self):
        self.data = {'name': "",
                     'mesh': "",
                     'mass': 0.0,
                     'scale': 1.0,
                     'inertial offset': {'position': {'x': 0, 'y': 0, 'z': 0}},
                     'position': {'x':0, 'y':0, 'z':0},
                     'rotation': {'r':0, 'p':0, 'y':0},
                     'color': 'blue_corn_flower'}

# Joint Template for the some commonly used of afJoint's data
class JointTemplate():
    def __init__(self):
        self.data = {'name': '',
                     'parent': '',
                     'child': '',
                     'parent axis': {'x': 0, 'y': 0.0, 'z': 1.0},
                     'parent pivot': {'x': 0, 'y': 0.0, 'z': 0},
                     'child axis': {'x': 0, 'y': 0.0, 'z': 1.0},
                     'child pivot': {'x': 0, 'y': 0.0, 'z': 0},
                     'joint limits': {'low': -1.2, 'high': 1.2},
                     'enable motor': 0,
                     'max motor impulse' : 0}

class CreateAFYAML(bpy.types.Operator):
    """Tooltip"""
    bl_idname = "myops.add_create_af_yaml"
    bl_label = "Write Multi-Body AF Config"

    def __init__(self):
        self.bodiesList = []
        self.jointsList = []

    def execute(self, context):
        self.saveAFYAML(context)
        return {'FINISHED'}

    # Courtesy of:
    # https://blender.stackexchange.com/questions/62040/get-center-of-geometry-of-an-object
    def computeLocalCOM(self, obj):
        vcos = [ v.co for v in obj.data.vertices ]
        findCenter = lambda l: ( max(l) + min(l) ) / 2
        x,y,z  = [ [ v[i] for v in vcos ] for i in range(3) ]
        center = [ findCenter(axis) for axis in [x,y,z] ]
        return center

    def loadBodyData(self, afYAML, obj):
        body = BodyTemplate()
        bodyData = body.data
        output_mesh = bpy.context.scene['mesh_output_type']
        extension = getExtension(output_mesh)
        bodyData['name'] = obj.name
        bodyData['mesh'] = obj.name + extension
        localPos = obj.matrix_local.translation
        localRot = obj.matrix_local.to_euler()
        bodyPos = bodyData['position']
        bodyRot = bodyData['rotation']
        bodyPos['x'] = round(localPos.x, 3)
        bodyPos['y'] = round(localPos.y, 3)
        bodyPos['z'] = round(localPos.z, 3)
        bodyRot['r'] = round(localRot[0], 3)
        bodyRot['p'] = round(localRot[1], 3)
        bodyRot['y'] = round(localRot[2], 3)
        if obj.rigid_body:
            bodyData['mass'] = round(obj.rigid_body.mass, 3)
        bodyCOM = self.computeLocalCOM(obj)
        bodyDPos = bodyData['inertial offset']['position']
        bodyDPos['x'] = round(bodyCOM[0], 3)
        bodyDPos['y'] = round(bodyCOM[1], 3)
        bodyDPos['z'] = round(bodyCOM[2], 3)
        afYAML[obj.name] = bodyData
        self.bodiesList.append(obj.name)

    def loadJointData(self, afYAML, obj):
        if obj.rigid_body_constraint:
            if obj.rigid_body_constraint.type == 'HINGE':
                hinge = obj.rigid_body_constraint
                joint = JointTemplate()
                jointData = joint.data
                if hinge.object1:
                    parent = hinge.object1
                    child = hinge.object2
                    jointData['name'] = parent.name + "-" + child.name
                    jointData['parent'] = parent.name
                    jointData['child'] = child.name
                    parentPivot, parentAxis = self.computeParentPivotAndAxis(parent, child)
                    parentPivotData = jointData["parent pivot"]
                    parentAxisData = jointData["parent axis"]
                    parentPivotData['x'] = round(parentPivot.x, 3)
                    parentPivotData['y'] = round(parentPivot.y, 3)
                    parentPivotData['z'] = round(parentPivot.z, 3)
                    parentAxisData['x'] = round(parentAxis.x, 3)
                    parentAxisData['y'] = round(parentAxis.y, 3)
                    parentAxisData['z'] = round(parentAxis.z, 3)
                    childPivotData = jointData["child pivot"]
                    childAxisData = jointData["child axis"]
                    childPivotData['x'] = 0.0
                    childPivotData['y'] = 0.0
                    childPivotData['z'] = 0.0
                    childAxisData['x'] = 0.0
                    childAxisData['y'] = 0.0
                    childAxisData['z'] = 1.0
                    jointLimitData = jointData["joint limits"]
                    jointLimitData['low'] = round(hinge.limit_ang_z_lower, 3)
                    jointLimitData['high'] = round(hinge.limit_ang_z_upper, 3)
                    afYAML[jointData['name']] = jointData
                    self.jointsList.append(jointData['name'])

    def computeParentPivotAndAxis(self, parent, child):
        # Transform of Parent in World
        T_p_w = parent.matrix_world.copy()
        # Transform of Child in World
        T_c_w = child.matrix_world.copy()

        # Copy over the transform to invert it
        T_w_p = T_p_w.copy()
        T_w_p.invert()
        # Transform of Child in Parent
        #T_c_p = T_w_p * T_c_w
        T_c_p = T_w_p * T_c_w
        parentPivot = T_c_p.translation
        #The third col of rotation matrix is the z axes of child in parent
        parentAxis = T_c_p.col[2]
        return parentPivot, parentAxis


    def saveAFYAML(self, context):
        numObjs = len(bpy.data.objects)
        save_to = context.scene.afyaml_conf_path
        fileName = os.path.basename(save_to)
        savePath = os.path.dirname(save_to)
        if not fileName:
            fileName = 'default.yaml'
        outputFileName = os.path.join(savePath, fileName)
        outputFile = open(outputFileName, 'w')
        print('Output filename is: ', outputFileName)
        afYAML = {}
        afYAML['high resolution path'] = os.path.join(context.scene.afyaml_mesh_path, 'high_res/')
        afYAML['low resolution path'] = os.path.join(context.scene.afyaml_mesh_path, 'low_res/')
        for obj in bpy.data.objects:
            self.loadBodyData(afYAML, obj)
            self.loadJointData(afYAML, obj)
        afYAML['bodies'] = self.bodiesList
        afYAML['joints'] = self.jointsList
        yaml.dump(afYAML, outputFile)

class SaveAFMeshes(bpy.types.Operator):
    bl_idname = "myops.add_save_af_meshes"
    bl_label = "Save Meshes"

    def execute(self, context):
        self.saveMeshes(context)
        return {'FINISHED'}

    def saveMeshes(self, context):
        # First deselect all objects
        for obj in bpy.data.objects:
            obj.select = False

        save_path = context.scene.afyaml_mesh_path
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
            objTrans = obj.matrix_world.copy()
            obj.matrix_world.translation = mathutils.Vector([0,0,0])
            obj.rotation_euler = mathutils.Euler([0,0,0])
            # Mesh Type is .stl
            if mesh_type == MeshType.meshSTL.value:
                objName = obj.name + '.STL'
                filename_high_res = os.path.join(high_res_path, objName)
                filename_low_res = os.path.join(low_res_path, objName)
                bpy.ops.export_mesh.stl(filepath=filename_high_res, use_selection=True, use_mesh_modifiers=False)
                bpy.ops.export_mesh.stl(filepath=filename_low_res, use_selection=True, use_mesh_modifiers=True)
            elif mesh_type == MeshType.meshOBJ.value:
                objName = obj.name + '.OBJ'
                filename_high_res = os.path.join(high_res_path, objName)
                filename_low_res = os.path.join(low_res_path, objName)
                bpy.ops.export_scene.obj(filepath=filename_high_res, use_selection=True, use_mesh_modifiers=False)
                bpy.ops.export_scene.obj(filepath=filename_low_res, use_selection=True, use_mesh_modifiers=True)
            elif mesh_type == MeshType.mesh3DS.value:
                objName = obj.name + '.3DS'
                filename_high_res = os.path.join(high_res_path, objName)
                filename_low_res = os.path.join(low_res_path, objName)
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
                objName = obj.name + '.PLY'
                filename_high_res = os.path.join(high_res_path, objName)
                filename_low_res = os.path.join(low_res_path, objName)
                bpy.context.scene.objects.active = obj
                bpy.ops.export_mesh.ply(filepath=filename_high_res, use_mesh_modifiers=False)
                bpy.ops.export_mesh.ply(filepath=filename_low_res, use_mesh_modifiers=True)
                # Make sure to deselect the mesh
                bpy.context.scene.objects.active = None

            obj.matrix_world = objTrans
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
            decimateMod = obj.modifiers.new('decimateMod', 'DECIMATE')
            if len(obj.data.vertices) > vertices_max:
                reduction_ratio = vertices_max / len(obj.data.vertices)
                decimateMod.use_symmetry = False
                decimateMod.use_collapse_triangulate = True
                decimateMod.ratio = reduction_ratio
                decimateMod.show_viewport = True
        return {'FINISHED'}


class CreateAFYAMLPanel(bpy.types.Panel):
    """Creates a Panel in the Tool Shelf"""
    bl_label = "AF FILE CREATION"
    bl_idname = "OBJECT_PT_afYAML"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = "AF Multi-Body"

    bpy.types.Scene.afyaml_conf_path = bpy.props.StringProperty \
      (
      name = "Config (Save To)",
      default = "",
      description = "Define the root path of the project",
      subtype = 'FILE_PATH'
      )

    bpy.types.Scene.afyaml_mesh_path = bpy.props.StringProperty \
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

    #bpy.context.scene['mesh_output_type'] = 0

    bpy.types.Scene.mesh_max_vertices = bpy.props.IntProperty \
        (
        name = "",
        default = 150,
        description = "The maximum number of vertices the low resolution collision mesh is allowed to have",
        )


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
        layout.column().prop(context.scene, 'afyaml_mesh_path')

        # Select the Mesh-Type for saving the meshes
        col= layout.column()
        col.alignment = 'CENTER'
        col.prop(context.scene, 'mesh_output_type')

        # Meshes Save Button
        col = layout.column()
        col.alignment = 'CENTER'
        col.operator("myops.add_save_af_meshes")

        layout.label(text="Step 3: GENERATE AF MULTIBODY CONFIG")

        # Config File Save Location
        col = layout.column()
        col.prop(context.scene, 'afyaml_conf_path')
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

def register():
    bpy.utils.register_class(ToggleModifiersVisibility)
    bpy.utils.register_class(RemoveModifiers)
    bpy.utils.register_class(GenerateLowResMeshModifiers)
    bpy.utils.register_class(CreateAFYAML)
    bpy.utils.register_class(SaveAFMeshes)
    bpy.utils.register_class(CreateAFYAMLPanel)

def unregister():
    bpy.utils.unregister_class(ToggleModifiersVisibility)
    bpy.utils.unregister_class(RemoveModifiers)
    bpy.utils.unregister_class(GenerateLowResMeshModifiers)
    bpy.utils.unregister_class(CreateAFYAML)
    bpy.utils.unregister_class(SaveAFMeshes)
    bpy.utils.unregister_class(CreateAFYAMLPanel)

if __name__ == "__main__":
    register()
