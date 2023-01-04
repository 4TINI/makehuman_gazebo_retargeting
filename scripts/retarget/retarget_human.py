
import bpy   
import sys
import os
import collada
import traceback
import shutil
import pathlib

argv = sys.argv
argv = argv[argv.index("--") + 1:]  # get all args after "--"

mhx2_file_path = argv[0]
bvh_file_path = argv[1]
output_name = argv[2]

mhx2_file_name = os.path.splitext(os.path.basename(mhx2_file_path))[0]

model_path = '../../models/' + mhx2_file_name
textures_destination = model_path + '/textures/'
collada_filepath = model_path + '/' + output_name+'.dae'

def main():
    bpy.ops.preferences.addon_refresh()
    
    
    file_extension = pathlib.Path(bvh_file_path).suffix
    print(bvh_file_path)
    print(file_extension)
    
    if file_extension == '.bvh':
        bpy.ops.import_anim.bvh(filepath=bvh_file_path, use_fps_scale=True, update_scene_duration=True, axis_forward = '-Z', axis_up = 'Y')
    elif file_extension == '.fbx':
        print("hey")
        bpy.ops.import_scene.fbx(filepath=bvh_file_path)
    
    sel = bpy.context.selected_objects
    animation_name = sel[0].name

    bpy.ops.import_scene.makehuman_mhx2(filepath=mhx2_file_path)
    
    D = bpy.data
    model_name = D.collections[0].name

    bpy.context.scene.rsl_retargeting_armature_source = bpy.data.objects[animation_name]
    bpy.context.scene.rsl_retargeting_armature_target = bpy.data.objects[model_name]

    bpy.ops.rsl.build_bone_list()
    bpy.ops.rsl.retarget_animation()
    bpy.context.view_layer.objects.active = bpy.data.objects[animation_name].select_set(True)
    bpy.ops.object.delete()
    
    for o in bpy.context.scene.objects:
        print(o)
        if o.type == 'MESH':
            for material in o.material_slots:
                bpy.data.materials[material.name].node_tree.nodes["Principled BSDF"].inputs[19].default_value = (0.5, 0.5, 0.5, 1)

    
    bpy.ops.wm.collada_export(filepath=collada_filepath)
    
    try:
        col = collada.Collada(collada_filepath,
                            ignore=[collada.DaeUnsupportedError, collada.DaeBrokenRefError])
        
        # iterate over geometries in the mesh
        for geom in col.geometries:
            tex_name = col.materials[geom.primitives[0].material].effect.params[0].image.path
            col.materials[geom.primitives[0].material].effect.params[0].image.path = 'textures/' + tex_name         
        col.write(collada_filepath)
    
    except BaseException:
        traceback.print_exc()
        print("Failed to load collada file.")
        sys.exit(1)

    files_list = os.listdir(model_path)
    
    for file_name in files_list:
        # Split file name and extension
        name, ext = os.path.splitext(file_name)
        
        if ext == '.png':
            if not os.path.exists(textures_destination + file_name):
    		    # Move all png files
                shutil.move(model_path + '/' + file_name, textures_destination)
            else:
                os.remove(model_path + '/' + file_name)         

if __name__ == "__main__":
    main()  