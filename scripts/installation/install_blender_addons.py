import bpy
import addon_utils
import os

def install_addon(addon_name, addon_path):
    for addon in bpy.context.preferences.addons:
        print(addon.module)

    mod = None

    if addon_name not in addon_utils.addons_fake_modules:
        print("%s: Addon not installed." % addon_name)
        bpy.ops.preferences.addon_install(overwrite=True, target='DEFAULT', filepath=addon_path)
        addon_utils.enable(addon_name, default_set=True, persistent=True)
    else:
        default, state = addon_utils.check(addon_name)
        print(default)
        print(state)
        
        if not state:
            try:
                mod = addon_utils.enable(addon_name, default_set=False, persistent=False)
            except:
                print("%s: Could not enable Addon on the fly." % addon_name )

    if mod:
        print("%s: enabled and running." % addon_name)
    

def main():
    bpy.ops.preferences.addon_refresh()

    install_addon('import_runtime_mhx2', os.path.expanduser('~')+'/Downloads/mhx-blender-latest.zip')
    install_addon('rokoko-studio-live-blender-master', os.path.expanduser('~')+'/Downloads/rokoko-studio-live-blender-master.zip')
    
    bpy.context.scene.pip_module_name = "pycollada"
    bpy.context.scene.pip_module_name = "numpy"
    bpy.context.scene.pip_module_name = "genpy"
    bpy.context.scene.pip_module_name = "genmsg"
    
if __name__ == "__main__":
    main()
    