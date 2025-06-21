bl_info = {
    "name": "Blackened Bones",
    "author": "Exul Anima",
    "version": (1, 0, 0),
    # Minimum version tested
    "blender": (3, 6, 0),
    "location": "File > Import-Export > Havok 4.0.0-r1 physics (.hkx)",
    "description": "Import/Export Havok 4.0.0-r1 HKX files from Super Smash Bros Brawl.",
    "warning": "",
    "doc_url": "",
    "category": "Import-Export",
    "wiki_url": "https://github.com/exul-anima/Blackened-Bones/wiki",
    "tracker_url": "https://github.com/exul-anima/Blackened-Bones/issues",
}

import bpy

if bpy.app.version >= (4, 0, 0):
    raise Exception("Blackened Bones not compatible wihth 4.0.0 or higher.")

from . import HavokImport
from . import HavokExport

# Only needed if you want to add into a dynamic menu
def menu_func_import(self, context):
    self.layout.operator(HavokImport.Importer.bl_idname, text="Havok 4.0.0-r1 physics (.hkx)")

def menu_func_export(self, context):
    self.layout.operator(HavokExport.Exporter.bl_idname, text="Havok 4.0.0-r1 physics (.hkx)")


# Register and add to the "file selector" menu (required to use F3 search "Text Export Operator" for quick access).
def register():
    bpy.utils.register_class(HavokImport.Importer)
    bpy.utils.register_class(HavokExport.Exporter)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export)


def unregister():
    bpy.utils.unregister_class(HavokImport.Importer)
    bpy.utils.unregister_class(HavokExport.Exporter)
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export)


#if __name__ == "__main__":
#    register()
