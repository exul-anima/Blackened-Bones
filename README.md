# Blackened-Bones
_If you know, you know._

## What is this?
This is a blender addon to import/export Havok physics (HKX files) for characters in Super Smash Bros. Brawl. The game uses an ancient version of the Havok SDK released in 2006 (4.0.0-r1) that's now completely lost. This is a partial reverse-engineering of the format, enough to import the files for editing and previewing in blender, and to export them back into the game.

## Prerequisites
- Blender 3.6 LTS (3.6.7)
- [BrawlBox model/animation addon](https://github.com/Minon/scripts/blob/main/blender/blender_to_brawlbox_maya_exporter.py) by Wayde Brandon Moss
- [Rigid Body Bones addon](https://github.com/Pauan/blender-rigid-body-bones) by Pauan

## Instructions
Read the [wiki](https://github.com/exul-anima/Blackened-Bones/wiki) here.

## File Format Documentation
The HKX file format for Havok 4.0.0-r1 is partially reversed, with notes throughout the source describing important aspects of the spec. It should be good enough that a fully open-source implementation can be made later by someone else with this as a base. I just don't have the time to do that full reversing myself.

## Disclaimers
- This has only been tested on Blender 3.6.7; while other versions might work, I don't know for sure. For now, 3.6.7 will be the only blender version officially supported by this addon.
- As this is only a partial reversing of the format, you will need to get a donor file for the plugin to take certain data from. I cannot provide this data for copyright reasons so you will have to source it yourself from the game (more info in the wiki).
