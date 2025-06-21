import numpy
import struct
import math
import mathutils

import bpy
from bpy_extras.io_utils import ImportHelper
from bpy.types import Operator
from bpy.props import StringProperty
from bpy.utils import register_class
from mathutils import Matrix

## Important offsets
pointerOffsetDataBegin = 0x84               # Pointer to offset of where the data we care about begins.
pointerOffsetTable1 = 0x88                  # \
pointerOffsetTable2 = 0x8C                  #  > Pointer to offsets of three tables of pointers, described in greater detail elsewhere.
pointerOffsetTable3 = 0x90                  # /
pointerOffsetDataEnd = [0x94, 0x98, 0x9C]   # Pointer to offset of where the data we care about ends.


## Utility functions to interpret binary data.

def getByte(bytes, offset):
    return bytes[offset]

def bytesToFloat(bytes, offset):
    floatBytes = bytes[offset:(offset + 0x4)]
    floatValue = struct.unpack('>f', floatBytes)[0]
    return floatValue

def bytesToWord(bytes, offset):
    intBytes = bytes[offset:(offset + 0x4)]
    return int.from_bytes(intBytes, byteorder = 'big')

def bytesToHalfWord(bytes, offset):
    intBytes = bytes[offset:(offset + 0x2)]
    return int.from_bytes(intBytes, byteorder = 'big')

def bytesToString(bytes, offset):
    decodedString = ""
    charPointer = 0x00
    while getByte(bytes, offset + charPointer) != 0x00:
        decodedString += chr(getByte(bytes, offset + charPointer))
        charPointer += 1
    return decodedString


## Utility functions for processing attributes of lines in 3D space.

def lineMidpoint(line):
    midpoint = []
    point1 = line[0]
    point2 = line[1]
    for c in range(len(point1)):
        midpoint.append((point1[c] + point2[c]) / 2)
    return midpoint

def lineAngle(line):
    angle = []
    point1 = line[0]
    point2 = line[1]
    for c in range(len(point1)):
        midpoint.append((point1[c] + point2[c]) / 2)
    return midpoint

def lineLength(line):
    length = 0
    point1 = line[0]
    point2 = line[1]
    for c in range(len(point1)):
       length += (math.pow(point2[c] - point1[c], 2))
    return math.sqrt(length)

def lineMoveToOrigin(line):
    return [line[1][0] - line[0][0], line[1][1] - line[0][1], line[1][2] - line[0][2]]


## Returns number of rigid body colliders described in the file.

def numRigidBodies(hkx):
    offsetDataBegin = bytesToWord(hkx, pointerOffsetDataBegin)                               # Where the data we care about begins.
    offsetTable1 = bytesToWord(hkx, pointerOffsetTable1)                                     # Table 1
    numRigidBodiesPointer = bytesToWord(hkx, offsetDataBegin + offsetTable1 + 0x48)          # \ Returns number of physics rigid bodies.
    return bytesToHalfWord(hkx, offsetDataBegin + numRigidBodiesPointer + 0xA)               # /


## Returns number of constraints described in the file.
## There will always be more rigid bodies than constraints.
## Constraints need rigid bodies to exist!

def numConstraints(hkx):
    offsetDataBegin = bytesToWord(hkx, pointerOffsetDataBegin)                               # Where the data we care about begins.
    offsetTable1 = bytesToWord(hkx, pointerOffsetTable1)                                     # Table 1
    numRigidBodiesPointer = bytesToWord(hkx, offsetDataBegin + offsetTable1 + 0x50)          # \ Returns number of physics constraints.
    return bytesToHalfWord(hkx, offsetDataBegin + numRigidBodiesPointer + 0xA)               # /


## Table 1 is pretty straightforward.
## It's a 2 column table of offsets.
## The first column is the offset from the data beginning to the rigid body or constraint attribute data.
## The second column is the offset from the data beginning to the string within the attribute data corresponding to the physics collider's bone.

def parseTable1(hkx):
    
    rigidBodyJumpTable = []
    constraintJumpTable = []
    rigidBodyStringTable = []
    constraintStringTable = []
    
    offsetDataBegin = bytesToWord(hkx, pointerOffsetDataBegin)                               # Where the data we care about begins.
    offsetTable1 = bytesToWord(hkx, pointerOffsetTable1) + offsetDataBegin                   # Table 1
    entryOffset = 0x4
    
    # Searches for start of jump table for rigid body data.
    while bytesToString(hkx, bytesToWord(hkx, offsetTable1 + entryOffset) + offsetDataBegin) != "Default Physics System":
        entryOffset += 0x8
    startTableRigidBodies = offsetTable1 + entryOffset + 0x4
    
    # Store collider jump table to list.
    entryOffset = 0x0
    for x in range(numRigidBodies(hkx)):
        rigidBodyJumpTable.append(bytesToWord(hkx, startTableRigidBodies + entryOffset) - 0x10 + offsetDataBegin)
        entryOffset += 0x8
    for x in range(numConstraints(hkx)):
        constraintJumpTable.append(bytesToWord(hkx, startTableRigidBodies + entryOffset) - 0x20 + offsetDataBegin)
        entryOffset += 0x8
    
    entryOffset = 0x4
    for x in range(numRigidBodies(hkx)):
        rigidBodyStringTable.append(bytesToWord(hkx, startTableRigidBodies + entryOffset) + offsetDataBegin)
        entryOffset += 0x8
    for x in range(numConstraints(hkx)):
        constraintStringTable.append(bytesToWord(hkx, startTableRigidBodies + entryOffset) + offsetDataBegin)
        entryOffset += 0x8
    
    combinedJumpTable = [[rigidBodyJumpTable, rigidBodyStringTable], [constraintJumpTable, constraintStringTable]]
    
    return combinedJumpTable


def hasConstraint(hkx, entry):
    
    combinedTable = parseTable1(hkx)
    for s in range(len(combinedTable[1][1])):
        if bytesToString(hkx, combinedTable[0][1][entry]) == bytesToString(hkx, combinedTable[1][1][s]):
            return [True, s]
    return [False, -1]


## Part 1 of Table 2 stores pointers to the rigid bodies' and constraints' main data blocks, arranged as a 3 column list in RAM.
## 
## Column 1 is the offset to a pointer to the immediate parent node of the collider data block.
## > In BrawlCrate this shows up as "EntryX", with hkRigidBody or hkConstraintInstance as its immediate child node.
## > These pointers occur starting at 0x200(?) from the data beginning offset (0x790 from the beginning of the file?).
##   > Tentative guess as the placement of this is ambiguous.
## Column 2 is always 1 for the entries we care to edit. Some have a value of 2 but this is unknown why.
## Column 3 is a pointer to the actual main data block.
## > This is stored with all rigid body entries first, followed by all constraint instance entries.

def parseTable2Part1(hkx):
    
    rigidBodyEntries = []
    constraintEntries = []
    
    offsetDataBegin = bytesToWord(hkx, pointerOffsetDataBegin)              # Where the data we care about begins.
    offsetTable2 = bytesToWord(hkx, pointerOffsetTable2) + offsetDataBegin  # \ Table 2, Part 1.
    entryOffset = 0x3C                                                      # /
    
    for x in range(numRigidBodies(hkx)):
        rigidBodyEntries.append(bytesToWord(hkx, offsetTable2 + entryOffset + 0x8) + offsetDataBegin)
        if hasConstraint(hkx, x)[0] == False:
            constraintEntries.append(None)
            entryOffset += 0x1 * 0xC
        else:
            constraintEntries.append(bytesToWord(hkx, offsetTable2 + entryOffset + 0x8 + 0xC) + offsetDataBegin)
            entryOffset += 0x2 * 0xC
    
    return [rigidBodyEntries, constraintEntries]


## Part 2 of Table 2 stores pointers to the rigid bodies' shape data (capsule dimensions) and 
## deactivator data (I think this describes conditions when the physics sim stops), arranged as a 3 column list in RAM.
## 
## Column 1 is the offset to a pointer to the immediate parent node of either the collider shape data or deactivator data.
## > Shape Data:
##   > In BrawlCrate this shows up under the name "collidable" under a hkRigidBody entry.
##   > This pointer occurs 0x1C into the main data block for the respective rigid body, and is patched in at runtime.
## > Deactivator Data:
##   > In BrawlCrate this shows up under the name "deactivator" under a hkRigidBody entry.
##   > This pointer occurs 0x68 into the main data block for the respective rigid body, and is patched in at runtime.
##   > Only rigid body colliders with constraints have this.
## Column 2 is always 1 for the entries we care to edit.
## Column 3 is a pointer to the actual shape data or deactivator data itself.
## 
## The entries are stored grouped per collider sequentially.
## > Example:
## > collider0 shapeData
## > collider1 shapeData
## > collider1 deactivatorData
## > ...

def parseTable2Part2(hkx):
    
    shapeEntries = []
    deactivatorEntries = []
    
    offsetDataBegin = bytesToWord(hkx, pointerOffsetDataBegin)                              # Where the data we care about begins.
    offsetTable2 = bytesToWord(hkx, pointerOffsetTable2) + offsetDataBegin                  # \ Table 2, Part 2.
    entryOffset = 0x3C + (numRigidBodies(hkx) + numConstraints(hkx)) * 0xC                  # /
    
    for x in range(numRigidBodies(hkx)):
        shapeEntries.append(bytesToWord(hkx, offsetTable2 + entryOffset + 0x8) + offsetDataBegin)
        if hasConstraint(hkx, x)[0] == False:
            deactivatorEntries.append(None)
            entryOffset += 0x1 * 0xC
        else:
            deactivatorEntries.append(bytesToWord(hkx, offsetTable2 + entryOffset + 0x8 + 0xC) + offsetDataBegin)
            entryOffset += 0x2 * 0xC
    
    return [shapeEntries, deactivatorEntries]


## Part 3 of Table 2 stores pointers to the constraints' attributes data, the data of the rigid body it belongs to,
## and the parent of said rigid body, all arranged as a 3 column list in RAM.
## 
## Column 1 is the offset to a pointer to the immediate parent node of either the constraint data, rigid body data, or parent data.
## > Constraint Attribute Data:
##   > In BrawlCrate this shows up under the name "data" under a hkConstraintInstance entry.
##   > This pointer occurs 0xC into the main data block for the respective constraint, and is patched in at runtime.
## > Rigid Body Data:
##   > In BrawlCrate this shows up under the name "entities" under a hkRigidBody entry.
##   > This pointer occurs 0x14 into the main data block for the respective constraint, and is patched in at runtime.
## > Parent Data:
##   > In BrawlCrate this shows up under the name "deactivator" under hkRigidBody entry.
##   > This pointer occurs 0x18 into the main data block for the respective constraint, and is patched in at runtime.
## Column 2 is always 1 for the entries we care to edit.
## Column 3 is a pointer to the actual constraint, rigid body, or parent data itself.
## 
## The entries are stored grouped per collider sequentially.
## > Example:
## > constraint0 attributeData
## > constraint0 rigidBodyData
## > constraint0 parentData
## > constraint1 attributeData
## > ...

def parseTable2Part3(hkx):
    
    constraintEntries = []
    rigidBodyEntries = []
    parentEntries = []
    
    offsetDataBegin = bytesToWord(hkx, pointerOffsetDataBegin)                              # Where the data we care about begins.
    offsetTable2 = bytesToWord(hkx, pointerOffsetTable2) + offsetDataBegin                  # \ Table 2, Part 2.
    entryOffset = 0x3C + (numRigidBodies(hkx) + numConstraints(hkx)) * 0xC                  # /
    
    # Calculate offset to beginning of part 3 of table 2.
    for x in range(numRigidBodies(hkx)):
        if hasConstraint(hkx, x)[0] == False:
            entryOffset += 0x1 * 0xC
        else:
            entryOffset += 0x2 * 0xC
    
    for x in range(numConstraints(hkx)):
        constraintEntries.append(bytesToWord(hkx, offsetTable2 + entryOffset + 0x8) + offsetDataBegin)
        rigidBodyEntries.append(bytesToWord(hkx, offsetTable2 + entryOffset + 0x8 + 0xC) + offsetDataBegin)
        parentEntries.append(bytesToWord(hkx, offsetTable2 + entryOffset + 0x8 + (0xC * 2)) + offsetDataBegin)
        entryOffset += 0xC * 3
    
    return [constraintEntries, rigidBodyEntries, parentEntries]


def getCapsuleName(hkx, entry):
    nameTable = parseTable1(hkx)[0][1]
    return bytesToString(hkx, nameTable[entry])


def getCapsuleTransform(hkx, entry):
    
    # Local transforms relative to assigned bone.
    # Initialize here.
    location = []
    capsuleRadius = 0.0
    capsuleLength = 0.0
    
    shapesTable = parseTable2Part2(hkx)[0]  # Offset to rigid body shape data (describing the actual collider shape).
    capsuleRadius = bytesToFloat(hkx, shapesTable[entry] + 0xC)    # Radius of capsule.
    line = [[bytesToFloat(hkx, shapesTable[entry] + 0x10), bytesToFloat(hkx, shapesTable[entry] + 0x14), bytesToFloat(hkx, shapesTable[entry] + 0x18)], [bytesToFloat(hkx, shapesTable[entry] + 0x20), bytesToFloat(hkx, shapesTable[entry] + 0x24), bytesToFloat(hkx, shapesTable[entry] + 0x28)]]     # Line describing length of cylindrical portion of capsule.
    
    capsuleLength = lineLength(line)
    location = lineMidpoint(line)
    output = [capsuleRadius, capsuleLength, location, lineMoveToOrigin(line)]
    
    return output


def getCapsuleAttributes(hkx, entry):
    
    # Physics attributes of current collider.
    # Initialize here.
    mass = 0.0      # How heavy the collider is in kilograms.
    friction = 0.0  # Resistance of the collider to movement.
    
    # How bouncy collisions are (efficiency in collision's energy transfer).
    # Goes from 0 (all energy lost, no bounces) to 1 (no energy lost, perfectly elastic bounces).
    restitution = 0.0
    
    linearDamp = 0.0    # Percentage of linear velocity lost over time.
    angularDamp = 0.0   # Percentage of angular velocity lost over time.
    
    # This determines the baseline behavior (type) for the collider, specifically how its inertia is calculated.
    # Only IDs 7 and 8 are used to my knowledge.
    #
    # 0 => MOTION_INVALID                   = Invalid, used when a collider type is undefined. Never use this.
    # 1 => MOTION_DYNAMIC                   = Automatically chooses between MOTION_SPHERE_INERTIA and MOTION_BOX_INERTIA at runtime.
    # 2 => MOTION_SPHERE_INERTIA            = Inertia is calculated as if the collider is a sphere, irrespective of its actual shape.
    # 3 => MOTION_STABILIZED_SPHERE_INERTIA = Dampened version of MOTION_SPHERE_INERTIA for greater stability.
    # 4 => MOTION_BOX_INERTIA               = Inertia is calculated as if the collider is a rectangular prism, irrespective of its actual shape.
    # 5 => MOTION_STABILIZED_BOX_INERTIA    = Dampened version of MOTION_BOX_INERTIA for greater stability.
    # 6 => MOTION_KEYFRAMED                 = Colliders that can only move with keyframed Havok animations. Brawl doesn't seem to use this.
    # 7 => MOTION_FIXED                     = Optimized version of MOTION_KEYFRAMED used for objects that never move. Used to anchor physics affected colliders to the model's skeleton in Brawl.
    # 8 => MOTION_THIN_BOX_INERTIA          = More stable version of MOTION_BOX_INERTIA for very thin objects. Most colliders in Brawl use this, as they tend to be thinly-dimensioned capsules.
    # 9 => MOTION_MAX_ID                    = Demarks the end of the list of collider types. Never use this.
    
    colliderType = 0
    
    # Atrributes used to deactivate physics under certain circumstances to save compute.
    deactivationEnabled = False         # Enables physics deactivation.
    deactivationLinearVelocity = 0.0    # Linear velocity threshold.
    deactivationAngularVelocity = 0.0   # Angular velocity threshold.
    
    # Get pointers to rigid body main data for iterated collider.
    rigidBodiesTable = parseTable1(hkx)[0][0]     # Offset to rigid body main data.
    
    # Get mass.
    massInverse = bytesToFloat(hkx, rigidBodiesTable[entry] + 0x17C)
    if massInverse != 0:
        mass = 1 / massInverse  # Mass is stored as its inverse (1 / mass) to enable computational shortcuts during runtime.
    else:
        mass = float("inf")     # This is to avoid division by zero.
    
    # Get friction and restitution.
    friction = bytesToFloat(hkx, rigidBodiesTable[entry] + 0x60)
    restitution = bytesToFloat(hkx, rigidBodiesTable[entry] + 0x64)
    
    # Get velocity damping values.
    linearDamp = bytesToFloat(hkx, rigidBodiesTable[entry] + 0x15C)
    angularDamp = bytesToFloat(hkx, rigidBodiesTable[entry] + 0x160)
    
    # Get collider type.
    colliderType = getByte(hkx, rigidBodiesTable[entry] + 0xA8)
    
    # Get deactivation attributes.
    deactivatorsTable = parseTable2Part2(hkx)[1]    # Offset to rigid body deactivator data.
    if deactivatorsTable[entry] is None:
        deactivationEnabled = False         # If a deactivator doesn't exist, don't enable it and don't set its values.
    else:
        deactivationEnabled = True
        deactivationLinearVelocity = bytesToFloat(hkx, deactivatorsTable[entry] + 0x5C)
        deactivationAngularVelocity = bytesToFloat(hkx, deactivatorsTable[entry] + 0x60)
    
    centerOfMassLocalFix = False
    if bytesToFloat(hkx, rigidBodiesTable[entry] + 0x134) >= 0.00009765625:  # Needed due to bug in Brawl's Havok implementation. See exporter for details.
        centerOfMassLocalFix = True
    
    return [mass, friction, restitution, [linearDamp, angularDamp], colliderType, [deactivationEnabled, deactivationLinearVelocity, deactivationAngularVelocity], centerOfMassLocalFix]


def getCapsuleConstraints(hkx, entry, modelArmatureObj):
    
    if hasConstraint(hkx, entry)[0] == False:
        return None
    
    constraintTable = parseTable2Part3(hkx)[0]  # Offset to constraint data.
    
    # Get values for rotation constraints on all axes.
    # Initialize here.
    minRotX = bytesToFloat(hkx, constraintTable[hasConstraint(hkx, entry)[1]] + 0x104)
    maxRotX = bytesToFloat(hkx, constraintTable[hasConstraint(hkx, entry)[1]] + 0x108)
    minRotY = bytesToFloat(hkx, constraintTable[hasConstraint(hkx, entry)[1]] + 0x12C)
    maxRotY = bytesToFloat(hkx, constraintTable[hasConstraint(hkx, entry)[1]] + 0x130)
    minRotZ = bytesToFloat(hkx, constraintTable[hasConstraint(hkx, entry)[1]] + 0x118)
    maxRotZ = bytesToFloat(hkx, constraintTable[hasConstraint(hkx, entry)[1]] + 0x11C)
    
    # Normalize rotations to within -pi to pi, clamping between the limits.
    rotConstraints = [minRotX, maxRotX, minRotY, maxRotY, minRotZ, maxRotZ]
    for c in range(len(rotConstraints)):
        if rotConstraints[c] > math.pi / 2:
            rotConstraints[c] = math.pi / 2
        if rotConstraints[c] < -math.pi / 2:
            rotConstraints[c] = -math.pi / 2
    minRotX = rotConstraints[0]
    maxRotX = rotConstraints[1]
    minRotY = rotConstraints[2]
    maxRotY = rotConstraints[3]
    minRotZ = rotConstraints[4]
    maxRotZ = rotConstraints[5]
    
    rotationLimits = [[minRotX, maxRotX], [minRotY, maxRotY], [minRotZ, maxRotZ]]
    
    # Get angular friction (equivalent to Blender spring constraint with same stiffness and damping force).
    angularFriction = bytesToFloat(hkx, constraintTable[hasConstraint(hkx, entry)[1]] + 0xF8)
    
    return [rotationLimits, angularFriction]


def createCapsule(modelArmatureObj, transforms, attributes, constraints, nameRaw):
    
    modelArmature = bpy.data.objects[modelArmatureObj].data.name
    name = nameRaw.removeprefix("ragdoll_")
    
    # Get basic capsule transform data.
    x = transforms[2][0]    # \
    y = transforms[2][1]    #  > Local coordinates.
    z = transforms[2][2]    # /
    
    x_dir = transforms[3][0]    # \
    y_dir = transforms[3][1]    #  > Direction capsule is oriented.
    z_dir = transforms[3][2]    # /
    dirVectorCapsule = (x_dir, y_dir, z_dir)
    
    # Get bone's physics data attributes to populate and start populating.
    bonePhysicsData = bpy.data.armatures[modelArmature].bones[name].rigid_body_bones
    bonePhysicsData.enabled = True
    bpy.data.armatures[modelArmature].bones.active = bpy.data.armatures[modelArmature].bones[name]
    bonePhysicsData.type = "ACTIVE"
    
    # Remove any compound shape colliders in case the bone somehow has these already.
    # They won't be needed for this, a simple capsule collider works just fine.
    for c in range(len(bonePhysicsData.compounds)):
        bpy.ops.rigid_body_bones.remove_compound()
    
    bonePhysicsData.collision_shape = "CAPSULE"
    
    bonePhysicsData.origin = 0.5    # Set origin to middle of capsule shape.
    
    boneLength = bpy.data.armatures[modelArmature].bones[name].length
    bonePhysicsData.scale_width = transforms[0] / boneLength * 2                        # Diameter of capsule ends.
    bonePhysicsData.scale_length = (transforms[1] / boneLength) + bonePhysicsData.scale_width   # Length of capsule.
    
    bonePhysicsData.location = (x, y, z)
    
    boneLine = []
    boneTail = bpy.data.objects[modelArmatureObj].pose.bones[name].tail
    boneHead = bpy.data.objects[modelArmatureObj].pose.bones[name].head
    boneLine.append(boneHead)
    boneLine.append(boneTail)
    bonePhysicsData.location.y -= lineLength(boneLine) / 2
    
    capsuleAngle = mathutils.Vector(dirVectorCapsule).to_track_quat('Y', 'Z').to_euler()
    bonePhysicsData.rotation.x = capsuleAngle[0]
    bonePhysicsData.rotation.y = capsuleAngle[1]
    bonePhysicsData.rotation.z = capsuleAngle[2]
    
    if attributes[0] == float("inf") and attributes[4] == 0x7:
        bonePhysicsData.type = "PASSIVE"
        for a in range(len(bonePhysicsData.collision_collections)):
            bonePhysicsData.collision_collections[a] = False    # \ Set passive colliders to separate collision layer as they don't collide with actives in Brawl.
        bonePhysicsData.collision_collections[1] = True         # /
    else:
        # Note that due to a quirk in how unit scaling works, some measurement units like length and mass will display as much smaller
        # than they actually are. Temporarily change the unit scale (Scene > Units) to 1 to better see what is actually represented.
        # Remember to return your unit scale back to what it was before!!!
        # Example: at 0.01 unit scale, what's shown as 0.000005 kg is actually just 5 kg.
        #
        # Just remember everything is still in standard metric units under the hood.
        # Length is still in meters, mass is in kilograms, etc. 
        # Blender just shows it weirdly with unit scaling turned on.
        # Toggle between values on the unit scaling to get intuition for what is affected by the option.
        
        bonePhysicsData.mass = attributes[0]            # Mass of collider.
        bonePhysicsData.friction = attributes[1]        # Friction.
        bonePhysicsData.restitution = attributes[2]     # Restitution (collision bounciness).
        bonePhysicsData.linear_damping = attributes[3][0]     # Linear damping.
        bonePhysicsData.angular_damping = attributes[3][1]    # Angular damping.
        
        if attributes[5][0] == True:
            bonePhysicsData.use_deactivation = True         # \ Enable physics deactivation.
            bonePhysicsData.use_start_deactivated = False   # /
            bonePhysicsData.deactivate_linear_velocity = attributes[5][1]   # Linear velocity deactivation threshold.
            bonePhysicsData.deactivate_angular_velocity = attributes[5][2]  # Angular velocity deactivation threshold.
        
        if constraints is not None:
            bonePhysicsData.use_limit_lin_x = True  # \
            bonePhysicsData.use_limit_lin_y = True  #  > Enable translation limit constraints.
            bonePhysicsData.use_limit_lin_z = True  # /
            
            bonePhysicsData.limit_lin_x_lower = 0.0     # \ 
            bonePhysicsData.limit_lin_x_upper = 0.0     # | Set all translation limits to 0 so joints don't dislocate in the ragdoll.
            bonePhysicsData.limit_lin_y_lower = 0.0     # | I have never seen any colliders use translational movements.
            bonePhysicsData.limit_lin_y_upper = 0.0     # | They seem to only use conically-bound rotation constraints.
            bonePhysicsData.limit_lin_z_lower = 0.0     # | This is consistent with future implementations in Sm4sh and Ultimate.
            bonePhysicsData.limit_lin_z_upper = 0.0     # / 
            
            bonePhysicsData.use_limit_ang_x = True  # \
            bonePhysicsData.use_limit_ang_y = True  #  > Enable rotation limit constraints.
            bonePhysicsData.use_limit_ang_z = True  # /
            
            bonePhysicsData.limit_ang_x_lower = constraints[0][0][0]   # X rotation lower bound.
            bonePhysicsData.limit_ang_x_upper = constraints[0][0][1]   # X rotation upper bound.
            bonePhysicsData.limit_ang_y_lower = constraints[0][1][0]   # Y rotation lower bound.
            bonePhysicsData.limit_ang_y_upper = constraints[0][1][1]   # Y rotation upper bound.
            bonePhysicsData.limit_ang_z_lower = constraints[0][2][0]   # Z rotation lower bound.
            bonePhysicsData.limit_ang_z_upper = constraints[0][2][1]   # Z rotation upper bound.
            
            bonePhysicsData.use_override_solver_iterations = attributes[6]  # Patch in a fix for center of mass calculations. See exporter for details.
            
            bonePhysicsData.use_spring_ang_x = True  # \
            bonePhysicsData.use_spring_ang_y = True  #  > Enable rotational spring constraints.
            bonePhysicsData.use_spring_ang_z = True  # /
            
            # Rotational spring constraints are used to represent Havok's angular friction constraint.
            # The two parts of spring constraints in blender are Stiffness and Damping.
            # 
            # Stiffness is how much force is exerted by the spring to move a certain distance.
            # > Directly controls the resistance of the spring to other forces like gravity, hence "stiffness".
            # > Measured in Newtons per meter (N/m).
            # 
            # Damping is how much force is exerted and time is taken by the spring to undo movement a certain distance.
            # > Directly controls the time it takes for the spring to stop moving and return to rest, hence "damping".
            # > Measured in Newton-seconds per meter (Ns/m)
            # 
            # The ratio between these two is the damping ratio, which indicates the behavior of the spring's movement.
            # When stiffness and damping are equal, the damping ratio is 1, which means the spring will return to rest as fast as 
            # possible without overshooting (extra bouncing around) or undershooting (exceedingly slow movement).
            # > A damping ratio of 1 is called "critically damped".
            # > This emulates having a regular joint with additional "stiffness", which is what Havok's constraint is.
            # 
            # MAKE SURE STIFFNESS AND DAMPING ARE THE SAME FOR ACCURATE PREVIEWS IN BLENDER!!! 
            # Havok has no understanding of stiffness and damping as separate values, only of angular friction, 
            # which blender's spring constraint is approximating here.
            # 
            # In this implementation the angular friction value is multiplied by 1000, as Blender uses N/mm instead of N/m.
            # This compensation is independent of blender's unit scaling.
            
            bonePhysicsData.spring_stiffness_ang_x = constraints[1] * 1000.0    # X axis stiffness.
            bonePhysicsData.spring_damping_ang_x = constraints[1] * 1000.0      # X axis damping.
            bonePhysicsData.spring_stiffness_ang_y = constraints[1] * 1000.0    # Y axis stiffness.
            bonePhysicsData.spring_damping_ang_y = constraints[1] * 1000.0      # Y axis damping.
            bonePhysicsData.spring_stiffness_ang_z = constraints[1] * 1000.0    # Z axis stiffness.
            bonePhysicsData.spring_damping_ang_z = constraints[1] * 1000.0      # Z axis damping.
            
            bonePhysicsData.use_spring_x = False    # \
            bonePhysicsData.use_spring_y = False    #  > Disable translational spring constraints in case they're somehow enabled.
            bonePhysicsData.use_spring_z = False    # /
            
            for a in range(len(bonePhysicsData.collision_collections)):
                bonePhysicsData.collision_collections[a] = False    # \ Set passive colliders to separate collision layer as they don't collide with actives in Brawl.
            bonePhysicsData.collision_collections[0] = True         # /
    
    bpy.ops.object.mode_set(mode = "OBJECT")
    
    return None


## Utility functions for error checking and foolproofing the add-on.

def ShowMessageBox(message = "", title = "Message Box", icon = 'INFO'):

    def draw(self, context):
        self.layout.label(text=message)

    bpy.context.window_manager.popup_menu(draw, title = title, icon = icon)

def isArmatureSelected():
    if bpy.context.object == None or bpy.context.object.type != "ARMATURE":
        ShowMessageBox("Make sure the model armature is selected first!", "Havok Importer", "ERROR")
        return None
    else:
        return bpy.context.object.name

def doBonesMatch(hkx, modelArmatureObj):
    for x in range(numRigidBodies(hkx)):
        name = getCapsuleName(hkx, x).removeprefix("ragdoll_")
        currentBone = bpy.data.objects[modelArmatureObj].pose.bones.get(name)
        if currentBone is None:
            ShowMessageBox("The destination armature doesn't have bone \"" + name + "\"!", "Havok Importer", "ERROR")
            return False
    return True


## Main function.

def havokImport(filepath):
    
    modelArmature = isArmatureSelected()
    if isArmatureSelected() == None:
        return None
    
    f = open(filepath, "rb")
    hkx = f.read()

    offsetDataBegin = bytesToWord(hkx, pointerOffsetDataBegin)
    offsetTable1 = bytesToWord(hkx, pointerOffsetTable1)
    offsetTable2 = bytesToWord(hkx, pointerOffsetTable2)
    offsetTable3 = bytesToWord(hkx, pointerOffsetTable3)
    offsetDataEnd = bytesToWord(hkx, pointerOffsetDataEnd[0])
    
    if doBonesMatch(hkx, modelArmature) == False:
        return None
    
    for x in range(numRigidBodies(hkx)):
        createCapsule(modelArmature, getCapsuleTransform(hkx, x), getCapsuleAttributes(hkx, x), getCapsuleConstraints(hkx, x, modelArmature), getCapsuleName(hkx, x))
        
    bpy.ops.object.mode_set(mode = "POSE")
    
    f.close()
    
    return None

 
class Importer(Operator, ImportHelper):
    bl_idname = "blackenedbones.import"
    bl_label = "Import Havok Packfile"
    bl_options = {"PRESET", "UNDO"}
 
    filename_ext = ".hkx"
    
    filter_glob: StringProperty(
        default="*.hkx",
        options={"HIDDEN"}
    )
 
    def execute(self, context):
        if self.filepath.lower()[-4:] != ".hkx":
            ShowMessageBox("Choose a valid Havok packfile (HKX) to import!", "Havok Importer", "ERROR")
            return {"FINISHED"}
        havokImport(self.filepath)
        ShowMessageBox("Imported Havok physics successfully!", "Havok Importer", "INFO")
        print("Imported Havok Packfile: ", self.filepath, '\n')
        return {"FINISHED"}
 
 
#register_class(Importer)
 
#bpy.ops.test.import_tst("INVOKE_DEFAULT")
