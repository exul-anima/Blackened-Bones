import numpy
import struct
import math
import mathutils
import os

import bpy
from bpy_extras.io_utils import ExportHelper
from bpy.types import Operator
from bpy.props import StringProperty
from bpy.utils import register_class
from mathutils import Matrix, Vector, Euler


## Important offsets
pointerOffsetDataBegin = 0x84               # Pointer to offset of where the data we care about begins.
pointerOffsetTable1 = 0x88                  # \
pointerOffsetTable2 = 0x8C                  #  > Pointer to offsets of three tables of pointers, described in greater detail elsewhere.
pointerOffsetTable3 = 0x90                  # /
pointerOffsetDataEnd = [0x94, 0x98, 0x9C]   # Pointer to offset of where the data we care about ends.
pointerOffsetFooter = 0xB4


## Utility functions to interpret binary data.

def getByte(bytes, offset):
    return bytes[offset]

def bytesToFloat(bytes, offset):
    floatBytes = bytes[offset:(offset + 0x4)]
    floatValue = struct.unpack(">f", floatBytes)[0]
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

def appendPadding(bytes, length):
    for x in range(length):
        bytes.append(0x00)
    return None

def floatToBytes(floatValue):
    floatBytes = struct.pack(">f", floatValue)
    return floatBytes

def appendFloat(dataBlock, floatValue):
    for b in floatToBytes(floatValue):
        dataBlock.append(b)
    return None


## Utility functions for processing attributes of lines in 3D space.

def lineMidpoint(line):
    midpoint = []
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


## Utility functions for processing objects in blender.

def getChildren(myObject): 
    children = [] 
    for ob in bpy.data.objects: 
        if ob.parent == myObject:
            children.append(ob) 
    return children

def doArmaturesMatch(modelArmatureObj1, modelArmatureObj2):
    for x in modelArmatureObj1.pose.bones:
        currentBone = modelArmatureObj2.pose.bones[x.name].name
        if currentBone is None:
            return False
    return True


## Return matrix of rigid body and constraint data for the armature.

def getColliderMatrix(modelArmatureObj):
    
    limit = 5e-4    # Limit below which to floor small values that should be zero but aren't due to floating point imprecision.
    
    # Initialize all relevant variables to default values.
    
    name = ""               # Collider name.
    physicsEnabled = False  # Physics enable/disable.
    active = "PASSIVE"       # If collider is affected by external physics forces or animation keyframes only.
    shape = "BOX"           # Shape of collider.
    mass = 0.0              # Mass of collider in kg.
    
    rotationLimitsEnabled = [False, False, False]               # Enable/disable rotation limits.
    rotationLimits = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]       # Rotation limits set per axis.
    translationLimitsEnabled = [False, False, False]            # Enable/disable translation limits.
    translationLimits = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]    # Translation limits per axis.
    
    angularSpringsEnabled = [False, False, False]   # Enable/disable angular (rotational) spring forces.
    angularSpringsStiffness = [0.0, 0.0, 0.0]       # Angular spring stiffness force set per axis in N/mm.
    angularSpringsDamping = [0.0, 0.0, 0.0]         # Angular spring damping force set per axis in N*s/mm.
    linearSpringsEnabled = [False, False, False]    # Enable/disable linear (translational) spring forces.
    
    colliderOriginRelative = 0.5                # Collider origin offset relative to bone (0 = origin at bone head, 1 = origin at bone tail). No effect on center of mass.
    colliderLocation = [0.0, 0.0, 0.0]  # Collider location in m.
    colliderRotation = [0.0, 0.0, 0.0]  # Collider rotation in rad.
    colliderWidthRelative = 0.0         # Collider width relative to bone length.
    colliderLengthRelative = 0.0        # Collider length relative to bone length.
    
    friction = 0.0          # Surface friction of collider.
    restitution = 0.0       # Restitution (bounciness) of surface of collider.
    angularDamping = 0.0    # Percentage of angular velocity lost over time.
    linearDamping = 0.0     # Percentage of linear velocity lost over time.
    
    deactivationEnabled = False         # Enable/disable physics deactivation.
    angularDeactivationVelocity = 0.0   # Minimum angular velocity under which collision forces stopped being calculated.
    linearDeactivationVelocity = 0.0    # Minimum linear velocity under which collision forces stopped being calculated.
    
    # Havok-specific attributes.
    
    colliderRadius = 0.0            # Radius of colldier capsule caps.
    colliderLength = 0.0            # Length between the centers of the radii for the capsule.
    vertex0 = [0.0, 0.0, 0.0]       # Center of collider's first capsule cap.
    vertex1 = [0.0, 0.0, 0.0]       # Center of collider's second capsule cap.
    
    centerOfMass0 = centerOfMass1 = [0.0, 0.0, 0.0] # Center of mass of collider in global coordinates.
    rotation0 = rotation1 = [0.0, 0.0, 0.0, 1.0]    # Rotation of collider in quaternions.
    centerOfMassLocal = [0.0, 0.0, 0.0]             # Center of mass of collider in local coordinates.
    
    angularFriction = 0.0   # In Havok, this is frictional force in N/m applied to increase force required to rotate a collider. Replicated with spring forces in blender, see below.
    
    centerOfMassLocalFix = False    # Flag to work around center of mass bug in physics engine, see below.
    
    # Get physics attributes per collider to be exported and merge them into master list.
    
    modelArmature = bpy.data.objects[modelArmatureObj].data.name
    output = []
    
    # Iterate over properties per collider to put them into a matrix of properties to be exported.
    
    for i in bpy.data.objects[modelArmatureObj].pose.bones:
        name = i.name
        bonePhysicsData = bpy.data.armatures[modelArmature].bones[name].rigid_body_bones
        
        physicsEnabled = bonePhysicsData.enabled
        if physicsEnabled == False:
            continue    # If physics is disabled for a bone, then it will be skipped on export.
        
        active = bonePhysicsData.type
        shape = bonePhysicsData.collision_shape
        mass = bonePhysicsData.mass
        
        if shape != "CAPSULE":
            print("ERROR: The collider", name, "is not a capsule!")
            return None
        
        colliderOriginRelative = bonePhysicsData.origin
        colliderLocation = [bonePhysicsData.location[0], bonePhysicsData.location[1], bonePhysicsData.location[2]]
        colliderRotation = [bonePhysicsData.rotation[0], bonePhysicsData.rotation[1], bonePhysicsData.rotation[2]]
        colliderWidthRelative = bonePhysicsData.scale_width
        colliderLengthRelative = bonePhysicsData.scale_length
        
        if colliderWidthRelative <= limit:
            print("ERROR: Collider", name, "has a width of zero!")
            return None
        if colliderLengthRelative <= limit:
            print("ERROR: Collider", name, "has a length of zero!")
            return None
        
        friction = bonePhysicsData.friction
        restitution = bonePhysicsData.restitution
        angularDamping = bonePhysicsData.angular_damping
        linearDamping = bonePhysicsData.linear_damping
        
        # Translate transforms attributes to Havok equivalents.
        
        boneLength = bpy.data.armatures[modelArmature].bones[name].length
        colliderRadius = boneLength * colliderWidthRelative * 0.5
        colliderLength = (boneLength * colliderLengthRelative) - (colliderRadius * 2)
        
        def capsuleTransformLine(unitVector, colliderLocation, boneLength, colliderOriginRelative, colliderLength, limit, vertex0, vertex1):
            
            # Calculate vertex0 (first vertex of line defining capsule transforms)
            
            u = unitVector.copy()
            u.x *= (colliderLength * colliderOriginRelative)
            u.y *= (colliderLength * colliderOriginRelative)
            u.z *= (colliderLength * colliderOriginRelative)
            c = colliderLocation.copy()
            c[1] += boneLength * colliderOriginRelative
            for axis in range(len(unitVector)):
                if -limit <= c[axis] <= limit:
                    c[axis] = 0.0
                vertex0[axis] = c[axis] + u[axis]
                if -limit <= vertex0[axis] <= limit:
                    vertex0[axis] = 0.0

            # Calculate vertex1 (second vertex of line defining capsule transforms)
            
            v = -u
            c = colliderLocation.copy()
            c[1] += boneLength * colliderOriginRelative
            for axis in range(len(unitVector)):
                if -limit <= c[axis] <= limit:
                    c[axis] = 0.0
                vertex1[axis] = c[axis] + v[axis]
                if -limit <= vertex1[axis] <= limit:
                    vertex1[axis] = 0.0
            
            return [vertex0, vertex1, c]
        
        if active == "ACTIVE":
            unitVector = bpy.data.objects[name + " [Active]"].matrix_local[1].to_3d()
        else:
            unitVector = bpy.data.objects[name + " [Passive]"].matrix_local[1].to_3d()
        
        vertex0 = capsuleTransformLine(unitVector, colliderLocation, boneLength, colliderOriginRelative, colliderLength, limit, vertex0, vertex1)[0]
        vertex1 = capsuleTransformLine(unitVector, colliderLocation, boneLength, colliderOriginRelative, colliderLength, limit, vertex0, vertex1)[1]
        
        # Calculate Havok-specific physics attributes.
        
        # Center of mass calculations.
        # This part was painful.
        
        centerOfMassLocalFix = bonePhysicsData.use_override_solver_iterations
        for child in getChildren(bpy.data.objects[modelArmatureObj]):
            if child is not None and child.type == "ARMATURE" and doArmaturesMatch(bpy.data.objects[modelArmatureObj], child) == True:
                #print("collider coordinates: ", lineMidpoint([vertex0, vertex1]))

                childMatrix = child.data.bones[name].matrix_local           # Matrix of current bone relative to armature origin.
                parentName = child.data.bones[name].parent.name             # \ Matrix of parent bone relative to armature origin.
                parentMatrix = child.data.bones[parentName].matrix_local    # /
                transformTemp = parentMatrix.inverted() @ childMatrix       # Calculate basis of bone relative to parent.
                if active == "ACTIVE":
                    localCenter = Vector(lineMidpoint([vertex0, vertex1])) @ transformTemp  # Local center of mass oriented to basis matrix.
                    transformTemp[0][3] += localCenter[0]           # \
                    transformTemp[1][3] += localCenter[1]           #  > Adjust basis matrix by local location of collider.
                    transformTemp[2][3] += localCenter[2]           # /
                if active == "ACTIVE":
                    centerOfMassLocal = lineMidpoint([vertex0, vertex1])    # Local center of mass.
                    for x in range(len(centerOfMassLocal)):
                        if -limit <= centerOfMassLocal[x] <= limit:
                            centerOfMassLocal[x] = 0.0
                    if centerOfMassLocalFix == True:
                        centerOfMassLocal[1] = 0.00009765625            # Needed due to bug in Brawl's Havok implementation. See below NOTE.
                transformTemp = parentMatrix @ transformTemp                # Get back adjusted local matrix relative to armature origin.
                if active == "ACTIVE":
                    #print(name, " ", transformTemp)
                    #print(transformTemp.to_quaternion())
                    rotation0Temp = [0, 0, 0, 1]
                    rotation0Temp[0] = transformTemp.to_quaternion()[1]     # \
                    rotation0Temp[1] = transformTemp.to_quaternion()[2]     # |
                    rotation0Temp[2] = transformTemp.to_quaternion()[3]     # | Get quaternion representations of rotation for collider's center of mass.
                    rotation0Temp[3] = transformTemp.to_quaternion()[0]     # |
                    rotation0 = rotation0Temp.copy()                        # |
                    rotation1 = rotation0Temp.copy()                        # /
                transformTemp = transformTemp.transposed()              # \ Armature space (for our intents and purposes global) center of mass.
                centerOfMass0 = list(transformTemp[3])[0:3]             # /
                for x in range(len(centerOfMass0)):
                    if -limit <= centerOfMass0[x] <= limit:
                        centerOfMass0[x] = 0.0
                centerOfMass1 = centerOfMass0                               # Assign centerOfMass1 to value of centerOfMass0.
                
                ## NOTE: 
                ## The centerOfMassLocal variable is super weird. For some bones it deviates from what should be the physical 
                ## center of mass by setting an axis to 1 when it should be 0, from my tests this is seemingly always the Y axis,
                ## resulting in physics calculations getting thrown way off.
                ## 
                ## Turns out, even if you set the erroneous axis to a VERY small positive number by the magnitude of 1.0 x 10^-4,
                ## the problem rights itself and the center of mass behaves correctly, bar the tiny offset's imperceptible effect.
                ## This seems to be a bug either in Havok 4.0.0-r1 or Brawl's implementation of it, I can't say for sure.
                ## 
                ## What I can say is that regardless of the behavior's origin, it has been the biggest pain in the ass getting this
                ## god-forsaken physics spec to work. If the bug can be tracked down and fixed in-engine, then the very small nonzero
                ## axis required can be safely floored back down to zero. Until then this duct tape fix stays.

        if active == "ACTIVE":
            
            rotationLimitsEnabled = [bonePhysicsData.use_limit_ang_x, bonePhysicsData.use_limit_ang_y, bonePhysicsData.use_limit_ang_z]
            rotationLimits = [[bonePhysicsData.limit_ang_x_lower, bonePhysicsData.limit_ang_x_upper], [bonePhysicsData.limit_ang_y_lower, bonePhysicsData.limit_ang_y_upper], [bonePhysicsData.limit_ang_z_lower, bonePhysicsData.limit_ang_z_upper]]
            translationLimitsEnabled = [bonePhysicsData.use_limit_lin_x, bonePhysicsData.use_limit_lin_y, bonePhysicsData.use_limit_lin_z]
            translationLimits = [[bonePhysicsData.limit_lin_x_lower, bonePhysicsData.limit_lin_x_upper], [bonePhysicsData.limit_lin_y_lower, bonePhysicsData.limit_lin_y_upper], [bonePhysicsData.limit_lin_z_lower, bonePhysicsData.limit_lin_z_upper]]
            
            if rotationLimitsEnabled != [True, True, True]:
                print("ERROR: rotation limits disabled on bone " + name + ".\nEnable it in the bone's collider settings.")
                return None
            for axis in rotationLimits:
                for bound in axis:
                    #print("bound", abs(bound))
                    #print(math.pi / 2)
                    if abs(bound) > (math.pi / 2) + limit:
                        print("ERROR: invalid rotation limits on bone " + name + ".")
                        print("Make sure the rotation limits are between -90 and 90 degrees, inclusive.")
                        return None
            if translationLimitsEnabled != [True, True, True] or translationLimits != [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]:
                print("ERROR: translation limits enabled on bone " + name + ".\nDisable it in the bone's collider settings.")
                return None
            
            angularSpringsEnabled = [bonePhysicsData.use_spring_ang_x, bonePhysicsData.use_spring_ang_y, bonePhysicsData.use_spring_ang_z]
            angularSpringsStiffness = [bonePhysicsData.spring_stiffness_ang_x, bonePhysicsData.spring_stiffness_ang_y, bonePhysicsData.spring_stiffness_ang_z]
            angularSpringsDamping = [bonePhysicsData.spring_damping_ang_x, bonePhysicsData.spring_damping_ang_y, bonePhysicsData.spring_damping_ang_z]
            linearSpringsEnabled = [bonePhysicsData.use_spring_x, bonePhysicsData.use_spring_y, bonePhysicsData.use_spring_z]
            
            if angularSpringsEnabled != [True, True, True]:
                print("ERROR: angular springs disabled on bone " + name + ".\nEnable it in the bone's collider settings.")
                return None
            for k in angularSpringsStiffness:
                if k != angularSpringsStiffness[0]:
                    print("ERROR: spring stiffness must be equal across all axes on bone " + name + ".")    # If spring stiffness is not the same across all axes, the physics aren't compatible with Havok.
                    return None
            if angularSpringsStiffness != angularSpringsDamping:
                print("ERROR: spring damping must be equal across all axes on bone " + name + ".")  # If spring stiffness and damping values are not identical, the physics aren't compatible with Havok.
                return None
            if linearSpringsEnabled != [False, False, False]:
                print("ERROR: linear springs enabled on bone " + name + ".\nDisable it in the bone's collider settings.")
                return None
            
            angularFriction = angularSpringsStiffness[0] / 1000.0
            
            deactivationEnabled = bonePhysicsData.use_deactivation
            angularDeactivationVelocity = bonePhysicsData.deactivate_angular_velocity
            linearDeactivationVelocity = bonePhysicsData.deactivate_linear_velocity
            
            if deactivationEnabled != True:
                print("ERROR: active collider", name, "needs deactivation data! \nEnable it in the bone's collider settings.")
                return None
                
        currentRow = []
        currentRow.append(name)
        currentRow.append([active, shape, mass])
        currentRow.append(rotationLimits)
        currentRow.append(angularFriction)
        currentRow.append([colliderRadius, vertex0.copy(), vertex1.copy(), colliderLength, colliderRotation])
        currentRow.append([centerOfMass0, centerOfMass1, rotation0, rotation1, centerOfMassLocal])
        currentRow.append([friction, restitution, angularDamping, linearDamping])
        currentRow.append([deactivationEnabled, angularDeactivationVelocity, linearDeactivationVelocity])
        
        output.append(currentRow)
        #print("ROW ", name + " \n", currentRow)

    return output


## Compute inverse inertia tensor of a capsule. It's stored inverted for optimization reasons.

def capsuleInverseInertiaTensor(r, l, m):
    
    v = math.pi * r**2 * ((4 / 3) * r + l)
    c = m / v
    x = (0.5 * l * math.pi * r**4) + ((8 * math.pi * r**5) / 15)
    y = z = (1 / 60) * math.pi * (5 * l**3 * r**2 + 20 * l**2 * r**3 + 45 * l * r**4 + 32 * r**5)
    x *= c
    y *= c
    z *= c
    
    return [1 / x, 1 / y, 1 / z, 1 / m]     # Also include inverse mass in the tensor vector for optimization reasons.


## Build rigid body data blocks for each collider.

def buildRigidBodyData(modelArmatureObj, colliderMatrix):
    
    limit = 5e-4    # Limit below which to floor small values that should be zero but aren't due to floating point imprecision.
    
    rigidBodies = []
    rigidBodyStringOffsets = []
    rigidBodyNames = []
    shapeEntries = []
    deactivatorEntries = []
    activeStatus = []
    
    for c in colliderMatrix:

        name = c[0]
        active = c[1][0]
        mass = c[1][2]
        colliderRadius = c[4][0]
        vertex0 = c[4][1]
        vertex1 = c[4][2]
        colliderLength = c[4][3]
        colliderRotation = c[4][4]
        centerOfMass0 = c[5][0]
        centerOfMass1 = c[5][1]
        rotation0 = c[5][2]
        rotation1 = c[5][3]
        centerOfMassLocal = c[5][4]
        friction = c[6][0]
        restitution = c[6][1]
        angularDamping = c[6][2]
        linearDamping = c[6][3]
        deactivationEnabled = c[7][0]
        angularDeactivationVelocity = c[7][1]
        linearDeactivationVelocity = c[7][2]
        
        dataBlock = bytearray()
        activeStatus.append(active)
        
        # Construct hkRigidBody, the main data structure of the constraint data block.
        
        appendPadding(dataBlock, 0x20)  # These bytes get automatically filled with values at runtime, keep as zeroes.
        dataBlock.extend(b"\xFF\xFF\xFF\xFF")   # Indicate no shape keys for the colliders.
        appendPadding(dataBlock, 0x08)  # These bytes get automatically filled with values at runtime, keep as zeroes.
        dataBlock.extend(b"\xFF\xFF\xFF\xE4")   # Named ownerOffset, has unknown functionality. Always -28.
        appendPadding(dataBlock, 0x04)  # Named id (also called broadPhaseHandle), used to quickly identify colliders, filled in at runtime.
        dataBlock.append(0x01)          # Determines whether broadPhaseHandle id is 16-bit or 32-bit. Always 0x01 for 32-bit. 
        dataBlock.append(0xEC)          # Also named ownerOffset, also unknown functionality. Always -20.
        
        if active == "PASSIVE":
            dataBlock.append(0x00)
            dataBlock.append(0x01)  # Set objectQualityType to HK_COLLIDABLE_QUALITY_KEYFRAMED, for colliders that only move with animation keyframes.
        else:
            dataBlock.append(0x00)
            dataBlock.append(0x03)  # Set objectQualityType to HK_COLLIDABLE_QUALITY_DEBRIS_SIMPLE_TOI, for colliders affected by physics forces.
        appendPadding(dataBlock, 0x04)  # Named collisionFilterInfo, for grouping which colliders can interact with each other. Unused in Brawl.
        if active == "PASSIVE":
            dataBlock.extend(b"\x7F\x7F\xFF\xEE")   # Set the tolerance for how much colliders can clip into each other. Set to very high value on passive colliders.
        else:
            appendFloat(dataBlock, 0.03175)     # Default tolerance for active colliders is 0.03175.
        
        # These bytes get automatically filled with values at runtime, keep as zeroes.
        appendPadding(dataBlock, 0x0C)
        appendPadding(dataBlock, 0x08)
        dataBlock.append(0xC0)
        appendPadding(dataBlock, 0x03)
        appendPadding(dataBlock, 0x04)
        
        dataBlock.append(0x01)      # Response type when colliding with the surfaces of other colliders. Always RESPONSE_SIMPLE_CONTACT.
        appendPadding(dataBlock, 0x03)
        appendFloat(dataBlock, friction)        # Collider surface friction.
        appendFloat(dataBlock, restitution)     # Collider surface restitution.
        
        appendPadding(dataBlock, 0x24)
        dataBlock.append(0xC0)
        appendPadding(dataBlock, 0x03)
        dataBlock.extend(b"\xFF\xFF\xFF\xFF")
        appendPadding(dataBlock, 0x0C)
        
        # Build motion section, which determines various physical properties related to mass.
        
        appendPadding(dataBlock, 0x08)
        if active == "PASSIVE":
            dataBlock.append(0x07)  # Determines how to calculate inertia for collider. Passive colliders are set to MOTION_FIXED.
        else:
            dataBlock.append(0x08)  # Active colliders are set to MOTION_THIN_BOX_INERTIA.
        appendPadding(dataBlock, 0x03)
        appendPadding(dataBlock, 0x04)
        
        # Sets up transform matrix for mass-based properties.
        for child in getChildren(bpy.data.objects[modelArmatureObj]):
            if child is not None and child.type == "ARMATURE" and doArmaturesMatch(bpy.data.objects[modelArmatureObj], child) == True:
                transformLocal = child.data.bones[name].matrix_local
                transformLocal = transformLocal.transposed()
                transform = [[], [], [], []]
                transform[0] = list(transformLocal[0])
                transform[1] = list(transformLocal[1])
                transform[2] = list(transformLocal[2])
                transform[3] = list(transformLocal[3])

        for x in transform:
            for y in range(len(x)):
               if -limit <= x[y] <= limit:
                   x[y] = 0.0
        for x in transform:
            for y in x:
                appendFloat(dataBlock, y)
        
        for x in centerOfMass0:
            appendFloat(dataBlock, x)   # Global center of mass.
        appendFloat(dataBlock, 0.0)     # Append extra 0.0 to make a 4D vector.
        for x in centerOfMass1:
            appendFloat(dataBlock, x)   # Global center of mass (again).
        appendFloat(dataBlock, 0.0)     # Append extra 0.0 to make a 4D vector.
        for x in rotation0:
            appendFloat(dataBlock, x)   # Rotation of collider as a quaternion.
        for x in rotation1:
            appendFloat(dataBlock, x)   # Rotation of collider as a quaternion (again).
        for x in centerOfMassLocal:
            appendFloat(dataBlock, x)   # Local center of mass.
        appendFloat(dataBlock, 0.0)     # Append extra 0.0 to make a 4D vector.
        
        appendPadding(dataBlock, 0x10)  # Named deltaAngle. Filled during runtime? Always seems to be zeroes in static analysis.
        appendFloat(dataBlock, colliderLength + colliderRadius * 2 + 0.1)   # Sphere that completely encapsulates the collider. Used for collision optimizations at runtime.
        appendFloat(dataBlock, 200)     # Maximum linear velocity.
        appendFloat(dataBlock, 10)      # Maximum angular velocity.
        appendFloat(dataBlock, linearDamping)   # Linear damping.
        appendFloat(dataBlock, angularDamping)  # Angular damping.
        if active == "PASSIVE":
            dataBlock.append(0x00)
            dataBlock.append(0x01)  # Named deactivationClass. Always 0x1 on passive colliders.
            dataBlock.append(0xBD)
            dataBlock.append(0x2F)  # Named deactivationCounter. Always 0xBD2F on passive colliders.
        else:
            dataBlock.append(0x00)
            dataBlock.append(0x02)  # Always 0x2 on active colliders.
            dataBlock.append(0x00)
            dataBlock.append(0x14)  # Always 0x14 on active colliders.
        appendPadding(dataBlock, 0x08)
        
        tensor = [0.0, 0.0, 0.0, 0.0]   # If passive collider, inertia tensor is all zeroes.
        if active == "ACTIVE":
            tensor = capsuleInverseInertiaTensor(colliderRadius, colliderLength, mass)    # Calculate capsule inertia tensor.
        for k in tensor:
            appendFloat(dataBlock, k)
        
        appendPadding(dataBlock, 0x64)
        dataBlock.extend(b"\xFF\xFF\xFF\xFF")
        appendPadding(dataBlock, 0x08)
        
        rigidBodyStringOffsets.append(len(dataBlock))
        for x in ("ragdoll_" + name).encode("utf-8"):
            dataBlock.append(x)     # Name of collider.
        for x in range(0x10 - (len("ragdoll_" + name) % 0x10)):
            dataBlock.append(0x00)  # 16-byte-aligned padding for string.
        rigidBodyNames.append("ragdoll_" + name)
        
        # Construct collider shape definition.
        
        shapeEntries.append(len(dataBlock))
        appendPadding(dataBlock, 0x0C)
        appendFloat(dataBlock, colliderRadius)
        for x in vertex0:
            appendFloat(dataBlock, x)
        appendFloat(dataBlock, colliderRadius)
        for x in vertex1:
            appendFloat(dataBlock, x)
        appendFloat(dataBlock, colliderRadius)
        
        # Construct deactivator definition.
        
        if active == "ACTIVE":
            deactivatorEntries.append(len(dataBlock))
            appendPadding(dataBlock, 0x10)
            dataBlock.extend(b"\x7F\x7F\xFF\xEE\x7F\x7F\xFF\xEE\x7F\x7F\xFF\xEE\x7F\x7F\xFF\xEE")
            dataBlock.extend(b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x3F\x80\x00\x00")
            dataBlock.extend(b"\x7F\x7F\xFF\xEE\x7F\x7F\xFF\xEE\x7F\x7F\xFF\xEE\x7F\x7F\xFF\xEE")
            dataBlock.extend(b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x3F\x80\x00\x00")
            appendFloat(dataBlock, -1.0)
            appendFloat(dataBlock, 0.01)
            appendFloat(dataBlock, 0.005)
            appendFloat(dataBlock, linearDeactivationVelocity)
            appendFloat(dataBlock, angularDeactivationVelocity)
            appendPadding(dataBlock, 0x0C)
        else:
            deactivatorEntries.append(0x00)
        
        rigidBodies.append(dataBlock)
    
    return [rigidBodies, rigidBodyStringOffsets, rigidBodyNames, shapeEntries, deactivatorEntries, activeStatus]


## Build constraint data blocks for each collider that needs one.

def buildConstraintData(modelArmatureObj, colliderMatrix):
    
    limit = 5e-4    # Limit below which to floor small values that should be zero but aren't due to floating point imprecision.
    
    constraints = []
    constraintStringOffsets = []
    constraintNames = []
    dataEntries = []
    
    for c in colliderMatrix:
        
        active = c[1][0]
        if active == "PASSIVE":
            continue    # If the collider doesn't have a constraint, skip it.
        
        name = c[0]
        rotationLimits = c[2]
        angularFriction = c[3]
        
        dataBlock = bytearray()
        
        # Construct hkConstraintInstance, the main data structure of the constraint data block.
        
        appendPadding(dataBlock, 0x1C)  # These bytes get automatically filled with values at runtime, keep as zeroes.
        
        dataBlock.append(0x01)      # Internal priority (PRIORITY_PSI). Unsure what it does but ragdoll constraints require it.
        appendPadding(dataBlock, 0x03)
        
        appendPadding(dataBlock, 0x10)
        
        constraintStringOffsets.append(len(dataBlock))
        for x in ("ragdoll_" + name).encode("utf-8"):
            dataBlock.append(x)     # Name of collider.
        for x in range(0x10 - (len("ragdoll_" + name) % 0x10)):
            dataBlock.append(0x00)  # 16-byte-aligned padding for string.
        constraintNames.append("ragdoll_" + name)
                
        # Build hkConstraintData and accompanying constraint atoms within.
        
        dataEntries.append(len(dataBlock))
        appendPadding(dataBlock, 0x10)  # These bytes get automatically filled with values at runtime, keep as zeroes.
        
        # Build transforms atom, which defines matrices for constraint pivots.
        
        dataBlock.append(0x00)
        dataBlock.append(0x02)
        appendPadding(dataBlock, 0x0E)
        
        transformA = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]   # Matrix of bone's X rotation relative to previous bone. Initialize to identity.
        transformB = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]   # Basis matrix of the bone minus the X rotation. Initialize as identity.
        transformAtemp = Matrix.Identity(4)
        transformBtemp = Matrix.Identity(4)
        
        # Calculate basis matrix of current bone.
        
        for child in getChildren(bpy.data.objects[modelArmatureObj]):
            if child is not None and child.type == "ARMATURE" and doArmaturesMatch(bpy.data.objects[modelArmatureObj], child) == True:
                childMatrix = child.data.bones[name].matrix_local           # Matrix of current bone relative to armature origin.
                parentName = child.data.bones[name].parent.name
                parentMatrix = child.data.bones[parentName].matrix_local    # Matrix of parent bone relative to armature origin.
                transformBtemp = parentMatrix.inverted() @ childMatrix      # Calculate basis of bone relative to parent.
                
                xAngle = Euler((transformBtemp.to_euler().x, 0.0, 0.0))     # Extract X component of transformB matrix.
                transformAtemp = xAngle.to_matrix().to_4x4()                # Matrix of X rotation, to be transformA.
                transformBtemp @= transformAtemp.transposed()               # Remove X rotation from transformB matrix.
                transformBtemp = transformBtemp.transposed()
                                
                transformA[0] = list(transformAtemp[0])
                transformA[1] = list(transformAtemp[1])
                transformA[2] = list(transformAtemp[2])
                transformA[3] = list(transformAtemp[3])
                
                transformB[0] = list(transformBtemp[0])
                transformB[1] = list(transformBtemp[1])
                transformB[2] = list(transformBtemp[2])
                transformB[3] = list(transformBtemp[3])
        
        for x in transformA:
            for y in range(len(x)):
               if -limit <= x[y] <= limit:
                   x[y] = 0.0
        
        for x in transformB:
            for y in range(len(x)):
               if -limit <= x[y] <= limit:
                   x[y] = 0.0
        
        for x in transformA:
            for y in x:
               appendFloat(dataBlock, y)
        for x in transformB:
            for y in x:
                appendFloat(dataBlock, y)
        
        # Build ragdollMotors atom, which goes unused in Brawl. Not gonna bother with making it work right now, leaving it disabled.
        
        dataBlock.append(0x00)
        dataBlock.append(0x13)      # Set constraint type to TYPE_RAGDOLL_MOTOR.
        appendPadding(dataBlock, 0x02)  # Set to zeroes to disable.
        dataBlock.append(0x00)
        dataBlock.append(0x60)      # Default value for initializedOffset.
        dataBlock.append(0x00)
        dataBlock.append(0x64)      # Default value for previousTargetAnglesOffset.
        appendPadding(dataBlock, 0x08)
        
        for x in range(len(transformB)):
            if x >= 3:
                continue
            for y in range(len(transformB[x])):
                appendFloat(dataBlock, transformB[x][y])    # First 3 rows of transformB.
        
        appendPadding(dataBlock, 0x10)
        
        # Build angFriction atom, which defines friction on joints' rotational axes.
        
        dataBlock.append(0x00)
        dataBlock.append(0x11)      # Set constraint type to TYPE_ANG_FRICTION.
        dataBlock.append(0x01)      # Enable constraint.
        dataBlock.append(0x00)      # Specify first friction axis (always X).
        dataBlock.append(0x03)      # Specify number of affected axes (always all 3).
        appendPadding(dataBlock, 0x03)  # Padding.
        appendFloat(dataBlock, angularFriction)    # Set angular friction value.
        
        # Build twistLimit atom, controls X rotation of collider.
        
        dataBlock.append(0x00)
        dataBlock.append(0x0F)      # Set constraint type to TYPE_TWIST_LIMIT.
        dataBlock.append(0x01)      # Enable constraint.
        dataBlock.append(0x00)      # Specify twist axis (always X in Brawl).
        dataBlock.append(0x01)      # Specify orthogonal reference axis (always Y in Brawl).
        appendPadding(dataBlock, 0x03)  # Padding.
        appendFloat(dataBlock, rotationLimits[0][0])    # Set lower rotation bound.
        appendFloat(dataBlock, rotationLimits[0][1])    # Set upper rotation bound.
        appendFloat(dataBlock, 0.8)     # Set angularLimitsTauFactor, which determines how much colliders ease into hitting rotation limits. Always 0.8 in Brawl.
        
        # Build coneLimit atom, controls Z rotation of collider.
        
        dataBlock.append(0x00)
        dataBlock.append(0x10)      # Set constraint type to TYPE_CONE_LIMIT.
        dataBlock.append(0x01)      # Enable constraint.
        dataBlock.append(0x00)      # Specify twistAxisInA (always X in Brawl).
        dataBlock.append(0x00)      # Specify orthogonal refAxisInB (always X in Brawl).
        dataBlock.append(0x00)      # Specify angleMeasurementMode, which determines what defines an angle of zero. Set to ZERO_WHEN_VECTORS_ALIGNED.
        appendPadding(dataBlock, 0x02)  # Padding.
        appendFloat(dataBlock, rotationLimits[2][0])    # Set lower rotation bound.
        appendFloat(dataBlock, rotationLimits[2][1])    # Set upper rotation bound.
        appendFloat(dataBlock, 0.8)     # Set angularLimitsTauFactor, which determines how much colliders ease into hitting rotation limits. Always 0.8 in Brawl.
        
        # Build planesLimit atom, controls Y rotation of collider.
        
        dataBlock.append(0x00)
        dataBlock.append(0x10)      # Set constraint type to TYPE_CONE_LIMIT.
        dataBlock.append(0x01)      # Enable constraint.
        dataBlock.append(0x00)      # Specify twistAxisInA (always X in Brawl).
        dataBlock.append(0x01)      # Specify orthogonal refAxisInB (always Y in Brawl).
        dataBlock.append(0x01)      # Specify angleMeasurementMode, which determines what defines an angle of zero. Set to ZERO_WHEN_VECTORS_PERPENDICULAR.
        appendPadding(dataBlock, 0x02)  # Padding.
        appendFloat(dataBlock, rotationLimits[1][0])    # Set lower rotation bound.
        appendFloat(dataBlock, rotationLimits[1][1])    # Set upper rotation bound.
        appendFloat(dataBlock, 0.8)     # Set angularLimitsTauFactor, which determines how much colliders ease into hitting rotation limits. Always 0.8 in Brawl.
        
        # Build ballSocket atom, which defines the constraint with all translations locked but all rotations free.
        
        dataBlock.append(0x00)
        dataBlock.append(0x05)      # Set constraint type to TYPE_ANG_FRICTION.
        appendPadding(dataBlock, 0x06)  # Padding.
        
        constraints.append(dataBlock)
    
    return [constraints, constraintStringOffsets, constraintNames, dataEntries]


## Sort the colliders by hierarchical highest passive parent.
## Might not be strictly necessary for the file spec but I already made it.
## Might as well use it. Besides, it helps organize things.

def sortColliders(rigidBodyArray, constraintArray, modelArmatureObj):
    
    rigidBodies = rigidBodyArray[0]
    rigidBodyStringOffsets = rigidBodyArray[1]
    rigidBodyNames = rigidBodyArray[2]
    shapeEntries = rigidBodyArray[3]
    deactivatorEntries = rigidBodyArray[4]
    activeStatus = rigidBodyArray[5]
    
    #print(activeStatus)
    passiveIDs = [i for i, x in enumerate(activeStatus) if x == "PASSIVE"]
    
    passiveBodies = []
    passiveBodyStringOffsets = []
    passiveBodyNames = []
    passiveShapeEntries = []
    passiveDeactivatorEntries = []

    activeBodies = rigidBodies.copy()
    activeBodyStringOffsets = rigidBodyStringOffsets.copy()
    activeBodyNames = rigidBodyNames.copy()
    activeShapeEntries = shapeEntries.copy()
    activeDeactivatorEntries = deactivatorEntries.copy()
    
    colliderBodies = []
    colliderBodyStringOffsets = []
    colliderBodyNames = []
    colliderShapeEntries = []
    colliderDeactivatorEntries = []
    
    constraints = constraintArray[0]
    constraintStringOffsets = constraintArray[1]
    constraintNames = constraintArray[2]
    dataEntries = constraintArray[3]
    
    sortedConstraints = []
    sortedConstraintStringOffsets = []
    sortedConstraintNames = []
    sortedDataEntries = []
    
    for x in passiveIDs:
        passiveBodies.append(rigidBodies[x])
        passiveBodyStringOffsets.append(rigidBodyStringOffsets[x])
        passiveBodyNames.append(rigidBodyNames[x])
        passiveShapeEntries.append(shapeEntries[x])
        passiveDeactivatorEntries.append(deactivatorEntries[x])
        
        activeBodies.remove(rigidBodies[x])
        activeBodyStringOffsets.remove(rigidBodyStringOffsets[x])
        activeBodyNames.remove(rigidBodyNames[x])
        activeShapeEntries.remove(shapeEntries[x])
        activeDeactivatorEntries.remove(deactivatorEntries[x])
    
    modelArmature = bpy.data.objects[modelArmatureObj].data
    
    for bone in modelArmature.bones:
        bonePhysicsData = bone.rigid_body_bones
        if bonePhysicsData.enabled == True:
            for x in range(len(passiveBodyNames)):
                if passiveBodyNames[x] == "ragdoll_" + bone.name:
                    print("Passive collider found!\t\t", passiveBodyNames[x])
                    colliderBodies.append(passiveBodies[x])
                    colliderBodyStringOffsets.append(passiveBodyStringOffsets[x])
                    colliderBodyNames.append(passiveBodyNames[x])
                    colliderShapeEntries.append(passiveShapeEntries[x])
                    colliderDeactivatorEntries.append(passiveDeactivatorEntries[x])
                    break
            for y in range(len(activeBodyNames)):
                if activeBodyNames[y] == "ragdoll_" + bone.name:
                    print("    Active collider found!\t", activeBodyNames[y])
                    colliderBodies.append(activeBodies[y])
                    colliderBodyStringOffsets.append(activeBodyStringOffsets[y])
                    colliderBodyNames.append(activeBodyNames[y])
                    colliderShapeEntries.append(activeShapeEntries[y])
                    colliderDeactivatorEntries.append(activeDeactivatorEntries[y])

                    sortedConstraints.append(constraints[constraintNames.index(activeBodyNames[y])])
                    sortedConstraintStringOffsets.append(constraintStringOffsets[constraintNames.index(activeBodyNames[y])])
                    sortedConstraintNames.append(constraintNames[constraintNames.index(activeBodyNames[y])])
                    sortedDataEntries.append(dataEntries[constraintNames.index(activeBodyNames[y])])
                    break
    
    sortedRigidBodyArray = [colliderBodies, colliderBodyStringOffsets, colliderBodyNames, colliderShapeEntries, colliderDeactivatorEntries]
    sortedConstraintArray = [sortedConstraints, sortedConstraintStringOffsets, sortedConstraintNames, sortedDataEntries]
    
    return [sortedRigidBodyArray, sortedConstraintArray]


## Build the pointer tables for the file. In-depth descriptions of the structures being constructed here
## can be found in the importer script for saving on redundancy.

def createTable1(rigidBodyArray, constraintArray, dataBlocksAttributes, offsetDataBegin, headerPos1, headerPos2, dataTypeDelimit):
    
    rigidBodyStringOffsets = rigidBodyArray[1]
    constraintStringOffsets = constraintArray[1]
    
    # Unfortunately hardcoded data from Toon Link's HKX data. Needs replacement for fully open-source version.
    table = bytearray([
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x30, 
    0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x50, 
    0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0xF0, 
    0x00, 0x00, 0x00, 0x74, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x90, 0x00, 0x00, 0x01, 0x40, 
    0x00, 0x00, 0x01, 0x8C, 0x00, 0x00, 0x01, 0xA0, 0x00, 0x00, 0x01, 0xB8, 0x00, 0x00, 0x02, 0x00, 
    0x00, 0x00, 0x01, 0xC4, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x01, 0xE8, 0x00, 0x00, 0x02, 0x80, 
    ])
    
    for x in range(0x40, len(table), 0x04):
        if bytesToWord(table, x) == 0x240:
            table[x:x + 4] = struct.pack(">I", dataTypeDelimit - 0x590)
        elif bytesToWord(table, x) < 0x240:
            table[x:x + 4] = struct.pack(">I", bytesToWord(table, x) + headerPos1 - 0x720)
        else:
            table[x:x + 4] = struct.pack(">I", bytesToWord(table, x) + headerPos2 - 0x810)
    
    # Insert rigid body pointers.
    numRigidBodies = 0
    for x in range(len(rigidBodyStringOffsets)):
        table.extend(struct.pack(">I", dataBlocksAttributes[0][x] - offsetDataBegin + 0x10))    # Pointer to data block.
        table.extend(struct.pack(">I", dataBlocksAttributes[0][x] - offsetDataBegin + rigidBodyStringOffsets[x]))   # Pointer to name string.
        numRigidBodies += 1
    # Insert constraint pointers.
    for x in range(len(constraintStringOffsets)):
        table.extend(struct.pack(">I", dataBlocksAttributes[0][x + numRigidBodies] - offsetDataBegin + 0x20))    # Pointer to data block.
        table.extend(struct.pack(">I", dataBlocksAttributes[0][x + numRigidBodies] - offsetDataBegin + constraintStringOffsets[x]))   # Pointer to name string.
    
    for x in range(0x10 - (len(table) % 0x10)):
        table.append(0xFF)     # 16-byte-aligned padding for string.
    
    return table
    
def createTable2Part1(rigidBodyArray, constraintArray, dataBlocksAttributes, offsetDataBegin, headerPos1):
    
    rigidBodies = rigidBodyArray[0]
    constraints = constraintArray[0]
    
    # More hardcoding to remove later. Assume any of these byte arrays are hardcoded.
    table = bytearray([
    0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x70, 
    0x00, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x80, 
    0x00, 0x00, 0x00, 0x2C, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x3E, 0x90, 
    0x00, 0x00, 0x01, 0xA0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0xB0
    ])
    
    table[0x20:0x24] = struct.pack(">I", bytesToWord(table, 0x20) + headerPos1 - 0x720)
    table[0x30:0x34] = struct.pack(">I", bytesToWord(table, 0x30) + headerPos1 - 0x720)
    table[0x38:0x3C] = struct.pack(">I", bytesToWord(table, 0x38) + headerPos1 - 0x720)
    
    # Insert rigid body pointers.
    o = headerPos1 + 0x70 - offsetDataBegin
    for x in range(len(rigidBodies)):
        table.extend(struct.pack(">I", o + (x * 0x04)))
        table.extend(struct.pack(">I", 0x01))
        table.extend(struct.pack(">I", dataBlocksAttributes[0][x] - offsetDataBegin))
    
    # Insert constraint pointers.
    p = 0x10 - (o + (len(rigidBodies) * 0x04)) % 0x10
    for x in range(len(rigidBodies), len(rigidBodies) + len(constraints)):
        table.extend(struct.pack(">I", o + (x * 0x04) + p))
        table.extend(struct.pack(">I", 0x01))
        table.extend(struct.pack(">I", dataBlocksAttributes[0][x] - offsetDataBegin))
    
    return table

def createTable2Part2(rigidBodyArray, constraintArray, dataBlocksAttributes, offsetDataBegin, headerPos1, headerPos2):
    
    rigidBodies = rigidBodyArray[0]
    rigidBodyNames = rigidBodyArray[2]
    shapeEntries = rigidBodyArray[3]
    deactivatorEntries = rigidBodyArray[4]
    
    table = bytearray()
    
    # Insert rigid body pointers.
    for x in range(len(rigidBodies)):
        table.extend(struct.pack(">I", dataBlocksAttributes[0][x] - offsetDataBegin + 0x1C))
        table.extend(struct.pack(">I", 0x01))
        table.extend(struct.pack(">I", dataBlocksAttributes[0][x] - offsetDataBegin + shapeEntries[x]))
        if deactivatorEntries[x] != 0x00:
            table.extend(struct.pack(">I", dataBlocksAttributes[0][x] - offsetDataBegin + 0x68))
            table.extend(struct.pack(">I", 0x01))
            table.extend(struct.pack(">I", dataBlocksAttributes[0][x] - offsetDataBegin + deactivatorEntries[x]))
    
    return table

def createTable2Part3(rigidBodyArray, constraintArray, dataBlocksAttributes, offsetDataBegin, headerPos1, headerPos2, modelArmatureObj):
    
    rigidBodies = rigidBodyArray[0]
    rigidBodyNames = rigidBodyArray[2]
    
    constraints = constraintArray[0]
    constraintNames = constraintArray[2]
    dataEntries = constraintArray[3]
    
    table = bytearray()
    
    # Insert constraint pointers.
    for x in range(len(constraints)):
        table.extend(struct.pack(">I", dataBlocksAttributes[0][x + len(rigidBodies)] - offsetDataBegin + 0x0C))
        table.extend(struct.pack(">I", 0x01))
        table.extend(struct.pack(">I", dataBlocksAttributes[0][x + len(rigidBodies)] - offsetDataBegin + dataEntries[x]))
        
        table.extend(struct.pack(">I", dataBlocksAttributes[0][x + len(rigidBodies)] - offsetDataBegin + 0x14))
        table.extend(struct.pack(">I", 0x01))
        table.extend(struct.pack(">I", dataBlocksAttributes[0][rigidBodyNames.index(constraintNames[x])] - offsetDataBegin))
        
        table.extend(struct.pack(">I", dataBlocksAttributes[0][x + len(rigidBodies)] - offsetDataBegin + 0x18))
        table.extend(struct.pack(">I", 0x01))
        searchName = constraintNames[x].removeprefix("ragdoll_")
        modelArmature = bpy.data.objects[modelArmatureObj].data.name
        bonePhysicsData = bpy.data.armatures[modelArmature].bones[searchName].rigid_body_bones
        table.extend(struct.pack(">I", dataBlocksAttributes[0][rigidBodyNames.index("ragdoll_" + bonePhysicsData.parent)] - offsetDataBegin))
    
    return table

## Table 3 stores pointers to all data blocks in the file, along with constants dictating what type each data block pointed to is.
## The table is arranged in 3 columns in RAM. 
## Column 1 is the pointer to the described data blocks. 
## Column 2 is always zero for the entries we care to edit.
## Column 3 is a constant indicating which type of data is being pointed to in column 1.
## > 0x1C7 = rigid body instance
## > 0xEC  = shape data
## > 0x1A4 = deactivator data
## > 0x442 = constraint instance
## > 0x187 = constraint data

def createTable3(rigidBodyArray, constraintArray, dataBlocksAttributes, offsetDataBegin, offsetTable2Part1, offsetTable2Part2, offsetTable2Part3, table2, headerPos1):
    
    rigidBodies = rigidBodyArray[0]
    rigidBodyNames = rigidBodyArray[2]
    shapeEntries = rigidBodyArray[3]
    deactivatorEntries = rigidBodyArray[4]
    
    constraints = constraintArray[0]
    constraintNames = constraintArray[2]
    dataEntries = constraintArray[3]
    
    # Sadly more hardcoding.
    table = bytearray([
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x30, 
    0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x0A, 
    0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAD, 
    0x00, 0x00, 0x01, 0xB0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 
    ])
    
    table[0x18:0x1C] = struct.pack(">I", bytesToWord(table, 0x18) + headerPos1 - 0x720)
    table[0x24:0x28] = struct.pack(">I", bytesToWord(table, 0x24) + headerPos1 - 0x720)
    
    for x in range(len(rigidBodies)):
        table.extend(struct.pack(">I", dataBlocksAttributes[0][x] - offsetDataBegin))
        table.extend(struct.pack(">I", 0x00))
        table.extend(struct.pack(">I", 0x1C7))
        
        table.extend(struct.pack(">I", dataBlocksAttributes[0][x] - offsetDataBegin + shapeEntries[x]))
        table.extend(struct.pack(">I", 0x00))
        table.extend(struct.pack(">I", 0xEC))
        
        if deactivatorEntries[x] != 0x00:
            table.extend(struct.pack(">I", dataBlocksAttributes[0][x] - offsetDataBegin + deactivatorEntries[x]))
            table.extend(struct.pack(">I", 0x00))
            table.extend(struct.pack(">I", 0x1A4))
        
    for x in range(len(constraints)):
        table.extend(struct.pack(">I", dataBlocksAttributes[0][x + len(rigidBodies)] - offsetDataBegin))
        table.extend(struct.pack(">I", 0x00))
        table.extend(struct.pack(">I", 0x442))
        
        table.extend(struct.pack(">I", dataBlocksAttributes[0][x + len(rigidBodies)] - offsetDataBegin + dataEntries[x]))
        table.extend(struct.pack(">I", 0x00))
        table.extend(struct.pack(">I", 0x187))
    
    return table


def buildHKX(hkx, rigidBodyArray, constraintArray, modelArmatureObj):
    
    rigidBodies = rigidBodyArray[0]
    constraints = constraintArray[0]
    
    offsetDataBegin = bytesToWord(hkx, pointerOffsetDataBegin)
    offsetTable1 = bytesToWord(hkx, pointerOffsetTable1)
    offsetTable2 = bytesToWord(hkx, pointerOffsetTable2)
    offsetTable3 = bytesToWord(hkx, pointerOffsetTable3)
    offsetDataEnd1 = bytesToWord(hkx, pointerOffsetDataEnd[0])
    offsetDataEnd2 = bytesToWord(hkx, pointerOffsetDataEnd[1])
    offsetDataEnd3 = bytesToWord(hkx, pointerOffsetDataEnd[2])
    offsetFooter = bytesToWord(hkx, pointerOffsetFooter)
    
    # Keep in mind much of the data handling related to the file header and footer is hardcoded to get this tool out there in time.
    # To remove this hardcoding, a full reverse engineering must be done of the header and footer of the file, which I do
    # NOT have the time to do, and would rather this not get delayed and become lost tech again.
    # Perhaps someone else can finish this part for a fully open-source implementation.
    # Until then, this works fine. All it needs is a file to pull data from placed in the accompanying folder.
    
    hkxHeader = hkx[:0x830]
    hkxFooter = hkx[0x4EE0:]
    
    # Header editing.
    finalHKX = bytearray()      # Final file bytearray.
    finalHKX.extend(hkxHeader)  # Start with the header.
    headerDataPointersOffset = 0x790
    
    programString = "Blender " + bpy.app.version_string
    while len(programString) < 0x10:
        programString += '\0'
    finalHKX[0x680:0x690] = programString.encode("ascii")
    
    # Create words of null bytes in header that seem to correspond to amount of rigid bodies.
    # They aren't filled by pointers it seems, but the amount of words of null space correspond to the number of rigid bodies.
    headerDataPointers = bytearray()
    for x in range(len(rigidBodies)):
        headerDataPointers.extend(b"\x00\x00\x00\x00")
    for x in range(0x10 - (len(headerDataPointers) % 0x10) + 0x10):
        headerDataPointers.append(0x00)  # 16-byte-aligned padding for string.
    
    finalHKX[0x6D0:0x720] = headerDataPointers              # Insert new header pointers data block.
    headerPos1 = 0x6D0 + len(headerDataPointers)            # Record new position of the data block.
    
    # Creates more words of null bytes necessary for pointers filled in at runtime.
    headerDataPointers2 = bytearray()
    for x in range(len(rigidBodies)):
        headerDataPointers2.extend(b"\x00\x00\x00\x00")
    for x in range(0x10 - (len(headerDataPointers2) % 0x10)):
        headerDataPointers2.append(0x00)     # 16-byte-aligned padding for string.
    dataTypeDelimit = headerPos1 + 0x70 + len(headerDataPointers2)  # Record new position of the data block.
    for x in range(len(constraints)):
        headerDataPointers2.extend(b"\x00\x00\x00\x00")
    for x in range(0x10 - (len(headerDataPointers2) % 0x10)):
        headerDataPointers2.append(0x00)     # 16-byte-aligned padding for string.
    
    finalHKX[(headerPos1 + 0x70):(headerPos1 + 0xF0)] = headerDataPointers2     # Insert new header pointers data block.
    headerPos2 = headerPos1 + 0x70 + len(headerDataPointers2)   # Record new position of the data block.
    
    extraTableOffset = len(finalHKX) - len(hkxHeader)   # Extra offset to push forward or back all the pointer tables relative to vanilla Toon Link.
    dataBlocksAttributes = [[], []]     # List of locations and lengths of new data blocks (rigid bodies and constraints).
    
    # Insert rigid body and constraint data blocks, and record their locations and lengths in the file.
    for rigidBody in rigidBodies:
        dataBlocksAttributes[0].append(len(finalHKX))
        finalHKX.extend(rigidBody)
        dataBlocksAttributes[1].append(len(rigidBody))
    for constraint in constraints:
        dataBlocksAttributes[0].append(len(finalHKX))
        finalHKX.extend(constraint)
        dataBlocksAttributes[1].append(len(constraint))
    
    # Create the three pointer tables and update said pointers.
    
    # Build Table 1.
    finalHKX[pointerOffsetTable1:pointerOffsetTable1 + 4] = struct.pack(">I", len(finalHKX) - offsetDataBegin)
    offsetTable1 = len(finalHKX) - offsetDataBegin
    finalHKX.extend(createTable1(rigidBodyArray, constraintArray, dataBlocksAttributes, offsetDataBegin, headerPos1, headerPos2, dataTypeDelimit))
    
    # Place important information from Table 1 into the header.
    numMeshesOffset = bytesToWord(finalHKX, offsetTable1 + offsetDataBegin + 0x38) + offsetDataBegin
    numRigidBodiesOffset = bytesToWord(finalHKX, offsetTable1 + offsetDataBegin + 0x48) + offsetDataBegin
    numConstraintsOffset = bytesToWord(finalHKX, offsetTable1 + offsetDataBegin + 0x50) + offsetDataBegin
    
    # Number of "meshes" described in the file. Same as number of rigid bodies.
    finalHKX[(numMeshesOffset + 0x04):(numMeshesOffset + 0x08)] = struct.pack(">I", len(rigidBodies))
    
    # Number of rigid bodies described in the file.
    finalHKX[(numRigidBodiesOffset + 0x04):(numRigidBodiesOffset + 0x08)] = struct.pack(">I", len(rigidBodies))
    finalHKX[(numRigidBodiesOffset + 0x08):(numRigidBodiesOffset + 0x0C)] = struct.pack(">I", len(rigidBodies) + 0xC0000000)
    
    # Number of constraints described in the file.
    finalHKX[(numConstraintsOffset + 0x04):(numConstraintsOffset + 0x08)] = struct.pack(">I", len(constraints))
    finalHKX[(numConstraintsOffset + 0x08):(numConstraintsOffset + 0x0C)] = struct.pack(">I", len(constraints) + 0xC0000000)
    
    # Build Table 2.
    finalHKX[pointerOffsetTable2:pointerOffsetTable2 + 4] = struct.pack(">I", len(finalHKX) - offsetDataBegin)
    offsetTable2Part1 = len(finalHKX) - offsetDataBegin
    finalHKX.extend(createTable2Part1(rigidBodyArray, constraintArray, dataBlocksAttributes, offsetDataBegin, headerPos1))
    offsetTable2Part2 = len(finalHKX) - offsetDataBegin
    finalHKX.extend(createTable2Part2(rigidBodyArray, constraintArray, dataBlocksAttributes, offsetDataBegin, headerPos1, headerPos2))
    offsetTable2Part3 = len(finalHKX) - offsetDataBegin
    finalHKX.extend(createTable2Part3(rigidBodyArray, constraintArray, dataBlocksAttributes, offsetDataBegin, headerPos1, headerPos2, modelArmatureObj))
    for x in range(0x10 - (len(finalHKX) % 0x10)):
        finalHKX.append(0xFF)     # 16-byte-aligned padding.
    for x in range(0xB0):
        finalHKX.append(0xFF)     # Lotsa padding.
    
    # Build Table 3.
    finalHKX[pointerOffsetTable3:pointerOffsetTable3 + 4] = struct.pack(">I", len(finalHKX) - offsetDataBegin)
    offsetTable3 = len(finalHKX) - offsetDataBegin
    table2 = finalHKX[(offsetTable2 + offsetDataBegin):(offsetTable3 + offsetDataBegin)]
    finalHKX.extend(createTable3(rigidBodyArray, constraintArray, dataBlocksAttributes, offsetDataBegin, offsetTable2Part1, offsetTable2Part2, offsetTable2Part3, table2, headerPos1))
    
    finalHKX[pointerOffsetDataEnd[0]:pointerOffsetDataEnd[0] + 4] = struct.pack(">I", len(finalHKX) - offsetDataBegin)
    finalHKX[pointerOffsetDataEnd[1]:pointerOffsetDataEnd[1] + 4] = struct.pack(">I", len(finalHKX) - offsetDataBegin)
    finalHKX[pointerOffsetDataEnd[2]:pointerOffsetDataEnd[2] + 4] = struct.pack(">I", len(finalHKX) - offsetDataBegin)
    
    finalHKX[pointerOffsetFooter:pointerOffsetFooter + 4] = struct.pack(">I", len(finalHKX))
    
    finalHKX.extend(hkxFooter)
    
    return finalHKX


## Utility functions for error checking and foolproofing the add-on.

def ShowMessageBox(message = "", title = "Message Box", icon = 'INFO'):

    def draw(self, context):
        self.layout.label(text=message)

    bpy.context.window_manager.popup_menu(draw, title = title, icon = icon)

def isArmatureSelected():
    if bpy.context.object == None or bpy.context.object.type != "ARMATURE":
        ShowMessageBox("Make sure the model armature is selected first!", "Havok Exporter", "ERROR")
        return None
    else:
        return bpy.context.object.name

def doBonesMatch(hkx, modelArmatureObj):
    for x in range(numRigidBodies(hkx)):
        name = getCapsuleName(hkx, x).removeprefix("ragdoll_")
        currentBone = bpy.data.objects[modelArmatureObj].pose.bones.get(name)
        if currentBone is None:
            ShowMessageBox("The destination armature doesn't have bone \"" + name + "\"!", "Havok Exporter", "ERROR")
            return False
    return True


## Main function.

def havokExport(filepath):
    
    modelArmature = isArmatureSelected()
    if isArmatureSelected() == None:
        return None
    
    rigidBodyArray = buildRigidBodyData(modelArmature, getColliderMatrix(modelArmature))
    constraintArray = buildConstraintData(modelArmature, getColliderMatrix(modelArmature))
    
    sortedArrays = sortColliders(rigidBodyArray, constraintArray, modelArmature)
    sortedRigidBodyArray = sortedArrays[0]
    sortedConstraintArray = sortedArrays[1]
    
    # The exporter borrows header and footer data from Toon Link's Havok file.
    # This is done to save time in RE, as these parts need only minimal editing and take up half the damn filesize.
    # I'm not postponing this tool for another year just to reverse something not absolutely necessary to getting it working.
    # This does mean that borrowing data from another file is required, thus I can't share that data in a git repository.
    # That data will need to be ripped from the game. It's pretty simple to do.
    # Ultimately I'd rather have a fully open-source implementation, but to get this actually out, to me this is a fair compromise.
    
    blender_addon_path = os.path.join(bpy.utils.script_path_user(), "addons")
    print("\nBlender addon path: " + blender_addon_path)
    donorFilepath = os.path.join(blender_addon_path, "BlackenedBones", "donorFile", "donor.hkx")
    
    try:
        f = open(donorFilepath, "rb")
    except:
        print("Donor file not found in addons directory. It should be here:\n", donorFilepath)
        print("Trying local directory of blend file instead.")
        donorFilepath = os.path.dirname(bpy.data.filepath)
        donorFilepath = os.path.join(donorFilepath, "donorFile", "donor.hkx")
        try:
            f = open(donorFilepath, "rb")
        except:
            print("Donor file not found in local directory.")
            print("Please place donor.hkx in the addons/donorFile folder.")
    
    hkx = f.read()
    print("\nFound donor file: " + donorFilepath + '\n')
    
    finalHKX = buildHKX(hkx, sortedRigidBodyArray, sortedConstraintArray, modelArmature)
    
    outputFile = open(filepath, "wb")
    outputFile.write(finalHKX)
    outputFile.close()
    f.close()
    
    return None

 
class Exporter(Operator, ExportHelper):
    bl_idname = "blackenedbones.export"
    bl_label = "Export Havok Packfile"
    bl_options = {"PRESET", "UNDO"}
 
    filename_ext = ".hkx"
    
    filter_glob: StringProperty(
        default="*.hkx",
        options={"HIDDEN"}
    )
 
    def execute(self, context):
        if self.filepath.lower()[-4:] != ".hkx":
            ShowMessageBox("Choose a valid filepath!\n (Make sure to save as HKX!)", "Havok Exporter", "ERROR")
            return {"FINISHED"}
        ret = havokExport(self.filepath)
        if ret is not None:
            ShowMessageBox("Exported Havok physics successfully!", "Havok Exporter", "INFO")
        print("Exported Havok Packfile: ", self.filepath, '\n')
        return {"FINISHED"}