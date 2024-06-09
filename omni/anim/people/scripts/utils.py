# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import carb
import math
import omni.usd
from pxr import Gf, UsdGeom, Usd, UsdSkel, AnimGraphSchema


class Utils:
    """
    Class provides utility functions and also stores config variables for character actions.
    """

    """
    ------------------------Config Variables------------------------
    """

    CONFIG = {
        "SecondPerNightyDegreeTurn": 0.15, # 0.25
        "MinDistanceToFinalTarget": 0.15, # 0.35
        "MinDistanceToIntermediateTarget": 0.25, # 0.45
        "WalkBlendTime": 0.2,
        "DistanceToOccupyQueueSpot": 1.5
    }

    ZERO_POS = carb.Float3(0, 0, 0)
    IDENTITY_ROT = carb.Float4(0, 0, 0, 1)

    """
    ------------------------Carb-Gf Conversion ------------------------
    """
    def carb_float4_to_quatd(float4):
        quat = Gf.Quatd()
        quat.SetReal(float4[3])
        quat.SetImaginary(float4[0], float4[1], float4[2])
        return quat

    def quatd_to_carb_float4(quatd):
        gf_i = quatd.GetImaginary()
        return carb.Float4(gf_i[0], gf_i[1], gf_i[2], quatd.GetReal())

    def vec3_to_float3(vec3):
        return carb.Float3(vec3[0], vec3[1], vec3[2])

    def float3_to_vec3d(float3):
        return Gf.Vec3d(float3[0], float3[1], float3[2])

    """
    ------------------------Carb vector utility functions ------------------------
    """
    def cap(f, lower, upper):
        return min(max(lower, f), upper)

    def equal3(a, b):
        return a.x == b.x and a.y == b.y and a.z == a.z 

    def add3(a, b):
        return carb.Float3(a[0] + b[0], a[1] + b[1], a[2] + b[2])

    def add4(a, b):
        return carb.Float4(a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3])

    def sub3(a, b):
        return carb.Float3(a[0] - b[0], a[1] - b[1], a[2] - b[2])

    def sub4(a, b):
        return carb.Float4(a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3])

    def length3(v):
        return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

    def length4(v):
        return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3])

    def scale3(v, f):
        return carb.Float3(v[0] * f, v[1] * f, v[2] * f)

    def scale4(v, f):
        return carb.Float4(v[0] * f, v[1] * f, v[2] * f, v[3] * f)

    def dot3(a, b):
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

    def dot4(a, b):
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]

    def cross3(a, b):
        return carb.Float3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)

    def dist3(a, b):
        return Utils.length3(Utils.sub3(b, a))

    def lerp3(a, b, t):
        return Utils.add3(a, Utils.scale3(Utils.sub3(b, a), t))

    def lerp4(a, b, t):
        return Utils.add4(a, Utils.scale4(Utils.sub4(b, a), t))

    def nlerp4(a, b, t):
        return Utils.normalize4(Utils.lerp4(a, b, t))

    def normalize3(v):
        gf_v = Gf.Vec3d(v.x, v.y, v.z)
        gf_v_normalized = gf_v.GetNormalized()
        return carb.Float3(gf_v_normalized[0], gf_v_normalized[1], gf_v_normalized[2])

    def normalize4(v):
        length = Utils.length4(v)
        if length > 0.001:
            return Utils.scale4(v, 1 / length)
        else:
            return Utils.IDENTITY_ROT

    """
    ------------------------Getters/Setters for Prims------------------------
    """

    def get_prim_pos(stage, prim_path, prim=None):
        if prim is None:
            prim = stage.GetPrimAtPath(prim_path)
        if prim:
            gf_translation = UsdGeom.XformCache().GetLocalToWorldTransform(prim).ExtractTranslation()
            return carb.Float3(gf_translation[0], gf_translation[1], gf_translation[2])
        else:
            return None

    def get_prim_rot_quat(stage, prim_path, prim=None):
        if prim is None:
            prim = stage.GetPrimAtPath(prim_path)
        if prim:
            gf_rotation = UsdGeom.XformCache().GetLocalToWorldTransform(prim).ExtractRotationQuat()
            return gf_rotation
        else:
            return None

    def get_prim_scale(stage, prim_path, prim=None):
        if prim is None:
            prim = stage.GetPrimAtPath(prim_path)
        if prim:
            gf_scale = prim.GetAttribute("xformOp:scale").Get()
            return gf_scale
        else:
            return None

    def set_prim_pos(prim, new_pos):
        prim.GetAttribute("xformOp:translate").Set(new_pos)

    def set_prim_rot(prim, new_rot):
        prim.GetAttribute("xformOp:orient").Set(new_rot)

    def set_prim_rot_pivot(prim, pivot):
        prim.GetAttribute("xformOp:translate:pivot").Set(pivot)

    def set_prim_trans(prim, new_pos, new_rot):
        trans_mat = Gf.Matrix4d(Gf.Rotation(new_rot), new_pos)
        prim.GetAttribute("xformOp:transform").Set(trans_mat)

    def set_prim_scale(prim, new_scale):
        prim.GetAttribute("xformOp:scale").Set(new_scale)


    """
    ------------------------Utilities Functions for Characters------------------------
    """
    def get_character_transform(c):
        pos = carb.Float3(0, 0, 0)
        rot = carb.Float4(0, 0, 0, 0)
        c.get_world_transform(pos, rot)
        return pos, rot

    def get_character_pos(c):
        pos, rot = Utils.get_character_transform(c)
        return pos

    def get_character_rot(c):
        pos, rot = Utils.get_character_transform(c)
        return rot

    """
    ------------------------Other Utilities Functions------------------------
    """

    def rotZ3(v, d):
        rotYMat = Gf.Matrix3d(Gf.Rotation(Gf.Vec3d(0, 0, 1), d))
        vRot = Gf.Vec3d(v.x, v.y, v.z) * rotYMat
        return carb.Float3(vRot[0], vRot[1], vRot[2])
    
    def convert_to_angle(quat_rot):
        rot_in_angle = Gf.Rotation(Gf.Quatd(quat_rot.w, quat_rot.x, quat_rot.y, quat_rot.z))
        zaxis = rot_in_angle.GetAxis()[2]
        rot_angle = rot_in_angle.GetAngle()
        if zaxis < 0:
            rot_angle = -rot_angle
        return rot_angle

    def cal_rotation_difference(angle_a, angle_b):
        angle_diff = angle_a - angle_b
        if angle_diff < 0:
            angle_diff = -angle_diff
        if (360-angle_diff)<angle_diff:
            angle_diff = 360- angle_diff
        return angle_diff 

    def is_character(prim_path):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if prim.IsA(UsdSkel.Root) and prim.HasAPI(AnimGraphSchema.AnimationGraphAPI):
            return True
        else:
            return False
        
    def convert_angle_to_quatd(z_angle):
        rotation = None
        if z_angle < 0:
            rotation = Gf.Rotation(Gf.Vec3d(0,0,-1), -z_angle)
        else:
            rotation = Gf.Rotation(Gf.Vec3d(0,0,1), z_angle)
        quat_value = rotation.GetQuat()
        imaginary = quat_value.GetImaginary()
        real = quat_value.GetReal()
        return carb.Float4(imaginary[0], imaginary[1], imaginary[2], real)

    def get_object_radius(prim_path):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        box_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
        bound = box_cache.ComputeWorldBound(prim)
        range = bound.ComputeAlignedBox()
        bboxMin = range.GetMin()
        bboxMax = range.GetMax()
        bbox_x = bboxMax[0]-bboxMin[0]
        bbox_y = bboxMax[1]-bboxMin[1]
        radius = max(bbox_y, bbox_x)
        return radius/2
