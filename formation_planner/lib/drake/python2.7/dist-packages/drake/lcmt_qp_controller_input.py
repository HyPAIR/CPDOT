"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import drake.lcmt_joint_pd_override

import drake.lcmt_body_wrench_data

import drake.lcmt_support_data

import drake.lcmt_body_motion_data

import drake.lcmt_whole_body_data

import drake.lcmt_zmp_data

class lcmt_qp_controller_input(object):
    __slots__ = ["be_silent", "timestamp", "zmp_data", "num_support_data", "support_data", "num_tracked_bodies", "body_motion_data", "num_external_wrenches", "body_wrench_data", "whole_body_data", "num_joint_pd_overrides", "joint_pd_override", "param_set_name"]

    def __init__(self):
        self.be_silent = False
        self.timestamp = 0
        self.zmp_data = drake.lcmt_zmp_data()
        self.num_support_data = 0
        self.support_data = []
        self.num_tracked_bodies = 0
        self.body_motion_data = []
        self.num_external_wrenches = 0
        self.body_wrench_data = []
        self.whole_body_data = drake.lcmt_whole_body_data()
        self.num_joint_pd_overrides = 0
        self.joint_pd_override = []
        self.param_set_name = ""

    def encode(self):
        buf = BytesIO()
        buf.write(lcmt_qp_controller_input._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">bq", self.be_silent, self.timestamp))
        assert self.zmp_data._get_packed_fingerprint() == drake.lcmt_zmp_data._get_packed_fingerprint()
        self.zmp_data._encode_one(buf)
        buf.write(struct.pack(">i", self.num_support_data))
        for i0 in range(self.num_support_data):
            assert self.support_data[i0]._get_packed_fingerprint() == drake.lcmt_support_data._get_packed_fingerprint()
            self.support_data[i0]._encode_one(buf)
        buf.write(struct.pack(">i", self.num_tracked_bodies))
        for i0 in range(self.num_tracked_bodies):
            assert self.body_motion_data[i0]._get_packed_fingerprint() == drake.lcmt_body_motion_data._get_packed_fingerprint()
            self.body_motion_data[i0]._encode_one(buf)
        buf.write(struct.pack(">i", self.num_external_wrenches))
        for i0 in range(self.num_external_wrenches):
            assert self.body_wrench_data[i0]._get_packed_fingerprint() == drake.lcmt_body_wrench_data._get_packed_fingerprint()
            self.body_wrench_data[i0]._encode_one(buf)
        assert self.whole_body_data._get_packed_fingerprint() == drake.lcmt_whole_body_data._get_packed_fingerprint()
        self.whole_body_data._encode_one(buf)
        buf.write(struct.pack(">i", self.num_joint_pd_overrides))
        for i0 in range(self.num_joint_pd_overrides):
            assert self.joint_pd_override[i0]._get_packed_fingerprint() == drake.lcmt_joint_pd_override._get_packed_fingerprint()
            self.joint_pd_override[i0]._encode_one(buf)
        __param_set_name_encoded = self.param_set_name.encode('utf-8')
        buf.write(struct.pack('>I', len(__param_set_name_encoded)+1))
        buf.write(__param_set_name_encoded)
        buf.write(b"\0")

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != lcmt_qp_controller_input._get_packed_fingerprint():
            raise ValueError("Decode error")
        return lcmt_qp_controller_input._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = lcmt_qp_controller_input()
        self.be_silent = bool(struct.unpack('b', buf.read(1))[0])
        self.timestamp = struct.unpack(">q", buf.read(8))[0]
        self.zmp_data = drake.lcmt_zmp_data._decode_one(buf)
        self.num_support_data = struct.unpack(">i", buf.read(4))[0]
        self.support_data = []
        for i0 in range(self.num_support_data):
            self.support_data.append(drake.lcmt_support_data._decode_one(buf))
        self.num_tracked_bodies = struct.unpack(">i", buf.read(4))[0]
        self.body_motion_data = []
        for i0 in range(self.num_tracked_bodies):
            self.body_motion_data.append(drake.lcmt_body_motion_data._decode_one(buf))
        self.num_external_wrenches = struct.unpack(">i", buf.read(4))[0]
        self.body_wrench_data = []
        for i0 in range(self.num_external_wrenches):
            self.body_wrench_data.append(drake.lcmt_body_wrench_data._decode_one(buf))
        self.whole_body_data = drake.lcmt_whole_body_data._decode_one(buf)
        self.num_joint_pd_overrides = struct.unpack(">i", buf.read(4))[0]
        self.joint_pd_override = []
        for i0 in range(self.num_joint_pd_overrides):
            self.joint_pd_override.append(drake.lcmt_joint_pd_override._decode_one(buf))
        __param_set_name_len = struct.unpack('>I', buf.read(4))[0]
        self.param_set_name = buf.read(__param_set_name_len)[:-1].decode('utf-8', 'replace')
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if lcmt_qp_controller_input in parents: return 0
        newparents = parents + [lcmt_qp_controller_input]
        tmphash = (0x10daf84344767104+ drake.lcmt_zmp_data._get_hash_recursive(newparents)+ drake.lcmt_support_data._get_hash_recursive(newparents)+ drake.lcmt_body_motion_data._get_hash_recursive(newparents)+ drake.lcmt_body_wrench_data._get_hash_recursive(newparents)+ drake.lcmt_whole_body_data._get_hash_recursive(newparents)+ drake.lcmt_joint_pd_override._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if lcmt_qp_controller_input._packed_fingerprint is None:
            lcmt_qp_controller_input._packed_fingerprint = struct.pack(">Q", lcmt_qp_controller_input._get_hash_recursive([]))
        return lcmt_qp_controller_input._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

