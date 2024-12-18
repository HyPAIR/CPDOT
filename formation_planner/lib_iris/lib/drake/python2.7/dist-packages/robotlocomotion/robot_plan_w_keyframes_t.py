"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import robotlocomotion.grasp_transition_state_t

import bot_core.robot_state_t

class robot_plan_w_keyframes_t(object):
    __slots__ = ["utime", "robot_name", "num_keyframes", "num_breakpoints", "num_states", "is_keyframe", "is_breakpoint", "plan", "plan_info", "num_grasp_transitions", "grasps", "num_bytes", "matlab_data"]

    def __init__(self):
        self.utime = 0
        self.robot_name = ""
        self.num_keyframes = 0
        self.num_breakpoints = 0
        self.num_states = 0
        self.is_keyframe = []
        self.is_breakpoint = []
        self.plan = []
        self.plan_info = []
        self.num_grasp_transitions = 0
        self.grasps = []
        self.num_bytes = 0
        self.matlab_data = ""

    def encode(self):
        buf = BytesIO()
        buf.write(robot_plan_w_keyframes_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.utime))
        __robot_name_encoded = self.robot_name.encode('utf-8')
        buf.write(struct.pack('>I', len(__robot_name_encoded)+1))
        buf.write(__robot_name_encoded)
        buf.write(b"\0")
        buf.write(struct.pack(">iii", self.num_keyframes, self.num_breakpoints, self.num_states))
        buf.write(struct.pack('>%db' % self.num_states, *self.is_keyframe[:self.num_states]))
        buf.write(struct.pack('>%db' % self.num_states, *self.is_breakpoint[:self.num_states]))
        for i0 in range(self.num_states):
            assert self.plan[i0]._get_packed_fingerprint() == bot_core.robot_state_t._get_packed_fingerprint()
            self.plan[i0]._encode_one(buf)
        buf.write(struct.pack('>%di' % self.num_states, *self.plan_info[:self.num_states]))
        buf.write(struct.pack(">i", self.num_grasp_transitions))
        for i0 in range(self.num_grasp_transitions):
            assert self.grasps[i0]._get_packed_fingerprint() == robotlocomotion.grasp_transition_state_t._get_packed_fingerprint()
            self.grasps[i0]._encode_one(buf)
        buf.write(struct.pack(">i", self.num_bytes))
        buf.write(bytearray(self.matlab_data[:self.num_bytes]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != robot_plan_w_keyframes_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return robot_plan_w_keyframes_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = robot_plan_w_keyframes_t()
        self.utime = struct.unpack(">q", buf.read(8))[0]
        __robot_name_len = struct.unpack('>I', buf.read(4))[0]
        self.robot_name = buf.read(__robot_name_len)[:-1].decode('utf-8', 'replace')
        self.num_keyframes, self.num_breakpoints, self.num_states = struct.unpack(">iii", buf.read(12))
        self.is_keyframe = map(bool, struct.unpack('>%db' % self.num_states, buf.read(self.num_states)))
        self.is_breakpoint = map(bool, struct.unpack('>%db' % self.num_states, buf.read(self.num_states)))
        self.plan = []
        for i0 in range(self.num_states):
            self.plan.append(bot_core.robot_state_t._decode_one(buf))
        self.plan_info = struct.unpack('>%di' % self.num_states, buf.read(self.num_states * 4))
        self.num_grasp_transitions = struct.unpack(">i", buf.read(4))[0]
        self.grasps = []
        for i0 in range(self.num_grasp_transitions):
            self.grasps.append(robotlocomotion.grasp_transition_state_t._decode_one(buf))
        self.num_bytes = struct.unpack(">i", buf.read(4))[0]
        self.matlab_data = buf.read(self.num_bytes)
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if robot_plan_w_keyframes_t in parents: return 0
        newparents = parents + [robot_plan_w_keyframes_t]
        tmphash = (0xf27c84dcdd3fbfd0+ bot_core.robot_state_t._get_hash_recursive(newparents)+ robotlocomotion.grasp_transition_state_t._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if robot_plan_w_keyframes_t._packed_fingerprint is None:
            robot_plan_w_keyframes_t._packed_fingerprint = struct.pack(">Q", robot_plan_w_keyframes_t._get_hash_recursive([]))
        return robot_plan_w_keyframes_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

