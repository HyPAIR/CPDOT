"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import drake.lcmt_constrained_values

class lcmt_desired_body_motion(object):
    __slots__ = ["timestamp", "body_name", "control_during_contact", "constrained_accelerations"]

    def __init__(self):
        self.timestamp = 0
        self.body_name = ""
        self.control_during_contact = False
        self.constrained_accelerations = drake.lcmt_constrained_values()

    def encode(self):
        buf = BytesIO()
        buf.write(lcmt_desired_body_motion._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.timestamp))
        __body_name_encoded = self.body_name.encode('utf-8')
        buf.write(struct.pack('>I', len(__body_name_encoded)+1))
        buf.write(__body_name_encoded)
        buf.write(b"\0")
        buf.write(struct.pack(">b", self.control_during_contact))
        assert self.constrained_accelerations._get_packed_fingerprint() == drake.lcmt_constrained_values._get_packed_fingerprint()
        self.constrained_accelerations._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != lcmt_desired_body_motion._get_packed_fingerprint():
            raise ValueError("Decode error")
        return lcmt_desired_body_motion._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = lcmt_desired_body_motion()
        self.timestamp = struct.unpack(">q", buf.read(8))[0]
        __body_name_len = struct.unpack('>I', buf.read(4))[0]
        self.body_name = buf.read(__body_name_len)[:-1].decode('utf-8', 'replace')
        self.control_during_contact = bool(struct.unpack('b', buf.read(1))[0])
        self.constrained_accelerations = drake.lcmt_constrained_values._decode_one(buf)
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if lcmt_desired_body_motion in parents: return 0
        newparents = parents + [lcmt_desired_body_motion]
        tmphash = (0xb89e4df9a079e47+ drake.lcmt_constrained_values._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if lcmt_desired_body_motion._packed_fingerprint is None:
            lcmt_desired_body_motion._packed_fingerprint = struct.pack(">Q", lcmt_desired_body_motion._get_hash_recursive([]))
        return lcmt_desired_body_motion._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

