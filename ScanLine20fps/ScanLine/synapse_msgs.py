"""
Minimal nanopb-compatible protobuf encoder for synapse messages.

Encodes only the messages needed for the ScanLine controller.
No external protobuf library required — uses raw proto3 wire format.

Proto definitions (from synapse_protobuf/proto/):
    message Vector3 { double x = 1; double y = 2; double z = 3; }
    message Twist   { Vector3 linear = 1; Vector3 angular = 2; }
"""

import struct


def _encode_double(value: float) -> bytes:
    """Encode a double as little-endian 8 bytes (proto wire type 1 = fixed64)."""
    return struct.pack("<d", value)


def _encode_field_tag(field_number: int, wire_type: int) -> bytes:
    """Encode a protobuf field tag as a varint."""
    return _encode_varint((field_number << 3) | wire_type)


def _encode_varint(value: int) -> bytes:
    """Encode an unsigned integer as a protobuf varint."""
    result = bytearray()
    while value > 0x7F:
        result.append((value & 0x7F) | 0x80)
        value >>= 7
    result.append(value & 0x7F)
    return bytes(result)


def encode_vector3(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> bytes:
    """Encode a Vector3 message.

    Proto3: zero-value fields are omitted.
    """
    data = bytearray()
    if x != 0.0:
        data.extend(_encode_field_tag(1, 1))  # field 1, wire type 1 (fixed64)
        data.extend(_encode_double(x))
    if y != 0.0:
        data.extend(_encode_field_tag(2, 1))
        data.extend(_encode_double(y))
    if z != 0.0:
        data.extend(_encode_field_tag(3, 1))
        data.extend(_encode_double(z))
    return bytes(data)


def encode_twist(linear_x: float = 0.0, linear_y: float = 0.0, linear_z: float = 0.0,
                 angular_x: float = 0.0, angular_y: float = 0.0, angular_z: float = 0.0) -> bytes:
    """Encode a Twist message (linear + angular Vector3).

    This is the cmd_vel message format expected by the CANHUB-K3.

    Args:
        linear_x: Forward speed (m/s). Maps to cmd_vel.linear.x.
        linear_y: Lateral speed (unused in b3rb).
        linear_z: Vertical speed (unused in b3rb).
        angular_x: Roll rate (unused in b3rb).
        angular_y: Pitch rate (unused in b3rb).
        angular_z: Yaw/steering rate. Maps to cmd_vel.angular.z.

    Returns:
        Protobuf-encoded bytes for the Twist message.
    """
    data = bytearray()

    # field 1: linear (Vector3) — wire type 2 (length-delimited)
    linear = encode_vector3(linear_x, linear_y, linear_z)
    if linear:
        data.extend(_encode_field_tag(1, 2))
        data.extend(_encode_varint(len(linear)))
        data.extend(linear)

    # field 2: angular (Vector3) — wire type 2 (length-delimited)
    angular = encode_vector3(angular_x, angular_y, angular_z)
    if angular:
        data.extend(_encode_field_tag(2, 2))
        data.extend(_encode_varint(len(angular)))
        data.extend(angular)

    return bytes(data)
