# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/common_msgs/external_command_msgs/free_space_command.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common.proto import header_pb2 as modules_dot_common_dot_proto_dot_header__pb2
from modules.common_msgs.external_command_msgs import geometry_pb2 as modules_dot_common__msgs_dot_external__command__msgs_dot_geometry__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/common_msgs/external_command_msgs/free_space_command.proto',
  package='apollo.external_command',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\nBmodules/common_msgs/external_command_msgs/free_space_command.proto\x12\x17\x61pollo.external_command\x1a!modules/common/proto/header.proto\x1a\x38modules/common_msgs/external_command_msgs/geometry.proto\"\x9b\x02\n\x10\x46reeSpaceCommand\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.apollo.common.Header\x12\x16\n\ncommand_id\x18\x02 \x01(\x03:\x02-1\x12\x38\n\x11parking_spot_pose\x18\x03 \x02(\x0b\x32\x1d.apollo.external_command.Pose\x12=\n\x10non_drivable_roi\x18\x04 \x03(\x0b\x32#.apollo.external_command.RoiPolygon\x12\x39\n\x0c\x64rivable_roi\x18\x05 \x02(\x0b\x32#.apollo.external_command.RoiPolygon\x12\x14\n\x0ctarget_speed\x18\x06 \x01(\x01'
  ,
  dependencies=[modules_dot_common_dot_proto_dot_header__pb2.DESCRIPTOR,modules_dot_common__msgs_dot_external__command__msgs_dot_geometry__pb2.DESCRIPTOR,])




_FREESPACECOMMAND = _descriptor.Descriptor(
  name='FreeSpaceCommand',
  full_name='apollo.external_command.FreeSpaceCommand',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='apollo.external_command.FreeSpaceCommand.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='command_id', full_name='apollo.external_command.FreeSpaceCommand.command_id', index=1,
      number=2, type=3, cpp_type=2, label=1,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='parking_spot_pose', full_name='apollo.external_command.FreeSpaceCommand.parking_spot_pose', index=2,
      number=3, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='non_drivable_roi', full_name='apollo.external_command.FreeSpaceCommand.non_drivable_roi', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='drivable_roi', full_name='apollo.external_command.FreeSpaceCommand.drivable_roi', index=4,
      number=5, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='target_speed', full_name='apollo.external_command.FreeSpaceCommand.target_speed', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=189,
  serialized_end=472,
)

_FREESPACECOMMAND.fields_by_name['header'].message_type = modules_dot_common_dot_proto_dot_header__pb2._HEADER
_FREESPACECOMMAND.fields_by_name['parking_spot_pose'].message_type = modules_dot_common__msgs_dot_external__command__msgs_dot_geometry__pb2._POSE
_FREESPACECOMMAND.fields_by_name['non_drivable_roi'].message_type = modules_dot_common__msgs_dot_external__command__msgs_dot_geometry__pb2._ROIPOLYGON
_FREESPACECOMMAND.fields_by_name['drivable_roi'].message_type = modules_dot_common__msgs_dot_external__command__msgs_dot_geometry__pb2._ROIPOLYGON
DESCRIPTOR.message_types_by_name['FreeSpaceCommand'] = _FREESPACECOMMAND
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

FreeSpaceCommand = _reflection.GeneratedProtocolMessageType('FreeSpaceCommand', (_message.Message,), {
  'DESCRIPTOR' : _FREESPACECOMMAND,
  '__module__' : 'modules.common_msgs.external_command_msgs.free_space_command_pb2'
  # @@protoc_insertion_point(class_scope:apollo.external_command.FreeSpaceCommand)
  })
_sym_db.RegisterMessage(FreeSpaceCommand)


# @@protoc_insertion_point(module_scope)
