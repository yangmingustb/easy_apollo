# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: clock.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='clock.proto',
  package='apollo.cyber.proto',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x0b\x63lock.proto\x12\x12\x61pollo.cyber.proto\"\x16\n\x05\x43lock\x12\r\n\x05\x63lock\x18\x01 \x02(\x04'
)




_CLOCK = _descriptor.Descriptor(
  name='Clock',
  full_name='apollo.cyber.proto.Clock',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='clock', full_name='apollo.cyber.proto.Clock.clock', index=0,
      number=1, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
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
  serialized_start=35,
  serialized_end=57,
)

DESCRIPTOR.message_types_by_name['Clock'] = _CLOCK
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Clock = _reflection.GeneratedProtocolMessageType('Clock', (_message.Message,), {
  'DESCRIPTOR' : _CLOCK,
  '__module__' : 'clock_pb2'
  # @@protoc_insertion_point(class_scope:apollo.cyber.proto.Clock)
  })
_sym_db.RegisterMessage(Clock)


# @@protoc_insertion_point(module_scope)
