# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/dreamview/proto/hmi_config.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/dreamview/proto/hmi_config.proto',
  package='apollo.dreamview',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n(modules/dreamview/proto/hmi_config.proto\x12\x10\x61pollo.dreamview\"\xc0\x02\n\tHMIConfig\x12\x35\n\x05modes\x18\x01 \x03(\x0b\x32&.apollo.dreamview.HMIConfig.ModesEntry\x12\x33\n\x04maps\x18\x02 \x03(\x0b\x32%.apollo.dreamview.HMIConfig.MapsEntry\x12;\n\x08vehicles\x18\x03 \x03(\x0b\x32).apollo.dreamview.HMIConfig.VehiclesEntry\x1a,\n\nModesEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\r\n\x05value\x18\x02 \x01(\t:\x02\x38\x01\x1a+\n\tMapsEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\r\n\x05value\x18\x02 \x01(\t:\x02\x38\x01\x1a/\n\rVehiclesEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\r\n\x05value\x18\x02 \x01(\t:\x02\x38\x01\"}\n\x0bVehicleData\x12:\n\ndata_files\x18\x01 \x03(\x0b\x32&.apollo.dreamview.VehicleData.DataFile\x1a\x32\n\x08\x44\x61taFile\x12\x13\n\x0bsource_path\x18\x01 \x01(\t\x12\x11\n\tdest_path\x18\x02 \x01(\t*\xb4\x04\n\tHMIAction\x12\x08\n\x04NONE\x10\x00\x12\x0e\n\nSETUP_MODE\x10\x01\x12\x0e\n\nRESET_MODE\x10\x02\x12\x13\n\x0f\x45NTER_AUTO_MODE\x10\x03\x12\r\n\tDISENGAGE\x10\x04\x12\x0f\n\x0b\x43HANGE_MODE\x10\x05\x12\x0e\n\nCHANGE_MAP\x10\x06\x12\x12\n\x0e\x43HANGE_VEHICLE\x10\x07\x12\x10\n\x0cSTART_MODULE\x10\x08\x12\x0f\n\x0bSTOP_MODULE\x10\t\x12\x13\n\x0f\x43HANGE_SCENARIO\x10\n\x12\x17\n\x13\x43HANGE_SCENARIO_SET\x10\x0b\x12\x12\n\x0eLOAD_SCENARIOS\x10\x0c\x12\x17\n\x13\x44\x45LETE_SCENARIO_SET\x10\r\x12\x17\n\x13LOAD_DYNAMIC_MODELS\x10\x0e\x12\x18\n\x14\x43HANGE_DYNAMIC_MODEL\x10\x0f\x12\x18\n\x14\x44\x45LETE_DYNAMIC_MODEL\x10\x10\x12\x11\n\rCHANGE_RECORD\x10\x11\x12\x11\n\rDELETE_RECORD\x10\x12\x12\x10\n\x0cLOAD_RECORDS\x10\x13\x12\x0f\n\x0bSTOP_RECORD\x10\x14\x12\x14\n\x10\x43HANGE_OPERATION\x10\x15\x12\x17\n\x13\x44\x45LETE_VEHICLE_CONF\x10\x16\x12\x13\n\x0f\x44\x45LETE_V2X_CONF\x10\x17\x12\x0e\n\nDELETE_MAP\x10\x18\x12\x14\n\x10LOAD_RTK_RECORDS\x10\x19\x12\x15\n\x11\x43HANGE_RTK_RECORD\x10\x1a\x12\x0f\n\x0bLOAD_RECORD\x10\x1b'
)

_HMIACTION = _descriptor.EnumDescriptor(
  name='HMIAction',
  full_name='apollo.dreamview.HMIAction',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NONE', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SETUP_MODE', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='RESET_MODE', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ENTER_AUTO_MODE', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='DISENGAGE', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CHANGE_MODE', index=5, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CHANGE_MAP', index=6, number=6,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CHANGE_VEHICLE', index=7, number=7,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='START_MODULE', index=8, number=8,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='STOP_MODULE', index=9, number=9,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CHANGE_SCENARIO', index=10, number=10,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CHANGE_SCENARIO_SET', index=11, number=11,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='LOAD_SCENARIOS', index=12, number=12,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='DELETE_SCENARIO_SET', index=13, number=13,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='LOAD_DYNAMIC_MODELS', index=14, number=14,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CHANGE_DYNAMIC_MODEL', index=15, number=15,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='DELETE_DYNAMIC_MODEL', index=16, number=16,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CHANGE_RECORD', index=17, number=17,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='DELETE_RECORD', index=18, number=18,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='LOAD_RECORDS', index=19, number=19,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='STOP_RECORD', index=20, number=20,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CHANGE_OPERATION', index=21, number=21,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='DELETE_VEHICLE_CONF', index=22, number=22,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='DELETE_V2X_CONF', index=23, number=23,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='DELETE_MAP', index=24, number=24,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='LOAD_RTK_RECORDS', index=25, number=25,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CHANGE_RTK_RECORD', index=26, number=26,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='LOAD_RECORD', index=27, number=27,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=513,
  serialized_end=1077,
)
_sym_db.RegisterEnumDescriptor(_HMIACTION)

HMIAction = enum_type_wrapper.EnumTypeWrapper(_HMIACTION)
NONE = 0
SETUP_MODE = 1
RESET_MODE = 2
ENTER_AUTO_MODE = 3
DISENGAGE = 4
CHANGE_MODE = 5
CHANGE_MAP = 6
CHANGE_VEHICLE = 7
START_MODULE = 8
STOP_MODULE = 9
CHANGE_SCENARIO = 10
CHANGE_SCENARIO_SET = 11
LOAD_SCENARIOS = 12
DELETE_SCENARIO_SET = 13
LOAD_DYNAMIC_MODELS = 14
CHANGE_DYNAMIC_MODEL = 15
DELETE_DYNAMIC_MODEL = 16
CHANGE_RECORD = 17
DELETE_RECORD = 18
LOAD_RECORDS = 19
STOP_RECORD = 20
CHANGE_OPERATION = 21
DELETE_VEHICLE_CONF = 22
DELETE_V2X_CONF = 23
DELETE_MAP = 24
LOAD_RTK_RECORDS = 25
CHANGE_RTK_RECORD = 26
LOAD_RECORD = 27



_HMICONFIG_MODESENTRY = _descriptor.Descriptor(
  name='ModesEntry',
  full_name='apollo.dreamview.HMIConfig.ModesEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='apollo.dreamview.HMIConfig.ModesEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.dreamview.HMIConfig.ModesEntry.value', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=b'8\001',
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=245,
  serialized_end=289,
)

_HMICONFIG_MAPSENTRY = _descriptor.Descriptor(
  name='MapsEntry',
  full_name='apollo.dreamview.HMIConfig.MapsEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='apollo.dreamview.HMIConfig.MapsEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.dreamview.HMIConfig.MapsEntry.value', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=b'8\001',
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=291,
  serialized_end=334,
)

_HMICONFIG_VEHICLESENTRY = _descriptor.Descriptor(
  name='VehiclesEntry',
  full_name='apollo.dreamview.HMIConfig.VehiclesEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='apollo.dreamview.HMIConfig.VehiclesEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.dreamview.HMIConfig.VehiclesEntry.value', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=b'8\001',
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=336,
  serialized_end=383,
)

_HMICONFIG = _descriptor.Descriptor(
  name='HMIConfig',
  full_name='apollo.dreamview.HMIConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='modes', full_name='apollo.dreamview.HMIConfig.modes', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='maps', full_name='apollo.dreamview.HMIConfig.maps', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='vehicles', full_name='apollo.dreamview.HMIConfig.vehicles', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_HMICONFIG_MODESENTRY, _HMICONFIG_MAPSENTRY, _HMICONFIG_VEHICLESENTRY, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=63,
  serialized_end=383,
)


_VEHICLEDATA_DATAFILE = _descriptor.Descriptor(
  name='DataFile',
  full_name='apollo.dreamview.VehicleData.DataFile',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='source_path', full_name='apollo.dreamview.VehicleData.DataFile.source_path', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='dest_path', full_name='apollo.dreamview.VehicleData.DataFile.dest_path', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
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
  serialized_start=460,
  serialized_end=510,
)

_VEHICLEDATA = _descriptor.Descriptor(
  name='VehicleData',
  full_name='apollo.dreamview.VehicleData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='data_files', full_name='apollo.dreamview.VehicleData.data_files', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_VEHICLEDATA_DATAFILE, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=385,
  serialized_end=510,
)

_HMICONFIG_MODESENTRY.containing_type = _HMICONFIG
_HMICONFIG_MAPSENTRY.containing_type = _HMICONFIG
_HMICONFIG_VEHICLESENTRY.containing_type = _HMICONFIG
_HMICONFIG.fields_by_name['modes'].message_type = _HMICONFIG_MODESENTRY
_HMICONFIG.fields_by_name['maps'].message_type = _HMICONFIG_MAPSENTRY
_HMICONFIG.fields_by_name['vehicles'].message_type = _HMICONFIG_VEHICLESENTRY
_VEHICLEDATA_DATAFILE.containing_type = _VEHICLEDATA
_VEHICLEDATA.fields_by_name['data_files'].message_type = _VEHICLEDATA_DATAFILE
DESCRIPTOR.message_types_by_name['HMIConfig'] = _HMICONFIG
DESCRIPTOR.message_types_by_name['VehicleData'] = _VEHICLEDATA
DESCRIPTOR.enum_types_by_name['HMIAction'] = _HMIACTION
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

HMIConfig = _reflection.GeneratedProtocolMessageType('HMIConfig', (_message.Message,), {

  'ModesEntry' : _reflection.GeneratedProtocolMessageType('ModesEntry', (_message.Message,), {
    'DESCRIPTOR' : _HMICONFIG_MODESENTRY,
    '__module__' : 'modules.dreamview.proto.hmi_config_pb2'
    # @@protoc_insertion_point(class_scope:apollo.dreamview.HMIConfig.ModesEntry)
    })
  ,

  'MapsEntry' : _reflection.GeneratedProtocolMessageType('MapsEntry', (_message.Message,), {
    'DESCRIPTOR' : _HMICONFIG_MAPSENTRY,
    '__module__' : 'modules.dreamview.proto.hmi_config_pb2'
    # @@protoc_insertion_point(class_scope:apollo.dreamview.HMIConfig.MapsEntry)
    })
  ,

  'VehiclesEntry' : _reflection.GeneratedProtocolMessageType('VehiclesEntry', (_message.Message,), {
    'DESCRIPTOR' : _HMICONFIG_VEHICLESENTRY,
    '__module__' : 'modules.dreamview.proto.hmi_config_pb2'
    # @@protoc_insertion_point(class_scope:apollo.dreamview.HMIConfig.VehiclesEntry)
    })
  ,
  'DESCRIPTOR' : _HMICONFIG,
  '__module__' : 'modules.dreamview.proto.hmi_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.dreamview.HMIConfig)
  })
_sym_db.RegisterMessage(HMIConfig)
_sym_db.RegisterMessage(HMIConfig.ModesEntry)
_sym_db.RegisterMessage(HMIConfig.MapsEntry)
_sym_db.RegisterMessage(HMIConfig.VehiclesEntry)

VehicleData = _reflection.GeneratedProtocolMessageType('VehicleData', (_message.Message,), {

  'DataFile' : _reflection.GeneratedProtocolMessageType('DataFile', (_message.Message,), {
    'DESCRIPTOR' : _VEHICLEDATA_DATAFILE,
    '__module__' : 'modules.dreamview.proto.hmi_config_pb2'
    # @@protoc_insertion_point(class_scope:apollo.dreamview.VehicleData.DataFile)
    })
  ,
  'DESCRIPTOR' : _VEHICLEDATA,
  '__module__' : 'modules.dreamview.proto.hmi_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.dreamview.VehicleData)
  })
_sym_db.RegisterMessage(VehicleData)
_sym_db.RegisterMessage(VehicleData.DataFile)


_HMICONFIG_MODESENTRY._options = None
_HMICONFIG_MAPSENTRY._options = None
_HMICONFIG_VEHICLESENTRY._options = None
# @@protoc_insertion_point(module_scope)
