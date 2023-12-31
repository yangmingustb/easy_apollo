// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/audio/proto/audio_common.proto

#include "modules/audio/proto/audio_common.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
namespace apollo {
namespace audio {
}  // namespace audio
}  // namespace apollo
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_modules_2faudio_2fproto_2faudio_5fcommon_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_modules_2faudio_2fproto_2faudio_5fcommon_2eproto[3];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2faudio_2fproto_2faudio_5fcommon_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2faudio_2fproto_2faudio_5fcommon_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_modules_2faudio_2fproto_2faudio_5fcommon_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n&modules/audio/proto/audio_common.proto"
  "\022\014apollo.audio*K\n\014MovingResult\022\013\n\007UNKNOW"
  "N\020\000\022\017\n\013APPROACHING\020\001\022\r\n\tDEPARTING\020\002\022\016\n\nS"
  "TATIONARY\020\003*G\n\tAudioType\022\020\n\014UNKNOWN_TYPE"
  "\020\000\022\n\n\006POLICE\020\001\022\r\n\tAMBULANCE\020\002\022\r\n\tFIRETRU"
  "CK\020\003*Q\n\016AudioDirection\022\025\n\021UNKNOWN_DIRECT"
  "ION\020\000\022\t\n\005FRONT\020\001\022\010\n\004LEFT\020\002\022\010\n\004BACK\020\003\022\t\n\005"
  "RIGHT\020\004"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2faudio_2fproto_2faudio_5fcommon_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_modules_2faudio_2fproto_2faudio_5fcommon_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2faudio_2fproto_2faudio_5fcommon_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2faudio_2fproto_2faudio_5fcommon_2eproto = {
  false, false, descriptor_table_protodef_modules_2faudio_2fproto_2faudio_5fcommon_2eproto, "modules/audio/proto/audio_common.proto", 287,
  &descriptor_table_modules_2faudio_2fproto_2faudio_5fcommon_2eproto_once, descriptor_table_modules_2faudio_2fproto_2faudio_5fcommon_2eproto_sccs, descriptor_table_modules_2faudio_2fproto_2faudio_5fcommon_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_modules_2faudio_2fproto_2faudio_5fcommon_2eproto::offsets,
  file_level_metadata_modules_2faudio_2fproto_2faudio_5fcommon_2eproto, 0, file_level_enum_descriptors_modules_2faudio_2fproto_2faudio_5fcommon_2eproto, file_level_service_descriptors_modules_2faudio_2fproto_2faudio_5fcommon_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_modules_2faudio_2fproto_2faudio_5fcommon_2eproto = (static_cast<void>(::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_modules_2faudio_2fproto_2faudio_5fcommon_2eproto)), true);
namespace apollo {
namespace audio {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* MovingResult_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_modules_2faudio_2fproto_2faudio_5fcommon_2eproto);
  return file_level_enum_descriptors_modules_2faudio_2fproto_2faudio_5fcommon_2eproto[0];
}
bool MovingResult_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
      return true;
    default:
      return false;
  }
}

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* AudioType_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_modules_2faudio_2fproto_2faudio_5fcommon_2eproto);
  return file_level_enum_descriptors_modules_2faudio_2fproto_2faudio_5fcommon_2eproto[1];
}
bool AudioType_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
      return true;
    default:
      return false;
  }
}

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* AudioDirection_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_modules_2faudio_2fproto_2faudio_5fcommon_2eproto);
  return file_level_enum_descriptors_modules_2faudio_2fproto_2faudio_5fcommon_2eproto[2];
}
bool AudioDirection_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
      return true;
    default:
      return false;
  }
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace audio
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
