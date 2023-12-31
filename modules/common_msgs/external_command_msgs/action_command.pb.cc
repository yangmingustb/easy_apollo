// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/common_msgs/external_command_msgs/action_command.proto

#include "modules/common_msgs/external_command_msgs/action_command.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_modules_2fcommon_2fproto_2fheader_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_Header_modules_2fcommon_2fproto_2fheader_2eproto;
namespace apollo {
namespace external_command {
class ActionCommandDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ActionCommand> _instance;
} _ActionCommand_default_instance_;
}  // namespace external_command
}  // namespace apollo
static void InitDefaultsscc_info_ActionCommand_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::external_command::_ActionCommand_default_instance_;
    new (ptr) ::apollo::external_command::ActionCommand();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_ActionCommand_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_ActionCommand_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto}, {
      &scc_info_Header_modules_2fcommon_2fproto_2fheader_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto[1];
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::external_command::ActionCommand, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::external_command::ActionCommand, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::apollo::external_command::ActionCommand, header_),
  PROTOBUF_FIELD_OFFSET(::apollo::external_command::ActionCommand, command_id_),
  PROTOBUF_FIELD_OFFSET(::apollo::external_command::ActionCommand, command_),
  0,
  2,
  1,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::apollo::external_command::ActionCommand)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::external_command::_ActionCommand_default_instance_),
};

const char descriptor_table_protodef_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n>modules/common_msgs/external_command_m"
  "sgs/action_command.proto\022\027apollo.externa"
  "l_command\032!modules/common/proto/header.p"
  "roto\"\213\001\n\rActionCommand\022%\n\006header\030\001 \001(\0132\025"
  ".apollo.common.Header\022\026\n\ncommand_id\030\002 \001("
  "\003:\002-1\022;\n\007command\030\003 \002(\0162*.apollo.external"
  "_command.ActionCommandType*\261\001\n\021ActionCom"
  "mandType\022\n\n\006FOLLOW\020\001\022\017\n\013CHANGE_LEFT\020\002\022\020\n"
  "\014CHANGE_RIGHT\020\003\022\r\n\tPULL_OVER\020\004\022\010\n\004STOP\020\005"
  "\022\t\n\005START\020\006\022\022\n\016CLEAR_PLANNING\020\007\022\024\n\020SWITC"
  "H_TO_MANUAL\0202\022\022\n\016SWITCH_TO_AUTO\0203\022\013\n\007VIN"
  "_REQ\0204"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto_deps[1] = {
  &::descriptor_table_modules_2fcommon_2fproto_2fheader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto_sccs[1] = {
  &scc_info_ActionCommand_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto = {
  false, false, descriptor_table_protodef_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto, "modules/common_msgs/external_command_msgs/action_command.proto", 446,
  &descriptor_table_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto_once, descriptor_table_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto_sccs, descriptor_table_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto_deps, 1, 1,
  schemas, file_default_instances, TableStruct_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto::offsets,
  file_level_metadata_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto, 1, file_level_enum_descriptors_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto, file_level_service_descriptors_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto = (static_cast<void>(::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto)), true);
namespace apollo {
namespace external_command {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* ActionCommandType_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto);
  return file_level_enum_descriptors_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto[0];
}
bool ActionCommandType_IsValid(int value) {
  switch (value) {
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 50:
    case 51:
    case 52:
      return true;
    default:
      return false;
  }
}


// ===================================================================

class ActionCommand::_Internal {
 public:
  using HasBits = decltype(std::declval<ActionCommand>()._has_bits_);
  static const ::apollo::common::Header& header(const ActionCommand* msg);
  static void set_has_header(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_command_id(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_command(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static bool MissingRequiredFields(const HasBits& has_bits) {
    return ((has_bits[0] & 0x00000002) ^ 0x00000002) != 0;
  }
};

const ::apollo::common::Header&
ActionCommand::_Internal::header(const ActionCommand* msg) {
  return *msg->header_;
}
void ActionCommand::clear_header() {
  if (header_ != nullptr) header_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
ActionCommand::ActionCommand(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:apollo.external_command.ActionCommand)
}
ActionCommand::ActionCommand(const ActionCommand& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    header_ = new ::apollo::common::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  ::memcpy(&command_, &from.command_,
    static_cast<size_t>(reinterpret_cast<char*>(&command_id_) -
    reinterpret_cast<char*>(&command_)) + sizeof(command_id_));
  // @@protoc_insertion_point(copy_constructor:apollo.external_command.ActionCommand)
}

void ActionCommand::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_ActionCommand_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto.base);
  header_ = nullptr;
  command_ = 1;
  command_id_ = PROTOBUF_LONGLONG(-1);
}

ActionCommand::~ActionCommand() {
  // @@protoc_insertion_point(destructor:apollo.external_command.ActionCommand)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void ActionCommand::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
  if (this != internal_default_instance()) delete header_;
}

void ActionCommand::ArenaDtor(void* object) {
  ActionCommand* _this = reinterpret_cast< ActionCommand* >(object);
  (void)_this;
}
void ActionCommand::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void ActionCommand::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ActionCommand& ActionCommand::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ActionCommand_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2faction_5fcommand_2eproto.base);
  return *internal_default_instance();
}


void ActionCommand::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.external_command.ActionCommand)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(header_ != nullptr);
      header_->Clear();
    }
    command_ = 1;
    command_id_ = PROTOBUF_LONGLONG(-1);
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* ActionCommand::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional .apollo.common.Header header = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_header(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional int64 command_id = 2 [default = -1];
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          _Internal::set_has_command_id(&has_bits);
          command_id_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // required .apollo.external_command.ActionCommandType command = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 24)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::apollo::external_command::ActionCommandType_IsValid(val))) {
            _internal_set_command(static_cast<::apollo::external_command::ActionCommandType>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(3, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* ActionCommand::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.external_command.ActionCommand)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.Header header = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::header(this), target, stream);
  }

  // optional int64 command_id = 2 [default = -1];
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt64ToArray(2, this->_internal_command_id(), target);
  }

  // required .apollo.external_command.ActionCommandType command = 3;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      3, this->_internal_command(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.external_command.ActionCommand)
  return target;
}

size_t ActionCommand::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.external_command.ActionCommand)
  size_t total_size = 0;

  // required .apollo.external_command.ActionCommandType command = 3;
  if (_internal_has_command()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_command());
  }
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // optional .apollo.common.Header header = 1;
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *header_);
  }

  // optional int64 command_id = 2 [default = -1];
  if (cached_has_bits & 0x00000004u) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int64Size(
        this->_internal_command_id());
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ActionCommand::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.external_command.ActionCommand)
  GOOGLE_DCHECK_NE(&from, this);
  const ActionCommand* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ActionCommand>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.external_command.ActionCommand)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.external_command.ActionCommand)
    MergeFrom(*source);
  }
}

void ActionCommand::MergeFrom(const ActionCommand& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.external_command.ActionCommand)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_header()->::apollo::common::Header::MergeFrom(from._internal_header());
    }
    if (cached_has_bits & 0x00000002u) {
      command_ = from.command_;
    }
    if (cached_has_bits & 0x00000004u) {
      command_id_ = from.command_id_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void ActionCommand::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.external_command.ActionCommand)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ActionCommand::CopyFrom(const ActionCommand& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.external_command.ActionCommand)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ActionCommand::IsInitialized() const {
  if (_Internal::MissingRequiredFields(_has_bits_)) return false;
  return true;
}

void ActionCommand::InternalSwap(ActionCommand* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(header_, other->header_);
  swap(command_, other->command_);
  swap(command_id_, other->command_id_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ActionCommand::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace external_command
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::external_command::ActionCommand* Arena::CreateMaybeMessage< ::apollo::external_command::ActionCommand >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::external_command::ActionCommand >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
