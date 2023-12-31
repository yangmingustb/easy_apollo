// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/proto/perception_lane.proto

#include "modules/perception/proto/perception_lane.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_modules_2fperception_2fproto_2fperception_5fcamera_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_CameraCalibrator_modules_2fperception_2fproto_2fperception_5fcamera_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_modules_2fperception_2fproto_2fperception_5fcamera_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<4> scc_info_CameraLaneLine_modules_2fperception_2fproto_2fperception_5fcamera_2eproto;
namespace apollo {
namespace perception {
class PerceptionLanesDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<PerceptionLanes> _instance;
} _PerceptionLanes_default_instance_;
}  // namespace perception
}  // namespace apollo
static void InitDefaultsscc_info_PerceptionLanes_modules_2fperception_2fproto_2fperception_5flane_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::perception::_PerceptionLanes_default_instance_;
    new (ptr) ::apollo::perception::PerceptionLanes();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<3> scc_info_PerceptionLanes_modules_2fperception_2fproto_2fperception_5flane_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 3, 0, InitDefaultsscc_info_PerceptionLanes_modules_2fperception_2fproto_2fperception_5flane_2eproto}, {
      &scc_info_Header_modules_2fcommon_2fproto_2fheader_2eproto.base,
      &scc_info_CameraCalibrator_modules_2fperception_2fproto_2fperception_5fcamera_2eproto.base,
      &scc_info_CameraLaneLine_modules_2fperception_2fproto_2fperception_5fcamera_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fperception_2fproto_2fperception_5flane_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fperception_2fproto_2fperception_5flane_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fperception_2fproto_2fperception_5flane_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fperception_2fproto_2fperception_5flane_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::perception::PerceptionLanes, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::PerceptionLanes, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::apollo::perception::PerceptionLanes, header_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::PerceptionLanes, source_topic_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::PerceptionLanes, error_code_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::PerceptionLanes, camera_calibrator_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::PerceptionLanes, camera_laneline_),
  1,
  0,
  3,
  2,
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 10, sizeof(::apollo::perception::PerceptionLanes)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::perception::_PerceptionLanes_default_instance_),
};

const char descriptor_table_protodef_modules_2fperception_2fproto_2fperception_5flane_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n.modules/perception/proto/perception_la"
  "ne.proto\022\021apollo.perception\032!modules/com"
  "mon/proto/header.proto\0320modules/percepti"
  "on/proto/perception_camera.proto\"\243\002\n\017Per"
  "ceptionLanes\022%\n\006header\030\001 \001(\0132\025.apollo.co"
  "mmon.Header\022\024\n\014source_topic\030\002 \001(\t\022I\n\nerr"
  "or_code\030\003 \001(\0162).apollo.perception.camera"
  ".CameraErrorCode:\nERROR_NONE\022E\n\021camera_c"
  "alibrator\030\004 \001(\0132*.apollo.perception.came"
  "ra.CameraCalibrator\022A\n\017camera_laneline\030\005"
  " \003(\0132(.apollo.perception.camera.CameraLa"
  "neLine"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fperception_2fproto_2fperception_5flane_2eproto_deps[2] = {
  &::descriptor_table_modules_2fcommon_2fproto_2fheader_2eproto,
  &::descriptor_table_modules_2fperception_2fproto_2fperception_5fcamera_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_modules_2fperception_2fproto_2fperception_5flane_2eproto_sccs[1] = {
  &scc_info_PerceptionLanes_modules_2fperception_2fproto_2fperception_5flane_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fperception_2fproto_2fperception_5flane_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2fproto_2fperception_5flane_2eproto = {
  false, false, descriptor_table_protodef_modules_2fperception_2fproto_2fperception_5flane_2eproto, "modules/perception/proto/perception_lane.proto", 446,
  &descriptor_table_modules_2fperception_2fproto_2fperception_5flane_2eproto_once, descriptor_table_modules_2fperception_2fproto_2fperception_5flane_2eproto_sccs, descriptor_table_modules_2fperception_2fproto_2fperception_5flane_2eproto_deps, 1, 2,
  schemas, file_default_instances, TableStruct_modules_2fperception_2fproto_2fperception_5flane_2eproto::offsets,
  file_level_metadata_modules_2fperception_2fproto_2fperception_5flane_2eproto, 1, file_level_enum_descriptors_modules_2fperception_2fproto_2fperception_5flane_2eproto, file_level_service_descriptors_modules_2fperception_2fproto_2fperception_5flane_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_modules_2fperception_2fproto_2fperception_5flane_2eproto = (static_cast<void>(::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_modules_2fperception_2fproto_2fperception_5flane_2eproto)), true);
namespace apollo {
namespace perception {

// ===================================================================

class PerceptionLanes::_Internal {
 public:
  using HasBits = decltype(std::declval<PerceptionLanes>()._has_bits_);
  static const ::apollo::common::Header& header(const PerceptionLanes* msg);
  static void set_has_header(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_source_topic(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_error_code(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static const ::apollo::perception::camera::CameraCalibrator& camera_calibrator(const PerceptionLanes* msg);
  static void set_has_camera_calibrator(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

const ::apollo::common::Header&
PerceptionLanes::_Internal::header(const PerceptionLanes* msg) {
  return *msg->header_;
}
const ::apollo::perception::camera::CameraCalibrator&
PerceptionLanes::_Internal::camera_calibrator(const PerceptionLanes* msg) {
  return *msg->camera_calibrator_;
}
void PerceptionLanes::clear_header() {
  if (header_ != nullptr) header_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
void PerceptionLanes::clear_camera_calibrator() {
  if (camera_calibrator_ != nullptr) camera_calibrator_->Clear();
  _has_bits_[0] &= ~0x00000004u;
}
void PerceptionLanes::clear_camera_laneline() {
  camera_laneline_.Clear();
}
PerceptionLanes::PerceptionLanes(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena),
  camera_laneline_(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:apollo.perception.PerceptionLanes)
}
PerceptionLanes::PerceptionLanes(const PerceptionLanes& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_),
      camera_laneline_(from.camera_laneline_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  source_topic_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_source_topic()) {
    source_topic_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_source_topic(), 
      GetArena());
  }
  if (from._internal_has_header()) {
    header_ = new ::apollo::common::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  if (from._internal_has_camera_calibrator()) {
    camera_calibrator_ = new ::apollo::perception::camera::CameraCalibrator(*from.camera_calibrator_);
  } else {
    camera_calibrator_ = nullptr;
  }
  error_code_ = from.error_code_;
  // @@protoc_insertion_point(copy_constructor:apollo.perception.PerceptionLanes)
}

void PerceptionLanes::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_PerceptionLanes_modules_2fperception_2fproto_2fperception_5flane_2eproto.base);
  source_topic_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  ::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
      reinterpret_cast<char*>(&header_) - reinterpret_cast<char*>(this)),
      0, static_cast<size_t>(reinterpret_cast<char*>(&error_code_) -
      reinterpret_cast<char*>(&header_)) + sizeof(error_code_));
}

PerceptionLanes::~PerceptionLanes() {
  // @@protoc_insertion_point(destructor:apollo.perception.PerceptionLanes)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void PerceptionLanes::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
  source_topic_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (this != internal_default_instance()) delete header_;
  if (this != internal_default_instance()) delete camera_calibrator_;
}

void PerceptionLanes::ArenaDtor(void* object) {
  PerceptionLanes* _this = reinterpret_cast< PerceptionLanes* >(object);
  (void)_this;
}
void PerceptionLanes::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void PerceptionLanes::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const PerceptionLanes& PerceptionLanes::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_PerceptionLanes_modules_2fperception_2fproto_2fperception_5flane_2eproto.base);
  return *internal_default_instance();
}


void PerceptionLanes::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.PerceptionLanes)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  camera_laneline_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      source_topic_.ClearNonDefaultToEmpty();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(header_ != nullptr);
      header_->Clear();
    }
    if (cached_has_bits & 0x00000004u) {
      GOOGLE_DCHECK(camera_calibrator_ != nullptr);
      camera_calibrator_->Clear();
    }
  }
  error_code_ = 0;
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* PerceptionLanes::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
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
      // optional string source_topic = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          auto str = _internal_mutable_source_topic();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.PerceptionLanes.source_topic");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .apollo.perception.camera.CameraErrorCode error_code = 3 [default = ERROR_NONE];
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 24)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::apollo::perception::camera::CameraErrorCode_IsValid(val))) {
            _internal_set_error_code(static_cast<::apollo::perception::camera::CameraErrorCode>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(3, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional .apollo.perception.camera.CameraCalibrator camera_calibrator = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 34)) {
          ptr = ctx->ParseMessage(_internal_mutable_camera_calibrator(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated .apollo.perception.camera.CameraLaneLine camera_laneline = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 42)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_camera_laneline(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<42>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* PerceptionLanes::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.PerceptionLanes)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.Header header = 1;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::header(this), target, stream);
  }

  // optional string source_topic = 2;
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_source_topic().data(), static_cast<int>(this->_internal_source_topic().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.PerceptionLanes.source_topic");
    target = stream->WriteStringMaybeAliased(
        2, this->_internal_source_topic(), target);
  }

  // optional .apollo.perception.camera.CameraErrorCode error_code = 3 [default = ERROR_NONE];
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      3, this->_internal_error_code(), target);
  }

  // optional .apollo.perception.camera.CameraCalibrator camera_calibrator = 4;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        4, _Internal::camera_calibrator(this), target, stream);
  }

  // repeated .apollo.perception.camera.CameraLaneLine camera_laneline = 5;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_camera_laneline_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(5, this->_internal_camera_laneline(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.PerceptionLanes)
  return target;
}

size_t PerceptionLanes::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.PerceptionLanes)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .apollo.perception.camera.CameraLaneLine camera_laneline = 5;
  total_size += 1UL * this->_internal_camera_laneline_size();
  for (const auto& msg : this->camera_laneline_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    // optional string source_topic = 2;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_source_topic());
    }

    // optional .apollo.common.Header header = 1;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *header_);
    }

    // optional .apollo.perception.camera.CameraCalibrator camera_calibrator = 4;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *camera_calibrator_);
    }

    // optional .apollo.perception.camera.CameraErrorCode error_code = 3 [default = ERROR_NONE];
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_error_code());
    }

  }
  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void PerceptionLanes::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.perception.PerceptionLanes)
  GOOGLE_DCHECK_NE(&from, this);
  const PerceptionLanes* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<PerceptionLanes>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.perception.PerceptionLanes)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.perception.PerceptionLanes)
    MergeFrom(*source);
  }
}

void PerceptionLanes::MergeFrom(const PerceptionLanes& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.PerceptionLanes)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  camera_laneline_.MergeFrom(from.camera_laneline_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_set_source_topic(from._internal_source_topic());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_header()->::apollo::common::Header::MergeFrom(from._internal_header());
    }
    if (cached_has_bits & 0x00000004u) {
      _internal_mutable_camera_calibrator()->::apollo::perception::camera::CameraCalibrator::MergeFrom(from._internal_camera_calibrator());
    }
    if (cached_has_bits & 0x00000008u) {
      error_code_ = from.error_code_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void PerceptionLanes::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.perception.PerceptionLanes)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void PerceptionLanes::CopyFrom(const PerceptionLanes& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.PerceptionLanes)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PerceptionLanes::IsInitialized() const {
  return true;
}

void PerceptionLanes::InternalSwap(PerceptionLanes* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  camera_laneline_.InternalSwap(&other->camera_laneline_);
  source_topic_.Swap(&other->source_topic_, &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(PerceptionLanes, error_code_)
      + sizeof(PerceptionLanes::error_code_)
      - PROTOBUF_FIELD_OFFSET(PerceptionLanes, header_)>(
          reinterpret_cast<char*>(&header_),
          reinterpret_cast<char*>(&other->header_));
}

::PROTOBUF_NAMESPACE_ID::Metadata PerceptionLanes::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::perception::PerceptionLanes* Arena::CreateMaybeMessage< ::apollo::perception::PerceptionLanes >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::perception::PerceptionLanes >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
