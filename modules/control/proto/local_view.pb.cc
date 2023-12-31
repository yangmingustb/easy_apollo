// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/control/proto/local_view.proto

#include "modules/control/proto/local_view.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_modules_2fcanbus_2fproto_2fchassis_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<8> scc_info_Chassis_modules_2fcanbus_2fproto_2fchassis_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_modules_2fcommon_2fproto_2fheader_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_Header_modules_2fcommon_2fproto_2fheader_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_modules_2fcontrol_2fproto_2fpad_5fmsg_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_PadMessage_modules_2fcontrol_2fproto_2fpad_5fmsg_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_modules_2flocalization_2fproto_2flocalization_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<6> scc_info_LocalizationEstimate_modules_2flocalization_2fproto_2flocalization_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_modules_2fplanning_2fproto_2fplanning_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<11> scc_info_ADCTrajectory_modules_2fplanning_2fproto_2fplanning_2eproto;
namespace apollo {
namespace control {
class LocalViewDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<LocalView> _instance;
} _LocalView_default_instance_;
}  // namespace control
}  // namespace apollo
static void InitDefaultsscc_info_LocalView_modules_2fcontrol_2fproto_2flocal_5fview_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::control::_LocalView_default_instance_;
    new (ptr) ::apollo::control::LocalView();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<5> scc_info_LocalView_modules_2fcontrol_2fproto_2flocal_5fview_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 5, 0, InitDefaultsscc_info_LocalView_modules_2fcontrol_2fproto_2flocal_5fview_2eproto}, {
      &scc_info_Header_modules_2fcommon_2fproto_2fheader_2eproto.base,
      &scc_info_Chassis_modules_2fcanbus_2fproto_2fchassis_2eproto.base,
      &scc_info_ADCTrajectory_modules_2fplanning_2fproto_2fplanning_2eproto.base,
      &scc_info_LocalizationEstimate_modules_2flocalization_2fproto_2flocalization_2eproto.base,
      &scc_info_PadMessage_modules_2fcontrol_2fproto_2fpad_5fmsg_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fcontrol_2fproto_2flocal_5fview_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fcontrol_2fproto_2flocal_5fview_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fcontrol_2fproto_2flocal_5fview_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fcontrol_2fproto_2flocal_5fview_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::control::LocalView, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::control::LocalView, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::apollo::control::LocalView, header_),
  PROTOBUF_FIELD_OFFSET(::apollo::control::LocalView, chassis_),
  PROTOBUF_FIELD_OFFSET(::apollo::control::LocalView, trajectory_),
  PROTOBUF_FIELD_OFFSET(::apollo::control::LocalView, localization_),
  PROTOBUF_FIELD_OFFSET(::apollo::control::LocalView, pad_msg_),
  0,
  1,
  2,
  3,
  4,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 10, sizeof(::apollo::control::LocalView)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::control::_LocalView_default_instance_),
};

const char descriptor_table_protodef_modules_2fcontrol_2fproto_2flocal_5fview_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n&modules/control/proto/local_view.proto"
  "\022\016apollo.control\032\"modules/canbus/proto/c"
  "hassis.proto\032!modules/common/proto/heade"
  "r.proto\032#modules/control/proto/pad_msg.p"
  "roto\032-modules/localization/proto/localiz"
  "ation.proto\032%modules/planning/proto/plan"
  "ning.proto\"\375\001\n\tLocalView\022%\n\006header\030\001 \001(\013"
  "2\025.apollo.common.Header\022\'\n\007chassis\030\002 \001(\013"
  "2\026.apollo.canbus.Chassis\0222\n\ntrajectory\030\003"
  " \001(\0132\036.apollo.planning.ADCTrajectory\022\?\n\014"
  "localization\030\004 \001(\0132).apollo.localization"
  ".LocalizationEstimate\022+\n\007pad_msg\030\005 \001(\0132\032"
  ".apollo.control.PadMessage"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fcontrol_2fproto_2flocal_5fview_2eproto_deps[5] = {
  &::descriptor_table_modules_2fcanbus_2fproto_2fchassis_2eproto,
  &::descriptor_table_modules_2fcommon_2fproto_2fheader_2eproto,
  &::descriptor_table_modules_2fcontrol_2fproto_2fpad_5fmsg_2eproto,
  &::descriptor_table_modules_2flocalization_2fproto_2flocalization_2eproto,
  &::descriptor_table_modules_2fplanning_2fproto_2fplanning_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_modules_2fcontrol_2fproto_2flocal_5fview_2eproto_sccs[1] = {
  &scc_info_LocalView_modules_2fcontrol_2fproto_2flocal_5fview_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fcontrol_2fproto_2flocal_5fview_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fcontrol_2fproto_2flocal_5fview_2eproto = {
  false, false, descriptor_table_protodef_modules_2fcontrol_2fproto_2flocal_5fview_2eproto, "modules/control/proto/local_view.proto", 506,
  &descriptor_table_modules_2fcontrol_2fproto_2flocal_5fview_2eproto_once, descriptor_table_modules_2fcontrol_2fproto_2flocal_5fview_2eproto_sccs, descriptor_table_modules_2fcontrol_2fproto_2flocal_5fview_2eproto_deps, 1, 5,
  schemas, file_default_instances, TableStruct_modules_2fcontrol_2fproto_2flocal_5fview_2eproto::offsets,
  file_level_metadata_modules_2fcontrol_2fproto_2flocal_5fview_2eproto, 1, file_level_enum_descriptors_modules_2fcontrol_2fproto_2flocal_5fview_2eproto, file_level_service_descriptors_modules_2fcontrol_2fproto_2flocal_5fview_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_modules_2fcontrol_2fproto_2flocal_5fview_2eproto = (static_cast<void>(::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_modules_2fcontrol_2fproto_2flocal_5fview_2eproto)), true);
namespace apollo {
namespace control {

// ===================================================================

class LocalView::_Internal {
 public:
  using HasBits = decltype(std::declval<LocalView>()._has_bits_);
  static const ::apollo::common::Header& header(const LocalView* msg);
  static void set_has_header(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static const ::apollo::canbus::Chassis& chassis(const LocalView* msg);
  static void set_has_chassis(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static const ::apollo::planning::ADCTrajectory& trajectory(const LocalView* msg);
  static void set_has_trajectory(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static const ::apollo::localization::LocalizationEstimate& localization(const LocalView* msg);
  static void set_has_localization(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static const ::apollo::control::PadMessage& pad_msg(const LocalView* msg);
  static void set_has_pad_msg(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
};

const ::apollo::common::Header&
LocalView::_Internal::header(const LocalView* msg) {
  return *msg->header_;
}
const ::apollo::canbus::Chassis&
LocalView::_Internal::chassis(const LocalView* msg) {
  return *msg->chassis_;
}
const ::apollo::planning::ADCTrajectory&
LocalView::_Internal::trajectory(const LocalView* msg) {
  return *msg->trajectory_;
}
const ::apollo::localization::LocalizationEstimate&
LocalView::_Internal::localization(const LocalView* msg) {
  return *msg->localization_;
}
const ::apollo::control::PadMessage&
LocalView::_Internal::pad_msg(const LocalView* msg) {
  return *msg->pad_msg_;
}
void LocalView::clear_header() {
  if (header_ != nullptr) header_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void LocalView::clear_chassis() {
  if (chassis_ != nullptr) chassis_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
void LocalView::clear_trajectory() {
  if (trajectory_ != nullptr) trajectory_->Clear();
  _has_bits_[0] &= ~0x00000004u;
}
void LocalView::clear_localization() {
  if (localization_ != nullptr) localization_->Clear();
  _has_bits_[0] &= ~0x00000008u;
}
void LocalView::clear_pad_msg() {
  if (pad_msg_ != nullptr) pad_msg_->Clear();
  _has_bits_[0] &= ~0x00000010u;
}
LocalView::LocalView(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:apollo.control.LocalView)
}
LocalView::LocalView(const LocalView& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    header_ = new ::apollo::common::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  if (from._internal_has_chassis()) {
    chassis_ = new ::apollo::canbus::Chassis(*from.chassis_);
  } else {
    chassis_ = nullptr;
  }
  if (from._internal_has_trajectory()) {
    trajectory_ = new ::apollo::planning::ADCTrajectory(*from.trajectory_);
  } else {
    trajectory_ = nullptr;
  }
  if (from._internal_has_localization()) {
    localization_ = new ::apollo::localization::LocalizationEstimate(*from.localization_);
  } else {
    localization_ = nullptr;
  }
  if (from._internal_has_pad_msg()) {
    pad_msg_ = new ::apollo::control::PadMessage(*from.pad_msg_);
  } else {
    pad_msg_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:apollo.control.LocalView)
}

void LocalView::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_LocalView_modules_2fcontrol_2fproto_2flocal_5fview_2eproto.base);
  ::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
      reinterpret_cast<char*>(&header_) - reinterpret_cast<char*>(this)),
      0, static_cast<size_t>(reinterpret_cast<char*>(&pad_msg_) -
      reinterpret_cast<char*>(&header_)) + sizeof(pad_msg_));
}

LocalView::~LocalView() {
  // @@protoc_insertion_point(destructor:apollo.control.LocalView)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void LocalView::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
  if (this != internal_default_instance()) delete header_;
  if (this != internal_default_instance()) delete chassis_;
  if (this != internal_default_instance()) delete trajectory_;
  if (this != internal_default_instance()) delete localization_;
  if (this != internal_default_instance()) delete pad_msg_;
}

void LocalView::ArenaDtor(void* object) {
  LocalView* _this = reinterpret_cast< LocalView* >(object);
  (void)_this;
}
void LocalView::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void LocalView::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const LocalView& LocalView::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_LocalView_modules_2fcontrol_2fproto_2flocal_5fview_2eproto.base);
  return *internal_default_instance();
}


void LocalView::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.control.LocalView)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(header_ != nullptr);
      header_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(chassis_ != nullptr);
      chassis_->Clear();
    }
    if (cached_has_bits & 0x00000004u) {
      GOOGLE_DCHECK(trajectory_ != nullptr);
      trajectory_->Clear();
    }
    if (cached_has_bits & 0x00000008u) {
      GOOGLE_DCHECK(localization_ != nullptr);
      localization_->Clear();
    }
    if (cached_has_bits & 0x00000010u) {
      GOOGLE_DCHECK(pad_msg_ != nullptr);
      pad_msg_->Clear();
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* LocalView::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
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
      // optional .apollo.canbus.Chassis chassis = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_chassis(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .apollo.planning.ADCTrajectory trajectory = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          ptr = ctx->ParseMessage(_internal_mutable_trajectory(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .apollo.localization.LocalizationEstimate localization = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 34)) {
          ptr = ctx->ParseMessage(_internal_mutable_localization(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .apollo.control.PadMessage pad_msg = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 42)) {
          ptr = ctx->ParseMessage(_internal_mutable_pad_msg(), ptr);
          CHK_(ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* LocalView::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.control.LocalView)
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

  // optional .apollo.canbus.Chassis chassis = 2;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::chassis(this), target, stream);
  }

  // optional .apollo.planning.ADCTrajectory trajectory = 3;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        3, _Internal::trajectory(this), target, stream);
  }

  // optional .apollo.localization.LocalizationEstimate localization = 4;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        4, _Internal::localization(this), target, stream);
  }

  // optional .apollo.control.PadMessage pad_msg = 5;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        5, _Internal::pad_msg(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.control.LocalView)
  return target;
}

size_t LocalView::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.control.LocalView)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    // optional .apollo.common.Header header = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *header_);
    }

    // optional .apollo.canbus.Chassis chassis = 2;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *chassis_);
    }

    // optional .apollo.planning.ADCTrajectory trajectory = 3;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *trajectory_);
    }

    // optional .apollo.localization.LocalizationEstimate localization = 4;
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *localization_);
    }

    // optional .apollo.control.PadMessage pad_msg = 5;
    if (cached_has_bits & 0x00000010u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *pad_msg_);
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

void LocalView::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.control.LocalView)
  GOOGLE_DCHECK_NE(&from, this);
  const LocalView* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<LocalView>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.control.LocalView)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.control.LocalView)
    MergeFrom(*source);
  }
}

void LocalView::MergeFrom(const LocalView& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.control.LocalView)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_header()->::apollo::common::Header::MergeFrom(from._internal_header());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_chassis()->::apollo::canbus::Chassis::MergeFrom(from._internal_chassis());
    }
    if (cached_has_bits & 0x00000004u) {
      _internal_mutable_trajectory()->::apollo::planning::ADCTrajectory::MergeFrom(from._internal_trajectory());
    }
    if (cached_has_bits & 0x00000008u) {
      _internal_mutable_localization()->::apollo::localization::LocalizationEstimate::MergeFrom(from._internal_localization());
    }
    if (cached_has_bits & 0x00000010u) {
      _internal_mutable_pad_msg()->::apollo::control::PadMessage::MergeFrom(from._internal_pad_msg());
    }
  }
}

void LocalView::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.control.LocalView)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void LocalView::CopyFrom(const LocalView& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.control.LocalView)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool LocalView::IsInitialized() const {
  return true;
}

void LocalView::InternalSwap(LocalView* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(LocalView, pad_msg_)
      + sizeof(LocalView::pad_msg_)
      - PROTOBUF_FIELD_OFFSET(LocalView, header_)>(
          reinterpret_cast<char*>(&header_),
          reinterpret_cast<char*>(&other->header_));
}

::PROTOBUF_NAMESPACE_ID::Metadata LocalView::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace control
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::control::LocalView* Arena::CreateMaybeMessage< ::apollo::control::LocalView >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::control::LocalView >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
