// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/dreamview/proto/camera_update.proto

#include "modules/dreamview/proto/camera_update.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_modules_2fperception_2fproto_2fperception_5fobstacle_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_BBox2D_modules_2fperception_2fproto_2fperception_5fobstacle_2eproto;
namespace apollo {
namespace dreamview {
class CameraUpdateDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<CameraUpdate> _instance;
} _CameraUpdate_default_instance_;
}  // namespace dreamview
}  // namespace apollo
static void InitDefaultsscc_info_CameraUpdate_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::dreamview::_CameraUpdate_default_instance_;
    new (ptr) ::apollo::dreamview::CameraUpdate();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_CameraUpdate_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_CameraUpdate_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto}, {
      &scc_info_BBox2D_modules_2fperception_2fproto_2fperception_5fobstacle_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto[1];
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::dreamview::CameraUpdate, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::dreamview::CameraUpdate, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::apollo::dreamview::CameraUpdate, localization_),
  PROTOBUF_FIELD_OFFSET(::apollo::dreamview::CameraUpdate, localization2camera_tf_),
  PROTOBUF_FIELD_OFFSET(::apollo::dreamview::CameraUpdate, image_),
  PROTOBUF_FIELD_OFFSET(::apollo::dreamview::CameraUpdate, image_aspect_ratio_),
  PROTOBUF_FIELD_OFFSET(::apollo::dreamview::CameraUpdate, bbox2d_),
  PROTOBUF_FIELD_OFFSET(::apollo::dreamview::CameraUpdate, obstacles_id_),
  PROTOBUF_FIELD_OFFSET(::apollo::dreamview::CameraUpdate, obstacles_sub_type_),
  PROTOBUF_FIELD_OFFSET(::apollo::dreamview::CameraUpdate, k_image_scale_),
  ~0u,
  ~0u,
  0,
  1,
  ~0u,
  ~0u,
  ~0u,
  2,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 13, sizeof(::apollo::dreamview::CameraUpdate)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::dreamview::_CameraUpdate_default_instance_),
};

const char descriptor_table_protodef_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n+modules/dreamview/proto/camera_update."
  "proto\022\020apollo.dreamview\0322modules/percept"
  "ion/proto/perception_obstacle.proto\"\352\003\n\014"
  "CameraUpdate\022\024\n\014localization\030\001 \003(\001\022\036\n\026lo"
  "calization2camera_tf\030\002 \003(\001\022\r\n\005image\030\003 \001("
  "\014\022\032\n\022image_aspect_ratio\030\004 \001(\001\022)\n\006bbox2d\030"
  "\005 \003(\0132\031.apollo.perception.BBox2D\022\024\n\014obst"
  "acles_id\030\006 \003(\005\022B\n\022obstacles_sub_type\030\007 \003"
  "(\0162&.apollo.dreamview.CameraUpdate.SubTy"
  "pe\022\025\n\rk_image_scale\030\010 \001(\001\"\334\001\n\007SubType\022\016\n"
  "\nST_UNKNOWN\020\000\022\026\n\022ST_UNKNOWN_MOVABLE\020\001\022\030\n"
  "\024ST_UNKNOWN_UNMOVABLE\020\002\022\n\n\006ST_CAR\020\003\022\n\n\006S"
  "T_VAN\020\004\022\014\n\010ST_TRUCK\020\005\022\n\n\006ST_BUS\020\006\022\016\n\nST_"
  "CYCLIST\020\007\022\023\n\017ST_MOTORCYCLIST\020\010\022\021\n\rST_TRI"
  "CYCLIST\020\t\022\021\n\rST_PEDESTRIAN\020\n\022\022\n\016ST_TRAFF"
  "ICCONE\020\013"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto_deps[1] = {
  &::descriptor_table_modules_2fperception_2fproto_2fperception_5fobstacle_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto_sccs[1] = {
  &scc_info_CameraUpdate_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto = {
  false, false, descriptor_table_protodef_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto, "modules/dreamview/proto/camera_update.proto", 608,
  &descriptor_table_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto_once, descriptor_table_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto_sccs, descriptor_table_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto_deps, 1, 1,
  schemas, file_default_instances, TableStruct_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto::offsets,
  file_level_metadata_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto, 1, file_level_enum_descriptors_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto, file_level_service_descriptors_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto = (static_cast<void>(::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto)), true);
namespace apollo {
namespace dreamview {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* CameraUpdate_SubType_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto);
  return file_level_enum_descriptors_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto[0];
}
bool CameraUpdate_SubType_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
      return true;
    default:
      return false;
  }
}

#if (__cplusplus < 201703) && (!defined(_MSC_VER) || _MSC_VER >= 1900)
constexpr CameraUpdate_SubType CameraUpdate::ST_UNKNOWN;
constexpr CameraUpdate_SubType CameraUpdate::ST_UNKNOWN_MOVABLE;
constexpr CameraUpdate_SubType CameraUpdate::ST_UNKNOWN_UNMOVABLE;
constexpr CameraUpdate_SubType CameraUpdate::ST_CAR;
constexpr CameraUpdate_SubType CameraUpdate::ST_VAN;
constexpr CameraUpdate_SubType CameraUpdate::ST_TRUCK;
constexpr CameraUpdate_SubType CameraUpdate::ST_BUS;
constexpr CameraUpdate_SubType CameraUpdate::ST_CYCLIST;
constexpr CameraUpdate_SubType CameraUpdate::ST_MOTORCYCLIST;
constexpr CameraUpdate_SubType CameraUpdate::ST_TRICYCLIST;
constexpr CameraUpdate_SubType CameraUpdate::ST_PEDESTRIAN;
constexpr CameraUpdate_SubType CameraUpdate::ST_TRAFFICCONE;
constexpr CameraUpdate_SubType CameraUpdate::SubType_MIN;
constexpr CameraUpdate_SubType CameraUpdate::SubType_MAX;
constexpr int CameraUpdate::SubType_ARRAYSIZE;
#endif  // (__cplusplus < 201703) && (!defined(_MSC_VER) || _MSC_VER >= 1900)

// ===================================================================

class CameraUpdate::_Internal {
 public:
  using HasBits = decltype(std::declval<CameraUpdate>()._has_bits_);
  static void set_has_image(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_image_aspect_ratio(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_k_image_scale(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

void CameraUpdate::clear_bbox2d() {
  bbox2d_.Clear();
}
CameraUpdate::CameraUpdate(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena),
  localization_(arena),
  localization2camera_tf_(arena),
  bbox2d_(arena),
  obstacles_id_(arena),
  obstacles_sub_type_(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:apollo.dreamview.CameraUpdate)
}
CameraUpdate::CameraUpdate(const CameraUpdate& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_),
      localization_(from.localization_),
      localization2camera_tf_(from.localization2camera_tf_),
      bbox2d_(from.bbox2d_),
      obstacles_id_(from.obstacles_id_),
      obstacles_sub_type_(from.obstacles_sub_type_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  image_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_image()) {
    image_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_image(), 
      GetArena());
  }
  ::memcpy(&image_aspect_ratio_, &from.image_aspect_ratio_,
    static_cast<size_t>(reinterpret_cast<char*>(&k_image_scale_) -
    reinterpret_cast<char*>(&image_aspect_ratio_)) + sizeof(k_image_scale_));
  // @@protoc_insertion_point(copy_constructor:apollo.dreamview.CameraUpdate)
}

void CameraUpdate::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_CameraUpdate_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto.base);
  image_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  ::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
      reinterpret_cast<char*>(&image_aspect_ratio_) - reinterpret_cast<char*>(this)),
      0, static_cast<size_t>(reinterpret_cast<char*>(&k_image_scale_) -
      reinterpret_cast<char*>(&image_aspect_ratio_)) + sizeof(k_image_scale_));
}

CameraUpdate::~CameraUpdate() {
  // @@protoc_insertion_point(destructor:apollo.dreamview.CameraUpdate)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void CameraUpdate::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
  image_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

void CameraUpdate::ArenaDtor(void* object) {
  CameraUpdate* _this = reinterpret_cast< CameraUpdate* >(object);
  (void)_this;
}
void CameraUpdate::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void CameraUpdate::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const CameraUpdate& CameraUpdate::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_CameraUpdate_modules_2fdreamview_2fproto_2fcamera_5fupdate_2eproto.base);
  return *internal_default_instance();
}


void CameraUpdate::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.dreamview.CameraUpdate)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  localization_.Clear();
  localization2camera_tf_.Clear();
  bbox2d_.Clear();
  obstacles_id_.Clear();
  obstacles_sub_type_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    image_.ClearNonDefaultToEmpty();
  }
  if (cached_has_bits & 0x00000006u) {
    ::memset(&image_aspect_ratio_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&k_image_scale_) -
        reinterpret_cast<char*>(&image_aspect_ratio_)) + sizeof(k_image_scale_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* CameraUpdate::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated double localization = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 9)) {
          ptr -= 1;
          do {
            ptr += 1;
            _internal_add_localization(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr));
            ptr += sizeof(double);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<9>(ptr));
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedDoubleParser(_internal_mutable_localization(), ptr, ctx);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated double localization2camera_tf = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 17)) {
          ptr -= 1;
          do {
            ptr += 1;
            _internal_add_localization2camera_tf(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr));
            ptr += sizeof(double);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<17>(ptr));
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedDoubleParser(_internal_mutable_localization2camera_tf(), ptr, ctx);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bytes image = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          auto str = _internal_mutable_image();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional double image_aspect_ratio = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 33)) {
          _Internal::set_has_image_aspect_ratio(&has_bits);
          image_aspect_ratio_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // repeated .apollo.perception.BBox2D bbox2d = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 42)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_bbox2d(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<42>(ptr));
        } else goto handle_unusual;
        continue;
      // repeated int32 obstacles_id = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 48)) {
          ptr -= 1;
          do {
            ptr += 1;
            _internal_add_obstacles_id(::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr));
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<48>(ptr));
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 50) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedInt32Parser(_internal_mutable_obstacles_id(), ptr, ctx);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated .apollo.dreamview.CameraUpdate.SubType obstacles_sub_type = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 56)) {
          ptr -= 1;
          do {
            ptr += 1;
            ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
            CHK_(ptr);
            if (PROTOBUF_PREDICT_TRUE(::apollo::dreamview::CameraUpdate_SubType_IsValid(val))) {
              _internal_add_obstacles_sub_type(static_cast<::apollo::dreamview::CameraUpdate_SubType>(val));
            } else {
              ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(7, val, mutable_unknown_fields());
            }
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<56>(ptr));
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 58) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedEnumParser<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(_internal_mutable_obstacles_sub_type(), ptr, ctx, ::apollo::dreamview::CameraUpdate_SubType_IsValid, &_internal_metadata_, 7);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional double k_image_scale = 8;
      case 8:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 65)) {
          _Internal::set_has_k_image_scale(&has_bits);
          k_image_scale_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
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

::PROTOBUF_NAMESPACE_ID::uint8* CameraUpdate::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.dreamview.CameraUpdate)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated double localization = 1;
  for (int i = 0, n = this->_internal_localization_size(); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(1, this->_internal_localization(i), target);
  }

  // repeated double localization2camera_tf = 2;
  for (int i = 0, n = this->_internal_localization2camera_tf_size(); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(2, this->_internal_localization2camera_tf(i), target);
  }

  cached_has_bits = _has_bits_[0];
  // optional bytes image = 3;
  if (cached_has_bits & 0x00000001u) {
    target = stream->WriteBytesMaybeAliased(
        3, this->_internal_image(), target);
  }

  // optional double image_aspect_ratio = 4;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(4, this->_internal_image_aspect_ratio(), target);
  }

  // repeated .apollo.perception.BBox2D bbox2d = 5;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_bbox2d_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(5, this->_internal_bbox2d(i), target, stream);
  }

  // repeated int32 obstacles_id = 6;
  for (int i = 0, n = this->_internal_obstacles_id_size(); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(6, this->_internal_obstacles_id(i), target);
  }

  // repeated .apollo.dreamview.CameraUpdate.SubType obstacles_sub_type = 7;
  for (int i = 0, n = this->_internal_obstacles_sub_type_size(); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
        7, this->_internal_obstacles_sub_type(i), target);
  }

  // optional double k_image_scale = 8;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(8, this->_internal_k_image_scale(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.dreamview.CameraUpdate)
  return target;
}

size_t CameraUpdate::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.dreamview.CameraUpdate)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated double localization = 1;
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_localization_size());
    size_t data_size = 8UL * count;
    total_size += 1 *
                  ::PROTOBUF_NAMESPACE_ID::internal::FromIntSize(this->_internal_localization_size());
    total_size += data_size;
  }

  // repeated double localization2camera_tf = 2;
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_localization2camera_tf_size());
    size_t data_size = 8UL * count;
    total_size += 1 *
                  ::PROTOBUF_NAMESPACE_ID::internal::FromIntSize(this->_internal_localization2camera_tf_size());
    total_size += data_size;
  }

  // repeated .apollo.perception.BBox2D bbox2d = 5;
  total_size += 1UL * this->_internal_bbox2d_size();
  for (const auto& msg : this->bbox2d_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // repeated int32 obstacles_id = 6;
  {
    size_t data_size = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      Int32Size(this->obstacles_id_);
    total_size += 1 *
                  ::PROTOBUF_NAMESPACE_ID::internal::FromIntSize(this->_internal_obstacles_id_size());
    total_size += data_size;
  }

  // repeated .apollo.dreamview.CameraUpdate.SubType obstacles_sub_type = 7;
  {
    size_t data_size = 0;
    unsigned int count = static_cast<unsigned int>(this->_internal_obstacles_sub_type_size());for (unsigned int i = 0; i < count; i++) {
      data_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(
        this->_internal_obstacles_sub_type(static_cast<int>(i)));
    }
    total_size += (1UL * count) + data_size;
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // optional bytes image = 3;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::BytesSize(
          this->_internal_image());
    }

    // optional double image_aspect_ratio = 4;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 + 8;
    }

    // optional double k_image_scale = 8;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 + 8;
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

void CameraUpdate::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.dreamview.CameraUpdate)
  GOOGLE_DCHECK_NE(&from, this);
  const CameraUpdate* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<CameraUpdate>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.dreamview.CameraUpdate)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.dreamview.CameraUpdate)
    MergeFrom(*source);
  }
}

void CameraUpdate::MergeFrom(const CameraUpdate& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.dreamview.CameraUpdate)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  localization_.MergeFrom(from.localization_);
  localization2camera_tf_.MergeFrom(from.localization2camera_tf_);
  bbox2d_.MergeFrom(from.bbox2d_);
  obstacles_id_.MergeFrom(from.obstacles_id_);
  obstacles_sub_type_.MergeFrom(from.obstacles_sub_type_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_set_image(from._internal_image());
    }
    if (cached_has_bits & 0x00000002u) {
      image_aspect_ratio_ = from.image_aspect_ratio_;
    }
    if (cached_has_bits & 0x00000004u) {
      k_image_scale_ = from.k_image_scale_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void CameraUpdate::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.dreamview.CameraUpdate)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void CameraUpdate::CopyFrom(const CameraUpdate& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.dreamview.CameraUpdate)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CameraUpdate::IsInitialized() const {
  return true;
}

void CameraUpdate::InternalSwap(CameraUpdate* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  localization_.InternalSwap(&other->localization_);
  localization2camera_tf_.InternalSwap(&other->localization2camera_tf_);
  bbox2d_.InternalSwap(&other->bbox2d_);
  obstacles_id_.InternalSwap(&other->obstacles_id_);
  obstacles_sub_type_.InternalSwap(&other->obstacles_sub_type_);
  image_.Swap(&other->image_, &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(CameraUpdate, k_image_scale_)
      + sizeof(CameraUpdate::k_image_scale_)
      - PROTOBUF_FIELD_OFFSET(CameraUpdate, image_aspect_ratio_)>(
          reinterpret_cast<char*>(&image_aspect_ratio_),
          reinterpret_cast<char*>(&other->image_aspect_ratio_));
}

::PROTOBUF_NAMESPACE_ID::Metadata CameraUpdate::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace dreamview
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::dreamview::CameraUpdate* Arena::CreateMaybeMessage< ::apollo::dreamview::CameraUpdate >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::dreamview::CameraUpdate >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
