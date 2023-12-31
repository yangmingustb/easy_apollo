// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/drivers/lidar/proto/lidar_parameter.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fdrivers_2flidar_2fproto_2flidar_5fparameter_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fdrivers_2flidar_2fproto_2flidar_5fparameter_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3014000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3014000 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2fdrivers_2flidar_2fproto_2flidar_5fparameter_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fdrivers_2flidar_2fproto_2flidar_5fparameter_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fdrivers_2flidar_2fproto_2flidar_5fparameter_2eproto;
namespace apollo {
namespace drivers {
namespace lidar {
class LidarParameter;
class LidarParameterDefaultTypeInternal;
extern LidarParameterDefaultTypeInternal _LidarParameter_default_instance_;
}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::drivers::lidar::LidarParameter* Arena::CreateMaybeMessage<::apollo::drivers::lidar::LidarParameter>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace drivers {
namespace lidar {

enum LidarParameter_LidarBrand : int {
  LidarParameter_LidarBrand_VELODYNE = 0,
  LidarParameter_LidarBrand_HESAI = 1,
  LidarParameter_LidarBrand_ROBOSENSE = 2
};
bool LidarParameter_LidarBrand_IsValid(int value);
constexpr LidarParameter_LidarBrand LidarParameter_LidarBrand_LidarBrand_MIN = LidarParameter_LidarBrand_VELODYNE;
constexpr LidarParameter_LidarBrand LidarParameter_LidarBrand_LidarBrand_MAX = LidarParameter_LidarBrand_ROBOSENSE;
constexpr int LidarParameter_LidarBrand_LidarBrand_ARRAYSIZE = LidarParameter_LidarBrand_LidarBrand_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* LidarParameter_LidarBrand_descriptor();
template<typename T>
inline const std::string& LidarParameter_LidarBrand_Name(T enum_t_value) {
  static_assert(::std::is_same<T, LidarParameter_LidarBrand>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function LidarParameter_LidarBrand_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    LidarParameter_LidarBrand_descriptor(), enum_t_value);
}
inline bool LidarParameter_LidarBrand_Parse(
    ::PROTOBUF_NAMESPACE_ID::ConstStringParam name, LidarParameter_LidarBrand* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<LidarParameter_LidarBrand>(
    LidarParameter_LidarBrand_descriptor(), name, value);
}
enum LidarParameter_LidarChannelId : int {
  LidarParameter_LidarChannelId_CHANNEL_ID_ZERO = 0,
  LidarParameter_LidarChannelId_CHANNEL_ID_ONE = 1,
  LidarParameter_LidarChannelId_CHANNEL_ID_TWO = 2,
  LidarParameter_LidarChannelId_CHANNEL_ID_THREE = 3
};
bool LidarParameter_LidarChannelId_IsValid(int value);
constexpr LidarParameter_LidarChannelId LidarParameter_LidarChannelId_LidarChannelId_MIN = LidarParameter_LidarChannelId_CHANNEL_ID_ZERO;
constexpr LidarParameter_LidarChannelId LidarParameter_LidarChannelId_LidarChannelId_MAX = LidarParameter_LidarChannelId_CHANNEL_ID_THREE;
constexpr int LidarParameter_LidarChannelId_LidarChannelId_ARRAYSIZE = LidarParameter_LidarChannelId_LidarChannelId_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* LidarParameter_LidarChannelId_descriptor();
template<typename T>
inline const std::string& LidarParameter_LidarChannelId_Name(T enum_t_value) {
  static_assert(::std::is_same<T, LidarParameter_LidarChannelId>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function LidarParameter_LidarChannelId_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    LidarParameter_LidarChannelId_descriptor(), enum_t_value);
}
inline bool LidarParameter_LidarChannelId_Parse(
    ::PROTOBUF_NAMESPACE_ID::ConstStringParam name, LidarParameter_LidarChannelId* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<LidarParameter_LidarChannelId>(
    LidarParameter_LidarChannelId_descriptor(), name, value);
}
// ===================================================================

class LidarParameter PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.drivers.lidar.LidarParameter) */ {
 public:
  inline LidarParameter() : LidarParameter(nullptr) {}
  virtual ~LidarParameter();

  LidarParameter(const LidarParameter& from);
  LidarParameter(LidarParameter&& from) noexcept
    : LidarParameter() {
    *this = ::std::move(from);
  }

  inline LidarParameter& operator=(const LidarParameter& from) {
    CopyFrom(from);
    return *this;
  }
  inline LidarParameter& operator=(LidarParameter&& from) noexcept {
    if (GetArena() == from.GetArena()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance);
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const LidarParameter& default_instance();

  static inline const LidarParameter* internal_default_instance() {
    return reinterpret_cast<const LidarParameter*>(
               &_LidarParameter_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(LidarParameter& a, LidarParameter& b) {
    a.Swap(&b);
  }
  inline void Swap(LidarParameter* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(LidarParameter* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LidarParameter* New() const final {
    return CreateMaybeMessage<LidarParameter>(nullptr);
  }

  LidarParameter* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LidarParameter>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LidarParameter& from);
  void MergeFrom(const LidarParameter& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(LidarParameter* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.drivers.lidar.LidarParameter";
  }
  protected:
  explicit LidarParameter(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fdrivers_2flidar_2fproto_2flidar_5fparameter_2eproto);
    return ::descriptor_table_modules_2fdrivers_2flidar_2fproto_2flidar_5fparameter_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  typedef LidarParameter_LidarBrand LidarBrand;
  static constexpr LidarBrand VELODYNE =
    LidarParameter_LidarBrand_VELODYNE;
  static constexpr LidarBrand HESAI =
    LidarParameter_LidarBrand_HESAI;
  static constexpr LidarBrand ROBOSENSE =
    LidarParameter_LidarBrand_ROBOSENSE;
  static inline bool LidarBrand_IsValid(int value) {
    return LidarParameter_LidarBrand_IsValid(value);
  }
  static constexpr LidarBrand LidarBrand_MIN =
    LidarParameter_LidarBrand_LidarBrand_MIN;
  static constexpr LidarBrand LidarBrand_MAX =
    LidarParameter_LidarBrand_LidarBrand_MAX;
  static constexpr int LidarBrand_ARRAYSIZE =
    LidarParameter_LidarBrand_LidarBrand_ARRAYSIZE;
  static inline const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor*
  LidarBrand_descriptor() {
    return LidarParameter_LidarBrand_descriptor();
  }
  template<typename T>
  static inline const std::string& LidarBrand_Name(T enum_t_value) {
    static_assert(::std::is_same<T, LidarBrand>::value ||
      ::std::is_integral<T>::value,
      "Incorrect type passed to function LidarBrand_Name.");
    return LidarParameter_LidarBrand_Name(enum_t_value);
  }
  static inline bool LidarBrand_Parse(::PROTOBUF_NAMESPACE_ID::ConstStringParam name,
      LidarBrand* value) {
    return LidarParameter_LidarBrand_Parse(name, value);
  }

  typedef LidarParameter_LidarChannelId LidarChannelId;
  static constexpr LidarChannelId CHANNEL_ID_ZERO =
    LidarParameter_LidarChannelId_CHANNEL_ID_ZERO;
  static constexpr LidarChannelId CHANNEL_ID_ONE =
    LidarParameter_LidarChannelId_CHANNEL_ID_ONE;
  static constexpr LidarChannelId CHANNEL_ID_TWO =
    LidarParameter_LidarChannelId_CHANNEL_ID_TWO;
  static constexpr LidarChannelId CHANNEL_ID_THREE =
    LidarParameter_LidarChannelId_CHANNEL_ID_THREE;
  static inline bool LidarChannelId_IsValid(int value) {
    return LidarParameter_LidarChannelId_IsValid(value);
  }
  static constexpr LidarChannelId LidarChannelId_MIN =
    LidarParameter_LidarChannelId_LidarChannelId_MIN;
  static constexpr LidarChannelId LidarChannelId_MAX =
    LidarParameter_LidarChannelId_LidarChannelId_MAX;
  static constexpr int LidarChannelId_ARRAYSIZE =
    LidarParameter_LidarChannelId_LidarChannelId_ARRAYSIZE;
  static inline const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor*
  LidarChannelId_descriptor() {
    return LidarParameter_LidarChannelId_descriptor();
  }
  template<typename T>
  static inline const std::string& LidarChannelId_Name(T enum_t_value) {
    static_assert(::std::is_same<T, LidarChannelId>::value ||
      ::std::is_integral<T>::value,
      "Incorrect type passed to function LidarChannelId_Name.");
    return LidarParameter_LidarChannelId_Name(enum_t_value);
  }
  static inline bool LidarChannelId_Parse(::PROTOBUF_NAMESPACE_ID::ConstStringParam name,
      LidarChannelId* value) {
    return LidarParameter_LidarChannelId_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  enum : int {
    kBrandFieldNumber = 1,
  };
  // required .apollo.drivers.lidar.LidarParameter.LidarBrand brand = 1;
  bool has_brand() const;
  private:
  bool _internal_has_brand() const;
  public:
  void clear_brand();
  ::apollo::drivers::lidar::LidarParameter_LidarBrand brand() const;
  void set_brand(::apollo::drivers::lidar::LidarParameter_LidarBrand value);
  private:
  ::apollo::drivers::lidar::LidarParameter_LidarBrand _internal_brand() const;
  void _internal_set_brand(::apollo::drivers::lidar::LidarParameter_LidarBrand value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.drivers.lidar.LidarParameter)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  int brand_;
  friend struct ::TableStruct_modules_2fdrivers_2flidar_2fproto_2flidar_5fparameter_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// LidarParameter

// required .apollo.drivers.lidar.LidarParameter.LidarBrand brand = 1;
inline bool LidarParameter::_internal_has_brand() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool LidarParameter::has_brand() const {
  return _internal_has_brand();
}
inline void LidarParameter::clear_brand() {
  brand_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::apollo::drivers::lidar::LidarParameter_LidarBrand LidarParameter::_internal_brand() const {
  return static_cast< ::apollo::drivers::lidar::LidarParameter_LidarBrand >(brand_);
}
inline ::apollo::drivers::lidar::LidarParameter_LidarBrand LidarParameter::brand() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.lidar.LidarParameter.brand)
  return _internal_brand();
}
inline void LidarParameter::_internal_set_brand(::apollo::drivers::lidar::LidarParameter_LidarBrand value) {
  assert(::apollo::drivers::lidar::LidarParameter_LidarBrand_IsValid(value));
  _has_bits_[0] |= 0x00000001u;
  brand_ = value;
}
inline void LidarParameter::set_brand(::apollo::drivers::lidar::LidarParameter_LidarBrand value) {
  _internal_set_brand(value);
  // @@protoc_insertion_point(field_set:apollo.drivers.lidar.LidarParameter.brand)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::apollo::drivers::lidar::LidarParameter_LidarBrand> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::drivers::lidar::LidarParameter_LidarBrand>() {
  return ::apollo::drivers::lidar::LidarParameter_LidarBrand_descriptor();
}
template <> struct is_proto_enum< ::apollo::drivers::lidar::LidarParameter_LidarChannelId> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::drivers::lidar::LidarParameter_LidarChannelId>() {
  return ::apollo::drivers::lidar::LidarParameter_LidarChannelId_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fdrivers_2flidar_2fproto_2flidar_5fparameter_2eproto
