// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/v2x/proto/v2x_rsi.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto

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
#include <google/protobuf/unknown_field_set.h>
#include "modules/common/proto/header.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[2]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto;
namespace apollo {
namespace v2x {
class RsiMsg;
class RsiMsgDefaultTypeInternal;
extern RsiMsgDefaultTypeInternal _RsiMsg_default_instance_;
class RsiPoint;
class RsiPointDefaultTypeInternal;
extern RsiPointDefaultTypeInternal _RsiPoint_default_instance_;
}  // namespace v2x
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::v2x::RsiMsg* Arena::CreateMaybeMessage<::apollo::v2x::RsiMsg>(Arena*);
template<> ::apollo::v2x::RsiPoint* Arena::CreateMaybeMessage<::apollo::v2x::RsiPoint>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace v2x {

// ===================================================================

class RsiPoint PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.v2x.RsiPoint) */ {
 public:
  inline RsiPoint() : RsiPoint(nullptr) {}
  virtual ~RsiPoint();

  RsiPoint(const RsiPoint& from);
  RsiPoint(RsiPoint&& from) noexcept
    : RsiPoint() {
    *this = ::std::move(from);
  }

  inline RsiPoint& operator=(const RsiPoint& from) {
    CopyFrom(from);
    return *this;
  }
  inline RsiPoint& operator=(RsiPoint&& from) noexcept {
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
  static const RsiPoint& default_instance();

  static inline const RsiPoint* internal_default_instance() {
    return reinterpret_cast<const RsiPoint*>(
               &_RsiPoint_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(RsiPoint& a, RsiPoint& b) {
    a.Swap(&b);
  }
  inline void Swap(RsiPoint* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(RsiPoint* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline RsiPoint* New() const final {
    return CreateMaybeMessage<RsiPoint>(nullptr);
  }

  RsiPoint* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<RsiPoint>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const RsiPoint& from);
  void MergeFrom(const RsiPoint& from);
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
  void InternalSwap(RsiPoint* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.v2x.RsiPoint";
  }
  protected:
  explicit RsiPoint(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto);
    return ::descriptor_table_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kXFieldNumber = 1,
    kYFieldNumber = 2,
  };
  // optional double x = 1;
  bool has_x() const;
  private:
  bool _internal_has_x() const;
  public:
  void clear_x();
  double x() const;
  void set_x(double value);
  private:
  double _internal_x() const;
  void _internal_set_x(double value);
  public:

  // optional double y = 2;
  bool has_y() const;
  private:
  bool _internal_has_y() const;
  public:
  void clear_y();
  double y() const;
  void set_y(double value);
  private:
  double _internal_y() const;
  void _internal_set_y(double value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.v2x.RsiPoint)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  double x_;
  double y_;
  friend struct ::TableStruct_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto;
};
// -------------------------------------------------------------------

class RsiMsg PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.v2x.RsiMsg) */ {
 public:
  inline RsiMsg() : RsiMsg(nullptr) {}
  virtual ~RsiMsg();

  RsiMsg(const RsiMsg& from);
  RsiMsg(RsiMsg&& from) noexcept
    : RsiMsg() {
    *this = ::std::move(from);
  }

  inline RsiMsg& operator=(const RsiMsg& from) {
    CopyFrom(from);
    return *this;
  }
  inline RsiMsg& operator=(RsiMsg&& from) noexcept {
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
  static const RsiMsg& default_instance();

  static inline const RsiMsg* internal_default_instance() {
    return reinterpret_cast<const RsiMsg*>(
               &_RsiMsg_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(RsiMsg& a, RsiMsg& b) {
    a.Swap(&b);
  }
  inline void Swap(RsiMsg* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(RsiMsg* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline RsiMsg* New() const final {
    return CreateMaybeMessage<RsiMsg>(nullptr);
  }

  RsiMsg* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<RsiMsg>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const RsiMsg& from);
  void MergeFrom(const RsiMsg& from);
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
  void InternalSwap(RsiMsg* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.v2x.RsiMsg";
  }
  protected:
  explicit RsiMsg(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto);
    return ::descriptor_table_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kPointsFieldNumber = 2,
    kDescriptionFieldNumber = 6,
    kHeaderFieldNumber = 1,
    kSpeedFieldNumber = 3,
    kLowSpeedFieldNumber = 4,
    kHighSpeedFieldNumber = 5,
    kRadiusFieldNumber = 8,
    kRsiTypeFieldNumber = 7,
    kPriorityFieldNumber = 9,
  };
  // repeated .apollo.v2x.RsiPoint points = 2;
  int points_size() const;
  private:
  int _internal_points_size() const;
  public:
  void clear_points();
  ::apollo::v2x::RsiPoint* mutable_points(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::v2x::RsiPoint >*
      mutable_points();
  private:
  const ::apollo::v2x::RsiPoint& _internal_points(int index) const;
  ::apollo::v2x::RsiPoint* _internal_add_points();
  public:
  const ::apollo::v2x::RsiPoint& points(int index) const;
  ::apollo::v2x::RsiPoint* add_points();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::v2x::RsiPoint >&
      points() const;

  // optional string description = 6;
  bool has_description() const;
  private:
  bool _internal_has_description() const;
  public:
  void clear_description();
  const std::string& description() const;
  void set_description(const std::string& value);
  void set_description(std::string&& value);
  void set_description(const char* value);
  void set_description(const char* value, size_t size);
  std::string* mutable_description();
  std::string* release_description();
  void set_allocated_description(std::string* description);
  private:
  const std::string& _internal_description() const;
  void _internal_set_description(const std::string& value);
  std::string* _internal_mutable_description();
  public:

  // optional .apollo.common.Header header = 1;
  bool has_header() const;
  private:
  bool _internal_has_header() const;
  public:
  void clear_header();
  const ::apollo::common::Header& header() const;
  ::apollo::common::Header* release_header();
  ::apollo::common::Header* mutable_header();
  void set_allocated_header(::apollo::common::Header* header);
  private:
  const ::apollo::common::Header& _internal_header() const;
  ::apollo::common::Header* _internal_mutable_header();
  public:
  void unsafe_arena_set_allocated_header(
      ::apollo::common::Header* header);
  ::apollo::common::Header* unsafe_arena_release_header();

  // optional double speed = 3;
  bool has_speed() const;
  private:
  bool _internal_has_speed() const;
  public:
  void clear_speed();
  double speed() const;
  void set_speed(double value);
  private:
  double _internal_speed() const;
  void _internal_set_speed(double value);
  public:

  // optional double low_speed = 4;
  bool has_low_speed() const;
  private:
  bool _internal_has_low_speed() const;
  public:
  void clear_low_speed();
  double low_speed() const;
  void set_low_speed(double value);
  private:
  double _internal_low_speed() const;
  void _internal_set_low_speed(double value);
  public:

  // optional double high_speed = 5;
  bool has_high_speed() const;
  private:
  bool _internal_has_high_speed() const;
  public:
  void clear_high_speed();
  double high_speed() const;
  void set_high_speed(double value);
  private:
  double _internal_high_speed() const;
  void _internal_set_high_speed(double value);
  public:

  // optional double radius = 8;
  bool has_radius() const;
  private:
  bool _internal_has_radius() const;
  public:
  void clear_radius();
  double radius() const;
  void set_radius(double value);
  private:
  double _internal_radius() const;
  void _internal_set_radius(double value);
  public:

  // optional int32 rsi_type = 7;
  bool has_rsi_type() const;
  private:
  bool _internal_has_rsi_type() const;
  public:
  void clear_rsi_type();
  ::PROTOBUF_NAMESPACE_ID::int32 rsi_type() const;
  void set_rsi_type(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_rsi_type() const;
  void _internal_set_rsi_type(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // optional int32 priority = 9;
  bool has_priority() const;
  private:
  bool _internal_has_priority() const;
  public:
  void clear_priority();
  ::PROTOBUF_NAMESPACE_ID::int32 priority() const;
  void set_priority(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_priority() const;
  void _internal_set_priority(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.v2x.RsiMsg)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::v2x::RsiPoint > points_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr description_;
  ::apollo::common::Header* header_;
  double speed_;
  double low_speed_;
  double high_speed_;
  double radius_;
  ::PROTOBUF_NAMESPACE_ID::int32 rsi_type_;
  ::PROTOBUF_NAMESPACE_ID::int32 priority_;
  friend struct ::TableStruct_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// RsiPoint

// optional double x = 1;
inline bool RsiPoint::_internal_has_x() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool RsiPoint::has_x() const {
  return _internal_has_x();
}
inline void RsiPoint::clear_x() {
  x_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline double RsiPoint::_internal_x() const {
  return x_;
}
inline double RsiPoint::x() const {
  // @@protoc_insertion_point(field_get:apollo.v2x.RsiPoint.x)
  return _internal_x();
}
inline void RsiPoint::_internal_set_x(double value) {
  _has_bits_[0] |= 0x00000001u;
  x_ = value;
}
inline void RsiPoint::set_x(double value) {
  _internal_set_x(value);
  // @@protoc_insertion_point(field_set:apollo.v2x.RsiPoint.x)
}

// optional double y = 2;
inline bool RsiPoint::_internal_has_y() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool RsiPoint::has_y() const {
  return _internal_has_y();
}
inline void RsiPoint::clear_y() {
  y_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline double RsiPoint::_internal_y() const {
  return y_;
}
inline double RsiPoint::y() const {
  // @@protoc_insertion_point(field_get:apollo.v2x.RsiPoint.y)
  return _internal_y();
}
inline void RsiPoint::_internal_set_y(double value) {
  _has_bits_[0] |= 0x00000002u;
  y_ = value;
}
inline void RsiPoint::set_y(double value) {
  _internal_set_y(value);
  // @@protoc_insertion_point(field_set:apollo.v2x.RsiPoint.y)
}

// -------------------------------------------------------------------

// RsiMsg

// optional .apollo.common.Header header = 1;
inline bool RsiMsg::_internal_has_header() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || header_ != nullptr);
  return value;
}
inline bool RsiMsg::has_header() const {
  return _internal_has_header();
}
inline const ::apollo::common::Header& RsiMsg::_internal_header() const {
  const ::apollo::common::Header* p = header_;
  return p != nullptr ? *p : reinterpret_cast<const ::apollo::common::Header&>(
      ::apollo::common::_Header_default_instance_);
}
inline const ::apollo::common::Header& RsiMsg::header() const {
  // @@protoc_insertion_point(field_get:apollo.v2x.RsiMsg.header)
  return _internal_header();
}
inline void RsiMsg::unsafe_arena_set_allocated_header(
    ::apollo::common::Header* header) {
  if (GetArena() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  header_ = header;
  if (header) {
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:apollo.v2x.RsiMsg.header)
}
inline ::apollo::common::Header* RsiMsg::release_header() {
  _has_bits_[0] &= ~0x00000002u;
  ::apollo::common::Header* temp = header_;
  header_ = nullptr;
  if (GetArena() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
  return temp;
}
inline ::apollo::common::Header* RsiMsg::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:apollo.v2x.RsiMsg.header)
  _has_bits_[0] &= ~0x00000002u;
  ::apollo::common::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::apollo::common::Header* RsiMsg::_internal_mutable_header() {
  _has_bits_[0] |= 0x00000002u;
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::common::Header>(GetArena());
    header_ = p;
  }
  return header_;
}
inline ::apollo::common::Header* RsiMsg::mutable_header() {
  // @@protoc_insertion_point(field_mutable:apollo.v2x.RsiMsg.header)
  return _internal_mutable_header();
}
inline void RsiMsg::set_allocated_header(::apollo::common::Header* header) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArena();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  if (header) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
      reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header)->GetArena();
    if (message_arena != submessage_arena) {
      header = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:apollo.v2x.RsiMsg.header)
}

// repeated .apollo.v2x.RsiPoint points = 2;
inline int RsiMsg::_internal_points_size() const {
  return points_.size();
}
inline int RsiMsg::points_size() const {
  return _internal_points_size();
}
inline void RsiMsg::clear_points() {
  points_.Clear();
}
inline ::apollo::v2x::RsiPoint* RsiMsg::mutable_points(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.v2x.RsiMsg.points)
  return points_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::v2x::RsiPoint >*
RsiMsg::mutable_points() {
  // @@protoc_insertion_point(field_mutable_list:apollo.v2x.RsiMsg.points)
  return &points_;
}
inline const ::apollo::v2x::RsiPoint& RsiMsg::_internal_points(int index) const {
  return points_.Get(index);
}
inline const ::apollo::v2x::RsiPoint& RsiMsg::points(int index) const {
  // @@protoc_insertion_point(field_get:apollo.v2x.RsiMsg.points)
  return _internal_points(index);
}
inline ::apollo::v2x::RsiPoint* RsiMsg::_internal_add_points() {
  return points_.Add();
}
inline ::apollo::v2x::RsiPoint* RsiMsg::add_points() {
  // @@protoc_insertion_point(field_add:apollo.v2x.RsiMsg.points)
  return _internal_add_points();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::v2x::RsiPoint >&
RsiMsg::points() const {
  // @@protoc_insertion_point(field_list:apollo.v2x.RsiMsg.points)
  return points_;
}

// optional double speed = 3;
inline bool RsiMsg::_internal_has_speed() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool RsiMsg::has_speed() const {
  return _internal_has_speed();
}
inline void RsiMsg::clear_speed() {
  speed_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline double RsiMsg::_internal_speed() const {
  return speed_;
}
inline double RsiMsg::speed() const {
  // @@protoc_insertion_point(field_get:apollo.v2x.RsiMsg.speed)
  return _internal_speed();
}
inline void RsiMsg::_internal_set_speed(double value) {
  _has_bits_[0] |= 0x00000004u;
  speed_ = value;
}
inline void RsiMsg::set_speed(double value) {
  _internal_set_speed(value);
  // @@protoc_insertion_point(field_set:apollo.v2x.RsiMsg.speed)
}

// optional double low_speed = 4;
inline bool RsiMsg::_internal_has_low_speed() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool RsiMsg::has_low_speed() const {
  return _internal_has_low_speed();
}
inline void RsiMsg::clear_low_speed() {
  low_speed_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline double RsiMsg::_internal_low_speed() const {
  return low_speed_;
}
inline double RsiMsg::low_speed() const {
  // @@protoc_insertion_point(field_get:apollo.v2x.RsiMsg.low_speed)
  return _internal_low_speed();
}
inline void RsiMsg::_internal_set_low_speed(double value) {
  _has_bits_[0] |= 0x00000008u;
  low_speed_ = value;
}
inline void RsiMsg::set_low_speed(double value) {
  _internal_set_low_speed(value);
  // @@protoc_insertion_point(field_set:apollo.v2x.RsiMsg.low_speed)
}

// optional double high_speed = 5;
inline bool RsiMsg::_internal_has_high_speed() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool RsiMsg::has_high_speed() const {
  return _internal_has_high_speed();
}
inline void RsiMsg::clear_high_speed() {
  high_speed_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline double RsiMsg::_internal_high_speed() const {
  return high_speed_;
}
inline double RsiMsg::high_speed() const {
  // @@protoc_insertion_point(field_get:apollo.v2x.RsiMsg.high_speed)
  return _internal_high_speed();
}
inline void RsiMsg::_internal_set_high_speed(double value) {
  _has_bits_[0] |= 0x00000010u;
  high_speed_ = value;
}
inline void RsiMsg::set_high_speed(double value) {
  _internal_set_high_speed(value);
  // @@protoc_insertion_point(field_set:apollo.v2x.RsiMsg.high_speed)
}

// optional string description = 6;
inline bool RsiMsg::_internal_has_description() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool RsiMsg::has_description() const {
  return _internal_has_description();
}
inline void RsiMsg::clear_description() {
  description_.ClearToEmpty();
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& RsiMsg::description() const {
  // @@protoc_insertion_point(field_get:apollo.v2x.RsiMsg.description)
  return _internal_description();
}
inline void RsiMsg::set_description(const std::string& value) {
  _internal_set_description(value);
  // @@protoc_insertion_point(field_set:apollo.v2x.RsiMsg.description)
}
inline std::string* RsiMsg::mutable_description() {
  // @@protoc_insertion_point(field_mutable:apollo.v2x.RsiMsg.description)
  return _internal_mutable_description();
}
inline const std::string& RsiMsg::_internal_description() const {
  return description_.Get();
}
inline void RsiMsg::_internal_set_description(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  description_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArena());
}
inline void RsiMsg::set_description(std::string&& value) {
  _has_bits_[0] |= 0x00000001u;
  description_.Set(
    ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::move(value), GetArena());
  // @@protoc_insertion_point(field_set_rvalue:apollo.v2x.RsiMsg.description)
}
inline void RsiMsg::set_description(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000001u;
  description_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::string(value), GetArena());
  // @@protoc_insertion_point(field_set_char:apollo.v2x.RsiMsg.description)
}
inline void RsiMsg::set_description(const char* value,
    size_t size) {
  _has_bits_[0] |= 0x00000001u;
  description_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::string(
      reinterpret_cast<const char*>(value), size), GetArena());
  // @@protoc_insertion_point(field_set_pointer:apollo.v2x.RsiMsg.description)
}
inline std::string* RsiMsg::_internal_mutable_description() {
  _has_bits_[0] |= 0x00000001u;
  return description_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArena());
}
inline std::string* RsiMsg::release_description() {
  // @@protoc_insertion_point(field_release:apollo.v2x.RsiMsg.description)
  if (!_internal_has_description()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return description_.ReleaseNonDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline void RsiMsg::set_allocated_description(std::string* description) {
  if (description != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  description_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), description,
      GetArena());
  // @@protoc_insertion_point(field_set_allocated:apollo.v2x.RsiMsg.description)
}

// optional int32 rsi_type = 7;
inline bool RsiMsg::_internal_has_rsi_type() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool RsiMsg::has_rsi_type() const {
  return _internal_has_rsi_type();
}
inline void RsiMsg::clear_rsi_type() {
  rsi_type_ = 0;
  _has_bits_[0] &= ~0x00000040u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 RsiMsg::_internal_rsi_type() const {
  return rsi_type_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 RsiMsg::rsi_type() const {
  // @@protoc_insertion_point(field_get:apollo.v2x.RsiMsg.rsi_type)
  return _internal_rsi_type();
}
inline void RsiMsg::_internal_set_rsi_type(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000040u;
  rsi_type_ = value;
}
inline void RsiMsg::set_rsi_type(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_rsi_type(value);
  // @@protoc_insertion_point(field_set:apollo.v2x.RsiMsg.rsi_type)
}

// optional double radius = 8;
inline bool RsiMsg::_internal_has_radius() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool RsiMsg::has_radius() const {
  return _internal_has_radius();
}
inline void RsiMsg::clear_radius() {
  radius_ = 0;
  _has_bits_[0] &= ~0x00000020u;
}
inline double RsiMsg::_internal_radius() const {
  return radius_;
}
inline double RsiMsg::radius() const {
  // @@protoc_insertion_point(field_get:apollo.v2x.RsiMsg.radius)
  return _internal_radius();
}
inline void RsiMsg::_internal_set_radius(double value) {
  _has_bits_[0] |= 0x00000020u;
  radius_ = value;
}
inline void RsiMsg::set_radius(double value) {
  _internal_set_radius(value);
  // @@protoc_insertion_point(field_set:apollo.v2x.RsiMsg.radius)
}

// optional int32 priority = 9;
inline bool RsiMsg::_internal_has_priority() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool RsiMsg::has_priority() const {
  return _internal_has_priority();
}
inline void RsiMsg::clear_priority() {
  priority_ = 0;
  _has_bits_[0] &= ~0x00000080u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 RsiMsg::_internal_priority() const {
  return priority_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 RsiMsg::priority() const {
  // @@protoc_insertion_point(field_get:apollo.v2x.RsiMsg.priority)
  return _internal_priority();
}
inline void RsiMsg::_internal_set_priority(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000080u;
  priority_ = value;
}
inline void RsiMsg::set_priority(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_priority(value);
  // @@protoc_insertion_point(field_set:apollo.v2x.RsiMsg.priority)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace v2x
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fv2x_2fproto_2fv2x_5frsi_2eproto
