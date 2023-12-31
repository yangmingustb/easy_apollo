// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/common_msgs/external_command_msgs/lane_segment.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2flane_5fsegment_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2flane_5fsegment_2eproto

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
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2flane_5fsegment_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2flane_5fsegment_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2flane_5fsegment_2eproto;
namespace apollo {
namespace external_command {
class LaneSegment;
class LaneSegmentDefaultTypeInternal;
extern LaneSegmentDefaultTypeInternal _LaneSegment_default_instance_;
}  // namespace external_command
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::external_command::LaneSegment* Arena::CreateMaybeMessage<::apollo::external_command::LaneSegment>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace external_command {

// ===================================================================

class LaneSegment PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.external_command.LaneSegment) */ {
 public:
  inline LaneSegment() : LaneSegment(nullptr) {}
  virtual ~LaneSegment();

  LaneSegment(const LaneSegment& from);
  LaneSegment(LaneSegment&& from) noexcept
    : LaneSegment() {
    *this = ::std::move(from);
  }

  inline LaneSegment& operator=(const LaneSegment& from) {
    CopyFrom(from);
    return *this;
  }
  inline LaneSegment& operator=(LaneSegment&& from) noexcept {
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
  static const LaneSegment& default_instance();

  static inline const LaneSegment* internal_default_instance() {
    return reinterpret_cast<const LaneSegment*>(
               &_LaneSegment_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(LaneSegment& a, LaneSegment& b) {
    a.Swap(&b);
  }
  inline void Swap(LaneSegment* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(LaneSegment* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LaneSegment* New() const final {
    return CreateMaybeMessage<LaneSegment>(nullptr);
  }

  LaneSegment* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LaneSegment>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LaneSegment& from);
  void MergeFrom(const LaneSegment& from);
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
  void InternalSwap(LaneSegment* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.external_command.LaneSegment";
  }
  protected:
  explicit LaneSegment(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2flane_5fsegment_2eproto);
    return ::descriptor_table_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2flane_5fsegment_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kIdFieldNumber = 1,
    kStartSFieldNumber = 2,
    kEndSFieldNumber = 3,
  };
  // optional string id = 1;
  bool has_id() const;
  private:
  bool _internal_has_id() const;
  public:
  void clear_id();
  const std::string& id() const;
  void set_id(const std::string& value);
  void set_id(std::string&& value);
  void set_id(const char* value);
  void set_id(const char* value, size_t size);
  std::string* mutable_id();
  std::string* release_id();
  void set_allocated_id(std::string* id);
  private:
  const std::string& _internal_id() const;
  void _internal_set_id(const std::string& value);
  std::string* _internal_mutable_id();
  public:

  // optional double start_s = 2;
  bool has_start_s() const;
  private:
  bool _internal_has_start_s() const;
  public:
  void clear_start_s();
  double start_s() const;
  void set_start_s(double value);
  private:
  double _internal_start_s() const;
  void _internal_set_start_s(double value);
  public:

  // optional double end_s = 3;
  bool has_end_s() const;
  private:
  bool _internal_has_end_s() const;
  public:
  void clear_end_s();
  double end_s() const;
  void set_end_s(double value);
  private:
  double _internal_end_s() const;
  void _internal_set_end_s(double value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.external_command.LaneSegment)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr id_;
  double start_s_;
  double end_s_;
  friend struct ::TableStruct_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2flane_5fsegment_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// LaneSegment

// optional string id = 1;
inline bool LaneSegment::_internal_has_id() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool LaneSegment::has_id() const {
  return _internal_has_id();
}
inline void LaneSegment::clear_id() {
  id_.ClearToEmpty();
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& LaneSegment::id() const {
  // @@protoc_insertion_point(field_get:apollo.external_command.LaneSegment.id)
  return _internal_id();
}
inline void LaneSegment::set_id(const std::string& value) {
  _internal_set_id(value);
  // @@protoc_insertion_point(field_set:apollo.external_command.LaneSegment.id)
}
inline std::string* LaneSegment::mutable_id() {
  // @@protoc_insertion_point(field_mutable:apollo.external_command.LaneSegment.id)
  return _internal_mutable_id();
}
inline const std::string& LaneSegment::_internal_id() const {
  return id_.Get();
}
inline void LaneSegment::_internal_set_id(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  id_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArena());
}
inline void LaneSegment::set_id(std::string&& value) {
  _has_bits_[0] |= 0x00000001u;
  id_.Set(
    ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::move(value), GetArena());
  // @@protoc_insertion_point(field_set_rvalue:apollo.external_command.LaneSegment.id)
}
inline void LaneSegment::set_id(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000001u;
  id_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::string(value), GetArena());
  // @@protoc_insertion_point(field_set_char:apollo.external_command.LaneSegment.id)
}
inline void LaneSegment::set_id(const char* value,
    size_t size) {
  _has_bits_[0] |= 0x00000001u;
  id_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::string(
      reinterpret_cast<const char*>(value), size), GetArena());
  // @@protoc_insertion_point(field_set_pointer:apollo.external_command.LaneSegment.id)
}
inline std::string* LaneSegment::_internal_mutable_id() {
  _has_bits_[0] |= 0x00000001u;
  return id_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArena());
}
inline std::string* LaneSegment::release_id() {
  // @@protoc_insertion_point(field_release:apollo.external_command.LaneSegment.id)
  if (!_internal_has_id()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return id_.ReleaseNonDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline void LaneSegment::set_allocated_id(std::string* id) {
  if (id != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  id_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), id,
      GetArena());
  // @@protoc_insertion_point(field_set_allocated:apollo.external_command.LaneSegment.id)
}

// optional double start_s = 2;
inline bool LaneSegment::_internal_has_start_s() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool LaneSegment::has_start_s() const {
  return _internal_has_start_s();
}
inline void LaneSegment::clear_start_s() {
  start_s_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline double LaneSegment::_internal_start_s() const {
  return start_s_;
}
inline double LaneSegment::start_s() const {
  // @@protoc_insertion_point(field_get:apollo.external_command.LaneSegment.start_s)
  return _internal_start_s();
}
inline void LaneSegment::_internal_set_start_s(double value) {
  _has_bits_[0] |= 0x00000002u;
  start_s_ = value;
}
inline void LaneSegment::set_start_s(double value) {
  _internal_set_start_s(value);
  // @@protoc_insertion_point(field_set:apollo.external_command.LaneSegment.start_s)
}

// optional double end_s = 3;
inline bool LaneSegment::_internal_has_end_s() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool LaneSegment::has_end_s() const {
  return _internal_has_end_s();
}
inline void LaneSegment::clear_end_s() {
  end_s_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline double LaneSegment::_internal_end_s() const {
  return end_s_;
}
inline double LaneSegment::end_s() const {
  // @@protoc_insertion_point(field_get:apollo.external_command.LaneSegment.end_s)
  return _internal_end_s();
}
inline void LaneSegment::_internal_set_end_s(double value) {
  _has_bits_[0] |= 0x00000004u;
  end_s_ = value;
}
inline void LaneSegment::set_end_s(double value) {
  _internal_set_end_s(value);
  // @@protoc_insertion_point(field_set:apollo.external_command.LaneSegment.end_s)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace external_command
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fcommon_5fmsgs_2fexternal_5fcommand_5fmsgs_2flane_5fsegment_2eproto