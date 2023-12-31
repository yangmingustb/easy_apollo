// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/storytelling/proto/storytelling_config.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fstorytelling_2fproto_2fstorytelling_5fconfig_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fstorytelling_2fproto_2fstorytelling_5fconfig_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_modules_2fstorytelling_2fproto_2fstorytelling_5fconfig_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fstorytelling_2fproto_2fstorytelling_5fconfig_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fstorytelling_2fproto_2fstorytelling_5fconfig_2eproto;
namespace apollo {
namespace storytelling {
class StorytellingConfig;
class StorytellingConfigDefaultTypeInternal;
extern StorytellingConfigDefaultTypeInternal _StorytellingConfig_default_instance_;
class TopicConfig;
class TopicConfigDefaultTypeInternal;
extern TopicConfigDefaultTypeInternal _TopicConfig_default_instance_;
}  // namespace storytelling
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::storytelling::StorytellingConfig* Arena::CreateMaybeMessage<::apollo::storytelling::StorytellingConfig>(Arena*);
template<> ::apollo::storytelling::TopicConfig* Arena::CreateMaybeMessage<::apollo::storytelling::TopicConfig>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace storytelling {

// ===================================================================

class TopicConfig PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.storytelling.TopicConfig) */ {
 public:
  inline TopicConfig() : TopicConfig(nullptr) {}
  virtual ~TopicConfig();

  TopicConfig(const TopicConfig& from);
  TopicConfig(TopicConfig&& from) noexcept
    : TopicConfig() {
    *this = ::std::move(from);
  }

  inline TopicConfig& operator=(const TopicConfig& from) {
    CopyFrom(from);
    return *this;
  }
  inline TopicConfig& operator=(TopicConfig&& from) noexcept {
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
  static const TopicConfig& default_instance();

  static inline const TopicConfig* internal_default_instance() {
    return reinterpret_cast<const TopicConfig*>(
               &_TopicConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(TopicConfig& a, TopicConfig& b) {
    a.Swap(&b);
  }
  inline void Swap(TopicConfig* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(TopicConfig* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline TopicConfig* New() const final {
    return CreateMaybeMessage<TopicConfig>(nullptr);
  }

  TopicConfig* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<TopicConfig>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const TopicConfig& from);
  void MergeFrom(const TopicConfig& from);
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
  void InternalSwap(TopicConfig* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.storytelling.TopicConfig";
  }
  protected:
  explicit TopicConfig(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fstorytelling_2fproto_2fstorytelling_5fconfig_2eproto);
    return ::descriptor_table_modules_2fstorytelling_2fproto_2fstorytelling_5fconfig_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kPlanningTrajectoryTopicFieldNumber = 1,
    kStorytellingTopicFieldNumber = 2,
  };
  // optional string planning_trajectory_topic = 1;
  bool has_planning_trajectory_topic() const;
  private:
  bool _internal_has_planning_trajectory_topic() const;
  public:
  void clear_planning_trajectory_topic();
  const std::string& planning_trajectory_topic() const;
  void set_planning_trajectory_topic(const std::string& value);
  void set_planning_trajectory_topic(std::string&& value);
  void set_planning_trajectory_topic(const char* value);
  void set_planning_trajectory_topic(const char* value, size_t size);
  std::string* mutable_planning_trajectory_topic();
  std::string* release_planning_trajectory_topic();
  void set_allocated_planning_trajectory_topic(std::string* planning_trajectory_topic);
  private:
  const std::string& _internal_planning_trajectory_topic() const;
  void _internal_set_planning_trajectory_topic(const std::string& value);
  std::string* _internal_mutable_planning_trajectory_topic();
  public:

  // optional string storytelling_topic = 2;
  bool has_storytelling_topic() const;
  private:
  bool _internal_has_storytelling_topic() const;
  public:
  void clear_storytelling_topic();
  const std::string& storytelling_topic() const;
  void set_storytelling_topic(const std::string& value);
  void set_storytelling_topic(std::string&& value);
  void set_storytelling_topic(const char* value);
  void set_storytelling_topic(const char* value, size_t size);
  std::string* mutable_storytelling_topic();
  std::string* release_storytelling_topic();
  void set_allocated_storytelling_topic(std::string* storytelling_topic);
  private:
  const std::string& _internal_storytelling_topic() const;
  void _internal_set_storytelling_topic(const std::string& value);
  std::string* _internal_mutable_storytelling_topic();
  public:

  // @@protoc_insertion_point(class_scope:apollo.storytelling.TopicConfig)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr planning_trajectory_topic_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr storytelling_topic_;
  friend struct ::TableStruct_modules_2fstorytelling_2fproto_2fstorytelling_5fconfig_2eproto;
};
// -------------------------------------------------------------------

class StorytellingConfig PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.storytelling.StorytellingConfig) */ {
 public:
  inline StorytellingConfig() : StorytellingConfig(nullptr) {}
  virtual ~StorytellingConfig();

  StorytellingConfig(const StorytellingConfig& from);
  StorytellingConfig(StorytellingConfig&& from) noexcept
    : StorytellingConfig() {
    *this = ::std::move(from);
  }

  inline StorytellingConfig& operator=(const StorytellingConfig& from) {
    CopyFrom(from);
    return *this;
  }
  inline StorytellingConfig& operator=(StorytellingConfig&& from) noexcept {
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
  static const StorytellingConfig& default_instance();

  static inline const StorytellingConfig* internal_default_instance() {
    return reinterpret_cast<const StorytellingConfig*>(
               &_StorytellingConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(StorytellingConfig& a, StorytellingConfig& b) {
    a.Swap(&b);
  }
  inline void Swap(StorytellingConfig* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(StorytellingConfig* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline StorytellingConfig* New() const final {
    return CreateMaybeMessage<StorytellingConfig>(nullptr);
  }

  StorytellingConfig* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<StorytellingConfig>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const StorytellingConfig& from);
  void MergeFrom(const StorytellingConfig& from);
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
  void InternalSwap(StorytellingConfig* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.storytelling.StorytellingConfig";
  }
  protected:
  explicit StorytellingConfig(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fstorytelling_2fproto_2fstorytelling_5fconfig_2eproto);
    return ::descriptor_table_modules_2fstorytelling_2fproto_2fstorytelling_5fconfig_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kTopicConfigFieldNumber = 1,
  };
  // optional .apollo.storytelling.TopicConfig topic_config = 1;
  bool has_topic_config() const;
  private:
  bool _internal_has_topic_config() const;
  public:
  void clear_topic_config();
  const ::apollo::storytelling::TopicConfig& topic_config() const;
  ::apollo::storytelling::TopicConfig* release_topic_config();
  ::apollo::storytelling::TopicConfig* mutable_topic_config();
  void set_allocated_topic_config(::apollo::storytelling::TopicConfig* topic_config);
  private:
  const ::apollo::storytelling::TopicConfig& _internal_topic_config() const;
  ::apollo::storytelling::TopicConfig* _internal_mutable_topic_config();
  public:
  void unsafe_arena_set_allocated_topic_config(
      ::apollo::storytelling::TopicConfig* topic_config);
  ::apollo::storytelling::TopicConfig* unsafe_arena_release_topic_config();

  // @@protoc_insertion_point(class_scope:apollo.storytelling.StorytellingConfig)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::apollo::storytelling::TopicConfig* topic_config_;
  friend struct ::TableStruct_modules_2fstorytelling_2fproto_2fstorytelling_5fconfig_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// TopicConfig

// optional string planning_trajectory_topic = 1;
inline bool TopicConfig::_internal_has_planning_trajectory_topic() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool TopicConfig::has_planning_trajectory_topic() const {
  return _internal_has_planning_trajectory_topic();
}
inline void TopicConfig::clear_planning_trajectory_topic() {
  planning_trajectory_topic_.ClearToEmpty();
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& TopicConfig::planning_trajectory_topic() const {
  // @@protoc_insertion_point(field_get:apollo.storytelling.TopicConfig.planning_trajectory_topic)
  return _internal_planning_trajectory_topic();
}
inline void TopicConfig::set_planning_trajectory_topic(const std::string& value) {
  _internal_set_planning_trajectory_topic(value);
  // @@protoc_insertion_point(field_set:apollo.storytelling.TopicConfig.planning_trajectory_topic)
}
inline std::string* TopicConfig::mutable_planning_trajectory_topic() {
  // @@protoc_insertion_point(field_mutable:apollo.storytelling.TopicConfig.planning_trajectory_topic)
  return _internal_mutable_planning_trajectory_topic();
}
inline const std::string& TopicConfig::_internal_planning_trajectory_topic() const {
  return planning_trajectory_topic_.Get();
}
inline void TopicConfig::_internal_set_planning_trajectory_topic(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  planning_trajectory_topic_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArena());
}
inline void TopicConfig::set_planning_trajectory_topic(std::string&& value) {
  _has_bits_[0] |= 0x00000001u;
  planning_trajectory_topic_.Set(
    ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::move(value), GetArena());
  // @@protoc_insertion_point(field_set_rvalue:apollo.storytelling.TopicConfig.planning_trajectory_topic)
}
inline void TopicConfig::set_planning_trajectory_topic(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000001u;
  planning_trajectory_topic_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::string(value), GetArena());
  // @@protoc_insertion_point(field_set_char:apollo.storytelling.TopicConfig.planning_trajectory_topic)
}
inline void TopicConfig::set_planning_trajectory_topic(const char* value,
    size_t size) {
  _has_bits_[0] |= 0x00000001u;
  planning_trajectory_topic_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::string(
      reinterpret_cast<const char*>(value), size), GetArena());
  // @@protoc_insertion_point(field_set_pointer:apollo.storytelling.TopicConfig.planning_trajectory_topic)
}
inline std::string* TopicConfig::_internal_mutable_planning_trajectory_topic() {
  _has_bits_[0] |= 0x00000001u;
  return planning_trajectory_topic_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArena());
}
inline std::string* TopicConfig::release_planning_trajectory_topic() {
  // @@protoc_insertion_point(field_release:apollo.storytelling.TopicConfig.planning_trajectory_topic)
  if (!_internal_has_planning_trajectory_topic()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return planning_trajectory_topic_.ReleaseNonDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline void TopicConfig::set_allocated_planning_trajectory_topic(std::string* planning_trajectory_topic) {
  if (planning_trajectory_topic != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  planning_trajectory_topic_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), planning_trajectory_topic,
      GetArena());
  // @@protoc_insertion_point(field_set_allocated:apollo.storytelling.TopicConfig.planning_trajectory_topic)
}

// optional string storytelling_topic = 2;
inline bool TopicConfig::_internal_has_storytelling_topic() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool TopicConfig::has_storytelling_topic() const {
  return _internal_has_storytelling_topic();
}
inline void TopicConfig::clear_storytelling_topic() {
  storytelling_topic_.ClearToEmpty();
  _has_bits_[0] &= ~0x00000002u;
}
inline const std::string& TopicConfig::storytelling_topic() const {
  // @@protoc_insertion_point(field_get:apollo.storytelling.TopicConfig.storytelling_topic)
  return _internal_storytelling_topic();
}
inline void TopicConfig::set_storytelling_topic(const std::string& value) {
  _internal_set_storytelling_topic(value);
  // @@protoc_insertion_point(field_set:apollo.storytelling.TopicConfig.storytelling_topic)
}
inline std::string* TopicConfig::mutable_storytelling_topic() {
  // @@protoc_insertion_point(field_mutable:apollo.storytelling.TopicConfig.storytelling_topic)
  return _internal_mutable_storytelling_topic();
}
inline const std::string& TopicConfig::_internal_storytelling_topic() const {
  return storytelling_topic_.Get();
}
inline void TopicConfig::_internal_set_storytelling_topic(const std::string& value) {
  _has_bits_[0] |= 0x00000002u;
  storytelling_topic_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArena());
}
inline void TopicConfig::set_storytelling_topic(std::string&& value) {
  _has_bits_[0] |= 0x00000002u;
  storytelling_topic_.Set(
    ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::move(value), GetArena());
  // @@protoc_insertion_point(field_set_rvalue:apollo.storytelling.TopicConfig.storytelling_topic)
}
inline void TopicConfig::set_storytelling_topic(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000002u;
  storytelling_topic_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::string(value), GetArena());
  // @@protoc_insertion_point(field_set_char:apollo.storytelling.TopicConfig.storytelling_topic)
}
inline void TopicConfig::set_storytelling_topic(const char* value,
    size_t size) {
  _has_bits_[0] |= 0x00000002u;
  storytelling_topic_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::string(
      reinterpret_cast<const char*>(value), size), GetArena());
  // @@protoc_insertion_point(field_set_pointer:apollo.storytelling.TopicConfig.storytelling_topic)
}
inline std::string* TopicConfig::_internal_mutable_storytelling_topic() {
  _has_bits_[0] |= 0x00000002u;
  return storytelling_topic_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArena());
}
inline std::string* TopicConfig::release_storytelling_topic() {
  // @@protoc_insertion_point(field_release:apollo.storytelling.TopicConfig.storytelling_topic)
  if (!_internal_has_storytelling_topic()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000002u;
  return storytelling_topic_.ReleaseNonDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline void TopicConfig::set_allocated_storytelling_topic(std::string* storytelling_topic) {
  if (storytelling_topic != nullptr) {
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  storytelling_topic_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), storytelling_topic,
      GetArena());
  // @@protoc_insertion_point(field_set_allocated:apollo.storytelling.TopicConfig.storytelling_topic)
}

// -------------------------------------------------------------------

// StorytellingConfig

// optional .apollo.storytelling.TopicConfig topic_config = 1;
inline bool StorytellingConfig::_internal_has_topic_config() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || topic_config_ != nullptr);
  return value;
}
inline bool StorytellingConfig::has_topic_config() const {
  return _internal_has_topic_config();
}
inline void StorytellingConfig::clear_topic_config() {
  if (topic_config_ != nullptr) topic_config_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::apollo::storytelling::TopicConfig& StorytellingConfig::_internal_topic_config() const {
  const ::apollo::storytelling::TopicConfig* p = topic_config_;
  return p != nullptr ? *p : reinterpret_cast<const ::apollo::storytelling::TopicConfig&>(
      ::apollo::storytelling::_TopicConfig_default_instance_);
}
inline const ::apollo::storytelling::TopicConfig& StorytellingConfig::topic_config() const {
  // @@protoc_insertion_point(field_get:apollo.storytelling.StorytellingConfig.topic_config)
  return _internal_topic_config();
}
inline void StorytellingConfig::unsafe_arena_set_allocated_topic_config(
    ::apollo::storytelling::TopicConfig* topic_config) {
  if (GetArena() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(topic_config_);
  }
  topic_config_ = topic_config;
  if (topic_config) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:apollo.storytelling.StorytellingConfig.topic_config)
}
inline ::apollo::storytelling::TopicConfig* StorytellingConfig::release_topic_config() {
  _has_bits_[0] &= ~0x00000001u;
  ::apollo::storytelling::TopicConfig* temp = topic_config_;
  topic_config_ = nullptr;
  if (GetArena() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
  return temp;
}
inline ::apollo::storytelling::TopicConfig* StorytellingConfig::unsafe_arena_release_topic_config() {
  // @@protoc_insertion_point(field_release:apollo.storytelling.StorytellingConfig.topic_config)
  _has_bits_[0] &= ~0x00000001u;
  ::apollo::storytelling::TopicConfig* temp = topic_config_;
  topic_config_ = nullptr;
  return temp;
}
inline ::apollo::storytelling::TopicConfig* StorytellingConfig::_internal_mutable_topic_config() {
  _has_bits_[0] |= 0x00000001u;
  if (topic_config_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::storytelling::TopicConfig>(GetArena());
    topic_config_ = p;
  }
  return topic_config_;
}
inline ::apollo::storytelling::TopicConfig* StorytellingConfig::mutable_topic_config() {
  // @@protoc_insertion_point(field_mutable:apollo.storytelling.StorytellingConfig.topic_config)
  return _internal_mutable_topic_config();
}
inline void StorytellingConfig::set_allocated_topic_config(::apollo::storytelling::TopicConfig* topic_config) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArena();
  if (message_arena == nullptr) {
    delete topic_config_;
  }
  if (topic_config) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
      ::PROTOBUF_NAMESPACE_ID::Arena::GetArena(topic_config);
    if (message_arena != submessage_arena) {
      topic_config = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, topic_config, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  topic_config_ = topic_config;
  // @@protoc_insertion_point(field_set_allocated:apollo.storytelling.StorytellingConfig.topic_config)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace storytelling
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fstorytelling_2fproto_2fstorytelling_5fconfig_2eproto
