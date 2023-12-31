// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: run_mode_conf.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_run_5fmode_5fconf_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_run_5fmode_5fconf_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_run_5fmode_5fconf_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_run_5fmode_5fconf_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_run_5fmode_5fconf_2eproto;
namespace apollo {
namespace cyber {
namespace proto {
class DebugMsg;
class DebugMsgDefaultTypeInternal;
extern DebugMsgDefaultTypeInternal _DebugMsg_default_instance_;
class RunModeConf;
class RunModeConfDefaultTypeInternal;
extern RunModeConfDefaultTypeInternal _RunModeConf_default_instance_;
}  // namespace proto
}  // namespace cyber
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::cyber::proto::DebugMsg* Arena::CreateMaybeMessage<::apollo::cyber::proto::DebugMsg>(Arena*);
template<> ::apollo::cyber::proto::RunModeConf* Arena::CreateMaybeMessage<::apollo::cyber::proto::RunModeConf>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace cyber {
namespace proto {

enum RunMode : int {
  MODE_REALITY = 0,
  MODE_SIMULATION = 1
};
bool RunMode_IsValid(int value);
constexpr RunMode RunMode_MIN = MODE_REALITY;
constexpr RunMode RunMode_MAX = MODE_SIMULATION;
constexpr int RunMode_ARRAYSIZE = RunMode_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* RunMode_descriptor();
template<typename T>
inline const std::string& RunMode_Name(T enum_t_value) {
  static_assert(::std::is_same<T, RunMode>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function RunMode_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    RunMode_descriptor(), enum_t_value);
}
inline bool RunMode_Parse(
    ::PROTOBUF_NAMESPACE_ID::ConstStringParam name, RunMode* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<RunMode>(
    RunMode_descriptor(), name, value);
}
enum ClockMode : int {
  MODE_CYBER = 0,
  MODE_MOCK = 1
};
bool ClockMode_IsValid(int value);
constexpr ClockMode ClockMode_MIN = MODE_CYBER;
constexpr ClockMode ClockMode_MAX = MODE_MOCK;
constexpr int ClockMode_ARRAYSIZE = ClockMode_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* ClockMode_descriptor();
template<typename T>
inline const std::string& ClockMode_Name(T enum_t_value) {
  static_assert(::std::is_same<T, ClockMode>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function ClockMode_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    ClockMode_descriptor(), enum_t_value);
}
inline bool ClockMode_Parse(
    ::PROTOBUF_NAMESPACE_ID::ConstStringParam name, ClockMode* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<ClockMode>(
    ClockMode_descriptor(), name, value);
}
enum DebugMode : int {
  none = 0,
  singleframe_step_forward = 1,
  pause = 2
};
bool DebugMode_IsValid(int value);
constexpr DebugMode DebugMode_MIN = none;
constexpr DebugMode DebugMode_MAX = pause;
constexpr int DebugMode_ARRAYSIZE = DebugMode_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* DebugMode_descriptor();
template<typename T>
inline const std::string& DebugMode_Name(T enum_t_value) {
  static_assert(::std::is_same<T, DebugMode>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function DebugMode_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    DebugMode_descriptor(), enum_t_value);
}
inline bool DebugMode_Parse(
    ::PROTOBUF_NAMESPACE_ID::ConstStringParam name, DebugMode* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<DebugMode>(
    DebugMode_descriptor(), name, value);
}
// ===================================================================

class DebugMsg PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.cyber.proto.DebugMsg) */ {
 public:
  inline DebugMsg() : DebugMsg(nullptr) {}
  virtual ~DebugMsg();

  DebugMsg(const DebugMsg& from);
  DebugMsg(DebugMsg&& from) noexcept
    : DebugMsg() {
    *this = ::std::move(from);
  }

  inline DebugMsg& operator=(const DebugMsg& from) {
    CopyFrom(from);
    return *this;
  }
  inline DebugMsg& operator=(DebugMsg&& from) noexcept {
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
  static const DebugMsg& default_instance();

  static inline const DebugMsg* internal_default_instance() {
    return reinterpret_cast<const DebugMsg*>(
               &_DebugMsg_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(DebugMsg& a, DebugMsg& b) {
    a.Swap(&b);
  }
  inline void Swap(DebugMsg* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(DebugMsg* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline DebugMsg* New() const final {
    return CreateMaybeMessage<DebugMsg>(nullptr);
  }

  DebugMsg* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<DebugMsg>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const DebugMsg& from);
  void MergeFrom(const DebugMsg& from);
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
  void InternalSwap(DebugMsg* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.cyber.proto.DebugMsg";
  }
  protected:
  explicit DebugMsg(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_run_5fmode_5fconf_2eproto);
    return ::descriptor_table_run_5fmode_5fconf_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDebugModeFieldNumber = 1,
  };
  // optional .apollo.cyber.proto.DebugMode debug_mode = 1 [default = none];
  bool has_debug_mode() const;
  private:
  bool _internal_has_debug_mode() const;
  public:
  void clear_debug_mode();
  ::apollo::cyber::proto::DebugMode debug_mode() const;
  void set_debug_mode(::apollo::cyber::proto::DebugMode value);
  private:
  ::apollo::cyber::proto::DebugMode _internal_debug_mode() const;
  void _internal_set_debug_mode(::apollo::cyber::proto::DebugMode value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.cyber.proto.DebugMsg)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  int debug_mode_;
  friend struct ::TableStruct_run_5fmode_5fconf_2eproto;
};
// -------------------------------------------------------------------

class RunModeConf PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.cyber.proto.RunModeConf) */ {
 public:
  inline RunModeConf() : RunModeConf(nullptr) {}
  virtual ~RunModeConf();

  RunModeConf(const RunModeConf& from);
  RunModeConf(RunModeConf&& from) noexcept
    : RunModeConf() {
    *this = ::std::move(from);
  }

  inline RunModeConf& operator=(const RunModeConf& from) {
    CopyFrom(from);
    return *this;
  }
  inline RunModeConf& operator=(RunModeConf&& from) noexcept {
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
  static const RunModeConf& default_instance();

  static inline const RunModeConf* internal_default_instance() {
    return reinterpret_cast<const RunModeConf*>(
               &_RunModeConf_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(RunModeConf& a, RunModeConf& b) {
    a.Swap(&b);
  }
  inline void Swap(RunModeConf* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(RunModeConf* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline RunModeConf* New() const final {
    return CreateMaybeMessage<RunModeConf>(nullptr);
  }

  RunModeConf* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<RunModeConf>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const RunModeConf& from);
  void MergeFrom(const RunModeConf& from);
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
  void InternalSwap(RunModeConf* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.cyber.proto.RunModeConf";
  }
  protected:
  explicit RunModeConf(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_run_5fmode_5fconf_2eproto);
    return ::descriptor_table_run_5fmode_5fconf_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kRunModeFieldNumber = 1,
    kClockModeFieldNumber = 2,
  };
  // optional .apollo.cyber.proto.RunMode run_mode = 1 [default = MODE_REALITY];
  bool has_run_mode() const;
  private:
  bool _internal_has_run_mode() const;
  public:
  void clear_run_mode();
  ::apollo::cyber::proto::RunMode run_mode() const;
  void set_run_mode(::apollo::cyber::proto::RunMode value);
  private:
  ::apollo::cyber::proto::RunMode _internal_run_mode() const;
  void _internal_set_run_mode(::apollo::cyber::proto::RunMode value);
  public:

  // optional .apollo.cyber.proto.ClockMode clock_mode = 2 [default = MODE_CYBER];
  bool has_clock_mode() const;
  private:
  bool _internal_has_clock_mode() const;
  public:
  void clear_clock_mode();
  ::apollo::cyber::proto::ClockMode clock_mode() const;
  void set_clock_mode(::apollo::cyber::proto::ClockMode value);
  private:
  ::apollo::cyber::proto::ClockMode _internal_clock_mode() const;
  void _internal_set_clock_mode(::apollo::cyber::proto::ClockMode value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.cyber.proto.RunModeConf)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  int run_mode_;
  int clock_mode_;
  friend struct ::TableStruct_run_5fmode_5fconf_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// DebugMsg

// optional .apollo.cyber.proto.DebugMode debug_mode = 1 [default = none];
inline bool DebugMsg::_internal_has_debug_mode() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool DebugMsg::has_debug_mode() const {
  return _internal_has_debug_mode();
}
inline void DebugMsg::clear_debug_mode() {
  debug_mode_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::apollo::cyber::proto::DebugMode DebugMsg::_internal_debug_mode() const {
  return static_cast< ::apollo::cyber::proto::DebugMode >(debug_mode_);
}
inline ::apollo::cyber::proto::DebugMode DebugMsg::debug_mode() const {
  // @@protoc_insertion_point(field_get:apollo.cyber.proto.DebugMsg.debug_mode)
  return _internal_debug_mode();
}
inline void DebugMsg::_internal_set_debug_mode(::apollo::cyber::proto::DebugMode value) {
  assert(::apollo::cyber::proto::DebugMode_IsValid(value));
  _has_bits_[0] |= 0x00000001u;
  debug_mode_ = value;
}
inline void DebugMsg::set_debug_mode(::apollo::cyber::proto::DebugMode value) {
  _internal_set_debug_mode(value);
  // @@protoc_insertion_point(field_set:apollo.cyber.proto.DebugMsg.debug_mode)
}

// -------------------------------------------------------------------

// RunModeConf

// optional .apollo.cyber.proto.RunMode run_mode = 1 [default = MODE_REALITY];
inline bool RunModeConf::_internal_has_run_mode() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool RunModeConf::has_run_mode() const {
  return _internal_has_run_mode();
}
inline void RunModeConf::clear_run_mode() {
  run_mode_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::apollo::cyber::proto::RunMode RunModeConf::_internal_run_mode() const {
  return static_cast< ::apollo::cyber::proto::RunMode >(run_mode_);
}
inline ::apollo::cyber::proto::RunMode RunModeConf::run_mode() const {
  // @@protoc_insertion_point(field_get:apollo.cyber.proto.RunModeConf.run_mode)
  return _internal_run_mode();
}
inline void RunModeConf::_internal_set_run_mode(::apollo::cyber::proto::RunMode value) {
  assert(::apollo::cyber::proto::RunMode_IsValid(value));
  _has_bits_[0] |= 0x00000001u;
  run_mode_ = value;
}
inline void RunModeConf::set_run_mode(::apollo::cyber::proto::RunMode value) {
  _internal_set_run_mode(value);
  // @@protoc_insertion_point(field_set:apollo.cyber.proto.RunModeConf.run_mode)
}

// optional .apollo.cyber.proto.ClockMode clock_mode = 2 [default = MODE_CYBER];
inline bool RunModeConf::_internal_has_clock_mode() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool RunModeConf::has_clock_mode() const {
  return _internal_has_clock_mode();
}
inline void RunModeConf::clear_clock_mode() {
  clock_mode_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::apollo::cyber::proto::ClockMode RunModeConf::_internal_clock_mode() const {
  return static_cast< ::apollo::cyber::proto::ClockMode >(clock_mode_);
}
inline ::apollo::cyber::proto::ClockMode RunModeConf::clock_mode() const {
  // @@protoc_insertion_point(field_get:apollo.cyber.proto.RunModeConf.clock_mode)
  return _internal_clock_mode();
}
inline void RunModeConf::_internal_set_clock_mode(::apollo::cyber::proto::ClockMode value) {
  assert(::apollo::cyber::proto::ClockMode_IsValid(value));
  _has_bits_[0] |= 0x00000002u;
  clock_mode_ = value;
}
inline void RunModeConf::set_clock_mode(::apollo::cyber::proto::ClockMode value) {
  _internal_set_clock_mode(value);
  // @@protoc_insertion_point(field_set:apollo.cyber.proto.RunModeConf.clock_mode)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace cyber
}  // namespace apollo

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::apollo::cyber::proto::RunMode> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::cyber::proto::RunMode>() {
  return ::apollo::cyber::proto::RunMode_descriptor();
}
template <> struct is_proto_enum< ::apollo::cyber::proto::ClockMode> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::cyber::proto::ClockMode>() {
  return ::apollo::cyber::proto::ClockMode_descriptor();
}
template <> struct is_proto_enum< ::apollo::cyber::proto::DebugMode> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::cyber::proto::DebugMode>() {
  return ::apollo::cyber::proto::DebugMode_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_run_5fmode_5fconf_2eproto
