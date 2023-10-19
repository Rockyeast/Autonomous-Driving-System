// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/common/proto/drive_event.proto

#ifndef PROTOBUF_INCLUDED_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto
#define PROTOBUF_INCLUDED_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3006000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3006000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
#include "modules/common/proto/header.pb.h"
#include "modules/localization/proto/pose.pb.h"
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto 

namespace protobuf_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[1];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto
namespace apollo {
namespace common {
class DriveEvent;
class DriveEventDefaultTypeInternal;
extern DriveEventDefaultTypeInternal _DriveEvent_default_instance_;
}  // namespace common
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::common::DriveEvent* Arena::CreateMaybeMessage<::apollo::common::DriveEvent>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace common {

enum DriveEvent_Type {
  DriveEvent_Type_CRITICAL = 0,
  DriveEvent_Type_PROBLEM = 1,
  DriveEvent_Type_DESIRED = 2,
  DriveEvent_Type_OUT_OF_SCOPE = 3
};
bool DriveEvent_Type_IsValid(int value);
const DriveEvent_Type DriveEvent_Type_Type_MIN = DriveEvent_Type_CRITICAL;
const DriveEvent_Type DriveEvent_Type_Type_MAX = DriveEvent_Type_OUT_OF_SCOPE;
const int DriveEvent_Type_Type_ARRAYSIZE = DriveEvent_Type_Type_MAX + 1;

const ::google::protobuf::EnumDescriptor* DriveEvent_Type_descriptor();
inline const ::std::string& DriveEvent_Type_Name(DriveEvent_Type value) {
  return ::google::protobuf::internal::NameOfEnum(
    DriveEvent_Type_descriptor(), value);
}
inline bool DriveEvent_Type_Parse(
    const ::std::string& name, DriveEvent_Type* value) {
  return ::google::protobuf::internal::ParseNamedEnum<DriveEvent_Type>(
    DriveEvent_Type_descriptor(), name, value);
}
// ===================================================================

class DriveEvent : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.common.DriveEvent) */ {
 public:
  DriveEvent();
  virtual ~DriveEvent();

  DriveEvent(const DriveEvent& from);

  inline DriveEvent& operator=(const DriveEvent& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  DriveEvent(DriveEvent&& from) noexcept
    : DriveEvent() {
    *this = ::std::move(from);
  }

  inline DriveEvent& operator=(DriveEvent&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const DriveEvent& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const DriveEvent* internal_default_instance() {
    return reinterpret_cast<const DriveEvent*>(
               &_DriveEvent_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(DriveEvent* other);
  friend void swap(DriveEvent& a, DriveEvent& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline DriveEvent* New() const final {
    return CreateMaybeMessage<DriveEvent>(NULL);
  }

  DriveEvent* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<DriveEvent>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const DriveEvent& from);
  void MergeFrom(const DriveEvent& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(DriveEvent* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  typedef DriveEvent_Type Type;
  static const Type CRITICAL =
    DriveEvent_Type_CRITICAL;
  static const Type PROBLEM =
    DriveEvent_Type_PROBLEM;
  static const Type DESIRED =
    DriveEvent_Type_DESIRED;
  static const Type OUT_OF_SCOPE =
    DriveEvent_Type_OUT_OF_SCOPE;
  static inline bool Type_IsValid(int value) {
    return DriveEvent_Type_IsValid(value);
  }
  static const Type Type_MIN =
    DriveEvent_Type_Type_MIN;
  static const Type Type_MAX =
    DriveEvent_Type_Type_MAX;
  static const int Type_ARRAYSIZE =
    DriveEvent_Type_Type_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  Type_descriptor() {
    return DriveEvent_Type_descriptor();
  }
  static inline const ::std::string& Type_Name(Type value) {
    return DriveEvent_Type_Name(value);
  }
  static inline bool Type_Parse(const ::std::string& name,
      Type* value) {
    return DriveEvent_Type_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // repeated .apollo.common.DriveEvent.Type type = 4;
  int type_size() const;
  void clear_type();
  static const int kTypeFieldNumber = 4;
  ::apollo::common::DriveEvent_Type type(int index) const;
  void set_type(int index, ::apollo::common::DriveEvent_Type value);
  void add_type(::apollo::common::DriveEvent_Type value);
  const ::google::protobuf::RepeatedField<int>& type() const;
  ::google::protobuf::RepeatedField<int>* mutable_type();

  // optional string event = 2;
  bool has_event() const;
  void clear_event();
  static const int kEventFieldNumber = 2;
  const ::std::string& event() const;
  void set_event(const ::std::string& value);
  #if LANG_CXX11
  void set_event(::std::string&& value);
  #endif
  void set_event(const char* value);
  void set_event(const char* value, size_t size);
  ::std::string* mutable_event();
  ::std::string* release_event();
  void set_allocated_event(::std::string* event);

  // optional .apollo.common.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  private:
  const ::apollo::common::Header& _internal_header() const;
  public:
  const ::apollo::common::Header& header() const;
  ::apollo::common::Header* release_header();
  ::apollo::common::Header* mutable_header();
  void set_allocated_header(::apollo::common::Header* header);

  // optional .apollo.localization.Pose location = 3;
  bool has_location() const;
  void clear_location();
  static const int kLocationFieldNumber = 3;
  private:
  const ::apollo::localization::Pose& _internal_location() const;
  public:
  const ::apollo::localization::Pose& location() const;
  ::apollo::localization::Pose* release_location();
  ::apollo::localization::Pose* mutable_location();
  void set_allocated_location(::apollo::localization::Pose* location);

  // optional bool is_reportable = 5;
  bool has_is_reportable() const;
  void clear_is_reportable();
  static const int kIsReportableFieldNumber = 5;
  bool is_reportable() const;
  void set_is_reportable(bool value);

  // @@protoc_insertion_point(class_scope:apollo.common.DriveEvent)
 private:
  void set_has_header();
  void clear_has_header();
  void set_has_event();
  void clear_has_event();
  void set_has_location();
  void clear_has_location();
  void set_has_is_reportable();
  void clear_has_is_reportable();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedField<int> type_;
  ::google::protobuf::internal::ArenaStringPtr event_;
  ::apollo::common::Header* header_;
  ::apollo::localization::Pose* location_;
  bool is_reportable_;
  friend struct ::protobuf_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// DriveEvent

// optional .apollo.common.Header header = 1;
inline bool DriveEvent::has_header() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void DriveEvent::set_has_header() {
  _has_bits_[0] |= 0x00000002u;
}
inline void DriveEvent::clear_has_header() {
  _has_bits_[0] &= ~0x00000002u;
}
inline const ::apollo::common::Header& DriveEvent::_internal_header() const {
  return *header_;
}
inline const ::apollo::common::Header& DriveEvent::header() const {
  const ::apollo::common::Header* p = header_;
  // @@protoc_insertion_point(field_get:apollo.common.DriveEvent.header)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::common::Header*>(
      &::apollo::common::_Header_default_instance_);
}
inline ::apollo::common::Header* DriveEvent::release_header() {
  // @@protoc_insertion_point(field_release:apollo.common.DriveEvent.header)
  clear_has_header();
  ::apollo::common::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::apollo::common::Header* DriveEvent::mutable_header() {
  set_has_header();
  if (header_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::common::Header>(GetArenaNoVirtual());
    header_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.common.DriveEvent.header)
  return header_;
}
inline void DriveEvent::set_allocated_header(::apollo::common::Header* header) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(header_);
  }
  if (header) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      header = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    set_has_header();
  } else {
    clear_has_header();
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:apollo.common.DriveEvent.header)
}

// optional string event = 2;
inline bool DriveEvent::has_event() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void DriveEvent::set_has_event() {
  _has_bits_[0] |= 0x00000001u;
}
inline void DriveEvent::clear_has_event() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void DriveEvent::clear_event() {
  event_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_event();
}
inline const ::std::string& DriveEvent::event() const {
  // @@protoc_insertion_point(field_get:apollo.common.DriveEvent.event)
  return event_.GetNoArena();
}
inline void DriveEvent::set_event(const ::std::string& value) {
  set_has_event();
  event_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:apollo.common.DriveEvent.event)
}
#if LANG_CXX11
inline void DriveEvent::set_event(::std::string&& value) {
  set_has_event();
  event_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.common.DriveEvent.event)
}
#endif
inline void DriveEvent::set_event(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_event();
  event_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.common.DriveEvent.event)
}
inline void DriveEvent::set_event(const char* value, size_t size) {
  set_has_event();
  event_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.common.DriveEvent.event)
}
inline ::std::string* DriveEvent::mutable_event() {
  set_has_event();
  // @@protoc_insertion_point(field_mutable:apollo.common.DriveEvent.event)
  return event_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* DriveEvent::release_event() {
  // @@protoc_insertion_point(field_release:apollo.common.DriveEvent.event)
  if (!has_event()) {
    return NULL;
  }
  clear_has_event();
  return event_.ReleaseNonDefaultNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void DriveEvent::set_allocated_event(::std::string* event) {
  if (event != NULL) {
    set_has_event();
  } else {
    clear_has_event();
  }
  event_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), event);
  // @@protoc_insertion_point(field_set_allocated:apollo.common.DriveEvent.event)
}

// optional .apollo.localization.Pose location = 3;
inline bool DriveEvent::has_location() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void DriveEvent::set_has_location() {
  _has_bits_[0] |= 0x00000004u;
}
inline void DriveEvent::clear_has_location() {
  _has_bits_[0] &= ~0x00000004u;
}
inline const ::apollo::localization::Pose& DriveEvent::_internal_location() const {
  return *location_;
}
inline const ::apollo::localization::Pose& DriveEvent::location() const {
  const ::apollo::localization::Pose* p = location_;
  // @@protoc_insertion_point(field_get:apollo.common.DriveEvent.location)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::localization::Pose*>(
      &::apollo::localization::_Pose_default_instance_);
}
inline ::apollo::localization::Pose* DriveEvent::release_location() {
  // @@protoc_insertion_point(field_release:apollo.common.DriveEvent.location)
  clear_has_location();
  ::apollo::localization::Pose* temp = location_;
  location_ = NULL;
  return temp;
}
inline ::apollo::localization::Pose* DriveEvent::mutable_location() {
  set_has_location();
  if (location_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::localization::Pose>(GetArenaNoVirtual());
    location_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.common.DriveEvent.location)
  return location_;
}
inline void DriveEvent::set_allocated_location(::apollo::localization::Pose* location) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(location_);
  }
  if (location) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      location = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, location, submessage_arena);
    }
    set_has_location();
  } else {
    clear_has_location();
  }
  location_ = location;
  // @@protoc_insertion_point(field_set_allocated:apollo.common.DriveEvent.location)
}

// repeated .apollo.common.DriveEvent.Type type = 4;
inline int DriveEvent::type_size() const {
  return type_.size();
}
inline void DriveEvent::clear_type() {
  type_.Clear();
}
inline ::apollo::common::DriveEvent_Type DriveEvent::type(int index) const {
  // @@protoc_insertion_point(field_get:apollo.common.DriveEvent.type)
  return static_cast< ::apollo::common::DriveEvent_Type >(type_.Get(index));
}
inline void DriveEvent::set_type(int index, ::apollo::common::DriveEvent_Type value) {
  assert(::apollo::common::DriveEvent_Type_IsValid(value));
  type_.Set(index, value);
  // @@protoc_insertion_point(field_set:apollo.common.DriveEvent.type)
}
inline void DriveEvent::add_type(::apollo::common::DriveEvent_Type value) {
  assert(::apollo::common::DriveEvent_Type_IsValid(value));
  type_.Add(value);
  // @@protoc_insertion_point(field_add:apollo.common.DriveEvent.type)
}
inline const ::google::protobuf::RepeatedField<int>&
DriveEvent::type() const {
  // @@protoc_insertion_point(field_list:apollo.common.DriveEvent.type)
  return type_;
}
inline ::google::protobuf::RepeatedField<int>*
DriveEvent::mutable_type() {
  // @@protoc_insertion_point(field_mutable_list:apollo.common.DriveEvent.type)
  return &type_;
}

// optional bool is_reportable = 5;
inline bool DriveEvent::has_is_reportable() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void DriveEvent::set_has_is_reportable() {
  _has_bits_[0] |= 0x00000008u;
}
inline void DriveEvent::clear_has_is_reportable() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void DriveEvent::clear_is_reportable() {
  is_reportable_ = false;
  clear_has_is_reportable();
}
inline bool DriveEvent::is_reportable() const {
  // @@protoc_insertion_point(field_get:apollo.common.DriveEvent.is_reportable)
  return is_reportable_;
}
inline void DriveEvent::set_is_reportable(bool value) {
  set_has_is_reportable();
  is_reportable_ = value;
  // @@protoc_insertion_point(field_set:apollo.common.DriveEvent.is_reportable)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace common
}  // namespace apollo

namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::apollo::common::DriveEvent_Type> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::common::DriveEvent_Type>() {
  return ::apollo::common::DriveEvent_Type_descriptor();
}

}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto
