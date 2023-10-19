// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/common/monitor_log/proto/monitor_log.proto

#ifndef PROTOBUF_INCLUDED_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto
#define PROTOBUF_INCLUDED_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto

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
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto 

namespace protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[2];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto
namespace apollo {
namespace common {
namespace monitor {
class MonitorMessage;
class MonitorMessageDefaultTypeInternal;
extern MonitorMessageDefaultTypeInternal _MonitorMessage_default_instance_;
class MonitorMessageItem;
class MonitorMessageItemDefaultTypeInternal;
extern MonitorMessageItemDefaultTypeInternal _MonitorMessageItem_default_instance_;
}  // namespace monitor
}  // namespace common
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::common::monitor::MonitorMessage* Arena::CreateMaybeMessage<::apollo::common::monitor::MonitorMessage>(Arena*);
template<> ::apollo::common::monitor::MonitorMessageItem* Arena::CreateMaybeMessage<::apollo::common::monitor::MonitorMessageItem>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace common {
namespace monitor {

enum MonitorMessageItem_MessageSource {
  MonitorMessageItem_MessageSource_UNKNOWN = 1,
  MonitorMessageItem_MessageSource_CANBUS = 2,
  MonitorMessageItem_MessageSource_CONTROL = 3,
  MonitorMessageItem_MessageSource_DECISION = 4,
  MonitorMessageItem_MessageSource_LOCALIZATION = 5,
  MonitorMessageItem_MessageSource_PLANNING = 6,
  MonitorMessageItem_MessageSource_PREDICTION = 7,
  MonitorMessageItem_MessageSource_SIMULATOR = 8,
  MonitorMessageItem_MessageSource_HWSYS = 9,
  MonitorMessageItem_MessageSource_ROUTING = 10,
  MonitorMessageItem_MessageSource_MONITOR = 11,
  MonitorMessageItem_MessageSource_HMI = 12,
  MonitorMessageItem_MessageSource_RELATIVE_MAP = 13,
  MonitorMessageItem_MessageSource_GNSS = 14,
  MonitorMessageItem_MessageSource_CONTI_RADAR = 15,
  MonitorMessageItem_MessageSource_RACOBIT_RADAR = 16,
  MonitorMessageItem_MessageSource_ULTRASONIC_RADAR = 17,
  MonitorMessageItem_MessageSource_MOBILEYE = 18,
  MonitorMessageItem_MessageSource_DELPHI_ESR = 19,
  MonitorMessageItem_MessageSource_STORYTELLING = 20,
  MonitorMessageItem_MessageSource_TASK_MANAGER = 21
};
bool MonitorMessageItem_MessageSource_IsValid(int value);
const MonitorMessageItem_MessageSource MonitorMessageItem_MessageSource_MessageSource_MIN = MonitorMessageItem_MessageSource_UNKNOWN;
const MonitorMessageItem_MessageSource MonitorMessageItem_MessageSource_MessageSource_MAX = MonitorMessageItem_MessageSource_TASK_MANAGER;
const int MonitorMessageItem_MessageSource_MessageSource_ARRAYSIZE = MonitorMessageItem_MessageSource_MessageSource_MAX + 1;

const ::google::protobuf::EnumDescriptor* MonitorMessageItem_MessageSource_descriptor();
inline const ::std::string& MonitorMessageItem_MessageSource_Name(MonitorMessageItem_MessageSource value) {
  return ::google::protobuf::internal::NameOfEnum(
    MonitorMessageItem_MessageSource_descriptor(), value);
}
inline bool MonitorMessageItem_MessageSource_Parse(
    const ::std::string& name, MonitorMessageItem_MessageSource* value) {
  return ::google::protobuf::internal::ParseNamedEnum<MonitorMessageItem_MessageSource>(
    MonitorMessageItem_MessageSource_descriptor(), name, value);
}
enum MonitorMessageItem_LogLevel {
  MonitorMessageItem_LogLevel_INFO = 0,
  MonitorMessageItem_LogLevel_WARN = 1,
  MonitorMessageItem_LogLevel_ERROR = 2,
  MonitorMessageItem_LogLevel_FATAL = 3
};
bool MonitorMessageItem_LogLevel_IsValid(int value);
const MonitorMessageItem_LogLevel MonitorMessageItem_LogLevel_LogLevel_MIN = MonitorMessageItem_LogLevel_INFO;
const MonitorMessageItem_LogLevel MonitorMessageItem_LogLevel_LogLevel_MAX = MonitorMessageItem_LogLevel_FATAL;
const int MonitorMessageItem_LogLevel_LogLevel_ARRAYSIZE = MonitorMessageItem_LogLevel_LogLevel_MAX + 1;

const ::google::protobuf::EnumDescriptor* MonitorMessageItem_LogLevel_descriptor();
inline const ::std::string& MonitorMessageItem_LogLevel_Name(MonitorMessageItem_LogLevel value) {
  return ::google::protobuf::internal::NameOfEnum(
    MonitorMessageItem_LogLevel_descriptor(), value);
}
inline bool MonitorMessageItem_LogLevel_Parse(
    const ::std::string& name, MonitorMessageItem_LogLevel* value) {
  return ::google::protobuf::internal::ParseNamedEnum<MonitorMessageItem_LogLevel>(
    MonitorMessageItem_LogLevel_descriptor(), name, value);
}
// ===================================================================

class MonitorMessageItem : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.common.monitor.MonitorMessageItem) */ {
 public:
  MonitorMessageItem();
  virtual ~MonitorMessageItem();

  MonitorMessageItem(const MonitorMessageItem& from);

  inline MonitorMessageItem& operator=(const MonitorMessageItem& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  MonitorMessageItem(MonitorMessageItem&& from) noexcept
    : MonitorMessageItem() {
    *this = ::std::move(from);
  }

  inline MonitorMessageItem& operator=(MonitorMessageItem&& from) noexcept {
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
  static const MonitorMessageItem& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MonitorMessageItem* internal_default_instance() {
    return reinterpret_cast<const MonitorMessageItem*>(
               &_MonitorMessageItem_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(MonitorMessageItem* other);
  friend void swap(MonitorMessageItem& a, MonitorMessageItem& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline MonitorMessageItem* New() const final {
    return CreateMaybeMessage<MonitorMessageItem>(NULL);
  }

  MonitorMessageItem* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<MonitorMessageItem>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const MonitorMessageItem& from);
  void MergeFrom(const MonitorMessageItem& from);
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
  void InternalSwap(MonitorMessageItem* other);
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

  typedef MonitorMessageItem_MessageSource MessageSource;
  static const MessageSource UNKNOWN =
    MonitorMessageItem_MessageSource_UNKNOWN;
  static const MessageSource CANBUS =
    MonitorMessageItem_MessageSource_CANBUS;
  static const MessageSource CONTROL =
    MonitorMessageItem_MessageSource_CONTROL;
  static const MessageSource DECISION =
    MonitorMessageItem_MessageSource_DECISION;
  static const MessageSource LOCALIZATION =
    MonitorMessageItem_MessageSource_LOCALIZATION;
  static const MessageSource PLANNING =
    MonitorMessageItem_MessageSource_PLANNING;
  static const MessageSource PREDICTION =
    MonitorMessageItem_MessageSource_PREDICTION;
  static const MessageSource SIMULATOR =
    MonitorMessageItem_MessageSource_SIMULATOR;
  static const MessageSource HWSYS =
    MonitorMessageItem_MessageSource_HWSYS;
  static const MessageSource ROUTING =
    MonitorMessageItem_MessageSource_ROUTING;
  static const MessageSource MONITOR =
    MonitorMessageItem_MessageSource_MONITOR;
  static const MessageSource HMI =
    MonitorMessageItem_MessageSource_HMI;
  static const MessageSource RELATIVE_MAP =
    MonitorMessageItem_MessageSource_RELATIVE_MAP;
  static const MessageSource GNSS =
    MonitorMessageItem_MessageSource_GNSS;
  static const MessageSource CONTI_RADAR =
    MonitorMessageItem_MessageSource_CONTI_RADAR;
  static const MessageSource RACOBIT_RADAR =
    MonitorMessageItem_MessageSource_RACOBIT_RADAR;
  static const MessageSource ULTRASONIC_RADAR =
    MonitorMessageItem_MessageSource_ULTRASONIC_RADAR;
  static const MessageSource MOBILEYE =
    MonitorMessageItem_MessageSource_MOBILEYE;
  static const MessageSource DELPHI_ESR =
    MonitorMessageItem_MessageSource_DELPHI_ESR;
  static const MessageSource STORYTELLING =
    MonitorMessageItem_MessageSource_STORYTELLING;
  static const MessageSource TASK_MANAGER =
    MonitorMessageItem_MessageSource_TASK_MANAGER;
  static inline bool MessageSource_IsValid(int value) {
    return MonitorMessageItem_MessageSource_IsValid(value);
  }
  static const MessageSource MessageSource_MIN =
    MonitorMessageItem_MessageSource_MessageSource_MIN;
  static const MessageSource MessageSource_MAX =
    MonitorMessageItem_MessageSource_MessageSource_MAX;
  static const int MessageSource_ARRAYSIZE =
    MonitorMessageItem_MessageSource_MessageSource_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  MessageSource_descriptor() {
    return MonitorMessageItem_MessageSource_descriptor();
  }
  static inline const ::std::string& MessageSource_Name(MessageSource value) {
    return MonitorMessageItem_MessageSource_Name(value);
  }
  static inline bool MessageSource_Parse(const ::std::string& name,
      MessageSource* value) {
    return MonitorMessageItem_MessageSource_Parse(name, value);
  }

  typedef MonitorMessageItem_LogLevel LogLevel;
  static const LogLevel INFO =
    MonitorMessageItem_LogLevel_INFO;
  static const LogLevel WARN =
    MonitorMessageItem_LogLevel_WARN;
  static const LogLevel ERROR =
    MonitorMessageItem_LogLevel_ERROR;
  static const LogLevel FATAL =
    MonitorMessageItem_LogLevel_FATAL;
  static inline bool LogLevel_IsValid(int value) {
    return MonitorMessageItem_LogLevel_IsValid(value);
  }
  static const LogLevel LogLevel_MIN =
    MonitorMessageItem_LogLevel_LogLevel_MIN;
  static const LogLevel LogLevel_MAX =
    MonitorMessageItem_LogLevel_LogLevel_MAX;
  static const int LogLevel_ARRAYSIZE =
    MonitorMessageItem_LogLevel_LogLevel_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  LogLevel_descriptor() {
    return MonitorMessageItem_LogLevel_descriptor();
  }
  static inline const ::std::string& LogLevel_Name(LogLevel value) {
    return MonitorMessageItem_LogLevel_Name(value);
  }
  static inline bool LogLevel_Parse(const ::std::string& name,
      LogLevel* value) {
    return MonitorMessageItem_LogLevel_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // optional string msg = 2;
  bool has_msg() const;
  void clear_msg();
  static const int kMsgFieldNumber = 2;
  const ::std::string& msg() const;
  void set_msg(const ::std::string& value);
  #if LANG_CXX11
  void set_msg(::std::string&& value);
  #endif
  void set_msg(const char* value);
  void set_msg(const char* value, size_t size);
  ::std::string* mutable_msg();
  ::std::string* release_msg();
  void set_allocated_msg(::std::string* msg);

  // optional .apollo.common.monitor.MonitorMessageItem.LogLevel log_level = 3 [default = INFO];
  bool has_log_level() const;
  void clear_log_level();
  static const int kLogLevelFieldNumber = 3;
  ::apollo::common::monitor::MonitorMessageItem_LogLevel log_level() const;
  void set_log_level(::apollo::common::monitor::MonitorMessageItem_LogLevel value);

  // optional .apollo.common.monitor.MonitorMessageItem.MessageSource source = 1 [default = UNKNOWN];
  bool has_source() const;
  void clear_source();
  static const int kSourceFieldNumber = 1;
  ::apollo::common::monitor::MonitorMessageItem_MessageSource source() const;
  void set_source(::apollo::common::monitor::MonitorMessageItem_MessageSource value);

  // @@protoc_insertion_point(class_scope:apollo.common.monitor.MonitorMessageItem)
 private:
  void set_has_source();
  void clear_has_source();
  void set_has_msg();
  void clear_has_msg();
  void set_has_log_level();
  void clear_has_log_level();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr msg_;
  int log_level_;
  int source_;
  friend struct ::protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class MonitorMessage : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.common.monitor.MonitorMessage) */ {
 public:
  MonitorMessage();
  virtual ~MonitorMessage();

  MonitorMessage(const MonitorMessage& from);

  inline MonitorMessage& operator=(const MonitorMessage& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  MonitorMessage(MonitorMessage&& from) noexcept
    : MonitorMessage() {
    *this = ::std::move(from);
  }

  inline MonitorMessage& operator=(MonitorMessage&& from) noexcept {
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
  static const MonitorMessage& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MonitorMessage* internal_default_instance() {
    return reinterpret_cast<const MonitorMessage*>(
               &_MonitorMessage_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(MonitorMessage* other);
  friend void swap(MonitorMessage& a, MonitorMessage& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline MonitorMessage* New() const final {
    return CreateMaybeMessage<MonitorMessage>(NULL);
  }

  MonitorMessage* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<MonitorMessage>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const MonitorMessage& from);
  void MergeFrom(const MonitorMessage& from);
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
  void InternalSwap(MonitorMessage* other);
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

  // accessors -------------------------------------------------------

  // repeated .apollo.common.monitor.MonitorMessageItem item = 2;
  int item_size() const;
  void clear_item();
  static const int kItemFieldNumber = 2;
  ::apollo::common::monitor::MonitorMessageItem* mutable_item(int index);
  ::google::protobuf::RepeatedPtrField< ::apollo::common::monitor::MonitorMessageItem >*
      mutable_item();
  const ::apollo::common::monitor::MonitorMessageItem& item(int index) const;
  ::apollo::common::monitor::MonitorMessageItem* add_item();
  const ::google::protobuf::RepeatedPtrField< ::apollo::common::monitor::MonitorMessageItem >&
      item() const;

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

  // @@protoc_insertion_point(class_scope:apollo.common.monitor.MonitorMessage)
 private:
  void set_has_header();
  void clear_has_header();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::apollo::common::monitor::MonitorMessageItem > item_;
  ::apollo::common::Header* header_;
  friend struct ::protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MonitorMessageItem

// optional .apollo.common.monitor.MonitorMessageItem.MessageSource source = 1 [default = UNKNOWN];
inline bool MonitorMessageItem::has_source() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void MonitorMessageItem::set_has_source() {
  _has_bits_[0] |= 0x00000004u;
}
inline void MonitorMessageItem::clear_has_source() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void MonitorMessageItem::clear_source() {
  source_ = 1;
  clear_has_source();
}
inline ::apollo::common::monitor::MonitorMessageItem_MessageSource MonitorMessageItem::source() const {
  // @@protoc_insertion_point(field_get:apollo.common.monitor.MonitorMessageItem.source)
  return static_cast< ::apollo::common::monitor::MonitorMessageItem_MessageSource >(source_);
}
inline void MonitorMessageItem::set_source(::apollo::common::monitor::MonitorMessageItem_MessageSource value) {
  assert(::apollo::common::monitor::MonitorMessageItem_MessageSource_IsValid(value));
  set_has_source();
  source_ = value;
  // @@protoc_insertion_point(field_set:apollo.common.monitor.MonitorMessageItem.source)
}

// optional string msg = 2;
inline bool MonitorMessageItem::has_msg() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void MonitorMessageItem::set_has_msg() {
  _has_bits_[0] |= 0x00000001u;
}
inline void MonitorMessageItem::clear_has_msg() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void MonitorMessageItem::clear_msg() {
  msg_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_msg();
}
inline const ::std::string& MonitorMessageItem::msg() const {
  // @@protoc_insertion_point(field_get:apollo.common.monitor.MonitorMessageItem.msg)
  return msg_.GetNoArena();
}
inline void MonitorMessageItem::set_msg(const ::std::string& value) {
  set_has_msg();
  msg_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:apollo.common.monitor.MonitorMessageItem.msg)
}
#if LANG_CXX11
inline void MonitorMessageItem::set_msg(::std::string&& value) {
  set_has_msg();
  msg_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.common.monitor.MonitorMessageItem.msg)
}
#endif
inline void MonitorMessageItem::set_msg(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_msg();
  msg_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.common.monitor.MonitorMessageItem.msg)
}
inline void MonitorMessageItem::set_msg(const char* value, size_t size) {
  set_has_msg();
  msg_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.common.monitor.MonitorMessageItem.msg)
}
inline ::std::string* MonitorMessageItem::mutable_msg() {
  set_has_msg();
  // @@protoc_insertion_point(field_mutable:apollo.common.monitor.MonitorMessageItem.msg)
  return msg_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* MonitorMessageItem::release_msg() {
  // @@protoc_insertion_point(field_release:apollo.common.monitor.MonitorMessageItem.msg)
  if (!has_msg()) {
    return NULL;
  }
  clear_has_msg();
  return msg_.ReleaseNonDefaultNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void MonitorMessageItem::set_allocated_msg(::std::string* msg) {
  if (msg != NULL) {
    set_has_msg();
  } else {
    clear_has_msg();
  }
  msg_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), msg);
  // @@protoc_insertion_point(field_set_allocated:apollo.common.monitor.MonitorMessageItem.msg)
}

// optional .apollo.common.monitor.MonitorMessageItem.LogLevel log_level = 3 [default = INFO];
inline bool MonitorMessageItem::has_log_level() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void MonitorMessageItem::set_has_log_level() {
  _has_bits_[0] |= 0x00000002u;
}
inline void MonitorMessageItem::clear_has_log_level() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void MonitorMessageItem::clear_log_level() {
  log_level_ = 0;
  clear_has_log_level();
}
inline ::apollo::common::monitor::MonitorMessageItem_LogLevel MonitorMessageItem::log_level() const {
  // @@protoc_insertion_point(field_get:apollo.common.monitor.MonitorMessageItem.log_level)
  return static_cast< ::apollo::common::monitor::MonitorMessageItem_LogLevel >(log_level_);
}
inline void MonitorMessageItem::set_log_level(::apollo::common::monitor::MonitorMessageItem_LogLevel value) {
  assert(::apollo::common::monitor::MonitorMessageItem_LogLevel_IsValid(value));
  set_has_log_level();
  log_level_ = value;
  // @@protoc_insertion_point(field_set:apollo.common.monitor.MonitorMessageItem.log_level)
}

// -------------------------------------------------------------------

// MonitorMessage

// optional .apollo.common.Header header = 1;
inline bool MonitorMessage::has_header() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void MonitorMessage::set_has_header() {
  _has_bits_[0] |= 0x00000001u;
}
inline void MonitorMessage::clear_has_header() {
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::apollo::common::Header& MonitorMessage::_internal_header() const {
  return *header_;
}
inline const ::apollo::common::Header& MonitorMessage::header() const {
  const ::apollo::common::Header* p = header_;
  // @@protoc_insertion_point(field_get:apollo.common.monitor.MonitorMessage.header)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::common::Header*>(
      &::apollo::common::_Header_default_instance_);
}
inline ::apollo::common::Header* MonitorMessage::release_header() {
  // @@protoc_insertion_point(field_release:apollo.common.monitor.MonitorMessage.header)
  clear_has_header();
  ::apollo::common::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::apollo::common::Header* MonitorMessage::mutable_header() {
  set_has_header();
  if (header_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::common::Header>(GetArenaNoVirtual());
    header_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.common.monitor.MonitorMessage.header)
  return header_;
}
inline void MonitorMessage::set_allocated_header(::apollo::common::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:apollo.common.monitor.MonitorMessage.header)
}

// repeated .apollo.common.monitor.MonitorMessageItem item = 2;
inline int MonitorMessage::item_size() const {
  return item_.size();
}
inline void MonitorMessage::clear_item() {
  item_.Clear();
}
inline ::apollo::common::monitor::MonitorMessageItem* MonitorMessage::mutable_item(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.common.monitor.MonitorMessage.item)
  return item_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::common::monitor::MonitorMessageItem >*
MonitorMessage::mutable_item() {
  // @@protoc_insertion_point(field_mutable_list:apollo.common.monitor.MonitorMessage.item)
  return &item_;
}
inline const ::apollo::common::monitor::MonitorMessageItem& MonitorMessage::item(int index) const {
  // @@protoc_insertion_point(field_get:apollo.common.monitor.MonitorMessage.item)
  return item_.Get(index);
}
inline ::apollo::common::monitor::MonitorMessageItem* MonitorMessage::add_item() {
  // @@protoc_insertion_point(field_add:apollo.common.monitor.MonitorMessage.item)
  return item_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::common::monitor::MonitorMessageItem >&
MonitorMessage::item() const {
  // @@protoc_insertion_point(field_list:apollo.common.monitor.MonitorMessage.item)
  return item_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace monitor
}  // namespace common
}  // namespace apollo

namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::apollo::common::monitor::MonitorMessageItem_MessageSource> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::common::monitor::MonitorMessageItem_MessageSource>() {
  return ::apollo::common::monitor::MonitorMessageItem_MessageSource_descriptor();
}
template <> struct is_proto_enum< ::apollo::common::monitor::MonitorMessageItem_LogLevel> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::common::monitor::MonitorMessageItem_LogLevel>() {
  return ::apollo::common::monitor::MonitorMessageItem_LogLevel_descriptor();
}

}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto
