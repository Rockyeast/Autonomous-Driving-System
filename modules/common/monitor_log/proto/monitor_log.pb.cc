// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/common/monitor_log/proto/monitor_log.proto

#include "modules/common/monitor_log/proto/monitor_log.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)

namespace protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_MonitorMessageItem;
}  // namespace protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto
namespace protobuf_modules_2fcommon_2fproto_2fheader_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fcommon_2fproto_2fheader_2eproto ::google::protobuf::internal::SCCInfo<1> scc_info_Header;
}  // namespace protobuf_modules_2fcommon_2fproto_2fheader_2eproto
namespace apollo {
namespace common {
namespace monitor {
class MonitorMessageItemDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<MonitorMessageItem>
      _instance;
} _MonitorMessageItem_default_instance_;
class MonitorMessageDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<MonitorMessage>
      _instance;
} _MonitorMessage_default_instance_;
}  // namespace monitor
}  // namespace common
}  // namespace apollo
namespace protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto {
static void InitDefaultsMonitorMessageItem() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::common::monitor::_MonitorMessageItem_default_instance_;
    new (ptr) ::apollo::common::monitor::MonitorMessageItem();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::common::monitor::MonitorMessageItem::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_MonitorMessageItem =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsMonitorMessageItem}, {}};

static void InitDefaultsMonitorMessage() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::common::monitor::_MonitorMessage_default_instance_;
    new (ptr) ::apollo::common::monitor::MonitorMessage();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::common::monitor::MonitorMessage::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<2> scc_info_MonitorMessage =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 2, InitDefaultsMonitorMessage}, {
      &protobuf_modules_2fcommon_2fproto_2fheader_2eproto::scc_info_Header.base,
      &protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto::scc_info_MonitorMessageItem.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_MonitorMessageItem.base);
  ::google::protobuf::internal::InitSCC(&scc_info_MonitorMessage.base);
}

::google::protobuf::Metadata file_level_metadata[2];
const ::google::protobuf::EnumDescriptor* file_level_enum_descriptors[2];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::monitor::MonitorMessageItem, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::monitor::MonitorMessageItem, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::monitor::MonitorMessageItem, source_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::monitor::MonitorMessageItem, msg_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::monitor::MonitorMessageItem, log_level_),
  2,
  0,
  1,
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::monitor::MonitorMessage, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::monitor::MonitorMessage, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::monitor::MonitorMessage, header_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::monitor::MonitorMessage, item_),
  0,
  ~0u,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::apollo::common::monitor::MonitorMessageItem)},
  { 11, 18, sizeof(::apollo::common::monitor::MonitorMessage)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::common::monitor::_MonitorMessageItem_default_instance_),
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::common::monitor::_MonitorMessage_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "modules/common/monitor_log/proto/monitor_log.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, file_level_enum_descriptors, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 2);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n2modules/common/monitor_log/proto/monit"
      "or_log.proto\022\025apollo.common.monitor\032!mod"
      "ules/common/proto/header.proto\"\301\004\n\022Monit"
      "orMessageItem\022P\n\006source\030\001 \001(\01627.apollo.c"
      "ommon.monitor.MonitorMessageItem.Message"
      "Source:\007UNKNOWN\022\013\n\003msg\030\002 \001(\t\022K\n\tlog_leve"
      "l\030\003 \001(\01622.apollo.common.monitor.MonitorM"
      "essageItem.LogLevel:\004INFO\"\310\002\n\rMessageSou"
      "rce\022\013\n\007UNKNOWN\020\001\022\n\n\006CANBUS\020\002\022\013\n\007CONTROL\020"
      "\003\022\014\n\010DECISION\020\004\022\020\n\014LOCALIZATION\020\005\022\014\n\010PLA"
      "NNING\020\006\022\016\n\nPREDICTION\020\007\022\r\n\tSIMULATOR\020\010\022\t"
      "\n\005HWSYS\020\t\022\013\n\007ROUTING\020\n\022\013\n\007MONITOR\020\013\022\007\n\003H"
      "MI\020\014\022\020\n\014RELATIVE_MAP\020\r\022\010\n\004GNSS\020\016\022\017\n\013CONT"
      "I_RADAR\020\017\022\021\n\rRACOBIT_RADAR\020\020\022\024\n\020ULTRASON"
      "IC_RADAR\020\021\022\014\n\010MOBILEYE\020\022\022\016\n\nDELPHI_ESR\020\023"
      "\022\020\n\014STORYTELLING\020\024\022\020\n\014TASK_MANAGER\020\025\"4\n\010"
      "LogLevel\022\010\n\004INFO\020\000\022\010\n\004WARN\020\001\022\t\n\005ERROR\020\002\022"
      "\t\n\005FATAL\020\003\"p\n\016MonitorMessage\022%\n\006header\030\001"
      " \001(\0132\025.apollo.common.Header\0227\n\004item\030\002 \003("
      "\0132).apollo.common.monitor.MonitorMessage"
      "Item"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 804);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "modules/common/monitor_log/proto/monitor_log.proto", &protobuf_RegisterTypes);
  ::protobuf_modules_2fcommon_2fproto_2fheader_2eproto::AddDescriptors();
}

void AddDescriptors() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto
namespace apollo {
namespace common {
namespace monitor {
const ::google::protobuf::EnumDescriptor* MonitorMessageItem_MessageSource_descriptor() {
  protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto::file_level_enum_descriptors[0];
}
bool MonitorMessageItem_MessageSource_IsValid(int value) {
  switch (value) {
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
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
    case 18:
    case 19:
    case 20:
    case 21:
      return true;
    default:
      return false;
  }
}

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const MonitorMessageItem_MessageSource MonitorMessageItem::UNKNOWN;
const MonitorMessageItem_MessageSource MonitorMessageItem::CANBUS;
const MonitorMessageItem_MessageSource MonitorMessageItem::CONTROL;
const MonitorMessageItem_MessageSource MonitorMessageItem::DECISION;
const MonitorMessageItem_MessageSource MonitorMessageItem::LOCALIZATION;
const MonitorMessageItem_MessageSource MonitorMessageItem::PLANNING;
const MonitorMessageItem_MessageSource MonitorMessageItem::PREDICTION;
const MonitorMessageItem_MessageSource MonitorMessageItem::SIMULATOR;
const MonitorMessageItem_MessageSource MonitorMessageItem::HWSYS;
const MonitorMessageItem_MessageSource MonitorMessageItem::ROUTING;
const MonitorMessageItem_MessageSource MonitorMessageItem::MONITOR;
const MonitorMessageItem_MessageSource MonitorMessageItem::HMI;
const MonitorMessageItem_MessageSource MonitorMessageItem::RELATIVE_MAP;
const MonitorMessageItem_MessageSource MonitorMessageItem::GNSS;
const MonitorMessageItem_MessageSource MonitorMessageItem::CONTI_RADAR;
const MonitorMessageItem_MessageSource MonitorMessageItem::RACOBIT_RADAR;
const MonitorMessageItem_MessageSource MonitorMessageItem::ULTRASONIC_RADAR;
const MonitorMessageItem_MessageSource MonitorMessageItem::MOBILEYE;
const MonitorMessageItem_MessageSource MonitorMessageItem::DELPHI_ESR;
const MonitorMessageItem_MessageSource MonitorMessageItem::STORYTELLING;
const MonitorMessageItem_MessageSource MonitorMessageItem::TASK_MANAGER;
const MonitorMessageItem_MessageSource MonitorMessageItem::MessageSource_MIN;
const MonitorMessageItem_MessageSource MonitorMessageItem::MessageSource_MAX;
const int MonitorMessageItem::MessageSource_ARRAYSIZE;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900
const ::google::protobuf::EnumDescriptor* MonitorMessageItem_LogLevel_descriptor() {
  protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto::file_level_enum_descriptors[1];
}
bool MonitorMessageItem_LogLevel_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
      return true;
    default:
      return false;
  }
}

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const MonitorMessageItem_LogLevel MonitorMessageItem::INFO;
const MonitorMessageItem_LogLevel MonitorMessageItem::WARN;
const MonitorMessageItem_LogLevel MonitorMessageItem::ERROR;
const MonitorMessageItem_LogLevel MonitorMessageItem::FATAL;
const MonitorMessageItem_LogLevel MonitorMessageItem::LogLevel_MIN;
const MonitorMessageItem_LogLevel MonitorMessageItem::LogLevel_MAX;
const int MonitorMessageItem::LogLevel_ARRAYSIZE;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

// ===================================================================

void MonitorMessageItem::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int MonitorMessageItem::kSourceFieldNumber;
const int MonitorMessageItem::kMsgFieldNumber;
const int MonitorMessageItem::kLogLevelFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

MonitorMessageItem::MonitorMessageItem()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto::scc_info_MonitorMessageItem.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.common.monitor.MonitorMessageItem)
}
MonitorMessageItem::MonitorMessageItem(const MonitorMessageItem& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  msg_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_msg()) {
    msg_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.msg_);
  }
  ::memcpy(&log_level_, &from.log_level_,
    static_cast<size_t>(reinterpret_cast<char*>(&source_) -
    reinterpret_cast<char*>(&log_level_)) + sizeof(source_));
  // @@protoc_insertion_point(copy_constructor:apollo.common.monitor.MonitorMessageItem)
}

void MonitorMessageItem::SharedCtor() {
  msg_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  log_level_ = 0;
  source_ = 1;
}

MonitorMessageItem::~MonitorMessageItem() {
  // @@protoc_insertion_point(destructor:apollo.common.monitor.MonitorMessageItem)
  SharedDtor();
}

void MonitorMessageItem::SharedDtor() {
  msg_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}

void MonitorMessageItem::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* MonitorMessageItem::descriptor() {
  ::protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const MonitorMessageItem& MonitorMessageItem::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto::scc_info_MonitorMessageItem.base);
  return *internal_default_instance();
}


void MonitorMessageItem::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.common.monitor.MonitorMessageItem)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    msg_.ClearNonDefaultToEmptyNoArena();
  }
  if (cached_has_bits & 6u) {
    log_level_ = 0;
    source_ = 1;
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool MonitorMessageItem::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.common.monitor.MonitorMessageItem)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .apollo.common.monitor.MonitorMessageItem.MessageSource source = 1 [default = UNKNOWN];
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(8u /* 8 & 0xFF */)) {
          int value;
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   int, ::google::protobuf::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          if (::apollo::common::monitor::MonitorMessageItem_MessageSource_IsValid(value)) {
            set_source(static_cast< ::apollo::common::monitor::MonitorMessageItem_MessageSource >(value));
          } else {
            mutable_unknown_fields()->AddVarint(
                1, static_cast< ::google::protobuf::uint64>(value));
          }
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional string msg = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_msg()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->msg().data(), static_cast<int>(this->msg().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "apollo.common.monitor.MonitorMessageItem.msg");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.common.monitor.MonitorMessageItem.LogLevel log_level = 3 [default = INFO];
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(24u /* 24 & 0xFF */)) {
          int value;
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   int, ::google::protobuf::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          if (::apollo::common::monitor::MonitorMessageItem_LogLevel_IsValid(value)) {
            set_log_level(static_cast< ::apollo::common::monitor::MonitorMessageItem_LogLevel >(value));
          } else {
            mutable_unknown_fields()->AddVarint(
                3, static_cast< ::google::protobuf::uint64>(value));
          }
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:apollo.common.monitor.MonitorMessageItem)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.common.monitor.MonitorMessageItem)
  return false;
#undef DO_
}

void MonitorMessageItem::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.common.monitor.MonitorMessageItem)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.monitor.MonitorMessageItem.MessageSource source = 1 [default = UNKNOWN];
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteEnum(
      1, this->source(), output);
  }

  // optional string msg = 2;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->msg().data(), static_cast<int>(this->msg().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.common.monitor.MonitorMessageItem.msg");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      2, this->msg(), output);
  }

  // optional .apollo.common.monitor.MonitorMessageItem.LogLevel log_level = 3 [default = INFO];
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteEnum(
      3, this->log_level(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.common.monitor.MonitorMessageItem)
}

::google::protobuf::uint8* MonitorMessageItem::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.common.monitor.MonitorMessageItem)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.monitor.MonitorMessageItem.MessageSource source = 1 [default = UNKNOWN];
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteEnumToArray(
      1, this->source(), target);
  }

  // optional string msg = 2;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->msg().data(), static_cast<int>(this->msg().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.common.monitor.MonitorMessageItem.msg");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        2, this->msg(), target);
  }

  // optional .apollo.common.monitor.MonitorMessageItem.LogLevel log_level = 3 [default = INFO];
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteEnumToArray(
      3, this->log_level(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.common.monitor.MonitorMessageItem)
  return target;
}

size_t MonitorMessageItem::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.common.monitor.MonitorMessageItem)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 7u) {
    // optional string msg = 2;
    if (has_msg()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->msg());
    }

    // optional .apollo.common.monitor.MonitorMessageItem.LogLevel log_level = 3 [default = INFO];
    if (has_log_level()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::EnumSize(this->log_level());
    }

    // optional .apollo.common.monitor.MonitorMessageItem.MessageSource source = 1 [default = UNKNOWN];
    if (has_source()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::EnumSize(this->source());
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void MonitorMessageItem::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.common.monitor.MonitorMessageItem)
  GOOGLE_DCHECK_NE(&from, this);
  const MonitorMessageItem* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const MonitorMessageItem>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.common.monitor.MonitorMessageItem)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.common.monitor.MonitorMessageItem)
    MergeFrom(*source);
  }
}

void MonitorMessageItem::MergeFrom(const MonitorMessageItem& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.common.monitor.MonitorMessageItem)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 7u) {
    if (cached_has_bits & 0x00000001u) {
      set_has_msg();
      msg_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.msg_);
    }
    if (cached_has_bits & 0x00000002u) {
      log_level_ = from.log_level_;
    }
    if (cached_has_bits & 0x00000004u) {
      source_ = from.source_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void MonitorMessageItem::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.common.monitor.MonitorMessageItem)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void MonitorMessageItem::CopyFrom(const MonitorMessageItem& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.common.monitor.MonitorMessageItem)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool MonitorMessageItem::IsInitialized() const {
  return true;
}

void MonitorMessageItem::Swap(MonitorMessageItem* other) {
  if (other == this) return;
  InternalSwap(other);
}
void MonitorMessageItem::InternalSwap(MonitorMessageItem* other) {
  using std::swap;
  msg_.Swap(&other->msg_, &::google::protobuf::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  swap(log_level_, other->log_level_);
  swap(source_, other->source_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata MonitorMessageItem::GetMetadata() const {
  protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto::file_level_metadata[kIndexInFileMessages];
}


// ===================================================================

void MonitorMessage::InitAsDefaultInstance() {
  ::apollo::common::monitor::_MonitorMessage_default_instance_._instance.get_mutable()->header_ = const_cast< ::apollo::common::Header*>(
      ::apollo::common::Header::internal_default_instance());
}
void MonitorMessage::clear_header() {
  if (header_ != NULL) header_->Clear();
  clear_has_header();
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int MonitorMessage::kHeaderFieldNumber;
const int MonitorMessage::kItemFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

MonitorMessage::MonitorMessage()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto::scc_info_MonitorMessage.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.common.monitor.MonitorMessage)
}
MonitorMessage::MonitorMessage(const MonitorMessage& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      item_(from.item_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_header()) {
    header_ = new ::apollo::common::Header(*from.header_);
  } else {
    header_ = NULL;
  }
  // @@protoc_insertion_point(copy_constructor:apollo.common.monitor.MonitorMessage)
}

void MonitorMessage::SharedCtor() {
  header_ = NULL;
}

MonitorMessage::~MonitorMessage() {
  // @@protoc_insertion_point(destructor:apollo.common.monitor.MonitorMessage)
  SharedDtor();
}

void MonitorMessage::SharedDtor() {
  if (this != internal_default_instance()) delete header_;
}

void MonitorMessage::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* MonitorMessage::descriptor() {
  ::protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const MonitorMessage& MonitorMessage::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto::scc_info_MonitorMessage.base);
  return *internal_default_instance();
}


void MonitorMessage::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.common.monitor.MonitorMessage)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  item_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(header_ != NULL);
    header_->Clear();
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool MonitorMessage::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.common.monitor.MonitorMessage)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .apollo.common.Header header = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_header()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated .apollo.common.monitor.MonitorMessageItem item = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
                input, add_item()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:apollo.common.monitor.MonitorMessage)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.common.monitor.MonitorMessage)
  return false;
#undef DO_
}

void MonitorMessage::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.common.monitor.MonitorMessage)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.Header header = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->_internal_header(), output);
  }

  // repeated .apollo.common.monitor.MonitorMessageItem item = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->item_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2,
      this->item(static_cast<int>(i)),
      output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.common.monitor.MonitorMessage)
}

::google::protobuf::uint8* MonitorMessage::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.common.monitor.MonitorMessage)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.Header header = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->_internal_header(), deterministic, target);
  }

  // repeated .apollo.common.monitor.MonitorMessageItem item = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->item_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        2, this->item(static_cast<int>(i)), deterministic, target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.common.monitor.MonitorMessage)
  return target;
}

size_t MonitorMessage::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.common.monitor.MonitorMessage)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  // repeated .apollo.common.monitor.MonitorMessageItem item = 2;
  {
    unsigned int count = static_cast<unsigned int>(this->item_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->item(static_cast<int>(i)));
    }
  }

  // optional .apollo.common.Header header = 1;
  if (has_header()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *header_);
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void MonitorMessage::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.common.monitor.MonitorMessage)
  GOOGLE_DCHECK_NE(&from, this);
  const MonitorMessage* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const MonitorMessage>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.common.monitor.MonitorMessage)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.common.monitor.MonitorMessage)
    MergeFrom(*source);
  }
}

void MonitorMessage::MergeFrom(const MonitorMessage& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.common.monitor.MonitorMessage)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  item_.MergeFrom(from.item_);
  if (from.has_header()) {
    mutable_header()->::apollo::common::Header::MergeFrom(from.header());
  }
}

void MonitorMessage::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.common.monitor.MonitorMessage)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void MonitorMessage::CopyFrom(const MonitorMessage& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.common.monitor.MonitorMessage)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool MonitorMessage::IsInitialized() const {
  return true;
}

void MonitorMessage::Swap(MonitorMessage* other) {
  if (other == this) return;
  InternalSwap(other);
}
void MonitorMessage::InternalSwap(MonitorMessage* other) {
  using std::swap;
  CastToBase(&item_)->InternalSwap(CastToBase(&other->item_));
  swap(header_, other->header_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata MonitorMessage::GetMetadata() const {
  protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fcommon_2fmonitor_5flog_2fproto_2fmonitor_5flog_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace monitor
}  // namespace common
}  // namespace apollo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::common::monitor::MonitorMessageItem* Arena::CreateMaybeMessage< ::apollo::common::monitor::MonitorMessageItem >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::common::monitor::MonitorMessageItem >(arena);
}
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::common::monitor::MonitorMessage* Arena::CreateMaybeMessage< ::apollo::common::monitor::MonitorMessage >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::common::monitor::MonitorMessage >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
