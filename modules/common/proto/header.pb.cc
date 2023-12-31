// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/common/proto/header.proto

#include "modules/common/proto/header.pb.h"

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

namespace protobuf_modules_2fcommon_2fproto_2ferror_5fcode_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fcommon_2fproto_2ferror_5fcode_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_StatusPb;
}  // namespace protobuf_modules_2fcommon_2fproto_2ferror_5fcode_2eproto
namespace apollo {
namespace common {
class HeaderDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<Header>
      _instance;
} _Header_default_instance_;
}  // namespace common
}  // namespace apollo
namespace protobuf_modules_2fcommon_2fproto_2fheader_2eproto {
static void InitDefaultsHeader() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::common::_Header_default_instance_;
    new (ptr) ::apollo::common::Header();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::common::Header::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<1> scc_info_Header =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 1, InitDefaultsHeader}, {
      &protobuf_modules_2fcommon_2fproto_2ferror_5fcode_2eproto::scc_info_StatusPb.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_Header.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::Header, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::Header, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::Header, timestamp_sec_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::Header, module_name_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::Header, sequence_num_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::Header, lidar_timestamp_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::Header, camera_timestamp_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::Header, radar_timestamp_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::Header, version_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::Header, status_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::Header, frame_id_),
  3,
  0,
  7,
  4,
  5,
  6,
  8,
  2,
  1,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 14, sizeof(::apollo::common::Header)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::common::_Header_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "modules/common/proto/header.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n!modules/common/proto/header.proto\022\rapo"
      "llo.common\032%modules/common/proto/error_c"
      "ode.proto\"\345\001\n\006Header\022\025\n\rtimestamp_sec\030\001 "
      "\001(\001\022\023\n\013module_name\030\002 \001(\t\022\024\n\014sequence_num"
      "\030\003 \001(\r\022\027\n\017lidar_timestamp\030\004 \001(\004\022\030\n\020camer"
      "a_timestamp\030\005 \001(\004\022\027\n\017radar_timestamp\030\006 \001"
      "(\004\022\022\n\007version\030\007 \001(\r:\0011\022\'\n\006status\030\010 \001(\0132\027"
      ".apollo.common.StatusPb\022\020\n\010frame_id\030\t \001("
      "\t"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 321);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "modules/common/proto/header.proto", &protobuf_RegisterTypes);
  ::protobuf_modules_2fcommon_2fproto_2ferror_5fcode_2eproto::AddDescriptors();
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
}  // namespace protobuf_modules_2fcommon_2fproto_2fheader_2eproto
namespace apollo {
namespace common {

// ===================================================================

void Header::InitAsDefaultInstance() {
  ::apollo::common::_Header_default_instance_._instance.get_mutable()->status_ = const_cast< ::apollo::common::StatusPb*>(
      ::apollo::common::StatusPb::internal_default_instance());
}
void Header::clear_status() {
  if (status_ != NULL) status_->Clear();
  clear_has_status();
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Header::kTimestampSecFieldNumber;
const int Header::kModuleNameFieldNumber;
const int Header::kSequenceNumFieldNumber;
const int Header::kLidarTimestampFieldNumber;
const int Header::kCameraTimestampFieldNumber;
const int Header::kRadarTimestampFieldNumber;
const int Header::kVersionFieldNumber;
const int Header::kStatusFieldNumber;
const int Header::kFrameIdFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Header::Header()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fcommon_2fproto_2fheader_2eproto::scc_info_Header.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.common.Header)
}
Header::Header(const Header& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  module_name_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_module_name()) {
    module_name_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.module_name_);
  }
  frame_id_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_frame_id()) {
    frame_id_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.frame_id_);
  }
  if (from.has_status()) {
    status_ = new ::apollo::common::StatusPb(*from.status_);
  } else {
    status_ = NULL;
  }
  ::memcpy(&timestamp_sec_, &from.timestamp_sec_,
    static_cast<size_t>(reinterpret_cast<char*>(&version_) -
    reinterpret_cast<char*>(&timestamp_sec_)) + sizeof(version_));
  // @@protoc_insertion_point(copy_constructor:apollo.common.Header)
}

void Header::SharedCtor() {
  module_name_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  frame_id_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  ::memset(&status_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&sequence_num_) -
      reinterpret_cast<char*>(&status_)) + sizeof(sequence_num_));
  version_ = 1u;
}

Header::~Header() {
  // @@protoc_insertion_point(destructor:apollo.common.Header)
  SharedDtor();
}

void Header::SharedDtor() {
  module_name_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  frame_id_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (this != internal_default_instance()) delete status_;
}

void Header::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* Header::descriptor() {
  ::protobuf_modules_2fcommon_2fproto_2fheader_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fcommon_2fproto_2fheader_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const Header& Header::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fcommon_2fproto_2fheader_2eproto::scc_info_Header.base);
  return *internal_default_instance();
}


void Header::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.common.Header)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 7u) {
    if (cached_has_bits & 0x00000001u) {
      module_name_.ClearNonDefaultToEmptyNoArena();
    }
    if (cached_has_bits & 0x00000002u) {
      frame_id_.ClearNonDefaultToEmptyNoArena();
    }
    if (cached_has_bits & 0x00000004u) {
      GOOGLE_DCHECK(status_ != NULL);
      status_->Clear();
    }
  }
  if (cached_has_bits & 248u) {
    ::memset(&timestamp_sec_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&sequence_num_) -
        reinterpret_cast<char*>(&timestamp_sec_)) + sizeof(sequence_num_));
  }
  version_ = 1u;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool Header::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.common.Header)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional double timestamp_sec = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {
          set_has_timestamp_sec();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &timestamp_sec_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional string module_name = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_module_name()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->module_name().data(), static_cast<int>(this->module_name().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "apollo.common.Header.module_name");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional uint32 sequence_num = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(24u /* 24 & 0xFF */)) {
          set_has_sequence_num();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &sequence_num_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional uint64 lidar_timestamp = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(32u /* 32 & 0xFF */)) {
          set_has_lidar_timestamp();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint64, ::google::protobuf::internal::WireFormatLite::TYPE_UINT64>(
                 input, &lidar_timestamp_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional uint64 camera_timestamp = 5;
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(40u /* 40 & 0xFF */)) {
          set_has_camera_timestamp();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint64, ::google::protobuf::internal::WireFormatLite::TYPE_UINT64>(
                 input, &camera_timestamp_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional uint64 radar_timestamp = 6;
      case 6: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(48u /* 48 & 0xFF */)) {
          set_has_radar_timestamp();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint64, ::google::protobuf::internal::WireFormatLite::TYPE_UINT64>(
                 input, &radar_timestamp_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional uint32 version = 7 [default = 1];
      case 7: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(56u /* 56 & 0xFF */)) {
          set_has_version();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &version_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.common.StatusPb status = 8;
      case 8: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(66u /* 66 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_status()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional string frame_id = 9;
      case 9: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(74u /* 74 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_frame_id()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->frame_id().data(), static_cast<int>(this->frame_id().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "apollo.common.Header.frame_id");
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
  // @@protoc_insertion_point(parse_success:apollo.common.Header)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.common.Header)
  return false;
#undef DO_
}

void Header::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.common.Header)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double timestamp_sec = 1;
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->timestamp_sec(), output);
  }

  // optional string module_name = 2;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->module_name().data(), static_cast<int>(this->module_name().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.common.Header.module_name");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      2, this->module_name(), output);
  }

  // optional uint32 sequence_num = 3;
  if (cached_has_bits & 0x00000080u) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(3, this->sequence_num(), output);
  }

  // optional uint64 lidar_timestamp = 4;
  if (cached_has_bits & 0x00000010u) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt64(4, this->lidar_timestamp(), output);
  }

  // optional uint64 camera_timestamp = 5;
  if (cached_has_bits & 0x00000020u) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt64(5, this->camera_timestamp(), output);
  }

  // optional uint64 radar_timestamp = 6;
  if (cached_has_bits & 0x00000040u) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt64(6, this->radar_timestamp(), output);
  }

  // optional uint32 version = 7 [default = 1];
  if (cached_has_bits & 0x00000100u) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(7, this->version(), output);
  }

  // optional .apollo.common.StatusPb status = 8;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      8, this->_internal_status(), output);
  }

  // optional string frame_id = 9;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->frame_id().data(), static_cast<int>(this->frame_id().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.common.Header.frame_id");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      9, this->frame_id(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.common.Header)
}

::google::protobuf::uint8* Header::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.common.Header)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double timestamp_sec = 1;
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->timestamp_sec(), target);
  }

  // optional string module_name = 2;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->module_name().data(), static_cast<int>(this->module_name().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.common.Header.module_name");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        2, this->module_name(), target);
  }

  // optional uint32 sequence_num = 3;
  if (cached_has_bits & 0x00000080u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(3, this->sequence_num(), target);
  }

  // optional uint64 lidar_timestamp = 4;
  if (cached_has_bits & 0x00000010u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt64ToArray(4, this->lidar_timestamp(), target);
  }

  // optional uint64 camera_timestamp = 5;
  if (cached_has_bits & 0x00000020u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt64ToArray(5, this->camera_timestamp(), target);
  }

  // optional uint64 radar_timestamp = 6;
  if (cached_has_bits & 0x00000040u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt64ToArray(6, this->radar_timestamp(), target);
  }

  // optional uint32 version = 7 [default = 1];
  if (cached_has_bits & 0x00000100u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(7, this->version(), target);
  }

  // optional .apollo.common.StatusPb status = 8;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        8, this->_internal_status(), deterministic, target);
  }

  // optional string frame_id = 9;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->frame_id().data(), static_cast<int>(this->frame_id().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.common.Header.frame_id");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        9, this->frame_id(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.common.Header)
  return target;
}

size_t Header::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.common.Header)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 255u) {
    // optional string module_name = 2;
    if (has_module_name()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->module_name());
    }

    // optional string frame_id = 9;
    if (has_frame_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->frame_id());
    }

    // optional .apollo.common.StatusPb status = 8;
    if (has_status()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *status_);
    }

    // optional double timestamp_sec = 1;
    if (has_timestamp_sec()) {
      total_size += 1 + 8;
    }

    // optional uint64 lidar_timestamp = 4;
    if (has_lidar_timestamp()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt64Size(
          this->lidar_timestamp());
    }

    // optional uint64 camera_timestamp = 5;
    if (has_camera_timestamp()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt64Size(
          this->camera_timestamp());
    }

    // optional uint64 radar_timestamp = 6;
    if (has_radar_timestamp()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt64Size(
          this->radar_timestamp());
    }

    // optional uint32 sequence_num = 3;
    if (has_sequence_num()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->sequence_num());
    }

  }
  // optional uint32 version = 7 [default = 1];
  if (has_version()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::UInt32Size(
        this->version());
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Header::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.common.Header)
  GOOGLE_DCHECK_NE(&from, this);
  const Header* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const Header>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.common.Header)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.common.Header)
    MergeFrom(*source);
  }
}

void Header::MergeFrom(const Header& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.common.Header)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 255u) {
    if (cached_has_bits & 0x00000001u) {
      set_has_module_name();
      module_name_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.module_name_);
    }
    if (cached_has_bits & 0x00000002u) {
      set_has_frame_id();
      frame_id_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.frame_id_);
    }
    if (cached_has_bits & 0x00000004u) {
      mutable_status()->::apollo::common::StatusPb::MergeFrom(from.status());
    }
    if (cached_has_bits & 0x00000008u) {
      timestamp_sec_ = from.timestamp_sec_;
    }
    if (cached_has_bits & 0x00000010u) {
      lidar_timestamp_ = from.lidar_timestamp_;
    }
    if (cached_has_bits & 0x00000020u) {
      camera_timestamp_ = from.camera_timestamp_;
    }
    if (cached_has_bits & 0x00000040u) {
      radar_timestamp_ = from.radar_timestamp_;
    }
    if (cached_has_bits & 0x00000080u) {
      sequence_num_ = from.sequence_num_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  if (cached_has_bits & 0x00000100u) {
    set_version(from.version());
  }
}

void Header::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.common.Header)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Header::CopyFrom(const Header& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.common.Header)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Header::IsInitialized() const {
  return true;
}

void Header::Swap(Header* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Header::InternalSwap(Header* other) {
  using std::swap;
  module_name_.Swap(&other->module_name_, &::google::protobuf::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  frame_id_.Swap(&other->frame_id_, &::google::protobuf::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  swap(status_, other->status_);
  swap(timestamp_sec_, other->timestamp_sec_);
  swap(lidar_timestamp_, other->lidar_timestamp_);
  swap(camera_timestamp_, other->camera_timestamp_);
  swap(radar_timestamp_, other->radar_timestamp_);
  swap(sequence_num_, other->sequence_num_);
  swap(version_, other->version_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata Header::GetMetadata() const {
  protobuf_modules_2fcommon_2fproto_2fheader_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fcommon_2fproto_2fheader_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace common
}  // namespace apollo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::common::Header* Arena::CreateMaybeMessage< ::apollo::common::Header >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::common::Header >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
