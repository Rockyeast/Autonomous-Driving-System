// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/proto/pbf_tracker_config.proto

#include "modules/perception/proto/pbf_tracker_config.pb.h"

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

namespace apollo {
namespace perception {
namespace fusion {
class PbfTrackerConfigDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<PbfTrackerConfig>
      _instance;
} _PbfTrackerConfig_default_instance_;
}  // namespace fusion
}  // namespace perception
}  // namespace apollo
namespace protobuf_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto {
static void InitDefaultsPbfTrackerConfig() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.DefaultConstruct();
  *::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get_mutable() = ::std::string("DstTypeFusion", 13);
  ::google::protobuf::internal::OnShutdownDestroyString(
      ::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get_mutable());
  ::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.DefaultConstruct();
  *::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get_mutable() = ::std::string("KalmanMotionFusion", 18);
  ::google::protobuf::internal::OnShutdownDestroyString(
      ::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get_mutable());
  ::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.DefaultConstruct();
  *::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get_mutable() = ::std::string("PbfShapeFusion", 14);
  ::google::protobuf::internal::OnShutdownDestroyString(
      ::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get_mutable());
  ::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existence_fusion_method_.DefaultConstruct();
  *::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existence_fusion_method_.get_mutable() = ::std::string("DstExistenceFusion", 18);
  ::google::protobuf::internal::OnShutdownDestroyString(
      ::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existence_fusion_method_.get_mutable());
  {
    void* ptr = &::apollo::perception::fusion::_PbfTrackerConfig_default_instance_;
    new (ptr) ::apollo::perception::fusion::PbfTrackerConfig();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::perception::fusion::PbfTrackerConfig::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_PbfTrackerConfig =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsPbfTrackerConfig}, {}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_PbfTrackerConfig.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::fusion::PbfTrackerConfig, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::fusion::PbfTrackerConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::fusion::PbfTrackerConfig, type_fusion_method_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::fusion::PbfTrackerConfig, motion_fusion_method_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::fusion::PbfTrackerConfig, shape_fusion_method_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::fusion::PbfTrackerConfig, existence_fusion_method_),
  0,
  1,
  2,
  3,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 9, sizeof(::apollo::perception::fusion::PbfTrackerConfig)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::perception::fusion::_PbfTrackerConfig_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "modules/perception/proto/pbf_tracker_config.proto", schemas, file_default_instances, TableStruct::offsets,
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
      "\n1modules/perception/proto/pbf_tracker_c"
      "onfig.proto\022\030apollo.perception.fusion\"\321\001"
      "\n\020PbfTrackerConfig\022)\n\022type_fusion_method"
      "\030\001 \001(\t:\rDstTypeFusion\0220\n\024motion_fusion_m"
      "ethod\030\002 \001(\t:\022KalmanMotionFusion\022+\n\023shape"
      "_fusion_method\030\003 \001(\t:\016PbfShapeFusion\0223\n\027"
      "existence_fusion_method\030\004 \001(\t:\022DstExiste"
      "nceFusion"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 289);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "modules/perception/proto/pbf_tracker_config.proto", &protobuf_RegisterTypes);
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
}  // namespace protobuf_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto
namespace apollo {
namespace perception {
namespace fusion {

// ===================================================================

void PbfTrackerConfig::InitAsDefaultInstance() {
}
::google::protobuf::internal::ExplicitlyConstructed<::std::string> PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_;
::google::protobuf::internal::ExplicitlyConstructed<::std::string> PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_;
::google::protobuf::internal::ExplicitlyConstructed<::std::string> PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_;
::google::protobuf::internal::ExplicitlyConstructed<::std::string> PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existence_fusion_method_;
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int PbfTrackerConfig::kTypeFusionMethodFieldNumber;
const int PbfTrackerConfig::kMotionFusionMethodFieldNumber;
const int PbfTrackerConfig::kShapeFusionMethodFieldNumber;
const int PbfTrackerConfig::kExistenceFusionMethodFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

PbfTrackerConfig::PbfTrackerConfig()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto::scc_info_PbfTrackerConfig.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.perception.fusion.PbfTrackerConfig)
}
PbfTrackerConfig::PbfTrackerConfig(const PbfTrackerConfig& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  type_fusion_method_.UnsafeSetDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get());
  if (from.has_type_fusion_method()) {
    type_fusion_method_.AssignWithDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get(), from.type_fusion_method_);
  }
  motion_fusion_method_.UnsafeSetDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get());
  if (from.has_motion_fusion_method()) {
    motion_fusion_method_.AssignWithDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get(), from.motion_fusion_method_);
  }
  shape_fusion_method_.UnsafeSetDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get());
  if (from.has_shape_fusion_method()) {
    shape_fusion_method_.AssignWithDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get(), from.shape_fusion_method_);
  }
  existence_fusion_method_.UnsafeSetDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existence_fusion_method_.get());
  if (from.has_existence_fusion_method()) {
    existence_fusion_method_.AssignWithDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existence_fusion_method_.get(), from.existence_fusion_method_);
  }
  // @@protoc_insertion_point(copy_constructor:apollo.perception.fusion.PbfTrackerConfig)
}

void PbfTrackerConfig::SharedCtor() {
  type_fusion_method_.UnsafeSetDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get());
  motion_fusion_method_.UnsafeSetDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get());
  shape_fusion_method_.UnsafeSetDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get());
  existence_fusion_method_.UnsafeSetDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existence_fusion_method_.get());
}

PbfTrackerConfig::~PbfTrackerConfig() {
  // @@protoc_insertion_point(destructor:apollo.perception.fusion.PbfTrackerConfig)
  SharedDtor();
}

void PbfTrackerConfig::SharedDtor() {
  type_fusion_method_.DestroyNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get());
  motion_fusion_method_.DestroyNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get());
  shape_fusion_method_.DestroyNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get());
  existence_fusion_method_.DestroyNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existence_fusion_method_.get());
}

void PbfTrackerConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* PbfTrackerConfig::descriptor() {
  ::protobuf_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const PbfTrackerConfig& PbfTrackerConfig::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto::scc_info_PbfTrackerConfig.base);
  return *internal_default_instance();
}


void PbfTrackerConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.fusion.PbfTrackerConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 15u) {
    if (cached_has_bits & 0x00000001u) {
      type_fusion_method_.UnsafeMutablePointer()->assign(*&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get());
    }
    if (cached_has_bits & 0x00000002u) {
      motion_fusion_method_.UnsafeMutablePointer()->assign(*&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get());
    }
    if (cached_has_bits & 0x00000004u) {
      shape_fusion_method_.UnsafeMutablePointer()->assign(*&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get());
    }
    if (cached_has_bits & 0x00000008u) {
      existence_fusion_method_.UnsafeMutablePointer()->assign(*&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existence_fusion_method_.get());
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool PbfTrackerConfig::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.perception.fusion.PbfTrackerConfig)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional string type_fusion_method = 1 [default = "DstTypeFusion"];
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_type_fusion_method()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->type_fusion_method().data(), static_cast<int>(this->type_fusion_method().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "apollo.perception.fusion.PbfTrackerConfig.type_fusion_method");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional string motion_fusion_method = 2 [default = "KalmanMotionFusion"];
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_motion_fusion_method()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->motion_fusion_method().data(), static_cast<int>(this->motion_fusion_method().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "apollo.perception.fusion.PbfTrackerConfig.motion_fusion_method");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional string shape_fusion_method = 3 [default = "PbfShapeFusion"];
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(26u /* 26 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_shape_fusion_method()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->shape_fusion_method().data(), static_cast<int>(this->shape_fusion_method().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "apollo.perception.fusion.PbfTrackerConfig.shape_fusion_method");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional string existence_fusion_method = 4 [default = "DstExistenceFusion"];
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(34u /* 34 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_existence_fusion_method()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->existence_fusion_method().data(), static_cast<int>(this->existence_fusion_method().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "apollo.perception.fusion.PbfTrackerConfig.existence_fusion_method");
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
  // @@protoc_insertion_point(parse_success:apollo.perception.fusion.PbfTrackerConfig)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.perception.fusion.PbfTrackerConfig)
  return false;
#undef DO_
}

void PbfTrackerConfig::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.perception.fusion.PbfTrackerConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional string type_fusion_method = 1 [default = "DstTypeFusion"];
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->type_fusion_method().data(), static_cast<int>(this->type_fusion_method().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.perception.fusion.PbfTrackerConfig.type_fusion_method");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      1, this->type_fusion_method(), output);
  }

  // optional string motion_fusion_method = 2 [default = "KalmanMotionFusion"];
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->motion_fusion_method().data(), static_cast<int>(this->motion_fusion_method().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.perception.fusion.PbfTrackerConfig.motion_fusion_method");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      2, this->motion_fusion_method(), output);
  }

  // optional string shape_fusion_method = 3 [default = "PbfShapeFusion"];
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->shape_fusion_method().data(), static_cast<int>(this->shape_fusion_method().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.perception.fusion.PbfTrackerConfig.shape_fusion_method");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      3, this->shape_fusion_method(), output);
  }

  // optional string existence_fusion_method = 4 [default = "DstExistenceFusion"];
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->existence_fusion_method().data(), static_cast<int>(this->existence_fusion_method().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.perception.fusion.PbfTrackerConfig.existence_fusion_method");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      4, this->existence_fusion_method(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.perception.fusion.PbfTrackerConfig)
}

::google::protobuf::uint8* PbfTrackerConfig::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.fusion.PbfTrackerConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional string type_fusion_method = 1 [default = "DstTypeFusion"];
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->type_fusion_method().data(), static_cast<int>(this->type_fusion_method().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.perception.fusion.PbfTrackerConfig.type_fusion_method");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        1, this->type_fusion_method(), target);
  }

  // optional string motion_fusion_method = 2 [default = "KalmanMotionFusion"];
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->motion_fusion_method().data(), static_cast<int>(this->motion_fusion_method().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.perception.fusion.PbfTrackerConfig.motion_fusion_method");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        2, this->motion_fusion_method(), target);
  }

  // optional string shape_fusion_method = 3 [default = "PbfShapeFusion"];
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->shape_fusion_method().data(), static_cast<int>(this->shape_fusion_method().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.perception.fusion.PbfTrackerConfig.shape_fusion_method");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        3, this->shape_fusion_method(), target);
  }

  // optional string existence_fusion_method = 4 [default = "DstExistenceFusion"];
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->existence_fusion_method().data(), static_cast<int>(this->existence_fusion_method().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.perception.fusion.PbfTrackerConfig.existence_fusion_method");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        4, this->existence_fusion_method(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.fusion.PbfTrackerConfig)
  return target;
}

size_t PbfTrackerConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.fusion.PbfTrackerConfig)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 15u) {
    // optional string type_fusion_method = 1 [default = "DstTypeFusion"];
    if (has_type_fusion_method()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->type_fusion_method());
    }

    // optional string motion_fusion_method = 2 [default = "KalmanMotionFusion"];
    if (has_motion_fusion_method()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->motion_fusion_method());
    }

    // optional string shape_fusion_method = 3 [default = "PbfShapeFusion"];
    if (has_shape_fusion_method()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->shape_fusion_method());
    }

    // optional string existence_fusion_method = 4 [default = "DstExistenceFusion"];
    if (has_existence_fusion_method()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->existence_fusion_method());
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void PbfTrackerConfig::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.perception.fusion.PbfTrackerConfig)
  GOOGLE_DCHECK_NE(&from, this);
  const PbfTrackerConfig* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const PbfTrackerConfig>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.perception.fusion.PbfTrackerConfig)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.perception.fusion.PbfTrackerConfig)
    MergeFrom(*source);
  }
}

void PbfTrackerConfig::MergeFrom(const PbfTrackerConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.fusion.PbfTrackerConfig)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 15u) {
    if (cached_has_bits & 0x00000001u) {
      set_has_type_fusion_method();
      type_fusion_method_.AssignWithDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get(), from.type_fusion_method_);
    }
    if (cached_has_bits & 0x00000002u) {
      set_has_motion_fusion_method();
      motion_fusion_method_.AssignWithDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get(), from.motion_fusion_method_);
    }
    if (cached_has_bits & 0x00000004u) {
      set_has_shape_fusion_method();
      shape_fusion_method_.AssignWithDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get(), from.shape_fusion_method_);
    }
    if (cached_has_bits & 0x00000008u) {
      set_has_existence_fusion_method();
      existence_fusion_method_.AssignWithDefault(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existence_fusion_method_.get(), from.existence_fusion_method_);
    }
  }
}

void PbfTrackerConfig::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.perception.fusion.PbfTrackerConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void PbfTrackerConfig::CopyFrom(const PbfTrackerConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.fusion.PbfTrackerConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PbfTrackerConfig::IsInitialized() const {
  return true;
}

void PbfTrackerConfig::Swap(PbfTrackerConfig* other) {
  if (other == this) return;
  InternalSwap(other);
}
void PbfTrackerConfig::InternalSwap(PbfTrackerConfig* other) {
  using std::swap;
  type_fusion_method_.Swap(&other->type_fusion_method_, &::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get(),
    GetArenaNoVirtual());
  motion_fusion_method_.Swap(&other->motion_fusion_method_, &::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get(),
    GetArenaNoVirtual());
  shape_fusion_method_.Swap(&other->shape_fusion_method_, &::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get(),
    GetArenaNoVirtual());
  existence_fusion_method_.Swap(&other->existence_fusion_method_, &::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existence_fusion_method_.get(),
    GetArenaNoVirtual());
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata PbfTrackerConfig::GetMetadata() const {
  protobuf_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace fusion
}  // namespace perception
}  // namespace apollo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::perception::fusion::PbfTrackerConfig* Arena::CreateMaybeMessage< ::apollo::perception::fusion::PbfTrackerConfig >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::perception::fusion::PbfTrackerConfig >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
