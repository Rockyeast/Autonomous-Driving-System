// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/proto/fused_classifier_config.proto

#include "modules/perception/proto/fused_classifier_config.pb.h"

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
namespace lidar {
class FusedClassifierConfigDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<FusedClassifierConfig>
      _instance;
} _FusedClassifierConfig_default_instance_;
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
namespace protobuf_modules_2fperception_2fproto_2ffused_5fclassifier_5fconfig_2eproto {
static void InitDefaultsFusedClassifierConfig() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_one_shot_fusion_method_.DefaultConstruct();
  *::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_one_shot_fusion_method_.get_mutable() = ::std::string("CCRFOneShotTypeFusion", 21);
  ::google::protobuf::internal::OnShutdownDestroyString(
      ::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_one_shot_fusion_method_.get_mutable());
  ::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_sequence_fusion_method_.DefaultConstruct();
  *::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_sequence_fusion_method_.get_mutable() = ::std::string("CCRFSequenceTypeFusion", 22);
  ::google::protobuf::internal::OnShutdownDestroyString(
      ::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_sequence_fusion_method_.get_mutable());
  {
    void* ptr = &::apollo::perception::lidar::_FusedClassifierConfig_default_instance_;
    new (ptr) ::apollo::perception::lidar::FusedClassifierConfig();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::perception::lidar::FusedClassifierConfig::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_FusedClassifierConfig =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsFusedClassifierConfig}, {}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_FusedClassifierConfig.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::lidar::FusedClassifierConfig, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::lidar::FusedClassifierConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::lidar::FusedClassifierConfig, temporal_window_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::lidar::FusedClassifierConfig, enable_temporal_fusion_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::lidar::FusedClassifierConfig, one_shot_fusion_method_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::lidar::FusedClassifierConfig, sequence_fusion_method_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::lidar::FusedClassifierConfig, use_tracked_objects_),
  4,
  2,
  0,
  1,
  3,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 10, sizeof(::apollo::perception::lidar::FusedClassifierConfig)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::perception::lidar::_FusedClassifierConfig_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "modules/perception/proto/fused_classifier_config.proto", schemas, file_default_instances, TableStruct::offsets,
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
      "\n6modules/perception/proto/fused_classif"
      "ier_config.proto\022\027apollo.perception.lida"
      "r\"\354\001\n\025FusedClassifierConfig\022\033\n\017temporal_"
      "window\030\001 \001(\001:\00220\022$\n\026enable_temporal_fusi"
      "on\030\002 \001(\010:\004true\0225\n\026one_shot_fusion_method"
      "\030\003 \001(\t:\025CCRFOneShotTypeFusion\0226\n\026sequenc"
      "e_fusion_method\030\004 \001(\t:\026CCRFSequenceTypeF"
      "usion\022!\n\023use_tracked_objects\030\005 \001(\010:\004true"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 320);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "modules/perception/proto/fused_classifier_config.proto", &protobuf_RegisterTypes);
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
}  // namespace protobuf_modules_2fperception_2fproto_2ffused_5fclassifier_5fconfig_2eproto
namespace apollo {
namespace perception {
namespace lidar {

// ===================================================================

void FusedClassifierConfig::InitAsDefaultInstance() {
}
::google::protobuf::internal::ExplicitlyConstructed<::std::string> FusedClassifierConfig::_i_give_permission_to_break_this_code_default_one_shot_fusion_method_;
::google::protobuf::internal::ExplicitlyConstructed<::std::string> FusedClassifierConfig::_i_give_permission_to_break_this_code_default_sequence_fusion_method_;
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int FusedClassifierConfig::kTemporalWindowFieldNumber;
const int FusedClassifierConfig::kEnableTemporalFusionFieldNumber;
const int FusedClassifierConfig::kOneShotFusionMethodFieldNumber;
const int FusedClassifierConfig::kSequenceFusionMethodFieldNumber;
const int FusedClassifierConfig::kUseTrackedObjectsFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

FusedClassifierConfig::FusedClassifierConfig()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fperception_2fproto_2ffused_5fclassifier_5fconfig_2eproto::scc_info_FusedClassifierConfig.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.perception.lidar.FusedClassifierConfig)
}
FusedClassifierConfig::FusedClassifierConfig(const FusedClassifierConfig& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  one_shot_fusion_method_.UnsafeSetDefault(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_one_shot_fusion_method_.get());
  if (from.has_one_shot_fusion_method()) {
    one_shot_fusion_method_.AssignWithDefault(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_one_shot_fusion_method_.get(), from.one_shot_fusion_method_);
  }
  sequence_fusion_method_.UnsafeSetDefault(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_sequence_fusion_method_.get());
  if (from.has_sequence_fusion_method()) {
    sequence_fusion_method_.AssignWithDefault(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_sequence_fusion_method_.get(), from.sequence_fusion_method_);
  }
  ::memcpy(&enable_temporal_fusion_, &from.enable_temporal_fusion_,
    static_cast<size_t>(reinterpret_cast<char*>(&temporal_window_) -
    reinterpret_cast<char*>(&enable_temporal_fusion_)) + sizeof(temporal_window_));
  // @@protoc_insertion_point(copy_constructor:apollo.perception.lidar.FusedClassifierConfig)
}

void FusedClassifierConfig::SharedCtor() {
  one_shot_fusion_method_.UnsafeSetDefault(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_one_shot_fusion_method_.get());
  sequence_fusion_method_.UnsafeSetDefault(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_sequence_fusion_method_.get());
  enable_temporal_fusion_ = true;
  use_tracked_objects_ = true;
  temporal_window_ = 20;
}

FusedClassifierConfig::~FusedClassifierConfig() {
  // @@protoc_insertion_point(destructor:apollo.perception.lidar.FusedClassifierConfig)
  SharedDtor();
}

void FusedClassifierConfig::SharedDtor() {
  one_shot_fusion_method_.DestroyNoArena(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_one_shot_fusion_method_.get());
  sequence_fusion_method_.DestroyNoArena(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_sequence_fusion_method_.get());
}

void FusedClassifierConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* FusedClassifierConfig::descriptor() {
  ::protobuf_modules_2fperception_2fproto_2ffused_5fclassifier_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fperception_2fproto_2ffused_5fclassifier_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const FusedClassifierConfig& FusedClassifierConfig::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fperception_2fproto_2ffused_5fclassifier_5fconfig_2eproto::scc_info_FusedClassifierConfig.base);
  return *internal_default_instance();
}


void FusedClassifierConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.lidar.FusedClassifierConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 31u) {
    if (cached_has_bits & 0x00000001u) {
      one_shot_fusion_method_.UnsafeMutablePointer()->assign(*&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_one_shot_fusion_method_.get());
    }
    if (cached_has_bits & 0x00000002u) {
      sequence_fusion_method_.UnsafeMutablePointer()->assign(*&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_sequence_fusion_method_.get());
    }
    enable_temporal_fusion_ = true;
    use_tracked_objects_ = true;
    temporal_window_ = 20;
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool FusedClassifierConfig::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.perception.lidar.FusedClassifierConfig)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional double temporal_window = 1 [default = 20];
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {
          set_has_temporal_window();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &temporal_window_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional bool enable_temporal_fusion = 2 [default = true];
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(16u /* 16 & 0xFF */)) {
          set_has_enable_temporal_fusion();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &enable_temporal_fusion_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional string one_shot_fusion_method = 3 [default = "CCRFOneShotTypeFusion"];
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(26u /* 26 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_one_shot_fusion_method()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->one_shot_fusion_method().data(), static_cast<int>(this->one_shot_fusion_method().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "apollo.perception.lidar.FusedClassifierConfig.one_shot_fusion_method");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional string sequence_fusion_method = 4 [default = "CCRFSequenceTypeFusion"];
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(34u /* 34 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_sequence_fusion_method()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->sequence_fusion_method().data(), static_cast<int>(this->sequence_fusion_method().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "apollo.perception.lidar.FusedClassifierConfig.sequence_fusion_method");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional bool use_tracked_objects = 5 [default = true];
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(40u /* 40 & 0xFF */)) {
          set_has_use_tracked_objects();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &use_tracked_objects_)));
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
  // @@protoc_insertion_point(parse_success:apollo.perception.lidar.FusedClassifierConfig)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.perception.lidar.FusedClassifierConfig)
  return false;
#undef DO_
}

void FusedClassifierConfig::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.perception.lidar.FusedClassifierConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double temporal_window = 1 [default = 20];
  if (cached_has_bits & 0x00000010u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->temporal_window(), output);
  }

  // optional bool enable_temporal_fusion = 2 [default = true];
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(2, this->enable_temporal_fusion(), output);
  }

  // optional string one_shot_fusion_method = 3 [default = "CCRFOneShotTypeFusion"];
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->one_shot_fusion_method().data(), static_cast<int>(this->one_shot_fusion_method().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.perception.lidar.FusedClassifierConfig.one_shot_fusion_method");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      3, this->one_shot_fusion_method(), output);
  }

  // optional string sequence_fusion_method = 4 [default = "CCRFSequenceTypeFusion"];
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->sequence_fusion_method().data(), static_cast<int>(this->sequence_fusion_method().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.perception.lidar.FusedClassifierConfig.sequence_fusion_method");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      4, this->sequence_fusion_method(), output);
  }

  // optional bool use_tracked_objects = 5 [default = true];
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(5, this->use_tracked_objects(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.perception.lidar.FusedClassifierConfig)
}

::google::protobuf::uint8* FusedClassifierConfig::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.lidar.FusedClassifierConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double temporal_window = 1 [default = 20];
  if (cached_has_bits & 0x00000010u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->temporal_window(), target);
  }

  // optional bool enable_temporal_fusion = 2 [default = true];
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(2, this->enable_temporal_fusion(), target);
  }

  // optional string one_shot_fusion_method = 3 [default = "CCRFOneShotTypeFusion"];
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->one_shot_fusion_method().data(), static_cast<int>(this->one_shot_fusion_method().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.perception.lidar.FusedClassifierConfig.one_shot_fusion_method");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        3, this->one_shot_fusion_method(), target);
  }

  // optional string sequence_fusion_method = 4 [default = "CCRFSequenceTypeFusion"];
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->sequence_fusion_method().data(), static_cast<int>(this->sequence_fusion_method().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.perception.lidar.FusedClassifierConfig.sequence_fusion_method");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        4, this->sequence_fusion_method(), target);
  }

  // optional bool use_tracked_objects = 5 [default = true];
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(5, this->use_tracked_objects(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.lidar.FusedClassifierConfig)
  return target;
}

size_t FusedClassifierConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.lidar.FusedClassifierConfig)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 31u) {
    // optional string one_shot_fusion_method = 3 [default = "CCRFOneShotTypeFusion"];
    if (has_one_shot_fusion_method()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->one_shot_fusion_method());
    }

    // optional string sequence_fusion_method = 4 [default = "CCRFSequenceTypeFusion"];
    if (has_sequence_fusion_method()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->sequence_fusion_method());
    }

    // optional bool enable_temporal_fusion = 2 [default = true];
    if (has_enable_temporal_fusion()) {
      total_size += 1 + 1;
    }

    // optional bool use_tracked_objects = 5 [default = true];
    if (has_use_tracked_objects()) {
      total_size += 1 + 1;
    }

    // optional double temporal_window = 1 [default = 20];
    if (has_temporal_window()) {
      total_size += 1 + 8;
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void FusedClassifierConfig::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.perception.lidar.FusedClassifierConfig)
  GOOGLE_DCHECK_NE(&from, this);
  const FusedClassifierConfig* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const FusedClassifierConfig>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.perception.lidar.FusedClassifierConfig)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.perception.lidar.FusedClassifierConfig)
    MergeFrom(*source);
  }
}

void FusedClassifierConfig::MergeFrom(const FusedClassifierConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.lidar.FusedClassifierConfig)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 31u) {
    if (cached_has_bits & 0x00000001u) {
      set_has_one_shot_fusion_method();
      one_shot_fusion_method_.AssignWithDefault(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_one_shot_fusion_method_.get(), from.one_shot_fusion_method_);
    }
    if (cached_has_bits & 0x00000002u) {
      set_has_sequence_fusion_method();
      sequence_fusion_method_.AssignWithDefault(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_sequence_fusion_method_.get(), from.sequence_fusion_method_);
    }
    if (cached_has_bits & 0x00000004u) {
      enable_temporal_fusion_ = from.enable_temporal_fusion_;
    }
    if (cached_has_bits & 0x00000008u) {
      use_tracked_objects_ = from.use_tracked_objects_;
    }
    if (cached_has_bits & 0x00000010u) {
      temporal_window_ = from.temporal_window_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void FusedClassifierConfig::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.perception.lidar.FusedClassifierConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void FusedClassifierConfig::CopyFrom(const FusedClassifierConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.lidar.FusedClassifierConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool FusedClassifierConfig::IsInitialized() const {
  return true;
}

void FusedClassifierConfig::Swap(FusedClassifierConfig* other) {
  if (other == this) return;
  InternalSwap(other);
}
void FusedClassifierConfig::InternalSwap(FusedClassifierConfig* other) {
  using std::swap;
  one_shot_fusion_method_.Swap(&other->one_shot_fusion_method_, &::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_one_shot_fusion_method_.get(),
    GetArenaNoVirtual());
  sequence_fusion_method_.Swap(&other->sequence_fusion_method_, &::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_sequence_fusion_method_.get(),
    GetArenaNoVirtual());
  swap(enable_temporal_fusion_, other->enable_temporal_fusion_);
  swap(use_tracked_objects_, other->use_tracked_objects_);
  swap(temporal_window_, other->temporal_window_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata FusedClassifierConfig::GetMetadata() const {
  protobuf_modules_2fperception_2fproto_2ffused_5fclassifier_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fperception_2fproto_2ffused_5fclassifier_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::perception::lidar::FusedClassifierConfig* Arena::CreateMaybeMessage< ::apollo::perception::lidar::FusedClassifierConfig >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::perception::lidar::FusedClassifierConfig >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)