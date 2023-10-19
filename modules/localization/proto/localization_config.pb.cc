// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/localization/proto/localization_config.proto

#include "modules/localization/proto/localization_config.pb.h"

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
namespace localization {
class LocalizationConfigDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<LocalizationConfig>
      _instance;
} _LocalizationConfig_default_instance_;
}  // namespace localization
}  // namespace apollo
namespace protobuf_modules_2flocalization_2fproto_2flocalization_5fconfig_2eproto {
static void InitDefaultsLocalizationConfig() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::localization::_LocalizationConfig_default_instance_;
    new (ptr) ::apollo::localization::LocalizationConfig();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::localization::LocalizationConfig::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_LocalizationConfig =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsLocalizationConfig}, {}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_LocalizationConfig.base);
}

::google::protobuf::Metadata file_level_metadata[1];
const ::google::protobuf::EnumDescriptor* file_level_enum_descriptors[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::localization::LocalizationConfig, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::localization::LocalizationConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::localization::LocalizationConfig, localization_type_),
  0,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 6, sizeof(::apollo::localization::LocalizationConfig)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::localization::_LocalizationConfig_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "modules/localization/proto/localization_config.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, file_level_enum_descriptors, NULL);
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
      "\n4modules/localization/proto/localizatio"
      "n_config.proto\022\023apollo.localization\"\224\001\n\022"
      "LocalizationConfig\022X\n\021localization_type\030"
      "\001 \001(\01628.apollo.localization.Localization"
      "Config.LocalizationType:\003RTK\"$\n\020Localiza"
      "tionType\022\007\n\003RTK\020\000\022\007\n\003MSF\020\001"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 226);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "modules/localization/proto/localization_config.proto", &protobuf_RegisterTypes);
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
}  // namespace protobuf_modules_2flocalization_2fproto_2flocalization_5fconfig_2eproto
namespace apollo {
namespace localization {
const ::google::protobuf::EnumDescriptor* LocalizationConfig_LocalizationType_descriptor() {
  protobuf_modules_2flocalization_2fproto_2flocalization_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_modules_2flocalization_2fproto_2flocalization_5fconfig_2eproto::file_level_enum_descriptors[0];
}
bool LocalizationConfig_LocalizationType_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
      return true;
    default:
      return false;
  }
}

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const LocalizationConfig_LocalizationType LocalizationConfig::RTK;
const LocalizationConfig_LocalizationType LocalizationConfig::MSF;
const LocalizationConfig_LocalizationType LocalizationConfig::LocalizationType_MIN;
const LocalizationConfig_LocalizationType LocalizationConfig::LocalizationType_MAX;
const int LocalizationConfig::LocalizationType_ARRAYSIZE;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

// ===================================================================

void LocalizationConfig::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int LocalizationConfig::kLocalizationTypeFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

LocalizationConfig::LocalizationConfig()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2flocalization_2fproto_2flocalization_5fconfig_2eproto::scc_info_LocalizationConfig.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.localization.LocalizationConfig)
}
LocalizationConfig::LocalizationConfig(const LocalizationConfig& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  localization_type_ = from.localization_type_;
  // @@protoc_insertion_point(copy_constructor:apollo.localization.LocalizationConfig)
}

void LocalizationConfig::SharedCtor() {
  localization_type_ = 0;
}

LocalizationConfig::~LocalizationConfig() {
  // @@protoc_insertion_point(destructor:apollo.localization.LocalizationConfig)
  SharedDtor();
}

void LocalizationConfig::SharedDtor() {
}

void LocalizationConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* LocalizationConfig::descriptor() {
  ::protobuf_modules_2flocalization_2fproto_2flocalization_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2flocalization_2fproto_2flocalization_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const LocalizationConfig& LocalizationConfig::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2flocalization_2fproto_2flocalization_5fconfig_2eproto::scc_info_LocalizationConfig.base);
  return *internal_default_instance();
}


void LocalizationConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.localization.LocalizationConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  localization_type_ = 0;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool LocalizationConfig::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.localization.LocalizationConfig)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .apollo.localization.LocalizationConfig.LocalizationType localization_type = 1 [default = RTK];
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(8u /* 8 & 0xFF */)) {
          int value;
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   int, ::google::protobuf::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          if (::apollo::localization::LocalizationConfig_LocalizationType_IsValid(value)) {
            set_localization_type(static_cast< ::apollo::localization::LocalizationConfig_LocalizationType >(value));
          } else {
            mutable_unknown_fields()->AddVarint(
                1, static_cast< ::google::protobuf::uint64>(value));
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
  // @@protoc_insertion_point(parse_success:apollo.localization.LocalizationConfig)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.localization.LocalizationConfig)
  return false;
#undef DO_
}

void LocalizationConfig::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.localization.LocalizationConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.localization.LocalizationConfig.LocalizationType localization_type = 1 [default = RTK];
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteEnum(
      1, this->localization_type(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.localization.LocalizationConfig)
}

::google::protobuf::uint8* LocalizationConfig::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.localization.LocalizationConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.localization.LocalizationConfig.LocalizationType localization_type = 1 [default = RTK];
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteEnumToArray(
      1, this->localization_type(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.localization.LocalizationConfig)
  return target;
}

size_t LocalizationConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.localization.LocalizationConfig)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  // optional .apollo.localization.LocalizationConfig.LocalizationType localization_type = 1 [default = RTK];
  if (has_localization_type()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::EnumSize(this->localization_type());
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void LocalizationConfig::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.localization.LocalizationConfig)
  GOOGLE_DCHECK_NE(&from, this);
  const LocalizationConfig* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const LocalizationConfig>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.localization.LocalizationConfig)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.localization.LocalizationConfig)
    MergeFrom(*source);
  }
}

void LocalizationConfig::MergeFrom(const LocalizationConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.localization.LocalizationConfig)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.has_localization_type()) {
    set_localization_type(from.localization_type());
  }
}

void LocalizationConfig::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.localization.LocalizationConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void LocalizationConfig::CopyFrom(const LocalizationConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.localization.LocalizationConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool LocalizationConfig::IsInitialized() const {
  return true;
}

void LocalizationConfig::Swap(LocalizationConfig* other) {
  if (other == this) return;
  InternalSwap(other);
}
void LocalizationConfig::InternalSwap(LocalizationConfig* other) {
  using std::swap;
  swap(localization_type_, other->localization_type_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata LocalizationConfig::GetMetadata() const {
  protobuf_modules_2flocalization_2fproto_2flocalization_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2flocalization_2fproto_2flocalization_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace localization
}  // namespace apollo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::localization::LocalizationConfig* Arena::CreateMaybeMessage< ::apollo::localization::LocalizationConfig >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::localization::LocalizationConfig >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
