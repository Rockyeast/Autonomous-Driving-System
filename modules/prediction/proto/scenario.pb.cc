// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/prediction/proto/scenario.proto

#include "modules/prediction/proto/scenario.pb.h"

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
namespace prediction {
class ScenarioDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<Scenario>
      _instance;
} _Scenario_default_instance_;
}  // namespace prediction
}  // namespace apollo
namespace protobuf_modules_2fprediction_2fproto_2fscenario_2eproto {
static void InitDefaultsScenario() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::prediction::_Scenario_default_instance_;
    new (ptr) ::apollo::prediction::Scenario();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::prediction::Scenario::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_Scenario =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsScenario}, {}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_Scenario.base);
}

::google::protobuf::Metadata file_level_metadata[1];
const ::google::protobuf::EnumDescriptor* file_level_enum_descriptors[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::prediction::Scenario, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::prediction::Scenario, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::prediction::Scenario, type_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::prediction::Scenario, junction_id_),
  1,
  0,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, sizeof(::apollo::prediction::Scenario)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::prediction::_Scenario_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "modules/prediction/proto/scenario.proto", schemas, file_default_instances, TableStruct::offsets,
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
      "\n\'modules/prediction/proto/scenario.prot"
      "o\022\021apollo.prediction\"\350\001\n\010Scenario\0227\n\004typ"
      "e\030\001 \001(\0162 .apollo.prediction.Scenario.Typ"
      "e:\007UNKNOWN\022\023\n\013junction_id\030\002 \001(\t\"\215\001\n\004Type"
      "\022\013\n\007UNKNOWN\020\000\022\013\n\006CRUISE\020\350\007\022\021\n\014CRUISE_URB"
      "AN\020\351\007\022\023\n\016CRUISE_HIGHWAY\020\352\007\022\r\n\010JUNCTION\020\320"
      "\017\022\033\n\026JUNCTION_TRAFFIC_LIGHT\020\321\017\022\027\n\022JUNCTI"
      "ON_STOP_SIGN\020\322\017"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 295);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "modules/prediction/proto/scenario.proto", &protobuf_RegisterTypes);
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
}  // namespace protobuf_modules_2fprediction_2fproto_2fscenario_2eproto
namespace apollo {
namespace prediction {
const ::google::protobuf::EnumDescriptor* Scenario_Type_descriptor() {
  protobuf_modules_2fprediction_2fproto_2fscenario_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_modules_2fprediction_2fproto_2fscenario_2eproto::file_level_enum_descriptors[0];
}
bool Scenario_Type_IsValid(int value) {
  switch (value) {
    case 0:
    case 1000:
    case 1001:
    case 1002:
    case 2000:
    case 2001:
    case 2002:
      return true;
    default:
      return false;
  }
}

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const Scenario_Type Scenario::UNKNOWN;
const Scenario_Type Scenario::CRUISE;
const Scenario_Type Scenario::CRUISE_URBAN;
const Scenario_Type Scenario::CRUISE_HIGHWAY;
const Scenario_Type Scenario::JUNCTION;
const Scenario_Type Scenario::JUNCTION_TRAFFIC_LIGHT;
const Scenario_Type Scenario::JUNCTION_STOP_SIGN;
const Scenario_Type Scenario::Type_MIN;
const Scenario_Type Scenario::Type_MAX;
const int Scenario::Type_ARRAYSIZE;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

// ===================================================================

void Scenario::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Scenario::kTypeFieldNumber;
const int Scenario::kJunctionIdFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Scenario::Scenario()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fprediction_2fproto_2fscenario_2eproto::scc_info_Scenario.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.prediction.Scenario)
}
Scenario::Scenario(const Scenario& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  junction_id_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_junction_id()) {
    junction_id_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.junction_id_);
  }
  type_ = from.type_;
  // @@protoc_insertion_point(copy_constructor:apollo.prediction.Scenario)
}

void Scenario::SharedCtor() {
  junction_id_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  type_ = 0;
}

Scenario::~Scenario() {
  // @@protoc_insertion_point(destructor:apollo.prediction.Scenario)
  SharedDtor();
}

void Scenario::SharedDtor() {
  junction_id_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}

void Scenario::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* Scenario::descriptor() {
  ::protobuf_modules_2fprediction_2fproto_2fscenario_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fprediction_2fproto_2fscenario_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const Scenario& Scenario::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fprediction_2fproto_2fscenario_2eproto::scc_info_Scenario.base);
  return *internal_default_instance();
}


void Scenario::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.prediction.Scenario)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    junction_id_.ClearNonDefaultToEmptyNoArena();
  }
  type_ = 0;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool Scenario::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.prediction.Scenario)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .apollo.prediction.Scenario.Type type = 1 [default = UNKNOWN];
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(8u /* 8 & 0xFF */)) {
          int value;
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   int, ::google::protobuf::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          if (::apollo::prediction::Scenario_Type_IsValid(value)) {
            set_type(static_cast< ::apollo::prediction::Scenario_Type >(value));
          } else {
            mutable_unknown_fields()->AddVarint(
                1, static_cast< ::google::protobuf::uint64>(value));
          }
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional string junction_id = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_junction_id()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->junction_id().data(), static_cast<int>(this->junction_id().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "apollo.prediction.Scenario.junction_id");
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
  // @@protoc_insertion_point(parse_success:apollo.prediction.Scenario)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.prediction.Scenario)
  return false;
#undef DO_
}

void Scenario::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.prediction.Scenario)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.prediction.Scenario.Type type = 1 [default = UNKNOWN];
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteEnum(
      1, this->type(), output);
  }

  // optional string junction_id = 2;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->junction_id().data(), static_cast<int>(this->junction_id().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.prediction.Scenario.junction_id");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      2, this->junction_id(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.prediction.Scenario)
}

::google::protobuf::uint8* Scenario::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.prediction.Scenario)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.prediction.Scenario.Type type = 1 [default = UNKNOWN];
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteEnumToArray(
      1, this->type(), target);
  }

  // optional string junction_id = 2;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->junction_id().data(), static_cast<int>(this->junction_id().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.prediction.Scenario.junction_id");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        2, this->junction_id(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.prediction.Scenario)
  return target;
}

size_t Scenario::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.prediction.Scenario)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 3u) {
    // optional string junction_id = 2;
    if (has_junction_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->junction_id());
    }

    // optional .apollo.prediction.Scenario.Type type = 1 [default = UNKNOWN];
    if (has_type()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::EnumSize(this->type());
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Scenario::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.prediction.Scenario)
  GOOGLE_DCHECK_NE(&from, this);
  const Scenario* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const Scenario>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.prediction.Scenario)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.prediction.Scenario)
    MergeFrom(*source);
  }
}

void Scenario::MergeFrom(const Scenario& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.prediction.Scenario)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 3u) {
    if (cached_has_bits & 0x00000001u) {
      set_has_junction_id();
      junction_id_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.junction_id_);
    }
    if (cached_has_bits & 0x00000002u) {
      type_ = from.type_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void Scenario::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.prediction.Scenario)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Scenario::CopyFrom(const Scenario& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.prediction.Scenario)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Scenario::IsInitialized() const {
  return true;
}

void Scenario::Swap(Scenario* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Scenario::InternalSwap(Scenario* other) {
  using std::swap;
  junction_id_.Swap(&other->junction_id_, &::google::protobuf::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  swap(type_, other->type_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata Scenario::GetMetadata() const {
  protobuf_modules_2fprediction_2fproto_2fscenario_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fprediction_2fproto_2fscenario_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace prediction
}  // namespace apollo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::prediction::Scenario* Arena::CreateMaybeMessage< ::apollo::prediction::Scenario >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::prediction::Scenario >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)