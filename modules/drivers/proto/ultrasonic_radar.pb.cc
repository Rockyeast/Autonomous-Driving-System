// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/drivers/proto/ultrasonic_radar.proto

#include "modules/drivers/proto/ultrasonic_radar.pb.h"

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

namespace protobuf_modules_2fcommon_2fproto_2fheader_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fcommon_2fproto_2fheader_2eproto ::google::protobuf::internal::SCCInfo<1> scc_info_Header;
}  // namespace protobuf_modules_2fcommon_2fproto_2fheader_2eproto
namespace apollo {
namespace drivers {
class UltrasonicDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<Ultrasonic>
      _instance;
} _Ultrasonic_default_instance_;
}  // namespace drivers
}  // namespace apollo
namespace protobuf_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto {
static void InitDefaultsUltrasonic() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::drivers::_Ultrasonic_default_instance_;
    new (ptr) ::apollo::drivers::Ultrasonic();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::drivers::Ultrasonic::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<1> scc_info_Ultrasonic =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 1, InitDefaultsUltrasonic}, {
      &protobuf_modules_2fcommon_2fproto_2fheader_2eproto::scc_info_Header.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_Ultrasonic.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::Ultrasonic, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::Ultrasonic, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::Ultrasonic, header_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::Ultrasonic, ranges_),
  0,
  ~0u,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, sizeof(::apollo::drivers::Ultrasonic)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::drivers::_Ultrasonic_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "modules/drivers/proto/ultrasonic_radar.proto", schemas, file_default_instances, TableStruct::offsets,
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
      "\n,modules/drivers/proto/ultrasonic_radar"
      ".proto\022\016apollo.drivers\032!modules/common/p"
      "roto/header.proto\"C\n\nUltrasonic\022%\n\006heade"
      "r\030\001 \001(\0132\025.apollo.common.Header\022\016\n\006ranges"
      "\030\002 \003(\002"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 166);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "modules/drivers/proto/ultrasonic_radar.proto", &protobuf_RegisterTypes);
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
}  // namespace protobuf_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto
namespace apollo {
namespace drivers {

// ===================================================================

void Ultrasonic::InitAsDefaultInstance() {
  ::apollo::drivers::_Ultrasonic_default_instance_._instance.get_mutable()->header_ = const_cast< ::apollo::common::Header*>(
      ::apollo::common::Header::internal_default_instance());
}
void Ultrasonic::clear_header() {
  if (header_ != NULL) header_->Clear();
  clear_has_header();
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Ultrasonic::kHeaderFieldNumber;
const int Ultrasonic::kRangesFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Ultrasonic::Ultrasonic()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto::scc_info_Ultrasonic.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.drivers.Ultrasonic)
}
Ultrasonic::Ultrasonic(const Ultrasonic& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      ranges_(from.ranges_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_header()) {
    header_ = new ::apollo::common::Header(*from.header_);
  } else {
    header_ = NULL;
  }
  // @@protoc_insertion_point(copy_constructor:apollo.drivers.Ultrasonic)
}

void Ultrasonic::SharedCtor() {
  header_ = NULL;
}

Ultrasonic::~Ultrasonic() {
  // @@protoc_insertion_point(destructor:apollo.drivers.Ultrasonic)
  SharedDtor();
}

void Ultrasonic::SharedDtor() {
  if (this != internal_default_instance()) delete header_;
}

void Ultrasonic::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* Ultrasonic::descriptor() {
  ::protobuf_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const Ultrasonic& Ultrasonic::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto::scc_info_Ultrasonic.base);
  return *internal_default_instance();
}


void Ultrasonic::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.drivers.Ultrasonic)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ranges_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(header_ != NULL);
    header_->Clear();
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool Ultrasonic::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.drivers.Ultrasonic)
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

      // repeated float ranges = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(21u /* 21 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 1, 21u, input, this->mutable_ranges())));
        } else if (
            static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, this->mutable_ranges())));
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
  // @@protoc_insertion_point(parse_success:apollo.drivers.Ultrasonic)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.drivers.Ultrasonic)
  return false;
#undef DO_
}

void Ultrasonic::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.drivers.Ultrasonic)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.Header header = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->_internal_header(), output);
  }

  // repeated float ranges = 2;
  for (int i = 0, n = this->ranges_size(); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(
      2, this->ranges(i), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.drivers.Ultrasonic)
}

::google::protobuf::uint8* Ultrasonic::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.drivers.Ultrasonic)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.Header header = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->_internal_header(), deterministic, target);
  }

  // repeated float ranges = 2;
  target = ::google::protobuf::internal::WireFormatLite::
    WriteFloatToArray(2, this->ranges_, target);

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.drivers.Ultrasonic)
  return target;
}

size_t Ultrasonic::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.drivers.Ultrasonic)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  // repeated float ranges = 2;
  {
    unsigned int count = static_cast<unsigned int>(this->ranges_size());
    size_t data_size = 4UL * count;
    total_size += 1 *
                  ::google::protobuf::internal::FromIntSize(this->ranges_size());
    total_size += data_size;
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

void Ultrasonic::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.drivers.Ultrasonic)
  GOOGLE_DCHECK_NE(&from, this);
  const Ultrasonic* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const Ultrasonic>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.drivers.Ultrasonic)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.drivers.Ultrasonic)
    MergeFrom(*source);
  }
}

void Ultrasonic::MergeFrom(const Ultrasonic& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.drivers.Ultrasonic)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  ranges_.MergeFrom(from.ranges_);
  if (from.has_header()) {
    mutable_header()->::apollo::common::Header::MergeFrom(from.header());
  }
}

void Ultrasonic::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.drivers.Ultrasonic)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Ultrasonic::CopyFrom(const Ultrasonic& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.drivers.Ultrasonic)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Ultrasonic::IsInitialized() const {
  return true;
}

void Ultrasonic::Swap(Ultrasonic* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Ultrasonic::InternalSwap(Ultrasonic* other) {
  using std::swap;
  ranges_.InternalSwap(&other->ranges_);
  swap(header_, other->header_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata Ultrasonic::GetMetadata() const {
  protobuf_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fdrivers_2fproto_2fultrasonic_5fradar_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace drivers
}  // namespace apollo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::drivers::Ultrasonic* Arena::CreateMaybeMessage< ::apollo::drivers::Ultrasonic >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::drivers::Ultrasonic >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)