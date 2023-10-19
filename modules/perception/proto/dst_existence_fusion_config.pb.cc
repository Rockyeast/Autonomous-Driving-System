// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/proto/dst_existence_fusion_config.proto

#include "modules/perception/proto/dst_existence_fusion_config.pb.h"

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

namespace protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_CameraValidDist;
}  // namespace protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto
namespace apollo {
namespace perception {
class CameraValidDistDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<CameraValidDist>
      _instance;
} _CameraValidDist_default_instance_;
class DstExistenceFusionConfigDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<DstExistenceFusionConfig>
      _instance;
} _DstExistenceFusionConfig_default_instance_;
}  // namespace perception
}  // namespace apollo
namespace protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto {
static void InitDefaultsCameraValidDist() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::perception::_CameraValidDist_default_instance_;
    new (ptr) ::apollo::perception::CameraValidDist();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::perception::CameraValidDist::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_CameraValidDist =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsCameraValidDist}, {}};

static void InitDefaultsDstExistenceFusionConfig() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::perception::_DstExistenceFusionConfig_default_instance_;
    new (ptr) ::apollo::perception::DstExistenceFusionConfig();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::perception::DstExistenceFusionConfig::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<1> scc_info_DstExistenceFusionConfig =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 1, InitDefaultsDstExistenceFusionConfig}, {
      &protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto::scc_info_CameraValidDist.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_CameraValidDist.base);
  ::google::protobuf::internal::InitSCC(&scc_info_DstExistenceFusionConfig.base);
}

::google::protobuf::Metadata file_level_metadata[2];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::CameraValidDist, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::CameraValidDist, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::CameraValidDist, camera_name_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::CameraValidDist, valid_dist_),
  0,
  1,
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::DstExistenceFusionConfig, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::DstExistenceFusionConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::DstExistenceFusionConfig, track_object_max_match_distance_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::DstExistenceFusionConfig, camera_valid_dist_),
  0,
  ~0u,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, sizeof(::apollo::perception::CameraValidDist)},
  { 9, 16, sizeof(::apollo::perception::DstExistenceFusionConfig)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::perception::_CameraValidDist_default_instance_),
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::perception::_DstExistenceFusionConfig_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "modules/perception/proto/dst_existence_fusion_config.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, NULL, NULL);
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
      "\n:modules/perception/proto/dst_existence"
      "_fusion_config.proto\022\021apollo.perception\""
      "\?\n\017CameraValidDist\022\025\n\013camera_name\030\001 \001(\t:"
      "\000\022\025\n\nvalid_dist\030\002 \001(\001:\0010\"\205\001\n\030DstExistenc"
      "eFusionConfig\022*\n\037track_object_max_match_"
      "distance\030\001 \001(\001:\0014\022=\n\021camera_valid_dist\030\002"
      " \003(\0132\".apollo.perception.CameraValidDist"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 280);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "modules/perception/proto/dst_existence_fusion_config.proto", &protobuf_RegisterTypes);
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
}  // namespace protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto
namespace apollo {
namespace perception {

// ===================================================================

void CameraValidDist::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int CameraValidDist::kCameraNameFieldNumber;
const int CameraValidDist::kValidDistFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

CameraValidDist::CameraValidDist()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto::scc_info_CameraValidDist.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.perception.CameraValidDist)
}
CameraValidDist::CameraValidDist(const CameraValidDist& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  camera_name_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_camera_name()) {
    camera_name_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.camera_name_);
  }
  valid_dist_ = from.valid_dist_;
  // @@protoc_insertion_point(copy_constructor:apollo.perception.CameraValidDist)
}

void CameraValidDist::SharedCtor() {
  camera_name_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  valid_dist_ = 0;
}

CameraValidDist::~CameraValidDist() {
  // @@protoc_insertion_point(destructor:apollo.perception.CameraValidDist)
  SharedDtor();
}

void CameraValidDist::SharedDtor() {
  camera_name_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}

void CameraValidDist::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* CameraValidDist::descriptor() {
  ::protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const CameraValidDist& CameraValidDist::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto::scc_info_CameraValidDist.base);
  return *internal_default_instance();
}


void CameraValidDist::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.CameraValidDist)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    camera_name_.ClearNonDefaultToEmptyNoArena();
  }
  valid_dist_ = 0;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool CameraValidDist::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.perception.CameraValidDist)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional string camera_name = 1 [default = ""];
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_camera_name()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->camera_name().data(), static_cast<int>(this->camera_name().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "apollo.perception.CameraValidDist.camera_name");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double valid_dist = 2 [default = 0];
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(17u /* 17 & 0xFF */)) {
          set_has_valid_dist();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &valid_dist_)));
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
  // @@protoc_insertion_point(parse_success:apollo.perception.CameraValidDist)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.perception.CameraValidDist)
  return false;
#undef DO_
}

void CameraValidDist::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.perception.CameraValidDist)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional string camera_name = 1 [default = ""];
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->camera_name().data(), static_cast<int>(this->camera_name().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.perception.CameraValidDist.camera_name");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      1, this->camera_name(), output);
  }

  // optional double valid_dist = 2 [default = 0];
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->valid_dist(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.perception.CameraValidDist)
}

::google::protobuf::uint8* CameraValidDist::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.CameraValidDist)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional string camera_name = 1 [default = ""];
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->camera_name().data(), static_cast<int>(this->camera_name().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.perception.CameraValidDist.camera_name");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        1, this->camera_name(), target);
  }

  // optional double valid_dist = 2 [default = 0];
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->valid_dist(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.CameraValidDist)
  return target;
}

size_t CameraValidDist::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.CameraValidDist)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 3u) {
    // optional string camera_name = 1 [default = ""];
    if (has_camera_name()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->camera_name());
    }

    // optional double valid_dist = 2 [default = 0];
    if (has_valid_dist()) {
      total_size += 1 + 8;
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void CameraValidDist::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.perception.CameraValidDist)
  GOOGLE_DCHECK_NE(&from, this);
  const CameraValidDist* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const CameraValidDist>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.perception.CameraValidDist)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.perception.CameraValidDist)
    MergeFrom(*source);
  }
}

void CameraValidDist::MergeFrom(const CameraValidDist& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.CameraValidDist)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 3u) {
    if (cached_has_bits & 0x00000001u) {
      set_has_camera_name();
      camera_name_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.camera_name_);
    }
    if (cached_has_bits & 0x00000002u) {
      valid_dist_ = from.valid_dist_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void CameraValidDist::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.perception.CameraValidDist)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void CameraValidDist::CopyFrom(const CameraValidDist& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.CameraValidDist)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CameraValidDist::IsInitialized() const {
  return true;
}

void CameraValidDist::Swap(CameraValidDist* other) {
  if (other == this) return;
  InternalSwap(other);
}
void CameraValidDist::InternalSwap(CameraValidDist* other) {
  using std::swap;
  camera_name_.Swap(&other->camera_name_, &::google::protobuf::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  swap(valid_dist_, other->valid_dist_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata CameraValidDist::GetMetadata() const {
  protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages];
}


// ===================================================================

void DstExistenceFusionConfig::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int DstExistenceFusionConfig::kTrackObjectMaxMatchDistanceFieldNumber;
const int DstExistenceFusionConfig::kCameraValidDistFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

DstExistenceFusionConfig::DstExistenceFusionConfig()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto::scc_info_DstExistenceFusionConfig.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.perception.DstExistenceFusionConfig)
}
DstExistenceFusionConfig::DstExistenceFusionConfig(const DstExistenceFusionConfig& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      camera_valid_dist_(from.camera_valid_dist_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  track_object_max_match_distance_ = from.track_object_max_match_distance_;
  // @@protoc_insertion_point(copy_constructor:apollo.perception.DstExistenceFusionConfig)
}

void DstExistenceFusionConfig::SharedCtor() {
  track_object_max_match_distance_ = 4;
}

DstExistenceFusionConfig::~DstExistenceFusionConfig() {
  // @@protoc_insertion_point(destructor:apollo.perception.DstExistenceFusionConfig)
  SharedDtor();
}

void DstExistenceFusionConfig::SharedDtor() {
}

void DstExistenceFusionConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* DstExistenceFusionConfig::descriptor() {
  ::protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const DstExistenceFusionConfig& DstExistenceFusionConfig::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto::scc_info_DstExistenceFusionConfig.base);
  return *internal_default_instance();
}


void DstExistenceFusionConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.DstExistenceFusionConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  camera_valid_dist_.Clear();
  track_object_max_match_distance_ = 4;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool DstExistenceFusionConfig::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.perception.DstExistenceFusionConfig)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional double track_object_max_match_distance = 1 [default = 4];
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {
          set_has_track_object_max_match_distance();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &track_object_max_match_distance_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated .apollo.perception.CameraValidDist camera_valid_dist = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
                input, add_camera_valid_dist()));
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
  // @@protoc_insertion_point(parse_success:apollo.perception.DstExistenceFusionConfig)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.perception.DstExistenceFusionConfig)
  return false;
#undef DO_
}

void DstExistenceFusionConfig::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.perception.DstExistenceFusionConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double track_object_max_match_distance = 1 [default = 4];
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->track_object_max_match_distance(), output);
  }

  // repeated .apollo.perception.CameraValidDist camera_valid_dist = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->camera_valid_dist_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2,
      this->camera_valid_dist(static_cast<int>(i)),
      output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.perception.DstExistenceFusionConfig)
}

::google::protobuf::uint8* DstExistenceFusionConfig::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.DstExistenceFusionConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double track_object_max_match_distance = 1 [default = 4];
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->track_object_max_match_distance(), target);
  }

  // repeated .apollo.perception.CameraValidDist camera_valid_dist = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->camera_valid_dist_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        2, this->camera_valid_dist(static_cast<int>(i)), deterministic, target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.DstExistenceFusionConfig)
  return target;
}

size_t DstExistenceFusionConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.DstExistenceFusionConfig)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  // repeated .apollo.perception.CameraValidDist camera_valid_dist = 2;
  {
    unsigned int count = static_cast<unsigned int>(this->camera_valid_dist_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->camera_valid_dist(static_cast<int>(i)));
    }
  }

  // optional double track_object_max_match_distance = 1 [default = 4];
  if (has_track_object_max_match_distance()) {
    total_size += 1 + 8;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void DstExistenceFusionConfig::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.perception.DstExistenceFusionConfig)
  GOOGLE_DCHECK_NE(&from, this);
  const DstExistenceFusionConfig* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const DstExistenceFusionConfig>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.perception.DstExistenceFusionConfig)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.perception.DstExistenceFusionConfig)
    MergeFrom(*source);
  }
}

void DstExistenceFusionConfig::MergeFrom(const DstExistenceFusionConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.DstExistenceFusionConfig)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  camera_valid_dist_.MergeFrom(from.camera_valid_dist_);
  if (from.has_track_object_max_match_distance()) {
    set_track_object_max_match_distance(from.track_object_max_match_distance());
  }
}

void DstExistenceFusionConfig::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.perception.DstExistenceFusionConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void DstExistenceFusionConfig::CopyFrom(const DstExistenceFusionConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.DstExistenceFusionConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool DstExistenceFusionConfig::IsInitialized() const {
  return true;
}

void DstExistenceFusionConfig::Swap(DstExistenceFusionConfig* other) {
  if (other == this) return;
  InternalSwap(other);
}
void DstExistenceFusionConfig::InternalSwap(DstExistenceFusionConfig* other) {
  using std::swap;
  CastToBase(&camera_valid_dist_)->InternalSwap(CastToBase(&other->camera_valid_dist_));
  swap(track_object_max_match_distance_, other->track_object_max_match_distance_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata DstExistenceFusionConfig::GetMetadata() const {
  protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace perception
}  // namespace apollo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::perception::CameraValidDist* Arena::CreateMaybeMessage< ::apollo::perception::CameraValidDist >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::perception::CameraValidDist >(arena);
}
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::perception::DstExistenceFusionConfig* Arena::CreateMaybeMessage< ::apollo::perception::DstExistenceFusionConfig >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::perception::DstExistenceFusionConfig >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)