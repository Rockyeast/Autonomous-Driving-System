// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/proto/roi_boundary_filter_config.proto

#include "modules/perception/proto/roi_boundary_filter_config.pb.h"

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
class ROIBoundaryFilterConfigDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<ROIBoundaryFilterConfig>
      _instance;
} _ROIBoundaryFilterConfig_default_instance_;
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
namespace protobuf_modules_2fperception_2fproto_2froi_5fboundary_5ffilter_5fconfig_2eproto {
static void InitDefaultsROIBoundaryFilterConfig() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::perception::lidar::_ROIBoundaryFilterConfig_default_instance_;
    new (ptr) ::apollo::perception::lidar::ROIBoundaryFilterConfig();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::perception::lidar::ROIBoundaryFilterConfig::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_ROIBoundaryFilterConfig =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsROIBoundaryFilterConfig}, {}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_ROIBoundaryFilterConfig.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::lidar::ROIBoundaryFilterConfig, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::lidar::ROIBoundaryFilterConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::lidar::ROIBoundaryFilterConfig, distance_to_boundary_threshold_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::lidar::ROIBoundaryFilterConfig, confidence_threshold_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::lidar::ROIBoundaryFilterConfig, cross_roi_threshold_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::perception::lidar::ROIBoundaryFilterConfig, inside_threshold_),
  0,
  1,
  2,
  3,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 9, sizeof(::apollo::perception::lidar::ROIBoundaryFilterConfig)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::perception::lidar::_ROIBoundaryFilterConfig_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "modules/perception/proto/roi_boundary_filter_config.proto", schemas, file_default_instances, TableStruct::offsets,
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
      "\n9modules/perception/proto/roi_boundary_"
      "filter_config.proto\022\027apollo.perception.l"
      "idar\"\246\001\n\027ROIBoundaryFilterConfig\022)\n\036dist"
      "ance_to_boundary_threshold\030\001 \001(\001:\0011\022!\n\024c"
      "onfidence_threshold\030\002 \001(\002:\0030.5\022 \n\023cross_"
      "roi_threshold\030\003 \001(\002:\0030.6\022\033\n\020inside_thres"
      "hold\030\004 \001(\001:\0011"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 253);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "modules/perception/proto/roi_boundary_filter_config.proto", &protobuf_RegisterTypes);
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
}  // namespace protobuf_modules_2fperception_2fproto_2froi_5fboundary_5ffilter_5fconfig_2eproto
namespace apollo {
namespace perception {
namespace lidar {

// ===================================================================

void ROIBoundaryFilterConfig::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int ROIBoundaryFilterConfig::kDistanceToBoundaryThresholdFieldNumber;
const int ROIBoundaryFilterConfig::kConfidenceThresholdFieldNumber;
const int ROIBoundaryFilterConfig::kCrossRoiThresholdFieldNumber;
const int ROIBoundaryFilterConfig::kInsideThresholdFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

ROIBoundaryFilterConfig::ROIBoundaryFilterConfig()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fperception_2fproto_2froi_5fboundary_5ffilter_5fconfig_2eproto::scc_info_ROIBoundaryFilterConfig.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.perception.lidar.ROIBoundaryFilterConfig)
}
ROIBoundaryFilterConfig::ROIBoundaryFilterConfig(const ROIBoundaryFilterConfig& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&distance_to_boundary_threshold_, &from.distance_to_boundary_threshold_,
    static_cast<size_t>(reinterpret_cast<char*>(&inside_threshold_) -
    reinterpret_cast<char*>(&distance_to_boundary_threshold_)) + sizeof(inside_threshold_));
  // @@protoc_insertion_point(copy_constructor:apollo.perception.lidar.ROIBoundaryFilterConfig)
}

void ROIBoundaryFilterConfig::SharedCtor() {
  distance_to_boundary_threshold_ = 1;
  confidence_threshold_ = 0.5f;
  cross_roi_threshold_ = 0.6f;
  inside_threshold_ = 1;
}

ROIBoundaryFilterConfig::~ROIBoundaryFilterConfig() {
  // @@protoc_insertion_point(destructor:apollo.perception.lidar.ROIBoundaryFilterConfig)
  SharedDtor();
}

void ROIBoundaryFilterConfig::SharedDtor() {
}

void ROIBoundaryFilterConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* ROIBoundaryFilterConfig::descriptor() {
  ::protobuf_modules_2fperception_2fproto_2froi_5fboundary_5ffilter_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fperception_2fproto_2froi_5fboundary_5ffilter_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const ROIBoundaryFilterConfig& ROIBoundaryFilterConfig::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fperception_2fproto_2froi_5fboundary_5ffilter_5fconfig_2eproto::scc_info_ROIBoundaryFilterConfig.base);
  return *internal_default_instance();
}


void ROIBoundaryFilterConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.lidar.ROIBoundaryFilterConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 15u) {
    distance_to_boundary_threshold_ = 1;
    confidence_threshold_ = 0.5f;
    cross_roi_threshold_ = 0.6f;
    inside_threshold_ = 1;
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool ROIBoundaryFilterConfig::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.perception.lidar.ROIBoundaryFilterConfig)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional double distance_to_boundary_threshold = 1 [default = 1];
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {
          set_has_distance_to_boundary_threshold();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &distance_to_boundary_threshold_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional float confidence_threshold = 2 [default = 0.5];
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(21u /* 21 & 0xFF */)) {
          set_has_confidence_threshold();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &confidence_threshold_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional float cross_roi_threshold = 3 [default = 0.6];
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(29u /* 29 & 0xFF */)) {
          set_has_cross_roi_threshold();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &cross_roi_threshold_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double inside_threshold = 4 [default = 1];
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(33u /* 33 & 0xFF */)) {
          set_has_inside_threshold();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &inside_threshold_)));
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
  // @@protoc_insertion_point(parse_success:apollo.perception.lidar.ROIBoundaryFilterConfig)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.perception.lidar.ROIBoundaryFilterConfig)
  return false;
#undef DO_
}

void ROIBoundaryFilterConfig::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.perception.lidar.ROIBoundaryFilterConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double distance_to_boundary_threshold = 1 [default = 1];
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->distance_to_boundary_threshold(), output);
  }

  // optional float confidence_threshold = 2 [default = 0.5];
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(2, this->confidence_threshold(), output);
  }

  // optional float cross_roi_threshold = 3 [default = 0.6];
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(3, this->cross_roi_threshold(), output);
  }

  // optional double inside_threshold = 4 [default = 1];
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(4, this->inside_threshold(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.perception.lidar.ROIBoundaryFilterConfig)
}

::google::protobuf::uint8* ROIBoundaryFilterConfig::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.lidar.ROIBoundaryFilterConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double distance_to_boundary_threshold = 1 [default = 1];
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->distance_to_boundary_threshold(), target);
  }

  // optional float confidence_threshold = 2 [default = 0.5];
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(2, this->confidence_threshold(), target);
  }

  // optional float cross_roi_threshold = 3 [default = 0.6];
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(3, this->cross_roi_threshold(), target);
  }

  // optional double inside_threshold = 4 [default = 1];
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(4, this->inside_threshold(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.lidar.ROIBoundaryFilterConfig)
  return target;
}

size_t ROIBoundaryFilterConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.lidar.ROIBoundaryFilterConfig)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 15u) {
    // optional double distance_to_boundary_threshold = 1 [default = 1];
    if (has_distance_to_boundary_threshold()) {
      total_size += 1 + 8;
    }

    // optional float confidence_threshold = 2 [default = 0.5];
    if (has_confidence_threshold()) {
      total_size += 1 + 4;
    }

    // optional float cross_roi_threshold = 3 [default = 0.6];
    if (has_cross_roi_threshold()) {
      total_size += 1 + 4;
    }

    // optional double inside_threshold = 4 [default = 1];
    if (has_inside_threshold()) {
      total_size += 1 + 8;
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ROIBoundaryFilterConfig::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.perception.lidar.ROIBoundaryFilterConfig)
  GOOGLE_DCHECK_NE(&from, this);
  const ROIBoundaryFilterConfig* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const ROIBoundaryFilterConfig>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.perception.lidar.ROIBoundaryFilterConfig)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.perception.lidar.ROIBoundaryFilterConfig)
    MergeFrom(*source);
  }
}

void ROIBoundaryFilterConfig::MergeFrom(const ROIBoundaryFilterConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.lidar.ROIBoundaryFilterConfig)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 15u) {
    if (cached_has_bits & 0x00000001u) {
      distance_to_boundary_threshold_ = from.distance_to_boundary_threshold_;
    }
    if (cached_has_bits & 0x00000002u) {
      confidence_threshold_ = from.confidence_threshold_;
    }
    if (cached_has_bits & 0x00000004u) {
      cross_roi_threshold_ = from.cross_roi_threshold_;
    }
    if (cached_has_bits & 0x00000008u) {
      inside_threshold_ = from.inside_threshold_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void ROIBoundaryFilterConfig::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.perception.lidar.ROIBoundaryFilterConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ROIBoundaryFilterConfig::CopyFrom(const ROIBoundaryFilterConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.lidar.ROIBoundaryFilterConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ROIBoundaryFilterConfig::IsInitialized() const {
  return true;
}

void ROIBoundaryFilterConfig::Swap(ROIBoundaryFilterConfig* other) {
  if (other == this) return;
  InternalSwap(other);
}
void ROIBoundaryFilterConfig::InternalSwap(ROIBoundaryFilterConfig* other) {
  using std::swap;
  swap(distance_to_boundary_threshold_, other->distance_to_boundary_threshold_);
  swap(confidence_threshold_, other->confidence_threshold_);
  swap(cross_roi_threshold_, other->cross_roi_threshold_);
  swap(inside_threshold_, other->inside_threshold_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata ROIBoundaryFilterConfig::GetMetadata() const {
  protobuf_modules_2fperception_2fproto_2froi_5fboundary_5ffilter_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fperception_2fproto_2froi_5fboundary_5ffilter_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::perception::lidar::ROIBoundaryFilterConfig* Arena::CreateMaybeMessage< ::apollo::perception::lidar::ROIBoundaryFilterConfig >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::perception::lidar::ROIBoundaryFilterConfig >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
