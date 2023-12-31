// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/drivers/proto/pointcloud.proto

#include "modules/drivers/proto/pointcloud.pb.h"

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
namespace protobuf_modules_2fdrivers_2fproto_2fpointcloud_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fdrivers_2fproto_2fpointcloud_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_PointXYZIT;
}  // namespace protobuf_modules_2fdrivers_2fproto_2fpointcloud_2eproto
namespace apollo {
namespace drivers {
class PointXYZITDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<PointXYZIT>
      _instance;
} _PointXYZIT_default_instance_;
class PointCloudDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<PointCloud>
      _instance;
} _PointCloud_default_instance_;
}  // namespace drivers
}  // namespace apollo
namespace protobuf_modules_2fdrivers_2fproto_2fpointcloud_2eproto {
static void InitDefaultsPointXYZIT() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::drivers::_PointXYZIT_default_instance_;
    new (ptr) ::apollo::drivers::PointXYZIT();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::drivers::PointXYZIT::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_PointXYZIT =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsPointXYZIT}, {}};

static void InitDefaultsPointCloud() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::drivers::_PointCloud_default_instance_;
    new (ptr) ::apollo::drivers::PointCloud();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::drivers::PointCloud::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<2> scc_info_PointCloud =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 2, InitDefaultsPointCloud}, {
      &protobuf_modules_2fcommon_2fproto_2fheader_2eproto::scc_info_Header.base,
      &protobuf_modules_2fdrivers_2fproto_2fpointcloud_2eproto::scc_info_PointXYZIT.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_PointXYZIT.base);
  ::google::protobuf::internal::InitSCC(&scc_info_PointCloud.base);
}

::google::protobuf::Metadata file_level_metadata[2];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::PointXYZIT, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::PointXYZIT, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::PointXYZIT, x_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::PointXYZIT, y_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::PointXYZIT, z_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::PointXYZIT, intensity_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::PointXYZIT, timestamp_),
  3,
  4,
  2,
  1,
  0,
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::PointCloud, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::PointCloud, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::PointCloud, header_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::PointCloud, frame_id_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::PointCloud, is_dense_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::PointCloud, point_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::PointCloud, measurement_time_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::PointCloud, width_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::drivers::PointCloud, height_),
  1,
  0,
  2,
  ~0u,
  4,
  3,
  5,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 10, sizeof(::apollo::drivers::PointXYZIT)},
  { 15, 27, sizeof(::apollo::drivers::PointCloud)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::drivers::_PointXYZIT_default_instance_),
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::drivers::_PointCloud_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "modules/drivers/proto/pointcloud.proto", schemas, file_default_instances, TableStruct::offsets,
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
      "\n&modules/drivers/proto/pointcloud.proto"
      "\022\016apollo.drivers\032!modules/common/proto/h"
      "eader.proto\"h\n\nPointXYZIT\022\016\n\001x\030\001 \001(\002:\003na"
      "n\022\016\n\001y\030\002 \001(\002:\003nan\022\016\n\001z\030\003 \001(\002:\003nan\022\024\n\tint"
      "ensity\030\004 \001(\r:\0010\022\024\n\ttimestamp\030\005 \001(\004:\0010\"\273\001"
      "\n\nPointCloud\022%\n\006header\030\001 \001(\0132\025.apollo.co"
      "mmon.Header\022\020\n\010frame_id\030\002 \001(\t\022\020\n\010is_dens"
      "e\030\003 \001(\010\022)\n\005point\030\004 \003(\0132\032.apollo.drivers."
      "PointXYZIT\022\030\n\020measurement_time\030\005 \001(\001\022\r\n\005"
      "width\030\006 \001(\r\022\016\n\006height\030\007 \001(\r"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 387);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "modules/drivers/proto/pointcloud.proto", &protobuf_RegisterTypes);
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
}  // namespace protobuf_modules_2fdrivers_2fproto_2fpointcloud_2eproto
namespace apollo {
namespace drivers {

// ===================================================================

void PointXYZIT::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int PointXYZIT::kXFieldNumber;
const int PointXYZIT::kYFieldNumber;
const int PointXYZIT::kZFieldNumber;
const int PointXYZIT::kIntensityFieldNumber;
const int PointXYZIT::kTimestampFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

PointXYZIT::PointXYZIT()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fdrivers_2fproto_2fpointcloud_2eproto::scc_info_PointXYZIT.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.drivers.PointXYZIT)
}
PointXYZIT::PointXYZIT(const PointXYZIT& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&timestamp_, &from.timestamp_,
    static_cast<size_t>(reinterpret_cast<char*>(&y_) -
    reinterpret_cast<char*>(&timestamp_)) + sizeof(y_));
  // @@protoc_insertion_point(copy_constructor:apollo.drivers.PointXYZIT)
}

void PointXYZIT::SharedCtor() {
  ::memset(&timestamp_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&intensity_) -
      reinterpret_cast<char*>(&timestamp_)) + sizeof(intensity_));
  z_ = static_cast<float>(::google::protobuf::internal::NaN());
  x_ = static_cast<float>(::google::protobuf::internal::NaN());
  y_ = static_cast<float>(::google::protobuf::internal::NaN());
}

PointXYZIT::~PointXYZIT() {
  // @@protoc_insertion_point(destructor:apollo.drivers.PointXYZIT)
  SharedDtor();
}

void PointXYZIT::SharedDtor() {
}

void PointXYZIT::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* PointXYZIT::descriptor() {
  ::protobuf_modules_2fdrivers_2fproto_2fpointcloud_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fdrivers_2fproto_2fpointcloud_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const PointXYZIT& PointXYZIT::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fdrivers_2fproto_2fpointcloud_2eproto::scc_info_PointXYZIT.base);
  return *internal_default_instance();
}


void PointXYZIT::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.drivers.PointXYZIT)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 31u) {
    ::memset(&timestamp_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&intensity_) -
        reinterpret_cast<char*>(&timestamp_)) + sizeof(intensity_));
    z_ = static_cast<float>(::google::protobuf::internal::NaN());
    x_ = static_cast<float>(::google::protobuf::internal::NaN());
    y_ = static_cast<float>(::google::protobuf::internal::NaN());
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool PointXYZIT::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.drivers.PointXYZIT)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional float x = 1 [default = nan];
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(13u /* 13 & 0xFF */)) {
          set_has_x();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &x_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional float y = 2 [default = nan];
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(21u /* 21 & 0xFF */)) {
          set_has_y();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &y_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional float z = 3 [default = nan];
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(29u /* 29 & 0xFF */)) {
          set_has_z();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &z_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional uint32 intensity = 4 [default = 0];
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(32u /* 32 & 0xFF */)) {
          set_has_intensity();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &intensity_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional uint64 timestamp = 5 [default = 0];
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(40u /* 40 & 0xFF */)) {
          set_has_timestamp();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint64, ::google::protobuf::internal::WireFormatLite::TYPE_UINT64>(
                 input, &timestamp_)));
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
  // @@protoc_insertion_point(parse_success:apollo.drivers.PointXYZIT)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.drivers.PointXYZIT)
  return false;
#undef DO_
}

void PointXYZIT::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.drivers.PointXYZIT)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional float x = 1 [default = nan];
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(1, this->x(), output);
  }

  // optional float y = 2 [default = nan];
  if (cached_has_bits & 0x00000010u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(2, this->y(), output);
  }

  // optional float z = 3 [default = nan];
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(3, this->z(), output);
  }

  // optional uint32 intensity = 4 [default = 0];
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(4, this->intensity(), output);
  }

  // optional uint64 timestamp = 5 [default = 0];
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt64(5, this->timestamp(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.drivers.PointXYZIT)
}

::google::protobuf::uint8* PointXYZIT::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.drivers.PointXYZIT)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional float x = 1 [default = nan];
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(1, this->x(), target);
  }

  // optional float y = 2 [default = nan];
  if (cached_has_bits & 0x00000010u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(2, this->y(), target);
  }

  // optional float z = 3 [default = nan];
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(3, this->z(), target);
  }

  // optional uint32 intensity = 4 [default = 0];
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(4, this->intensity(), target);
  }

  // optional uint64 timestamp = 5 [default = 0];
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt64ToArray(5, this->timestamp(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.drivers.PointXYZIT)
  return target;
}

size_t PointXYZIT::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.drivers.PointXYZIT)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 31u) {
    // optional uint64 timestamp = 5 [default = 0];
    if (has_timestamp()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt64Size(
          this->timestamp());
    }

    // optional uint32 intensity = 4 [default = 0];
    if (has_intensity()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->intensity());
    }

    // optional float z = 3 [default = nan];
    if (has_z()) {
      total_size += 1 + 4;
    }

    // optional float x = 1 [default = nan];
    if (has_x()) {
      total_size += 1 + 4;
    }

    // optional float y = 2 [default = nan];
    if (has_y()) {
      total_size += 1 + 4;
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void PointXYZIT::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.drivers.PointXYZIT)
  GOOGLE_DCHECK_NE(&from, this);
  const PointXYZIT* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const PointXYZIT>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.drivers.PointXYZIT)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.drivers.PointXYZIT)
    MergeFrom(*source);
  }
}

void PointXYZIT::MergeFrom(const PointXYZIT& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.drivers.PointXYZIT)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 31u) {
    if (cached_has_bits & 0x00000001u) {
      timestamp_ = from.timestamp_;
    }
    if (cached_has_bits & 0x00000002u) {
      intensity_ = from.intensity_;
    }
    if (cached_has_bits & 0x00000004u) {
      z_ = from.z_;
    }
    if (cached_has_bits & 0x00000008u) {
      x_ = from.x_;
    }
    if (cached_has_bits & 0x00000010u) {
      y_ = from.y_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void PointXYZIT::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.drivers.PointXYZIT)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void PointXYZIT::CopyFrom(const PointXYZIT& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.drivers.PointXYZIT)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PointXYZIT::IsInitialized() const {
  return true;
}

void PointXYZIT::Swap(PointXYZIT* other) {
  if (other == this) return;
  InternalSwap(other);
}
void PointXYZIT::InternalSwap(PointXYZIT* other) {
  using std::swap;
  swap(timestamp_, other->timestamp_);
  swap(intensity_, other->intensity_);
  swap(z_, other->z_);
  swap(x_, other->x_);
  swap(y_, other->y_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata PointXYZIT::GetMetadata() const {
  protobuf_modules_2fdrivers_2fproto_2fpointcloud_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fdrivers_2fproto_2fpointcloud_2eproto::file_level_metadata[kIndexInFileMessages];
}


// ===================================================================

void PointCloud::InitAsDefaultInstance() {
  ::apollo::drivers::_PointCloud_default_instance_._instance.get_mutable()->header_ = const_cast< ::apollo::common::Header*>(
      ::apollo::common::Header::internal_default_instance());
}
void PointCloud::clear_header() {
  if (header_ != NULL) header_->Clear();
  clear_has_header();
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int PointCloud::kHeaderFieldNumber;
const int PointCloud::kFrameIdFieldNumber;
const int PointCloud::kIsDenseFieldNumber;
const int PointCloud::kPointFieldNumber;
const int PointCloud::kMeasurementTimeFieldNumber;
const int PointCloud::kWidthFieldNumber;
const int PointCloud::kHeightFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

PointCloud::PointCloud()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fdrivers_2fproto_2fpointcloud_2eproto::scc_info_PointCloud.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.drivers.PointCloud)
}
PointCloud::PointCloud(const PointCloud& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      point_(from.point_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  frame_id_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_frame_id()) {
    frame_id_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.frame_id_);
  }
  if (from.has_header()) {
    header_ = new ::apollo::common::Header(*from.header_);
  } else {
    header_ = NULL;
  }
  ::memcpy(&is_dense_, &from.is_dense_,
    static_cast<size_t>(reinterpret_cast<char*>(&height_) -
    reinterpret_cast<char*>(&is_dense_)) + sizeof(height_));
  // @@protoc_insertion_point(copy_constructor:apollo.drivers.PointCloud)
}

void PointCloud::SharedCtor() {
  frame_id_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  ::memset(&header_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&height_) -
      reinterpret_cast<char*>(&header_)) + sizeof(height_));
}

PointCloud::~PointCloud() {
  // @@protoc_insertion_point(destructor:apollo.drivers.PointCloud)
  SharedDtor();
}

void PointCloud::SharedDtor() {
  frame_id_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (this != internal_default_instance()) delete header_;
}

void PointCloud::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* PointCloud::descriptor() {
  ::protobuf_modules_2fdrivers_2fproto_2fpointcloud_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fdrivers_2fproto_2fpointcloud_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const PointCloud& PointCloud::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fdrivers_2fproto_2fpointcloud_2eproto::scc_info_PointCloud.base);
  return *internal_default_instance();
}


void PointCloud::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.drivers.PointCloud)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  point_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 3u) {
    if (cached_has_bits & 0x00000001u) {
      frame_id_.ClearNonDefaultToEmptyNoArena();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(header_ != NULL);
      header_->Clear();
    }
  }
  if (cached_has_bits & 60u) {
    ::memset(&is_dense_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&height_) -
        reinterpret_cast<char*>(&is_dense_)) + sizeof(height_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool PointCloud::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.drivers.PointCloud)
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

      // optional string frame_id = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_frame_id()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->frame_id().data(), static_cast<int>(this->frame_id().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "apollo.drivers.PointCloud.frame_id");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional bool is_dense = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(24u /* 24 & 0xFF */)) {
          set_has_is_dense();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &is_dense_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated .apollo.drivers.PointXYZIT point = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(34u /* 34 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
                input, add_point()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double measurement_time = 5;
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(41u /* 41 & 0xFF */)) {
          set_has_measurement_time();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &measurement_time_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional uint32 width = 6;
      case 6: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(48u /* 48 & 0xFF */)) {
          set_has_width();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &width_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional uint32 height = 7;
      case 7: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(56u /* 56 & 0xFF */)) {
          set_has_height();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &height_)));
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
  // @@protoc_insertion_point(parse_success:apollo.drivers.PointCloud)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.drivers.PointCloud)
  return false;
#undef DO_
}

void PointCloud::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.drivers.PointCloud)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.Header header = 1;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->_internal_header(), output);
  }

  // optional string frame_id = 2;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->frame_id().data(), static_cast<int>(this->frame_id().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.drivers.PointCloud.frame_id");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      2, this->frame_id(), output);
  }

  // optional bool is_dense = 3;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(3, this->is_dense(), output);
  }

  // repeated .apollo.drivers.PointXYZIT point = 4;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->point_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      4,
      this->point(static_cast<int>(i)),
      output);
  }

  // optional double measurement_time = 5;
  if (cached_has_bits & 0x00000010u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(5, this->measurement_time(), output);
  }

  // optional uint32 width = 6;
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(6, this->width(), output);
  }

  // optional uint32 height = 7;
  if (cached_has_bits & 0x00000020u) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(7, this->height(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.drivers.PointCloud)
}

::google::protobuf::uint8* PointCloud::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.drivers.PointCloud)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.Header header = 1;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->_internal_header(), deterministic, target);
  }

  // optional string frame_id = 2;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->frame_id().data(), static_cast<int>(this->frame_id().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.drivers.PointCloud.frame_id");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        2, this->frame_id(), target);
  }

  // optional bool is_dense = 3;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(3, this->is_dense(), target);
  }

  // repeated .apollo.drivers.PointXYZIT point = 4;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->point_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        4, this->point(static_cast<int>(i)), deterministic, target);
  }

  // optional double measurement_time = 5;
  if (cached_has_bits & 0x00000010u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(5, this->measurement_time(), target);
  }

  // optional uint32 width = 6;
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(6, this->width(), target);
  }

  // optional uint32 height = 7;
  if (cached_has_bits & 0x00000020u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(7, this->height(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.drivers.PointCloud)
  return target;
}

size_t PointCloud::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.drivers.PointCloud)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  // repeated .apollo.drivers.PointXYZIT point = 4;
  {
    unsigned int count = static_cast<unsigned int>(this->point_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->point(static_cast<int>(i)));
    }
  }

  if (_has_bits_[0 / 32] & 63u) {
    // optional string frame_id = 2;
    if (has_frame_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->frame_id());
    }

    // optional .apollo.common.Header header = 1;
    if (has_header()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *header_);
    }

    // optional bool is_dense = 3;
    if (has_is_dense()) {
      total_size += 1 + 1;
    }

    // optional uint32 width = 6;
    if (has_width()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->width());
    }

    // optional double measurement_time = 5;
    if (has_measurement_time()) {
      total_size += 1 + 8;
    }

    // optional uint32 height = 7;
    if (has_height()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->height());
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void PointCloud::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.drivers.PointCloud)
  GOOGLE_DCHECK_NE(&from, this);
  const PointCloud* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const PointCloud>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.drivers.PointCloud)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.drivers.PointCloud)
    MergeFrom(*source);
  }
}

void PointCloud::MergeFrom(const PointCloud& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.drivers.PointCloud)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  point_.MergeFrom(from.point_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 63u) {
    if (cached_has_bits & 0x00000001u) {
      set_has_frame_id();
      frame_id_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.frame_id_);
    }
    if (cached_has_bits & 0x00000002u) {
      mutable_header()->::apollo::common::Header::MergeFrom(from.header());
    }
    if (cached_has_bits & 0x00000004u) {
      is_dense_ = from.is_dense_;
    }
    if (cached_has_bits & 0x00000008u) {
      width_ = from.width_;
    }
    if (cached_has_bits & 0x00000010u) {
      measurement_time_ = from.measurement_time_;
    }
    if (cached_has_bits & 0x00000020u) {
      height_ = from.height_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void PointCloud::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.drivers.PointCloud)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void PointCloud::CopyFrom(const PointCloud& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.drivers.PointCloud)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PointCloud::IsInitialized() const {
  return true;
}

void PointCloud::Swap(PointCloud* other) {
  if (other == this) return;
  InternalSwap(other);
}
void PointCloud::InternalSwap(PointCloud* other) {
  using std::swap;
  CastToBase(&point_)->InternalSwap(CastToBase(&other->point_));
  frame_id_.Swap(&other->frame_id_, &::google::protobuf::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  swap(header_, other->header_);
  swap(is_dense_, other->is_dense_);
  swap(width_, other->width_);
  swap(measurement_time_, other->measurement_time_);
  swap(height_, other->height_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata PointCloud::GetMetadata() const {
  protobuf_modules_2fdrivers_2fproto_2fpointcloud_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fdrivers_2fproto_2fpointcloud_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace drivers
}  // namespace apollo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::drivers::PointXYZIT* Arena::CreateMaybeMessage< ::apollo::drivers::PointXYZIT >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::drivers::PointXYZIT >(arena);
}
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::drivers::PointCloud* Arena::CreateMaybeMessage< ::apollo::drivers::PointCloud >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::drivers::PointCloud >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
