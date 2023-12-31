// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/planning/proto/lattice_structure.proto

#include "modules/planning/proto/lattice_structure.pb.h"

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

namespace protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_StopPoint;
}  // namespace protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto
namespace apollo {
namespace planning {
class StopPointDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<StopPoint>
      _instance;
} _StopPoint_default_instance_;
class PlanningTargetDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<PlanningTarget>
      _instance;
} _PlanningTarget_default_instance_;
}  // namespace planning
}  // namespace apollo
namespace protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto {
static void InitDefaultsStopPoint() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::planning::_StopPoint_default_instance_;
    new (ptr) ::apollo::planning::StopPoint();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::planning::StopPoint::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_StopPoint =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsStopPoint}, {}};

static void InitDefaultsPlanningTarget() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::planning::_PlanningTarget_default_instance_;
    new (ptr) ::apollo::planning::PlanningTarget();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::planning::PlanningTarget::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<1> scc_info_PlanningTarget =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 1, InitDefaultsPlanningTarget}, {
      &protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto::scc_info_StopPoint.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_StopPoint.base);
  ::google::protobuf::internal::InitSCC(&scc_info_PlanningTarget.base);
}

::google::protobuf::Metadata file_level_metadata[2];
const ::google::protobuf::EnumDescriptor* file_level_enum_descriptors[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::StopPoint, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::StopPoint, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::StopPoint, s_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::StopPoint, type_),
  0,
  1,
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::PlanningTarget, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::PlanningTarget, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::PlanningTarget, stop_point_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::PlanningTarget, cruise_speed_),
  0,
  1,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, sizeof(::apollo::planning::StopPoint)},
  { 9, 16, sizeof(::apollo::planning::PlanningTarget)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::planning::_StopPoint_default_instance_),
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::planning::_PlanningTarget_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "modules/planning/proto/lattice_structure.proto", schemas, file_default_instances, TableStruct::offsets,
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
      "\n.modules/planning/proto/lattice_structu"
      "re.proto\022\017apollo.planning\"g\n\tStopPoint\022\t"
      "\n\001s\030\001 \001(\001\0223\n\004type\030\002 \001(\0162\037.apollo.plannin"
      "g.StopPoint.Type:\004HARD\"\032\n\004Type\022\010\n\004HARD\020\000"
      "\022\010\n\004SOFT\020\001\"V\n\016PlanningTarget\022.\n\nstop_poi"
      "nt\030\001 \001(\0132\032.apollo.planning.StopPoint\022\024\n\014"
      "cruise_speed\030\002 \001(\001"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 258);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "modules/planning/proto/lattice_structure.proto", &protobuf_RegisterTypes);
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
}  // namespace protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto
namespace apollo {
namespace planning {
const ::google::protobuf::EnumDescriptor* StopPoint_Type_descriptor() {
  protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto::file_level_enum_descriptors[0];
}
bool StopPoint_Type_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
      return true;
    default:
      return false;
  }
}

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const StopPoint_Type StopPoint::HARD;
const StopPoint_Type StopPoint::SOFT;
const StopPoint_Type StopPoint::Type_MIN;
const StopPoint_Type StopPoint::Type_MAX;
const int StopPoint::Type_ARRAYSIZE;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

// ===================================================================

void StopPoint::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int StopPoint::kSFieldNumber;
const int StopPoint::kTypeFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

StopPoint::StopPoint()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto::scc_info_StopPoint.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.planning.StopPoint)
}
StopPoint::StopPoint(const StopPoint& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&s_, &from.s_,
    static_cast<size_t>(reinterpret_cast<char*>(&type_) -
    reinterpret_cast<char*>(&s_)) + sizeof(type_));
  // @@protoc_insertion_point(copy_constructor:apollo.planning.StopPoint)
}

void StopPoint::SharedCtor() {
  ::memset(&s_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&type_) -
      reinterpret_cast<char*>(&s_)) + sizeof(type_));
}

StopPoint::~StopPoint() {
  // @@protoc_insertion_point(destructor:apollo.planning.StopPoint)
  SharedDtor();
}

void StopPoint::SharedDtor() {
}

void StopPoint::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* StopPoint::descriptor() {
  ::protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const StopPoint& StopPoint::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto::scc_info_StopPoint.base);
  return *internal_default_instance();
}


void StopPoint::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.planning.StopPoint)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 3u) {
    ::memset(&s_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&type_) -
        reinterpret_cast<char*>(&s_)) + sizeof(type_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool StopPoint::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.planning.StopPoint)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional double s = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {
          set_has_s();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &s_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.planning.StopPoint.Type type = 2 [default = HARD];
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(16u /* 16 & 0xFF */)) {
          int value;
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   int, ::google::protobuf::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          if (::apollo::planning::StopPoint_Type_IsValid(value)) {
            set_type(static_cast< ::apollo::planning::StopPoint_Type >(value));
          } else {
            mutable_unknown_fields()->AddVarint(
                2, static_cast< ::google::protobuf::uint64>(value));
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
  // @@protoc_insertion_point(parse_success:apollo.planning.StopPoint)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.planning.StopPoint)
  return false;
#undef DO_
}

void StopPoint::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.planning.StopPoint)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double s = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->s(), output);
  }

  // optional .apollo.planning.StopPoint.Type type = 2 [default = HARD];
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteEnum(
      2, this->type(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.planning.StopPoint)
}

::google::protobuf::uint8* StopPoint::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.planning.StopPoint)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double s = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->s(), target);
  }

  // optional .apollo.planning.StopPoint.Type type = 2 [default = HARD];
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteEnumToArray(
      2, this->type(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.planning.StopPoint)
  return target;
}

size_t StopPoint::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.planning.StopPoint)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 3u) {
    // optional double s = 1;
    if (has_s()) {
      total_size += 1 + 8;
    }

    // optional .apollo.planning.StopPoint.Type type = 2 [default = HARD];
    if (has_type()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::EnumSize(this->type());
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void StopPoint::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.planning.StopPoint)
  GOOGLE_DCHECK_NE(&from, this);
  const StopPoint* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const StopPoint>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.planning.StopPoint)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.planning.StopPoint)
    MergeFrom(*source);
  }
}

void StopPoint::MergeFrom(const StopPoint& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.planning.StopPoint)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 3u) {
    if (cached_has_bits & 0x00000001u) {
      s_ = from.s_;
    }
    if (cached_has_bits & 0x00000002u) {
      type_ = from.type_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void StopPoint::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.planning.StopPoint)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void StopPoint::CopyFrom(const StopPoint& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.planning.StopPoint)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool StopPoint::IsInitialized() const {
  return true;
}

void StopPoint::Swap(StopPoint* other) {
  if (other == this) return;
  InternalSwap(other);
}
void StopPoint::InternalSwap(StopPoint* other) {
  using std::swap;
  swap(s_, other->s_);
  swap(type_, other->type_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata StopPoint::GetMetadata() const {
  protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto::file_level_metadata[kIndexInFileMessages];
}


// ===================================================================

void PlanningTarget::InitAsDefaultInstance() {
  ::apollo::planning::_PlanningTarget_default_instance_._instance.get_mutable()->stop_point_ = const_cast< ::apollo::planning::StopPoint*>(
      ::apollo::planning::StopPoint::internal_default_instance());
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int PlanningTarget::kStopPointFieldNumber;
const int PlanningTarget::kCruiseSpeedFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

PlanningTarget::PlanningTarget()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto::scc_info_PlanningTarget.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.planning.PlanningTarget)
}
PlanningTarget::PlanningTarget(const PlanningTarget& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_stop_point()) {
    stop_point_ = new ::apollo::planning::StopPoint(*from.stop_point_);
  } else {
    stop_point_ = NULL;
  }
  cruise_speed_ = from.cruise_speed_;
  // @@protoc_insertion_point(copy_constructor:apollo.planning.PlanningTarget)
}

void PlanningTarget::SharedCtor() {
  ::memset(&stop_point_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&cruise_speed_) -
      reinterpret_cast<char*>(&stop_point_)) + sizeof(cruise_speed_));
}

PlanningTarget::~PlanningTarget() {
  // @@protoc_insertion_point(destructor:apollo.planning.PlanningTarget)
  SharedDtor();
}

void PlanningTarget::SharedDtor() {
  if (this != internal_default_instance()) delete stop_point_;
}

void PlanningTarget::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* PlanningTarget::descriptor() {
  ::protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const PlanningTarget& PlanningTarget::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto::scc_info_PlanningTarget.base);
  return *internal_default_instance();
}


void PlanningTarget::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.planning.PlanningTarget)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(stop_point_ != NULL);
    stop_point_->Clear();
  }
  cruise_speed_ = 0;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool PlanningTarget::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.planning.PlanningTarget)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .apollo.planning.StopPoint stop_point = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_stop_point()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double cruise_speed = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(17u /* 17 & 0xFF */)) {
          set_has_cruise_speed();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &cruise_speed_)));
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
  // @@protoc_insertion_point(parse_success:apollo.planning.PlanningTarget)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.planning.PlanningTarget)
  return false;
#undef DO_
}

void PlanningTarget::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.planning.PlanningTarget)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.planning.StopPoint stop_point = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->_internal_stop_point(), output);
  }

  // optional double cruise_speed = 2;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->cruise_speed(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.planning.PlanningTarget)
}

::google::protobuf::uint8* PlanningTarget::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.planning.PlanningTarget)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.planning.StopPoint stop_point = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->_internal_stop_point(), deterministic, target);
  }

  // optional double cruise_speed = 2;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->cruise_speed(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.planning.PlanningTarget)
  return target;
}

size_t PlanningTarget::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.planning.PlanningTarget)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 3u) {
    // optional .apollo.planning.StopPoint stop_point = 1;
    if (has_stop_point()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *stop_point_);
    }

    // optional double cruise_speed = 2;
    if (has_cruise_speed()) {
      total_size += 1 + 8;
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void PlanningTarget::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.planning.PlanningTarget)
  GOOGLE_DCHECK_NE(&from, this);
  const PlanningTarget* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const PlanningTarget>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.planning.PlanningTarget)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.planning.PlanningTarget)
    MergeFrom(*source);
  }
}

void PlanningTarget::MergeFrom(const PlanningTarget& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.planning.PlanningTarget)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 3u) {
    if (cached_has_bits & 0x00000001u) {
      mutable_stop_point()->::apollo::planning::StopPoint::MergeFrom(from.stop_point());
    }
    if (cached_has_bits & 0x00000002u) {
      cruise_speed_ = from.cruise_speed_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void PlanningTarget::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.planning.PlanningTarget)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void PlanningTarget::CopyFrom(const PlanningTarget& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.planning.PlanningTarget)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PlanningTarget::IsInitialized() const {
  return true;
}

void PlanningTarget::Swap(PlanningTarget* other) {
  if (other == this) return;
  InternalSwap(other);
}
void PlanningTarget::InternalSwap(PlanningTarget* other) {
  using std::swap;
  swap(stop_point_, other->stop_point_);
  swap(cruise_speed_, other->cruise_speed_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata PlanningTarget::GetMetadata() const {
  protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace planning
}  // namespace apollo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::planning::StopPoint* Arena::CreateMaybeMessage< ::apollo::planning::StopPoint >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::planning::StopPoint >(arena);
}
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::planning::PlanningTarget* Arena::CreateMaybeMessage< ::apollo::planning::PlanningTarget >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::planning::PlanningTarget >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
