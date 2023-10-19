// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/planning/proto/planning_stats.proto

#include "modules/planning/proto/planning_stats.pb.h"

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

namespace protobuf_modules_2fplanning_2fproto_2fplanning_5fstats_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fplanning_2fproto_2fplanning_5fstats_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_StatsGroup;
}  // namespace protobuf_modules_2fplanning_2fproto_2fplanning_5fstats_2eproto
namespace apollo {
namespace planning {
class StatsGroupDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<StatsGroup>
      _instance;
} _StatsGroup_default_instance_;
class PlanningStatsDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<PlanningStats>
      _instance;
} _PlanningStats_default_instance_;
}  // namespace planning
}  // namespace apollo
namespace protobuf_modules_2fplanning_2fproto_2fplanning_5fstats_2eproto {
static void InitDefaultsStatsGroup() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::planning::_StatsGroup_default_instance_;
    new (ptr) ::apollo::planning::StatsGroup();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::planning::StatsGroup::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_StatsGroup =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsStatsGroup}, {}};

static void InitDefaultsPlanningStats() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::planning::_PlanningStats_default_instance_;
    new (ptr) ::apollo::planning::PlanningStats();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::planning::PlanningStats::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<1> scc_info_PlanningStats =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 1, InitDefaultsPlanningStats}, {
      &protobuf_modules_2fplanning_2fproto_2fplanning_5fstats_2eproto::scc_info_StatsGroup.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_StatsGroup.base);
  ::google::protobuf::internal::InitSCC(&scc_info_PlanningStats.base);
}

::google::protobuf::Metadata file_level_metadata[2];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::StatsGroup, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::StatsGroup, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::StatsGroup, max_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::StatsGroup, min_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::StatsGroup, sum_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::StatsGroup, avg_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::StatsGroup, num_),
  0,
  4,
  1,
  2,
  3,
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::PlanningStats, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::PlanningStats, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::PlanningStats, total_path_length_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::PlanningStats, total_path_time_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::PlanningStats, v_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::PlanningStats, a_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::PlanningStats, kappa_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::planning::PlanningStats, dkappa_),
  0,
  1,
  2,
  3,
  4,
  5,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 10, sizeof(::apollo::planning::StatsGroup)},
  { 15, 26, sizeof(::apollo::planning::PlanningStats)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::planning::_StatsGroup_default_instance_),
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::planning::_PlanningStats_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "modules/planning/proto/planning_stats.proto", schemas, file_default_instances, TableStruct::offsets,
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
      "\n+modules/planning/proto/planning_stats."
      "proto\022\017apollo.planning\"Z\n\nStatsGroup\022\013\n\003"
      "max\030\001 \001(\001\022\030\n\003min\030\002 \001(\001:\01310000000000\022\013\n\003s"
      "um\030\003 \001(\001\022\013\n\003avg\030\004 \001(\001\022\013\n\003num\030\005 \001(\005\"\246\002\n\rP"
      "lanningStats\0226\n\021total_path_length\030\001 \001(\0132"
      "\033.apollo.planning.StatsGroup\0224\n\017total_pa"
      "th_time\030\002 \001(\0132\033.apollo.planning.StatsGro"
      "up\022&\n\001v\030\003 \001(\0132\033.apollo.planning.StatsGro"
      "up\022&\n\001a\030\004 \001(\0132\033.apollo.planning.StatsGro"
      "up\022*\n\005kappa\030\005 \001(\0132\033.apollo.planning.Stat"
      "sGroup\022+\n\006dkappa\030\006 \001(\0132\033.apollo.planning"
      ".StatsGroup"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 451);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "modules/planning/proto/planning_stats.proto", &protobuf_RegisterTypes);
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
}  // namespace protobuf_modules_2fplanning_2fproto_2fplanning_5fstats_2eproto
namespace apollo {
namespace planning {

// ===================================================================

void StatsGroup::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int StatsGroup::kMaxFieldNumber;
const int StatsGroup::kMinFieldNumber;
const int StatsGroup::kSumFieldNumber;
const int StatsGroup::kAvgFieldNumber;
const int StatsGroup::kNumFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

StatsGroup::StatsGroup()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fplanning_2fproto_2fplanning_5fstats_2eproto::scc_info_StatsGroup.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.planning.StatsGroup)
}
StatsGroup::StatsGroup(const StatsGroup& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&max_, &from.max_,
    static_cast<size_t>(reinterpret_cast<char*>(&min_) -
    reinterpret_cast<char*>(&max_)) + sizeof(min_));
  // @@protoc_insertion_point(copy_constructor:apollo.planning.StatsGroup)
}

void StatsGroup::SharedCtor() {
  ::memset(&max_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&num_) -
      reinterpret_cast<char*>(&max_)) + sizeof(num_));
  min_ = 10000000000;
}

StatsGroup::~StatsGroup() {
  // @@protoc_insertion_point(destructor:apollo.planning.StatsGroup)
  SharedDtor();
}

void StatsGroup::SharedDtor() {
}

void StatsGroup::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* StatsGroup::descriptor() {
  ::protobuf_modules_2fplanning_2fproto_2fplanning_5fstats_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fplanning_2fproto_2fplanning_5fstats_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const StatsGroup& StatsGroup::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fplanning_2fproto_2fplanning_5fstats_2eproto::scc_info_StatsGroup.base);
  return *internal_default_instance();
}


void StatsGroup::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.planning.StatsGroup)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 31u) {
    ::memset(&max_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&num_) -
        reinterpret_cast<char*>(&max_)) + sizeof(num_));
    min_ = 10000000000;
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool StatsGroup::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.planning.StatsGroup)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional double max = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {
          set_has_max();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &max_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double min = 2 [default = 10000000000];
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(17u /* 17 & 0xFF */)) {
          set_has_min();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &min_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double sum = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(25u /* 25 & 0xFF */)) {
          set_has_sum();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &sum_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double avg = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(33u /* 33 & 0xFF */)) {
          set_has_avg();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &avg_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional int32 num = 5;
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(40u /* 40 & 0xFF */)) {
          set_has_num();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &num_)));
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
  // @@protoc_insertion_point(parse_success:apollo.planning.StatsGroup)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.planning.StatsGroup)
  return false;
#undef DO_
}

void StatsGroup::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.planning.StatsGroup)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double max = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->max(), output);
  }

  // optional double min = 2 [default = 10000000000];
  if (cached_has_bits & 0x00000010u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->min(), output);
  }

  // optional double sum = 3;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->sum(), output);
  }

  // optional double avg = 4;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(4, this->avg(), output);
  }

  // optional int32 num = 5;
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(5, this->num(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.planning.StatsGroup)
}

::google::protobuf::uint8* StatsGroup::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.planning.StatsGroup)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double max = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->max(), target);
  }

  // optional double min = 2 [default = 10000000000];
  if (cached_has_bits & 0x00000010u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->min(), target);
  }

  // optional double sum = 3;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->sum(), target);
  }

  // optional double avg = 4;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(4, this->avg(), target);
  }

  // optional int32 num = 5;
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(5, this->num(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.planning.StatsGroup)
  return target;
}

size_t StatsGroup::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.planning.StatsGroup)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 31u) {
    // optional double max = 1;
    if (has_max()) {
      total_size += 1 + 8;
    }

    // optional double sum = 3;
    if (has_sum()) {
      total_size += 1 + 8;
    }

    // optional double avg = 4;
    if (has_avg()) {
      total_size += 1 + 8;
    }

    // optional int32 num = 5;
    if (has_num()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->num());
    }

    // optional double min = 2 [default = 10000000000];
    if (has_min()) {
      total_size += 1 + 8;
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void StatsGroup::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.planning.StatsGroup)
  GOOGLE_DCHECK_NE(&from, this);
  const StatsGroup* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const StatsGroup>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.planning.StatsGroup)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.planning.StatsGroup)
    MergeFrom(*source);
  }
}

void StatsGroup::MergeFrom(const StatsGroup& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.planning.StatsGroup)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 31u) {
    if (cached_has_bits & 0x00000001u) {
      max_ = from.max_;
    }
    if (cached_has_bits & 0x00000002u) {
      sum_ = from.sum_;
    }
    if (cached_has_bits & 0x00000004u) {
      avg_ = from.avg_;
    }
    if (cached_has_bits & 0x00000008u) {
      num_ = from.num_;
    }
    if (cached_has_bits & 0x00000010u) {
      min_ = from.min_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void StatsGroup::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.planning.StatsGroup)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void StatsGroup::CopyFrom(const StatsGroup& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.planning.StatsGroup)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool StatsGroup::IsInitialized() const {
  return true;
}

void StatsGroup::Swap(StatsGroup* other) {
  if (other == this) return;
  InternalSwap(other);
}
void StatsGroup::InternalSwap(StatsGroup* other) {
  using std::swap;
  swap(max_, other->max_);
  swap(sum_, other->sum_);
  swap(avg_, other->avg_);
  swap(num_, other->num_);
  swap(min_, other->min_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata StatsGroup::GetMetadata() const {
  protobuf_modules_2fplanning_2fproto_2fplanning_5fstats_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fplanning_2fproto_2fplanning_5fstats_2eproto::file_level_metadata[kIndexInFileMessages];
}


// ===================================================================

void PlanningStats::InitAsDefaultInstance() {
  ::apollo::planning::_PlanningStats_default_instance_._instance.get_mutable()->total_path_length_ = const_cast< ::apollo::planning::StatsGroup*>(
      ::apollo::planning::StatsGroup::internal_default_instance());
  ::apollo::planning::_PlanningStats_default_instance_._instance.get_mutable()->total_path_time_ = const_cast< ::apollo::planning::StatsGroup*>(
      ::apollo::planning::StatsGroup::internal_default_instance());
  ::apollo::planning::_PlanningStats_default_instance_._instance.get_mutable()->v_ = const_cast< ::apollo::planning::StatsGroup*>(
      ::apollo::planning::StatsGroup::internal_default_instance());
  ::apollo::planning::_PlanningStats_default_instance_._instance.get_mutable()->a_ = const_cast< ::apollo::planning::StatsGroup*>(
      ::apollo::planning::StatsGroup::internal_default_instance());
  ::apollo::planning::_PlanningStats_default_instance_._instance.get_mutable()->kappa_ = const_cast< ::apollo::planning::StatsGroup*>(
      ::apollo::planning::StatsGroup::internal_default_instance());
  ::apollo::planning::_PlanningStats_default_instance_._instance.get_mutable()->dkappa_ = const_cast< ::apollo::planning::StatsGroup*>(
      ::apollo::planning::StatsGroup::internal_default_instance());
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int PlanningStats::kTotalPathLengthFieldNumber;
const int PlanningStats::kTotalPathTimeFieldNumber;
const int PlanningStats::kVFieldNumber;
const int PlanningStats::kAFieldNumber;
const int PlanningStats::kKappaFieldNumber;
const int PlanningStats::kDkappaFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

PlanningStats::PlanningStats()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fplanning_2fproto_2fplanning_5fstats_2eproto::scc_info_PlanningStats.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.planning.PlanningStats)
}
PlanningStats::PlanningStats(const PlanningStats& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_total_path_length()) {
    total_path_length_ = new ::apollo::planning::StatsGroup(*from.total_path_length_);
  } else {
    total_path_length_ = NULL;
  }
  if (from.has_total_path_time()) {
    total_path_time_ = new ::apollo::planning::StatsGroup(*from.total_path_time_);
  } else {
    total_path_time_ = NULL;
  }
  if (from.has_v()) {
    v_ = new ::apollo::planning::StatsGroup(*from.v_);
  } else {
    v_ = NULL;
  }
  if (from.has_a()) {
    a_ = new ::apollo::planning::StatsGroup(*from.a_);
  } else {
    a_ = NULL;
  }
  if (from.has_kappa()) {
    kappa_ = new ::apollo::planning::StatsGroup(*from.kappa_);
  } else {
    kappa_ = NULL;
  }
  if (from.has_dkappa()) {
    dkappa_ = new ::apollo::planning::StatsGroup(*from.dkappa_);
  } else {
    dkappa_ = NULL;
  }
  // @@protoc_insertion_point(copy_constructor:apollo.planning.PlanningStats)
}

void PlanningStats::SharedCtor() {
  ::memset(&total_path_length_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&dkappa_) -
      reinterpret_cast<char*>(&total_path_length_)) + sizeof(dkappa_));
}

PlanningStats::~PlanningStats() {
  // @@protoc_insertion_point(destructor:apollo.planning.PlanningStats)
  SharedDtor();
}

void PlanningStats::SharedDtor() {
  if (this != internal_default_instance()) delete total_path_length_;
  if (this != internal_default_instance()) delete total_path_time_;
  if (this != internal_default_instance()) delete v_;
  if (this != internal_default_instance()) delete a_;
  if (this != internal_default_instance()) delete kappa_;
  if (this != internal_default_instance()) delete dkappa_;
}

void PlanningStats::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* PlanningStats::descriptor() {
  ::protobuf_modules_2fplanning_2fproto_2fplanning_5fstats_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fplanning_2fproto_2fplanning_5fstats_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const PlanningStats& PlanningStats::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fplanning_2fproto_2fplanning_5fstats_2eproto::scc_info_PlanningStats.base);
  return *internal_default_instance();
}


void PlanningStats::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.planning.PlanningStats)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 63u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(total_path_length_ != NULL);
      total_path_length_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(total_path_time_ != NULL);
      total_path_time_->Clear();
    }
    if (cached_has_bits & 0x00000004u) {
      GOOGLE_DCHECK(v_ != NULL);
      v_->Clear();
    }
    if (cached_has_bits & 0x00000008u) {
      GOOGLE_DCHECK(a_ != NULL);
      a_->Clear();
    }
    if (cached_has_bits & 0x00000010u) {
      GOOGLE_DCHECK(kappa_ != NULL);
      kappa_->Clear();
    }
    if (cached_has_bits & 0x00000020u) {
      GOOGLE_DCHECK(dkappa_ != NULL);
      dkappa_->Clear();
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool PlanningStats::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.planning.PlanningStats)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .apollo.planning.StatsGroup total_path_length = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_total_path_length()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.planning.StatsGroup total_path_time = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_total_path_time()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.planning.StatsGroup v = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(26u /* 26 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_v()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.planning.StatsGroup a = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(34u /* 34 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_a()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.planning.StatsGroup kappa = 5;
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(42u /* 42 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_kappa()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.planning.StatsGroup dkappa = 6;
      case 6: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(50u /* 50 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_dkappa()));
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
  // @@protoc_insertion_point(parse_success:apollo.planning.PlanningStats)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.planning.PlanningStats)
  return false;
#undef DO_
}

void PlanningStats::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.planning.PlanningStats)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.planning.StatsGroup total_path_length = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->_internal_total_path_length(), output);
  }

  // optional .apollo.planning.StatsGroup total_path_time = 2;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, this->_internal_total_path_time(), output);
  }

  // optional .apollo.planning.StatsGroup v = 3;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3, this->_internal_v(), output);
  }

  // optional .apollo.planning.StatsGroup a = 4;
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      4, this->_internal_a(), output);
  }

  // optional .apollo.planning.StatsGroup kappa = 5;
  if (cached_has_bits & 0x00000010u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      5, this->_internal_kappa(), output);
  }

  // optional .apollo.planning.StatsGroup dkappa = 6;
  if (cached_has_bits & 0x00000020u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      6, this->_internal_dkappa(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.planning.PlanningStats)
}

::google::protobuf::uint8* PlanningStats::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.planning.PlanningStats)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.planning.StatsGroup total_path_length = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->_internal_total_path_length(), deterministic, target);
  }

  // optional .apollo.planning.StatsGroup total_path_time = 2;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        2, this->_internal_total_path_time(), deterministic, target);
  }

  // optional .apollo.planning.StatsGroup v = 3;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        3, this->_internal_v(), deterministic, target);
  }

  // optional .apollo.planning.StatsGroup a = 4;
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        4, this->_internal_a(), deterministic, target);
  }

  // optional .apollo.planning.StatsGroup kappa = 5;
  if (cached_has_bits & 0x00000010u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        5, this->_internal_kappa(), deterministic, target);
  }

  // optional .apollo.planning.StatsGroup dkappa = 6;
  if (cached_has_bits & 0x00000020u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        6, this->_internal_dkappa(), deterministic, target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.planning.PlanningStats)
  return target;
}

size_t PlanningStats::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.planning.PlanningStats)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 63u) {
    // optional .apollo.planning.StatsGroup total_path_length = 1;
    if (has_total_path_length()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *total_path_length_);
    }

    // optional .apollo.planning.StatsGroup total_path_time = 2;
    if (has_total_path_time()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *total_path_time_);
    }

    // optional .apollo.planning.StatsGroup v = 3;
    if (has_v()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *v_);
    }

    // optional .apollo.planning.StatsGroup a = 4;
    if (has_a()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *a_);
    }

    // optional .apollo.planning.StatsGroup kappa = 5;
    if (has_kappa()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *kappa_);
    }

    // optional .apollo.planning.StatsGroup dkappa = 6;
    if (has_dkappa()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *dkappa_);
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void PlanningStats::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.planning.PlanningStats)
  GOOGLE_DCHECK_NE(&from, this);
  const PlanningStats* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const PlanningStats>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.planning.PlanningStats)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.planning.PlanningStats)
    MergeFrom(*source);
  }
}

void PlanningStats::MergeFrom(const PlanningStats& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.planning.PlanningStats)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 63u) {
    if (cached_has_bits & 0x00000001u) {
      mutable_total_path_length()->::apollo::planning::StatsGroup::MergeFrom(from.total_path_length());
    }
    if (cached_has_bits & 0x00000002u) {
      mutable_total_path_time()->::apollo::planning::StatsGroup::MergeFrom(from.total_path_time());
    }
    if (cached_has_bits & 0x00000004u) {
      mutable_v()->::apollo::planning::StatsGroup::MergeFrom(from.v());
    }
    if (cached_has_bits & 0x00000008u) {
      mutable_a()->::apollo::planning::StatsGroup::MergeFrom(from.a());
    }
    if (cached_has_bits & 0x00000010u) {
      mutable_kappa()->::apollo::planning::StatsGroup::MergeFrom(from.kappa());
    }
    if (cached_has_bits & 0x00000020u) {
      mutable_dkappa()->::apollo::planning::StatsGroup::MergeFrom(from.dkappa());
    }
  }
}

void PlanningStats::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.planning.PlanningStats)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void PlanningStats::CopyFrom(const PlanningStats& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.planning.PlanningStats)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PlanningStats::IsInitialized() const {
  return true;
}

void PlanningStats::Swap(PlanningStats* other) {
  if (other == this) return;
  InternalSwap(other);
}
void PlanningStats::InternalSwap(PlanningStats* other) {
  using std::swap;
  swap(total_path_length_, other->total_path_length_);
  swap(total_path_time_, other->total_path_time_);
  swap(v_, other->v_);
  swap(a_, other->a_);
  swap(kappa_, other->kappa_);
  swap(dkappa_, other->dkappa_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata PlanningStats::GetMetadata() const {
  protobuf_modules_2fplanning_2fproto_2fplanning_5fstats_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fplanning_2fproto_2fplanning_5fstats_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace planning
}  // namespace apollo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::planning::StatsGroup* Arena::CreateMaybeMessage< ::apollo::planning::StatsGroup >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::planning::StatsGroup >(arena);
}
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::planning::PlanningStats* Arena::CreateMaybeMessage< ::apollo::planning::PlanningStats >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::planning::PlanningStats >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)