// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/map/proto/map_parking_space.proto

#include "modules/map/proto/map_parking_space.pb.h"

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

namespace protobuf_modules_2fmap_2fproto_2fmap_5fgeometry_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fmap_2fproto_2fmap_5fgeometry_2eproto ::google::protobuf::internal::SCCInfo<1> scc_info_Polygon;
}  // namespace protobuf_modules_2fmap_2fproto_2fmap_5fgeometry_2eproto
namespace protobuf_modules_2fmap_2fproto_2fmap_5fid_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fmap_2fproto_2fmap_5fid_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_Id;
}  // namespace protobuf_modules_2fmap_2fproto_2fmap_5fid_2eproto
namespace apollo {
namespace hdmap {
class ParkingSpaceDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<ParkingSpace>
      _instance;
} _ParkingSpace_default_instance_;
class ParkingLotDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<ParkingLot>
      _instance;
} _ParkingLot_default_instance_;
}  // namespace hdmap
}  // namespace apollo
namespace protobuf_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto {
static void InitDefaultsParkingSpace() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::hdmap::_ParkingSpace_default_instance_;
    new (ptr) ::apollo::hdmap::ParkingSpace();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::hdmap::ParkingSpace::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<2> scc_info_ParkingSpace =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 2, InitDefaultsParkingSpace}, {
      &protobuf_modules_2fmap_2fproto_2fmap_5fid_2eproto::scc_info_Id.base,
      &protobuf_modules_2fmap_2fproto_2fmap_5fgeometry_2eproto::scc_info_Polygon.base,}};

static void InitDefaultsParkingLot() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::hdmap::_ParkingLot_default_instance_;
    new (ptr) ::apollo::hdmap::ParkingLot();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::hdmap::ParkingLot::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<2> scc_info_ParkingLot =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 2, InitDefaultsParkingLot}, {
      &protobuf_modules_2fmap_2fproto_2fmap_5fid_2eproto::scc_info_Id.base,
      &protobuf_modules_2fmap_2fproto_2fmap_5fgeometry_2eproto::scc_info_Polygon.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_ParkingSpace.base);
  ::google::protobuf::internal::InitSCC(&scc_info_ParkingLot.base);
}

::google::protobuf::Metadata file_level_metadata[2];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::hdmap::ParkingSpace, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::hdmap::ParkingSpace, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::hdmap::ParkingSpace, id_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::hdmap::ParkingSpace, polygon_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::hdmap::ParkingSpace, overlap_id_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::hdmap::ParkingSpace, heading_),
  0,
  1,
  ~0u,
  2,
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::hdmap::ParkingLot, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::hdmap::ParkingLot, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::hdmap::ParkingLot, id_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::hdmap::ParkingLot, polygon_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::hdmap::ParkingLot, overlap_id_),
  0,
  1,
  ~0u,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 9, sizeof(::apollo::hdmap::ParkingSpace)},
  { 13, 21, sizeof(::apollo::hdmap::ParkingLot)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::hdmap::_ParkingSpace_default_instance_),
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::hdmap::_ParkingLot_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "modules/map/proto/map_parking_space.proto", schemas, file_default_instances, TableStruct::offsets,
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
      "\n)modules/map/proto/map_parking_space.pr"
      "oto\022\014apollo.hdmap\032$modules/map/proto/map"
      "_geometry.proto\032\036modules/map/proto/map_i"
      "d.proto\"\213\001\n\014ParkingSpace\022\034\n\002id\030\001 \001(\0132\020.a"
      "pollo.hdmap.Id\022&\n\007polygon\030\002 \001(\0132\025.apollo"
      ".hdmap.Polygon\022$\n\noverlap_id\030\003 \003(\0132\020.apo"
      "llo.hdmap.Id\022\017\n\007heading\030\004 \001(\001\"x\n\nParking"
      "Lot\022\034\n\002id\030\001 \001(\0132\020.apollo.hdmap.Id\022&\n\007pol"
      "ygon\030\002 \001(\0132\025.apollo.hdmap.Polygon\022$\n\nove"
      "rlap_id\030\003 \003(\0132\020.apollo.hdmap.Id"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 391);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "modules/map/proto/map_parking_space.proto", &protobuf_RegisterTypes);
  ::protobuf_modules_2fmap_2fproto_2fmap_5fgeometry_2eproto::AddDescriptors();
  ::protobuf_modules_2fmap_2fproto_2fmap_5fid_2eproto::AddDescriptors();
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
}  // namespace protobuf_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto
namespace apollo {
namespace hdmap {

// ===================================================================

void ParkingSpace::InitAsDefaultInstance() {
  ::apollo::hdmap::_ParkingSpace_default_instance_._instance.get_mutable()->id_ = const_cast< ::apollo::hdmap::Id*>(
      ::apollo::hdmap::Id::internal_default_instance());
  ::apollo::hdmap::_ParkingSpace_default_instance_._instance.get_mutable()->polygon_ = const_cast< ::apollo::hdmap::Polygon*>(
      ::apollo::hdmap::Polygon::internal_default_instance());
}
void ParkingSpace::clear_id() {
  if (id_ != NULL) id_->Clear();
  clear_has_id();
}
void ParkingSpace::clear_polygon() {
  if (polygon_ != NULL) polygon_->Clear();
  clear_has_polygon();
}
void ParkingSpace::clear_overlap_id() {
  overlap_id_.Clear();
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int ParkingSpace::kIdFieldNumber;
const int ParkingSpace::kPolygonFieldNumber;
const int ParkingSpace::kOverlapIdFieldNumber;
const int ParkingSpace::kHeadingFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

ParkingSpace::ParkingSpace()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto::scc_info_ParkingSpace.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.hdmap.ParkingSpace)
}
ParkingSpace::ParkingSpace(const ParkingSpace& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      overlap_id_(from.overlap_id_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_id()) {
    id_ = new ::apollo::hdmap::Id(*from.id_);
  } else {
    id_ = NULL;
  }
  if (from.has_polygon()) {
    polygon_ = new ::apollo::hdmap::Polygon(*from.polygon_);
  } else {
    polygon_ = NULL;
  }
  heading_ = from.heading_;
  // @@protoc_insertion_point(copy_constructor:apollo.hdmap.ParkingSpace)
}

void ParkingSpace::SharedCtor() {
  ::memset(&id_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&heading_) -
      reinterpret_cast<char*>(&id_)) + sizeof(heading_));
}

ParkingSpace::~ParkingSpace() {
  // @@protoc_insertion_point(destructor:apollo.hdmap.ParkingSpace)
  SharedDtor();
}

void ParkingSpace::SharedDtor() {
  if (this != internal_default_instance()) delete id_;
  if (this != internal_default_instance()) delete polygon_;
}

void ParkingSpace::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* ParkingSpace::descriptor() {
  ::protobuf_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const ParkingSpace& ParkingSpace::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto::scc_info_ParkingSpace.base);
  return *internal_default_instance();
}


void ParkingSpace::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.hdmap.ParkingSpace)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  overlap_id_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 3u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(id_ != NULL);
      id_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(polygon_ != NULL);
      polygon_->Clear();
    }
  }
  heading_ = 0;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool ParkingSpace::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.hdmap.ParkingSpace)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .apollo.hdmap.Id id = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_id()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.hdmap.Polygon polygon = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_polygon()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated .apollo.hdmap.Id overlap_id = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(26u /* 26 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
                input, add_overlap_id()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double heading = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(33u /* 33 & 0xFF */)) {
          set_has_heading();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &heading_)));
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
  // @@protoc_insertion_point(parse_success:apollo.hdmap.ParkingSpace)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.hdmap.ParkingSpace)
  return false;
#undef DO_
}

void ParkingSpace::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.hdmap.ParkingSpace)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.hdmap.Id id = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->_internal_id(), output);
  }

  // optional .apollo.hdmap.Polygon polygon = 2;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, this->_internal_polygon(), output);
  }

  // repeated .apollo.hdmap.Id overlap_id = 3;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->overlap_id_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3,
      this->overlap_id(static_cast<int>(i)),
      output);
  }

  // optional double heading = 4;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(4, this->heading(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.hdmap.ParkingSpace)
}

::google::protobuf::uint8* ParkingSpace::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.hdmap.ParkingSpace)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.hdmap.Id id = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->_internal_id(), deterministic, target);
  }

  // optional .apollo.hdmap.Polygon polygon = 2;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        2, this->_internal_polygon(), deterministic, target);
  }

  // repeated .apollo.hdmap.Id overlap_id = 3;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->overlap_id_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        3, this->overlap_id(static_cast<int>(i)), deterministic, target);
  }

  // optional double heading = 4;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(4, this->heading(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.hdmap.ParkingSpace)
  return target;
}

size_t ParkingSpace::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.hdmap.ParkingSpace)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  // repeated .apollo.hdmap.Id overlap_id = 3;
  {
    unsigned int count = static_cast<unsigned int>(this->overlap_id_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->overlap_id(static_cast<int>(i)));
    }
  }

  if (_has_bits_[0 / 32] & 7u) {
    // optional .apollo.hdmap.Id id = 1;
    if (has_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *id_);
    }

    // optional .apollo.hdmap.Polygon polygon = 2;
    if (has_polygon()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *polygon_);
    }

    // optional double heading = 4;
    if (has_heading()) {
      total_size += 1 + 8;
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ParkingSpace::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.hdmap.ParkingSpace)
  GOOGLE_DCHECK_NE(&from, this);
  const ParkingSpace* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const ParkingSpace>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.hdmap.ParkingSpace)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.hdmap.ParkingSpace)
    MergeFrom(*source);
  }
}

void ParkingSpace::MergeFrom(const ParkingSpace& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.hdmap.ParkingSpace)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  overlap_id_.MergeFrom(from.overlap_id_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 7u) {
    if (cached_has_bits & 0x00000001u) {
      mutable_id()->::apollo::hdmap::Id::MergeFrom(from.id());
    }
    if (cached_has_bits & 0x00000002u) {
      mutable_polygon()->::apollo::hdmap::Polygon::MergeFrom(from.polygon());
    }
    if (cached_has_bits & 0x00000004u) {
      heading_ = from.heading_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void ParkingSpace::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.hdmap.ParkingSpace)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ParkingSpace::CopyFrom(const ParkingSpace& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.hdmap.ParkingSpace)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ParkingSpace::IsInitialized() const {
  return true;
}

void ParkingSpace::Swap(ParkingSpace* other) {
  if (other == this) return;
  InternalSwap(other);
}
void ParkingSpace::InternalSwap(ParkingSpace* other) {
  using std::swap;
  CastToBase(&overlap_id_)->InternalSwap(CastToBase(&other->overlap_id_));
  swap(id_, other->id_);
  swap(polygon_, other->polygon_);
  swap(heading_, other->heading_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata ParkingSpace::GetMetadata() const {
  protobuf_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto::file_level_metadata[kIndexInFileMessages];
}


// ===================================================================

void ParkingLot::InitAsDefaultInstance() {
  ::apollo::hdmap::_ParkingLot_default_instance_._instance.get_mutable()->id_ = const_cast< ::apollo::hdmap::Id*>(
      ::apollo::hdmap::Id::internal_default_instance());
  ::apollo::hdmap::_ParkingLot_default_instance_._instance.get_mutable()->polygon_ = const_cast< ::apollo::hdmap::Polygon*>(
      ::apollo::hdmap::Polygon::internal_default_instance());
}
void ParkingLot::clear_id() {
  if (id_ != NULL) id_->Clear();
  clear_has_id();
}
void ParkingLot::clear_polygon() {
  if (polygon_ != NULL) polygon_->Clear();
  clear_has_polygon();
}
void ParkingLot::clear_overlap_id() {
  overlap_id_.Clear();
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int ParkingLot::kIdFieldNumber;
const int ParkingLot::kPolygonFieldNumber;
const int ParkingLot::kOverlapIdFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

ParkingLot::ParkingLot()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto::scc_info_ParkingLot.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.hdmap.ParkingLot)
}
ParkingLot::ParkingLot(const ParkingLot& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      overlap_id_(from.overlap_id_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_id()) {
    id_ = new ::apollo::hdmap::Id(*from.id_);
  } else {
    id_ = NULL;
  }
  if (from.has_polygon()) {
    polygon_ = new ::apollo::hdmap::Polygon(*from.polygon_);
  } else {
    polygon_ = NULL;
  }
  // @@protoc_insertion_point(copy_constructor:apollo.hdmap.ParkingLot)
}

void ParkingLot::SharedCtor() {
  ::memset(&id_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&polygon_) -
      reinterpret_cast<char*>(&id_)) + sizeof(polygon_));
}

ParkingLot::~ParkingLot() {
  // @@protoc_insertion_point(destructor:apollo.hdmap.ParkingLot)
  SharedDtor();
}

void ParkingLot::SharedDtor() {
  if (this != internal_default_instance()) delete id_;
  if (this != internal_default_instance()) delete polygon_;
}

void ParkingLot::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* ParkingLot::descriptor() {
  ::protobuf_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const ParkingLot& ParkingLot::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto::scc_info_ParkingLot.base);
  return *internal_default_instance();
}


void ParkingLot::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.hdmap.ParkingLot)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  overlap_id_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 3u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(id_ != NULL);
      id_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(polygon_ != NULL);
      polygon_->Clear();
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool ParkingLot::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.hdmap.ParkingLot)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .apollo.hdmap.Id id = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_id()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.hdmap.Polygon polygon = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_polygon()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated .apollo.hdmap.Id overlap_id = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(26u /* 26 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
                input, add_overlap_id()));
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
  // @@protoc_insertion_point(parse_success:apollo.hdmap.ParkingLot)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.hdmap.ParkingLot)
  return false;
#undef DO_
}

void ParkingLot::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.hdmap.ParkingLot)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.hdmap.Id id = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->_internal_id(), output);
  }

  // optional .apollo.hdmap.Polygon polygon = 2;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, this->_internal_polygon(), output);
  }

  // repeated .apollo.hdmap.Id overlap_id = 3;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->overlap_id_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3,
      this->overlap_id(static_cast<int>(i)),
      output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.hdmap.ParkingLot)
}

::google::protobuf::uint8* ParkingLot::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.hdmap.ParkingLot)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.hdmap.Id id = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->_internal_id(), deterministic, target);
  }

  // optional .apollo.hdmap.Polygon polygon = 2;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        2, this->_internal_polygon(), deterministic, target);
  }

  // repeated .apollo.hdmap.Id overlap_id = 3;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->overlap_id_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        3, this->overlap_id(static_cast<int>(i)), deterministic, target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.hdmap.ParkingLot)
  return target;
}

size_t ParkingLot::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.hdmap.ParkingLot)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  // repeated .apollo.hdmap.Id overlap_id = 3;
  {
    unsigned int count = static_cast<unsigned int>(this->overlap_id_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->overlap_id(static_cast<int>(i)));
    }
  }

  if (_has_bits_[0 / 32] & 3u) {
    // optional .apollo.hdmap.Id id = 1;
    if (has_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *id_);
    }

    // optional .apollo.hdmap.Polygon polygon = 2;
    if (has_polygon()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *polygon_);
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ParkingLot::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.hdmap.ParkingLot)
  GOOGLE_DCHECK_NE(&from, this);
  const ParkingLot* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const ParkingLot>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.hdmap.ParkingLot)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.hdmap.ParkingLot)
    MergeFrom(*source);
  }
}

void ParkingLot::MergeFrom(const ParkingLot& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.hdmap.ParkingLot)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  overlap_id_.MergeFrom(from.overlap_id_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 3u) {
    if (cached_has_bits & 0x00000001u) {
      mutable_id()->::apollo::hdmap::Id::MergeFrom(from.id());
    }
    if (cached_has_bits & 0x00000002u) {
      mutable_polygon()->::apollo::hdmap::Polygon::MergeFrom(from.polygon());
    }
  }
}

void ParkingLot::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.hdmap.ParkingLot)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ParkingLot::CopyFrom(const ParkingLot& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.hdmap.ParkingLot)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ParkingLot::IsInitialized() const {
  return true;
}

void ParkingLot::Swap(ParkingLot* other) {
  if (other == this) return;
  InternalSwap(other);
}
void ParkingLot::InternalSwap(ParkingLot* other) {
  using std::swap;
  CastToBase(&overlap_id_)->InternalSwap(CastToBase(&other->overlap_id_));
  swap(id_, other->id_);
  swap(polygon_, other->polygon_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata ParkingLot::GetMetadata() const {
  protobuf_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace hdmap
}  // namespace apollo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::hdmap::ParkingSpace* Arena::CreateMaybeMessage< ::apollo::hdmap::ParkingSpace >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::hdmap::ParkingSpace >(arena);
}
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::hdmap::ParkingLot* Arena::CreateMaybeMessage< ::apollo::hdmap::ParkingLot >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::hdmap::ParkingLot >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
