// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/planning/proto/lattice_structure.proto

#ifndef PROTOBUF_INCLUDED_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto
#define PROTOBUF_INCLUDED_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3006000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3006000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto 

namespace protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[2];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto
namespace apollo {
namespace planning {
class PlanningTarget;
class PlanningTargetDefaultTypeInternal;
extern PlanningTargetDefaultTypeInternal _PlanningTarget_default_instance_;
class StopPoint;
class StopPointDefaultTypeInternal;
extern StopPointDefaultTypeInternal _StopPoint_default_instance_;
}  // namespace planning
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::planning::PlanningTarget* Arena::CreateMaybeMessage<::apollo::planning::PlanningTarget>(Arena*);
template<> ::apollo::planning::StopPoint* Arena::CreateMaybeMessage<::apollo::planning::StopPoint>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace planning {

enum StopPoint_Type {
  StopPoint_Type_HARD = 0,
  StopPoint_Type_SOFT = 1
};
bool StopPoint_Type_IsValid(int value);
const StopPoint_Type StopPoint_Type_Type_MIN = StopPoint_Type_HARD;
const StopPoint_Type StopPoint_Type_Type_MAX = StopPoint_Type_SOFT;
const int StopPoint_Type_Type_ARRAYSIZE = StopPoint_Type_Type_MAX + 1;

const ::google::protobuf::EnumDescriptor* StopPoint_Type_descriptor();
inline const ::std::string& StopPoint_Type_Name(StopPoint_Type value) {
  return ::google::protobuf::internal::NameOfEnum(
    StopPoint_Type_descriptor(), value);
}
inline bool StopPoint_Type_Parse(
    const ::std::string& name, StopPoint_Type* value) {
  return ::google::protobuf::internal::ParseNamedEnum<StopPoint_Type>(
    StopPoint_Type_descriptor(), name, value);
}
// ===================================================================

class StopPoint : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.planning.StopPoint) */ {
 public:
  StopPoint();
  virtual ~StopPoint();

  StopPoint(const StopPoint& from);

  inline StopPoint& operator=(const StopPoint& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  StopPoint(StopPoint&& from) noexcept
    : StopPoint() {
    *this = ::std::move(from);
  }

  inline StopPoint& operator=(StopPoint&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const StopPoint& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const StopPoint* internal_default_instance() {
    return reinterpret_cast<const StopPoint*>(
               &_StopPoint_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(StopPoint* other);
  friend void swap(StopPoint& a, StopPoint& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline StopPoint* New() const final {
    return CreateMaybeMessage<StopPoint>(NULL);
  }

  StopPoint* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<StopPoint>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const StopPoint& from);
  void MergeFrom(const StopPoint& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(StopPoint* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  typedef StopPoint_Type Type;
  static const Type HARD =
    StopPoint_Type_HARD;
  static const Type SOFT =
    StopPoint_Type_SOFT;
  static inline bool Type_IsValid(int value) {
    return StopPoint_Type_IsValid(value);
  }
  static const Type Type_MIN =
    StopPoint_Type_Type_MIN;
  static const Type Type_MAX =
    StopPoint_Type_Type_MAX;
  static const int Type_ARRAYSIZE =
    StopPoint_Type_Type_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  Type_descriptor() {
    return StopPoint_Type_descriptor();
  }
  static inline const ::std::string& Type_Name(Type value) {
    return StopPoint_Type_Name(value);
  }
  static inline bool Type_Parse(const ::std::string& name,
      Type* value) {
    return StopPoint_Type_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // optional double s = 1;
  bool has_s() const;
  void clear_s();
  static const int kSFieldNumber = 1;
  double s() const;
  void set_s(double value);

  // optional .apollo.planning.StopPoint.Type type = 2 [default = HARD];
  bool has_type() const;
  void clear_type();
  static const int kTypeFieldNumber = 2;
  ::apollo::planning::StopPoint_Type type() const;
  void set_type(::apollo::planning::StopPoint_Type value);

  // @@protoc_insertion_point(class_scope:apollo.planning.StopPoint)
 private:
  void set_has_s();
  void clear_has_s();
  void set_has_type();
  void clear_has_type();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  double s_;
  int type_;
  friend struct ::protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class PlanningTarget : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.planning.PlanningTarget) */ {
 public:
  PlanningTarget();
  virtual ~PlanningTarget();

  PlanningTarget(const PlanningTarget& from);

  inline PlanningTarget& operator=(const PlanningTarget& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  PlanningTarget(PlanningTarget&& from) noexcept
    : PlanningTarget() {
    *this = ::std::move(from);
  }

  inline PlanningTarget& operator=(PlanningTarget&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const PlanningTarget& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PlanningTarget* internal_default_instance() {
    return reinterpret_cast<const PlanningTarget*>(
               &_PlanningTarget_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(PlanningTarget* other);
  friend void swap(PlanningTarget& a, PlanningTarget& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline PlanningTarget* New() const final {
    return CreateMaybeMessage<PlanningTarget>(NULL);
  }

  PlanningTarget* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<PlanningTarget>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const PlanningTarget& from);
  void MergeFrom(const PlanningTarget& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(PlanningTarget* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional .apollo.planning.StopPoint stop_point = 1;
  bool has_stop_point() const;
  void clear_stop_point();
  static const int kStopPointFieldNumber = 1;
  private:
  const ::apollo::planning::StopPoint& _internal_stop_point() const;
  public:
  const ::apollo::planning::StopPoint& stop_point() const;
  ::apollo::planning::StopPoint* release_stop_point();
  ::apollo::planning::StopPoint* mutable_stop_point();
  void set_allocated_stop_point(::apollo::planning::StopPoint* stop_point);

  // optional double cruise_speed = 2;
  bool has_cruise_speed() const;
  void clear_cruise_speed();
  static const int kCruiseSpeedFieldNumber = 2;
  double cruise_speed() const;
  void set_cruise_speed(double value);

  // @@protoc_insertion_point(class_scope:apollo.planning.PlanningTarget)
 private:
  void set_has_stop_point();
  void clear_has_stop_point();
  void set_has_cruise_speed();
  void clear_has_cruise_speed();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::apollo::planning::StopPoint* stop_point_;
  double cruise_speed_;
  friend struct ::protobuf_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// StopPoint

// optional double s = 1;
inline bool StopPoint::has_s() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void StopPoint::set_has_s() {
  _has_bits_[0] |= 0x00000001u;
}
inline void StopPoint::clear_has_s() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void StopPoint::clear_s() {
  s_ = 0;
  clear_has_s();
}
inline double StopPoint::s() const {
  // @@protoc_insertion_point(field_get:apollo.planning.StopPoint.s)
  return s_;
}
inline void StopPoint::set_s(double value) {
  set_has_s();
  s_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.StopPoint.s)
}

// optional .apollo.planning.StopPoint.Type type = 2 [default = HARD];
inline bool StopPoint::has_type() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void StopPoint::set_has_type() {
  _has_bits_[0] |= 0x00000002u;
}
inline void StopPoint::clear_has_type() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void StopPoint::clear_type() {
  type_ = 0;
  clear_has_type();
}
inline ::apollo::planning::StopPoint_Type StopPoint::type() const {
  // @@protoc_insertion_point(field_get:apollo.planning.StopPoint.type)
  return static_cast< ::apollo::planning::StopPoint_Type >(type_);
}
inline void StopPoint::set_type(::apollo::planning::StopPoint_Type value) {
  assert(::apollo::planning::StopPoint_Type_IsValid(value));
  set_has_type();
  type_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.StopPoint.type)
}

// -------------------------------------------------------------------

// PlanningTarget

// optional .apollo.planning.StopPoint stop_point = 1;
inline bool PlanningTarget::has_stop_point() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void PlanningTarget::set_has_stop_point() {
  _has_bits_[0] |= 0x00000001u;
}
inline void PlanningTarget::clear_has_stop_point() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void PlanningTarget::clear_stop_point() {
  if (stop_point_ != NULL) stop_point_->Clear();
  clear_has_stop_point();
}
inline const ::apollo::planning::StopPoint& PlanningTarget::_internal_stop_point() const {
  return *stop_point_;
}
inline const ::apollo::planning::StopPoint& PlanningTarget::stop_point() const {
  const ::apollo::planning::StopPoint* p = stop_point_;
  // @@protoc_insertion_point(field_get:apollo.planning.PlanningTarget.stop_point)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::planning::StopPoint*>(
      &::apollo::planning::_StopPoint_default_instance_);
}
inline ::apollo::planning::StopPoint* PlanningTarget::release_stop_point() {
  // @@protoc_insertion_point(field_release:apollo.planning.PlanningTarget.stop_point)
  clear_has_stop_point();
  ::apollo::planning::StopPoint* temp = stop_point_;
  stop_point_ = NULL;
  return temp;
}
inline ::apollo::planning::StopPoint* PlanningTarget::mutable_stop_point() {
  set_has_stop_point();
  if (stop_point_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::planning::StopPoint>(GetArenaNoVirtual());
    stop_point_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.planning.PlanningTarget.stop_point)
  return stop_point_;
}
inline void PlanningTarget::set_allocated_stop_point(::apollo::planning::StopPoint* stop_point) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete stop_point_;
  }
  if (stop_point) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      stop_point = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, stop_point, submessage_arena);
    }
    set_has_stop_point();
  } else {
    clear_has_stop_point();
  }
  stop_point_ = stop_point;
  // @@protoc_insertion_point(field_set_allocated:apollo.planning.PlanningTarget.stop_point)
}

// optional double cruise_speed = 2;
inline bool PlanningTarget::has_cruise_speed() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void PlanningTarget::set_has_cruise_speed() {
  _has_bits_[0] |= 0x00000002u;
}
inline void PlanningTarget::clear_has_cruise_speed() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void PlanningTarget::clear_cruise_speed() {
  cruise_speed_ = 0;
  clear_has_cruise_speed();
}
inline double PlanningTarget::cruise_speed() const {
  // @@protoc_insertion_point(field_get:apollo.planning.PlanningTarget.cruise_speed)
  return cruise_speed_;
}
inline void PlanningTarget::set_cruise_speed(double value) {
  set_has_cruise_speed();
  cruise_speed_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.PlanningTarget.cruise_speed)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace planning
}  // namespace apollo

namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::apollo::planning::StopPoint_Type> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::planning::StopPoint_Type>() {
  return ::apollo::planning::StopPoint_Type_descriptor();
}

}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_modules_2fplanning_2fproto_2flattice_5fstructure_2eproto
