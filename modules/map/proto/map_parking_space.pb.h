// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/map/proto/map_parking_space.proto

#ifndef PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto
#define PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto

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
#include <google/protobuf/unknown_field_set.h>
#include "modules/map/proto/map_geometry.pb.h"
#include "modules/map/proto/map_id.pb.h"
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto 

namespace protobuf_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto {
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
}  // namespace protobuf_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto
namespace apollo {
namespace hdmap {
class ParkingLot;
class ParkingLotDefaultTypeInternal;
extern ParkingLotDefaultTypeInternal _ParkingLot_default_instance_;
class ParkingSpace;
class ParkingSpaceDefaultTypeInternal;
extern ParkingSpaceDefaultTypeInternal _ParkingSpace_default_instance_;
}  // namespace hdmap
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::hdmap::ParkingLot* Arena::CreateMaybeMessage<::apollo::hdmap::ParkingLot>(Arena*);
template<> ::apollo::hdmap::ParkingSpace* Arena::CreateMaybeMessage<::apollo::hdmap::ParkingSpace>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace hdmap {

// ===================================================================

class ParkingSpace : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.hdmap.ParkingSpace) */ {
 public:
  ParkingSpace();
  virtual ~ParkingSpace();

  ParkingSpace(const ParkingSpace& from);

  inline ParkingSpace& operator=(const ParkingSpace& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  ParkingSpace(ParkingSpace&& from) noexcept
    : ParkingSpace() {
    *this = ::std::move(from);
  }

  inline ParkingSpace& operator=(ParkingSpace&& from) noexcept {
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
  static const ParkingSpace& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ParkingSpace* internal_default_instance() {
    return reinterpret_cast<const ParkingSpace*>(
               &_ParkingSpace_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(ParkingSpace* other);
  friend void swap(ParkingSpace& a, ParkingSpace& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline ParkingSpace* New() const final {
    return CreateMaybeMessage<ParkingSpace>(NULL);
  }

  ParkingSpace* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<ParkingSpace>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const ParkingSpace& from);
  void MergeFrom(const ParkingSpace& from);
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
  void InternalSwap(ParkingSpace* other);
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

  // repeated .apollo.hdmap.Id overlap_id = 3;
  int overlap_id_size() const;
  void clear_overlap_id();
  static const int kOverlapIdFieldNumber = 3;
  ::apollo::hdmap::Id* mutable_overlap_id(int index);
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >*
      mutable_overlap_id();
  const ::apollo::hdmap::Id& overlap_id(int index) const;
  ::apollo::hdmap::Id* add_overlap_id();
  const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >&
      overlap_id() const;

  // optional .apollo.hdmap.Id id = 1;
  bool has_id() const;
  void clear_id();
  static const int kIdFieldNumber = 1;
  private:
  const ::apollo::hdmap::Id& _internal_id() const;
  public:
  const ::apollo::hdmap::Id& id() const;
  ::apollo::hdmap::Id* release_id();
  ::apollo::hdmap::Id* mutable_id();
  void set_allocated_id(::apollo::hdmap::Id* id);

  // optional .apollo.hdmap.Polygon polygon = 2;
  bool has_polygon() const;
  void clear_polygon();
  static const int kPolygonFieldNumber = 2;
  private:
  const ::apollo::hdmap::Polygon& _internal_polygon() const;
  public:
  const ::apollo::hdmap::Polygon& polygon() const;
  ::apollo::hdmap::Polygon* release_polygon();
  ::apollo::hdmap::Polygon* mutable_polygon();
  void set_allocated_polygon(::apollo::hdmap::Polygon* polygon);

  // optional double heading = 4;
  bool has_heading() const;
  void clear_heading();
  static const int kHeadingFieldNumber = 4;
  double heading() const;
  void set_heading(double value);

  // @@protoc_insertion_point(class_scope:apollo.hdmap.ParkingSpace)
 private:
  void set_has_id();
  void clear_has_id();
  void set_has_polygon();
  void clear_has_polygon();
  void set_has_heading();
  void clear_has_heading();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id > overlap_id_;
  ::apollo::hdmap::Id* id_;
  ::apollo::hdmap::Polygon* polygon_;
  double heading_;
  friend struct ::protobuf_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class ParkingLot : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.hdmap.ParkingLot) */ {
 public:
  ParkingLot();
  virtual ~ParkingLot();

  ParkingLot(const ParkingLot& from);

  inline ParkingLot& operator=(const ParkingLot& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  ParkingLot(ParkingLot&& from) noexcept
    : ParkingLot() {
    *this = ::std::move(from);
  }

  inline ParkingLot& operator=(ParkingLot&& from) noexcept {
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
  static const ParkingLot& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ParkingLot* internal_default_instance() {
    return reinterpret_cast<const ParkingLot*>(
               &_ParkingLot_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(ParkingLot* other);
  friend void swap(ParkingLot& a, ParkingLot& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline ParkingLot* New() const final {
    return CreateMaybeMessage<ParkingLot>(NULL);
  }

  ParkingLot* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<ParkingLot>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const ParkingLot& from);
  void MergeFrom(const ParkingLot& from);
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
  void InternalSwap(ParkingLot* other);
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

  // repeated .apollo.hdmap.Id overlap_id = 3;
  int overlap_id_size() const;
  void clear_overlap_id();
  static const int kOverlapIdFieldNumber = 3;
  ::apollo::hdmap::Id* mutable_overlap_id(int index);
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >*
      mutable_overlap_id();
  const ::apollo::hdmap::Id& overlap_id(int index) const;
  ::apollo::hdmap::Id* add_overlap_id();
  const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >&
      overlap_id() const;

  // optional .apollo.hdmap.Id id = 1;
  bool has_id() const;
  void clear_id();
  static const int kIdFieldNumber = 1;
  private:
  const ::apollo::hdmap::Id& _internal_id() const;
  public:
  const ::apollo::hdmap::Id& id() const;
  ::apollo::hdmap::Id* release_id();
  ::apollo::hdmap::Id* mutable_id();
  void set_allocated_id(::apollo::hdmap::Id* id);

  // optional .apollo.hdmap.Polygon polygon = 2;
  bool has_polygon() const;
  void clear_polygon();
  static const int kPolygonFieldNumber = 2;
  private:
  const ::apollo::hdmap::Polygon& _internal_polygon() const;
  public:
  const ::apollo::hdmap::Polygon& polygon() const;
  ::apollo::hdmap::Polygon* release_polygon();
  ::apollo::hdmap::Polygon* mutable_polygon();
  void set_allocated_polygon(::apollo::hdmap::Polygon* polygon);

  // @@protoc_insertion_point(class_scope:apollo.hdmap.ParkingLot)
 private:
  void set_has_id();
  void clear_has_id();
  void set_has_polygon();
  void clear_has_polygon();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id > overlap_id_;
  ::apollo::hdmap::Id* id_;
  ::apollo::hdmap::Polygon* polygon_;
  friend struct ::protobuf_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ParkingSpace

// optional .apollo.hdmap.Id id = 1;
inline bool ParkingSpace::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void ParkingSpace::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void ParkingSpace::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::apollo::hdmap::Id& ParkingSpace::_internal_id() const {
  return *id_;
}
inline const ::apollo::hdmap::Id& ParkingSpace::id() const {
  const ::apollo::hdmap::Id* p = id_;
  // @@protoc_insertion_point(field_get:apollo.hdmap.ParkingSpace.id)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::hdmap::Id*>(
      &::apollo::hdmap::_Id_default_instance_);
}
inline ::apollo::hdmap::Id* ParkingSpace::release_id() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.ParkingSpace.id)
  clear_has_id();
  ::apollo::hdmap::Id* temp = id_;
  id_ = NULL;
  return temp;
}
inline ::apollo::hdmap::Id* ParkingSpace::mutable_id() {
  set_has_id();
  if (id_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::hdmap::Id>(GetArenaNoVirtual());
    id_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.ParkingSpace.id)
  return id_;
}
inline void ParkingSpace::set_allocated_id(::apollo::hdmap::Id* id) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(id_);
  }
  if (id) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      id = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, id, submessage_arena);
    }
    set_has_id();
  } else {
    clear_has_id();
  }
  id_ = id;
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.ParkingSpace.id)
}

// optional .apollo.hdmap.Polygon polygon = 2;
inline bool ParkingSpace::has_polygon() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void ParkingSpace::set_has_polygon() {
  _has_bits_[0] |= 0x00000002u;
}
inline void ParkingSpace::clear_has_polygon() {
  _has_bits_[0] &= ~0x00000002u;
}
inline const ::apollo::hdmap::Polygon& ParkingSpace::_internal_polygon() const {
  return *polygon_;
}
inline const ::apollo::hdmap::Polygon& ParkingSpace::polygon() const {
  const ::apollo::hdmap::Polygon* p = polygon_;
  // @@protoc_insertion_point(field_get:apollo.hdmap.ParkingSpace.polygon)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::hdmap::Polygon*>(
      &::apollo::hdmap::_Polygon_default_instance_);
}
inline ::apollo::hdmap::Polygon* ParkingSpace::release_polygon() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.ParkingSpace.polygon)
  clear_has_polygon();
  ::apollo::hdmap::Polygon* temp = polygon_;
  polygon_ = NULL;
  return temp;
}
inline ::apollo::hdmap::Polygon* ParkingSpace::mutable_polygon() {
  set_has_polygon();
  if (polygon_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::hdmap::Polygon>(GetArenaNoVirtual());
    polygon_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.ParkingSpace.polygon)
  return polygon_;
}
inline void ParkingSpace::set_allocated_polygon(::apollo::hdmap::Polygon* polygon) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(polygon_);
  }
  if (polygon) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      polygon = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, polygon, submessage_arena);
    }
    set_has_polygon();
  } else {
    clear_has_polygon();
  }
  polygon_ = polygon;
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.ParkingSpace.polygon)
}

// repeated .apollo.hdmap.Id overlap_id = 3;
inline int ParkingSpace::overlap_id_size() const {
  return overlap_id_.size();
}
inline ::apollo::hdmap::Id* ParkingSpace::mutable_overlap_id(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.ParkingSpace.overlap_id)
  return overlap_id_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >*
ParkingSpace::mutable_overlap_id() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.ParkingSpace.overlap_id)
  return &overlap_id_;
}
inline const ::apollo::hdmap::Id& ParkingSpace::overlap_id(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.ParkingSpace.overlap_id)
  return overlap_id_.Get(index);
}
inline ::apollo::hdmap::Id* ParkingSpace::add_overlap_id() {
  // @@protoc_insertion_point(field_add:apollo.hdmap.ParkingSpace.overlap_id)
  return overlap_id_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >&
ParkingSpace::overlap_id() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.ParkingSpace.overlap_id)
  return overlap_id_;
}

// optional double heading = 4;
inline bool ParkingSpace::has_heading() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void ParkingSpace::set_has_heading() {
  _has_bits_[0] |= 0x00000004u;
}
inline void ParkingSpace::clear_has_heading() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void ParkingSpace::clear_heading() {
  heading_ = 0;
  clear_has_heading();
}
inline double ParkingSpace::heading() const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.ParkingSpace.heading)
  return heading_;
}
inline void ParkingSpace::set_heading(double value) {
  set_has_heading();
  heading_ = value;
  // @@protoc_insertion_point(field_set:apollo.hdmap.ParkingSpace.heading)
}

// -------------------------------------------------------------------

// ParkingLot

// optional .apollo.hdmap.Id id = 1;
inline bool ParkingLot::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void ParkingLot::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void ParkingLot::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::apollo::hdmap::Id& ParkingLot::_internal_id() const {
  return *id_;
}
inline const ::apollo::hdmap::Id& ParkingLot::id() const {
  const ::apollo::hdmap::Id* p = id_;
  // @@protoc_insertion_point(field_get:apollo.hdmap.ParkingLot.id)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::hdmap::Id*>(
      &::apollo::hdmap::_Id_default_instance_);
}
inline ::apollo::hdmap::Id* ParkingLot::release_id() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.ParkingLot.id)
  clear_has_id();
  ::apollo::hdmap::Id* temp = id_;
  id_ = NULL;
  return temp;
}
inline ::apollo::hdmap::Id* ParkingLot::mutable_id() {
  set_has_id();
  if (id_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::hdmap::Id>(GetArenaNoVirtual());
    id_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.ParkingLot.id)
  return id_;
}
inline void ParkingLot::set_allocated_id(::apollo::hdmap::Id* id) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(id_);
  }
  if (id) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      id = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, id, submessage_arena);
    }
    set_has_id();
  } else {
    clear_has_id();
  }
  id_ = id;
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.ParkingLot.id)
}

// optional .apollo.hdmap.Polygon polygon = 2;
inline bool ParkingLot::has_polygon() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void ParkingLot::set_has_polygon() {
  _has_bits_[0] |= 0x00000002u;
}
inline void ParkingLot::clear_has_polygon() {
  _has_bits_[0] &= ~0x00000002u;
}
inline const ::apollo::hdmap::Polygon& ParkingLot::_internal_polygon() const {
  return *polygon_;
}
inline const ::apollo::hdmap::Polygon& ParkingLot::polygon() const {
  const ::apollo::hdmap::Polygon* p = polygon_;
  // @@protoc_insertion_point(field_get:apollo.hdmap.ParkingLot.polygon)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::hdmap::Polygon*>(
      &::apollo::hdmap::_Polygon_default_instance_);
}
inline ::apollo::hdmap::Polygon* ParkingLot::release_polygon() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.ParkingLot.polygon)
  clear_has_polygon();
  ::apollo::hdmap::Polygon* temp = polygon_;
  polygon_ = NULL;
  return temp;
}
inline ::apollo::hdmap::Polygon* ParkingLot::mutable_polygon() {
  set_has_polygon();
  if (polygon_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::hdmap::Polygon>(GetArenaNoVirtual());
    polygon_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.ParkingLot.polygon)
  return polygon_;
}
inline void ParkingLot::set_allocated_polygon(::apollo::hdmap::Polygon* polygon) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(polygon_);
  }
  if (polygon) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      polygon = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, polygon, submessage_arena);
    }
    set_has_polygon();
  } else {
    clear_has_polygon();
  }
  polygon_ = polygon;
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.ParkingLot.polygon)
}

// repeated .apollo.hdmap.Id overlap_id = 3;
inline int ParkingLot::overlap_id_size() const {
  return overlap_id_.size();
}
inline ::apollo::hdmap::Id* ParkingLot::mutable_overlap_id(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.ParkingLot.overlap_id)
  return overlap_id_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >*
ParkingLot::mutable_overlap_id() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.ParkingLot.overlap_id)
  return &overlap_id_;
}
inline const ::apollo::hdmap::Id& ParkingLot::overlap_id(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.ParkingLot.overlap_id)
  return overlap_id_.Get(index);
}
inline ::apollo::hdmap::Id* ParkingLot::add_overlap_id() {
  // @@protoc_insertion_point(field_add:apollo.hdmap.ParkingLot.overlap_id)
  return overlap_id_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >&
ParkingLot::overlap_id() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.ParkingLot.overlap_id)
  return overlap_id_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace hdmap
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fparking_5fspace_2eproto