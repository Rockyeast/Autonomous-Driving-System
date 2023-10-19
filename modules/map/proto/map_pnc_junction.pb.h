// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/map/proto/map_pnc_junction.proto

#ifndef PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fpnc_5fjunction_2eproto
#define PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fpnc_5fjunction_2eproto

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
#include "modules/map/proto/map_id.pb.h"
#include "modules/map/proto/map_geometry.pb.h"
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fmap_2fproto_2fmap_5fpnc_5fjunction_2eproto 

namespace protobuf_modules_2fmap_2fproto_2fmap_5fpnc_5fjunction_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[3];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_modules_2fmap_2fproto_2fmap_5fpnc_5fjunction_2eproto
namespace apollo {
namespace hdmap {
class PNCJunction;
class PNCJunctionDefaultTypeInternal;
extern PNCJunctionDefaultTypeInternal _PNCJunction_default_instance_;
class Passage;
class PassageDefaultTypeInternal;
extern PassageDefaultTypeInternal _Passage_default_instance_;
class PassageGroup;
class PassageGroupDefaultTypeInternal;
extern PassageGroupDefaultTypeInternal _PassageGroup_default_instance_;
}  // namespace hdmap
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::hdmap::PNCJunction* Arena::CreateMaybeMessage<::apollo::hdmap::PNCJunction>(Arena*);
template<> ::apollo::hdmap::Passage* Arena::CreateMaybeMessage<::apollo::hdmap::Passage>(Arena*);
template<> ::apollo::hdmap::PassageGroup* Arena::CreateMaybeMessage<::apollo::hdmap::PassageGroup>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace hdmap {

enum Passage_Type {
  Passage_Type_UNKNOWN = 0,
  Passage_Type_ENTRANCE = 1,
  Passage_Type_EXIT = 2
};
bool Passage_Type_IsValid(int value);
const Passage_Type Passage_Type_Type_MIN = Passage_Type_UNKNOWN;
const Passage_Type Passage_Type_Type_MAX = Passage_Type_EXIT;
const int Passage_Type_Type_ARRAYSIZE = Passage_Type_Type_MAX + 1;

const ::google::protobuf::EnumDescriptor* Passage_Type_descriptor();
inline const ::std::string& Passage_Type_Name(Passage_Type value) {
  return ::google::protobuf::internal::NameOfEnum(
    Passage_Type_descriptor(), value);
}
inline bool Passage_Type_Parse(
    const ::std::string& name, Passage_Type* value) {
  return ::google::protobuf::internal::ParseNamedEnum<Passage_Type>(
    Passage_Type_descriptor(), name, value);
}
// ===================================================================

class Passage : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.hdmap.Passage) */ {
 public:
  Passage();
  virtual ~Passage();

  Passage(const Passage& from);

  inline Passage& operator=(const Passage& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Passage(Passage&& from) noexcept
    : Passage() {
    *this = ::std::move(from);
  }

  inline Passage& operator=(Passage&& from) noexcept {
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
  static const Passage& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Passage* internal_default_instance() {
    return reinterpret_cast<const Passage*>(
               &_Passage_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(Passage* other);
  friend void swap(Passage& a, Passage& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Passage* New() const final {
    return CreateMaybeMessage<Passage>(NULL);
  }

  Passage* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Passage>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Passage& from);
  void MergeFrom(const Passage& from);
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
  void InternalSwap(Passage* other);
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

  typedef Passage_Type Type;
  static const Type UNKNOWN =
    Passage_Type_UNKNOWN;
  static const Type ENTRANCE =
    Passage_Type_ENTRANCE;
  static const Type EXIT =
    Passage_Type_EXIT;
  static inline bool Type_IsValid(int value) {
    return Passage_Type_IsValid(value);
  }
  static const Type Type_MIN =
    Passage_Type_Type_MIN;
  static const Type Type_MAX =
    Passage_Type_Type_MAX;
  static const int Type_ARRAYSIZE =
    Passage_Type_Type_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  Type_descriptor() {
    return Passage_Type_descriptor();
  }
  static inline const ::std::string& Type_Name(Type value) {
    return Passage_Type_Name(value);
  }
  static inline bool Type_Parse(const ::std::string& name,
      Type* value) {
    return Passage_Type_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // repeated .apollo.hdmap.Id signal_id = 2;
  int signal_id_size() const;
  void clear_signal_id();
  static const int kSignalIdFieldNumber = 2;
  ::apollo::hdmap::Id* mutable_signal_id(int index);
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >*
      mutable_signal_id();
  const ::apollo::hdmap::Id& signal_id(int index) const;
  ::apollo::hdmap::Id* add_signal_id();
  const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >&
      signal_id() const;

  // repeated .apollo.hdmap.Id yield_id = 3;
  int yield_id_size() const;
  void clear_yield_id();
  static const int kYieldIdFieldNumber = 3;
  ::apollo::hdmap::Id* mutable_yield_id(int index);
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >*
      mutable_yield_id();
  const ::apollo::hdmap::Id& yield_id(int index) const;
  ::apollo::hdmap::Id* add_yield_id();
  const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >&
      yield_id() const;

  // repeated .apollo.hdmap.Id stop_sign_id = 4;
  int stop_sign_id_size() const;
  void clear_stop_sign_id();
  static const int kStopSignIdFieldNumber = 4;
  ::apollo::hdmap::Id* mutable_stop_sign_id(int index);
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >*
      mutable_stop_sign_id();
  const ::apollo::hdmap::Id& stop_sign_id(int index) const;
  ::apollo::hdmap::Id* add_stop_sign_id();
  const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >&
      stop_sign_id() const;

  // repeated .apollo.hdmap.Id lane_id = 5;
  int lane_id_size() const;
  void clear_lane_id();
  static const int kLaneIdFieldNumber = 5;
  ::apollo::hdmap::Id* mutable_lane_id(int index);
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >*
      mutable_lane_id();
  const ::apollo::hdmap::Id& lane_id(int index) const;
  ::apollo::hdmap::Id* add_lane_id();
  const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >&
      lane_id() const;

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

  // optional .apollo.hdmap.Passage.Type type = 6;
  bool has_type() const;
  void clear_type();
  static const int kTypeFieldNumber = 6;
  ::apollo::hdmap::Passage_Type type() const;
  void set_type(::apollo::hdmap::Passage_Type value);

  // @@protoc_insertion_point(class_scope:apollo.hdmap.Passage)
 private:
  void set_has_id();
  void clear_has_id();
  void set_has_type();
  void clear_has_type();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id > signal_id_;
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id > yield_id_;
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id > stop_sign_id_;
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id > lane_id_;
  ::apollo::hdmap::Id* id_;
  int type_;
  friend struct ::protobuf_modules_2fmap_2fproto_2fmap_5fpnc_5fjunction_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class PassageGroup : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.hdmap.PassageGroup) */ {
 public:
  PassageGroup();
  virtual ~PassageGroup();

  PassageGroup(const PassageGroup& from);

  inline PassageGroup& operator=(const PassageGroup& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  PassageGroup(PassageGroup&& from) noexcept
    : PassageGroup() {
    *this = ::std::move(from);
  }

  inline PassageGroup& operator=(PassageGroup&& from) noexcept {
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
  static const PassageGroup& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PassageGroup* internal_default_instance() {
    return reinterpret_cast<const PassageGroup*>(
               &_PassageGroup_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(PassageGroup* other);
  friend void swap(PassageGroup& a, PassageGroup& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline PassageGroup* New() const final {
    return CreateMaybeMessage<PassageGroup>(NULL);
  }

  PassageGroup* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<PassageGroup>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const PassageGroup& from);
  void MergeFrom(const PassageGroup& from);
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
  void InternalSwap(PassageGroup* other);
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

  // repeated .apollo.hdmap.Passage passage = 2;
  int passage_size() const;
  void clear_passage();
  static const int kPassageFieldNumber = 2;
  ::apollo::hdmap::Passage* mutable_passage(int index);
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Passage >*
      mutable_passage();
  const ::apollo::hdmap::Passage& passage(int index) const;
  ::apollo::hdmap::Passage* add_passage();
  const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Passage >&
      passage() const;

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

  // @@protoc_insertion_point(class_scope:apollo.hdmap.PassageGroup)
 private:
  void set_has_id();
  void clear_has_id();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Passage > passage_;
  ::apollo::hdmap::Id* id_;
  friend struct ::protobuf_modules_2fmap_2fproto_2fmap_5fpnc_5fjunction_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class PNCJunction : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.hdmap.PNCJunction) */ {
 public:
  PNCJunction();
  virtual ~PNCJunction();

  PNCJunction(const PNCJunction& from);

  inline PNCJunction& operator=(const PNCJunction& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  PNCJunction(PNCJunction&& from) noexcept
    : PNCJunction() {
    *this = ::std::move(from);
  }

  inline PNCJunction& operator=(PNCJunction&& from) noexcept {
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
  static const PNCJunction& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PNCJunction* internal_default_instance() {
    return reinterpret_cast<const PNCJunction*>(
               &_PNCJunction_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    2;

  void Swap(PNCJunction* other);
  friend void swap(PNCJunction& a, PNCJunction& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline PNCJunction* New() const final {
    return CreateMaybeMessage<PNCJunction>(NULL);
  }

  PNCJunction* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<PNCJunction>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const PNCJunction& from);
  void MergeFrom(const PNCJunction& from);
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
  void InternalSwap(PNCJunction* other);
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

  // repeated .apollo.hdmap.PassageGroup passage_group = 4;
  int passage_group_size() const;
  void clear_passage_group();
  static const int kPassageGroupFieldNumber = 4;
  ::apollo::hdmap::PassageGroup* mutable_passage_group(int index);
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::PassageGroup >*
      mutable_passage_group();
  const ::apollo::hdmap::PassageGroup& passage_group(int index) const;
  ::apollo::hdmap::PassageGroup* add_passage_group();
  const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::PassageGroup >&
      passage_group() const;

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

  // @@protoc_insertion_point(class_scope:apollo.hdmap.PNCJunction)
 private:
  void set_has_id();
  void clear_has_id();
  void set_has_polygon();
  void clear_has_polygon();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id > overlap_id_;
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::PassageGroup > passage_group_;
  ::apollo::hdmap::Id* id_;
  ::apollo::hdmap::Polygon* polygon_;
  friend struct ::protobuf_modules_2fmap_2fproto_2fmap_5fpnc_5fjunction_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Passage

// optional .apollo.hdmap.Id id = 1;
inline bool Passage::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Passage::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Passage::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::apollo::hdmap::Id& Passage::_internal_id() const {
  return *id_;
}
inline const ::apollo::hdmap::Id& Passage::id() const {
  const ::apollo::hdmap::Id* p = id_;
  // @@protoc_insertion_point(field_get:apollo.hdmap.Passage.id)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::hdmap::Id*>(
      &::apollo::hdmap::_Id_default_instance_);
}
inline ::apollo::hdmap::Id* Passage::release_id() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.Passage.id)
  clear_has_id();
  ::apollo::hdmap::Id* temp = id_;
  id_ = NULL;
  return temp;
}
inline ::apollo::hdmap::Id* Passage::mutable_id() {
  set_has_id();
  if (id_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::hdmap::Id>(GetArenaNoVirtual());
    id_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.Passage.id)
  return id_;
}
inline void Passage::set_allocated_id(::apollo::hdmap::Id* id) {
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
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.Passage.id)
}

// repeated .apollo.hdmap.Id signal_id = 2;
inline int Passage::signal_id_size() const {
  return signal_id_.size();
}
inline ::apollo::hdmap::Id* Passage::mutable_signal_id(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.Passage.signal_id)
  return signal_id_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >*
Passage::mutable_signal_id() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.Passage.signal_id)
  return &signal_id_;
}
inline const ::apollo::hdmap::Id& Passage::signal_id(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.Passage.signal_id)
  return signal_id_.Get(index);
}
inline ::apollo::hdmap::Id* Passage::add_signal_id() {
  // @@protoc_insertion_point(field_add:apollo.hdmap.Passage.signal_id)
  return signal_id_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >&
Passage::signal_id() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.Passage.signal_id)
  return signal_id_;
}

// repeated .apollo.hdmap.Id yield_id = 3;
inline int Passage::yield_id_size() const {
  return yield_id_.size();
}
inline ::apollo::hdmap::Id* Passage::mutable_yield_id(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.Passage.yield_id)
  return yield_id_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >*
Passage::mutable_yield_id() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.Passage.yield_id)
  return &yield_id_;
}
inline const ::apollo::hdmap::Id& Passage::yield_id(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.Passage.yield_id)
  return yield_id_.Get(index);
}
inline ::apollo::hdmap::Id* Passage::add_yield_id() {
  // @@protoc_insertion_point(field_add:apollo.hdmap.Passage.yield_id)
  return yield_id_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >&
Passage::yield_id() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.Passage.yield_id)
  return yield_id_;
}

// repeated .apollo.hdmap.Id stop_sign_id = 4;
inline int Passage::stop_sign_id_size() const {
  return stop_sign_id_.size();
}
inline ::apollo::hdmap::Id* Passage::mutable_stop_sign_id(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.Passage.stop_sign_id)
  return stop_sign_id_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >*
Passage::mutable_stop_sign_id() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.Passage.stop_sign_id)
  return &stop_sign_id_;
}
inline const ::apollo::hdmap::Id& Passage::stop_sign_id(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.Passage.stop_sign_id)
  return stop_sign_id_.Get(index);
}
inline ::apollo::hdmap::Id* Passage::add_stop_sign_id() {
  // @@protoc_insertion_point(field_add:apollo.hdmap.Passage.stop_sign_id)
  return stop_sign_id_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >&
Passage::stop_sign_id() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.Passage.stop_sign_id)
  return stop_sign_id_;
}

// repeated .apollo.hdmap.Id lane_id = 5;
inline int Passage::lane_id_size() const {
  return lane_id_.size();
}
inline ::apollo::hdmap::Id* Passage::mutable_lane_id(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.Passage.lane_id)
  return lane_id_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >*
Passage::mutable_lane_id() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.Passage.lane_id)
  return &lane_id_;
}
inline const ::apollo::hdmap::Id& Passage::lane_id(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.Passage.lane_id)
  return lane_id_.Get(index);
}
inline ::apollo::hdmap::Id* Passage::add_lane_id() {
  // @@protoc_insertion_point(field_add:apollo.hdmap.Passage.lane_id)
  return lane_id_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >&
Passage::lane_id() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.Passage.lane_id)
  return lane_id_;
}

// optional .apollo.hdmap.Passage.Type type = 6;
inline bool Passage::has_type() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Passage::set_has_type() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Passage::clear_has_type() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Passage::clear_type() {
  type_ = 0;
  clear_has_type();
}
inline ::apollo::hdmap::Passage_Type Passage::type() const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.Passage.type)
  return static_cast< ::apollo::hdmap::Passage_Type >(type_);
}
inline void Passage::set_type(::apollo::hdmap::Passage_Type value) {
  assert(::apollo::hdmap::Passage_Type_IsValid(value));
  set_has_type();
  type_ = value;
  // @@protoc_insertion_point(field_set:apollo.hdmap.Passage.type)
}

// -------------------------------------------------------------------

// PassageGroup

// optional .apollo.hdmap.Id id = 1;
inline bool PassageGroup::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void PassageGroup::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void PassageGroup::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::apollo::hdmap::Id& PassageGroup::_internal_id() const {
  return *id_;
}
inline const ::apollo::hdmap::Id& PassageGroup::id() const {
  const ::apollo::hdmap::Id* p = id_;
  // @@protoc_insertion_point(field_get:apollo.hdmap.PassageGroup.id)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::hdmap::Id*>(
      &::apollo::hdmap::_Id_default_instance_);
}
inline ::apollo::hdmap::Id* PassageGroup::release_id() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.PassageGroup.id)
  clear_has_id();
  ::apollo::hdmap::Id* temp = id_;
  id_ = NULL;
  return temp;
}
inline ::apollo::hdmap::Id* PassageGroup::mutable_id() {
  set_has_id();
  if (id_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::hdmap::Id>(GetArenaNoVirtual());
    id_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.PassageGroup.id)
  return id_;
}
inline void PassageGroup::set_allocated_id(::apollo::hdmap::Id* id) {
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
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.PassageGroup.id)
}

// repeated .apollo.hdmap.Passage passage = 2;
inline int PassageGroup::passage_size() const {
  return passage_.size();
}
inline void PassageGroup::clear_passage() {
  passage_.Clear();
}
inline ::apollo::hdmap::Passage* PassageGroup::mutable_passage(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.PassageGroup.passage)
  return passage_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Passage >*
PassageGroup::mutable_passage() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.PassageGroup.passage)
  return &passage_;
}
inline const ::apollo::hdmap::Passage& PassageGroup::passage(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.PassageGroup.passage)
  return passage_.Get(index);
}
inline ::apollo::hdmap::Passage* PassageGroup::add_passage() {
  // @@protoc_insertion_point(field_add:apollo.hdmap.PassageGroup.passage)
  return passage_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Passage >&
PassageGroup::passage() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.PassageGroup.passage)
  return passage_;
}

// -------------------------------------------------------------------

// PNCJunction

// optional .apollo.hdmap.Id id = 1;
inline bool PNCJunction::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void PNCJunction::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void PNCJunction::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::apollo::hdmap::Id& PNCJunction::_internal_id() const {
  return *id_;
}
inline const ::apollo::hdmap::Id& PNCJunction::id() const {
  const ::apollo::hdmap::Id* p = id_;
  // @@protoc_insertion_point(field_get:apollo.hdmap.PNCJunction.id)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::hdmap::Id*>(
      &::apollo::hdmap::_Id_default_instance_);
}
inline ::apollo::hdmap::Id* PNCJunction::release_id() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.PNCJunction.id)
  clear_has_id();
  ::apollo::hdmap::Id* temp = id_;
  id_ = NULL;
  return temp;
}
inline ::apollo::hdmap::Id* PNCJunction::mutable_id() {
  set_has_id();
  if (id_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::hdmap::Id>(GetArenaNoVirtual());
    id_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.PNCJunction.id)
  return id_;
}
inline void PNCJunction::set_allocated_id(::apollo::hdmap::Id* id) {
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
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.PNCJunction.id)
}

// optional .apollo.hdmap.Polygon polygon = 2;
inline bool PNCJunction::has_polygon() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void PNCJunction::set_has_polygon() {
  _has_bits_[0] |= 0x00000002u;
}
inline void PNCJunction::clear_has_polygon() {
  _has_bits_[0] &= ~0x00000002u;
}
inline const ::apollo::hdmap::Polygon& PNCJunction::_internal_polygon() const {
  return *polygon_;
}
inline const ::apollo::hdmap::Polygon& PNCJunction::polygon() const {
  const ::apollo::hdmap::Polygon* p = polygon_;
  // @@protoc_insertion_point(field_get:apollo.hdmap.PNCJunction.polygon)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::hdmap::Polygon*>(
      &::apollo::hdmap::_Polygon_default_instance_);
}
inline ::apollo::hdmap::Polygon* PNCJunction::release_polygon() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.PNCJunction.polygon)
  clear_has_polygon();
  ::apollo::hdmap::Polygon* temp = polygon_;
  polygon_ = NULL;
  return temp;
}
inline ::apollo::hdmap::Polygon* PNCJunction::mutable_polygon() {
  set_has_polygon();
  if (polygon_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::hdmap::Polygon>(GetArenaNoVirtual());
    polygon_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.PNCJunction.polygon)
  return polygon_;
}
inline void PNCJunction::set_allocated_polygon(::apollo::hdmap::Polygon* polygon) {
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
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.PNCJunction.polygon)
}

// repeated .apollo.hdmap.Id overlap_id = 3;
inline int PNCJunction::overlap_id_size() const {
  return overlap_id_.size();
}
inline ::apollo::hdmap::Id* PNCJunction::mutable_overlap_id(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.PNCJunction.overlap_id)
  return overlap_id_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >*
PNCJunction::mutable_overlap_id() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.PNCJunction.overlap_id)
  return &overlap_id_;
}
inline const ::apollo::hdmap::Id& PNCJunction::overlap_id(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.PNCJunction.overlap_id)
  return overlap_id_.Get(index);
}
inline ::apollo::hdmap::Id* PNCJunction::add_overlap_id() {
  // @@protoc_insertion_point(field_add:apollo.hdmap.PNCJunction.overlap_id)
  return overlap_id_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >&
PNCJunction::overlap_id() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.PNCJunction.overlap_id)
  return overlap_id_;
}

// repeated .apollo.hdmap.PassageGroup passage_group = 4;
inline int PNCJunction::passage_group_size() const {
  return passage_group_.size();
}
inline void PNCJunction::clear_passage_group() {
  passage_group_.Clear();
}
inline ::apollo::hdmap::PassageGroup* PNCJunction::mutable_passage_group(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.PNCJunction.passage_group)
  return passage_group_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::PassageGroup >*
PNCJunction::mutable_passage_group() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.PNCJunction.passage_group)
  return &passage_group_;
}
inline const ::apollo::hdmap::PassageGroup& PNCJunction::passage_group(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.PNCJunction.passage_group)
  return passage_group_.Get(index);
}
inline ::apollo::hdmap::PassageGroup* PNCJunction::add_passage_group() {
  // @@protoc_insertion_point(field_add:apollo.hdmap.PNCJunction.passage_group)
  return passage_group_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::PassageGroup >&
PNCJunction::passage_group() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.PNCJunction.passage_group)
  return passage_group_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace hdmap
}  // namespace apollo

namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::apollo::hdmap::Passage_Type> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::hdmap::Passage_Type>() {
  return ::apollo::hdmap::Passage_Type_descriptor();
}

}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fpnc_5fjunction_2eproto
