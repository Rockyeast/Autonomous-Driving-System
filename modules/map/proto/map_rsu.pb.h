// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/map/proto/map_rsu.proto

#ifndef PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5frsu_2eproto
#define PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5frsu_2eproto

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
#include "modules/map/proto/map_id.pb.h"
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fmap_2fproto_2fmap_5frsu_2eproto 

namespace protobuf_modules_2fmap_2fproto_2fmap_5frsu_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[1];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_modules_2fmap_2fproto_2fmap_5frsu_2eproto
namespace apollo {
namespace hdmap {
class RSU;
class RSUDefaultTypeInternal;
extern RSUDefaultTypeInternal _RSU_default_instance_;
}  // namespace hdmap
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::hdmap::RSU* Arena::CreateMaybeMessage<::apollo::hdmap::RSU>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace hdmap {

// ===================================================================

class RSU : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.hdmap.RSU) */ {
 public:
  RSU();
  virtual ~RSU();

  RSU(const RSU& from);

  inline RSU& operator=(const RSU& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  RSU(RSU&& from) noexcept
    : RSU() {
    *this = ::std::move(from);
  }

  inline RSU& operator=(RSU&& from) noexcept {
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
  static const RSU& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const RSU* internal_default_instance() {
    return reinterpret_cast<const RSU*>(
               &_RSU_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(RSU* other);
  friend void swap(RSU& a, RSU& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline RSU* New() const final {
    return CreateMaybeMessage<RSU>(NULL);
  }

  RSU* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<RSU>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const RSU& from);
  void MergeFrom(const RSU& from);
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
  void InternalSwap(RSU* other);
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

  // optional .apollo.hdmap.Id junction_id = 2;
  bool has_junction_id() const;
  void clear_junction_id();
  static const int kJunctionIdFieldNumber = 2;
  private:
  const ::apollo::hdmap::Id& _internal_junction_id() const;
  public:
  const ::apollo::hdmap::Id& junction_id() const;
  ::apollo::hdmap::Id* release_junction_id();
  ::apollo::hdmap::Id* mutable_junction_id();
  void set_allocated_junction_id(::apollo::hdmap::Id* junction_id);

  // @@protoc_insertion_point(class_scope:apollo.hdmap.RSU)
 private:
  void set_has_id();
  void clear_has_id();
  void set_has_junction_id();
  void clear_has_junction_id();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id > overlap_id_;
  ::apollo::hdmap::Id* id_;
  ::apollo::hdmap::Id* junction_id_;
  friend struct ::protobuf_modules_2fmap_2fproto_2fmap_5frsu_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// RSU

// optional .apollo.hdmap.Id id = 1;
inline bool RSU::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void RSU::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void RSU::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::apollo::hdmap::Id& RSU::_internal_id() const {
  return *id_;
}
inline const ::apollo::hdmap::Id& RSU::id() const {
  const ::apollo::hdmap::Id* p = id_;
  // @@protoc_insertion_point(field_get:apollo.hdmap.RSU.id)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::hdmap::Id*>(
      &::apollo::hdmap::_Id_default_instance_);
}
inline ::apollo::hdmap::Id* RSU::release_id() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.RSU.id)
  clear_has_id();
  ::apollo::hdmap::Id* temp = id_;
  id_ = NULL;
  return temp;
}
inline ::apollo::hdmap::Id* RSU::mutable_id() {
  set_has_id();
  if (id_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::hdmap::Id>(GetArenaNoVirtual());
    id_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.RSU.id)
  return id_;
}
inline void RSU::set_allocated_id(::apollo::hdmap::Id* id) {
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
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.RSU.id)
}

// optional .apollo.hdmap.Id junction_id = 2;
inline bool RSU::has_junction_id() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void RSU::set_has_junction_id() {
  _has_bits_[0] |= 0x00000002u;
}
inline void RSU::clear_has_junction_id() {
  _has_bits_[0] &= ~0x00000002u;
}
inline const ::apollo::hdmap::Id& RSU::_internal_junction_id() const {
  return *junction_id_;
}
inline const ::apollo::hdmap::Id& RSU::junction_id() const {
  const ::apollo::hdmap::Id* p = junction_id_;
  // @@protoc_insertion_point(field_get:apollo.hdmap.RSU.junction_id)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::hdmap::Id*>(
      &::apollo::hdmap::_Id_default_instance_);
}
inline ::apollo::hdmap::Id* RSU::release_junction_id() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.RSU.junction_id)
  clear_has_junction_id();
  ::apollo::hdmap::Id* temp = junction_id_;
  junction_id_ = NULL;
  return temp;
}
inline ::apollo::hdmap::Id* RSU::mutable_junction_id() {
  set_has_junction_id();
  if (junction_id_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::hdmap::Id>(GetArenaNoVirtual());
    junction_id_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.RSU.junction_id)
  return junction_id_;
}
inline void RSU::set_allocated_junction_id(::apollo::hdmap::Id* junction_id) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(junction_id_);
  }
  if (junction_id) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      junction_id = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, junction_id, submessage_arena);
    }
    set_has_junction_id();
  } else {
    clear_has_junction_id();
  }
  junction_id_ = junction_id;
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.RSU.junction_id)
}

// repeated .apollo.hdmap.Id overlap_id = 3;
inline int RSU::overlap_id_size() const {
  return overlap_id_.size();
}
inline ::apollo::hdmap::Id* RSU::mutable_overlap_id(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.RSU.overlap_id)
  return overlap_id_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >*
RSU::mutable_overlap_id() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.RSU.overlap_id)
  return &overlap_id_;
}
inline const ::apollo::hdmap::Id& RSU::overlap_id(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.RSU.overlap_id)
  return overlap_id_.Get(index);
}
inline ::apollo::hdmap::Id* RSU::add_overlap_id() {
  // @@protoc_insertion_point(field_add:apollo.hdmap.RSU.overlap_id)
  return overlap_id_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >&
RSU::overlap_id() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.RSU.overlap_id)
  return overlap_id_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace hdmap
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5frsu_2eproto