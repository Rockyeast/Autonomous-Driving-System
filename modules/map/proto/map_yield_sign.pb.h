// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/map/proto/map_yield_sign.proto

#ifndef PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fyield_5fsign_2eproto
#define PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fyield_5fsign_2eproto

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
#include "modules/map/proto/map_geometry.pb.h"
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fmap_2fproto_2fmap_5fyield_5fsign_2eproto 

namespace protobuf_modules_2fmap_2fproto_2fmap_5fyield_5fsign_2eproto {
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
}  // namespace protobuf_modules_2fmap_2fproto_2fmap_5fyield_5fsign_2eproto
namespace apollo {
namespace hdmap {
class YieldSign;
class YieldSignDefaultTypeInternal;
extern YieldSignDefaultTypeInternal _YieldSign_default_instance_;
}  // namespace hdmap
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::hdmap::YieldSign* Arena::CreateMaybeMessage<::apollo::hdmap::YieldSign>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace hdmap {

// ===================================================================

class YieldSign : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.hdmap.YieldSign) */ {
 public:
  YieldSign();
  virtual ~YieldSign();

  YieldSign(const YieldSign& from);

  inline YieldSign& operator=(const YieldSign& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  YieldSign(YieldSign&& from) noexcept
    : YieldSign() {
    *this = ::std::move(from);
  }

  inline YieldSign& operator=(YieldSign&& from) noexcept {
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
  static const YieldSign& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const YieldSign* internal_default_instance() {
    return reinterpret_cast<const YieldSign*>(
               &_YieldSign_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(YieldSign* other);
  friend void swap(YieldSign& a, YieldSign& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline YieldSign* New() const final {
    return CreateMaybeMessage<YieldSign>(NULL);
  }

  YieldSign* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<YieldSign>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const YieldSign& from);
  void MergeFrom(const YieldSign& from);
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
  void InternalSwap(YieldSign* other);
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

  // repeated .apollo.hdmap.Curve stop_line = 2;
  int stop_line_size() const;
  void clear_stop_line();
  static const int kStopLineFieldNumber = 2;
  ::apollo::hdmap::Curve* mutable_stop_line(int index);
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Curve >*
      mutable_stop_line();
  const ::apollo::hdmap::Curve& stop_line(int index) const;
  ::apollo::hdmap::Curve* add_stop_line();
  const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Curve >&
      stop_line() const;

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

  // @@protoc_insertion_point(class_scope:apollo.hdmap.YieldSign)
 private:
  void set_has_id();
  void clear_has_id();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Curve > stop_line_;
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id > overlap_id_;
  ::apollo::hdmap::Id* id_;
  friend struct ::protobuf_modules_2fmap_2fproto_2fmap_5fyield_5fsign_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// YieldSign

// optional .apollo.hdmap.Id id = 1;
inline bool YieldSign::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void YieldSign::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void YieldSign::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::apollo::hdmap::Id& YieldSign::_internal_id() const {
  return *id_;
}
inline const ::apollo::hdmap::Id& YieldSign::id() const {
  const ::apollo::hdmap::Id* p = id_;
  // @@protoc_insertion_point(field_get:apollo.hdmap.YieldSign.id)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::hdmap::Id*>(
      &::apollo::hdmap::_Id_default_instance_);
}
inline ::apollo::hdmap::Id* YieldSign::release_id() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.YieldSign.id)
  clear_has_id();
  ::apollo::hdmap::Id* temp = id_;
  id_ = NULL;
  return temp;
}
inline ::apollo::hdmap::Id* YieldSign::mutable_id() {
  set_has_id();
  if (id_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::hdmap::Id>(GetArenaNoVirtual());
    id_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.YieldSign.id)
  return id_;
}
inline void YieldSign::set_allocated_id(::apollo::hdmap::Id* id) {
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
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.YieldSign.id)
}

// repeated .apollo.hdmap.Curve stop_line = 2;
inline int YieldSign::stop_line_size() const {
  return stop_line_.size();
}
inline ::apollo::hdmap::Curve* YieldSign::mutable_stop_line(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.YieldSign.stop_line)
  return stop_line_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Curve >*
YieldSign::mutable_stop_line() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.YieldSign.stop_line)
  return &stop_line_;
}
inline const ::apollo::hdmap::Curve& YieldSign::stop_line(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.YieldSign.stop_line)
  return stop_line_.Get(index);
}
inline ::apollo::hdmap::Curve* YieldSign::add_stop_line() {
  // @@protoc_insertion_point(field_add:apollo.hdmap.YieldSign.stop_line)
  return stop_line_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Curve >&
YieldSign::stop_line() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.YieldSign.stop_line)
  return stop_line_;
}

// repeated .apollo.hdmap.Id overlap_id = 3;
inline int YieldSign::overlap_id_size() const {
  return overlap_id_.size();
}
inline ::apollo::hdmap::Id* YieldSign::mutable_overlap_id(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.YieldSign.overlap_id)
  return overlap_id_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >*
YieldSign::mutable_overlap_id() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.YieldSign.overlap_id)
  return &overlap_id_;
}
inline const ::apollo::hdmap::Id& YieldSign::overlap_id(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.YieldSign.overlap_id)
  return overlap_id_.Get(index);
}
inline ::apollo::hdmap::Id* YieldSign::add_overlap_id() {
  // @@protoc_insertion_point(field_add:apollo.hdmap.YieldSign.overlap_id)
  return overlap_id_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >&
YieldSign::overlap_id() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.YieldSign.overlap_id)
  return overlap_id_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace hdmap
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fyield_5fsign_2eproto