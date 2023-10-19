// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/localization/proto/imu.proto

#ifndef PROTOBUF_INCLUDED_modules_2flocalization_2fproto_2fimu_2eproto
#define PROTOBUF_INCLUDED_modules_2flocalization_2fproto_2fimu_2eproto

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
#include "modules/common/proto/header.pb.h"
#include "modules/localization/proto/pose.pb.h"
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2flocalization_2fproto_2fimu_2eproto 

namespace protobuf_modules_2flocalization_2fproto_2fimu_2eproto {
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
}  // namespace protobuf_modules_2flocalization_2fproto_2fimu_2eproto
namespace apollo {
namespace localization {
class CorrectedImu;
class CorrectedImuDefaultTypeInternal;
extern CorrectedImuDefaultTypeInternal _CorrectedImu_default_instance_;
}  // namespace localization
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::localization::CorrectedImu* Arena::CreateMaybeMessage<::apollo::localization::CorrectedImu>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace localization {

// ===================================================================

class CorrectedImu : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.localization.CorrectedImu) */ {
 public:
  CorrectedImu();
  virtual ~CorrectedImu();

  CorrectedImu(const CorrectedImu& from);

  inline CorrectedImu& operator=(const CorrectedImu& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  CorrectedImu(CorrectedImu&& from) noexcept
    : CorrectedImu() {
    *this = ::std::move(from);
  }

  inline CorrectedImu& operator=(CorrectedImu&& from) noexcept {
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
  static const CorrectedImu& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const CorrectedImu* internal_default_instance() {
    return reinterpret_cast<const CorrectedImu*>(
               &_CorrectedImu_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(CorrectedImu* other);
  friend void swap(CorrectedImu& a, CorrectedImu& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline CorrectedImu* New() const final {
    return CreateMaybeMessage<CorrectedImu>(NULL);
  }

  CorrectedImu* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<CorrectedImu>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const CorrectedImu& from);
  void MergeFrom(const CorrectedImu& from);
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
  void InternalSwap(CorrectedImu* other);
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

  // optional .apollo.common.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  private:
  const ::apollo::common::Header& _internal_header() const;
  public:
  const ::apollo::common::Header& header() const;
  ::apollo::common::Header* release_header();
  ::apollo::common::Header* mutable_header();
  void set_allocated_header(::apollo::common::Header* header);

  // optional .apollo.localization.Pose imu = 3;
  bool has_imu() const;
  void clear_imu();
  static const int kImuFieldNumber = 3;
  private:
  const ::apollo::localization::Pose& _internal_imu() const;
  public:
  const ::apollo::localization::Pose& imu() const;
  ::apollo::localization::Pose* release_imu();
  ::apollo::localization::Pose* mutable_imu();
  void set_allocated_imu(::apollo::localization::Pose* imu);

  // @@protoc_insertion_point(class_scope:apollo.localization.CorrectedImu)
 private:
  void set_has_header();
  void clear_has_header();
  void set_has_imu();
  void clear_has_imu();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::apollo::common::Header* header_;
  ::apollo::localization::Pose* imu_;
  friend struct ::protobuf_modules_2flocalization_2fproto_2fimu_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// CorrectedImu

// optional .apollo.common.Header header = 1;
inline bool CorrectedImu::has_header() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void CorrectedImu::set_has_header() {
  _has_bits_[0] |= 0x00000001u;
}
inline void CorrectedImu::clear_has_header() {
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::apollo::common::Header& CorrectedImu::_internal_header() const {
  return *header_;
}
inline const ::apollo::common::Header& CorrectedImu::header() const {
  const ::apollo::common::Header* p = header_;
  // @@protoc_insertion_point(field_get:apollo.localization.CorrectedImu.header)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::common::Header*>(
      &::apollo::common::_Header_default_instance_);
}
inline ::apollo::common::Header* CorrectedImu::release_header() {
  // @@protoc_insertion_point(field_release:apollo.localization.CorrectedImu.header)
  clear_has_header();
  ::apollo::common::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::apollo::common::Header* CorrectedImu::mutable_header() {
  set_has_header();
  if (header_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::common::Header>(GetArenaNoVirtual());
    header_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.localization.CorrectedImu.header)
  return header_;
}
inline void CorrectedImu::set_allocated_header(::apollo::common::Header* header) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(header_);
  }
  if (header) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      header = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    set_has_header();
  } else {
    clear_has_header();
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:apollo.localization.CorrectedImu.header)
}

// optional .apollo.localization.Pose imu = 3;
inline bool CorrectedImu::has_imu() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void CorrectedImu::set_has_imu() {
  _has_bits_[0] |= 0x00000002u;
}
inline void CorrectedImu::clear_has_imu() {
  _has_bits_[0] &= ~0x00000002u;
}
inline const ::apollo::localization::Pose& CorrectedImu::_internal_imu() const {
  return *imu_;
}
inline const ::apollo::localization::Pose& CorrectedImu::imu() const {
  const ::apollo::localization::Pose* p = imu_;
  // @@protoc_insertion_point(field_get:apollo.localization.CorrectedImu.imu)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::localization::Pose*>(
      &::apollo::localization::_Pose_default_instance_);
}
inline ::apollo::localization::Pose* CorrectedImu::release_imu() {
  // @@protoc_insertion_point(field_release:apollo.localization.CorrectedImu.imu)
  clear_has_imu();
  ::apollo::localization::Pose* temp = imu_;
  imu_ = NULL;
  return temp;
}
inline ::apollo::localization::Pose* CorrectedImu::mutable_imu() {
  set_has_imu();
  if (imu_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::localization::Pose>(GetArenaNoVirtual());
    imu_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.localization.CorrectedImu.imu)
  return imu_;
}
inline void CorrectedImu::set_allocated_imu(::apollo::localization::Pose* imu) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(imu_);
  }
  if (imu) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      imu = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, imu, submessage_arena);
    }
    set_has_imu();
  } else {
    clear_has_imu();
  }
  imu_ = imu;
  // @@protoc_insertion_point(field_set_allocated:apollo.localization.CorrectedImu.imu)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace localization
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_modules_2flocalization_2fproto_2fimu_2eproto
