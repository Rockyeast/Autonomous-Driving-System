// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/proto/dst_existence_fusion_config.proto

#ifndef PROTOBUF_INCLUDED_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto
#define PROTOBUF_INCLUDED_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto

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
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto 

namespace protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto {
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
}  // namespace protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto
namespace apollo {
namespace perception {
class CameraValidDist;
class CameraValidDistDefaultTypeInternal;
extern CameraValidDistDefaultTypeInternal _CameraValidDist_default_instance_;
class DstExistenceFusionConfig;
class DstExistenceFusionConfigDefaultTypeInternal;
extern DstExistenceFusionConfigDefaultTypeInternal _DstExistenceFusionConfig_default_instance_;
}  // namespace perception
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::perception::CameraValidDist* Arena::CreateMaybeMessage<::apollo::perception::CameraValidDist>(Arena*);
template<> ::apollo::perception::DstExistenceFusionConfig* Arena::CreateMaybeMessage<::apollo::perception::DstExistenceFusionConfig>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace perception {

// ===================================================================

class CameraValidDist : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.perception.CameraValidDist) */ {
 public:
  CameraValidDist();
  virtual ~CameraValidDist();

  CameraValidDist(const CameraValidDist& from);

  inline CameraValidDist& operator=(const CameraValidDist& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  CameraValidDist(CameraValidDist&& from) noexcept
    : CameraValidDist() {
    *this = ::std::move(from);
  }

  inline CameraValidDist& operator=(CameraValidDist&& from) noexcept {
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
  static const CameraValidDist& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const CameraValidDist* internal_default_instance() {
    return reinterpret_cast<const CameraValidDist*>(
               &_CameraValidDist_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(CameraValidDist* other);
  friend void swap(CameraValidDist& a, CameraValidDist& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline CameraValidDist* New() const final {
    return CreateMaybeMessage<CameraValidDist>(NULL);
  }

  CameraValidDist* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<CameraValidDist>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const CameraValidDist& from);
  void MergeFrom(const CameraValidDist& from);
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
  void InternalSwap(CameraValidDist* other);
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

  // optional string camera_name = 1 [default = ""];
  bool has_camera_name() const;
  void clear_camera_name();
  static const int kCameraNameFieldNumber = 1;
  const ::std::string& camera_name() const;
  void set_camera_name(const ::std::string& value);
  #if LANG_CXX11
  void set_camera_name(::std::string&& value);
  #endif
  void set_camera_name(const char* value);
  void set_camera_name(const char* value, size_t size);
  ::std::string* mutable_camera_name();
  ::std::string* release_camera_name();
  void set_allocated_camera_name(::std::string* camera_name);

  // optional double valid_dist = 2 [default = 0];
  bool has_valid_dist() const;
  void clear_valid_dist();
  static const int kValidDistFieldNumber = 2;
  double valid_dist() const;
  void set_valid_dist(double value);

  // @@protoc_insertion_point(class_scope:apollo.perception.CameraValidDist)
 private:
  void set_has_camera_name();
  void clear_has_camera_name();
  void set_has_valid_dist();
  void clear_has_valid_dist();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr camera_name_;
  double valid_dist_;
  friend struct ::protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class DstExistenceFusionConfig : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.perception.DstExistenceFusionConfig) */ {
 public:
  DstExistenceFusionConfig();
  virtual ~DstExistenceFusionConfig();

  DstExistenceFusionConfig(const DstExistenceFusionConfig& from);

  inline DstExistenceFusionConfig& operator=(const DstExistenceFusionConfig& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  DstExistenceFusionConfig(DstExistenceFusionConfig&& from) noexcept
    : DstExistenceFusionConfig() {
    *this = ::std::move(from);
  }

  inline DstExistenceFusionConfig& operator=(DstExistenceFusionConfig&& from) noexcept {
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
  static const DstExistenceFusionConfig& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const DstExistenceFusionConfig* internal_default_instance() {
    return reinterpret_cast<const DstExistenceFusionConfig*>(
               &_DstExistenceFusionConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(DstExistenceFusionConfig* other);
  friend void swap(DstExistenceFusionConfig& a, DstExistenceFusionConfig& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline DstExistenceFusionConfig* New() const final {
    return CreateMaybeMessage<DstExistenceFusionConfig>(NULL);
  }

  DstExistenceFusionConfig* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<DstExistenceFusionConfig>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const DstExistenceFusionConfig& from);
  void MergeFrom(const DstExistenceFusionConfig& from);
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
  void InternalSwap(DstExistenceFusionConfig* other);
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

  // repeated .apollo.perception.CameraValidDist camera_valid_dist = 2;
  int camera_valid_dist_size() const;
  void clear_camera_valid_dist();
  static const int kCameraValidDistFieldNumber = 2;
  ::apollo::perception::CameraValidDist* mutable_camera_valid_dist(int index);
  ::google::protobuf::RepeatedPtrField< ::apollo::perception::CameraValidDist >*
      mutable_camera_valid_dist();
  const ::apollo::perception::CameraValidDist& camera_valid_dist(int index) const;
  ::apollo::perception::CameraValidDist* add_camera_valid_dist();
  const ::google::protobuf::RepeatedPtrField< ::apollo::perception::CameraValidDist >&
      camera_valid_dist() const;

  // optional double track_object_max_match_distance = 1 [default = 4];
  bool has_track_object_max_match_distance() const;
  void clear_track_object_max_match_distance();
  static const int kTrackObjectMaxMatchDistanceFieldNumber = 1;
  double track_object_max_match_distance() const;
  void set_track_object_max_match_distance(double value);

  // @@protoc_insertion_point(class_scope:apollo.perception.DstExistenceFusionConfig)
 private:
  void set_has_track_object_max_match_distance();
  void clear_has_track_object_max_match_distance();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::apollo::perception::CameraValidDist > camera_valid_dist_;
  double track_object_max_match_distance_;
  friend struct ::protobuf_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// CameraValidDist

// optional string camera_name = 1 [default = ""];
inline bool CameraValidDist::has_camera_name() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void CameraValidDist::set_has_camera_name() {
  _has_bits_[0] |= 0x00000001u;
}
inline void CameraValidDist::clear_has_camera_name() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void CameraValidDist::clear_camera_name() {
  camera_name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_camera_name();
}
inline const ::std::string& CameraValidDist::camera_name() const {
  // @@protoc_insertion_point(field_get:apollo.perception.CameraValidDist.camera_name)
  return camera_name_.GetNoArena();
}
inline void CameraValidDist::set_camera_name(const ::std::string& value) {
  set_has_camera_name();
  camera_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:apollo.perception.CameraValidDist.camera_name)
}
#if LANG_CXX11
inline void CameraValidDist::set_camera_name(::std::string&& value) {
  set_has_camera_name();
  camera_name_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.perception.CameraValidDist.camera_name)
}
#endif
inline void CameraValidDist::set_camera_name(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_camera_name();
  camera_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.perception.CameraValidDist.camera_name)
}
inline void CameraValidDist::set_camera_name(const char* value, size_t size) {
  set_has_camera_name();
  camera_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.perception.CameraValidDist.camera_name)
}
inline ::std::string* CameraValidDist::mutable_camera_name() {
  set_has_camera_name();
  // @@protoc_insertion_point(field_mutable:apollo.perception.CameraValidDist.camera_name)
  return camera_name_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* CameraValidDist::release_camera_name() {
  // @@protoc_insertion_point(field_release:apollo.perception.CameraValidDist.camera_name)
  if (!has_camera_name()) {
    return NULL;
  }
  clear_has_camera_name();
  return camera_name_.ReleaseNonDefaultNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void CameraValidDist::set_allocated_camera_name(::std::string* camera_name) {
  if (camera_name != NULL) {
    set_has_camera_name();
  } else {
    clear_has_camera_name();
  }
  camera_name_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), camera_name);
  // @@protoc_insertion_point(field_set_allocated:apollo.perception.CameraValidDist.camera_name)
}

// optional double valid_dist = 2 [default = 0];
inline bool CameraValidDist::has_valid_dist() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void CameraValidDist::set_has_valid_dist() {
  _has_bits_[0] |= 0x00000002u;
}
inline void CameraValidDist::clear_has_valid_dist() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void CameraValidDist::clear_valid_dist() {
  valid_dist_ = 0;
  clear_has_valid_dist();
}
inline double CameraValidDist::valid_dist() const {
  // @@protoc_insertion_point(field_get:apollo.perception.CameraValidDist.valid_dist)
  return valid_dist_;
}
inline void CameraValidDist::set_valid_dist(double value) {
  set_has_valid_dist();
  valid_dist_ = value;
  // @@protoc_insertion_point(field_set:apollo.perception.CameraValidDist.valid_dist)
}

// -------------------------------------------------------------------

// DstExistenceFusionConfig

// optional double track_object_max_match_distance = 1 [default = 4];
inline bool DstExistenceFusionConfig::has_track_object_max_match_distance() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void DstExistenceFusionConfig::set_has_track_object_max_match_distance() {
  _has_bits_[0] |= 0x00000001u;
}
inline void DstExistenceFusionConfig::clear_has_track_object_max_match_distance() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void DstExistenceFusionConfig::clear_track_object_max_match_distance() {
  track_object_max_match_distance_ = 4;
  clear_has_track_object_max_match_distance();
}
inline double DstExistenceFusionConfig::track_object_max_match_distance() const {
  // @@protoc_insertion_point(field_get:apollo.perception.DstExistenceFusionConfig.track_object_max_match_distance)
  return track_object_max_match_distance_;
}
inline void DstExistenceFusionConfig::set_track_object_max_match_distance(double value) {
  set_has_track_object_max_match_distance();
  track_object_max_match_distance_ = value;
  // @@protoc_insertion_point(field_set:apollo.perception.DstExistenceFusionConfig.track_object_max_match_distance)
}

// repeated .apollo.perception.CameraValidDist camera_valid_dist = 2;
inline int DstExistenceFusionConfig::camera_valid_dist_size() const {
  return camera_valid_dist_.size();
}
inline void DstExistenceFusionConfig::clear_camera_valid_dist() {
  camera_valid_dist_.Clear();
}
inline ::apollo::perception::CameraValidDist* DstExistenceFusionConfig::mutable_camera_valid_dist(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.perception.DstExistenceFusionConfig.camera_valid_dist)
  return camera_valid_dist_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::perception::CameraValidDist >*
DstExistenceFusionConfig::mutable_camera_valid_dist() {
  // @@protoc_insertion_point(field_mutable_list:apollo.perception.DstExistenceFusionConfig.camera_valid_dist)
  return &camera_valid_dist_;
}
inline const ::apollo::perception::CameraValidDist& DstExistenceFusionConfig::camera_valid_dist(int index) const {
  // @@protoc_insertion_point(field_get:apollo.perception.DstExistenceFusionConfig.camera_valid_dist)
  return camera_valid_dist_.Get(index);
}
inline ::apollo::perception::CameraValidDist* DstExistenceFusionConfig::add_camera_valid_dist() {
  // @@protoc_insertion_point(field_add:apollo.perception.DstExistenceFusionConfig.camera_valid_dist)
  return camera_valid_dist_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::perception::CameraValidDist >&
DstExistenceFusionConfig::camera_valid_dist() const {
  // @@protoc_insertion_point(field_list:apollo.perception.DstExistenceFusionConfig.camera_valid_dist)
  return camera_valid_dist_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace perception
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_modules_2fperception_2fproto_2fdst_5fexistence_5ffusion_5fconfig_2eproto
