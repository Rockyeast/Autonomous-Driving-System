// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/proto/pbf_tracker_config.proto

#ifndef PROTOBUF_INCLUDED_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto
#define PROTOBUF_INCLUDED_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto 

namespace protobuf_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto {
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
}  // namespace protobuf_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto
namespace apollo {
namespace perception {
namespace fusion {
class PbfTrackerConfig;
class PbfTrackerConfigDefaultTypeInternal;
extern PbfTrackerConfigDefaultTypeInternal _PbfTrackerConfig_default_instance_;
}  // namespace fusion
}  // namespace perception
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::perception::fusion::PbfTrackerConfig* Arena::CreateMaybeMessage<::apollo::perception::fusion::PbfTrackerConfig>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace perception {
namespace fusion {

// ===================================================================

class PbfTrackerConfig : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.perception.fusion.PbfTrackerConfig) */ {
 public:
  PbfTrackerConfig();
  virtual ~PbfTrackerConfig();

  PbfTrackerConfig(const PbfTrackerConfig& from);

  inline PbfTrackerConfig& operator=(const PbfTrackerConfig& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  PbfTrackerConfig(PbfTrackerConfig&& from) noexcept
    : PbfTrackerConfig() {
    *this = ::std::move(from);
  }

  inline PbfTrackerConfig& operator=(PbfTrackerConfig&& from) noexcept {
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
  static const PbfTrackerConfig& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PbfTrackerConfig* internal_default_instance() {
    return reinterpret_cast<const PbfTrackerConfig*>(
               &_PbfTrackerConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(PbfTrackerConfig* other);
  friend void swap(PbfTrackerConfig& a, PbfTrackerConfig& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline PbfTrackerConfig* New() const final {
    return CreateMaybeMessage<PbfTrackerConfig>(NULL);
  }

  PbfTrackerConfig* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<PbfTrackerConfig>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const PbfTrackerConfig& from);
  void MergeFrom(const PbfTrackerConfig& from);
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
  void InternalSwap(PbfTrackerConfig* other);
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

  // optional string type_fusion_method = 1 [default = "DstTypeFusion"];
  bool has_type_fusion_method() const;
  void clear_type_fusion_method();
  static const int kTypeFusionMethodFieldNumber = 1;
  const ::std::string& type_fusion_method() const;
  void set_type_fusion_method(const ::std::string& value);
  #if LANG_CXX11
  void set_type_fusion_method(::std::string&& value);
  #endif
  void set_type_fusion_method(const char* value);
  void set_type_fusion_method(const char* value, size_t size);
  ::std::string* mutable_type_fusion_method();
  ::std::string* release_type_fusion_method();
  void set_allocated_type_fusion_method(::std::string* type_fusion_method);

  // optional string motion_fusion_method = 2 [default = "KalmanMotionFusion"];
  bool has_motion_fusion_method() const;
  void clear_motion_fusion_method();
  static const int kMotionFusionMethodFieldNumber = 2;
  const ::std::string& motion_fusion_method() const;
  void set_motion_fusion_method(const ::std::string& value);
  #if LANG_CXX11
  void set_motion_fusion_method(::std::string&& value);
  #endif
  void set_motion_fusion_method(const char* value);
  void set_motion_fusion_method(const char* value, size_t size);
  ::std::string* mutable_motion_fusion_method();
  ::std::string* release_motion_fusion_method();
  void set_allocated_motion_fusion_method(::std::string* motion_fusion_method);

  // optional string shape_fusion_method = 3 [default = "PbfShapeFusion"];
  bool has_shape_fusion_method() const;
  void clear_shape_fusion_method();
  static const int kShapeFusionMethodFieldNumber = 3;
  const ::std::string& shape_fusion_method() const;
  void set_shape_fusion_method(const ::std::string& value);
  #if LANG_CXX11
  void set_shape_fusion_method(::std::string&& value);
  #endif
  void set_shape_fusion_method(const char* value);
  void set_shape_fusion_method(const char* value, size_t size);
  ::std::string* mutable_shape_fusion_method();
  ::std::string* release_shape_fusion_method();
  void set_allocated_shape_fusion_method(::std::string* shape_fusion_method);

  // optional string existence_fusion_method = 4 [default = "DstExistenceFusion"];
  bool has_existence_fusion_method() const;
  void clear_existence_fusion_method();
  static const int kExistenceFusionMethodFieldNumber = 4;
  const ::std::string& existence_fusion_method() const;
  void set_existence_fusion_method(const ::std::string& value);
  #if LANG_CXX11
  void set_existence_fusion_method(::std::string&& value);
  #endif
  void set_existence_fusion_method(const char* value);
  void set_existence_fusion_method(const char* value, size_t size);
  ::std::string* mutable_existence_fusion_method();
  ::std::string* release_existence_fusion_method();
  void set_allocated_existence_fusion_method(::std::string* existence_fusion_method);

  // @@protoc_insertion_point(class_scope:apollo.perception.fusion.PbfTrackerConfig)
 private:
  void set_has_type_fusion_method();
  void clear_has_type_fusion_method();
  void set_has_motion_fusion_method();
  void clear_has_motion_fusion_method();
  void set_has_shape_fusion_method();
  void clear_has_shape_fusion_method();
  void set_has_existence_fusion_method();
  void clear_has_existence_fusion_method();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  public:
  static ::google::protobuf::internal::ExplicitlyConstructed< ::std::string> _i_give_permission_to_break_this_code_default_type_fusion_method_;
  private:
  ::google::protobuf::internal::ArenaStringPtr type_fusion_method_;
  public:
  static ::google::protobuf::internal::ExplicitlyConstructed< ::std::string> _i_give_permission_to_break_this_code_default_motion_fusion_method_;
  private:
  ::google::protobuf::internal::ArenaStringPtr motion_fusion_method_;
  public:
  static ::google::protobuf::internal::ExplicitlyConstructed< ::std::string> _i_give_permission_to_break_this_code_default_shape_fusion_method_;
  private:
  ::google::protobuf::internal::ArenaStringPtr shape_fusion_method_;
  public:
  static ::google::protobuf::internal::ExplicitlyConstructed< ::std::string> _i_give_permission_to_break_this_code_default_existence_fusion_method_;
  private:
  ::google::protobuf::internal::ArenaStringPtr existence_fusion_method_;
  friend struct ::protobuf_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PbfTrackerConfig

// optional string type_fusion_method = 1 [default = "DstTypeFusion"];
inline bool PbfTrackerConfig::has_type_fusion_method() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void PbfTrackerConfig::set_has_type_fusion_method() {
  _has_bits_[0] |= 0x00000001u;
}
inline void PbfTrackerConfig::clear_has_type_fusion_method() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void PbfTrackerConfig::clear_type_fusion_method() {
  type_fusion_method_.ClearToDefaultNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get());
  clear_has_type_fusion_method();
}
inline const ::std::string& PbfTrackerConfig::type_fusion_method() const {
  // @@protoc_insertion_point(field_get:apollo.perception.fusion.PbfTrackerConfig.type_fusion_method)
  return type_fusion_method_.GetNoArena();
}
inline void PbfTrackerConfig::set_type_fusion_method(const ::std::string& value) {
  set_has_type_fusion_method();
  type_fusion_method_.SetNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get(), value);
  // @@protoc_insertion_point(field_set:apollo.perception.fusion.PbfTrackerConfig.type_fusion_method)
}
#if LANG_CXX11
inline void PbfTrackerConfig::set_type_fusion_method(::std::string&& value) {
  set_has_type_fusion_method();
  type_fusion_method_.SetNoArena(
    &::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.perception.fusion.PbfTrackerConfig.type_fusion_method)
}
#endif
inline void PbfTrackerConfig::set_type_fusion_method(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_type_fusion_method();
  type_fusion_method_.SetNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.perception.fusion.PbfTrackerConfig.type_fusion_method)
}
inline void PbfTrackerConfig::set_type_fusion_method(const char* value, size_t size) {
  set_has_type_fusion_method();
  type_fusion_method_.SetNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.perception.fusion.PbfTrackerConfig.type_fusion_method)
}
inline ::std::string* PbfTrackerConfig::mutable_type_fusion_method() {
  set_has_type_fusion_method();
  // @@protoc_insertion_point(field_mutable:apollo.perception.fusion.PbfTrackerConfig.type_fusion_method)
  return type_fusion_method_.MutableNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get());
}
inline ::std::string* PbfTrackerConfig::release_type_fusion_method() {
  // @@protoc_insertion_point(field_release:apollo.perception.fusion.PbfTrackerConfig.type_fusion_method)
  if (!has_type_fusion_method()) {
    return NULL;
  }
  clear_has_type_fusion_method();
  return type_fusion_method_.ReleaseNonDefaultNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get());
}
inline void PbfTrackerConfig::set_allocated_type_fusion_method(::std::string* type_fusion_method) {
  if (type_fusion_method != NULL) {
    set_has_type_fusion_method();
  } else {
    clear_has_type_fusion_method();
  }
  type_fusion_method_.SetAllocatedNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_type_fusion_method_.get(), type_fusion_method);
  // @@protoc_insertion_point(field_set_allocated:apollo.perception.fusion.PbfTrackerConfig.type_fusion_method)
}

// optional string motion_fusion_method = 2 [default = "KalmanMotionFusion"];
inline bool PbfTrackerConfig::has_motion_fusion_method() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void PbfTrackerConfig::set_has_motion_fusion_method() {
  _has_bits_[0] |= 0x00000002u;
}
inline void PbfTrackerConfig::clear_has_motion_fusion_method() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void PbfTrackerConfig::clear_motion_fusion_method() {
  motion_fusion_method_.ClearToDefaultNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get());
  clear_has_motion_fusion_method();
}
inline const ::std::string& PbfTrackerConfig::motion_fusion_method() const {
  // @@protoc_insertion_point(field_get:apollo.perception.fusion.PbfTrackerConfig.motion_fusion_method)
  return motion_fusion_method_.GetNoArena();
}
inline void PbfTrackerConfig::set_motion_fusion_method(const ::std::string& value) {
  set_has_motion_fusion_method();
  motion_fusion_method_.SetNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get(), value);
  // @@protoc_insertion_point(field_set:apollo.perception.fusion.PbfTrackerConfig.motion_fusion_method)
}
#if LANG_CXX11
inline void PbfTrackerConfig::set_motion_fusion_method(::std::string&& value) {
  set_has_motion_fusion_method();
  motion_fusion_method_.SetNoArena(
    &::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.perception.fusion.PbfTrackerConfig.motion_fusion_method)
}
#endif
inline void PbfTrackerConfig::set_motion_fusion_method(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_motion_fusion_method();
  motion_fusion_method_.SetNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.perception.fusion.PbfTrackerConfig.motion_fusion_method)
}
inline void PbfTrackerConfig::set_motion_fusion_method(const char* value, size_t size) {
  set_has_motion_fusion_method();
  motion_fusion_method_.SetNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.perception.fusion.PbfTrackerConfig.motion_fusion_method)
}
inline ::std::string* PbfTrackerConfig::mutable_motion_fusion_method() {
  set_has_motion_fusion_method();
  // @@protoc_insertion_point(field_mutable:apollo.perception.fusion.PbfTrackerConfig.motion_fusion_method)
  return motion_fusion_method_.MutableNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get());
}
inline ::std::string* PbfTrackerConfig::release_motion_fusion_method() {
  // @@protoc_insertion_point(field_release:apollo.perception.fusion.PbfTrackerConfig.motion_fusion_method)
  if (!has_motion_fusion_method()) {
    return NULL;
  }
  clear_has_motion_fusion_method();
  return motion_fusion_method_.ReleaseNonDefaultNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get());
}
inline void PbfTrackerConfig::set_allocated_motion_fusion_method(::std::string* motion_fusion_method) {
  if (motion_fusion_method != NULL) {
    set_has_motion_fusion_method();
  } else {
    clear_has_motion_fusion_method();
  }
  motion_fusion_method_.SetAllocatedNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_motion_fusion_method_.get(), motion_fusion_method);
  // @@protoc_insertion_point(field_set_allocated:apollo.perception.fusion.PbfTrackerConfig.motion_fusion_method)
}

// optional string shape_fusion_method = 3 [default = "PbfShapeFusion"];
inline bool PbfTrackerConfig::has_shape_fusion_method() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void PbfTrackerConfig::set_has_shape_fusion_method() {
  _has_bits_[0] |= 0x00000004u;
}
inline void PbfTrackerConfig::clear_has_shape_fusion_method() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void PbfTrackerConfig::clear_shape_fusion_method() {
  shape_fusion_method_.ClearToDefaultNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get());
  clear_has_shape_fusion_method();
}
inline const ::std::string& PbfTrackerConfig::shape_fusion_method() const {
  // @@protoc_insertion_point(field_get:apollo.perception.fusion.PbfTrackerConfig.shape_fusion_method)
  return shape_fusion_method_.GetNoArena();
}
inline void PbfTrackerConfig::set_shape_fusion_method(const ::std::string& value) {
  set_has_shape_fusion_method();
  shape_fusion_method_.SetNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get(), value);
  // @@protoc_insertion_point(field_set:apollo.perception.fusion.PbfTrackerConfig.shape_fusion_method)
}
#if LANG_CXX11
inline void PbfTrackerConfig::set_shape_fusion_method(::std::string&& value) {
  set_has_shape_fusion_method();
  shape_fusion_method_.SetNoArena(
    &::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.perception.fusion.PbfTrackerConfig.shape_fusion_method)
}
#endif
inline void PbfTrackerConfig::set_shape_fusion_method(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_shape_fusion_method();
  shape_fusion_method_.SetNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.perception.fusion.PbfTrackerConfig.shape_fusion_method)
}
inline void PbfTrackerConfig::set_shape_fusion_method(const char* value, size_t size) {
  set_has_shape_fusion_method();
  shape_fusion_method_.SetNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.perception.fusion.PbfTrackerConfig.shape_fusion_method)
}
inline ::std::string* PbfTrackerConfig::mutable_shape_fusion_method() {
  set_has_shape_fusion_method();
  // @@protoc_insertion_point(field_mutable:apollo.perception.fusion.PbfTrackerConfig.shape_fusion_method)
  return shape_fusion_method_.MutableNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get());
}
inline ::std::string* PbfTrackerConfig::release_shape_fusion_method() {
  // @@protoc_insertion_point(field_release:apollo.perception.fusion.PbfTrackerConfig.shape_fusion_method)
  if (!has_shape_fusion_method()) {
    return NULL;
  }
  clear_has_shape_fusion_method();
  return shape_fusion_method_.ReleaseNonDefaultNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get());
}
inline void PbfTrackerConfig::set_allocated_shape_fusion_method(::std::string* shape_fusion_method) {
  if (shape_fusion_method != NULL) {
    set_has_shape_fusion_method();
  } else {
    clear_has_shape_fusion_method();
  }
  shape_fusion_method_.SetAllocatedNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_shape_fusion_method_.get(), shape_fusion_method);
  // @@protoc_insertion_point(field_set_allocated:apollo.perception.fusion.PbfTrackerConfig.shape_fusion_method)
}

// optional string existence_fusion_method = 4 [default = "DstExistenceFusion"];
inline bool PbfTrackerConfig::has_existence_fusion_method() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void PbfTrackerConfig::set_has_existence_fusion_method() {
  _has_bits_[0] |= 0x00000008u;
}
inline void PbfTrackerConfig::clear_has_existence_fusion_method() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void PbfTrackerConfig::clear_existence_fusion_method() {
  existence_fusion_method_.ClearToDefaultNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existence_fusion_method_.get());
  clear_has_existence_fusion_method();
}
inline const ::std::string& PbfTrackerConfig::existence_fusion_method() const {
  // @@protoc_insertion_point(field_get:apollo.perception.fusion.PbfTrackerConfig.existence_fusion_method)
  return existence_fusion_method_.GetNoArena();
}
inline void PbfTrackerConfig::set_existence_fusion_method(const ::std::string& value) {
  set_has_existence_fusion_method();
  existence_fusion_method_.SetNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existence_fusion_method_.get(), value);
  // @@protoc_insertion_point(field_set:apollo.perception.fusion.PbfTrackerConfig.existence_fusion_method)
}
#if LANG_CXX11
inline void PbfTrackerConfig::set_existence_fusion_method(::std::string&& value) {
  set_has_existence_fusion_method();
  existence_fusion_method_.SetNoArena(
    &::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existence_fusion_method_.get(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.perception.fusion.PbfTrackerConfig.existence_fusion_method)
}
#endif
inline void PbfTrackerConfig::set_existence_fusion_method(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_existence_fusion_method();
  existence_fusion_method_.SetNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existence_fusion_method_.get(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.perception.fusion.PbfTrackerConfig.existence_fusion_method)
}
inline void PbfTrackerConfig::set_existence_fusion_method(const char* value, size_t size) {
  set_has_existence_fusion_method();
  existence_fusion_method_.SetNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existence_fusion_method_.get(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.perception.fusion.PbfTrackerConfig.existence_fusion_method)
}
inline ::std::string* PbfTrackerConfig::mutable_existence_fusion_method() {
  set_has_existence_fusion_method();
  // @@protoc_insertion_point(field_mutable:apollo.perception.fusion.PbfTrackerConfig.existence_fusion_method)
  return existence_fusion_method_.MutableNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existence_fusion_method_.get());
}
inline ::std::string* PbfTrackerConfig::release_existence_fusion_method() {
  // @@protoc_insertion_point(field_release:apollo.perception.fusion.PbfTrackerConfig.existence_fusion_method)
  if (!has_existence_fusion_method()) {
    return NULL;
  }
  clear_has_existence_fusion_method();
  return existence_fusion_method_.ReleaseNonDefaultNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existence_fusion_method_.get());
}
inline void PbfTrackerConfig::set_allocated_existence_fusion_method(::std::string* existence_fusion_method) {
  if (existence_fusion_method != NULL) {
    set_has_existence_fusion_method();
  } else {
    clear_has_existence_fusion_method();
  }
  existence_fusion_method_.SetAllocatedNoArena(&::apollo::perception::fusion::PbfTrackerConfig::_i_give_permission_to_break_this_code_default_existence_fusion_method_.get(), existence_fusion_method);
  // @@protoc_insertion_point(field_set_allocated:apollo.perception.fusion.PbfTrackerConfig.existence_fusion_method)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace fusion
}  // namespace perception
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_modules_2fperception_2fproto_2fpbf_5ftracker_5fconfig_2eproto
