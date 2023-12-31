// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/proto/ccrf_type_fusion_config.proto

#ifndef PROTOBUF_INCLUDED_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto
#define PROTOBUF_INCLUDED_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto 

namespace protobuf_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto {
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
}  // namespace protobuf_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto
namespace apollo {
namespace perception {
namespace lidar {
class CcrfTypeFusionConfig;
class CcrfTypeFusionConfigDefaultTypeInternal;
extern CcrfTypeFusionConfigDefaultTypeInternal _CcrfTypeFusionConfig_default_instance_;
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::perception::lidar::CcrfTypeFusionConfig* Arena::CreateMaybeMessage<::apollo::perception::lidar::CcrfTypeFusionConfig>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace perception {
namespace lidar {

// ===================================================================

class CcrfTypeFusionConfig : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.perception.lidar.CcrfTypeFusionConfig) */ {
 public:
  CcrfTypeFusionConfig();
  virtual ~CcrfTypeFusionConfig();

  CcrfTypeFusionConfig(const CcrfTypeFusionConfig& from);

  inline CcrfTypeFusionConfig& operator=(const CcrfTypeFusionConfig& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  CcrfTypeFusionConfig(CcrfTypeFusionConfig&& from) noexcept
    : CcrfTypeFusionConfig() {
    *this = ::std::move(from);
  }

  inline CcrfTypeFusionConfig& operator=(CcrfTypeFusionConfig&& from) noexcept {
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
  static const CcrfTypeFusionConfig& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const CcrfTypeFusionConfig* internal_default_instance() {
    return reinterpret_cast<const CcrfTypeFusionConfig*>(
               &_CcrfTypeFusionConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(CcrfTypeFusionConfig* other);
  friend void swap(CcrfTypeFusionConfig& a, CcrfTypeFusionConfig& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline CcrfTypeFusionConfig* New() const final {
    return CreateMaybeMessage<CcrfTypeFusionConfig>(NULL);
  }

  CcrfTypeFusionConfig* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<CcrfTypeFusionConfig>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const CcrfTypeFusionConfig& from);
  void MergeFrom(const CcrfTypeFusionConfig& from);
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
  void InternalSwap(CcrfTypeFusionConfig* other);
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

  // optional string classifiers_property_file_path = 1 [default = ""];
  bool has_classifiers_property_file_path() const;
  void clear_classifiers_property_file_path();
  static const int kClassifiersPropertyFilePathFieldNumber = 1;
  const ::std::string& classifiers_property_file_path() const;
  void set_classifiers_property_file_path(const ::std::string& value);
  #if LANG_CXX11
  void set_classifiers_property_file_path(::std::string&& value);
  #endif
  void set_classifiers_property_file_path(const char* value);
  void set_classifiers_property_file_path(const char* value, size_t size);
  ::std::string* mutable_classifiers_property_file_path();
  ::std::string* release_classifiers_property_file_path();
  void set_allocated_classifiers_property_file_path(::std::string* classifiers_property_file_path);

  // optional string transition_property_file_path = 2 [default = ""];
  bool has_transition_property_file_path() const;
  void clear_transition_property_file_path();
  static const int kTransitionPropertyFilePathFieldNumber = 2;
  const ::std::string& transition_property_file_path() const;
  void set_transition_property_file_path(const ::std::string& value);
  #if LANG_CXX11
  void set_transition_property_file_path(::std::string&& value);
  #endif
  void set_transition_property_file_path(const char* value);
  void set_transition_property_file_path(const char* value, size_t size);
  ::std::string* mutable_transition_property_file_path();
  ::std::string* release_transition_property_file_path();
  void set_allocated_transition_property_file_path(::std::string* transition_property_file_path);

  // optional float transition_matrix_alpha = 3 [default = 1.8];
  bool has_transition_matrix_alpha() const;
  void clear_transition_matrix_alpha();
  static const int kTransitionMatrixAlphaFieldNumber = 3;
  float transition_matrix_alpha() const;
  void set_transition_matrix_alpha(float value);

  // @@protoc_insertion_point(class_scope:apollo.perception.lidar.CcrfTypeFusionConfig)
 private:
  void set_has_classifiers_property_file_path();
  void clear_has_classifiers_property_file_path();
  void set_has_transition_property_file_path();
  void clear_has_transition_property_file_path();
  void set_has_transition_matrix_alpha();
  void clear_has_transition_matrix_alpha();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr classifiers_property_file_path_;
  ::google::protobuf::internal::ArenaStringPtr transition_property_file_path_;
  float transition_matrix_alpha_;
  friend struct ::protobuf_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// CcrfTypeFusionConfig

// optional string classifiers_property_file_path = 1 [default = ""];
inline bool CcrfTypeFusionConfig::has_classifiers_property_file_path() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void CcrfTypeFusionConfig::set_has_classifiers_property_file_path() {
  _has_bits_[0] |= 0x00000001u;
}
inline void CcrfTypeFusionConfig::clear_has_classifiers_property_file_path() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void CcrfTypeFusionConfig::clear_classifiers_property_file_path() {
  classifiers_property_file_path_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_classifiers_property_file_path();
}
inline const ::std::string& CcrfTypeFusionConfig::classifiers_property_file_path() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.CcrfTypeFusionConfig.classifiers_property_file_path)
  return classifiers_property_file_path_.GetNoArena();
}
inline void CcrfTypeFusionConfig::set_classifiers_property_file_path(const ::std::string& value) {
  set_has_classifiers_property_file_path();
  classifiers_property_file_path_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.CcrfTypeFusionConfig.classifiers_property_file_path)
}
#if LANG_CXX11
inline void CcrfTypeFusionConfig::set_classifiers_property_file_path(::std::string&& value) {
  set_has_classifiers_property_file_path();
  classifiers_property_file_path_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.perception.lidar.CcrfTypeFusionConfig.classifiers_property_file_path)
}
#endif
inline void CcrfTypeFusionConfig::set_classifiers_property_file_path(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_classifiers_property_file_path();
  classifiers_property_file_path_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.perception.lidar.CcrfTypeFusionConfig.classifiers_property_file_path)
}
inline void CcrfTypeFusionConfig::set_classifiers_property_file_path(const char* value, size_t size) {
  set_has_classifiers_property_file_path();
  classifiers_property_file_path_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.perception.lidar.CcrfTypeFusionConfig.classifiers_property_file_path)
}
inline ::std::string* CcrfTypeFusionConfig::mutable_classifiers_property_file_path() {
  set_has_classifiers_property_file_path();
  // @@protoc_insertion_point(field_mutable:apollo.perception.lidar.CcrfTypeFusionConfig.classifiers_property_file_path)
  return classifiers_property_file_path_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* CcrfTypeFusionConfig::release_classifiers_property_file_path() {
  // @@protoc_insertion_point(field_release:apollo.perception.lidar.CcrfTypeFusionConfig.classifiers_property_file_path)
  if (!has_classifiers_property_file_path()) {
    return NULL;
  }
  clear_has_classifiers_property_file_path();
  return classifiers_property_file_path_.ReleaseNonDefaultNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void CcrfTypeFusionConfig::set_allocated_classifiers_property_file_path(::std::string* classifiers_property_file_path) {
  if (classifiers_property_file_path != NULL) {
    set_has_classifiers_property_file_path();
  } else {
    clear_has_classifiers_property_file_path();
  }
  classifiers_property_file_path_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), classifiers_property_file_path);
  // @@protoc_insertion_point(field_set_allocated:apollo.perception.lidar.CcrfTypeFusionConfig.classifiers_property_file_path)
}

// optional string transition_property_file_path = 2 [default = ""];
inline bool CcrfTypeFusionConfig::has_transition_property_file_path() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void CcrfTypeFusionConfig::set_has_transition_property_file_path() {
  _has_bits_[0] |= 0x00000002u;
}
inline void CcrfTypeFusionConfig::clear_has_transition_property_file_path() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void CcrfTypeFusionConfig::clear_transition_property_file_path() {
  transition_property_file_path_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_transition_property_file_path();
}
inline const ::std::string& CcrfTypeFusionConfig::transition_property_file_path() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.CcrfTypeFusionConfig.transition_property_file_path)
  return transition_property_file_path_.GetNoArena();
}
inline void CcrfTypeFusionConfig::set_transition_property_file_path(const ::std::string& value) {
  set_has_transition_property_file_path();
  transition_property_file_path_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.CcrfTypeFusionConfig.transition_property_file_path)
}
#if LANG_CXX11
inline void CcrfTypeFusionConfig::set_transition_property_file_path(::std::string&& value) {
  set_has_transition_property_file_path();
  transition_property_file_path_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.perception.lidar.CcrfTypeFusionConfig.transition_property_file_path)
}
#endif
inline void CcrfTypeFusionConfig::set_transition_property_file_path(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_transition_property_file_path();
  transition_property_file_path_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.perception.lidar.CcrfTypeFusionConfig.transition_property_file_path)
}
inline void CcrfTypeFusionConfig::set_transition_property_file_path(const char* value, size_t size) {
  set_has_transition_property_file_path();
  transition_property_file_path_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.perception.lidar.CcrfTypeFusionConfig.transition_property_file_path)
}
inline ::std::string* CcrfTypeFusionConfig::mutable_transition_property_file_path() {
  set_has_transition_property_file_path();
  // @@protoc_insertion_point(field_mutable:apollo.perception.lidar.CcrfTypeFusionConfig.transition_property_file_path)
  return transition_property_file_path_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* CcrfTypeFusionConfig::release_transition_property_file_path() {
  // @@protoc_insertion_point(field_release:apollo.perception.lidar.CcrfTypeFusionConfig.transition_property_file_path)
  if (!has_transition_property_file_path()) {
    return NULL;
  }
  clear_has_transition_property_file_path();
  return transition_property_file_path_.ReleaseNonDefaultNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void CcrfTypeFusionConfig::set_allocated_transition_property_file_path(::std::string* transition_property_file_path) {
  if (transition_property_file_path != NULL) {
    set_has_transition_property_file_path();
  } else {
    clear_has_transition_property_file_path();
  }
  transition_property_file_path_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), transition_property_file_path);
  // @@protoc_insertion_point(field_set_allocated:apollo.perception.lidar.CcrfTypeFusionConfig.transition_property_file_path)
}

// optional float transition_matrix_alpha = 3 [default = 1.8];
inline bool CcrfTypeFusionConfig::has_transition_matrix_alpha() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void CcrfTypeFusionConfig::set_has_transition_matrix_alpha() {
  _has_bits_[0] |= 0x00000004u;
}
inline void CcrfTypeFusionConfig::clear_has_transition_matrix_alpha() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void CcrfTypeFusionConfig::clear_transition_matrix_alpha() {
  transition_matrix_alpha_ = 1.8f;
  clear_has_transition_matrix_alpha();
}
inline float CcrfTypeFusionConfig::transition_matrix_alpha() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.CcrfTypeFusionConfig.transition_matrix_alpha)
  return transition_matrix_alpha_;
}
inline void CcrfTypeFusionConfig::set_transition_matrix_alpha(float value) {
  set_has_transition_matrix_alpha();
  transition_matrix_alpha_ = value;
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.CcrfTypeFusionConfig.transition_matrix_alpha)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto
