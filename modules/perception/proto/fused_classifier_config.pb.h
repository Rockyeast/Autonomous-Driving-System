// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/proto/fused_classifier_config.proto

#ifndef PROTOBUF_INCLUDED_modules_2fperception_2fproto_2ffused_5fclassifier_5fconfig_2eproto
#define PROTOBUF_INCLUDED_modules_2fperception_2fproto_2ffused_5fclassifier_5fconfig_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fperception_2fproto_2ffused_5fclassifier_5fconfig_2eproto 

namespace protobuf_modules_2fperception_2fproto_2ffused_5fclassifier_5fconfig_2eproto {
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
}  // namespace protobuf_modules_2fperception_2fproto_2ffused_5fclassifier_5fconfig_2eproto
namespace apollo {
namespace perception {
namespace lidar {
class FusedClassifierConfig;
class FusedClassifierConfigDefaultTypeInternal;
extern FusedClassifierConfigDefaultTypeInternal _FusedClassifierConfig_default_instance_;
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::perception::lidar::FusedClassifierConfig* Arena::CreateMaybeMessage<::apollo::perception::lidar::FusedClassifierConfig>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace perception {
namespace lidar {

// ===================================================================

class FusedClassifierConfig : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.perception.lidar.FusedClassifierConfig) */ {
 public:
  FusedClassifierConfig();
  virtual ~FusedClassifierConfig();

  FusedClassifierConfig(const FusedClassifierConfig& from);

  inline FusedClassifierConfig& operator=(const FusedClassifierConfig& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  FusedClassifierConfig(FusedClassifierConfig&& from) noexcept
    : FusedClassifierConfig() {
    *this = ::std::move(from);
  }

  inline FusedClassifierConfig& operator=(FusedClassifierConfig&& from) noexcept {
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
  static const FusedClassifierConfig& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const FusedClassifierConfig* internal_default_instance() {
    return reinterpret_cast<const FusedClassifierConfig*>(
               &_FusedClassifierConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(FusedClassifierConfig* other);
  friend void swap(FusedClassifierConfig& a, FusedClassifierConfig& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline FusedClassifierConfig* New() const final {
    return CreateMaybeMessage<FusedClassifierConfig>(NULL);
  }

  FusedClassifierConfig* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<FusedClassifierConfig>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const FusedClassifierConfig& from);
  void MergeFrom(const FusedClassifierConfig& from);
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
  void InternalSwap(FusedClassifierConfig* other);
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

  // optional string one_shot_fusion_method = 3 [default = "CCRFOneShotTypeFusion"];
  bool has_one_shot_fusion_method() const;
  void clear_one_shot_fusion_method();
  static const int kOneShotFusionMethodFieldNumber = 3;
  const ::std::string& one_shot_fusion_method() const;
  void set_one_shot_fusion_method(const ::std::string& value);
  #if LANG_CXX11
  void set_one_shot_fusion_method(::std::string&& value);
  #endif
  void set_one_shot_fusion_method(const char* value);
  void set_one_shot_fusion_method(const char* value, size_t size);
  ::std::string* mutable_one_shot_fusion_method();
  ::std::string* release_one_shot_fusion_method();
  void set_allocated_one_shot_fusion_method(::std::string* one_shot_fusion_method);

  // optional string sequence_fusion_method = 4 [default = "CCRFSequenceTypeFusion"];
  bool has_sequence_fusion_method() const;
  void clear_sequence_fusion_method();
  static const int kSequenceFusionMethodFieldNumber = 4;
  const ::std::string& sequence_fusion_method() const;
  void set_sequence_fusion_method(const ::std::string& value);
  #if LANG_CXX11
  void set_sequence_fusion_method(::std::string&& value);
  #endif
  void set_sequence_fusion_method(const char* value);
  void set_sequence_fusion_method(const char* value, size_t size);
  ::std::string* mutable_sequence_fusion_method();
  ::std::string* release_sequence_fusion_method();
  void set_allocated_sequence_fusion_method(::std::string* sequence_fusion_method);

  // optional bool enable_temporal_fusion = 2 [default = true];
  bool has_enable_temporal_fusion() const;
  void clear_enable_temporal_fusion();
  static const int kEnableTemporalFusionFieldNumber = 2;
  bool enable_temporal_fusion() const;
  void set_enable_temporal_fusion(bool value);

  // optional bool use_tracked_objects = 5 [default = true];
  bool has_use_tracked_objects() const;
  void clear_use_tracked_objects();
  static const int kUseTrackedObjectsFieldNumber = 5;
  bool use_tracked_objects() const;
  void set_use_tracked_objects(bool value);

  // optional double temporal_window = 1 [default = 20];
  bool has_temporal_window() const;
  void clear_temporal_window();
  static const int kTemporalWindowFieldNumber = 1;
  double temporal_window() const;
  void set_temporal_window(double value);

  // @@protoc_insertion_point(class_scope:apollo.perception.lidar.FusedClassifierConfig)
 private:
  void set_has_temporal_window();
  void clear_has_temporal_window();
  void set_has_enable_temporal_fusion();
  void clear_has_enable_temporal_fusion();
  void set_has_one_shot_fusion_method();
  void clear_has_one_shot_fusion_method();
  void set_has_sequence_fusion_method();
  void clear_has_sequence_fusion_method();
  void set_has_use_tracked_objects();
  void clear_has_use_tracked_objects();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  public:
  static ::google::protobuf::internal::ExplicitlyConstructed< ::std::string> _i_give_permission_to_break_this_code_default_one_shot_fusion_method_;
  private:
  ::google::protobuf::internal::ArenaStringPtr one_shot_fusion_method_;
  public:
  static ::google::protobuf::internal::ExplicitlyConstructed< ::std::string> _i_give_permission_to_break_this_code_default_sequence_fusion_method_;
  private:
  ::google::protobuf::internal::ArenaStringPtr sequence_fusion_method_;
  bool enable_temporal_fusion_;
  bool use_tracked_objects_;
  double temporal_window_;
  friend struct ::protobuf_modules_2fperception_2fproto_2ffused_5fclassifier_5fconfig_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// FusedClassifierConfig

// optional double temporal_window = 1 [default = 20];
inline bool FusedClassifierConfig::has_temporal_window() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void FusedClassifierConfig::set_has_temporal_window() {
  _has_bits_[0] |= 0x00000010u;
}
inline void FusedClassifierConfig::clear_has_temporal_window() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void FusedClassifierConfig::clear_temporal_window() {
  temporal_window_ = 20;
  clear_has_temporal_window();
}
inline double FusedClassifierConfig::temporal_window() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.FusedClassifierConfig.temporal_window)
  return temporal_window_;
}
inline void FusedClassifierConfig::set_temporal_window(double value) {
  set_has_temporal_window();
  temporal_window_ = value;
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.FusedClassifierConfig.temporal_window)
}

// optional bool enable_temporal_fusion = 2 [default = true];
inline bool FusedClassifierConfig::has_enable_temporal_fusion() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void FusedClassifierConfig::set_has_enable_temporal_fusion() {
  _has_bits_[0] |= 0x00000004u;
}
inline void FusedClassifierConfig::clear_has_enable_temporal_fusion() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void FusedClassifierConfig::clear_enable_temporal_fusion() {
  enable_temporal_fusion_ = true;
  clear_has_enable_temporal_fusion();
}
inline bool FusedClassifierConfig::enable_temporal_fusion() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.FusedClassifierConfig.enable_temporal_fusion)
  return enable_temporal_fusion_;
}
inline void FusedClassifierConfig::set_enable_temporal_fusion(bool value) {
  set_has_enable_temporal_fusion();
  enable_temporal_fusion_ = value;
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.FusedClassifierConfig.enable_temporal_fusion)
}

// optional string one_shot_fusion_method = 3 [default = "CCRFOneShotTypeFusion"];
inline bool FusedClassifierConfig::has_one_shot_fusion_method() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void FusedClassifierConfig::set_has_one_shot_fusion_method() {
  _has_bits_[0] |= 0x00000001u;
}
inline void FusedClassifierConfig::clear_has_one_shot_fusion_method() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void FusedClassifierConfig::clear_one_shot_fusion_method() {
  one_shot_fusion_method_.ClearToDefaultNoArena(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_one_shot_fusion_method_.get());
  clear_has_one_shot_fusion_method();
}
inline const ::std::string& FusedClassifierConfig::one_shot_fusion_method() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.FusedClassifierConfig.one_shot_fusion_method)
  return one_shot_fusion_method_.GetNoArena();
}
inline void FusedClassifierConfig::set_one_shot_fusion_method(const ::std::string& value) {
  set_has_one_shot_fusion_method();
  one_shot_fusion_method_.SetNoArena(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_one_shot_fusion_method_.get(), value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.FusedClassifierConfig.one_shot_fusion_method)
}
#if LANG_CXX11
inline void FusedClassifierConfig::set_one_shot_fusion_method(::std::string&& value) {
  set_has_one_shot_fusion_method();
  one_shot_fusion_method_.SetNoArena(
    &::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_one_shot_fusion_method_.get(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.perception.lidar.FusedClassifierConfig.one_shot_fusion_method)
}
#endif
inline void FusedClassifierConfig::set_one_shot_fusion_method(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_one_shot_fusion_method();
  one_shot_fusion_method_.SetNoArena(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_one_shot_fusion_method_.get(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.perception.lidar.FusedClassifierConfig.one_shot_fusion_method)
}
inline void FusedClassifierConfig::set_one_shot_fusion_method(const char* value, size_t size) {
  set_has_one_shot_fusion_method();
  one_shot_fusion_method_.SetNoArena(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_one_shot_fusion_method_.get(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.perception.lidar.FusedClassifierConfig.one_shot_fusion_method)
}
inline ::std::string* FusedClassifierConfig::mutable_one_shot_fusion_method() {
  set_has_one_shot_fusion_method();
  // @@protoc_insertion_point(field_mutable:apollo.perception.lidar.FusedClassifierConfig.one_shot_fusion_method)
  return one_shot_fusion_method_.MutableNoArena(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_one_shot_fusion_method_.get());
}
inline ::std::string* FusedClassifierConfig::release_one_shot_fusion_method() {
  // @@protoc_insertion_point(field_release:apollo.perception.lidar.FusedClassifierConfig.one_shot_fusion_method)
  if (!has_one_shot_fusion_method()) {
    return NULL;
  }
  clear_has_one_shot_fusion_method();
  return one_shot_fusion_method_.ReleaseNonDefaultNoArena(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_one_shot_fusion_method_.get());
}
inline void FusedClassifierConfig::set_allocated_one_shot_fusion_method(::std::string* one_shot_fusion_method) {
  if (one_shot_fusion_method != NULL) {
    set_has_one_shot_fusion_method();
  } else {
    clear_has_one_shot_fusion_method();
  }
  one_shot_fusion_method_.SetAllocatedNoArena(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_one_shot_fusion_method_.get(), one_shot_fusion_method);
  // @@protoc_insertion_point(field_set_allocated:apollo.perception.lidar.FusedClassifierConfig.one_shot_fusion_method)
}

// optional string sequence_fusion_method = 4 [default = "CCRFSequenceTypeFusion"];
inline bool FusedClassifierConfig::has_sequence_fusion_method() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void FusedClassifierConfig::set_has_sequence_fusion_method() {
  _has_bits_[0] |= 0x00000002u;
}
inline void FusedClassifierConfig::clear_has_sequence_fusion_method() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void FusedClassifierConfig::clear_sequence_fusion_method() {
  sequence_fusion_method_.ClearToDefaultNoArena(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_sequence_fusion_method_.get());
  clear_has_sequence_fusion_method();
}
inline const ::std::string& FusedClassifierConfig::sequence_fusion_method() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.FusedClassifierConfig.sequence_fusion_method)
  return sequence_fusion_method_.GetNoArena();
}
inline void FusedClassifierConfig::set_sequence_fusion_method(const ::std::string& value) {
  set_has_sequence_fusion_method();
  sequence_fusion_method_.SetNoArena(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_sequence_fusion_method_.get(), value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.FusedClassifierConfig.sequence_fusion_method)
}
#if LANG_CXX11
inline void FusedClassifierConfig::set_sequence_fusion_method(::std::string&& value) {
  set_has_sequence_fusion_method();
  sequence_fusion_method_.SetNoArena(
    &::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_sequence_fusion_method_.get(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.perception.lidar.FusedClassifierConfig.sequence_fusion_method)
}
#endif
inline void FusedClassifierConfig::set_sequence_fusion_method(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_sequence_fusion_method();
  sequence_fusion_method_.SetNoArena(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_sequence_fusion_method_.get(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.perception.lidar.FusedClassifierConfig.sequence_fusion_method)
}
inline void FusedClassifierConfig::set_sequence_fusion_method(const char* value, size_t size) {
  set_has_sequence_fusion_method();
  sequence_fusion_method_.SetNoArena(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_sequence_fusion_method_.get(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.perception.lidar.FusedClassifierConfig.sequence_fusion_method)
}
inline ::std::string* FusedClassifierConfig::mutable_sequence_fusion_method() {
  set_has_sequence_fusion_method();
  // @@protoc_insertion_point(field_mutable:apollo.perception.lidar.FusedClassifierConfig.sequence_fusion_method)
  return sequence_fusion_method_.MutableNoArena(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_sequence_fusion_method_.get());
}
inline ::std::string* FusedClassifierConfig::release_sequence_fusion_method() {
  // @@protoc_insertion_point(field_release:apollo.perception.lidar.FusedClassifierConfig.sequence_fusion_method)
  if (!has_sequence_fusion_method()) {
    return NULL;
  }
  clear_has_sequence_fusion_method();
  return sequence_fusion_method_.ReleaseNonDefaultNoArena(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_sequence_fusion_method_.get());
}
inline void FusedClassifierConfig::set_allocated_sequence_fusion_method(::std::string* sequence_fusion_method) {
  if (sequence_fusion_method != NULL) {
    set_has_sequence_fusion_method();
  } else {
    clear_has_sequence_fusion_method();
  }
  sequence_fusion_method_.SetAllocatedNoArena(&::apollo::perception::lidar::FusedClassifierConfig::_i_give_permission_to_break_this_code_default_sequence_fusion_method_.get(), sequence_fusion_method);
  // @@protoc_insertion_point(field_set_allocated:apollo.perception.lidar.FusedClassifierConfig.sequence_fusion_method)
}

// optional bool use_tracked_objects = 5 [default = true];
inline bool FusedClassifierConfig::has_use_tracked_objects() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void FusedClassifierConfig::set_has_use_tracked_objects() {
  _has_bits_[0] |= 0x00000008u;
}
inline void FusedClassifierConfig::clear_has_use_tracked_objects() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void FusedClassifierConfig::clear_use_tracked_objects() {
  use_tracked_objects_ = true;
  clear_has_use_tracked_objects();
}
inline bool FusedClassifierConfig::use_tracked_objects() const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.FusedClassifierConfig.use_tracked_objects)
  return use_tracked_objects_;
}
inline void FusedClassifierConfig::set_use_tracked_objects(bool value) {
  set_has_use_tracked_objects();
  use_tracked_objects_ = value;
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.FusedClassifierConfig.use_tracked_objects)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_modules_2fperception_2fproto_2ffused_5fclassifier_5fconfig_2eproto
