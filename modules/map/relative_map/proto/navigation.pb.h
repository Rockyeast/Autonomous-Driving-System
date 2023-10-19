// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/map/relative_map/proto/navigation.proto

#ifndef PROTOBUF_INCLUDED_modules_2fmap_2frelative_5fmap_2fproto_2fnavigation_2eproto
#define PROTOBUF_INCLUDED_modules_2fmap_2frelative_5fmap_2fproto_2fnavigation_2eproto

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
#include <google/protobuf/map.h>  // IWYU pragma: export
#include <google/protobuf/map_entry.h>
#include <google/protobuf/map_field_inl.h>
#include <google/protobuf/unknown_field_set.h>
#include "modules/common/proto/header.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/map/proto/map.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fmap_2frelative_5fmap_2fproto_2fnavigation_2eproto 

namespace protobuf_modules_2fmap_2frelative_5fmap_2fproto_2fnavigation_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[4];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_modules_2fmap_2frelative_5fmap_2fproto_2fnavigation_2eproto
namespace apollo {
namespace relative_map {
class MapMsg;
class MapMsgDefaultTypeInternal;
extern MapMsgDefaultTypeInternal _MapMsg_default_instance_;
class MapMsg_NavigationPathEntry_DoNotUse;
class MapMsg_NavigationPathEntry_DoNotUseDefaultTypeInternal;
extern MapMsg_NavigationPathEntry_DoNotUseDefaultTypeInternal _MapMsg_NavigationPathEntry_DoNotUse_default_instance_;
class NavigationInfo;
class NavigationInfoDefaultTypeInternal;
extern NavigationInfoDefaultTypeInternal _NavigationInfo_default_instance_;
class NavigationPath;
class NavigationPathDefaultTypeInternal;
extern NavigationPathDefaultTypeInternal _NavigationPath_default_instance_;
}  // namespace relative_map
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::relative_map::MapMsg* Arena::CreateMaybeMessage<::apollo::relative_map::MapMsg>(Arena*);
template<> ::apollo::relative_map::MapMsg_NavigationPathEntry_DoNotUse* Arena::CreateMaybeMessage<::apollo::relative_map::MapMsg_NavigationPathEntry_DoNotUse>(Arena*);
template<> ::apollo::relative_map::NavigationInfo* Arena::CreateMaybeMessage<::apollo::relative_map::NavigationInfo>(Arena*);
template<> ::apollo::relative_map::NavigationPath* Arena::CreateMaybeMessage<::apollo::relative_map::NavigationPath>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace relative_map {

// ===================================================================

class NavigationPath : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.relative_map.NavigationPath) */ {
 public:
  NavigationPath();
  virtual ~NavigationPath();

  NavigationPath(const NavigationPath& from);

  inline NavigationPath& operator=(const NavigationPath& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  NavigationPath(NavigationPath&& from) noexcept
    : NavigationPath() {
    *this = ::std::move(from);
  }

  inline NavigationPath& operator=(NavigationPath&& from) noexcept {
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
  static const NavigationPath& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const NavigationPath* internal_default_instance() {
    return reinterpret_cast<const NavigationPath*>(
               &_NavigationPath_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(NavigationPath* other);
  friend void swap(NavigationPath& a, NavigationPath& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline NavigationPath* New() const final {
    return CreateMaybeMessage<NavigationPath>(NULL);
  }

  NavigationPath* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<NavigationPath>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const NavigationPath& from);
  void MergeFrom(const NavigationPath& from);
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
  void InternalSwap(NavigationPath* other);
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

  // optional .apollo.common.Path path = 1;
  bool has_path() const;
  void clear_path();
  static const int kPathFieldNumber = 1;
  private:
  const ::apollo::common::Path& _internal_path() const;
  public:
  const ::apollo::common::Path& path() const;
  ::apollo::common::Path* release_path();
  ::apollo::common::Path* mutable_path();
  void set_allocated_path(::apollo::common::Path* path);

  // optional uint32 path_priority = 2;
  bool has_path_priority() const;
  void clear_path_priority();
  static const int kPathPriorityFieldNumber = 2;
  ::google::protobuf::uint32 path_priority() const;
  void set_path_priority(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:apollo.relative_map.NavigationPath)
 private:
  void set_has_path();
  void clear_has_path();
  void set_has_path_priority();
  void clear_has_path_priority();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::apollo::common::Path* path_;
  ::google::protobuf::uint32 path_priority_;
  friend struct ::protobuf_modules_2fmap_2frelative_5fmap_2fproto_2fnavigation_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class NavigationInfo : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.relative_map.NavigationInfo) */ {
 public:
  NavigationInfo();
  virtual ~NavigationInfo();

  NavigationInfo(const NavigationInfo& from);

  inline NavigationInfo& operator=(const NavigationInfo& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  NavigationInfo(NavigationInfo&& from) noexcept
    : NavigationInfo() {
    *this = ::std::move(from);
  }

  inline NavigationInfo& operator=(NavigationInfo&& from) noexcept {
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
  static const NavigationInfo& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const NavigationInfo* internal_default_instance() {
    return reinterpret_cast<const NavigationInfo*>(
               &_NavigationInfo_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(NavigationInfo* other);
  friend void swap(NavigationInfo& a, NavigationInfo& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline NavigationInfo* New() const final {
    return CreateMaybeMessage<NavigationInfo>(NULL);
  }

  NavigationInfo* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<NavigationInfo>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const NavigationInfo& from);
  void MergeFrom(const NavigationInfo& from);
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
  void InternalSwap(NavigationInfo* other);
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

  // repeated .apollo.relative_map.NavigationPath navigation_path = 2;
  int navigation_path_size() const;
  void clear_navigation_path();
  static const int kNavigationPathFieldNumber = 2;
  ::apollo::relative_map::NavigationPath* mutable_navigation_path(int index);
  ::google::protobuf::RepeatedPtrField< ::apollo::relative_map::NavigationPath >*
      mutable_navigation_path();
  const ::apollo::relative_map::NavigationPath& navigation_path(int index) const;
  ::apollo::relative_map::NavigationPath* add_navigation_path();
  const ::google::protobuf::RepeatedPtrField< ::apollo::relative_map::NavigationPath >&
      navigation_path() const;

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

  // @@protoc_insertion_point(class_scope:apollo.relative_map.NavigationInfo)
 private:
  void set_has_header();
  void clear_has_header();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::apollo::relative_map::NavigationPath > navigation_path_;
  ::apollo::common::Header* header_;
  friend struct ::protobuf_modules_2fmap_2frelative_5fmap_2fproto_2fnavigation_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class MapMsg_NavigationPathEntry_DoNotUse : public ::google::protobuf::internal::MapEntry<MapMsg_NavigationPathEntry_DoNotUse, 
    ::std::string, ::apollo::relative_map::NavigationPath,
    ::google::protobuf::internal::WireFormatLite::TYPE_STRING,
    ::google::protobuf::internal::WireFormatLite::TYPE_MESSAGE,
    0 > {
public:
  typedef ::google::protobuf::internal::MapEntry<MapMsg_NavigationPathEntry_DoNotUse, 
    ::std::string, ::apollo::relative_map::NavigationPath,
    ::google::protobuf::internal::WireFormatLite::TYPE_STRING,
    ::google::protobuf::internal::WireFormatLite::TYPE_MESSAGE,
    0 > SuperType;
  MapMsg_NavigationPathEntry_DoNotUse();
  MapMsg_NavigationPathEntry_DoNotUse(::google::protobuf::Arena* arena);
  void MergeFrom(const MapMsg_NavigationPathEntry_DoNotUse& other);
  static const MapMsg_NavigationPathEntry_DoNotUse* internal_default_instance() { return reinterpret_cast<const MapMsg_NavigationPathEntry_DoNotUse*>(&_MapMsg_NavigationPathEntry_DoNotUse_default_instance_); }
  void MergeFrom(const ::google::protobuf::Message& other) final;
  ::google::protobuf::Metadata GetMetadata() const;
};

// -------------------------------------------------------------------

class MapMsg : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.relative_map.MapMsg) */ {
 public:
  MapMsg();
  virtual ~MapMsg();

  MapMsg(const MapMsg& from);

  inline MapMsg& operator=(const MapMsg& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  MapMsg(MapMsg&& from) noexcept
    : MapMsg() {
    *this = ::std::move(from);
  }

  inline MapMsg& operator=(MapMsg&& from) noexcept {
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
  static const MapMsg& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MapMsg* internal_default_instance() {
    return reinterpret_cast<const MapMsg*>(
               &_MapMsg_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    3;

  void Swap(MapMsg* other);
  friend void swap(MapMsg& a, MapMsg& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline MapMsg* New() const final {
    return CreateMaybeMessage<MapMsg>(NULL);
  }

  MapMsg* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<MapMsg>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const MapMsg& from);
  void MergeFrom(const MapMsg& from);
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
  void InternalSwap(MapMsg* other);
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

  // map<string, .apollo.relative_map.NavigationPath> navigation_path = 3;
  int navigation_path_size() const;
  void clear_navigation_path();
  static const int kNavigationPathFieldNumber = 3;
  const ::google::protobuf::Map< ::std::string, ::apollo::relative_map::NavigationPath >&
      navigation_path() const;
  ::google::protobuf::Map< ::std::string, ::apollo::relative_map::NavigationPath >*
      mutable_navigation_path();

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

  // optional .apollo.hdmap.Map hdmap = 2;
  bool has_hdmap() const;
  void clear_hdmap();
  static const int kHdmapFieldNumber = 2;
  private:
  const ::apollo::hdmap::Map& _internal_hdmap() const;
  public:
  const ::apollo::hdmap::Map& hdmap() const;
  ::apollo::hdmap::Map* release_hdmap();
  ::apollo::hdmap::Map* mutable_hdmap();
  void set_allocated_hdmap(::apollo::hdmap::Map* hdmap);

  // optional .apollo.perception.LaneMarkers lane_marker = 4;
  bool has_lane_marker() const;
  void clear_lane_marker();
  static const int kLaneMarkerFieldNumber = 4;
  private:
  const ::apollo::perception::LaneMarkers& _internal_lane_marker() const;
  public:
  const ::apollo::perception::LaneMarkers& lane_marker() const;
  ::apollo::perception::LaneMarkers* release_lane_marker();
  ::apollo::perception::LaneMarkers* mutable_lane_marker();
  void set_allocated_lane_marker(::apollo::perception::LaneMarkers* lane_marker);

  // optional .apollo.localization.LocalizationEstimate localization = 5;
  bool has_localization() const;
  void clear_localization();
  static const int kLocalizationFieldNumber = 5;
  private:
  const ::apollo::localization::LocalizationEstimate& _internal_localization() const;
  public:
  const ::apollo::localization::LocalizationEstimate& localization() const;
  ::apollo::localization::LocalizationEstimate* release_localization();
  ::apollo::localization::LocalizationEstimate* mutable_localization();
  void set_allocated_localization(::apollo::localization::LocalizationEstimate* localization);

  // @@protoc_insertion_point(class_scope:apollo.relative_map.MapMsg)
 private:
  void set_has_header();
  void clear_has_header();
  void set_has_hdmap();
  void clear_has_hdmap();
  void set_has_lane_marker();
  void clear_has_lane_marker();
  void set_has_localization();
  void clear_has_localization();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::internal::MapField<
      MapMsg_NavigationPathEntry_DoNotUse,
      ::std::string, ::apollo::relative_map::NavigationPath,
      ::google::protobuf::internal::WireFormatLite::TYPE_STRING,
      ::google::protobuf::internal::WireFormatLite::TYPE_MESSAGE,
      0 > navigation_path_;
  ::apollo::common::Header* header_;
  ::apollo::hdmap::Map* hdmap_;
  ::apollo::perception::LaneMarkers* lane_marker_;
  ::apollo::localization::LocalizationEstimate* localization_;
  friend struct ::protobuf_modules_2fmap_2frelative_5fmap_2fproto_2fnavigation_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// NavigationPath

// optional .apollo.common.Path path = 1;
inline bool NavigationPath::has_path() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void NavigationPath::set_has_path() {
  _has_bits_[0] |= 0x00000001u;
}
inline void NavigationPath::clear_has_path() {
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::apollo::common::Path& NavigationPath::_internal_path() const {
  return *path_;
}
inline const ::apollo::common::Path& NavigationPath::path() const {
  const ::apollo::common::Path* p = path_;
  // @@protoc_insertion_point(field_get:apollo.relative_map.NavigationPath.path)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::common::Path*>(
      &::apollo::common::_Path_default_instance_);
}
inline ::apollo::common::Path* NavigationPath::release_path() {
  // @@protoc_insertion_point(field_release:apollo.relative_map.NavigationPath.path)
  clear_has_path();
  ::apollo::common::Path* temp = path_;
  path_ = NULL;
  return temp;
}
inline ::apollo::common::Path* NavigationPath::mutable_path() {
  set_has_path();
  if (path_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::common::Path>(GetArenaNoVirtual());
    path_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.relative_map.NavigationPath.path)
  return path_;
}
inline void NavigationPath::set_allocated_path(::apollo::common::Path* path) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(path_);
  }
  if (path) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      path = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, path, submessage_arena);
    }
    set_has_path();
  } else {
    clear_has_path();
  }
  path_ = path;
  // @@protoc_insertion_point(field_set_allocated:apollo.relative_map.NavigationPath.path)
}

// optional uint32 path_priority = 2;
inline bool NavigationPath::has_path_priority() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void NavigationPath::set_has_path_priority() {
  _has_bits_[0] |= 0x00000002u;
}
inline void NavigationPath::clear_has_path_priority() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void NavigationPath::clear_path_priority() {
  path_priority_ = 0u;
  clear_has_path_priority();
}
inline ::google::protobuf::uint32 NavigationPath::path_priority() const {
  // @@protoc_insertion_point(field_get:apollo.relative_map.NavigationPath.path_priority)
  return path_priority_;
}
inline void NavigationPath::set_path_priority(::google::protobuf::uint32 value) {
  set_has_path_priority();
  path_priority_ = value;
  // @@protoc_insertion_point(field_set:apollo.relative_map.NavigationPath.path_priority)
}

// -------------------------------------------------------------------

// NavigationInfo

// optional .apollo.common.Header header = 1;
inline bool NavigationInfo::has_header() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void NavigationInfo::set_has_header() {
  _has_bits_[0] |= 0x00000001u;
}
inline void NavigationInfo::clear_has_header() {
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::apollo::common::Header& NavigationInfo::_internal_header() const {
  return *header_;
}
inline const ::apollo::common::Header& NavigationInfo::header() const {
  const ::apollo::common::Header* p = header_;
  // @@protoc_insertion_point(field_get:apollo.relative_map.NavigationInfo.header)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::common::Header*>(
      &::apollo::common::_Header_default_instance_);
}
inline ::apollo::common::Header* NavigationInfo::release_header() {
  // @@protoc_insertion_point(field_release:apollo.relative_map.NavigationInfo.header)
  clear_has_header();
  ::apollo::common::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::apollo::common::Header* NavigationInfo::mutable_header() {
  set_has_header();
  if (header_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::common::Header>(GetArenaNoVirtual());
    header_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.relative_map.NavigationInfo.header)
  return header_;
}
inline void NavigationInfo::set_allocated_header(::apollo::common::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:apollo.relative_map.NavigationInfo.header)
}

// repeated .apollo.relative_map.NavigationPath navigation_path = 2;
inline int NavigationInfo::navigation_path_size() const {
  return navigation_path_.size();
}
inline void NavigationInfo::clear_navigation_path() {
  navigation_path_.Clear();
}
inline ::apollo::relative_map::NavigationPath* NavigationInfo::mutable_navigation_path(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.relative_map.NavigationInfo.navigation_path)
  return navigation_path_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::relative_map::NavigationPath >*
NavigationInfo::mutable_navigation_path() {
  // @@protoc_insertion_point(field_mutable_list:apollo.relative_map.NavigationInfo.navigation_path)
  return &navigation_path_;
}
inline const ::apollo::relative_map::NavigationPath& NavigationInfo::navigation_path(int index) const {
  // @@protoc_insertion_point(field_get:apollo.relative_map.NavigationInfo.navigation_path)
  return navigation_path_.Get(index);
}
inline ::apollo::relative_map::NavigationPath* NavigationInfo::add_navigation_path() {
  // @@protoc_insertion_point(field_add:apollo.relative_map.NavigationInfo.navigation_path)
  return navigation_path_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::relative_map::NavigationPath >&
NavigationInfo::navigation_path() const {
  // @@protoc_insertion_point(field_list:apollo.relative_map.NavigationInfo.navigation_path)
  return navigation_path_;
}

// -------------------------------------------------------------------

// -------------------------------------------------------------------

// MapMsg

// optional .apollo.common.Header header = 1;
inline bool MapMsg::has_header() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void MapMsg::set_has_header() {
  _has_bits_[0] |= 0x00000001u;
}
inline void MapMsg::clear_has_header() {
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::apollo::common::Header& MapMsg::_internal_header() const {
  return *header_;
}
inline const ::apollo::common::Header& MapMsg::header() const {
  const ::apollo::common::Header* p = header_;
  // @@protoc_insertion_point(field_get:apollo.relative_map.MapMsg.header)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::common::Header*>(
      &::apollo::common::_Header_default_instance_);
}
inline ::apollo::common::Header* MapMsg::release_header() {
  // @@protoc_insertion_point(field_release:apollo.relative_map.MapMsg.header)
  clear_has_header();
  ::apollo::common::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::apollo::common::Header* MapMsg::mutable_header() {
  set_has_header();
  if (header_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::common::Header>(GetArenaNoVirtual());
    header_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.relative_map.MapMsg.header)
  return header_;
}
inline void MapMsg::set_allocated_header(::apollo::common::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:apollo.relative_map.MapMsg.header)
}

// optional .apollo.hdmap.Map hdmap = 2;
inline bool MapMsg::has_hdmap() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void MapMsg::set_has_hdmap() {
  _has_bits_[0] |= 0x00000002u;
}
inline void MapMsg::clear_has_hdmap() {
  _has_bits_[0] &= ~0x00000002u;
}
inline const ::apollo::hdmap::Map& MapMsg::_internal_hdmap() const {
  return *hdmap_;
}
inline const ::apollo::hdmap::Map& MapMsg::hdmap() const {
  const ::apollo::hdmap::Map* p = hdmap_;
  // @@protoc_insertion_point(field_get:apollo.relative_map.MapMsg.hdmap)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::hdmap::Map*>(
      &::apollo::hdmap::_Map_default_instance_);
}
inline ::apollo::hdmap::Map* MapMsg::release_hdmap() {
  // @@protoc_insertion_point(field_release:apollo.relative_map.MapMsg.hdmap)
  clear_has_hdmap();
  ::apollo::hdmap::Map* temp = hdmap_;
  hdmap_ = NULL;
  return temp;
}
inline ::apollo::hdmap::Map* MapMsg::mutable_hdmap() {
  set_has_hdmap();
  if (hdmap_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::hdmap::Map>(GetArenaNoVirtual());
    hdmap_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.relative_map.MapMsg.hdmap)
  return hdmap_;
}
inline void MapMsg::set_allocated_hdmap(::apollo::hdmap::Map* hdmap) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(hdmap_);
  }
  if (hdmap) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      hdmap = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, hdmap, submessage_arena);
    }
    set_has_hdmap();
  } else {
    clear_has_hdmap();
  }
  hdmap_ = hdmap;
  // @@protoc_insertion_point(field_set_allocated:apollo.relative_map.MapMsg.hdmap)
}

// map<string, .apollo.relative_map.NavigationPath> navigation_path = 3;
inline int MapMsg::navigation_path_size() const {
  return navigation_path_.size();
}
inline void MapMsg::clear_navigation_path() {
  navigation_path_.Clear();
}
inline const ::google::protobuf::Map< ::std::string, ::apollo::relative_map::NavigationPath >&
MapMsg::navigation_path() const {
  // @@protoc_insertion_point(field_map:apollo.relative_map.MapMsg.navigation_path)
  return navigation_path_.GetMap();
}
inline ::google::protobuf::Map< ::std::string, ::apollo::relative_map::NavigationPath >*
MapMsg::mutable_navigation_path() {
  // @@protoc_insertion_point(field_mutable_map:apollo.relative_map.MapMsg.navigation_path)
  return navigation_path_.MutableMap();
}

// optional .apollo.perception.LaneMarkers lane_marker = 4;
inline bool MapMsg::has_lane_marker() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void MapMsg::set_has_lane_marker() {
  _has_bits_[0] |= 0x00000004u;
}
inline void MapMsg::clear_has_lane_marker() {
  _has_bits_[0] &= ~0x00000004u;
}
inline const ::apollo::perception::LaneMarkers& MapMsg::_internal_lane_marker() const {
  return *lane_marker_;
}
inline const ::apollo::perception::LaneMarkers& MapMsg::lane_marker() const {
  const ::apollo::perception::LaneMarkers* p = lane_marker_;
  // @@protoc_insertion_point(field_get:apollo.relative_map.MapMsg.lane_marker)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::perception::LaneMarkers*>(
      &::apollo::perception::_LaneMarkers_default_instance_);
}
inline ::apollo::perception::LaneMarkers* MapMsg::release_lane_marker() {
  // @@protoc_insertion_point(field_release:apollo.relative_map.MapMsg.lane_marker)
  clear_has_lane_marker();
  ::apollo::perception::LaneMarkers* temp = lane_marker_;
  lane_marker_ = NULL;
  return temp;
}
inline ::apollo::perception::LaneMarkers* MapMsg::mutable_lane_marker() {
  set_has_lane_marker();
  if (lane_marker_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::perception::LaneMarkers>(GetArenaNoVirtual());
    lane_marker_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.relative_map.MapMsg.lane_marker)
  return lane_marker_;
}
inline void MapMsg::set_allocated_lane_marker(::apollo::perception::LaneMarkers* lane_marker) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(lane_marker_);
  }
  if (lane_marker) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      lane_marker = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, lane_marker, submessage_arena);
    }
    set_has_lane_marker();
  } else {
    clear_has_lane_marker();
  }
  lane_marker_ = lane_marker;
  // @@protoc_insertion_point(field_set_allocated:apollo.relative_map.MapMsg.lane_marker)
}

// optional .apollo.localization.LocalizationEstimate localization = 5;
inline bool MapMsg::has_localization() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void MapMsg::set_has_localization() {
  _has_bits_[0] |= 0x00000008u;
}
inline void MapMsg::clear_has_localization() {
  _has_bits_[0] &= ~0x00000008u;
}
inline const ::apollo::localization::LocalizationEstimate& MapMsg::_internal_localization() const {
  return *localization_;
}
inline const ::apollo::localization::LocalizationEstimate& MapMsg::localization() const {
  const ::apollo::localization::LocalizationEstimate* p = localization_;
  // @@protoc_insertion_point(field_get:apollo.relative_map.MapMsg.localization)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::localization::LocalizationEstimate*>(
      &::apollo::localization::_LocalizationEstimate_default_instance_);
}
inline ::apollo::localization::LocalizationEstimate* MapMsg::release_localization() {
  // @@protoc_insertion_point(field_release:apollo.relative_map.MapMsg.localization)
  clear_has_localization();
  ::apollo::localization::LocalizationEstimate* temp = localization_;
  localization_ = NULL;
  return temp;
}
inline ::apollo::localization::LocalizationEstimate* MapMsg::mutable_localization() {
  set_has_localization();
  if (localization_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::localization::LocalizationEstimate>(GetArenaNoVirtual());
    localization_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.relative_map.MapMsg.localization)
  return localization_;
}
inline void MapMsg::set_allocated_localization(::apollo::localization::LocalizationEstimate* localization) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(localization_);
  }
  if (localization) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      localization = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, localization, submessage_arena);
    }
    set_has_localization();
  } else {
    clear_has_localization();
  }
  localization_ = localization;
  // @@protoc_insertion_point(field_set_allocated:apollo.relative_map.MapMsg.localization)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace relative_map
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_modules_2fmap_2frelative_5fmap_2fproto_2fnavigation_2eproto
