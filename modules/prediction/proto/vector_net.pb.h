// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/prediction/proto/vector_net.proto

#ifndef PROTOBUF_INCLUDED_modules_2fprediction_2fproto_2fvector_5fnet_2eproto
#define PROTOBUF_INCLUDED_modules_2fprediction_2fproto_2fvector_5fnet_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fprediction_2fproto_2fvector_5fnet_2eproto 

namespace protobuf_modules_2fprediction_2fproto_2fvector_5fnet_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[5];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_modules_2fprediction_2fproto_2fvector_5fnet_2eproto
namespace apollo {
namespace prediction {
class CarPosition;
class CarPositionDefaultTypeInternal;
extern CarPositionDefaultTypeInternal _CarPosition_default_instance_;
class Polyline;
class PolylineDefaultTypeInternal;
extern PolylineDefaultTypeInternal _Polyline_default_instance_;
class VNVector;
class VNVectorDefaultTypeInternal;
extern VNVectorDefaultTypeInternal _VNVector_default_instance_;
class VectorNetFeature;
class VectorNetFeatureDefaultTypeInternal;
extern VectorNetFeatureDefaultTypeInternal _VectorNetFeature_default_instance_;
class WorldCoord;
class WorldCoordDefaultTypeInternal;
extern WorldCoordDefaultTypeInternal _WorldCoord_default_instance_;
}  // namespace prediction
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::prediction::CarPosition* Arena::CreateMaybeMessage<::apollo::prediction::CarPosition>(Arena*);
template<> ::apollo::prediction::Polyline* Arena::CreateMaybeMessage<::apollo::prediction::Polyline>(Arena*);
template<> ::apollo::prediction::VNVector* Arena::CreateMaybeMessage<::apollo::prediction::VNVector>(Arena*);
template<> ::apollo::prediction::VectorNetFeature* Arena::CreateMaybeMessage<::apollo::prediction::VectorNetFeature>(Arena*);
template<> ::apollo::prediction::WorldCoord* Arena::CreateMaybeMessage<::apollo::prediction::WorldCoord>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace prediction {

// ===================================================================

class VNVector : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.prediction.VNVector) */ {
 public:
  VNVector();
  virtual ~VNVector();

  VNVector(const VNVector& from);

  inline VNVector& operator=(const VNVector& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  VNVector(VNVector&& from) noexcept
    : VNVector() {
    *this = ::std::move(from);
  }

  inline VNVector& operator=(VNVector&& from) noexcept {
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
  static const VNVector& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const VNVector* internal_default_instance() {
    return reinterpret_cast<const VNVector*>(
               &_VNVector_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(VNVector* other);
  friend void swap(VNVector& a, VNVector& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline VNVector* New() const final {
    return CreateMaybeMessage<VNVector>(NULL);
  }

  VNVector* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<VNVector>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const VNVector& from);
  void MergeFrom(const VNVector& from);
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
  void InternalSwap(VNVector* other);
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

  // repeated double element = 1;
  int element_size() const;
  void clear_element();
  static const int kElementFieldNumber = 1;
  double element(int index) const;
  void set_element(int index, double value);
  void add_element(double value);
  const ::google::protobuf::RepeatedField< double >&
      element() const;
  ::google::protobuf::RepeatedField< double >*
      mutable_element();

  // @@protoc_insertion_point(class_scope:apollo.prediction.VNVector)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedField< double > element_;
  friend struct ::protobuf_modules_2fprediction_2fproto_2fvector_5fnet_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class Polyline : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.prediction.Polyline) */ {
 public:
  Polyline();
  virtual ~Polyline();

  Polyline(const Polyline& from);

  inline Polyline& operator=(const Polyline& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Polyline(Polyline&& from) noexcept
    : Polyline() {
    *this = ::std::move(from);
  }

  inline Polyline& operator=(Polyline&& from) noexcept {
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
  static const Polyline& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Polyline* internal_default_instance() {
    return reinterpret_cast<const Polyline*>(
               &_Polyline_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(Polyline* other);
  friend void swap(Polyline& a, Polyline& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Polyline* New() const final {
    return CreateMaybeMessage<Polyline>(NULL);
  }

  Polyline* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Polyline>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Polyline& from);
  void MergeFrom(const Polyline& from);
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
  void InternalSwap(Polyline* other);
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

  // repeated .apollo.prediction.VNVector vector = 1;
  int vector_size() const;
  void clear_vector();
  static const int kVectorFieldNumber = 1;
  ::apollo::prediction::VNVector* mutable_vector(int index);
  ::google::protobuf::RepeatedPtrField< ::apollo::prediction::VNVector >*
      mutable_vector();
  const ::apollo::prediction::VNVector& vector(int index) const;
  ::apollo::prediction::VNVector* add_vector();
  const ::google::protobuf::RepeatedPtrField< ::apollo::prediction::VNVector >&
      vector() const;

  // optional double p_id_x = 2;
  bool has_p_id_x() const;
  void clear_p_id_x();
  static const int kPIdXFieldNumber = 2;
  double p_id_x() const;
  void set_p_id_x(double value);

  // optional double p_id_y = 3;
  bool has_p_id_y() const;
  void clear_p_id_y();
  static const int kPIdYFieldNumber = 3;
  double p_id_y() const;
  void set_p_id_y(double value);

  // @@protoc_insertion_point(class_scope:apollo.prediction.Polyline)
 private:
  void set_has_p_id_x();
  void clear_has_p_id_x();
  void set_has_p_id_y();
  void clear_has_p_id_y();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::apollo::prediction::VNVector > vector_;
  double p_id_x_;
  double p_id_y_;
  friend struct ::protobuf_modules_2fprediction_2fproto_2fvector_5fnet_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class CarPosition : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.prediction.CarPosition) */ {
 public:
  CarPosition();
  virtual ~CarPosition();

  CarPosition(const CarPosition& from);

  inline CarPosition& operator=(const CarPosition& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  CarPosition(CarPosition&& from) noexcept
    : CarPosition() {
    *this = ::std::move(from);
  }

  inline CarPosition& operator=(CarPosition&& from) noexcept {
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
  static const CarPosition& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const CarPosition* internal_default_instance() {
    return reinterpret_cast<const CarPosition*>(
               &_CarPosition_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    2;

  void Swap(CarPosition* other);
  friend void swap(CarPosition& a, CarPosition& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline CarPosition* New() const final {
    return CreateMaybeMessage<CarPosition>(NULL);
  }

  CarPosition* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<CarPosition>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const CarPosition& from);
  void MergeFrom(const CarPosition& from);
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
  void InternalSwap(CarPosition* other);
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

  // optional string id = 4;
  bool has_id() const;
  void clear_id();
  static const int kIdFieldNumber = 4;
  const ::std::string& id() const;
  void set_id(const ::std::string& value);
  #if LANG_CXX11
  void set_id(::std::string&& value);
  #endif
  void set_id(const char* value);
  void set_id(const char* value, size_t size);
  ::std::string* mutable_id();
  ::std::string* release_id();
  void set_allocated_id(::std::string* id);

  // optional double x = 1;
  bool has_x() const;
  void clear_x();
  static const int kXFieldNumber = 1;
  double x() const;
  void set_x(double value);

  // optional double y = 2;
  bool has_y() const;
  void clear_y();
  static const int kYFieldNumber = 2;
  double y() const;
  void set_y(double value);

  // optional double phi = 3;
  bool has_phi() const;
  void clear_phi();
  static const int kPhiFieldNumber = 3;
  double phi() const;
  void set_phi(double value);

  // @@protoc_insertion_point(class_scope:apollo.prediction.CarPosition)
 private:
  void set_has_x();
  void clear_has_x();
  void set_has_y();
  void clear_has_y();
  void set_has_phi();
  void clear_has_phi();
  void set_has_id();
  void clear_has_id();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr id_;
  double x_;
  double y_;
  double phi_;
  friend struct ::protobuf_modules_2fprediction_2fproto_2fvector_5fnet_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class VectorNetFeature : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.prediction.VectorNetFeature) */ {
 public:
  VectorNetFeature();
  virtual ~VectorNetFeature();

  VectorNetFeature(const VectorNetFeature& from);

  inline VectorNetFeature& operator=(const VectorNetFeature& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  VectorNetFeature(VectorNetFeature&& from) noexcept
    : VectorNetFeature() {
    *this = ::std::move(from);
  }

  inline VectorNetFeature& operator=(VectorNetFeature&& from) noexcept {
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
  static const VectorNetFeature& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const VectorNetFeature* internal_default_instance() {
    return reinterpret_cast<const VectorNetFeature*>(
               &_VectorNetFeature_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    3;

  void Swap(VectorNetFeature* other);
  friend void swap(VectorNetFeature& a, VectorNetFeature& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline VectorNetFeature* New() const final {
    return CreateMaybeMessage<VectorNetFeature>(NULL);
  }

  VectorNetFeature* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<VectorNetFeature>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const VectorNetFeature& from);
  void MergeFrom(const VectorNetFeature& from);
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
  void InternalSwap(VectorNetFeature* other);
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

  // repeated .apollo.prediction.Polyline polyline = 2;
  int polyline_size() const;
  void clear_polyline();
  static const int kPolylineFieldNumber = 2;
  ::apollo::prediction::Polyline* mutable_polyline(int index);
  ::google::protobuf::RepeatedPtrField< ::apollo::prediction::Polyline >*
      mutable_polyline();
  const ::apollo::prediction::Polyline& polyline(int index) const;
  ::apollo::prediction::Polyline* add_polyline();
  const ::google::protobuf::RepeatedPtrField< ::apollo::prediction::Polyline >&
      polyline() const;

  // optional .apollo.prediction.CarPosition car_position = 1;
  bool has_car_position() const;
  void clear_car_position();
  static const int kCarPositionFieldNumber = 1;
  private:
  const ::apollo::prediction::CarPosition& _internal_car_position() const;
  public:
  const ::apollo::prediction::CarPosition& car_position() const;
  ::apollo::prediction::CarPosition* release_car_position();
  ::apollo::prediction::CarPosition* mutable_car_position();
  void set_allocated_car_position(::apollo::prediction::CarPosition* car_position);

  // @@protoc_insertion_point(class_scope:apollo.prediction.VectorNetFeature)
 private:
  void set_has_car_position();
  void clear_has_car_position();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::apollo::prediction::Polyline > polyline_;
  ::apollo::prediction::CarPosition* car_position_;
  friend struct ::protobuf_modules_2fprediction_2fproto_2fvector_5fnet_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class WorldCoord : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.prediction.WorldCoord) */ {
 public:
  WorldCoord();
  virtual ~WorldCoord();

  WorldCoord(const WorldCoord& from);

  inline WorldCoord& operator=(const WorldCoord& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  WorldCoord(WorldCoord&& from) noexcept
    : WorldCoord() {
    *this = ::std::move(from);
  }

  inline WorldCoord& operator=(WorldCoord&& from) noexcept {
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
  static const WorldCoord& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const WorldCoord* internal_default_instance() {
    return reinterpret_cast<const WorldCoord*>(
               &_WorldCoord_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    4;

  void Swap(WorldCoord* other);
  friend void swap(WorldCoord& a, WorldCoord& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline WorldCoord* New() const final {
    return CreateMaybeMessage<WorldCoord>(NULL);
  }

  WorldCoord* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<WorldCoord>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const WorldCoord& from);
  void MergeFrom(const WorldCoord& from);
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
  void InternalSwap(WorldCoord* other);
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

  // repeated .apollo.prediction.CarPosition pose = 1;
  int pose_size() const;
  void clear_pose();
  static const int kPoseFieldNumber = 1;
  ::apollo::prediction::CarPosition* mutable_pose(int index);
  ::google::protobuf::RepeatedPtrField< ::apollo::prediction::CarPosition >*
      mutable_pose();
  const ::apollo::prediction::CarPosition& pose(int index) const;
  ::apollo::prediction::CarPosition* add_pose();
  const ::google::protobuf::RepeatedPtrField< ::apollo::prediction::CarPosition >&
      pose() const;

  // @@protoc_insertion_point(class_scope:apollo.prediction.WorldCoord)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::apollo::prediction::CarPosition > pose_;
  friend struct ::protobuf_modules_2fprediction_2fproto_2fvector_5fnet_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// VNVector

// repeated double element = 1;
inline int VNVector::element_size() const {
  return element_.size();
}
inline void VNVector::clear_element() {
  element_.Clear();
}
inline double VNVector::element(int index) const {
  // @@protoc_insertion_point(field_get:apollo.prediction.VNVector.element)
  return element_.Get(index);
}
inline void VNVector::set_element(int index, double value) {
  element_.Set(index, value);
  // @@protoc_insertion_point(field_set:apollo.prediction.VNVector.element)
}
inline void VNVector::add_element(double value) {
  element_.Add(value);
  // @@protoc_insertion_point(field_add:apollo.prediction.VNVector.element)
}
inline const ::google::protobuf::RepeatedField< double >&
VNVector::element() const {
  // @@protoc_insertion_point(field_list:apollo.prediction.VNVector.element)
  return element_;
}
inline ::google::protobuf::RepeatedField< double >*
VNVector::mutable_element() {
  // @@protoc_insertion_point(field_mutable_list:apollo.prediction.VNVector.element)
  return &element_;
}

// -------------------------------------------------------------------

// Polyline

// repeated .apollo.prediction.VNVector vector = 1;
inline int Polyline::vector_size() const {
  return vector_.size();
}
inline void Polyline::clear_vector() {
  vector_.Clear();
}
inline ::apollo::prediction::VNVector* Polyline::mutable_vector(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.prediction.Polyline.vector)
  return vector_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::prediction::VNVector >*
Polyline::mutable_vector() {
  // @@protoc_insertion_point(field_mutable_list:apollo.prediction.Polyline.vector)
  return &vector_;
}
inline const ::apollo::prediction::VNVector& Polyline::vector(int index) const {
  // @@protoc_insertion_point(field_get:apollo.prediction.Polyline.vector)
  return vector_.Get(index);
}
inline ::apollo::prediction::VNVector* Polyline::add_vector() {
  // @@protoc_insertion_point(field_add:apollo.prediction.Polyline.vector)
  return vector_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::prediction::VNVector >&
Polyline::vector() const {
  // @@protoc_insertion_point(field_list:apollo.prediction.Polyline.vector)
  return vector_;
}

// optional double p_id_x = 2;
inline bool Polyline::has_p_id_x() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Polyline::set_has_p_id_x() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Polyline::clear_has_p_id_x() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Polyline::clear_p_id_x() {
  p_id_x_ = 0;
  clear_has_p_id_x();
}
inline double Polyline::p_id_x() const {
  // @@protoc_insertion_point(field_get:apollo.prediction.Polyline.p_id_x)
  return p_id_x_;
}
inline void Polyline::set_p_id_x(double value) {
  set_has_p_id_x();
  p_id_x_ = value;
  // @@protoc_insertion_point(field_set:apollo.prediction.Polyline.p_id_x)
}

// optional double p_id_y = 3;
inline bool Polyline::has_p_id_y() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Polyline::set_has_p_id_y() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Polyline::clear_has_p_id_y() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Polyline::clear_p_id_y() {
  p_id_y_ = 0;
  clear_has_p_id_y();
}
inline double Polyline::p_id_y() const {
  // @@protoc_insertion_point(field_get:apollo.prediction.Polyline.p_id_y)
  return p_id_y_;
}
inline void Polyline::set_p_id_y(double value) {
  set_has_p_id_y();
  p_id_y_ = value;
  // @@protoc_insertion_point(field_set:apollo.prediction.Polyline.p_id_y)
}

// -------------------------------------------------------------------

// CarPosition

// optional double x = 1;
inline bool CarPosition::has_x() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void CarPosition::set_has_x() {
  _has_bits_[0] |= 0x00000002u;
}
inline void CarPosition::clear_has_x() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void CarPosition::clear_x() {
  x_ = 0;
  clear_has_x();
}
inline double CarPosition::x() const {
  // @@protoc_insertion_point(field_get:apollo.prediction.CarPosition.x)
  return x_;
}
inline void CarPosition::set_x(double value) {
  set_has_x();
  x_ = value;
  // @@protoc_insertion_point(field_set:apollo.prediction.CarPosition.x)
}

// optional double y = 2;
inline bool CarPosition::has_y() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void CarPosition::set_has_y() {
  _has_bits_[0] |= 0x00000004u;
}
inline void CarPosition::clear_has_y() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void CarPosition::clear_y() {
  y_ = 0;
  clear_has_y();
}
inline double CarPosition::y() const {
  // @@protoc_insertion_point(field_get:apollo.prediction.CarPosition.y)
  return y_;
}
inline void CarPosition::set_y(double value) {
  set_has_y();
  y_ = value;
  // @@protoc_insertion_point(field_set:apollo.prediction.CarPosition.y)
}

// optional double phi = 3;
inline bool CarPosition::has_phi() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void CarPosition::set_has_phi() {
  _has_bits_[0] |= 0x00000008u;
}
inline void CarPosition::clear_has_phi() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void CarPosition::clear_phi() {
  phi_ = 0;
  clear_has_phi();
}
inline double CarPosition::phi() const {
  // @@protoc_insertion_point(field_get:apollo.prediction.CarPosition.phi)
  return phi_;
}
inline void CarPosition::set_phi(double value) {
  set_has_phi();
  phi_ = value;
  // @@protoc_insertion_point(field_set:apollo.prediction.CarPosition.phi)
}

// optional string id = 4;
inline bool CarPosition::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void CarPosition::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void CarPosition::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void CarPosition::clear_id() {
  id_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_id();
}
inline const ::std::string& CarPosition::id() const {
  // @@protoc_insertion_point(field_get:apollo.prediction.CarPosition.id)
  return id_.GetNoArena();
}
inline void CarPosition::set_id(const ::std::string& value) {
  set_has_id();
  id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:apollo.prediction.CarPosition.id)
}
#if LANG_CXX11
inline void CarPosition::set_id(::std::string&& value) {
  set_has_id();
  id_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.prediction.CarPosition.id)
}
#endif
inline void CarPosition::set_id(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_id();
  id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.prediction.CarPosition.id)
}
inline void CarPosition::set_id(const char* value, size_t size) {
  set_has_id();
  id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.prediction.CarPosition.id)
}
inline ::std::string* CarPosition::mutable_id() {
  set_has_id();
  // @@protoc_insertion_point(field_mutable:apollo.prediction.CarPosition.id)
  return id_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* CarPosition::release_id() {
  // @@protoc_insertion_point(field_release:apollo.prediction.CarPosition.id)
  if (!has_id()) {
    return NULL;
  }
  clear_has_id();
  return id_.ReleaseNonDefaultNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void CarPosition::set_allocated_id(::std::string* id) {
  if (id != NULL) {
    set_has_id();
  } else {
    clear_has_id();
  }
  id_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), id);
  // @@protoc_insertion_point(field_set_allocated:apollo.prediction.CarPosition.id)
}

// -------------------------------------------------------------------

// VectorNetFeature

// optional .apollo.prediction.CarPosition car_position = 1;
inline bool VectorNetFeature::has_car_position() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void VectorNetFeature::set_has_car_position() {
  _has_bits_[0] |= 0x00000001u;
}
inline void VectorNetFeature::clear_has_car_position() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void VectorNetFeature::clear_car_position() {
  if (car_position_ != NULL) car_position_->Clear();
  clear_has_car_position();
}
inline const ::apollo::prediction::CarPosition& VectorNetFeature::_internal_car_position() const {
  return *car_position_;
}
inline const ::apollo::prediction::CarPosition& VectorNetFeature::car_position() const {
  const ::apollo::prediction::CarPosition* p = car_position_;
  // @@protoc_insertion_point(field_get:apollo.prediction.VectorNetFeature.car_position)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::prediction::CarPosition*>(
      &::apollo::prediction::_CarPosition_default_instance_);
}
inline ::apollo::prediction::CarPosition* VectorNetFeature::release_car_position() {
  // @@protoc_insertion_point(field_release:apollo.prediction.VectorNetFeature.car_position)
  clear_has_car_position();
  ::apollo::prediction::CarPosition* temp = car_position_;
  car_position_ = NULL;
  return temp;
}
inline ::apollo::prediction::CarPosition* VectorNetFeature::mutable_car_position() {
  set_has_car_position();
  if (car_position_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::prediction::CarPosition>(GetArenaNoVirtual());
    car_position_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.prediction.VectorNetFeature.car_position)
  return car_position_;
}
inline void VectorNetFeature::set_allocated_car_position(::apollo::prediction::CarPosition* car_position) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete car_position_;
  }
  if (car_position) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      car_position = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, car_position, submessage_arena);
    }
    set_has_car_position();
  } else {
    clear_has_car_position();
  }
  car_position_ = car_position;
  // @@protoc_insertion_point(field_set_allocated:apollo.prediction.VectorNetFeature.car_position)
}

// repeated .apollo.prediction.Polyline polyline = 2;
inline int VectorNetFeature::polyline_size() const {
  return polyline_.size();
}
inline void VectorNetFeature::clear_polyline() {
  polyline_.Clear();
}
inline ::apollo::prediction::Polyline* VectorNetFeature::mutable_polyline(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.prediction.VectorNetFeature.polyline)
  return polyline_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::prediction::Polyline >*
VectorNetFeature::mutable_polyline() {
  // @@protoc_insertion_point(field_mutable_list:apollo.prediction.VectorNetFeature.polyline)
  return &polyline_;
}
inline const ::apollo::prediction::Polyline& VectorNetFeature::polyline(int index) const {
  // @@protoc_insertion_point(field_get:apollo.prediction.VectorNetFeature.polyline)
  return polyline_.Get(index);
}
inline ::apollo::prediction::Polyline* VectorNetFeature::add_polyline() {
  // @@protoc_insertion_point(field_add:apollo.prediction.VectorNetFeature.polyline)
  return polyline_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::prediction::Polyline >&
VectorNetFeature::polyline() const {
  // @@protoc_insertion_point(field_list:apollo.prediction.VectorNetFeature.polyline)
  return polyline_;
}

// -------------------------------------------------------------------

// WorldCoord

// repeated .apollo.prediction.CarPosition pose = 1;
inline int WorldCoord::pose_size() const {
  return pose_.size();
}
inline void WorldCoord::clear_pose() {
  pose_.Clear();
}
inline ::apollo::prediction::CarPosition* WorldCoord::mutable_pose(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.prediction.WorldCoord.pose)
  return pose_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::apollo::prediction::CarPosition >*
WorldCoord::mutable_pose() {
  // @@protoc_insertion_point(field_mutable_list:apollo.prediction.WorldCoord.pose)
  return &pose_;
}
inline const ::apollo::prediction::CarPosition& WorldCoord::pose(int index) const {
  // @@protoc_insertion_point(field_get:apollo.prediction.WorldCoord.pose)
  return pose_.Get(index);
}
inline ::apollo::prediction::CarPosition* WorldCoord::add_pose() {
  // @@protoc_insertion_point(field_add:apollo.prediction.WorldCoord.pose)
  return pose_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::apollo::prediction::CarPosition >&
WorldCoord::pose() const {
  // @@protoc_insertion_point(field_list:apollo.prediction.WorldCoord.pose)
  return pose_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------

// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace prediction
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_modules_2fprediction_2fproto_2fvector_5fnet_2eproto
