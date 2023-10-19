// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/drivers/gnss/proto/imu.proto

#ifndef PROTOBUF_INCLUDED_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto
#define PROTOBUF_INCLUDED_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto

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
#include "modules/common/proto/geometry.pb.h"
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto 

namespace protobuf_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto {
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
}  // namespace protobuf_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto
namespace apollo {
namespace drivers {
namespace gnss {
class Imu;
class ImuDefaultTypeInternal;
extern ImuDefaultTypeInternal _Imu_default_instance_;
}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::drivers::gnss::Imu* Arena::CreateMaybeMessage<::apollo::drivers::gnss::Imu>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace drivers {
namespace gnss {

// ===================================================================

class Imu : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.drivers.gnss.Imu) */ {
 public:
  Imu();
  virtual ~Imu();

  Imu(const Imu& from);

  inline Imu& operator=(const Imu& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Imu(Imu&& from) noexcept
    : Imu() {
    *this = ::std::move(from);
  }

  inline Imu& operator=(Imu&& from) noexcept {
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
  static const Imu& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Imu* internal_default_instance() {
    return reinterpret_cast<const Imu*>(
               &_Imu_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(Imu* other);
  friend void swap(Imu& a, Imu& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Imu* New() const final {
    return CreateMaybeMessage<Imu>(NULL);
  }

  Imu* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Imu>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Imu& from);
  void MergeFrom(const Imu& from);
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
  void InternalSwap(Imu* other);
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

  // optional .apollo.common.Point3D linear_acceleration = 4;
  bool has_linear_acceleration() const;
  void clear_linear_acceleration();
  static const int kLinearAccelerationFieldNumber = 4;
  private:
  const ::apollo::common::Point3D& _internal_linear_acceleration() const;
  public:
  const ::apollo::common::Point3D& linear_acceleration() const;
  ::apollo::common::Point3D* release_linear_acceleration();
  ::apollo::common::Point3D* mutable_linear_acceleration();
  void set_allocated_linear_acceleration(::apollo::common::Point3D* linear_acceleration);

  // optional .apollo.common.Point3D angular_velocity = 5;
  bool has_angular_velocity() const;
  void clear_angular_velocity();
  static const int kAngularVelocityFieldNumber = 5;
  private:
  const ::apollo::common::Point3D& _internal_angular_velocity() const;
  public:
  const ::apollo::common::Point3D& angular_velocity() const;
  ::apollo::common::Point3D* release_angular_velocity();
  ::apollo::common::Point3D* mutable_angular_velocity();
  void set_allocated_angular_velocity(::apollo::common::Point3D* angular_velocity);

  // optional double measurement_time = 2;
  bool has_measurement_time() const;
  void clear_measurement_time();
  static const int kMeasurementTimeFieldNumber = 2;
  double measurement_time() const;
  void set_measurement_time(double value);

  // optional float measurement_span = 3 [default = 0];
  bool has_measurement_span() const;
  void clear_measurement_span();
  static const int kMeasurementSpanFieldNumber = 3;
  float measurement_span() const;
  void set_measurement_span(float value);

  // @@protoc_insertion_point(class_scope:apollo.drivers.gnss.Imu)
 private:
  void set_has_header();
  void clear_has_header();
  void set_has_measurement_time();
  void clear_has_measurement_time();
  void set_has_measurement_span();
  void clear_has_measurement_span();
  void set_has_linear_acceleration();
  void clear_has_linear_acceleration();
  void set_has_angular_velocity();
  void clear_has_angular_velocity();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::apollo::common::Header* header_;
  ::apollo::common::Point3D* linear_acceleration_;
  ::apollo::common::Point3D* angular_velocity_;
  double measurement_time_;
  float measurement_span_;
  friend struct ::protobuf_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Imu

// optional .apollo.common.Header header = 1;
inline bool Imu::has_header() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Imu::set_has_header() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Imu::clear_has_header() {
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::apollo::common::Header& Imu::_internal_header() const {
  return *header_;
}
inline const ::apollo::common::Header& Imu::header() const {
  const ::apollo::common::Header* p = header_;
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Imu.header)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::common::Header*>(
      &::apollo::common::_Header_default_instance_);
}
inline ::apollo::common::Header* Imu::release_header() {
  // @@protoc_insertion_point(field_release:apollo.drivers.gnss.Imu.header)
  clear_has_header();
  ::apollo::common::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::apollo::common::Header* Imu::mutable_header() {
  set_has_header();
  if (header_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::common::Header>(GetArenaNoVirtual());
    header_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.drivers.gnss.Imu.header)
  return header_;
}
inline void Imu::set_allocated_header(::apollo::common::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:apollo.drivers.gnss.Imu.header)
}

// optional double measurement_time = 2;
inline bool Imu::has_measurement_time() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Imu::set_has_measurement_time() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Imu::clear_has_measurement_time() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Imu::clear_measurement_time() {
  measurement_time_ = 0;
  clear_has_measurement_time();
}
inline double Imu::measurement_time() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Imu.measurement_time)
  return measurement_time_;
}
inline void Imu::set_measurement_time(double value) {
  set_has_measurement_time();
  measurement_time_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Imu.measurement_time)
}

// optional float measurement_span = 3 [default = 0];
inline bool Imu::has_measurement_span() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Imu::set_has_measurement_span() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Imu::clear_has_measurement_span() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Imu::clear_measurement_span() {
  measurement_span_ = 0;
  clear_has_measurement_span();
}
inline float Imu::measurement_span() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Imu.measurement_span)
  return measurement_span_;
}
inline void Imu::set_measurement_span(float value) {
  set_has_measurement_span();
  measurement_span_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Imu.measurement_span)
}

// optional .apollo.common.Point3D linear_acceleration = 4;
inline bool Imu::has_linear_acceleration() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Imu::set_has_linear_acceleration() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Imu::clear_has_linear_acceleration() {
  _has_bits_[0] &= ~0x00000002u;
}
inline const ::apollo::common::Point3D& Imu::_internal_linear_acceleration() const {
  return *linear_acceleration_;
}
inline const ::apollo::common::Point3D& Imu::linear_acceleration() const {
  const ::apollo::common::Point3D* p = linear_acceleration_;
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Imu.linear_acceleration)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::common::Point3D*>(
      &::apollo::common::_Point3D_default_instance_);
}
inline ::apollo::common::Point3D* Imu::release_linear_acceleration() {
  // @@protoc_insertion_point(field_release:apollo.drivers.gnss.Imu.linear_acceleration)
  clear_has_linear_acceleration();
  ::apollo::common::Point3D* temp = linear_acceleration_;
  linear_acceleration_ = NULL;
  return temp;
}
inline ::apollo::common::Point3D* Imu::mutable_linear_acceleration() {
  set_has_linear_acceleration();
  if (linear_acceleration_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::common::Point3D>(GetArenaNoVirtual());
    linear_acceleration_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.drivers.gnss.Imu.linear_acceleration)
  return linear_acceleration_;
}
inline void Imu::set_allocated_linear_acceleration(::apollo::common::Point3D* linear_acceleration) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(linear_acceleration_);
  }
  if (linear_acceleration) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      linear_acceleration = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, linear_acceleration, submessage_arena);
    }
    set_has_linear_acceleration();
  } else {
    clear_has_linear_acceleration();
  }
  linear_acceleration_ = linear_acceleration;
  // @@protoc_insertion_point(field_set_allocated:apollo.drivers.gnss.Imu.linear_acceleration)
}

// optional .apollo.common.Point3D angular_velocity = 5;
inline bool Imu::has_angular_velocity() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Imu::set_has_angular_velocity() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Imu::clear_has_angular_velocity() {
  _has_bits_[0] &= ~0x00000004u;
}
inline const ::apollo::common::Point3D& Imu::_internal_angular_velocity() const {
  return *angular_velocity_;
}
inline const ::apollo::common::Point3D& Imu::angular_velocity() const {
  const ::apollo::common::Point3D* p = angular_velocity_;
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Imu.angular_velocity)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::common::Point3D*>(
      &::apollo::common::_Point3D_default_instance_);
}
inline ::apollo::common::Point3D* Imu::release_angular_velocity() {
  // @@protoc_insertion_point(field_release:apollo.drivers.gnss.Imu.angular_velocity)
  clear_has_angular_velocity();
  ::apollo::common::Point3D* temp = angular_velocity_;
  angular_velocity_ = NULL;
  return temp;
}
inline ::apollo::common::Point3D* Imu::mutable_angular_velocity() {
  set_has_angular_velocity();
  if (angular_velocity_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::common::Point3D>(GetArenaNoVirtual());
    angular_velocity_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.drivers.gnss.Imu.angular_velocity)
  return angular_velocity_;
}
inline void Imu::set_allocated_angular_velocity(::apollo::common::Point3D* angular_velocity) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(angular_velocity_);
  }
  if (angular_velocity) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      angular_velocity = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, angular_velocity, submessage_arena);
    }
    set_has_angular_velocity();
  } else {
    clear_has_angular_velocity();
  }
  angular_velocity_ = angular_velocity;
  // @@protoc_insertion_point(field_set_allocated:apollo.drivers.gnss.Imu.angular_velocity)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_modules_2fdrivers_2fgnss_2fproto_2fimu_2eproto
