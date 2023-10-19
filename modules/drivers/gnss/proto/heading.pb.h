// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/drivers/gnss/proto/heading.proto

#ifndef PROTOBUF_INCLUDED_modules_2fdrivers_2fgnss_2fproto_2fheading_2eproto
#define PROTOBUF_INCLUDED_modules_2fdrivers_2fgnss_2fproto_2fheading_2eproto

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
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fdrivers_2fgnss_2fproto_2fheading_2eproto 

namespace protobuf_modules_2fdrivers_2fgnss_2fproto_2fheading_2eproto {
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
}  // namespace protobuf_modules_2fdrivers_2fgnss_2fproto_2fheading_2eproto
namespace apollo {
namespace drivers {
namespace gnss {
class Heading;
class HeadingDefaultTypeInternal;
extern HeadingDefaultTypeInternal _Heading_default_instance_;
}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::drivers::gnss::Heading* Arena::CreateMaybeMessage<::apollo::drivers::gnss::Heading>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace drivers {
namespace gnss {

// ===================================================================

class Heading : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.drivers.gnss.Heading) */ {
 public:
  Heading();
  virtual ~Heading();

  Heading(const Heading& from);

  inline Heading& operator=(const Heading& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Heading(Heading&& from) noexcept
    : Heading() {
    *this = ::std::move(from);
  }

  inline Heading& operator=(Heading&& from) noexcept {
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
  static const Heading& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Heading* internal_default_instance() {
    return reinterpret_cast<const Heading*>(
               &_Heading_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(Heading* other);
  friend void swap(Heading& a, Heading& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Heading* New() const final {
    return CreateMaybeMessage<Heading>(NULL);
  }

  Heading* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Heading>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Heading& from);
  void MergeFrom(const Heading& from);
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
  void InternalSwap(Heading* other);
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

  // optional bytes station_id = 11;
  bool has_station_id() const;
  void clear_station_id();
  static const int kStationIdFieldNumber = 11;
  const ::std::string& station_id() const;
  void set_station_id(const ::std::string& value);
  #if LANG_CXX11
  void set_station_id(::std::string&& value);
  #endif
  void set_station_id(const char* value);
  void set_station_id(const void* value, size_t size);
  ::std::string* mutable_station_id();
  ::std::string* release_station_id();
  void set_allocated_station_id(::std::string* station_id);

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

  // optional double measurement_time = 2;
  bool has_measurement_time() const;
  void clear_measurement_time();
  static const int kMeasurementTimeFieldNumber = 2;
  double measurement_time() const;
  void set_measurement_time(double value);

  // optional uint32 solution_status = 3;
  bool has_solution_status() const;
  void clear_solution_status();
  static const int kSolutionStatusFieldNumber = 3;
  ::google::protobuf::uint32 solution_status() const;
  void set_solution_status(::google::protobuf::uint32 value);

  // optional uint32 position_type = 4;
  bool has_position_type() const;
  void clear_position_type();
  static const int kPositionTypeFieldNumber = 4;
  ::google::protobuf::uint32 position_type() const;
  void set_position_type(::google::protobuf::uint32 value);

  // optional float baseline_length = 5;
  bool has_baseline_length() const;
  void clear_baseline_length();
  static const int kBaselineLengthFieldNumber = 5;
  float baseline_length() const;
  void set_baseline_length(float value);

  // optional float heading = 6;
  bool has_heading() const;
  void clear_heading();
  static const int kHeadingFieldNumber = 6;
  float heading() const;
  void set_heading(float value);

  // optional float pitch = 7;
  bool has_pitch() const;
  void clear_pitch();
  static const int kPitchFieldNumber = 7;
  float pitch() const;
  void set_pitch(float value);

  // optional float reserved = 8;
  bool has_reserved() const;
  void clear_reserved();
  static const int kReservedFieldNumber = 8;
  float reserved() const;
  void set_reserved(float value);

  // optional float heading_std_dev = 9;
  bool has_heading_std_dev() const;
  void clear_heading_std_dev();
  static const int kHeadingStdDevFieldNumber = 9;
  float heading_std_dev() const;
  void set_heading_std_dev(float value);

  // optional float pitch_std_dev = 10;
  bool has_pitch_std_dev() const;
  void clear_pitch_std_dev();
  static const int kPitchStdDevFieldNumber = 10;
  float pitch_std_dev() const;
  void set_pitch_std_dev(float value);

  // optional uint32 satellite_tracked_number = 12;
  bool has_satellite_tracked_number() const;
  void clear_satellite_tracked_number();
  static const int kSatelliteTrackedNumberFieldNumber = 12;
  ::google::protobuf::uint32 satellite_tracked_number() const;
  void set_satellite_tracked_number(::google::protobuf::uint32 value);

  // optional uint32 satellite_soulution_number = 13;
  bool has_satellite_soulution_number() const;
  void clear_satellite_soulution_number();
  static const int kSatelliteSoulutionNumberFieldNumber = 13;
  ::google::protobuf::uint32 satellite_soulution_number() const;
  void set_satellite_soulution_number(::google::protobuf::uint32 value);

  // optional uint32 satellite_number_obs = 14;
  bool has_satellite_number_obs() const;
  void clear_satellite_number_obs();
  static const int kSatelliteNumberObsFieldNumber = 14;
  ::google::protobuf::uint32 satellite_number_obs() const;
  void set_satellite_number_obs(::google::protobuf::uint32 value);

  // optional uint32 satellite_number_multi = 15;
  bool has_satellite_number_multi() const;
  void clear_satellite_number_multi();
  static const int kSatelliteNumberMultiFieldNumber = 15;
  ::google::protobuf::uint32 satellite_number_multi() const;
  void set_satellite_number_multi(::google::protobuf::uint32 value);

  // optional uint32 solution_source = 16;
  bool has_solution_source() const;
  void clear_solution_source();
  static const int kSolutionSourceFieldNumber = 16;
  ::google::protobuf::uint32 solution_source() const;
  void set_solution_source(::google::protobuf::uint32 value);

  // optional uint32 extended_solution_status = 17;
  bool has_extended_solution_status() const;
  void clear_extended_solution_status();
  static const int kExtendedSolutionStatusFieldNumber = 17;
  ::google::protobuf::uint32 extended_solution_status() const;
  void set_extended_solution_status(::google::protobuf::uint32 value);

  // optional uint32 galileo_beidou_sig_mask = 18;
  bool has_galileo_beidou_sig_mask() const;
  void clear_galileo_beidou_sig_mask();
  static const int kGalileoBeidouSigMaskFieldNumber = 18;
  ::google::protobuf::uint32 galileo_beidou_sig_mask() const;
  void set_galileo_beidou_sig_mask(::google::protobuf::uint32 value);

  // optional uint32 gps_glonass_sig_mask = 19;
  bool has_gps_glonass_sig_mask() const;
  void clear_gps_glonass_sig_mask();
  static const int kGpsGlonassSigMaskFieldNumber = 19;
  ::google::protobuf::uint32 gps_glonass_sig_mask() const;
  void set_gps_glonass_sig_mask(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:apollo.drivers.gnss.Heading)
 private:
  void set_has_header();
  void clear_has_header();
  void set_has_measurement_time();
  void clear_has_measurement_time();
  void set_has_solution_status();
  void clear_has_solution_status();
  void set_has_position_type();
  void clear_has_position_type();
  void set_has_baseline_length();
  void clear_has_baseline_length();
  void set_has_heading();
  void clear_has_heading();
  void set_has_pitch();
  void clear_has_pitch();
  void set_has_reserved();
  void clear_has_reserved();
  void set_has_heading_std_dev();
  void clear_has_heading_std_dev();
  void set_has_pitch_std_dev();
  void clear_has_pitch_std_dev();
  void set_has_station_id();
  void clear_has_station_id();
  void set_has_satellite_tracked_number();
  void clear_has_satellite_tracked_number();
  void set_has_satellite_soulution_number();
  void clear_has_satellite_soulution_number();
  void set_has_satellite_number_obs();
  void clear_has_satellite_number_obs();
  void set_has_satellite_number_multi();
  void clear_has_satellite_number_multi();
  void set_has_solution_source();
  void clear_has_solution_source();
  void set_has_extended_solution_status();
  void clear_has_extended_solution_status();
  void set_has_galileo_beidou_sig_mask();
  void clear_has_galileo_beidou_sig_mask();
  void set_has_gps_glonass_sig_mask();
  void clear_has_gps_glonass_sig_mask();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr station_id_;
  ::apollo::common::Header* header_;
  double measurement_time_;
  ::google::protobuf::uint32 solution_status_;
  ::google::protobuf::uint32 position_type_;
  float baseline_length_;
  float heading_;
  float pitch_;
  float reserved_;
  float heading_std_dev_;
  float pitch_std_dev_;
  ::google::protobuf::uint32 satellite_tracked_number_;
  ::google::protobuf::uint32 satellite_soulution_number_;
  ::google::protobuf::uint32 satellite_number_obs_;
  ::google::protobuf::uint32 satellite_number_multi_;
  ::google::protobuf::uint32 solution_source_;
  ::google::protobuf::uint32 extended_solution_status_;
  ::google::protobuf::uint32 galileo_beidou_sig_mask_;
  ::google::protobuf::uint32 gps_glonass_sig_mask_;
  friend struct ::protobuf_modules_2fdrivers_2fgnss_2fproto_2fheading_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Heading

// optional .apollo.common.Header header = 1;
inline bool Heading::has_header() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Heading::set_has_header() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Heading::clear_has_header() {
  _has_bits_[0] &= ~0x00000002u;
}
inline const ::apollo::common::Header& Heading::_internal_header() const {
  return *header_;
}
inline const ::apollo::common::Header& Heading::header() const {
  const ::apollo::common::Header* p = header_;
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Heading.header)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::common::Header*>(
      &::apollo::common::_Header_default_instance_);
}
inline ::apollo::common::Header* Heading::release_header() {
  // @@protoc_insertion_point(field_release:apollo.drivers.gnss.Heading.header)
  clear_has_header();
  ::apollo::common::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::apollo::common::Header* Heading::mutable_header() {
  set_has_header();
  if (header_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::common::Header>(GetArenaNoVirtual());
    header_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.drivers.gnss.Heading.header)
  return header_;
}
inline void Heading::set_allocated_header(::apollo::common::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:apollo.drivers.gnss.Heading.header)
}

// optional double measurement_time = 2;
inline bool Heading::has_measurement_time() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Heading::set_has_measurement_time() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Heading::clear_has_measurement_time() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Heading::clear_measurement_time() {
  measurement_time_ = 0;
  clear_has_measurement_time();
}
inline double Heading::measurement_time() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Heading.measurement_time)
  return measurement_time_;
}
inline void Heading::set_measurement_time(double value) {
  set_has_measurement_time();
  measurement_time_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Heading.measurement_time)
}

// optional uint32 solution_status = 3;
inline bool Heading::has_solution_status() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Heading::set_has_solution_status() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Heading::clear_has_solution_status() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Heading::clear_solution_status() {
  solution_status_ = 0u;
  clear_has_solution_status();
}
inline ::google::protobuf::uint32 Heading::solution_status() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Heading.solution_status)
  return solution_status_;
}
inline void Heading::set_solution_status(::google::protobuf::uint32 value) {
  set_has_solution_status();
  solution_status_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Heading.solution_status)
}

// optional uint32 position_type = 4;
inline bool Heading::has_position_type() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Heading::set_has_position_type() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Heading::clear_has_position_type() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Heading::clear_position_type() {
  position_type_ = 0u;
  clear_has_position_type();
}
inline ::google::protobuf::uint32 Heading::position_type() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Heading.position_type)
  return position_type_;
}
inline void Heading::set_position_type(::google::protobuf::uint32 value) {
  set_has_position_type();
  position_type_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Heading.position_type)
}

// optional float baseline_length = 5;
inline bool Heading::has_baseline_length() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void Heading::set_has_baseline_length() {
  _has_bits_[0] |= 0x00000020u;
}
inline void Heading::clear_has_baseline_length() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void Heading::clear_baseline_length() {
  baseline_length_ = 0;
  clear_has_baseline_length();
}
inline float Heading::baseline_length() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Heading.baseline_length)
  return baseline_length_;
}
inline void Heading::set_baseline_length(float value) {
  set_has_baseline_length();
  baseline_length_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Heading.baseline_length)
}

// optional float heading = 6;
inline bool Heading::has_heading() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void Heading::set_has_heading() {
  _has_bits_[0] |= 0x00000040u;
}
inline void Heading::clear_has_heading() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void Heading::clear_heading() {
  heading_ = 0;
  clear_has_heading();
}
inline float Heading::heading() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Heading.heading)
  return heading_;
}
inline void Heading::set_heading(float value) {
  set_has_heading();
  heading_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Heading.heading)
}

// optional float pitch = 7;
inline bool Heading::has_pitch() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void Heading::set_has_pitch() {
  _has_bits_[0] |= 0x00000080u;
}
inline void Heading::clear_has_pitch() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void Heading::clear_pitch() {
  pitch_ = 0;
  clear_has_pitch();
}
inline float Heading::pitch() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Heading.pitch)
  return pitch_;
}
inline void Heading::set_pitch(float value) {
  set_has_pitch();
  pitch_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Heading.pitch)
}

// optional float reserved = 8;
inline bool Heading::has_reserved() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void Heading::set_has_reserved() {
  _has_bits_[0] |= 0x00000100u;
}
inline void Heading::clear_has_reserved() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void Heading::clear_reserved() {
  reserved_ = 0;
  clear_has_reserved();
}
inline float Heading::reserved() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Heading.reserved)
  return reserved_;
}
inline void Heading::set_reserved(float value) {
  set_has_reserved();
  reserved_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Heading.reserved)
}

// optional float heading_std_dev = 9;
inline bool Heading::has_heading_std_dev() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
inline void Heading::set_has_heading_std_dev() {
  _has_bits_[0] |= 0x00000200u;
}
inline void Heading::clear_has_heading_std_dev() {
  _has_bits_[0] &= ~0x00000200u;
}
inline void Heading::clear_heading_std_dev() {
  heading_std_dev_ = 0;
  clear_has_heading_std_dev();
}
inline float Heading::heading_std_dev() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Heading.heading_std_dev)
  return heading_std_dev_;
}
inline void Heading::set_heading_std_dev(float value) {
  set_has_heading_std_dev();
  heading_std_dev_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Heading.heading_std_dev)
}

// optional float pitch_std_dev = 10;
inline bool Heading::has_pitch_std_dev() const {
  return (_has_bits_[0] & 0x00000400u) != 0;
}
inline void Heading::set_has_pitch_std_dev() {
  _has_bits_[0] |= 0x00000400u;
}
inline void Heading::clear_has_pitch_std_dev() {
  _has_bits_[0] &= ~0x00000400u;
}
inline void Heading::clear_pitch_std_dev() {
  pitch_std_dev_ = 0;
  clear_has_pitch_std_dev();
}
inline float Heading::pitch_std_dev() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Heading.pitch_std_dev)
  return pitch_std_dev_;
}
inline void Heading::set_pitch_std_dev(float value) {
  set_has_pitch_std_dev();
  pitch_std_dev_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Heading.pitch_std_dev)
}

// optional bytes station_id = 11;
inline bool Heading::has_station_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Heading::set_has_station_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Heading::clear_has_station_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Heading::clear_station_id() {
  station_id_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_station_id();
}
inline const ::std::string& Heading::station_id() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Heading.station_id)
  return station_id_.GetNoArena();
}
inline void Heading::set_station_id(const ::std::string& value) {
  set_has_station_id();
  station_id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Heading.station_id)
}
#if LANG_CXX11
inline void Heading::set_station_id(::std::string&& value) {
  set_has_station_id();
  station_id_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.drivers.gnss.Heading.station_id)
}
#endif
inline void Heading::set_station_id(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_station_id();
  station_id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.drivers.gnss.Heading.station_id)
}
inline void Heading::set_station_id(const void* value, size_t size) {
  set_has_station_id();
  station_id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.drivers.gnss.Heading.station_id)
}
inline ::std::string* Heading::mutable_station_id() {
  set_has_station_id();
  // @@protoc_insertion_point(field_mutable:apollo.drivers.gnss.Heading.station_id)
  return station_id_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Heading::release_station_id() {
  // @@protoc_insertion_point(field_release:apollo.drivers.gnss.Heading.station_id)
  if (!has_station_id()) {
    return NULL;
  }
  clear_has_station_id();
  return station_id_.ReleaseNonDefaultNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Heading::set_allocated_station_id(::std::string* station_id) {
  if (station_id != NULL) {
    set_has_station_id();
  } else {
    clear_has_station_id();
  }
  station_id_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), station_id);
  // @@protoc_insertion_point(field_set_allocated:apollo.drivers.gnss.Heading.station_id)
}

// optional uint32 satellite_tracked_number = 12;
inline bool Heading::has_satellite_tracked_number() const {
  return (_has_bits_[0] & 0x00000800u) != 0;
}
inline void Heading::set_has_satellite_tracked_number() {
  _has_bits_[0] |= 0x00000800u;
}
inline void Heading::clear_has_satellite_tracked_number() {
  _has_bits_[0] &= ~0x00000800u;
}
inline void Heading::clear_satellite_tracked_number() {
  satellite_tracked_number_ = 0u;
  clear_has_satellite_tracked_number();
}
inline ::google::protobuf::uint32 Heading::satellite_tracked_number() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Heading.satellite_tracked_number)
  return satellite_tracked_number_;
}
inline void Heading::set_satellite_tracked_number(::google::protobuf::uint32 value) {
  set_has_satellite_tracked_number();
  satellite_tracked_number_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Heading.satellite_tracked_number)
}

// optional uint32 satellite_soulution_number = 13;
inline bool Heading::has_satellite_soulution_number() const {
  return (_has_bits_[0] & 0x00001000u) != 0;
}
inline void Heading::set_has_satellite_soulution_number() {
  _has_bits_[0] |= 0x00001000u;
}
inline void Heading::clear_has_satellite_soulution_number() {
  _has_bits_[0] &= ~0x00001000u;
}
inline void Heading::clear_satellite_soulution_number() {
  satellite_soulution_number_ = 0u;
  clear_has_satellite_soulution_number();
}
inline ::google::protobuf::uint32 Heading::satellite_soulution_number() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Heading.satellite_soulution_number)
  return satellite_soulution_number_;
}
inline void Heading::set_satellite_soulution_number(::google::protobuf::uint32 value) {
  set_has_satellite_soulution_number();
  satellite_soulution_number_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Heading.satellite_soulution_number)
}

// optional uint32 satellite_number_obs = 14;
inline bool Heading::has_satellite_number_obs() const {
  return (_has_bits_[0] & 0x00002000u) != 0;
}
inline void Heading::set_has_satellite_number_obs() {
  _has_bits_[0] |= 0x00002000u;
}
inline void Heading::clear_has_satellite_number_obs() {
  _has_bits_[0] &= ~0x00002000u;
}
inline void Heading::clear_satellite_number_obs() {
  satellite_number_obs_ = 0u;
  clear_has_satellite_number_obs();
}
inline ::google::protobuf::uint32 Heading::satellite_number_obs() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Heading.satellite_number_obs)
  return satellite_number_obs_;
}
inline void Heading::set_satellite_number_obs(::google::protobuf::uint32 value) {
  set_has_satellite_number_obs();
  satellite_number_obs_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Heading.satellite_number_obs)
}

// optional uint32 satellite_number_multi = 15;
inline bool Heading::has_satellite_number_multi() const {
  return (_has_bits_[0] & 0x00004000u) != 0;
}
inline void Heading::set_has_satellite_number_multi() {
  _has_bits_[0] |= 0x00004000u;
}
inline void Heading::clear_has_satellite_number_multi() {
  _has_bits_[0] &= ~0x00004000u;
}
inline void Heading::clear_satellite_number_multi() {
  satellite_number_multi_ = 0u;
  clear_has_satellite_number_multi();
}
inline ::google::protobuf::uint32 Heading::satellite_number_multi() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Heading.satellite_number_multi)
  return satellite_number_multi_;
}
inline void Heading::set_satellite_number_multi(::google::protobuf::uint32 value) {
  set_has_satellite_number_multi();
  satellite_number_multi_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Heading.satellite_number_multi)
}

// optional uint32 solution_source = 16;
inline bool Heading::has_solution_source() const {
  return (_has_bits_[0] & 0x00008000u) != 0;
}
inline void Heading::set_has_solution_source() {
  _has_bits_[0] |= 0x00008000u;
}
inline void Heading::clear_has_solution_source() {
  _has_bits_[0] &= ~0x00008000u;
}
inline void Heading::clear_solution_source() {
  solution_source_ = 0u;
  clear_has_solution_source();
}
inline ::google::protobuf::uint32 Heading::solution_source() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Heading.solution_source)
  return solution_source_;
}
inline void Heading::set_solution_source(::google::protobuf::uint32 value) {
  set_has_solution_source();
  solution_source_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Heading.solution_source)
}

// optional uint32 extended_solution_status = 17;
inline bool Heading::has_extended_solution_status() const {
  return (_has_bits_[0] & 0x00010000u) != 0;
}
inline void Heading::set_has_extended_solution_status() {
  _has_bits_[0] |= 0x00010000u;
}
inline void Heading::clear_has_extended_solution_status() {
  _has_bits_[0] &= ~0x00010000u;
}
inline void Heading::clear_extended_solution_status() {
  extended_solution_status_ = 0u;
  clear_has_extended_solution_status();
}
inline ::google::protobuf::uint32 Heading::extended_solution_status() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Heading.extended_solution_status)
  return extended_solution_status_;
}
inline void Heading::set_extended_solution_status(::google::protobuf::uint32 value) {
  set_has_extended_solution_status();
  extended_solution_status_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Heading.extended_solution_status)
}

// optional uint32 galileo_beidou_sig_mask = 18;
inline bool Heading::has_galileo_beidou_sig_mask() const {
  return (_has_bits_[0] & 0x00020000u) != 0;
}
inline void Heading::set_has_galileo_beidou_sig_mask() {
  _has_bits_[0] |= 0x00020000u;
}
inline void Heading::clear_has_galileo_beidou_sig_mask() {
  _has_bits_[0] &= ~0x00020000u;
}
inline void Heading::clear_galileo_beidou_sig_mask() {
  galileo_beidou_sig_mask_ = 0u;
  clear_has_galileo_beidou_sig_mask();
}
inline ::google::protobuf::uint32 Heading::galileo_beidou_sig_mask() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Heading.galileo_beidou_sig_mask)
  return galileo_beidou_sig_mask_;
}
inline void Heading::set_galileo_beidou_sig_mask(::google::protobuf::uint32 value) {
  set_has_galileo_beidou_sig_mask();
  galileo_beidou_sig_mask_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Heading.galileo_beidou_sig_mask)
}

// optional uint32 gps_glonass_sig_mask = 19;
inline bool Heading::has_gps_glonass_sig_mask() const {
  return (_has_bits_[0] & 0x00040000u) != 0;
}
inline void Heading::set_has_gps_glonass_sig_mask() {
  _has_bits_[0] |= 0x00040000u;
}
inline void Heading::clear_has_gps_glonass_sig_mask() {
  _has_bits_[0] &= ~0x00040000u;
}
inline void Heading::clear_gps_glonass_sig_mask() {
  gps_glonass_sig_mask_ = 0u;
  clear_has_gps_glonass_sig_mask();
}
inline ::google::protobuf::uint32 Heading::gps_glonass_sig_mask() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.gnss.Heading.gps_glonass_sig_mask)
  return gps_glonass_sig_mask_;
}
inline void Heading::set_gps_glonass_sig_mask(::google::protobuf::uint32 value) {
  set_has_gps_glonass_sig_mask();
  gps_glonass_sig_mask_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.gnss.Heading.gps_glonass_sig_mask)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_modules_2fdrivers_2fgnss_2fproto_2fheading_2eproto
