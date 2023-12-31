// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/drivers/canbus/proto/can_card_parameter.proto

#ifndef PROTOBUF_INCLUDED_modules_2fdrivers_2fcanbus_2fproto_2fcan_5fcard_5fparameter_2eproto
#define PROTOBUF_INCLUDED_modules_2fdrivers_2fcanbus_2fproto_2fcan_5fcard_5fparameter_2eproto

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
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fdrivers_2fcanbus_2fproto_2fcan_5fcard_5fparameter_2eproto 

namespace protobuf_modules_2fdrivers_2fcanbus_2fproto_2fcan_5fcard_5fparameter_2eproto {
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
}  // namespace protobuf_modules_2fdrivers_2fcanbus_2fproto_2fcan_5fcard_5fparameter_2eproto
namespace apollo {
namespace drivers {
namespace canbus {
class CANCardParameter;
class CANCardParameterDefaultTypeInternal;
extern CANCardParameterDefaultTypeInternal _CANCardParameter_default_instance_;
}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::drivers::canbus::CANCardParameter* Arena::CreateMaybeMessage<::apollo::drivers::canbus::CANCardParameter>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace drivers {
namespace canbus {

enum CANCardParameter_CANCardBrand {
  CANCardParameter_CANCardBrand_FAKE_CAN = 0,
  CANCardParameter_CANCardBrand_ESD_CAN = 1,
  CANCardParameter_CANCardBrand_SOCKET_CAN_RAW = 2,
  CANCardParameter_CANCardBrand_HERMES_CAN = 3
};
bool CANCardParameter_CANCardBrand_IsValid(int value);
const CANCardParameter_CANCardBrand CANCardParameter_CANCardBrand_CANCardBrand_MIN = CANCardParameter_CANCardBrand_FAKE_CAN;
const CANCardParameter_CANCardBrand CANCardParameter_CANCardBrand_CANCardBrand_MAX = CANCardParameter_CANCardBrand_HERMES_CAN;
const int CANCardParameter_CANCardBrand_CANCardBrand_ARRAYSIZE = CANCardParameter_CANCardBrand_CANCardBrand_MAX + 1;

const ::google::protobuf::EnumDescriptor* CANCardParameter_CANCardBrand_descriptor();
inline const ::std::string& CANCardParameter_CANCardBrand_Name(CANCardParameter_CANCardBrand value) {
  return ::google::protobuf::internal::NameOfEnum(
    CANCardParameter_CANCardBrand_descriptor(), value);
}
inline bool CANCardParameter_CANCardBrand_Parse(
    const ::std::string& name, CANCardParameter_CANCardBrand* value) {
  return ::google::protobuf::internal::ParseNamedEnum<CANCardParameter_CANCardBrand>(
    CANCardParameter_CANCardBrand_descriptor(), name, value);
}
enum CANCardParameter_CANCardType {
  CANCardParameter_CANCardType_PCI_CARD = 0,
  CANCardParameter_CANCardType_USB_CARD = 1
};
bool CANCardParameter_CANCardType_IsValid(int value);
const CANCardParameter_CANCardType CANCardParameter_CANCardType_CANCardType_MIN = CANCardParameter_CANCardType_PCI_CARD;
const CANCardParameter_CANCardType CANCardParameter_CANCardType_CANCardType_MAX = CANCardParameter_CANCardType_USB_CARD;
const int CANCardParameter_CANCardType_CANCardType_ARRAYSIZE = CANCardParameter_CANCardType_CANCardType_MAX + 1;

const ::google::protobuf::EnumDescriptor* CANCardParameter_CANCardType_descriptor();
inline const ::std::string& CANCardParameter_CANCardType_Name(CANCardParameter_CANCardType value) {
  return ::google::protobuf::internal::NameOfEnum(
    CANCardParameter_CANCardType_descriptor(), value);
}
inline bool CANCardParameter_CANCardType_Parse(
    const ::std::string& name, CANCardParameter_CANCardType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<CANCardParameter_CANCardType>(
    CANCardParameter_CANCardType_descriptor(), name, value);
}
enum CANCardParameter_CANChannelId {
  CANCardParameter_CANChannelId_CHANNEL_ID_ZERO = 0,
  CANCardParameter_CANChannelId_CHANNEL_ID_ONE = 1,
  CANCardParameter_CANChannelId_CHANNEL_ID_TWO = 2,
  CANCardParameter_CANChannelId_CHANNEL_ID_THREE = 3,
  CANCardParameter_CANChannelId_CHANNEL_ID_FOUR = 4,
  CANCardParameter_CANChannelId_CHANNEL_ID_FIVE = 5,
  CANCardParameter_CANChannelId_CHANNEL_ID_SIX = 6,
  CANCardParameter_CANChannelId_CHANNEL_ID_SEVEN = 7
};
bool CANCardParameter_CANChannelId_IsValid(int value);
const CANCardParameter_CANChannelId CANCardParameter_CANChannelId_CANChannelId_MIN = CANCardParameter_CANChannelId_CHANNEL_ID_ZERO;
const CANCardParameter_CANChannelId CANCardParameter_CANChannelId_CANChannelId_MAX = CANCardParameter_CANChannelId_CHANNEL_ID_SEVEN;
const int CANCardParameter_CANChannelId_CANChannelId_ARRAYSIZE = CANCardParameter_CANChannelId_CANChannelId_MAX + 1;

const ::google::protobuf::EnumDescriptor* CANCardParameter_CANChannelId_descriptor();
inline const ::std::string& CANCardParameter_CANChannelId_Name(CANCardParameter_CANChannelId value) {
  return ::google::protobuf::internal::NameOfEnum(
    CANCardParameter_CANChannelId_descriptor(), value);
}
inline bool CANCardParameter_CANChannelId_Parse(
    const ::std::string& name, CANCardParameter_CANChannelId* value) {
  return ::google::protobuf::internal::ParseNamedEnum<CANCardParameter_CANChannelId>(
    CANCardParameter_CANChannelId_descriptor(), name, value);
}
enum CANCardParameter_CANInterface {
  CANCardParameter_CANInterface_NATIVE = 0,
  CANCardParameter_CANInterface_VIRTUAL = 1,
  CANCardParameter_CANInterface_SLCAN = 2
};
bool CANCardParameter_CANInterface_IsValid(int value);
const CANCardParameter_CANInterface CANCardParameter_CANInterface_CANInterface_MIN = CANCardParameter_CANInterface_NATIVE;
const CANCardParameter_CANInterface CANCardParameter_CANInterface_CANInterface_MAX = CANCardParameter_CANInterface_SLCAN;
const int CANCardParameter_CANInterface_CANInterface_ARRAYSIZE = CANCardParameter_CANInterface_CANInterface_MAX + 1;

const ::google::protobuf::EnumDescriptor* CANCardParameter_CANInterface_descriptor();
inline const ::std::string& CANCardParameter_CANInterface_Name(CANCardParameter_CANInterface value) {
  return ::google::protobuf::internal::NameOfEnum(
    CANCardParameter_CANInterface_descriptor(), value);
}
inline bool CANCardParameter_CANInterface_Parse(
    const ::std::string& name, CANCardParameter_CANInterface* value) {
  return ::google::protobuf::internal::ParseNamedEnum<CANCardParameter_CANInterface>(
    CANCardParameter_CANInterface_descriptor(), name, value);
}
// ===================================================================

class CANCardParameter : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.drivers.canbus.CANCardParameter) */ {
 public:
  CANCardParameter();
  virtual ~CANCardParameter();

  CANCardParameter(const CANCardParameter& from);

  inline CANCardParameter& operator=(const CANCardParameter& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  CANCardParameter(CANCardParameter&& from) noexcept
    : CANCardParameter() {
    *this = ::std::move(from);
  }

  inline CANCardParameter& operator=(CANCardParameter&& from) noexcept {
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
  static const CANCardParameter& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const CANCardParameter* internal_default_instance() {
    return reinterpret_cast<const CANCardParameter*>(
               &_CANCardParameter_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(CANCardParameter* other);
  friend void swap(CANCardParameter& a, CANCardParameter& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline CANCardParameter* New() const final {
    return CreateMaybeMessage<CANCardParameter>(NULL);
  }

  CANCardParameter* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<CANCardParameter>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const CANCardParameter& from);
  void MergeFrom(const CANCardParameter& from);
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
  void InternalSwap(CANCardParameter* other);
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

  typedef CANCardParameter_CANCardBrand CANCardBrand;
  static const CANCardBrand FAKE_CAN =
    CANCardParameter_CANCardBrand_FAKE_CAN;
  static const CANCardBrand ESD_CAN =
    CANCardParameter_CANCardBrand_ESD_CAN;
  static const CANCardBrand SOCKET_CAN_RAW =
    CANCardParameter_CANCardBrand_SOCKET_CAN_RAW;
  static const CANCardBrand HERMES_CAN =
    CANCardParameter_CANCardBrand_HERMES_CAN;
  static inline bool CANCardBrand_IsValid(int value) {
    return CANCardParameter_CANCardBrand_IsValid(value);
  }
  static const CANCardBrand CANCardBrand_MIN =
    CANCardParameter_CANCardBrand_CANCardBrand_MIN;
  static const CANCardBrand CANCardBrand_MAX =
    CANCardParameter_CANCardBrand_CANCardBrand_MAX;
  static const int CANCardBrand_ARRAYSIZE =
    CANCardParameter_CANCardBrand_CANCardBrand_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  CANCardBrand_descriptor() {
    return CANCardParameter_CANCardBrand_descriptor();
  }
  static inline const ::std::string& CANCardBrand_Name(CANCardBrand value) {
    return CANCardParameter_CANCardBrand_Name(value);
  }
  static inline bool CANCardBrand_Parse(const ::std::string& name,
      CANCardBrand* value) {
    return CANCardParameter_CANCardBrand_Parse(name, value);
  }

  typedef CANCardParameter_CANCardType CANCardType;
  static const CANCardType PCI_CARD =
    CANCardParameter_CANCardType_PCI_CARD;
  static const CANCardType USB_CARD =
    CANCardParameter_CANCardType_USB_CARD;
  static inline bool CANCardType_IsValid(int value) {
    return CANCardParameter_CANCardType_IsValid(value);
  }
  static const CANCardType CANCardType_MIN =
    CANCardParameter_CANCardType_CANCardType_MIN;
  static const CANCardType CANCardType_MAX =
    CANCardParameter_CANCardType_CANCardType_MAX;
  static const int CANCardType_ARRAYSIZE =
    CANCardParameter_CANCardType_CANCardType_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  CANCardType_descriptor() {
    return CANCardParameter_CANCardType_descriptor();
  }
  static inline const ::std::string& CANCardType_Name(CANCardType value) {
    return CANCardParameter_CANCardType_Name(value);
  }
  static inline bool CANCardType_Parse(const ::std::string& name,
      CANCardType* value) {
    return CANCardParameter_CANCardType_Parse(name, value);
  }

  typedef CANCardParameter_CANChannelId CANChannelId;
  static const CANChannelId CHANNEL_ID_ZERO =
    CANCardParameter_CANChannelId_CHANNEL_ID_ZERO;
  static const CANChannelId CHANNEL_ID_ONE =
    CANCardParameter_CANChannelId_CHANNEL_ID_ONE;
  static const CANChannelId CHANNEL_ID_TWO =
    CANCardParameter_CANChannelId_CHANNEL_ID_TWO;
  static const CANChannelId CHANNEL_ID_THREE =
    CANCardParameter_CANChannelId_CHANNEL_ID_THREE;
  static const CANChannelId CHANNEL_ID_FOUR =
    CANCardParameter_CANChannelId_CHANNEL_ID_FOUR;
  static const CANChannelId CHANNEL_ID_FIVE =
    CANCardParameter_CANChannelId_CHANNEL_ID_FIVE;
  static const CANChannelId CHANNEL_ID_SIX =
    CANCardParameter_CANChannelId_CHANNEL_ID_SIX;
  static const CANChannelId CHANNEL_ID_SEVEN =
    CANCardParameter_CANChannelId_CHANNEL_ID_SEVEN;
  static inline bool CANChannelId_IsValid(int value) {
    return CANCardParameter_CANChannelId_IsValid(value);
  }
  static const CANChannelId CANChannelId_MIN =
    CANCardParameter_CANChannelId_CANChannelId_MIN;
  static const CANChannelId CANChannelId_MAX =
    CANCardParameter_CANChannelId_CANChannelId_MAX;
  static const int CANChannelId_ARRAYSIZE =
    CANCardParameter_CANChannelId_CANChannelId_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  CANChannelId_descriptor() {
    return CANCardParameter_CANChannelId_descriptor();
  }
  static inline const ::std::string& CANChannelId_Name(CANChannelId value) {
    return CANCardParameter_CANChannelId_Name(value);
  }
  static inline bool CANChannelId_Parse(const ::std::string& name,
      CANChannelId* value) {
    return CANCardParameter_CANChannelId_Parse(name, value);
  }

  typedef CANCardParameter_CANInterface CANInterface;
  static const CANInterface NATIVE =
    CANCardParameter_CANInterface_NATIVE;
  static const CANInterface VIRTUAL =
    CANCardParameter_CANInterface_VIRTUAL;
  static const CANInterface SLCAN =
    CANCardParameter_CANInterface_SLCAN;
  static inline bool CANInterface_IsValid(int value) {
    return CANCardParameter_CANInterface_IsValid(value);
  }
  static const CANInterface CANInterface_MIN =
    CANCardParameter_CANInterface_CANInterface_MIN;
  static const CANInterface CANInterface_MAX =
    CANCardParameter_CANInterface_CANInterface_MAX;
  static const int CANInterface_ARRAYSIZE =
    CANCardParameter_CANInterface_CANInterface_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  CANInterface_descriptor() {
    return CANCardParameter_CANInterface_descriptor();
  }
  static inline const ::std::string& CANInterface_Name(CANInterface value) {
    return CANCardParameter_CANInterface_Name(value);
  }
  static inline bool CANInterface_Parse(const ::std::string& name,
      CANInterface* value) {
    return CANCardParameter_CANInterface_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // optional .apollo.drivers.canbus.CANCardParameter.CANCardBrand brand = 1;
  bool has_brand() const;
  void clear_brand();
  static const int kBrandFieldNumber = 1;
  ::apollo::drivers::canbus::CANCardParameter_CANCardBrand brand() const;
  void set_brand(::apollo::drivers::canbus::CANCardParameter_CANCardBrand value);

  // optional .apollo.drivers.canbus.CANCardParameter.CANCardType type = 2;
  bool has_type() const;
  void clear_type();
  static const int kTypeFieldNumber = 2;
  ::apollo::drivers::canbus::CANCardParameter_CANCardType type() const;
  void set_type(::apollo::drivers::canbus::CANCardParameter_CANCardType value);

  // optional .apollo.drivers.canbus.CANCardParameter.CANChannelId channel_id = 3;
  bool has_channel_id() const;
  void clear_channel_id();
  static const int kChannelIdFieldNumber = 3;
  ::apollo::drivers::canbus::CANCardParameter_CANChannelId channel_id() const;
  void set_channel_id(::apollo::drivers::canbus::CANCardParameter_CANChannelId value);

  // optional .apollo.drivers.canbus.CANCardParameter.CANInterface interface = 4;
  bool has_interface() const;
  void clear_interface();
  static const int kInterfaceFieldNumber = 4;
  ::apollo::drivers::canbus::CANCardParameter_CANInterface interface() const;
  void set_interface(::apollo::drivers::canbus::CANCardParameter_CANInterface value);

  // optional uint32 num_ports = 5 [default = 4];
  bool has_num_ports() const;
  void clear_num_ports();
  static const int kNumPortsFieldNumber = 5;
  ::google::protobuf::uint32 num_ports() const;
  void set_num_ports(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:apollo.drivers.canbus.CANCardParameter)
 private:
  void set_has_brand();
  void clear_has_brand();
  void set_has_type();
  void clear_has_type();
  void set_has_channel_id();
  void clear_has_channel_id();
  void set_has_interface();
  void clear_has_interface();
  void set_has_num_ports();
  void clear_has_num_ports();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  int brand_;
  int type_;
  int channel_id_;
  int interface_;
  ::google::protobuf::uint32 num_ports_;
  friend struct ::protobuf_modules_2fdrivers_2fcanbus_2fproto_2fcan_5fcard_5fparameter_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// CANCardParameter

// optional .apollo.drivers.canbus.CANCardParameter.CANCardBrand brand = 1;
inline bool CANCardParameter::has_brand() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void CANCardParameter::set_has_brand() {
  _has_bits_[0] |= 0x00000001u;
}
inline void CANCardParameter::clear_has_brand() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void CANCardParameter::clear_brand() {
  brand_ = 0;
  clear_has_brand();
}
inline ::apollo::drivers::canbus::CANCardParameter_CANCardBrand CANCardParameter::brand() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.canbus.CANCardParameter.brand)
  return static_cast< ::apollo::drivers::canbus::CANCardParameter_CANCardBrand >(brand_);
}
inline void CANCardParameter::set_brand(::apollo::drivers::canbus::CANCardParameter_CANCardBrand value) {
  assert(::apollo::drivers::canbus::CANCardParameter_CANCardBrand_IsValid(value));
  set_has_brand();
  brand_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.canbus.CANCardParameter.brand)
}

// optional .apollo.drivers.canbus.CANCardParameter.CANCardType type = 2;
inline bool CANCardParameter::has_type() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void CANCardParameter::set_has_type() {
  _has_bits_[0] |= 0x00000002u;
}
inline void CANCardParameter::clear_has_type() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void CANCardParameter::clear_type() {
  type_ = 0;
  clear_has_type();
}
inline ::apollo::drivers::canbus::CANCardParameter_CANCardType CANCardParameter::type() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.canbus.CANCardParameter.type)
  return static_cast< ::apollo::drivers::canbus::CANCardParameter_CANCardType >(type_);
}
inline void CANCardParameter::set_type(::apollo::drivers::canbus::CANCardParameter_CANCardType value) {
  assert(::apollo::drivers::canbus::CANCardParameter_CANCardType_IsValid(value));
  set_has_type();
  type_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.canbus.CANCardParameter.type)
}

// optional .apollo.drivers.canbus.CANCardParameter.CANChannelId channel_id = 3;
inline bool CANCardParameter::has_channel_id() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void CANCardParameter::set_has_channel_id() {
  _has_bits_[0] |= 0x00000004u;
}
inline void CANCardParameter::clear_has_channel_id() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void CANCardParameter::clear_channel_id() {
  channel_id_ = 0;
  clear_has_channel_id();
}
inline ::apollo::drivers::canbus::CANCardParameter_CANChannelId CANCardParameter::channel_id() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.canbus.CANCardParameter.channel_id)
  return static_cast< ::apollo::drivers::canbus::CANCardParameter_CANChannelId >(channel_id_);
}
inline void CANCardParameter::set_channel_id(::apollo::drivers::canbus::CANCardParameter_CANChannelId value) {
  assert(::apollo::drivers::canbus::CANCardParameter_CANChannelId_IsValid(value));
  set_has_channel_id();
  channel_id_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.canbus.CANCardParameter.channel_id)
}

// optional .apollo.drivers.canbus.CANCardParameter.CANInterface interface = 4;
inline bool CANCardParameter::has_interface() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void CANCardParameter::set_has_interface() {
  _has_bits_[0] |= 0x00000008u;
}
inline void CANCardParameter::clear_has_interface() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void CANCardParameter::clear_interface() {
  interface_ = 0;
  clear_has_interface();
}
inline ::apollo::drivers::canbus::CANCardParameter_CANInterface CANCardParameter::interface() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.canbus.CANCardParameter.interface)
  return static_cast< ::apollo::drivers::canbus::CANCardParameter_CANInterface >(interface_);
}
inline void CANCardParameter::set_interface(::apollo::drivers::canbus::CANCardParameter_CANInterface value) {
  assert(::apollo::drivers::canbus::CANCardParameter_CANInterface_IsValid(value));
  set_has_interface();
  interface_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.canbus.CANCardParameter.interface)
}

// optional uint32 num_ports = 5 [default = 4];
inline bool CANCardParameter::has_num_ports() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void CANCardParameter::set_has_num_ports() {
  _has_bits_[0] |= 0x00000010u;
}
inline void CANCardParameter::clear_has_num_ports() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void CANCardParameter::clear_num_ports() {
  num_ports_ = 4u;
  clear_has_num_ports();
}
inline ::google::protobuf::uint32 CANCardParameter::num_ports() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.canbus.CANCardParameter.num_ports)
  return num_ports_;
}
inline void CANCardParameter::set_num_ports(::google::protobuf::uint32 value) {
  set_has_num_ports();
  num_ports_ = value;
  // @@protoc_insertion_point(field_set:apollo.drivers.canbus.CANCardParameter.num_ports)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace canbus
}  // namespace drivers
}  // namespace apollo

namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::apollo::drivers::canbus::CANCardParameter_CANCardBrand> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::drivers::canbus::CANCardParameter_CANCardBrand>() {
  return ::apollo::drivers::canbus::CANCardParameter_CANCardBrand_descriptor();
}
template <> struct is_proto_enum< ::apollo::drivers::canbus::CANCardParameter_CANCardType> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::drivers::canbus::CANCardParameter_CANCardType>() {
  return ::apollo::drivers::canbus::CANCardParameter_CANCardType_descriptor();
}
template <> struct is_proto_enum< ::apollo::drivers::canbus::CANCardParameter_CANChannelId> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::drivers::canbus::CANCardParameter_CANChannelId>() {
  return ::apollo::drivers::canbus::CANCardParameter_CANChannelId_descriptor();
}
template <> struct is_proto_enum< ::apollo::drivers::canbus::CANCardParameter_CANInterface> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::drivers::canbus::CANCardParameter_CANInterface>() {
  return ::apollo::drivers::canbus::CANCardParameter_CANInterface_descriptor();
}

}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_modules_2fdrivers_2fcanbus_2fproto_2fcan_5fcard_5fparameter_2eproto
