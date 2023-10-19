// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/common/vehicle_state/proto/vehicle_state.proto

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)

namespace protobuf_modules_2flocalization_2fproto_2fpose_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2flocalization_2fproto_2fpose_2eproto ::google::protobuf::internal::SCCInfo<3> scc_info_Pose;
}  // namespace protobuf_modules_2flocalization_2fproto_2fpose_2eproto
namespace apollo {
namespace common {
class VehicleStateDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<VehicleState>
      _instance;
} _VehicleState_default_instance_;
}  // namespace common
}  // namespace apollo
namespace protobuf_modules_2fcommon_2fvehicle_5fstate_2fproto_2fvehicle_5fstate_2eproto {
static void InitDefaultsVehicleState() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::common::_VehicleState_default_instance_;
    new (ptr) ::apollo::common::VehicleState();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::common::VehicleState::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<1> scc_info_VehicleState =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 1, InitDefaultsVehicleState}, {
      &protobuf_modules_2flocalization_2fproto_2fpose_2eproto::scc_info_Pose.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_VehicleState.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::VehicleState, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::VehicleState, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::VehicleState, x_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::VehicleState, y_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::VehicleState, z_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::VehicleState, timestamp_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::VehicleState, roll_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::VehicleState, pitch_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::VehicleState, yaw_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::VehicleState, heading_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::VehicleState, kappa_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::VehicleState, linear_velocity_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::VehicleState, angular_velocity_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::VehicleState, linear_acceleration_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::VehicleState, gear_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::VehicleState, driving_mode_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::VehicleState, pose_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::VehicleState, steering_percentage_),
  1,
  2,
  3,
  4,
  5,
  6,
  7,
  8,
  9,
  10,
  11,
  12,
  13,
  14,
  0,
  15,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 21, sizeof(::apollo::common::VehicleState)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::common::_VehicleState_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "modules/common/vehicle_state/proto/vehicle_state.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n6modules/common/vehicle_state/proto/veh"
      "icle_state.proto\022\rapollo.common\032\"modules"
      "/canbus/proto/chassis.proto\032%modules/loc"
      "alization/proto/pose.proto\"\263\003\n\014VehicleSt"
      "ate\022\014\n\001x\030\001 \001(\001:\0010\022\014\n\001y\030\002 \001(\001:\0010\022\014\n\001z\030\003 \001"
      "(\001:\0010\022\024\n\ttimestamp\030\004 \001(\001:\0010\022\017\n\004roll\030\005 \001("
      "\001:\0010\022\020\n\005pitch\030\006 \001(\001:\0010\022\016\n\003yaw\030\007 \001(\001:\0010\022\022"
      "\n\007heading\030\010 \001(\001:\0010\022\020\n\005kappa\030\t \001(\001:\0010\022\032\n\017"
      "linear_velocity\030\n \001(\001:\0010\022\033\n\020angular_velo"
      "city\030\013 \001(\001:\0010\022\036\n\023linear_acceleration\030\014 \001"
      "(\001:\0010\0221\n\004gear\030\r \001(\0162#.apollo.canbus.Chas"
      "sis.GearPosition\0228\n\014driving_mode\030\016 \001(\0162\""
      ".apollo.canbus.Chassis.DrivingMode\022\'\n\004po"
      "se\030\017 \001(\0132\031.apollo.localization.Pose\022\033\n\023s"
      "teering_percentage\030\020 \001(\001"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 584);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "modules/common/vehicle_state/proto/vehicle_state.proto", &protobuf_RegisterTypes);
  ::protobuf_modules_2fcanbus_2fproto_2fchassis_2eproto::AddDescriptors();
  ::protobuf_modules_2flocalization_2fproto_2fpose_2eproto::AddDescriptors();
}

void AddDescriptors() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_modules_2fcommon_2fvehicle_5fstate_2fproto_2fvehicle_5fstate_2eproto
namespace apollo {
namespace common {

// ===================================================================

void VehicleState::InitAsDefaultInstance() {
  ::apollo::common::_VehicleState_default_instance_._instance.get_mutable()->pose_ = const_cast< ::apollo::localization::Pose*>(
      ::apollo::localization::Pose::internal_default_instance());
}
void VehicleState::clear_pose() {
  if (pose_ != NULL) pose_->Clear();
  clear_has_pose();
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int VehicleState::kXFieldNumber;
const int VehicleState::kYFieldNumber;
const int VehicleState::kZFieldNumber;
const int VehicleState::kTimestampFieldNumber;
const int VehicleState::kRollFieldNumber;
const int VehicleState::kPitchFieldNumber;
const int VehicleState::kYawFieldNumber;
const int VehicleState::kHeadingFieldNumber;
const int VehicleState::kKappaFieldNumber;
const int VehicleState::kLinearVelocityFieldNumber;
const int VehicleState::kAngularVelocityFieldNumber;
const int VehicleState::kLinearAccelerationFieldNumber;
const int VehicleState::kGearFieldNumber;
const int VehicleState::kDrivingModeFieldNumber;
const int VehicleState::kPoseFieldNumber;
const int VehicleState::kSteeringPercentageFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

VehicleState::VehicleState()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fcommon_2fvehicle_5fstate_2fproto_2fvehicle_5fstate_2eproto::scc_info_VehicleState.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.common.VehicleState)
}
VehicleState::VehicleState(const VehicleState& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_pose()) {
    pose_ = new ::apollo::localization::Pose(*from.pose_);
  } else {
    pose_ = NULL;
  }
  ::memcpy(&x_, &from.x_,
    static_cast<size_t>(reinterpret_cast<char*>(&steering_percentage_) -
    reinterpret_cast<char*>(&x_)) + sizeof(steering_percentage_));
  // @@protoc_insertion_point(copy_constructor:apollo.common.VehicleState)
}

void VehicleState::SharedCtor() {
  ::memset(&pose_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&steering_percentage_) -
      reinterpret_cast<char*>(&pose_)) + sizeof(steering_percentage_));
}

VehicleState::~VehicleState() {
  // @@protoc_insertion_point(destructor:apollo.common.VehicleState)
  SharedDtor();
}

void VehicleState::SharedDtor() {
  if (this != internal_default_instance()) delete pose_;
}

void VehicleState::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* VehicleState::descriptor() {
  ::protobuf_modules_2fcommon_2fvehicle_5fstate_2fproto_2fvehicle_5fstate_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fcommon_2fvehicle_5fstate_2fproto_2fvehicle_5fstate_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const VehicleState& VehicleState::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fcommon_2fvehicle_5fstate_2fproto_2fvehicle_5fstate_2eproto::scc_info_VehicleState.base);
  return *internal_default_instance();
}


void VehicleState::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.common.VehicleState)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(pose_ != NULL);
    pose_->Clear();
  }
  if (cached_has_bits & 254u) {
    ::memset(&x_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&yaw_) -
        reinterpret_cast<char*>(&x_)) + sizeof(yaw_));
  }
  if (cached_has_bits & 65280u) {
    ::memset(&heading_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&steering_percentage_) -
        reinterpret_cast<char*>(&heading_)) + sizeof(steering_percentage_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool VehicleState::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.common.VehicleState)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(16383u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional double x = 1 [default = 0];
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {
          set_has_x();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &x_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double y = 2 [default = 0];
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(17u /* 17 & 0xFF */)) {
          set_has_y();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &y_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double z = 3 [default = 0];
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(25u /* 25 & 0xFF */)) {
          set_has_z();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &z_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double timestamp = 4 [default = 0];
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(33u /* 33 & 0xFF */)) {
          set_has_timestamp();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &timestamp_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double roll = 5 [default = 0];
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(41u /* 41 & 0xFF */)) {
          set_has_roll();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &roll_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double pitch = 6 [default = 0];
      case 6: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(49u /* 49 & 0xFF */)) {
          set_has_pitch();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &pitch_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double yaw = 7 [default = 0];
      case 7: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(57u /* 57 & 0xFF */)) {
          set_has_yaw();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &yaw_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double heading = 8 [default = 0];
      case 8: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(65u /* 65 & 0xFF */)) {
          set_has_heading();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &heading_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double kappa = 9 [default = 0];
      case 9: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(73u /* 73 & 0xFF */)) {
          set_has_kappa();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &kappa_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double linear_velocity = 10 [default = 0];
      case 10: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(81u /* 81 & 0xFF */)) {
          set_has_linear_velocity();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &linear_velocity_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double angular_velocity = 11 [default = 0];
      case 11: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(89u /* 89 & 0xFF */)) {
          set_has_angular_velocity();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &angular_velocity_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double linear_acceleration = 12 [default = 0];
      case 12: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(97u /* 97 & 0xFF */)) {
          set_has_linear_acceleration();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &linear_acceleration_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.canbus.Chassis.GearPosition gear = 13;
      case 13: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(104u /* 104 & 0xFF */)) {
          int value;
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   int, ::google::protobuf::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          if (::apollo::canbus::Chassis_GearPosition_IsValid(value)) {
            set_gear(static_cast< ::apollo::canbus::Chassis_GearPosition >(value));
          } else {
            mutable_unknown_fields()->AddVarint(
                13, static_cast< ::google::protobuf::uint64>(value));
          }
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.canbus.Chassis.DrivingMode driving_mode = 14;
      case 14: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(112u /* 112 & 0xFF */)) {
          int value;
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   int, ::google::protobuf::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          if (::apollo::canbus::Chassis_DrivingMode_IsValid(value)) {
            set_driving_mode(static_cast< ::apollo::canbus::Chassis_DrivingMode >(value));
          } else {
            mutable_unknown_fields()->AddVarint(
                14, static_cast< ::google::protobuf::uint64>(value));
          }
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.localization.Pose pose = 15;
      case 15: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(122u /* 122 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_pose()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double steering_percentage = 16;
      case 16: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(129u /* 129 & 0xFF */)) {
          set_has_steering_percentage();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &steering_percentage_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:apollo.common.VehicleState)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.common.VehicleState)
  return false;
#undef DO_
}

void VehicleState::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.common.VehicleState)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double x = 1 [default = 0];
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->x(), output);
  }

  // optional double y = 2 [default = 0];
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->y(), output);
  }

  // optional double z = 3 [default = 0];
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->z(), output);
  }

  // optional double timestamp = 4 [default = 0];
  if (cached_has_bits & 0x00000010u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(4, this->timestamp(), output);
  }

  // optional double roll = 5 [default = 0];
  if (cached_has_bits & 0x00000020u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(5, this->roll(), output);
  }

  // optional double pitch = 6 [default = 0];
  if (cached_has_bits & 0x00000040u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(6, this->pitch(), output);
  }

  // optional double yaw = 7 [default = 0];
  if (cached_has_bits & 0x00000080u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(7, this->yaw(), output);
  }

  // optional double heading = 8 [default = 0];
  if (cached_has_bits & 0x00000100u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(8, this->heading(), output);
  }

  // optional double kappa = 9 [default = 0];
  if (cached_has_bits & 0x00000200u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(9, this->kappa(), output);
  }

  // optional double linear_velocity = 10 [default = 0];
  if (cached_has_bits & 0x00000400u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(10, this->linear_velocity(), output);
  }

  // optional double angular_velocity = 11 [default = 0];
  if (cached_has_bits & 0x00000800u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(11, this->angular_velocity(), output);
  }

  // optional double linear_acceleration = 12 [default = 0];
  if (cached_has_bits & 0x00001000u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(12, this->linear_acceleration(), output);
  }

  // optional .apollo.canbus.Chassis.GearPosition gear = 13;
  if (cached_has_bits & 0x00002000u) {
    ::google::protobuf::internal::WireFormatLite::WriteEnum(
      13, this->gear(), output);
  }

  // optional .apollo.canbus.Chassis.DrivingMode driving_mode = 14;
  if (cached_has_bits & 0x00004000u) {
    ::google::protobuf::internal::WireFormatLite::WriteEnum(
      14, this->driving_mode(), output);
  }

  // optional .apollo.localization.Pose pose = 15;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      15, this->_internal_pose(), output);
  }

  // optional double steering_percentage = 16;
  if (cached_has_bits & 0x00008000u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(16, this->steering_percentage(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.common.VehicleState)
}

::google::protobuf::uint8* VehicleState::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.common.VehicleState)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double x = 1 [default = 0];
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->x(), target);
  }

  // optional double y = 2 [default = 0];
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->y(), target);
  }

  // optional double z = 3 [default = 0];
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->z(), target);
  }

  // optional double timestamp = 4 [default = 0];
  if (cached_has_bits & 0x00000010u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(4, this->timestamp(), target);
  }

  // optional double roll = 5 [default = 0];
  if (cached_has_bits & 0x00000020u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(5, this->roll(), target);
  }

  // optional double pitch = 6 [default = 0];
  if (cached_has_bits & 0x00000040u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(6, this->pitch(), target);
  }

  // optional double yaw = 7 [default = 0];
  if (cached_has_bits & 0x00000080u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(7, this->yaw(), target);
  }

  // optional double heading = 8 [default = 0];
  if (cached_has_bits & 0x00000100u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(8, this->heading(), target);
  }

  // optional double kappa = 9 [default = 0];
  if (cached_has_bits & 0x00000200u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(9, this->kappa(), target);
  }

  // optional double linear_velocity = 10 [default = 0];
  if (cached_has_bits & 0x00000400u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(10, this->linear_velocity(), target);
  }

  // optional double angular_velocity = 11 [default = 0];
  if (cached_has_bits & 0x00000800u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(11, this->angular_velocity(), target);
  }

  // optional double linear_acceleration = 12 [default = 0];
  if (cached_has_bits & 0x00001000u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(12, this->linear_acceleration(), target);
  }

  // optional .apollo.canbus.Chassis.GearPosition gear = 13;
  if (cached_has_bits & 0x00002000u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteEnumToArray(
      13, this->gear(), target);
  }

  // optional .apollo.canbus.Chassis.DrivingMode driving_mode = 14;
  if (cached_has_bits & 0x00004000u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteEnumToArray(
      14, this->driving_mode(), target);
  }

  // optional .apollo.localization.Pose pose = 15;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        15, this->_internal_pose(), deterministic, target);
  }

  // optional double steering_percentage = 16;
  if (cached_has_bits & 0x00008000u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(16, this->steering_percentage(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.common.VehicleState)
  return target;
}

size_t VehicleState::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.common.VehicleState)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 255u) {
    // optional .apollo.localization.Pose pose = 15;
    if (has_pose()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *pose_);
    }

    // optional double x = 1 [default = 0];
    if (has_x()) {
      total_size += 1 + 8;
    }

    // optional double y = 2 [default = 0];
    if (has_y()) {
      total_size += 1 + 8;
    }

    // optional double z = 3 [default = 0];
    if (has_z()) {
      total_size += 1 + 8;
    }

    // optional double timestamp = 4 [default = 0];
    if (has_timestamp()) {
      total_size += 1 + 8;
    }

    // optional double roll = 5 [default = 0];
    if (has_roll()) {
      total_size += 1 + 8;
    }

    // optional double pitch = 6 [default = 0];
    if (has_pitch()) {
      total_size += 1 + 8;
    }

    // optional double yaw = 7 [default = 0];
    if (has_yaw()) {
      total_size += 1 + 8;
    }

  }
  if (_has_bits_[8 / 32] & 65280u) {
    // optional double heading = 8 [default = 0];
    if (has_heading()) {
      total_size += 1 + 8;
    }

    // optional double kappa = 9 [default = 0];
    if (has_kappa()) {
      total_size += 1 + 8;
    }

    // optional double linear_velocity = 10 [default = 0];
    if (has_linear_velocity()) {
      total_size += 1 + 8;
    }

    // optional double angular_velocity = 11 [default = 0];
    if (has_angular_velocity()) {
      total_size += 1 + 8;
    }

    // optional double linear_acceleration = 12 [default = 0];
    if (has_linear_acceleration()) {
      total_size += 1 + 8;
    }

    // optional .apollo.canbus.Chassis.GearPosition gear = 13;
    if (has_gear()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::EnumSize(this->gear());
    }

    // optional .apollo.canbus.Chassis.DrivingMode driving_mode = 14;
    if (has_driving_mode()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::EnumSize(this->driving_mode());
    }

    // optional double steering_percentage = 16;
    if (has_steering_percentage()) {
      total_size += 2 + 8;
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void VehicleState::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.common.VehicleState)
  GOOGLE_DCHECK_NE(&from, this);
  const VehicleState* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const VehicleState>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.common.VehicleState)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.common.VehicleState)
    MergeFrom(*source);
  }
}

void VehicleState::MergeFrom(const VehicleState& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.common.VehicleState)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 255u) {
    if (cached_has_bits & 0x00000001u) {
      mutable_pose()->::apollo::localization::Pose::MergeFrom(from.pose());
    }
    if (cached_has_bits & 0x00000002u) {
      x_ = from.x_;
    }
    if (cached_has_bits & 0x00000004u) {
      y_ = from.y_;
    }
    if (cached_has_bits & 0x00000008u) {
      z_ = from.z_;
    }
    if (cached_has_bits & 0x00000010u) {
      timestamp_ = from.timestamp_;
    }
    if (cached_has_bits & 0x00000020u) {
      roll_ = from.roll_;
    }
    if (cached_has_bits & 0x00000040u) {
      pitch_ = from.pitch_;
    }
    if (cached_has_bits & 0x00000080u) {
      yaw_ = from.yaw_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  if (cached_has_bits & 65280u) {
    if (cached_has_bits & 0x00000100u) {
      heading_ = from.heading_;
    }
    if (cached_has_bits & 0x00000200u) {
      kappa_ = from.kappa_;
    }
    if (cached_has_bits & 0x00000400u) {
      linear_velocity_ = from.linear_velocity_;
    }
    if (cached_has_bits & 0x00000800u) {
      angular_velocity_ = from.angular_velocity_;
    }
    if (cached_has_bits & 0x00001000u) {
      linear_acceleration_ = from.linear_acceleration_;
    }
    if (cached_has_bits & 0x00002000u) {
      gear_ = from.gear_;
    }
    if (cached_has_bits & 0x00004000u) {
      driving_mode_ = from.driving_mode_;
    }
    if (cached_has_bits & 0x00008000u) {
      steering_percentage_ = from.steering_percentage_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void VehicleState::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.common.VehicleState)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void VehicleState::CopyFrom(const VehicleState& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.common.VehicleState)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool VehicleState::IsInitialized() const {
  return true;
}

void VehicleState::Swap(VehicleState* other) {
  if (other == this) return;
  InternalSwap(other);
}
void VehicleState::InternalSwap(VehicleState* other) {
  using std::swap;
  swap(pose_, other->pose_);
  swap(x_, other->x_);
  swap(y_, other->y_);
  swap(z_, other->z_);
  swap(timestamp_, other->timestamp_);
  swap(roll_, other->roll_);
  swap(pitch_, other->pitch_);
  swap(yaw_, other->yaw_);
  swap(heading_, other->heading_);
  swap(kappa_, other->kappa_);
  swap(linear_velocity_, other->linear_velocity_);
  swap(angular_velocity_, other->angular_velocity_);
  swap(linear_acceleration_, other->linear_acceleration_);
  swap(gear_, other->gear_);
  swap(driving_mode_, other->driving_mode_);
  swap(steering_percentage_, other->steering_percentage_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata VehicleState::GetMetadata() const {
  protobuf_modules_2fcommon_2fvehicle_5fstate_2fproto_2fvehicle_5fstate_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fcommon_2fvehicle_5fstate_2fproto_2fvehicle_5fstate_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace common
}  // namespace apollo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::common::VehicleState* Arena::CreateMaybeMessage< ::apollo::common::VehicleState >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::common::VehicleState >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
