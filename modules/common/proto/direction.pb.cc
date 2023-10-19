// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/common/proto/direction.proto

#include "modules/common/proto/direction.pb.h"

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

namespace apollo {
namespace common {
}  // namespace common
}  // namespace apollo
namespace protobuf_modules_2fcommon_2fproto_2fdirection_2eproto {
void InitDefaults() {
}

const ::google::protobuf::EnumDescriptor* file_level_enum_descriptors[1];
const ::google::protobuf::uint32 TableStruct::offsets[1] = {};
static const ::google::protobuf::internal::MigrationSchema* schemas = NULL;
static const ::google::protobuf::Message* const* file_default_instances = NULL;

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "modules/common/proto/direction.proto", schemas, file_default_instances, TableStruct::offsets,
      NULL, file_level_enum_descriptors, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n$modules/common/proto/direction.proto\022\r"
      "apollo.common*q\n\tDirection\022\010\n\004EAST\020\000\022\010\n\004"
      "WEST\020\001\022\t\n\005SOUTH\020\002\022\t\n\005NORTH\020\003\022\r\n\tNORTHEAS"
      "T\020\004\022\r\n\tSOUTHEAST\020\005\022\r\n\tSOUTHWEST\020\006\022\r\n\tNOR"
      "THWEST\020\007"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 168);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "modules/common/proto/direction.proto", &protobuf_RegisterTypes);
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
}  // namespace protobuf_modules_2fcommon_2fproto_2fdirection_2eproto
namespace apollo {
namespace common {
const ::google::protobuf::EnumDescriptor* Direction_descriptor() {
  protobuf_modules_2fcommon_2fproto_2fdirection_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_modules_2fcommon_2fproto_2fdirection_2eproto::file_level_enum_descriptors[0];
}
bool Direction_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
      return true;
    default:
      return false;
  }
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace common
}  // namespace apollo
namespace google {
namespace protobuf {
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
