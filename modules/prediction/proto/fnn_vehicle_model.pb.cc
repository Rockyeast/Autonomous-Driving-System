// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/prediction/proto/fnn_vehicle_model.proto

#include "modules/prediction/proto/fnn_vehicle_model.pb.h"

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

namespace protobuf_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_Vector;
extern PROTOBUF_INTERNAL_EXPORT_protobuf_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto ::google::protobuf::internal::SCCInfo<2> scc_info_Layer;
}  // namespace protobuf_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto
namespace apollo {
namespace prediction {
class FnnVehicleModelDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<FnnVehicleModel>
      _instance;
} _FnnVehicleModel_default_instance_;
}  // namespace prediction
}  // namespace apollo
namespace protobuf_modules_2fprediction_2fproto_2ffnn_5fvehicle_5fmodel_2eproto {
static void InitDefaultsFnnVehicleModel() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::prediction::_FnnVehicleModel_default_instance_;
    new (ptr) ::apollo::prediction::FnnVehicleModel();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::prediction::FnnVehicleModel::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<2> scc_info_FnnVehicleModel =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 2, InitDefaultsFnnVehicleModel}, {
      &protobuf_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto::scc_info_Vector.base,
      &protobuf_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto::scc_info_Layer.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_FnnVehicleModel.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::prediction::FnnVehicleModel, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::prediction::FnnVehicleModel, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::prediction::FnnVehicleModel, dim_input_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::prediction::FnnVehicleModel, samples_mean_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::prediction::FnnVehicleModel, samples_std_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::prediction::FnnVehicleModel, num_layer_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::prediction::FnnVehicleModel, layer_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::prediction::FnnVehicleModel, dim_output_),
  2,
  0,
  1,
  3,
  ~0u,
  4,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 11, sizeof(::apollo::prediction::FnnVehicleModel)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::prediction::_FnnVehicleModel_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "modules/prediction/proto/fnn_vehicle_model.proto", schemas, file_default_instances, TableStruct::offsets,
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
      "\n0modules/prediction/proto/fnn_vehicle_m"
      "odel.proto\022\021apollo.prediction\032-modules/p"
      "rediction/proto/fnn_model_base.proto\"\325\001\n"
      "\017FnnVehicleModel\022\021\n\tdim_input\030\001 \001(\005\022/\n\014s"
      "amples_mean\030\002 \001(\0132\031.apollo.prediction.Ve"
      "ctor\022.\n\013samples_std\030\003 \001(\0132\031.apollo.predi"
      "ction.Vector\022\021\n\tnum_layer\030\004 \001(\005\022\'\n\005layer"
      "\030\005 \003(\0132\030.apollo.prediction.Layer\022\022\n\ndim_"
      "output\030\006 \001(\005"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 332);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "modules/prediction/proto/fnn_vehicle_model.proto", &protobuf_RegisterTypes);
  ::protobuf_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto::AddDescriptors();
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
}  // namespace protobuf_modules_2fprediction_2fproto_2ffnn_5fvehicle_5fmodel_2eproto
namespace apollo {
namespace prediction {

// ===================================================================

void FnnVehicleModel::InitAsDefaultInstance() {
  ::apollo::prediction::_FnnVehicleModel_default_instance_._instance.get_mutable()->samples_mean_ = const_cast< ::apollo::prediction::Vector*>(
      ::apollo::prediction::Vector::internal_default_instance());
  ::apollo::prediction::_FnnVehicleModel_default_instance_._instance.get_mutable()->samples_std_ = const_cast< ::apollo::prediction::Vector*>(
      ::apollo::prediction::Vector::internal_default_instance());
}
void FnnVehicleModel::clear_samples_mean() {
  if (samples_mean_ != NULL) samples_mean_->Clear();
  clear_has_samples_mean();
}
void FnnVehicleModel::clear_samples_std() {
  if (samples_std_ != NULL) samples_std_->Clear();
  clear_has_samples_std();
}
void FnnVehicleModel::clear_layer() {
  layer_.Clear();
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int FnnVehicleModel::kDimInputFieldNumber;
const int FnnVehicleModel::kSamplesMeanFieldNumber;
const int FnnVehicleModel::kSamplesStdFieldNumber;
const int FnnVehicleModel::kNumLayerFieldNumber;
const int FnnVehicleModel::kLayerFieldNumber;
const int FnnVehicleModel::kDimOutputFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

FnnVehicleModel::FnnVehicleModel()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_modules_2fprediction_2fproto_2ffnn_5fvehicle_5fmodel_2eproto::scc_info_FnnVehicleModel.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.prediction.FnnVehicleModel)
}
FnnVehicleModel::FnnVehicleModel(const FnnVehicleModel& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      layer_(from.layer_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_samples_mean()) {
    samples_mean_ = new ::apollo::prediction::Vector(*from.samples_mean_);
  } else {
    samples_mean_ = NULL;
  }
  if (from.has_samples_std()) {
    samples_std_ = new ::apollo::prediction::Vector(*from.samples_std_);
  } else {
    samples_std_ = NULL;
  }
  ::memcpy(&dim_input_, &from.dim_input_,
    static_cast<size_t>(reinterpret_cast<char*>(&dim_output_) -
    reinterpret_cast<char*>(&dim_input_)) + sizeof(dim_output_));
  // @@protoc_insertion_point(copy_constructor:apollo.prediction.FnnVehicleModel)
}

void FnnVehicleModel::SharedCtor() {
  ::memset(&samples_mean_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&dim_output_) -
      reinterpret_cast<char*>(&samples_mean_)) + sizeof(dim_output_));
}

FnnVehicleModel::~FnnVehicleModel() {
  // @@protoc_insertion_point(destructor:apollo.prediction.FnnVehicleModel)
  SharedDtor();
}

void FnnVehicleModel::SharedDtor() {
  if (this != internal_default_instance()) delete samples_mean_;
  if (this != internal_default_instance()) delete samples_std_;
}

void FnnVehicleModel::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* FnnVehicleModel::descriptor() {
  ::protobuf_modules_2fprediction_2fproto_2ffnn_5fvehicle_5fmodel_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fprediction_2fproto_2ffnn_5fvehicle_5fmodel_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const FnnVehicleModel& FnnVehicleModel::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_modules_2fprediction_2fproto_2ffnn_5fvehicle_5fmodel_2eproto::scc_info_FnnVehicleModel.base);
  return *internal_default_instance();
}


void FnnVehicleModel::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.prediction.FnnVehicleModel)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  layer_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 3u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(samples_mean_ != NULL);
      samples_mean_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(samples_std_ != NULL);
      samples_std_->Clear();
    }
  }
  if (cached_has_bits & 28u) {
    ::memset(&dim_input_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&dim_output_) -
        reinterpret_cast<char*>(&dim_input_)) + sizeof(dim_output_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool FnnVehicleModel::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.prediction.FnnVehicleModel)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional int32 dim_input = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(8u /* 8 & 0xFF */)) {
          set_has_dim_input();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &dim_input_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.prediction.Vector samples_mean = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_samples_mean()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.prediction.Vector samples_std = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(26u /* 26 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_samples_std()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional int32 num_layer = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(32u /* 32 & 0xFF */)) {
          set_has_num_layer();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &num_layer_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated .apollo.prediction.Layer layer = 5;
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(42u /* 42 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
                input, add_layer()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional int32 dim_output = 6;
      case 6: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(48u /* 48 & 0xFF */)) {
          set_has_dim_output();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &dim_output_)));
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
  // @@protoc_insertion_point(parse_success:apollo.prediction.FnnVehicleModel)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.prediction.FnnVehicleModel)
  return false;
#undef DO_
}

void FnnVehicleModel::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.prediction.FnnVehicleModel)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional int32 dim_input = 1;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(1, this->dim_input(), output);
  }

  // optional .apollo.prediction.Vector samples_mean = 2;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, this->_internal_samples_mean(), output);
  }

  // optional .apollo.prediction.Vector samples_std = 3;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3, this->_internal_samples_std(), output);
  }

  // optional int32 num_layer = 4;
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(4, this->num_layer(), output);
  }

  // repeated .apollo.prediction.Layer layer = 5;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->layer_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      5,
      this->layer(static_cast<int>(i)),
      output);
  }

  // optional int32 dim_output = 6;
  if (cached_has_bits & 0x00000010u) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(6, this->dim_output(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.prediction.FnnVehicleModel)
}

::google::protobuf::uint8* FnnVehicleModel::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.prediction.FnnVehicleModel)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional int32 dim_input = 1;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(1, this->dim_input(), target);
  }

  // optional .apollo.prediction.Vector samples_mean = 2;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        2, this->_internal_samples_mean(), deterministic, target);
  }

  // optional .apollo.prediction.Vector samples_std = 3;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        3, this->_internal_samples_std(), deterministic, target);
  }

  // optional int32 num_layer = 4;
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(4, this->num_layer(), target);
  }

  // repeated .apollo.prediction.Layer layer = 5;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->layer_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        5, this->layer(static_cast<int>(i)), deterministic, target);
  }

  // optional int32 dim_output = 6;
  if (cached_has_bits & 0x00000010u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(6, this->dim_output(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.prediction.FnnVehicleModel)
  return target;
}

size_t FnnVehicleModel::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.prediction.FnnVehicleModel)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  // repeated .apollo.prediction.Layer layer = 5;
  {
    unsigned int count = static_cast<unsigned int>(this->layer_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->layer(static_cast<int>(i)));
    }
  }

  if (_has_bits_[0 / 32] & 31u) {
    // optional .apollo.prediction.Vector samples_mean = 2;
    if (has_samples_mean()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *samples_mean_);
    }

    // optional .apollo.prediction.Vector samples_std = 3;
    if (has_samples_std()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *samples_std_);
    }

    // optional int32 dim_input = 1;
    if (has_dim_input()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->dim_input());
    }

    // optional int32 num_layer = 4;
    if (has_num_layer()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->num_layer());
    }

    // optional int32 dim_output = 6;
    if (has_dim_output()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->dim_output());
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void FnnVehicleModel::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.prediction.FnnVehicleModel)
  GOOGLE_DCHECK_NE(&from, this);
  const FnnVehicleModel* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const FnnVehicleModel>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.prediction.FnnVehicleModel)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.prediction.FnnVehicleModel)
    MergeFrom(*source);
  }
}

void FnnVehicleModel::MergeFrom(const FnnVehicleModel& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.prediction.FnnVehicleModel)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  layer_.MergeFrom(from.layer_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 31u) {
    if (cached_has_bits & 0x00000001u) {
      mutable_samples_mean()->::apollo::prediction::Vector::MergeFrom(from.samples_mean());
    }
    if (cached_has_bits & 0x00000002u) {
      mutable_samples_std()->::apollo::prediction::Vector::MergeFrom(from.samples_std());
    }
    if (cached_has_bits & 0x00000004u) {
      dim_input_ = from.dim_input_;
    }
    if (cached_has_bits & 0x00000008u) {
      num_layer_ = from.num_layer_;
    }
    if (cached_has_bits & 0x00000010u) {
      dim_output_ = from.dim_output_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void FnnVehicleModel::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.prediction.FnnVehicleModel)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void FnnVehicleModel::CopyFrom(const FnnVehicleModel& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.prediction.FnnVehicleModel)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool FnnVehicleModel::IsInitialized() const {
  return true;
}

void FnnVehicleModel::Swap(FnnVehicleModel* other) {
  if (other == this) return;
  InternalSwap(other);
}
void FnnVehicleModel::InternalSwap(FnnVehicleModel* other) {
  using std::swap;
  CastToBase(&layer_)->InternalSwap(CastToBase(&other->layer_));
  swap(samples_mean_, other->samples_mean_);
  swap(samples_std_, other->samples_std_);
  swap(dim_input_, other->dim_input_);
  swap(num_layer_, other->num_layer_);
  swap(dim_output_, other->dim_output_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata FnnVehicleModel::GetMetadata() const {
  protobuf_modules_2fprediction_2fproto_2ffnn_5fvehicle_5fmodel_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_modules_2fprediction_2fproto_2ffnn_5fvehicle_5fmodel_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace prediction
}  // namespace apollo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::prediction::FnnVehicleModel* Arena::CreateMaybeMessage< ::apollo::prediction::FnnVehicleModel >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::prediction::FnnVehicleModel >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
