# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: sensor_streaming.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='sensor_streaming.proto',
  package='sensorstreaming',
  syntax='proto3',
  serialized_options=b'\n io.grpc.examples.sensorstreamingB\017SensorStreamingP\001\242\002\003HLW',
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x16sensor_streaming.proto\x12\x0fsensorstreaming\"M\n\x16\x43\x61meraStreamingRequest\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x0c\x12\x12\n\ndataLength\x18\x02 \x01(\x05\x12\x11\n\ttimeStamp\x18\x03 \x01(\r\"*\n\x17\x43\x61meraStreamingResponse\x12\x0f\n\x07success\x18\x01 \x01(\x08\"*\n\x07Vector3\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\x12\t\n\x01z\x18\x03 \x01(\x02\"g\n\nLidarField\x12*\n\x08position\x18\x01 \x01(\x0b\x32\x18.sensorstreaming.Vector3\x12\x11\n\tintensity\x18\x02 \x01(\x02\x12\x0c\n\x04ring\x18\x03 \x01(\r\x12\x0c\n\x04time\x18\x04 \x01(\x02\"\xcb\x01\n\x15LidarStreamingRequest\x12\x15\n\rtimeInSeconds\x18\x01 \x01(\x01\x12\x0e\n\x06height\x18\x02 \x01(\r\x12\r\n\x05width\x18\x03 \x01(\r\x12\x13\n\x0bisBigEndian\x18\x04 \x01(\x08\x12\x12\n\npoint_step\x18\x05 \x01(\r\x12\x10\n\x08row_step\x18\x06 \x01(\r\x12\x30\n\x0bLidarFields\x18\x07 \x03(\x0b\x32\x1b.sensorstreaming.LidarField\x12\x0f\n\x07isDense\x18\x08 \x01(\x08\")\n\x16LidarStreamingResponse\x12\x0f\n\x07success\x18\x01 \x01(\x08\x32\xe4\x01\n\x0fSensorStreaming\x12i\n\x12StreamCameraSensor\x12\'.sensorstreaming.CameraStreamingRequest\x1a(.sensorstreaming.CameraStreamingResponse\"\x00\x12\x66\n\x11StreamLidarSensor\x12&.sensorstreaming.LidarStreamingRequest\x1a\'.sensorstreaming.LidarStreamingResponse\"\x00\x42;\n io.grpc.examples.sensorstreamingB\x0fSensorStreamingP\x01\xa2\x02\x03HLWb\x06proto3'
)




_CAMERASTREAMINGREQUEST = _descriptor.Descriptor(
  name='CameraStreamingRequest',
  full_name='sensorstreaming.CameraStreamingRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='sensorstreaming.CameraStreamingRequest.data', index=0,
      number=1, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=b"",
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='dataLength', full_name='sensorstreaming.CameraStreamingRequest.dataLength', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='timeStamp', full_name='sensorstreaming.CameraStreamingRequest.timeStamp', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=43,
  serialized_end=120,
)


_CAMERASTREAMINGRESPONSE = _descriptor.Descriptor(
  name='CameraStreamingResponse',
  full_name='sensorstreaming.CameraStreamingResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='success', full_name='sensorstreaming.CameraStreamingResponse.success', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=122,
  serialized_end=164,
)


_VECTOR3 = _descriptor.Descriptor(
  name='Vector3',
  full_name='sensorstreaming.Vector3',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='sensorstreaming.Vector3.x', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='y', full_name='sensorstreaming.Vector3.y', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='z', full_name='sensorstreaming.Vector3.z', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=166,
  serialized_end=208,
)


_LIDARFIELD = _descriptor.Descriptor(
  name='LidarField',
  full_name='sensorstreaming.LidarField',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='position', full_name='sensorstreaming.LidarField.position', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='intensity', full_name='sensorstreaming.LidarField.intensity', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='ring', full_name='sensorstreaming.LidarField.ring', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='time', full_name='sensorstreaming.LidarField.time', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=210,
  serialized_end=313,
)


_LIDARSTREAMINGREQUEST = _descriptor.Descriptor(
  name='LidarStreamingRequest',
  full_name='sensorstreaming.LidarStreamingRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='timeInSeconds', full_name='sensorstreaming.LidarStreamingRequest.timeInSeconds', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='height', full_name='sensorstreaming.LidarStreamingRequest.height', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='width', full_name='sensorstreaming.LidarStreamingRequest.width', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='isBigEndian', full_name='sensorstreaming.LidarStreamingRequest.isBigEndian', index=3,
      number=4, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='point_step', full_name='sensorstreaming.LidarStreamingRequest.point_step', index=4,
      number=5, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='row_step', full_name='sensorstreaming.LidarStreamingRequest.row_step', index=5,
      number=6, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='LidarFields', full_name='sensorstreaming.LidarStreamingRequest.LidarFields', index=6,
      number=7, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='isDense', full_name='sensorstreaming.LidarStreamingRequest.isDense', index=7,
      number=8, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=316,
  serialized_end=519,
)


_LIDARSTREAMINGRESPONSE = _descriptor.Descriptor(
  name='LidarStreamingResponse',
  full_name='sensorstreaming.LidarStreamingResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='success', full_name='sensorstreaming.LidarStreamingResponse.success', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=521,
  serialized_end=562,
)

_LIDARFIELD.fields_by_name['position'].message_type = _VECTOR3
_LIDARSTREAMINGREQUEST.fields_by_name['LidarFields'].message_type = _LIDARFIELD
DESCRIPTOR.message_types_by_name['CameraStreamingRequest'] = _CAMERASTREAMINGREQUEST
DESCRIPTOR.message_types_by_name['CameraStreamingResponse'] = _CAMERASTREAMINGRESPONSE
DESCRIPTOR.message_types_by_name['Vector3'] = _VECTOR3
DESCRIPTOR.message_types_by_name['LidarField'] = _LIDARFIELD
DESCRIPTOR.message_types_by_name['LidarStreamingRequest'] = _LIDARSTREAMINGREQUEST
DESCRIPTOR.message_types_by_name['LidarStreamingResponse'] = _LIDARSTREAMINGRESPONSE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

CameraStreamingRequest = _reflection.GeneratedProtocolMessageType('CameraStreamingRequest', (_message.Message,), {
  'DESCRIPTOR' : _CAMERASTREAMINGREQUEST,
  '__module__' : 'sensor_streaming_pb2'
  # @@protoc_insertion_point(class_scope:sensorstreaming.CameraStreamingRequest)
  })
_sym_db.RegisterMessage(CameraStreamingRequest)

CameraStreamingResponse = _reflection.GeneratedProtocolMessageType('CameraStreamingResponse', (_message.Message,), {
  'DESCRIPTOR' : _CAMERASTREAMINGRESPONSE,
  '__module__' : 'sensor_streaming_pb2'
  # @@protoc_insertion_point(class_scope:sensorstreaming.CameraStreamingResponse)
  })
_sym_db.RegisterMessage(CameraStreamingResponse)

Vector3 = _reflection.GeneratedProtocolMessageType('Vector3', (_message.Message,), {
  'DESCRIPTOR' : _VECTOR3,
  '__module__' : 'sensor_streaming_pb2'
  # @@protoc_insertion_point(class_scope:sensorstreaming.Vector3)
  })
_sym_db.RegisterMessage(Vector3)

LidarField = _reflection.GeneratedProtocolMessageType('LidarField', (_message.Message,), {
  'DESCRIPTOR' : _LIDARFIELD,
  '__module__' : 'sensor_streaming_pb2'
  # @@protoc_insertion_point(class_scope:sensorstreaming.LidarField)
  })
_sym_db.RegisterMessage(LidarField)

LidarStreamingRequest = _reflection.GeneratedProtocolMessageType('LidarStreamingRequest', (_message.Message,), {
  'DESCRIPTOR' : _LIDARSTREAMINGREQUEST,
  '__module__' : 'sensor_streaming_pb2'
  # @@protoc_insertion_point(class_scope:sensorstreaming.LidarStreamingRequest)
  })
_sym_db.RegisterMessage(LidarStreamingRequest)

LidarStreamingResponse = _reflection.GeneratedProtocolMessageType('LidarStreamingResponse', (_message.Message,), {
  'DESCRIPTOR' : _LIDARSTREAMINGRESPONSE,
  '__module__' : 'sensor_streaming_pb2'
  # @@protoc_insertion_point(class_scope:sensorstreaming.LidarStreamingResponse)
  })
_sym_db.RegisterMessage(LidarStreamingResponse)


DESCRIPTOR._options = None

_SENSORSTREAMING = _descriptor.ServiceDescriptor(
  name='SensorStreaming',
  full_name='sensorstreaming.SensorStreaming',
  file=DESCRIPTOR,
  index=0,
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_start=565,
  serialized_end=793,
  methods=[
  _descriptor.MethodDescriptor(
    name='StreamCameraSensor',
    full_name='sensorstreaming.SensorStreaming.StreamCameraSensor',
    index=0,
    containing_service=None,
    input_type=_CAMERASTREAMINGREQUEST,
    output_type=_CAMERASTREAMINGRESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='StreamLidarSensor',
    full_name='sensorstreaming.SensorStreaming.StreamLidarSensor',
    index=1,
    containing_service=None,
    input_type=_LIDARSTREAMINGREQUEST,
    output_type=_LIDARSTREAMINGRESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
])
_sym_db.RegisterServiceDescriptor(_SENSORSTREAMING)

DESCRIPTOR.services_by_name['SensorStreaming'] = _SENSORSTREAMING

# @@protoc_insertion_point(module_scope)