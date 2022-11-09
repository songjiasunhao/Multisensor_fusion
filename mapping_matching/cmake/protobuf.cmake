SET(Protobuf_INCLUDE_DIRS /usr/protobuf/include/) #版本问题protoc需要自己指定
SET(Protobuf_LIBRARIES /usr/protobuf/lib/libprotobuf.so)

include_directories(${Protobuf_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES ${Protobuf_LIBRARIES})