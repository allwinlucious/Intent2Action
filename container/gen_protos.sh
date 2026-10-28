#!/usr/bin/env bash
# Generates Python protobuf + gRPC stubs for the given Intrinsic .proto files and everything they import.
# Usage: gen_protos.sh <proto path relative to its root, e.g. intrinsic/resources/proto/resource_registry.proto>...
# Files of this project may also be given by a path under the project root (e.g. robot_service/simlink.proto).
set -euo pipefail

source "$(dirname "$0")/proto_includes.sh"
OUT="$PROJECT_DIR/gen"

protos=()
for proto in "$@"; do
  if [[ -f "$proto" && "$(realpath "$proto")" == "$PROJECT_DIR"/* ]]; then
    proto="$(realpath --relative-to="$PROJECT_DIR" "$proto")"
  fi
  protos+=("$proto")
done
set -- "${protos[@]}"

cd "$INTRINSIC_DIR"
mkdir -p "$OUT"
desc="$(mktemp)"
"$PY" -m grpc_tools.protoc "${INCLUDES[@]}" --include_imports --descriptor_set_out="$desc" "$@"
# Every file in the import closure except protobuf's own well-known types (those ship with the protobuf package).
mapfile -t files < <("$PY" -c "
import sys
from google.protobuf import descriptor_pb2
s = descriptor_pb2.FileDescriptorSet(); s.ParseFromString(open(sys.argv[1], 'rb').read())
print('\n'.join(f.name for f in s.file if not f.name.startswith('google/protobuf/')))" "$desc")
rm -f "$desc"
"$PY" -m grpc_tools.protoc "${INCLUDES[@]}" --python_out="$OUT" --grpc_python_out="$OUT" "${files[@]}"
# The MCP server (mcp_server/server.py) builds its tools from this descriptor set, comments included.
for proto in "$@"; do
  if [[ "$proto" == robot_service/robot_service.proto ]]; then
    "$PY" -m grpc_tools.protoc "${INCLUDES[@]}" --include_source_info --include_imports \
      --descriptor_set_out="$OUT/robot_service.desc" "$proto"
    echo "wrote $OUT/robot_service.desc"
  fi
done
echo "generated ${#files[@]} files into $OUT"
