#!/usr/bin/env bash
# Generates Python protobuf + gRPC stubs for the given Intrinsic .proto files and everything they import.
# Usage: gen_protos.sh <proto path relative to its root, e.g. intrinsic/resources/proto/resource_registry.proto>...
set -euo pipefail

source "$(dirname "$0")/proto_includes.sh"
OUT="$PROJECT_DIR/gen"

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
echo "generated ${#files[@]} files into $OUT"
