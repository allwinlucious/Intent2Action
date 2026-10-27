#!/usr/bin/env bash
# Encodes a RuntimeContext text proto (normally written by k3s) into the binary runtime_config.pb an Intrinsic service reads.
# Usage: encode_runtime.sh <in.txtpb> <out.pb> <proto file defining the config's Any type>...
set -euo pipefail

source "$(dirname "$0")/proto_includes.sh"

in="$(readlink -f "$1")"; out="$(readlink -f "$2")"; shift 2
cd "$INTRINSIC_DIR"
sed 's/ *#.*$//' "$in" | "$PY" -m grpc_tools.protoc "${INCLUDES[@]}" \
  --encode=intrinsic_proto.config.RuntimeContext intrinsic/resources/proto/runtime_context.proto "$@" > "$out"
echo "wrote $out ($(stat -c%s "$out") bytes)"
