# Sourced by the proto scripts: protoc include roots for Intrinsic's .proto files. Sets PY and INCLUDES.
INTRINSIC_DIR="${INTRINSIC_DIR:-$HOME/intrinsic-core}"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="$PROJECT_DIR/.venv/bin/python"
EXTERNAL="$(cd "$INTRINSIC_DIR" && readlink -f bazel-bin | sed 's|/execroot/.*||')/external"
WKT="$("$PY" -c "import grpc_tools,os;print(os.path.join(os.path.dirname(grpc_tools.__file__),'_proto'))")"
# Paths are relative to INTRINSIC_DIR; callers cd there before running protoc.
INCLUDES=(-I. -Iintrinsic_apis -Iintrinsic_sdk -Iintrinsic_motion_planning -Iintrinsic_kinematics -Iintrinsic_control
  -Iintrinsic_perception -Iintrinsic_hardware -I"$EXTERNAL/googleapis+" -I"$EXTERNAL/cel-spec+/proto" -I"$WKT"
  -I"$PROJECT_DIR")  # our own protos (robot_service/*.proto)
