# Python client

Use `. setup_env.sh` to prepare python environment.

## gRPC client

Use `python -m grpc_tools.protoc -Iproto_gen=../../proto --python_out=. --pyi_out=. --grpc_python_out=. ../../proto/sensors.proto`