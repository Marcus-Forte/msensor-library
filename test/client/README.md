# Python client

Use `. setup_env.sh` to prepare python environment.

## gRPC client (Environment must be activated)

Use `python -m grpc_tools.protoc -Iproto_gen=../../proto --python_out=. --pyi_out=. --grpc_python_out=. ../../proto/sensors.proto` to generate proto and grpc code if they need to be update.

Use `python client_*.py` to call the client listener.