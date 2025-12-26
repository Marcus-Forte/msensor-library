# Python client

Use `uv run python -m grpc_tools.protoc -Iproto_gen=../../proto --python_out=. --pyi_out=. --grpc_python_out=. ../../proto/sensors.proto` to generate proto and grpc code if they need to be update.

Use `uv run python client.py` to call the client listener.

The client uses `viser` to render sensor data in `http://localhost:8080`.