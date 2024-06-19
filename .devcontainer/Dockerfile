FROM --platform=linux/arm64 ubuntu:noble

RUN apt-get update && apt-get install -y git build-essential cmake clangd clang-format rpm

WORKDIR /deps

RUN git clone --recurse-submodules -b v1.64.0 --depth 1 --shallow-submodules https://github.com/grpc/grpc

RUN mkdir /deps/grpc/build && cd /deps/grpc/build && cmake -DgRPC_INSTALL=ON \
      -DgRPC_BUILD_TESTS=OFF \
      .. && make -j8 install