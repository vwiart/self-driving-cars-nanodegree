FROM debian:latest
MAINTAINER keybase.io/vwiart

WORKDIR /app
RUN apt-get update && \
    apt-get install -y curl git libuv1-dev libssl-dev gcc g++ cmake make zlib1g-dev && \
    git clone https://github.com/uWebSockets/uWebSockets && \
    cd uWebSockets && git checkout e94b6e1 && \
    mkdir build && cd build && cmake .. && make && make install && \
    ln -sf /usr/lib64/libuWS.so /usr/lib/libuWS.so && \
    cd /app && rm -R uWebSockets
COPY ./CMakeLists.txt /app
COPY ./src /app/src
RUN mkdir -p /app/build && cd /app/build && cmake .. && make
CMD ["/app/build/UnscentedKF"]
