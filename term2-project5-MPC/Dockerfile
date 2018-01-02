FROM debian:latest
LABEL maintainer="keybase.io/vwiart"

WORKDIR /app
RUN apt-get update && \
    apt-get install -y wget curl git libuv1-dev libssl-dev gcc g++ cmake make zlib1g-dev cppad libblas-dev gfortran unzip && \
    git clone https://github.com/uWebSockets/uWebSockets && \
    cd uWebSockets && git checkout e94b6e1 && \
    mkdir build && cd build && cmake .. && make && make install && \
    ln -sf /usr/lib64/libuWS.so /usr/lib/libuWS.so && \    
    cd /app && rm -R uWebSockets
RUN curl -O https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip && unzip Ipopt-3.12.7.zip && rm Ipopt-3.12.7.zip
# Installing BLAS
RUN cd /app/Ipopt-3.12.7/ThirdParty/Blas && \
    ./get.Blas && \
    mkdir build && cd build && ../configure --prefix=/usr/local --disable-shared --with-pic && make install
# Installing Lapack
RUN cd /app/Ipopt-3.12.7/ThirdParty/Lapack && \
    ./get.Lapack && \
    mkdir build && cd build && ../configure --prefix=/usr/local --disable-shared --with-pic --with-blas="/usr/local/lib/libcoinblas.a -lgfortran" && \
    make install
# Installing ASL & MUMPS
RUN cd /app/Ipopt-3.12.7/ThirdParty/ASL && ./get.ASL
RUN cd /app/Ipopt-3.12.7/ThirdParty/Mumps && ./get.Mumps
# Building IPOPT
RUN cd /app/Ipopt-3.12.7 && mkdir build && cd build && \
    ../configure --prefix=/usr/local coin_skip_warn_cxxflags=yes \
    --with-blas="/usr/local/lib/libcoinblas.a -lgfortran" \
    --with-lapack=/usr/local/lib/libcoinlapack.a && \
    make && make test && make -j1 install
ENV LD_LIBRARY_PATH=/usr/local/lib
COPY ./CMakeLists.txt /app
COPY ./src /app/src
RUN mkdir -p /app/build && cd /app/build && cmake .. && make
CMD ["/app/build/mpc"]
