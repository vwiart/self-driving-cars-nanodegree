@echo off
docker build -f Dockerfile.builder -t vwiart/sdcnd-t3p1-builder .
docker run --rm -it ^
    --name sdcnd-t3p1-builder ^
    -v %cd%/src:/app/src:ro ^
    -v %cd%/CMakeLists.txt:/app/CMakeLists.txt:ro ^
    -v %cd%/build:/app/build ^
    vwiart/sdcnd-t3p1-builder /bin/bash