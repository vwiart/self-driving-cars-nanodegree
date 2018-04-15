@echo off
docker build -f Dockerfile.runner -t vwiart/sdcnd-t3p1-runner .
docker run --rm -it ^
    --name sdcnd-t3p1-runner ^
    -e HIGHWAY_MAP=/app/data/highway_map.csv ^
    -p 4567:4567 ^
    -v %cd%/build:/app ^
    -v %cd%/data:/app/data ^
    vwiart/sdcnd-t3p1-runner /bin/bash