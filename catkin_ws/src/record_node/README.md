# Notes

Need to recompile for python3

```console
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
```


```
rosrun image_transport republish compressed in:=/output/image_raw/compressed_image raw out:=/output/image_raw/image

```