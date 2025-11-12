# hybrid a star

## build

```
cmake -B build -DBUILD_TESTS=ON && cmake --build build -j
```

Tested on Ubuntu 24.04 X86-64, gcc-12.4.0, cmake 3.28.3

## test case

**test_a_star_save**

Run A* and save the map and path to binary file "temp/test_a_star.bin". 
Use `python3 script/visualize.py temp/test_a_star.bin` to visualize the result.

**test_a_star**

Run A* 10000 times and show the successes counts.

**test_empty_map**

Run hybrid A* on empty map and save the map and path to binary file "temp/test_empty_map.bin".
Use `python3 script/visualize.py temp/test_empty_map.bin` to visualize the result.

**test_obstacle_map**

Run hybrid A* on obstacle map and save the map and path to binary file "temp/test_obstacle_map.bin".
Use `python3 script/visualize.py temp/test_obstacle_map.bin` to visualize the result.