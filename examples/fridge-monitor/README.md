Fridge Monitor
==============
What's going on inside of your refrigerator? What's happening in there?

This is a program that repeatedly prints lines of JSON to the USB CDC (e.g.
`/dev/ttyACM0`). Each line is an object containing the current temperature and
relative humidity as read by one of three DHT22 sensors that are connected to
the Pico in GPIO pins 15, 16, and 22.

Aside from [fridge-monitor.cpp][1] and the [picoro library][2], the example depends
on the Pico's C SDK, which is included as a git submodule.  To initialize the
submodule, run [init-submodules][3].  You can also set the path to another
installation of the C SDK by modifying the `PICO_SDK_PATH` variable in
[CMakeLists.txt][4].

Here's how to build the example in a `build/` directory at the root of this
repository, copy it to the Pico, and then monitor the running program using GNU
screen:

```console
$ pwd
/home/david/src/picoro

$ mkdir build

$ cd build

$ cmake ../examples/fridge-monitor
[...]

$ make -j
[...]

$ ls -l
total 1472
-rw-rw-r-- 1 david david  20827 Aug  3 00:19 CMakeCache.txt
drwxrwxr-x 7 david david   4096 Aug  3 00:19 CMakeFiles
-rw-rw-r-- 1 david david   1866 Aug  3 00:19 cmake_install.cmake
drwxrwxr-x 6 david david   4096 Aug  3 00:19 elf2uf2
-rwxrwxr-x 1 david david  39532 Aug  3 00:19 fridge-monitor.bin
-rw-rw-r-- 1 david david 722772 Aug  3 00:19 fridge-monitor.dis
-rwxrwxr-x 1 david david 124216 Aug  3 00:19 fridge-monitor.elf
-rw-rw-r-- 1 david david 246712 Aug  3 00:19 fridge-monitor.elf.map
-rw-rw-r-- 1 david david 111238 Aug  3 00:19 fridge-monitor.hex
-rw-rw-r-- 1 david david  79360 Aug  3 00:19 fridge-monitor.uf2
drwxrwxr-x 3 david david   4096 Aug  3 00:19 generated
-rw-rw-r-- 1 david david 112294 Aug  3 00:19 Makefile
drwxrwxr-x 3 david david   4096 Aug  3 00:19 picoro
drwxrwxr-x 6 david david   4096 Aug  3 00:19 pico-sdk
drwxrwxr-x 3 david david   4096 Aug  3 00:19 pioasm

$ cp fridge-monitor.uf2 /media/"$USER"/RPI-RP2

$ ls /dev/ttyACM*
/dev/ttyACM0

$ screen /dev/ttyACM0
{"sensor": "bottom shelf", "celsius": 27.3, "humidity_percent": 54.7}
{"sensor": "top shelf", "celsius": 27.2, "humidity_percent": 54.7}
{"sensor": "middle shelf", "celsius": 27.6, "humidity_percent": 54.8}
{"sensor": "bottom shelf", "celsius": 27.3, "humidity_percent": 54.8}
{"sensor": "top shelf", "celsius": 27.2, "humidity_percent": 54.7}
{"sensor": "middle shelf", "celsius": 27.6, "humidity_percent": 54.8}
{"sensor": "bottom shelf", "celsius": 27.3, "humidity_percent": 54.7}
{"sensor": "top shelf", "celsius": 27.2, "humidity_percent": 54.7}
{"sensor": "middle shelf", "celsius": 27.6, "humidity_percent": 54.7}
{"sensor": "bottom shelf", "celsius": 27.3, "humidity_percent": 54.7}
{"sensor": "top shelf", "celsius": 27.2, "humidity_percent": 54.7}
{"sensor": "middle shelf", "celsius": 27.7, "humidity_percent": 54.7}
^ad

[detached from 120450.pts-1.carbon]
```

[1]: fridge-monitor.cpp
[2]: ../../
[3]: ../../bin/init-submodules
[4]: CMakeLists.txt
