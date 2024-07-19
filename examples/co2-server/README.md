CO₂ Server
==========
This is an HTTP server that responds with JSON containing the CO₂ concentration
recently read from an SCD41 sensor.

Aside from [co2-server.cpp][1] and the [Picoro headers][2], the example depends
on the Pico's C SDK, which is included as a git submodule.  To initialize the
submodule, run [init-submodules][3].  You can also set the path to another
installation of the C SDK by modifying the `PICO_SDK_PATH` variable in
[CMakeLists.txt][4].

Here's how to build the example in a `build/` directory at the root of this repository:
```console
$ pwd
/home/david/src/picoro
$ bin/init-submodules
[...]
$ mkdir build
$ cd build
$ cmake ../examples/co2-server/
[...]
$ make -j
[...]
$ ls -l
total 6944
-rw-rw-r-- 1 david david   19927 Jul 19 15:43 CMakeCache.txt
drwxrwxr-x 7 david david    4096 Jul 19 15:44 CMakeFiles
-rw-rw-r-- 1 david david    1689 Jul 19 15:43 cmake_install.cmake
-rwxrwxr-x 1 david david  367076 Jul 19 15:44 co2-server.bin
-rw-rw-r-- 1 david david 2588874 Jul 19 15:44 co2-server.dis
-rwxrwxr-x 1 david david 1076568 Jul 19 15:44 co2-server.elf
-rw-rw-r-- 1 david david  999381 Jul 19 15:44 co2-server.elf.map
-rw-rw-r-- 1 david david 1032560 Jul 19 15:44 co2-server.hex
-rw-rw-r-- 1 david david  734208 Jul 19 15:44 co2-server.uf2
drwxrwxr-x 6 david david    4096 Jul 19 15:44 elf2uf2
drwxrwxr-x 3 david david    4096 Jul 19 15:43 generated
-rw-rw-r-- 1 david david  250217 Jul 19 15:43 Makefile
drwxrwxr-x 6 david david    4096 Jul 19 15:43 pico-sdk
drwxrwxr-x 3 david david    4096 Jul 19 15:44 pioasm
```

You can then copy `co2-server.uf2` into the Pico when it's connected in boot
select mode.

Once the server is running on the Pico, it responds to any HTTP request (any
TCP connection, really) with an HTTP response containing JSON:
```console
$ curl --verbose --no-progress-meter 'http://192.168.1.101' | jq
*   Trying 192.168.1.101:80...
* TCP_NODELAY set
* Connected to 192.168.1.101 (192.168.1.101) port 80 (#0)
> GET / HTTP/1.1
> Host: 192.168.1.101
> User-Agent: curl/7.68.0
> Accept: */*
>
* Mark bundle as not supporting multiuse
< HTTP/1.1 200 OK
< Connection: close
< Content-Type: application/json
<
{ [161 bytes data]
* Closing connection 0
{
  "sequence_number": 52738,
  "CO2_ppm": 613,
  "temperature_celsius": 27.324,
  "relative_humidity_percent": 55.055,
  "wifi_status_counts": [
    0,
    0,
    0,
    0,
    0,
    0,
    1055323
  ]
}
```

[1]: co2-server.cpp
[2]: ../../include/picoro/
[3]: ../../bin/init-submodules
[4]: CMakeLists.txt
