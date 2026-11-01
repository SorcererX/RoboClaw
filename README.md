# RoboClaw
A Linux C++ driver for RoboClaw motor controllers, using packet serial.

## Building packages
```
cmake -S . -B build/Release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/Release --target package
sudo apt install ./build/Release/libroboclaw-dev_*.deb
```
The package installs to `/usr`. If you installed an earlier version with `ninja install`, remove `/usr/local/lib/libRoboClaw.a` and `/usr/local/include/roboclaw.h`, otherwise the compiler keeps using those.

## API documentation
With Doxygen installed, build the `docs` target. The result is in `docs/html` in the build directory.
