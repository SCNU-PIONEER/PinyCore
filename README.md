# need
cmake >= 3.24  
ninja >= 1.1  
arm-none-eabi-toolchains >= 10.3.1  
kconfig-frontends  

# build
```
cmake -B build -G Ninja
ninja -C build menuconfig
ninja -C build
```
