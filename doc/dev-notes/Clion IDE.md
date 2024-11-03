https://www.jetbrains.com/help/clion/esp-idf.html#create-sample

When Cmake shows `Not Found`, check the error log

```
Error loading environment file '/Users/.../dev/esp/esp-idf-v5.1/export.sh': 'WARNING: Error while accessing the ESP-IDF version file in the Python environment: [Errno 2] No such file or directory: '/Users/fab/.espressif/python_env/idf5.1_py3.12_env/idf_version.txt'
WARNING: Error while accessing the ESP-IDF version file in the Python environment: [Errno 2] No such file or directory: '/Users/fab/.espressif/python_env/idf5.1_py3.12_env/idf_version.txt'
ERROR: /Users/fab/.espressif/python_env/idf5.1_py3.12_env/bin/python doesn't exist! Please run the install script or "idf_tools.py install-python-env" in order to create it
'
```


Open a new shell (base environment):
```
conda install python=3.12
cd esp-idf
./install.sh esp32s3
```


Tools / CMake / Reset Cache and Reload


# Remote Debug

* openocd acts as gdbserver


First try to debug from console: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/jtag-debugging/index.html#jtag-debugging-run-openocd
```
cd data
openocd -f board/esp32s3-builtin.cfg -c "program_esp ./build/main.bin 0x10000 verify exit compress"
openocd -f board/esp32s3-builtin.cfg  &
xtensa-esp32-elf-gdb -x gdbinit ../build/main.elf
 ```


Clion Settings / Embedded Dev / OpenOCD Location:
~/.espressif/tools/openocd-esp32/v0.12.0-esp32-20230921/openocd-esp32/bin/openocd

Clion Settings / Embedded Dev / RTOS : FreeRTOS

Create new Run Config
* Remote Debug
  * choose xtensa-gdb version
  * on apple M3: `dyld[]: missing symbol called` https://stackoverflow.com/questions/78855046/esp32-gdb-debugging-error-on-m1-mac-dyld-missing-symbol-called
  * could not get this to work on apple

* Install plugin https://github.com/ThexXTURBOXx/clion-embedded-esp32
  * on apple M3: `dyld[]: missing symbol called`

* (Bundled Clion) OpenOCD Download & Run
  * board/esp32s3-builtin.cfg for USB JTAG


* https://esp32.com/viewtopic.php?t=14013
* https://developer.apple.com/library/archive/documentation/DeveloperTools/gdb/gdb/gdb_18.html
* https://github.com/bazelbuild/intellij/issues/523
```
brew install gdbserver # doenst work
```