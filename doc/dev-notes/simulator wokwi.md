
https://docs.wokwi.com/guides/esp32-wifi#the-private-gateway

https://docs.wokwi.com/vscode/project-config


there's `config/lab/wokwi_mock`
* connects to the mock WiFi
* lower frequency interrupt timer for better performance
* speed-ratio of 14-28% (quite slow)


## wokwi cli (CI)

[install](https://docs.wokwi.com/wokwi-ci/cli-installation)

--elf build/fugu.elf


## wokwi debug

in vscode:
https://docs.wokwi.com/vscode/debugging#start-the-debugger



```

xtensa-esp32-elf-addr2line -e build/fugu-firmware.elf

0x4037eee5:0x3fcbefa0 0x4037eead:0x3fcbefc0 0x403879d1:0x3fcbefe0 0x40385794:0x3fcbf100 0x4037618d:0x3fcbf120 0x40376295:0x3fcbf140 0x403762c5:0x3fcbf170 0x40375eb0:0x3fcbf190 0x403792c9:0x3fcbf1b0 0x4212421a:0x3fcbf1d0 0x420cb03f:0x3fcbf200 0x420cb265:0x3fcbf220 0x420caea3:0x3fcbf240 0x420c990f:0x3fcbf260 0x420b8ab1:0x3fcbf280 0x420b8b6f:0x3fcbf2a0 0x420b6baa:0x3fcbf2c0 0x403885a5:0x3fcbf2e0 0x4037fc31:0x3fcbf310
```