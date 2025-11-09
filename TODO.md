* provisioning overrides
  * eg ina22x_resistor, voltage dividers etc that persists when flashing a new provisioning image
  * also wifi, coil. "extend" a config set?
* after calibration wait until ewm.avg is finite, filters have settled, then sweep

## components

add named components (mqtt, adc, etc)
* similar to a service
* wraps / abstracts a building block (hw driver or software component)
* interface:
  * begin(), end()
  * status
  * setLogLevel()
  * config namespace?
* user has access over console (e.g. restart, query stats, enable debug log, change config)

* detect high impedance battery connection
  * https://h.fabi.me/grafana/d/f4a22deb-8528-427d-9473-4e7b06c6d874/fugu-mppt?orgId=1&from=1741693099256&to=1741704737323

* 50hz/60hz band stop filter to remove inverter noise (notch filter)
  * https://www.youtube.com/watch?v=tpAA5eUb6eo

* boost mode, forced_pwm, reverse current
  * the largerDecrease update block causes excessive reverse current into the power supply (bat/LV terminal)
  * ignoring largerDecrease fixes the problem
* iout midpoint calibration fix! with ina226 we get an offset of
  0.9A ! https://github.com/fl4p/fugu-mppt-firmware/issues/28
* store last warnings, errors
* restart adc after some time
* simulate ina226 external reset
  se
* inspect ADC noise with scope-client

* manual sync control to find inductivity
* sweep web plot

# issues

# filtering

- med3 doesnt really help
- consider kalman
- consider smaller f_cut for Iout RC-filter (larger R)
- tracker accumulation buffer

```
I (64555) mppt: Grouping 214 D points (0.00,nan)~(1.00,29.83) into 100 bins, binW=0.010
Guru Meditation Error: Core  0 panic'ed (StoreProhibited). Exception was unhandled.

Core  0 register dump:
PC      : 0x40056fa1  PS      : 0x00060a30  A0      : 0x820275ca  A1      : 0x3fcb7e90  
--- 0x40056fa1: memcpy in ROM

A2      : 0x00000000  A3      : 0x3fcb8242  A4      : 0x00000003  A5      : 0x00000000  
A6      : 0x000094e2  A7      : 0x00000000  A8      : 0x00000000  A9      : 0x3fcb7e60  
A10     : 0x3fcb8258  A11     : 0x3fcb8074  A12     : 0x00000005  A13     : 0x3fcb825c  
A14     : 0x0000006d  A15     : 0x00000000  SAR     : 0x00000005  EXCCAUSE: 0x0000001d  
EXCVADDR: 0x00000000  LBEG    : 0x40056f5c  LEND    : 0x40056f72  LCOUNT  : 0xffffffff  
--- 0x40056f5c: memcpy in ROM
0x40056f72: memcpy in ROM



Backtrace: 0x40056f9e:0x3fcb7e90 0x420275c7:0x3fcb7ea0 0x42027986:0x3fcb7ec0 0x4200e731:0x3fcb7ee0 0x4201bc61:0x3fcb7f00 0x4201c675:0x3fcb82e0 0x4201c925:0x3fcb85a0 0x4201c9f9:0x3fcb8600 0x4201ca15:0x3fcb8650 0x4201da59:0x3fcb8670 0x4201e567:0x3fcb8690 0x4201e59a:0x3fcb86c0 0x4201d681:0x3fcb86e0 0x4201d749:0x3fcb8720 0x4202bb9c:0x3fcb8740 0x40381732:0x3fcb8760
--- 0x40056f9e: memcpy in ROM
0x420275c7: std::char_traits<char>::copy(char*, char const*, unsigned int) at /Users/fab/.espressif/tools/xtensa-esp32s3-elf/esp-12.2.0_20230208/xtensa-esp32s3-elf/xtensa-esp32s3-elf/include/c++/12.2.0/bits/char_traits.h:431
 (inlined by) std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >::_S_copy(char*, char const*, unsigned int) at /Users/fab/.espressif/tools/xtensa-esp32s3-elf/esp-12.2.0_20230208/xtensa-esp32s3-elf/xtensa-esp32s3-elf/include/c++/12.2.0/bits/basic_string.h:423
0x42027986: std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >::operator=(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >&&) at /Users/fab/.espressif/tools/xtensa-esp32s3-elf/esp-12.2.0_20230208/xtensa-esp32s3-elf/xtensa-esp32s3-elf/include/c++/12.2.0/bits/basic_string.h:867
0x4200e731: ascii::Text::operator=(ascii::Text&&) at /Users/fab/dev/pv/fugu-mppt-firmware/src/asciichart/text.h:11
0x4201bc61: ascii::Asciichart::PlotLineChart() at /Users/fab/dev/pv/fugu-mppt-firmware/src/asciichart/ascii.h:297 (discriminator 10)
0x4201c675: ascii::Asciichart::Plot() at /Users/fab/dev/pv/fugu-mppt-firmware/src/asciichart/ascii.h:113 (discriminator 6)
 (inlined by) Plot::_plotSeries(Series&, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&) at /Users/fab/dev/pv/fugu-mppt-firmware/src/etc/plot.h:88 (discriminator 6)
0x4201c925: Plot::plot() at /Users/fab/dev/pv/fugu-mppt-firmware/src/etc/plot.h:126 (discriminator 2)
0x4201c9f9: MpptController::_stopSweep(MpptControlMode, int, MpptController::CVP*)::{lambda()#1}::operator()() const at /Users/fab/dev/pv/fugu-mppt-firmware/src/mppt.h:537
0x4201ca15: void std::__invoke_impl<void, MpptController::_stopSweep(MpptControlMode, int, MpptController::CVP*)::{lambda()#1}&>(std::__invoke_other, MpptController::_stopSweep(MpptControlMode, int, MpptController::CVP*)::{lambda()#1}&) at /Users/fab/.espressif/tools/xtensa-esp32s3-elf/esp-12.2.0_20230208/xtensa-esp32s3-elf/xtensa-esp32s3-elf/include/c++/12.2.0/bits/invoke.h:61
 (inlined by) std::enable_if<is_invocable_r_v<void, MpptController::_stopSweep(MpptControlMode, int, MpptController::CVP*)::{lambda()#1}&>, void>::type std::__invoke_r<void, MpptController::_stopSweep(MpptControlMode, int, MpptController::CVP*)::{lambda()#1}&>(MpptController::_stopSweep(MpptControlMode, int, MpptController::CVP*)::{lambda()#1}&) at /Users/fab/.espressif/tools/xtensa-esp32s3-elf/esp-12.2.0_20230208/xtensa-esp32s3-elf/xtensa-esp32s3-elf/include/c++/12.2.0/bits/invoke.h:111
 (inlined by) std::_Function_handler<void (), MpptController::_stopSweep(MpptControlMode, int, MpptController::CVP*)::{lambda()#1}>::_M_invoke(std::_Any_data const&) at /Users/fab/.espressif/tools/xtensa-esp32s3-elf/esp-12.2.0_20230208/xtensa-esp32s3-elf/xtensa-esp32s3-elf/include/c++/12.2.0/bits/std_function.h:290
0x4201da59: std::function<void ()>::operator()() const at /Users/fab/.espressif/tools/xtensa-esp32s3-elf/esp-12.2.0_20230208/xtensa-esp32s3-elf/xtensa-esp32s3-elf/include/c++/12.2.0/bits/std_function.h:591
0x4201e567: TaskQueue::work() at /Users/fab/dev/pv/fugu-mppt-firmware/src/logging.cpp:237
0x4201e59a: process_queued_tasks() at /Users/fab/dev/pv/fugu-mppt-firmware/src/logging.cpp:249
0x4201d681: loopNetwork_task(void*) at /Users/fab/dev/pv/fugu-mppt-firmware/src/main.cpp:736
0x4201d749: loop() at /Users/fab/dev/pv/fugu-mppt-firmware/src/main.cpp:425
0x4202bb9c: loopTask(void*) at /Users/fab/dev/pv/fugu-mppt-firmware/components/arduino/cores/esp32/main.cpp:74
0x40381732: vPortTaskWrapper at /Users/fab/dev/esp/esp-idf-v5.1/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c:162





ELF file SHA256: de9ce413875f76aa

Rebooting...




```

```

sw
```

issues:
biggy goes in to download mode.
when connecting idf.py monitor --no reset and sending cntrl+t ctrl+z, it:

```
waiting for download
ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x15 (USB_UART_CHIP_RESET),boot:0x0 (DOWNLOAD(USB/UART0))
Saved PC:0x40041a76
--- 0x40041a76: ets_delay_us in ROM

waiting for download
ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x15 (USB_UART_CHIP_RESET),boot:0x0 (DOWNLOAD(USB/UART0))
Saved PC:0x40041a76
--- 0x40041a76: ets_delay_us in ROM

waiting for download

```

https://github.com/espressif/esp-idf/issues/13287
https://github.com/espressif/arduino-esp32/issues/6762
https://github.com/espressif/esptool/commit/0215786283660480e9ec85dd077e6fc2f46919e9#diff-cb25ea9381c2f7a537ecf1516335d3a9e8a711a9dbf3d1eff78453a611072103R356

# /Users/fab/.espressif/python_env/idf5.1_py3.9_env/lib/python3.9/site-packages/esptool/targets/esp32s3.py

```

207288640

timeout waiting for new adc sample!

```



panic
```
11 ┤                                                          ╭───╯                                  ╰─ 
 9 ┤                                                      ╭───╯                                         
 8 ┤                                                  ╭───╯                                             
Guru Meditation Error: Core  1 panic'ed (StoreProhibited). Exception was unhandled.

Core  1 register dump:
PC      : 0x40384f81  PS      : 0x00060233  A0      : 0x80384715  A1      : 0x3fccbc40  
--- 0x40384f81: insert_free_block at /Users/fab/dev/esp/esp-idf-v5.1/components/heap/tl
--- 0x40384f81: insert_free_block at /Users/fab/dev/esp/esp-idf-v5.1/components/heap/tlsf/tlsf.c:362
 (inlined by) block_insert at /Users/fab/dev/esp/esp-idf-v5.1/components/heap/tlsf/tlsf.c:388
 (inlined by) block_trim_free at /Users/fab/dev/esp/esp-idf-v5.1/components/heap/tlsf/tlsf.c:502
 (inlined by) block_prepare_used at /Users/fab/dev/esp/esp-idf-v5.1/components/heap/tlsf/tlsf.c:579
 (inlined by) tlsf_malloc at /Users/fab/dev/esp/esp-idf-v5.1/components/heap/tlsf/tlsf.c:1006

A2      : 0x3fcb084c  A3      : 0x00000005  A4      : 0x3fcb08a4  A5      : 0x3c1398b8  
A6      : 0x00007530  A7      : 0x00000020  A8      : 0x3fcd0390  A9      : 0x3fcd03a0  
A10     : 0x00000009  A11     : 0x00000000  A12     : 0x00000034  A13     : 0x00000000  
A14     : 0x3fcd03c8  A15     : 0x3fcd03a0  SAR     : 0x00000020  EXCCAUSE: 0x0000001d  
EXCVADDR: 0x00000011  LBEG    : 0x4211b199  LEND    : 0x4211b1b8  LCOUNT  : 0x00000000  
--- 0x4211b199: dsps_biquad_f32_ae32 at /Users/fab/dev/pv/fugu-mppt-firmware/managed_components/espressif__esp-dsp/modules/iir/biquad/dsps_biquad_f32_ae32.S:70
0x4211b1b8: loop_bq_end_m_ae32 at /Users/fab/dev/pv/fugu-mppt-firmware/managed_components/espressif__esp-dsp/modules/iir/biquad/dsps_biquad_f32_ae32.S:83



Backtrace: 0x40384f7e:0x3fccbc40 0x40384712:0x3fccbc60 0x40376495:0x3fccbc80 0x403764ed:0x3fccbca0 0x40376522:0x3fccbcc0 0x40386771:0x3fccbce0 0x420d42b5:0x3fccbd00 0x42010675:0x3fccbd20 0x420106a1:0x3fccbd40 0x42010702:0x3fccbd60 0x42010804:0x3fccbda0 0x420148a6:0x3fccbdc0 0x40381832:0x3fccbdf0
--- 0x40384f7e: insert_free_block at /Users/fab/dev/esp/esp-idf-v5.1/components/heap/tlsf/tlsf.c:360
 (inlined by) block_insert at /Users/fab/dev/esp/esp-idf-v5.1/components/heap/tlsf/tlsf.c:388
 (inlined by) block_trim_free at /Users/fab/dev/esp/esp-idf-v5.1/components/heap/tlsf/tlsf.c:502
 (inlined by) block_prepare_used at /Users/fab/dev/esp/esp-idf-v5.1/components/heap/tlsf/tlsf.c:579
 (inlined by) tlsf_malloc at /Users/fab/dev/esp/esp-idf-v5.1/components/heap/tlsf/tlsf.c:1006
0x40384712: multi_heap_malloc_impl at /Users/fab/dev/esp/esp-idf-v5.1/components/heap/multi_heap.c:207
0x40376495: heap_caps_malloc_base at /Users/fab/dev/esp/esp-idf-v5.1/components/heap/heap_caps.c:176
0x403764ed: heap_caps_malloc at /Users/fab/dev/esp/esp-idf-v5.1/components/heap/heap_caps.c:197
0x40376522: heap_caps_malloc_default at /Users/fab/dev/esp/esp-idf-v5.1/components/heap/heap_caps.c:223
0x40386771: malloc at /Users/fab/dev/esp/esp-idf-v5.1/components/newlib/heap.c:24
0x420d42b5: operator new(unsigned int) at /Users/brnomac003/.gitlab-runner/builds/qR2TxTby/1/idf/crosstool-NG/.build/xtensa-esp32s3-elf/src/gcc/libstdc++-v3/libsupc++/new_op.cc:50
0x42010675: void std::_Function_base::_Base_manager<ADC_Sampler::_updateAdc(ADC_Sampler::AdcState&)::{lambda(unsigned char, float)#1}>::_M_create<ADC_Sampler::_updateAdc(ADC_Sampler::AdcState&)::{lambda(unsigned char, float)#1}>(std::_Any_data&, ADC_Sampler::_updateAdc(ADC_Sampler::AdcState&)::{lambda(unsigned char, float)#1}&&, std::integral_constant<bool, false>) at /Users/fab/.espressif/tools/xtensa-esp32s3-elf/esp-12.2.0_20230208/xtensa-esp32s3-elf/xtensa-esp32s3-elf/include/c++/12.2.0/bits/std_function.h:161
0x420106a1: void std::_Function_base::_Base_manager<ADC_Sampler::_updateAdc(ADC_Sampler::AdcState&)::{lambda(unsigned char, float)#1}>::_M_init_functor<ADC_Sampler::_updateAdc(ADC_Sampler::AdcState&)::{lambda(unsigned char, float)#1}>(std::_Any_data&, ADC_Sampler::_updateAdc(ADC_Sampler::AdcState&)::{lambda(unsigned char, float)#1}&&) at /Users/fab/.espressif/tools/xtensa-esp32s3-elf/esp-12.2.0_20230208/xtensa-esp32s3-elf/xtensa-esp32s3-elf/include/c++/12.2.0/bits/std_function.h:215
 (inlined by) std::function<void (unsigned char const&, float)>::function<ADC_Sampler::_updateAdc(ADC_Sampler::AdcState&)::{lambda(unsigned char, float)#1}, void>(ADC_Sampler::_updateAdc(ADC_Sampler::AdcState&)::{lambda(unsigned char, float)#1}&&) at /Users/fab/.espressif/tools/xtensa-esp32s3-elf/esp-12.2.0_20230208/xtensa-esp32s3-elf/xtensa-esp32s3-elf/include/c++/12.2.0/bits/std_function.h:449
0x42010702: ADC_Sampler::_updateAdc(ADC_Sampler::AdcState&) at /Users/fab/dev/pv/fugu-mppt-firmware/src/adc/sampling.h:415
0x42010804: ADC_Sampler::update() at /Users/fab/dev/pv/fugu-mppt-firmware/src/adc/sampling.h:471
0x420148a6: loopRT(void*) at /Users/fab/dev/pv/fugu-mppt-firmware/src/main.cpp:518
0x40381832: vPortTaskWrapper at /Users/fab/dev/esp/esp-idf-v5.1/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c:162





ELF file SHA256: c4198f82faed558a

Rebooting...

```