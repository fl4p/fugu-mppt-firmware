/* Wire-guard RTC copy (.rtc_wire) at a fixed address: the top 0x200 B of rtc_slow_seg, above all
   data the linker places from its bottom, so RTC variables added elsewhere do not move it. */
SECTIONS
{
  .rtc_wire ORIGIN(rtc_slow_seg) + LENGTH(rtc_slow_seg) - 0x200 (NOLOAD) :
  {
    _rtc_wire_start = ABSOLUTE(.);
    *(.rtc_wire)
    _rtc_wire_end = ABSOLUTE(.);
  }
}
ASSERT(_rtc_force_slow_end <= _rtc_wire_start, "RTC slow data reaches .rtc_wire")
ASSERT(_rtc_wire_end <= ORIGIN(rtc_slow_seg) + LENGTH(rtc_slow_seg), ".rtc_wire exceeds 0x200 B")
