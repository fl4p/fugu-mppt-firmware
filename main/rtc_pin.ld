/* RTC state that must keep its address across firmware images (OTA): fixed slots at the top of
   rtc_slow_seg, above all data the linker places from its bottom, so RTC variables added elsewhere
   do not move them. Slots never move; a new block gets a new slot below the lowest one. */
SECTIONS
{
  .rtc_charger ORIGIN(rtc_slow_seg) + LENGTH(rtc_slow_seg) - 0x300 (NOLOAD) :
  {
    _rtc_charger_start = ABSOLUTE(.);
    *(.rtc_charger)
    _rtc_charger_end = ABSOLUTE(.);
  }
  .rtc_wire ORIGIN(rtc_slow_seg) + LENGTH(rtc_slow_seg) - 0x200 (NOLOAD) :
  {
    _rtc_wire_start = ABSOLUTE(.);
    *(.rtc_wire)
    _rtc_wire_end = ABSOLUTE(.);
  }
}
ASSERT(_rtc_force_slow_end <= _rtc_charger_start, "RTC slow data reaches the pinned RTC blocks")
ASSERT(_rtc_charger_end <= _rtc_wire_start, ".rtc_charger exceeds 0x100 B")
ASSERT(_rtc_wire_end <= ORIGIN(rtc_slow_seg) + LENGTH(rtc_slow_seg), ".rtc_wire exceeds 0x200 B")
