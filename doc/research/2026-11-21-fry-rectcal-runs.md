*this document is an LLM generated placeholder*

# fry rect-cal runs, 2026-11-19..21

Live offset -210 ct (-1312 ns), mcpwm, L0 80 uH. All runs failed safely; offset unchanged.

- 11-19 16:49 UTC, plain, float: `failed: limiting` (17 pts, hs=1875, ideal=692). dI within +-7 mA over LS 347-783.
- 11-20 13:47 UTC, force, 268 W CCM: `failed: Iout follows LS` (13 pts, hs=920). Status lost to an OTA.
- 11-21 09:11 and 09:29 UTC, plain: `refused: HS below calibration point` (HS ~830-840, ~12 W).
- 11-21 09:50 UTC, plain, 22 W DCM, charging: `failed: V step` (15 pts, 6 redone), Vin +1.09% vs limit +1.02% at LS 1644.

```
I (67994224) rectcal: state=failed: V step auto=0 needed=0 drv=mcpwm offset=-210 ct (-1312 ns) backstop=0 unsaved=0
I (67994227) rectcal: last: hs=877 ideal=1512 zc=0 peak=0 learned=+0 applied=+0 ct, dV 2.4%, Iout rise 0 mA (max 24), 15 pts, 6 redone
I (67994240) rectcal: at LS 1644: Vin +1.09% (-0.30..+1.02) Vout +0.00% (+-0.20) vs refs 64.73/23.46 V
I (67994249) rectcal: LS:dI/Vin/Vout 751:-0.000/-2/-1 809:+0.000/0/-1 868:-0.000/2/-1 927:+0.001/-1/-1 982:+0.000/10/0 1039:-0.003/15/0 1099:+0.001/3/-2
I (67994262) rectcal: LS:dI/Vin/Vout 1156:-0.003/11/-2 1214:-0.003/11/-1 1273:-0.003/12/-1 1330:-0.006/17/-1 1389:-0.007/13/-2 1446:-0.007/18/0
I (67994274) rectcal: LS:dI/Vin/Vout 1502:-0.009/27/-2 1554:-0.020/49/-1
V=60.6/23.47 I=0.38/ 0.95A  22.9W 24℃36℃ 512sps  0㎅/s DCM(H|L|Lm)=1057|1409|1409 st=⇡MPPT,1 lag=2106㎲ N=390778 rssi=-48
I (67994441) main: Conf '/littlefs/conf/coil.conf:L0' = '80e-6'
I (67994443) main: Conf '/littlefs/conf/coil.conf:rect_cal_auto' = '0'
I (67994445) main: Conf '/littlefs/conf/coil.conf:rect_offset_ns' = '-1312'
I (67994450) main: Conf '/littlefs/conf/coil.conf:rect_offset_drv' = 'mcpwm'
I (67994457) main: Conf '/littlefs/conf/coil.conf:rect_offset_cal' = '1'
V=60.5/23.47 I=0.38/ 0.95A  23.0W 24℃36℃ 512sps  0㎅/s DCM(H|L|Lm)=1057|1407|1407 st=⇣MPPT,1 lag=2106㎲ N=390871 rssi=-48
```

- 11-21 12:46 UTC, plain, 69 W DCM, charging, P spread 2.3 %/60 s: **done** (25 pts, 0 redone, hs=899, ideal=1648, zc=1616, learned -32 ct, applied -52 ct = -325 ns). Saved to coil.conf once the converter is off 10 s.

```
V=65.1/25.94 I=0.97/ 2.37A  63.4W 28℃40℃ 512sps  0㎅/s DCM(H|L|Lm)=1629|2323|2325 st=⇣MPPT,1 lag=2083㎲ N=569920 rssi=-48
I (78570821) rectcal: state=done auto=0 needed=0 drv=mcpwm offset=-52 ct (-325 ns) backstop=0 unsaved=1
I (78570823) rectcal: last: hs=899 ideal=1648 zc=1616 peak=1616 learned=-32 applied=-52 ct, dV 0.2%, Iout rise 0 mA (max 24), 25 pts, 0 redone
I (78570837) rectcal: LS:dI/Vin/Vout 823:-0.000/1/-1 887:-0.001/0/0 953:-0.002/0/-1 1017:+0.000/-1/-1 1082:-0.002/0/0 1147:-0.004/-1/1 1211:-0.006/1/0
I (78570850) rectcal: LS:dI/Vin/Vout 1275:-0.002/3/0 1341:-0.004/0/0 1405:-0.005/1/-1 1419:-0.004/2/1 1444:-0.003/0/-1 1470:-0.004/2/0 1470:-0.003/2/-1
I (78570863) rectcal: LS:dI/Vin/Vout 1497:-0.004/0/0 1522:-0.006/1/0 1534:-0.006/2/-1 1547:-0.006/2/-1 1574:-0.003/2/0 1599:-0.008/2/-1 1600:-0.007/1/-1
I (78570878) rectcal: LS:dI/Vin/Vout 1626:-0.010/2/0 1651:-0.017/4/1 1662:-0.021/5/-1 1719:-0.088/32/0
V=65.1/25.93 I=0.97/ 2.37A  63.5W 28℃40℃ 512sps  0㎅/s DCM(H|L|Lm)=1628|2324|2325 st=⇣MPPT,1 lag=2083㎲ N=570075 rssi=-48
I (78571136) main: Conf '/littlefs/conf/coil.conf:L0' = '80e-6'
I (78571137) main: Conf '/littlefs/conf/coil.conf:rect_cal_auto' = '0'
I (78571138) main: Conf '/littlefs/conf/coil.conf:rect_offset_ns' = '-1312'
I (78571145) main: Conf '/littlefs/conf/coil.conf:rect_offset_drv' = 'mcpwm'
I (78571151) main: Conf '/littlefs/conf/coil.conf:rect_offset_cal' = '1'
V=65.1/25.94 I=0.97/ 2.37A  63.4W 28℃40℃ 512sps  0㎅/s DCM(H|L|Lm)=1627|2323|2324 st=⇣MPPT,1 lag=2083㎲ N=570215 rssi=-48
```
