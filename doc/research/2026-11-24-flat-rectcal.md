*this document is an LLM generated placeholder*

# flat rect-cal run, 2026-11-24

flat: L0 56 uH, mcpwm, rect_offset_ns 0 (uncalibrated). Plain `rect-cal` at 09:01 UTC: 47 W DCM, 1.7 A, charging, P spread 2.3 %/60 s, no lock, no balance hold.

Result: `failed: sense follows LS` (12 pts, 1 redone), offset stays 0. Iout flat (rise 0 mA, bound 25 mA), Vin within limits; the Vout reading fell with the LS window (-0.51 % at LS 1352 against +-0.20 %, ideal 1395). Same mechanism as the documented 51 uH board with an unfiltered shared current/voltage monitor (diode-emulation.md, sense path), and as flat reading Vout 0.3-0.9 V low while converting.

```
V=55.0/26.24 I=0.76/ 1.55A  41.9W 28℃33℃ 1468sps  0㎅/s DCM(H|L|Lm)=1272|1342|1342 st=⇣MPPT,1 lag=1056㎲ N=1622955 rssi=-41
I (43918349) rectcal: state=failed: sense follows LS auto=0 needed=1 drv=mcpwm offset=0 ct (0 ns, uncalibrated) backstop=0 unsaved=0
I (43918355) rectcal: last: hs=999 ideal=1395 zc=0 peak=0 learned=+0 applied=+0 ct, dV 0.5%, Iout rise 0 mA (max 25), 12 pts, 1 redone
I (43918366) rectcal: at LS 1352: Vin +0.05% (-0.30..+0.41) Vout -0.51% (+-0.20) vs refs 63.45/26.49 V
I (43918376) rectcal: LS:dI/Vin/Vout 697:+0.003/-2/-6 754:+0.001/0/6 807:+0.001/0/-2 860:+0.000/1/-10 918:+0.000/0/4 970:-0.001/1/-4 1023:-0.002/1/-11
I (43918389) rectcal: LS:dI/Vin/Vout 1079:-0.005/2/-2 1134:-0.004/3/-6 1187:-0.004/1/-18 1244:-0.006/3/-4 1297:-0.007/4/-11
V=55.0/26.26 I=0.76/ 1.55A  41.8W 28℃33℃ 1468sps  0㎅/s DCM(H|L|Lm)=1272|1341|1341 st=⇣MPPT,1 lag=1056㎲ N=1623150 rssi=-41
I (43918454) main: Conf '/littlefs/conf/coil.conf:L0' = '56e-6'
I (43918455) main: Conf '/littlefs/conf/coil.conf:rect_offset_ns' = '0'
I (43918457) main: Conf '/littlefs/conf/coil.conf:rect_cal_auto' = '0'
V=54.9/26.24 I=0.76/ 1.55A  41.8W 28℃33℃ 1468sps  0㎅/s DCM(H|L|Lm)=1272|1340|1340 st=⇣MPPT,1 lag=1056㎲ N=1623243 rssi=-41
```
