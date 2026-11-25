*this document is an LLM generated placeholder*

# fry coil L, 2026-11-25 08:14-08:24 UTC: backflow switch open vs closed

Same morning, minutes apart, fry (MCPWM, fsw 38996 Hz, pwmMax 4103), Vin 71-77 V, Vout 26.6 V, fleet image fry-brk1-1127-gcb342b34:

| Run | Method | Backflow switch | L (ring fit) | median of points |
|---|---|---|---|---|
| 08:14 | on-device `measure-coil l0` | closed (firmware closes it once Iout > 0.1 A) | 76.47 uH | 75.02 uH |
| 08:15 | host `etc/measure_coil.py --mqtt fry --restore off` | open | 79.83 uH | 78.65 uH |
| 08:20 | host, same, plus `bf 1` once Iout > 0.1 A (`--close-bf`) | closed | 76.78 uH | 75.78 uH |

The open-switch run ended at H=1251 on `MPPT: High duty cycle with open backflow switch! (Idcm=2.01 ...)`, backoff 5 s [highD-bflowOpen] (protection, expected); the closed run completed all 48 points.

Conclusion: the May 2026 host sweep value 79.8 uH (and the 4-5 % gap to the November on-device runs, 75.8-77.0 uH) is the open switch body-diode drop: the output node sits one diode drop above the measured Vout, which inflates (Vin-Vout)/Vout and so L. The on-device code has closed the switch since its first version (comment in src/selftest/measure_coil.cpp; de63e98a records "+5 %"). flat was not re-measured with the switch open.

## On-device run (havan pv/fugu_console.log)

```
2026-11-25 08:14:09,217      fry: measure-coil L0: fsw=38996 pwmMax=4103 band 358..1288 step 19  Vin=76.18 Vout=26.56 M=0.349
2026-11-25 08:15:21,920      fry: measure-coil: L = 76.47 uH  (ring fit, 40 DCM pts, Iout>=0.32A, +-0.4% noise +-1.6% syst; ring 249 kHz = 5.4 nF node C, ripple +-11%, 2.7 periods; median of points 75.02 uH)
2026-11-25 08:15:21,920      fry: measure-coil: Iout zero -0.003/-0.002 A (off, before/after), subtracted
2026-11-25 08:15:21,921      fry: measure-coil: L(Ipk), +-noise/offsets: 2.1-3.0A 75.31uH+-0.7/5.3% | 3.0-3.8A 77.01uH+-0.6/3.3% | 3.8-4.2A 77.08uH+-0.6/2.5%
2026-11-25 08:15:21,921      fry: measure-coil: L(Ipk) slope +1.54 +-0.87 %/A over 2.1-4.2 A, Iout/on-time offsets add up to +-2.21: coil -2.42..+5.50 %/A, drop <= 2.4 %/A
2026-11-25 08:15:21,921      fry: measure-coil: done, manual PWM at duty 0 ('mppt' resumes tracking)
```

## Host sweep, switch open

```
saving output to /Users/fab/dev/pv/fugu-mppt-firmware/measure-coil-runs/20261125T081538Z-fry.log
args= mqtt=fry, steps=48, lo=0.25, hi=0.9, i_max=2.0, dwell=3.0, ls_steps=24, ls_lo=0.5, ls_hi=1.4, apply_margin=12, restore=off, yes=True
fsw=38995.86 Hz  period=4103 ticks (160.00 MHz)  pwmMax=4103  pwmCtrlMax=3782  coil.conf L0=75e-6
idle: Vin=76.27 Vout=26.59  M=Vout/Vin=0.349

       H      D    Vin   Vout   Iout mode    L_uH
     358  0.087  75.67  26.60  0.225  DCM   60.46
     377  0.092  75.62  26.60  0.241  DCM   62.63
     396  0.097  75.58  26.60  0.251  DCM   66.10
     415  0.101  75.57  26.61  0.259  DCM   70.36
     434  0.106  75.54  26.60  0.268  DCM   74.27
     453  0.110  75.53  26.60  0.276  DCM   78.79
     472  0.115  75.51  26.60  0.284  DCM   82.98
     491  0.120  75.49  26.60  0.291  DCM   87.38
     510  0.124  75.46  26.60  0.303  DCM   90.49
     529  0.129  75.42  26.60  0.324  DCM   91.03
     548  0.134  75.30  26.61  0.358  DCM   87.93
     567  0.138  75.12  26.60  0.437  DCM   76.75
     586  0.143  74.92  26.60  0.517  DCM   68.84
     605  0.147  74.86  26.61  0.555  DCM   68.20
     624  0.152  74.83  26.62  0.575  DCM   69.96
     643  0.157  74.81  26.61  0.589  DCM   72.50
     662  0.161  74.78  26.61  0.600  DCM   75.35
     681  0.166  74.76  26.61  0.608  DCM   78.64
     700  0.171  74.69  26.61  0.615  DCM   81.90
     719  0.175  74.68  26.61  0.625  DCM   84.97
     738  0.180  74.65  26.61  0.640  DCM   87.40
     757  0.184  74.61  26.62  0.658  DCM   89.20
     776  0.189  74.57  26.61  0.683  DCM   90.24
     795  0.194  74.44  26.61  0.731  DCM   88.16
     814  0.198  74.08  26.62  0.849  DCM   78.54
     833  0.203  73.78  26.63  0.957  DCM   72.12
     852  0.208  73.67  26.62  1.001  DCM   71.92
     871  0.212  73.64  26.63  1.030  DCM   72.97
     890  0.217  73.64  26.63  1.048  DCM   74.81
     909  0.222  73.67  26.63  1.062  DCM   77.10
     928  0.226  73.69  26.63  1.075  DCM   79.44
     947  0.231  73.74  26.64  1.087  DCM   81.94
     966  0.235  73.79  26.66  1.097  DCM   84.50
     985  0.240  73.82  26.66  1.113  DCM   86.71
    1004  0.245  73.85  26.66  1.133  DCM   88.54
    1023  0.249  73.79  26.67  1.163  DCM   89.32
    1042  0.254  73.66  26.67  1.213  DCM   88.48
    1061  0.259  73.27  26.68  1.305  DCM   84.02
    1080  0.263  72.59  26.68  1.455  DCM   76.28
    1099  0.268  72.04  26.68  1.550  DCM   72.70
    1118  0.272  71.80  26.69  1.599  DCM   72.25
    1137  0.277  71.66  26.69  1.626  DCM   73.10
    1156  0.282  71.59  26.69  1.645  DCM   74.49
    1175  0.286  71.56  26.70  1.661  DCM   76.15
    1194  0.291  71.50  26.69  1.674  DCM   77.88
    1213  0.296  71.51  26.70  1.685  DCM   79.82
    1232  0.300  71.48  26.70  1.695  DCM   81.75
       0  0.000  76.97  26.65  0.004  DCM        
  converter not switching after `dc 1251` (start blocked?); stopping  sweep

L = 79.83 uH  (ring fit, 37 DCM pts, +-0.4% noise +-1.7% syst; ring 241 kHz = 5.4 nF node C, ripple +-11%, 2.6 periods; median of points 78.65 uH)  Iout>=0.34A, IQR 15%
Iout zero -0.000/+0.003 A (off, before/after), subtracted
L(Ipk), +-noise/offsets: 2.1-2.9A 78.76uH+-0.7/5.5% | 2.9-3.6A 80.65uH+-0.7/3.5% | 3.7-4.3A 80.09uH+-0.6/2.5%
L(Ipk) slope +1.11 +-0.78 %/A over 2.1-4.3 A, Iout/on-time offsets add up to +-2.08: coil -2.52..+4.74 %/A, drop <= 2.5 %/A
  -> set coil.conf L0=7.983e-05   (was 75e-6)
  note: scales with Iout calibration & pwmMax; dead-time/DCR bias ~few % low
exit 0
```

## Host sweep, switch closed

```
saving output to /Users/fab/dev/pv/fugu-mppt-firmware/measure-coil-runs/20261125T0830Z-fry-closebf.log
args= mqtt=fry, steps=48, lo=0.25, hi=0.9, i_max=2.0, dwell=3.0, ls_steps=24, ls_lo=0.5, ls_hi=1.4, apply_margin=12, restore=mppt, close_bf=True, yes=True, save=/Users/fab/dev/pv/fugu-mppt-firmware/measure-coil-runs/20261125T0830Z-fry-closebf.log
fsw=38995.86 Hz  period=4103 ticks (160.00 MHz)  pwmMax=4103  pwmCtrlMax=3782  coil.conf L0=75e-6
idle: Vin=77.16 Vout=26.67  M=Vout/Vin=0.346

       H      D    Vin   Vout   Iout mode    L_uH
     355  0.087  76.74  26.68  0.229  DCM   60.28
     374  0.091  76.73  26.69  0.251  DCM   61.19
     393  0.096  76.73  26.69  0.259  DCM   65.24
     412  0.100  76.73  26.69  0.267  DCM   69.64
     431  0.105  76.71  26.68  0.274  DCM   74.38
     450  0.110  76.70  26.69  0.282  DCM   78.45
     469  0.114  76.68  26.69  0.293  DCM   82.07
     488  0.119  76.66  26.70  0.304  DCM   85.55
     507  0.124  76.64  26.70  0.320  DCM   87.64
     526  0.128  76.59  26.70  0.352  DCM   85.60
     545  0.133  76.43  26.70  0.432  DCM   74.58
     564  0.137  76.33  26.70  0.513  DCM   67.04
     583  0.142  76.28  26.70  0.554  DCM   66.18
     602  0.147  76.23  26.71  0.579  DCM   67.44
     621  0.151  76.21  26.70  0.593  DCM   69.98
     640  0.156  76.17  26.71  0.605  DCM   72.78
     659  0.161  76.15  26.71  0.615  DCM   75.85
     678  0.165  76.14  26.71  0.624  DCM   79.04
     697  0.170  76.12  26.72  0.635  DCM   82.06
     735  0.179  75.99  26.71  0.668  DCM   86.32
     754  0.184  75.85  26.72  0.701  DCM   86.15
     773  0.188  75.72  26.73  0.768  DCM   82.29
     792  0.193  75.38  26.72  0.913  DCM   71.80
     811  0.198  75.19  26.72  0.990  DCM   69.00
     830  0.202  75.11  26.73  1.024  DCM   69.61
     849  0.207  75.03  26.72  1.045  DCM   71.29
     868  0.212  74.95  26.72  1.060  DCM   73.25
     887  0.216  74.90  26.65  1.073  DCM   75.69
     906  0.221  74.85  26.66  1.084  DCM   78.01
     925  0.225  74.72  26.66  1.094  DCM   80.27
     944  0.230  74.61  26.65  1.105  DCM   82.48
     963  0.235  74.53  26.66  1.123  DCM   84.19
     982  0.239  74.41  26.66  1.143  DCM   85.61
    1001  0.244  74.24  26.66  1.179  DCM   85.80
    1020  0.249  73.96  26.66  1.245  DCM   83.49
    1039  0.253  73.32  26.66  1.384  DCM   76.23
    1058  0.258  72.69  26.67  1.509  DCM   70.84
    1077  0.262  72.32  26.67  1.569  DCM   69.73
    1096  0.267  72.00  26.68  1.601  DCM   69.90
    1115  0.272  71.77  26.68  1.620  DCM   70.91
    1134  0.276  71.62  26.68  1.633  DCM   72.36
    1153  0.281  71.42  26.69  1.643  DCM   73.79
    1172  0.286  71.30  26.67  1.653  DCM   75.55
    1191  0.290  71.14  26.67  1.660  DCM   77.24
    1210  0.295  71.02  26.66  1.668  DCM   78.99
    1229  0.300  70.82  26.65  1.676  DCM   80.53
    1248  0.304  70.64  26.66  1.687  DCM   81.96
    1267  0.309  70.43  26.66  1.698  DCM   83.28

L = 76.78 uH  (ring fit, 39 DCM pts, +-0.4% noise +-1.6% syst; ring 247 kHz = 5.4 nF node C, ripple +-11%, 2.8 periods; median of points 75.78 uH)  Iout>=0.34A, IQR 15%
Iout zero -0.001/+0.000 A (off, before/after), subtracted
L(Ipk), +-noise/offsets: 2.2-3.1A 76.09uH+-0.6/5.4% | 3.1-3.9A 77.38uH+-0.6/3.3% | 3.9-4.5A 76.88uH+-0.5/2.5%
L(Ipk) slope +1.02 +-0.62 %/A over 2.2-4.5 A, Iout/on-time offsets add up to +-2.05: coil -2.26..+4.30 %/A, drop <= 2.3 %/A
  -> set coil.conf L0=7.678e-05   (was 75e-6)
  note: scales with Iout calibration & pwmMax; dead-time/DCR bias ~few % low
exit 0
```
