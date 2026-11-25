*this document is an LLM generated placeholder*

# fry and flat coil L, 2026-11-25: backflow switch open vs closed

Same morning, minutes apart, fry (MCPWM, fsw 38996 Hz, pwmMax 4103), Vin 71-77 V, Vout 26.6 V, fleet image fry-brk1-1127-gcb342b34:

| Run | Method | Backflow switch | L (ring fit) | median of points |
|---|---|---|---|---|
| 08:14 | on-device `measure-coil l0` | closed (firmware closes it once Iout > 0.1 A) | 76.47 uH | 75.02 uH |
| 08:15 | host `etc/measure_coil.py --mqtt fry --restore off` | open | 79.83 uH | 78.65 uH |
| 08:20 | host, same, plus `bf 1` once Iout > 0.1 A (`--close-bf`) | closed | 76.78 uH | 75.78 uH |

The open-switch run ended at H=1251 on `MPPT: High duty cycle with open backflow switch! (Idcm=2.01 ...)`, backoff 5 s [highD-bflowOpen] (protection, expected); the closed run completed all 48 points.

Conclusion: the May 2026 host sweep value 79.8 uH (and the 4-5 % gap to the November on-device runs, 75.8-77.0 uH) is the open switch body-diode drop: the output node sits one diode drop above the measured Vout, which inflates (Vin-Vout)/Vout and so L. The on-device code has closed the switch since its first version (comment in src/selftest/measure_coil.cpp; de63e98a records "+5 %"). flat: same result, see below.

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

## flat, 08:27-08:35 UTC

Vin 61-70 V, Vout 26.6 V, same image. On-device 49.49 uH (ring fit; median 49.03); host sweep, switch open (main-checkout script without --close-bf) 51.36 uH (median 51.19); host sweep, switch closed (pushed script, default) 49.25 uH (median 49.01). Open adds +3.8 %, as on fry. Both host sweeps stopped at i-max 2 A (no highD trip on flat).

```
2026-11-25 08:27:50,572     flat: measure-coil L0: fsw=38996 pwmMax=4103 band 388..1399 step 21  Vin=70.26 Vout=26.61 M=0.379
2026-11-25 08:28:39,083     flat: measure-coil: L = 49.49 uH  (ring fit, 30 DCM pts, Iout>=0.40A, +-0.4% noise +-1.6% syst; ring 324 kHz = 4.9 nF node C, ripple +-7%, 3.1 periods; median of points 49.03 uH)
2026-11-25 08:28:39,095     flat: measure-coil: Iout zero -0.005/-0.001 A (off, before/after), subtracted
2026-11-25 08:28:39,095     flat: measure-coil: L(Ipk), +-noise/offsets: 2.7-3.6A 48.51uH+-0.6/5.1% | 3.7-4.7A 49.66uH+-0.6/3.3% | 4.7-5.6A 50.34uH+-0.5/2.4%
2026-11-25 08:28:39,095     flat: measure-coil: L(Ipk) slope +1.94 +-0.42 %/A over 2.7-5.6 A, Iout/on-time offsets add up to +-1.51: coil -0.41..+4.29 %/A, drop <= 0.4 %/A
2026-11-25 08:28:39,096     flat: measure-coil: done, manual PWM at duty 0 ('mppt' resumes tracking)
```

### Host sweep, switch open

```
saving output to /Users/fab/dev/pv/fugu-mppt-firmware/measure-coil-runs/20261125T082900Z-flat.log
args= mqtt=flat, steps=48, lo=0.25, hi=0.9, i_max=2.0, dwell=3.0, ls_steps=24, ls_lo=0.5, ls_hi=1.4, apply_margin=12, restore=off, yes=True
fsw=38995.86 Hz  period=4103 ticks (160.00 MHz)  pwmMax=4103  pwmCtrlMax=3782  coil.conf L0=48.6e-6
idle: Vin=70.03 Vout=26.58  M=Vout/Vin=0.380

       H      D    Vin   Vout   Iout mode    L_uH
     389  0.095  69.88  26.47  0.266  DCM   49.70
     410  0.100  69.87  26.49  0.287  DCM   51.08
     452  0.110  69.74  26.37  0.370  DCM   48.25
     494  0.120  69.69  26.48  0.467  DCM   45.22
     515  0.126  69.67  26.48  0.489  DCM   46.92
     536  0.131  69.64  26.46  0.508  DCM   48.91
     557  0.136  69.63  26.47  0.528  DCM   50.80
     578  0.141  69.61  26.44  0.543  DCM   53.21
     599  0.146  69.59  26.44  0.579  DCM   53.64
     620  0.151  69.57  26.45  0.611  DCM   54.34
     662  0.161  69.35  26.52  0.777  DCM   48.11
     683  0.166  69.31  26.42  0.837  DCM   47.78
     704  0.172  69.31  26.42  0.878  DCM   48.36
     725  0.177  69.29  26.47  0.904  DCM   49.65
     746  0.182  69.28  26.46  0.920  DCM   51.64
     767  0.187  69.25  26.46  0.948  DCM   52.94
     788  0.192  69.25  26.46  0.967  DCM   54.77
     809  0.197  69.24  26.47  1.007  DCM   55.38
     830  0.202  69.20  26.50  1.078  DCM   54.29
     851  0.207  69.10  26.18  1.228  DCM   50.91
     872  0.213  69.03  26.44  1.327  DCM   48.51
     893  0.218  69.01  26.48  1.378  DCM   48.83
     914  0.223  69.00  26.46  1.411  DCM   50.01
     935  0.228  69.00  26.47  1.439  DCM   51.30
     956  0.233  69.02  26.45  1.458  DCM   53.03
     977  0.238  69.02  26.48  1.485  DCM   54.28
     998  0.243  69.03  26.48  1.518  DCM   55.41
    1019  0.248  69.00  26.37  1.576  DCM   55.96
    1040  0.253  68.93  26.50  1.721  DCM   52.83
    1061  0.259  68.83  26.54  1.907  DCM   49.34
    1082  0.264  68.81  26.49  2.002  DCM   48.96
  Iout 2.00 > i-max 2.0; stopping  sweep

L = 51.36 uH  (ring fit, 28 DCM pts, +-0.4% noise +-1.6% syst; ring 317 kHz = 4.9 nF node C, ripple +-7%, 3.0 periods; median of points 51.19 uH)  Iout>=0.40A, IQR 10%
Iout zero -0.002/+0.004 A (off, before/after), subtracted
L(Ipk), +-noise/offsets: 2.7-3.6A 50.10uH+-0.6/5.0% | 3.7-4.5A 51.74uH+-0.5/3.3% | 4.6-5.4A 52.19uH+-0.5/2.4%
L(Ipk) slope +2.43 +-0.45 %/A over 2.7-5.4 A, Iout/on-time offsets add up to +-1.56: coil -0.03..+4.88 %/A, drop <= 0.0 %/A
  -> set coil.conf L0=5.136e-05   (was 48.6e-6)
  note: scales with Iout calibration & pwmMax; dead-time/DCR bias ~few % low
exit 0
```

### Host sweep, switch closed

```
saving output to /Users/fab/dev/pv/fugu-mppt-firmware/measure-coil-runs/20261125T083158Z-flat.log
args= mqtt=flat, steps=48, lo=0.25, hi=0.9, i_max=2.0, dwell=3.0, ls_steps=24, ls_lo=0.5, ls_hi=1.4, apply_margin=12, restore=mppt, close_bf=True, yes=True
fsw=38995.86 Hz  period=4103 ticks (160.00 MHz)  pwmMax=4103  pwmCtrlMax=3782  coil.conf L0=48.6e-6
idle: Vin=70.40 Vout=26.57  M=Vout/Vin=0.377

       H      D    Vin   Vout   Iout mode    L_uH
     387  0.094  70.29  26.44  0.270  DCM   49.19
     408  0.099  70.31  26.51  0.298  DCM   49.35
     429  0.105  70.30  26.44  0.342  DCM   47.81
     450  0.110  70.27  26.45  0.409  DCM   43.89
     471  0.115  70.26  26.37  0.445  DCM   44.43
     492  0.120  70.27  26.50  0.485  DCM   44.12
     513  0.125  70.27  26.47  0.505  DCM   46.11
     534  0.130  70.26  26.37  0.524  DCM   48.44
     555  0.135  70.26  26.44  0.548  DCM   49.87
     576  0.140  70.25  26.46  0.570  DCM   51.56
     597  0.146  70.23  26.43  0.612  DCM   51.65
     618  0.151  70.20  26.42  0.669  DCM   50.57
     639  0.156  70.16  26.39  0.758  DCM   47.74
     660  0.161  70.14  26.51  0.841  DCM   45.52
     681  0.166  70.10  26.44  0.878  DCM   46.54
     702  0.171  70.07  26.44  0.906  DCM   47.88
     723  0.176  70.05  26.49  0.927  DCM   49.48
     744  0.181  70.05  26.48  0.946  DCM   51.33
     765  0.186  70.04  26.46  0.980  DCM   52.46
     786  0.192  70.01  26.42  1.020  DCM   53.27
     807  0.197  69.97  26.52  1.102  DCM   51.63
     828  0.202  69.90  26.54  1.250  DCM   47.72
     849  0.207  69.87  26.42  1.347  DCM   46.84
     870  0.212  69.85  26.47  1.396  DCM   47.28
     891  0.217  69.84  26.50  1.425  DCM   48.45
     912  0.222  69.84  26.48  1.448  DCM   50.05
     933  0.227  69.83  26.48  1.475  DCM   51.38
     954  0.233  69.83  26.47  1.500  DCM   52.87
     975  0.238  69.80  26.51  1.543  DCM   53.49
     996  0.243  69.73  26.50  1.629  DCM   52.76
    1017  0.248  69.61  26.10  1.845  DCM   49.56
    1038  0.253  69.55  26.50  1.974  DCM   46.97
    1059  0.258  69.51  26.50  2.031  DCM   47.43
  Iout 2.03 > i-max 2.0; stopping  sweep

L = 49.25 uH  (ring fit, 30 DCM pts, +-0.4% noise +-1.7% syst; ring 318 kHz = 5.1 nF node C, ripple +-7%, 3.1 periods; median of points 49.01 uH)  Iout>=0.41A, IQR 9%
Iout zero +0.000/+0.002 A (off, before/after), subtracted
L(Ipk), +-noise/offsets: 2.6-3.6A 47.99uH+-0.6/5.5% | 3.7-4.6A 49.46uH+-0.5/3.5% | 4.7-5.6A 50.36uH+-0.5/2.5%
L(Ipk) slope +2.71 +-0.41 %/A over 2.6-5.6 A, Iout/on-time offsets add up to +-1.72: coil +0.17..+5.24 %/A, no drop resolved
  -> set coil.conf L0=4.925e-05   (was 48.6e-6)
  note: scales with Iout calibration & pwmMax; dead-time/DCR bias ~few % low
exit 0
```
