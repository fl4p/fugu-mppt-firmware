# CV not Bat

- Connect PV only, no bat.
- Expected: V_out_max at output

# Battery Interrupt

- Connect PV and Bat
- During charge, disconnect battery
- Expected: Output voltage spikes max +10%, then falls back to V_out_max
- Current: Total shutdown, then slow recovery

# UV

- V_in = 10.5V
- Expected: shutdown