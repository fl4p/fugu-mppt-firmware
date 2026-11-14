
Implement a tool that estimates the resistance of the battery connection path.
Bad connection at the charger or battery terminal or bad wiring can add significant resistance,
which is noticeable when charging current changes.

Here are data samples for Iout and Vout:
[I-data-2026-05-19 22_17_51.csv](../adcscope/data/bat-resistance-biggy-60m/I-data-2026-05-19%2022_17_51.csv)
[U_out-data-2026-05-19 22_17_40.csv](../adcscope/data/bat-resistance-biggy-60m/U_out-data-2026-05-19%2022_17_40.csv)

Create an algorithm in python that computes the output terminal resistance on a moving window.
In case there is not enough current variance, it should output nan. 