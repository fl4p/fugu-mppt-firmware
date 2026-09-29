---
title: "LFP Longevity"
sidebar_position: 10
mdx:
  format: mdx
---

import useBrokenLinks from '@docusaurus/useBrokenLinks';

export const Ref = ({n}) => {
  useBrokenLinks().collectAnchor(`ref-${n}`);
  return <a id={`ref-${n}`} />;
};

# LiFePO4 longevity: what the aging literature says about charge control

## 1. Introduction and scope

A solar charger decides three things that affect how long a lithium iron phosphate (LFP) pack
lasts: how full it charges the pack, how long it keeps the pack there, and under which
temperature and current it charges. This page surveys the published aging evidence on those
questions for cells with a LiFePO4 cathode and a graphite anode, and maps it onto the settings
in [`charger.conf`](../reference/config/charger.md).

Two kinds of aging are distinguished throughout. **Calendar aging** is the capacity loss of a
cell at rest; it depends on time, temperature and the state of charge (SoC) at which the cell is
kept. **Cycle aging** is the additional loss caused by charging and discharging; it depends on
charge throughput, depth of cycling, current and the SoC range cycled. Lifetime models such as
the one in [[4](#ref-4)] treat total loss as the sum of the two. A pack in a solar installation spends most
of its life at rest or cycling slowly, so calendar aging carries a large share of its loss (see
[§4.4](#44-how-much-does-cycling-matter-at-solar-rates)).

The evidence below comes from laboratory aging studies on single cells: 18650 and 26650
cylindrical cells, 240 mAh pouch cells and 100 Ah prismatic cells, one pack-level float test,
one cell datasheet, two reviews and one practitioner article. None of the studies tested the
large prismatic cells (around 280 Ah) commonly used in stationary packs. Transferring their
numbers to such a pack is an assumption that is repeated where it matters. Section 9 lists
the other limits of the evidence.

## 2. Aging mechanisms in brief

In LFP/graphite cells, capacity is lost mainly through **loss of lithium inventory** to the solid
electrolyte interphase (SEI) that grows on the graphite anode. The lithium consumed there is no
longer available for cycling, and the two electrodes slip out of balance. Keil et al. found this
electrode-balance shift to be the main cause of calendar fade, with the anode potential as its
driver: the more lithiated the graphite, the lower its potential and the faster the SEI grows
[[1](#ref-1)]. Zsoldos et al. measured the parasitic heat flow of lithiated graphite in electrolyte and
found that its reactivity rises step by step with SoC, although the graphite potential is nearly
constant over much of that range [[2](#ref-2)]. A second mechanism specific to LFP is iron dissolution from the
cathode and deposition on the anode, which accelerates lithium loss. In [[2](#ref-2)] it occurred during
cycling, was stronger at high temperature and high SoC, and did not occur during a voltage hold
(see [§5.3](#53-holding-a-full-cell-at-constant-voltage)).

At low temperature and high charge current, **lithium plating** takes over: metallic lithium
deposits on the graphite instead of intercalating into it. This also shows up as loss of lithium
inventory, but it can be much faster ([§6.2](#62-low-temperature-and-lithium-plating)).

Because the LFP cathode is stable when fully delithiated, the high-voltage side reactions that
age nickel- and cobalt-based cells near 100 % SoC were not seen for LFP in [[1](#ref-1)]. The SoC
dependence of LFP aging is therefore the SoC dependence of the graphite anode.

## 3. Storage state of charge and calendar aging

### 3.1 Calendar fade rises in steps with SoC

Keil et al. stored three 18650 cell types, among them the A123 APR18650M1A (1.1 Ah LFP), at
16 SoCs from 0 to 100 % and at 25, 40 and 50 °C for 9 to 10 months [[1](#ref-1)]. Capacity fade did not
rise steadily with SoC. It sat on plateaus covering 20–30 % of SoC or more, with a marked step
above about 70 % SoC for the LFP cells (about 60 % for the NCA and NMC cells). The step
coincides with the central peak of the graphite in differential voltage analysis, a graphite
lithiation of about 50 %. That peak sat at 73 % SoC in the LFP cell and at 57 % in the NCA cell;
the position depends on how the electrodes are balanced, so it differs between cell designs [[1](#ref-1),
Fig. 5 and text]. The LFP cells showed no additional fade toward 100 % SoC, their aging
"correlates entirely with the anode potential", and their resistance increase was the lowest of
the three types and largely independent of storage SoC [[1](#ref-1), Results and Conclusions]. After nine
months the LFP fade rate was about 0.2 percentage points per month at 25 °C and about 0.5 at
50 °C [[1](#ref-1), Results]. The authors recommend keeping the graphite less than 50 % lithiated for
long-term storage [[1](#ref-1), Conclusions].

Naumann et al. stored the Sony/Murata US26650FTC1 (3 Ah LFP) at 17 combinations of temperature
(0–60 °C) and SoC for 885 days [[3](#ref-3)]. Capacity loss grew with storage SoC, but between 37.5 % and
62.5 % SoC there was almost no difference at 40 °C [[3](#ref-3), Fig. 2c]. A cell stored at 50 % SoC and
25 °C lost about 4.7 % of its capacity in 885 days [[3](#ref-3), §3.1.2]. In the accompanying dissertation,
cells stored at 0 °C and 10 °C showed almost no aging compared with the higher temperatures,
although their periodic check-ups ran at 25 °C [[5](#ref-5), §4.4].

### 3.2 Temperature is the first-order calendar factor

Grolleau et al. stored a commercial 15 Ah graphite/LFP cell at 30, 45 and 60 °C and at 30, 65
and 100 % SoC for at least 450 days [[6](#ref-6)]. Fully charged cells lost less than 10 % in 450 days at
30 °C and 20 % at 45 °C; at 60 °C they reached 20 % loss in about 60 days, against about 100
days at 30 % SoC [[6](#ref-6), §3.1]. The authors conclude that storage SoC "is of secondary importance
compared to storage temperature, but its influence increases with temperature", and that below
30 °C the SoC influence predicted by their model is minor [[6](#ref-6), §3.2 and Conclusion]. Their model
estimates an end of life at 100 % SoC of 20, 13.5 and 9 years at 20, 25 and 30 °C [[6](#ref-6), Table 5];
those figures are extrapolations of a fitted model, not observations.

The two results are compatible. Three storage SoCs cannot resolve plateaus 20–30 % wide, and at
every temperature tested in [[6](#ref-6)] the fully charged cells still aged fastest. Temperature decides
how fast a cell ages at rest; SoC decides on which plateau it does so, and its weight grows with
temperature.

Lam et al. analysed calendar aging of several commercial cell types over up to 13 years,
including two K2 Energy LFP 18650 types stored at 24–85 °C and 50 or 100 % SoC for 7.8 years
[[7](#ref-7)]. Across the data set the activation energy of capacity fade generally decreased with
increasing SoC, and fits with a single activation energy did not describe the temperature
dependence well. The two LFP types from the same manufacturer had nearly identical temperature
dependence at 50 % SoC but very different dependence at 100 % SoC [[7](#ref-7)]. A temperature or SoC
sensitivity measured on one LFP cell is therefore not safely transferable to another.

### 3.3 Manufacturer guidance

The specification of a widely used 280 Ah prismatic cell, the EVE LF280K, gives a long-term
storage temperature of 0–35 °C (within one year), a storage SoC of 30–50 %, a self-discharge of
at most 3 % per month at 25 °C and 30–50 % SoC, and a recommended operating SoC range of
10–90 % [[8](#ref-8), §3 rows 6, 9 and 11; §7]. The low self-discharge means that a full pack loses
little charge by resting, so a charger has no need to keep topping it up.

## 4. Operating window and depth of cycling

### 4.1 The average SoC of the cycling window

Zsoldos et al. cycled 240 mAh LFP/graphite pouch cells in five SoC windows (0–25 %, 0–60 %,
0–80 %, 0–100 % and 75–100 %) at C/3, at 40 and 55 °C, with two electrolyte salts and two
graphites, for about 2500 h [[2](#ref-2)]. For both salts and both temperatures, capacity retention
ranked 0–25 % best, then 0–60 %, 0–80 %, 0–100 %, and 75–100 % worst [[2](#ref-2), Results]. The 75–100 %
window cycles a quarter of the capacity, the 0–100 % window all of it, so the ranking follows
average SoC rather than depth of discharge. In the fade-rate comparison, temperature changed the
fade rate by 15–50 %, the salt by 5–30 % and the SoC window by 250–400 % [[2](#ref-2), Fig. 2c–d]; the
authors conclude that average SoC was the most critical factor "over the factors of temperature,
depth of discharge, electrolyte salt choice or graphite choice" [[2](#ref-2), Conclusions]. After 2500 h
the best cells retained 97 % and the worst 76 % of their capacity. The authors caution that the
test was too short for lifetime extrapolation and mention preliminary, unpublished results
suggesting that cells cycled at high average SoC may recover in later cycles [[2](#ref-2), Discussion]. The
voltage cost of cycling low is small: the average discharge voltage was 3.3 V in the 75–100 %
window and 3.15 V in the 0–25 % window [[2](#ref-2), Discussion].

Charge rate was not varied in [[2](#ref-2)], and 25 °C was not tested.

### 4.2 Depth of cycle and the mid-SoC anomaly

Naumann et al. cycled the same 3 Ah 26650 cell as in [[3](#ref-3)] at 19 test points for 885 days, up to
about 10 600 full equivalent cycles (FEC) [[4](#ref-4)]. At the end of the test the total capacity loss was
higher for larger depths of cycle. Cells with shallow cycles (depth below 80 %) first lost
capacity faster, with a partly reversible loss, and then levelled off after about 1000 FEC
[[4](#ref-4), §3.1.3 and Conclusions]. For 20 % cycles, the window centred on 50 % SoC aged more than the
windows centred on 25 % or 75 % [[4](#ref-4), Conclusions]. The authors did not see the effect in two
realistic load profiles with moving SoC ranges and left its mechanism to a follow-up publication
[[4](#ref-4), Conclusions and Outlook]. A later paper by the same group, Spingler et al., reports a capacity recovery of
more than 10 % after continuous shallow cycling in three LFP/graphite cell models (two 26650, one
18650): a large part of the shallow-cycling loss was recovered by holding the cells at 0 % or
100 % SoC, and differential voltage analysis and post-mortem experiments point to strongly
non-uniform lithium distributions in the electrodes [[22](#ref-22), Abstract]. The dissertation reports that no clear relation between SoC range
and degradation could be stated for the first 5000 FEC [[5](#ref-5), §7.1].

Preger et al. cycled the A123 APR18650M1A over 0–100 %, 20–80 % and 40–60 % SoC, that is,
different depths around the same 50 % midpoint [[9](#ref-9)]. Fade increased with depth of discharge for
all three chemistries tested, but the SoC range had little effect on the capacity of the LFP
cells at 200 equivalent full cycles in their statistical analysis [[9](#ref-9), Fig. 5f]. The LFP cells
reached 80 % capacity after 2500 to 9000 equivalent full cycles; most had not reached 80 % when
the study ended, and for those the authors extrapolated the then-current linear fade rate [[9](#ref-9)].

### 4.3 High-rate results that point the other way

Stroe's thesis found low SoC worse than high SoC, and the conclusion of a pulse study points the
same way, although its measured capacities do not. In the thesis, 2.5 Ah LFP/graphite cells
cycled at 4 C, 42.5 °C and 35 % depth of cycle around average SoCs of 27.5, 50 and 72.5 %
lost capacity faster the lower the average SoC [[10](#ref-10), Table 5.2 and §7.2.4]. The cells had not
reached end of life, the fits were extrapolated, and the author notes that the result is
opposite to his own calendar aging result, where lower storage SoC aged less [[10](#ref-10), §7.2.4].
Kang et al. held commercial 18650 LFP cells at 30, 50, 70 and 90 % SoC and applied symmetric
4 C charge and discharge pulses of 18 s (2 % of capacity), 500 per cycle [[11](#ref-11), Table 2]. Their
measured capacity loss rose with SoC: 2.6–3.8 mAh at 30 %, 13.5–13.9 mAh at 50 %, 14.5–15.6 mAh
at 70 % and 91–93 mAh at 90 % SoC, on cells of about 1.88 Ah [[11](#ref-11), Table 3]. From
incremental-capacity analysis, however, the authors conclude that the loss of lithium inventory
was largest at 30 % SoC, and they advise avoiding low SoC [[11](#ref-11), Conclusion]. The low-SoC penalty
in [[11](#ref-11)] therefore rests on that diagnostic interpretation, not on the measured capacities.

Both studies used 4 C, twelve times the rate of [[2](#ref-2)]. Zsoldos et al. note that the Stroe result
could be confounded by lithium plating at fast currents and that storage at high SoC aged faster
in the same works, and they suggest that the failure regime may depend on current [[2](#ref-2),
Results]. None of these high-rate results was obtained at the currents of solar charging.

### 4.4 How much does cycling matter at solar rates?

Naumann et al. validated their combined model with a synthetic PV home-storage profile: SoC
between 5.4 % and 80 % (average 51.4 %), average charge rate 0.243 C, at 40 °C for 885 days [[4](#ref-4),
Table 4 and §3.3.2]. The model attributed 9.21 % capacity loss to calendar aging and 3.64 % to
cycling, so cycling was 28 % of the estimated total [[4](#ref-4), §3.3.2.1]. In the dissertation, a cell
cycled at 0.2 C with an 80 % depth of cycle lost about 14.5 % in 885 days at 40 °C, and its
aging was "dominated by calendar aging due to small additional cycle aging with low C-rates"
[[5](#ref-5), §4.6.2.2].

At solar charge rates, then, where the pack rests and how warm it is matter at least as much as
how it is cycled.

## 5. End-of-charge voltage, float and constant-voltage holds

### 5.1 End-of-charge voltage

The LF280K specification defines the standard charge as 0.5 C constant current to 3.65 V, then
constant voltage until the current falls to 0.05 C, at 25 °C [[8](#ref-8), §4.2]. Its cycle-life ratings
(at least 6000 cycles to 80 % at 25 °C, at least 2500 at 45 °C) are for that charge and a 0.5 C
discharge to 2.5 V, under a 300 kgf clamp [[8](#ref-8), §5.1 rows 4 and 5].

None of the reviewed sources compares a lower end-of-charge voltage, such as 3.50 or 3.55 V
with tail-current termination, with 3.65 V on the same cells at the same depth of cycle. Cao
et al. aged 100 Ah prismatic LFP cells at 23 °C in stages whose voltage window and current
changed during the test: the first stage cycled between 3.10 and 3.45 V (60 A charge, 95–100 A
discharge), later stages used windows such as 2.80–3.45 V and 2.80–3.60 V, and one cell had
stages with discharge cutoffs of 2.6 and 2.5 V [[12](#ref-12), §2.1 and §3.1.1]. For that new cell the paper
is internally inconsistent: the results give 33.3 % loss after 10 390 cycles (about 8000 FEC) at
3.275 % per 1000 cycles, the conclusions 33.9 % after 10 000 cycles at 3.26 %, and the capacities
in its Table 4 (107.1 Ah to 70.69 Ah) correspond to about 34 % [[12](#ref-12), §3.1.1, Table 4 and
Conclusions]. The authors observed that raising the charge cutoff voltage within 3.60 V did not
speed up aging, but that was a change of conditions during the test, not a controlled comparison
[[12](#ref-12), §3.1.1]. Their
second-life recommendation of 2.80–3.55 V (10–90 % SoC), 0.5 C charge and no high temperature
[[12](#ref-12), Conclusions] matches the manufacturer's recommended SoC window for that cell [[12](#ref-12), Table 1].

Because the upper part of the LFP charge curve is short, a lower end-of-charge voltage mainly
changes the current at which the cell counts as full. A practitioner article, which is the origin
of this firmware's termination line, interpolates the manufacturer's termination pair (3.65 V at
0.033 C or 0.05 C) linearly down to about 3.37 V at zero current, which it gives as the rest
voltage of a full cell, and recommends 3.50 V per cell as a conservative charging voltage [[13](#ref-13)].
It states that stopping short of 100 % costs only "a tiny fraction of capacity"; that is an
assertion, not a measurement.

### 5.2 Why the time at the top matters more than the voltage

Sections 3 and 4 show that the time spent at high SoC drives both calendar and cycle aging. A
full LFP pack loses little charge at rest ([§3.3](#33-manufacturer-guidance)), so it gains nothing
from being held full. What shortens its life is spending many hours on the high-SoC plateau,
whichever voltage it was charged to.

### 5.3 Holding a full cell at constant voltage

In [[2](#ref-2)], LFP pouch cells held at 3.0 V (0 % SoC) or 3.65 V (100 % SoC) at 60 °C for 1000 h,
with one C/3 cycle every 100 h, showed no iron deposition above background. The authors conclude
that iron dissolution needs cycling and that storage at high SoC ages the cell through SEI growth
[[2](#ref-2), Methods and Results]. A hold at the top is therefore ordinary high-SoC calendar aging, not a
separate mechanism.

Yi et al. built 18650-size LFP/graphite cells and compared float charging with continuous 1 C
cycling in a 2.2–3.65 V window at 25, 35, 45, 55 and 65 °C [[14](#ref-14)]. The float voltage is not stated
in the text. After 200 days of float at 25 and 35 °C, retention was above 95 %; at 65 °C it fell
below 65 % within 100 days. Cycling at 25 and 35 °C reached below 95 % within 100 days. The
authors found float aging "relatively mild" below 45 °C, with loss of active lithium as the main
mechanism [[14](#ref-14), Results]. Continuous 1 C cycling is a much heavier duty than a solar pack sees, so
the comparison says that float is not a fast failure mode, not that float is harmless.

Azzam et al. floated A123 18650 LFP cells at voltages from 3.2 to 3.6 V (3.33, 3.34, 3.35, 3.36,
3.38, 3.4, 3.5 and 3.6 V among them) [[15](#ref-15), Table 1]. Their capacity data are limited to 30 °C;
float currents were also measured from 5 to 50 °C [[15](#ref-15), Abstract]. With a model fitted to the 30 °C
data they split the float current into SEI growth, which rose over the whole voltage range, and
cathode lithiation, which stayed near 1.2 µA below 3.38 V and rose from there to 5 µA [[15](#ref-15), §3.3,
Fig. 12a]. The measured capacity-loss rate was not monotonic in float voltage: the 3.38 V cell
lost capacity more slowly than the 3.33 and 3.34 V cells, and the 3.4 V cell faster than the
3.5 V cell [[15](#ref-15), §3.1 and §3.3]. The authors caution that cathode lithiation can mask capacity
loss, so a lower measured fade rate does not by itself mean less degradation; the 3.38 V cell,
for instance, had one of the highest internal resistances [[15](#ref-15), §3.1].

Wei et al. floated a 32-cell pack of 180 Ah LFP cells in a substation DC supply at 115 V (about
3.59 V per cell) for one year, with the BMS discharging any cell that exceeded 3.65 V. The pack
kept 97 % of its initial capacity; internal resistances did not change greatly, and 94 % of the
cell voltages stayed stable [[16](#ref-16)]. There was one float voltage and no control group.

Takahashi and Shodai floated prismatic cells with a manganese-substituted LFP cathode at 4.0 V:
more than 70 % capacity after 24 months at 25 °C, 60 % after one month at 55 °C, with manganese
deposited on the anode [[17](#ref-17)]. At 4.0 V and with a different cathode, the result does not transfer
to plain LFP charged to 3.65 V or less.

A recent review recommends an LFP float window of 3.35–3.45 V per cell, with about 3.4 V
"widely regarded as ideal", and states that lowering a float setpoint by 100–300 mV "has been
shown to extend cycle life by a factor of two to five", citing [[19](#ref-19)] and [[16](#ref-16)] [[18](#ref-18), §5.1]. Neither
cited work supports that figure: [[19](#ref-19)] is a charging-protocol study on LiCoO2 18650 cells charged
to 4.2 V, with no float or LFP content, and [[16](#ref-16)] reports a single float condition without a
voltage comparison. The factor is therefore not used here.

Taken together, the reviewed sources neither show that a low float (around 3.4 V) harms an LFP
cell nor show that it helps. They do show that a hold at or near the end-of-charge voltage keeps
the cell on the fastest-aging calendar plateau.

## 6. Temperature

### 6.1 High temperature

Calendar fade rises strongly with temperature: about 0.2 percentage points per month at 25 °C
against 0.5 at 50 °C in [[1](#ref-1)]; below 10 % in 450 days at 30 °C against 20 % at 45 °C for full
cells in [[6](#ref-6)]. In [[9](#ref-9)] the LFP fade rate increased with temperature between 15 and 35 °C. The
LF280K cycle-life rating drops from at least 6000 cycles at 25 °C to at least 2500 at 45 °C [[8](#ref-8)].
In [[12](#ref-12)], one cell whose test temperature was raised from 45 to 55 °C aged at about 15.5 % per
1000 cycles during that period and then developed an aging knee.

Naumann et al. subtracted modelled calendar aging from their cycle tests at 25 and 40 °C. The
remaining cycle aging at 80 % depth agreed between the two temperatures up to about 8000 FEC;
at 100 % depth the curves began to differ after 4000 FEC [[4](#ref-4), §3.1.4]. Between 25 and 40 °C the
temperature penalty is therefore mostly calendar aging, which argues for keeping a *resting*
pack cool as much as a charging one.

### 6.2 Low temperature and lithium plating

The LF280K may be charged between 0 and 55 °C and discharged between −20 and 55 °C [[8](#ref-8), §3
rows 7 and 8].

Rauhala et al. performed post-mortem analysis on 2.3 Ah cylindrical graphite/LFP cells cycled
with an electric-vehicle profile (1 C charge to 3.6 V, then constant voltage) at room
temperature, 0 °C and −18 °C [[20](#ref-20)]. The cycle-life data, published earlier by Omar et al. and
reproduced in [[20](#ref-20), Table 2], give 2071 equivalent cycles to 80 % capacity at room temperature,
1850 at 0 °C and 185 at −18 °C. The post-mortem found lithium plating and disordering of the
graphite in the −18 °C cells, and the authors conclude that charging at sub-zero temperatures
"should be avoided in all applications" [[20](#ref-20), §3.1 and Conclusions].

Petzl et al. cycled 2.5 Ah 26650 graphite/LFP cells at −22 °C with 1 C or C/2 charging to full
or 80 % SoC [[21](#ref-21)]. Plating appeared as loss of cyclable lithium, was strongest early, and limited
itself because the lost lithium shifted the electrode balance; part of the loss was reversible,
and the ohmic resistance rose as electrolyte was consumed on the plated lithium [[21](#ref-21), Abstract and
§3].

Preger et al. relay earlier reports of a temperature at which LFP cycle aging is lowest, between
5 and 10 °C, with faster aging both above and below it [[9](#ref-9), Temperature dependence]. The work
they cite was not examined for this survey.

These studies refer to ambient or chamber temperature. A charging pack warms itself, so the
temperature that matters is that of the cells, as reported by the BMS.

## 7. Charge rate

The only reviewed study that isolates the charge rate in a controlled comparison is [[4](#ref-4)]
(0.2, 0.5 and 1 C at 80 % depth of cycle around 50 % SoC, 40 °C). Per day, higher currents
aged the cells faster, and after calendar aging was subtracted, higher C-rates also caused more
cycle aging; overall the authors report that "the C-rate showed only small influence" on
capacity loss. A cell with 2 C discharge changed its degradation rate after about 4000 FEC, which
the authors suggest may be lithium plating [[4](#ref-4), §3.1.2 and Conclusions]. Preger et al. varied only
the discharge rate, with charging fixed at 0.5 C, and found little rate dependence for LFP [[9](#ref-9),
Discharge rate dependence]. Cao et al. changed the current between test stages on 100 Ah
prismatic cells at 23 °C and reported that "the effect of the charge and discharge current on the aging speed is less
evident from the testing results", but the stages changed other conditions as well [[12](#ref-12), §2.1 and §3.1.1].

The LF280K standard charge current is 0.5 C and the maximum continuous current 1 C [[8](#ref-8), §3 rows
4 and 5]. At the rates a solar charger typically delivers to a large pack, the reviewed evidence
does not identify charge current as a significant aging factor, with one exception: at low
temperature, higher current is what causes plating ([§6.2](#62-low-temperature-and-lithium-plating)).
No reviewed study isolates the charge rate on large prismatic cells.

## 8. Implications for this charger's settings

The charger charges at constant current up to the pack voltage limit, then holds the highest
cell at `cv_eoc` while the current tapers, and declares the pack full when the highest cell
crosses a termination line between `cv_float` at zero current and `cv_eoc` at
`tail_c_rate × bat_c`. After termination it lowers its voltage target to `vout_max_fallback`
(by default `n_cells × cv_float`); while the highest cell is at or above `cv_float`, a
cell-voltage feedback loop can pull the target further down, by at most `vout_offset_max`. This
regulates a voltage, not the pack current. Charging is allowed again once the pack has discharged
`recharge_dod × bat_c` since the full point, or once its highest cell stays
`recharge_vfloor_band` below `cv_float`.
[LFP charging](../guide/charging/lfp-charging.md) and
[Termination](../guide/charging/termination.md) describe the logic in detail; the table below
uses the defaults from [`charger.conf`](../reference/config/charger.md).

One consequence of this design deserves attention. A voltage target near the rest voltage of a
full cell keeps the charge current small, so while the solar array covers the load the pack tends
to stay close to full and to discharge only when the load exceeds the solar power. How much
current actually flows depends on how the target compares with the pack's rest voltage and on
sensor offsets. `recharge_dod` sets when charging is allowed again; it does not limit how far the
pack discharges. With the default of 0.2 and light loads, a pack that is recharged to full as
soon as it has lost 20 % spends much of its time in the top fifth of its range, a high-SoC regime
like the 75–100 % window that aged fastest in [[2](#ref-2)], and above the calendar-aging step of [[1](#ref-1)]. A
larger `recharge_dod`, deep discharges overnight or cloudy days lower the average SoC; stopping
short of full lowers it on every cycle, which is what
[`partial_charge`](../guide/charging/termination.md#partial-charge-ceiling-partial_charge) does.

These mechanisms depend on data from the BMS. The termination line and the recharge release need
`bat_c` and the BMS cell-voltage and pack-current reports. `partial_charge` additionally needs a
full charge since boot (its Ah counter measures the deficit since the last full charge), fresh
cell-voltage and pack-current data (each expires after 180 s), and `recharge_dod` must be
smaller than `partial_charge`; after a reboot, or while the data are stale, the charger charges to full. The
temperature limits need a configured BMS temperature topic; each sensor expires one hour after
its last report, and with no fresh sensor the temperature policy is off.
`full_charge_interval` schedules a full charge; whether the BMS balances the cells during it
depends on the BMS's own balancing conditions.

| setting | default | what the evidence says | strength of evidence |
|---|---|---|---|
| `cv_eoc` | 3.5 V | Below the manufacturer's standard 3.65 V CV target [[8](#ref-8)]; matches the practitioner recommendation [[13](#ref-13)]. No reviewed study compares end-of-charge voltages on the same cells ([§5.1](#51-end-of-charge-voltage)), so neither the benefit nor the capacity given up is quantified. | low |
| `cv_float` | 3.325 V | Zero-current end of the termination line; with the default `vout_max_fallback` also the per-cell voltage target after termination. Below the 3.37 V rest voltage of a full cell given in [[13](#ref-13)] and below the 3.38 V at which the modelled cathode-lithiation current starts to rise in [[15](#ref-15)]; but in [[15](#ref-15)] the 3.33 V cell lost measured capacity faster than the 3.38 V cell, a comparison the authors caution may be masked by cathode lithiation. A target near the rest voltage ends charging; it does not lower the SoC. | low |
| `tail_c_rate` | 0.05 | The manufacturer's cutoff current, specified at 3.65 V [[8](#ref-8)]. At `cv_eoc` = 3.5 V, the interpolation in [[13](#ref-13)] for a 3.65 V / 0.05 C pair would terminate at about 0.023 C, so the default line terminates earlier. How much capacity that forgoes was not measured in any reviewed source. | medium for the value at 3.65 V; the rest is inference |
| `recharge_dod` | 0.20 | Allows charging again once the pack has discharged this fraction of `bat_c` since the last full charge; it does not bound how far the pack discharges. With light loads the pack spends much of its time in its top fifth, the high-SoC regime that aged fastest in [[2](#ref-2)] (tested there as 75–100 %). A larger value means fewer full charges and less time near 100 %. | medium |
| `partial_charge` | 0 (off) | The direction is well supported: lower average SoC ages LFP/graphite more slowly in calendar storage [[1](#ref-1), [3](#ref-3), [6](#ref-6)] and in low-rate cycling [[2](#ref-2)]. The best ceiling is cell-specific: the calendar step lay at 73 % SoC in the LFP cell of [[1](#ref-1)] and depends on electrode balancing; it has not been measured for large prismatic cells. The manufacturer's recommended range is 10–90 % [[8](#ref-8)]. Charging stops at `partial_charge` and resumes once the pack has discharged `recharge_dod` below it; prerequisites above. | high for the direction, low for any particular number |
| `full_charge_interval` | 7 d | Full charges are needed for BMS balancing (subject to the BMS's own conditions) and to re-zero the Ah counter. No reviewed source addresses how often an LFP pack needs one. | none (engineering choice) |
| `ibat_max` | 20 A | At or below 0.5 C, charge rate had little influence on aging in [[4](#ref-4)], and at low rates calendar aging dominated [[5](#ref-5)]. Stay at or below the manufacturer's 0.5 C standard charge [[8](#ref-8)]. | medium; transfer to large prismatic cells assumed |
| `bat_temp_min` | 0 °C | The manufacturer's lower charge limit [[8](#ref-8)]; plating and a drastic loss of cycle life below 0 °C [[20](#ref-20), [21](#ref-21)]. Acts only with fresh BMS temperature reports. | high |
| `bat_temp_derate` / `bat_temp_max` | 45 / 55 °C | 55 °C is the manufacturer's upper charge limit, and rated cycle life at 45 °C is less than half that at 25 °C [[8](#ref-8)]; calendar fade also rises steeply with temperature [[1](#ref-1), [6](#ref-6)]. Acts only with fresh BMS temperature reports. | medium |

For a pack that will not be used for weeks, the manufacturer's storage guidance applies: 30–50 %
SoC and 0–35 °C [[8](#ref-8)]. A cool resting pack ages much more slowly than a warm one [[3](#ref-3), [5](#ref-5), [6](#ref-6)].

The practitioner article [[13](#ref-13)] warns that a partial charge followed by a rest at the same point
"constitutes a memory writing cycle", that repeating it "gradually leads to near-complete loss of
usable capacity", and that a proper full charge from time to time erases it. A different memory
effect in LiFePO4 is established in the peer-reviewed literature: Sasaki et al. found that it
appears after a single cycle of partial charge and discharge, and that the slight voltage change
it causes can lead to substantial errors in estimating the SoC [[23](#ref-23), Abstract]. That effect
concerns the voltage curve, not capacity loss. None of the aging studies reviewed here tested
repeated partial charging with a hold at a fixed SoC, so this page can neither confirm nor rule
out a capacity effect of the `partial_charge` hold.

## 9. Limitations of the evidence

**Cell formats and cell types.** The results come from 18650, 26650 and pouch cells of 0.24 to
15 Ah, a 100 Ah prismatic cell [[12](#ref-12)] and one pack of 180 Ah cells [[16](#ref-16)]. Each study used one or two
cell types. The SoC at which calendar fade steps up depends on the electrode balance of the
design [[1](#ref-1)], and cells from the same manufacturer can differ strongly at 100 % SoC [[7](#ref-7)]. No
reviewed study tested a 280 Ah prismatic cell under controlled conditions.

**Temperature ranges.** The operating-window ranking of [[2](#ref-2)] was measured at 40 and 55 °C only.
The calendar plateaus of [[1](#ref-1)] were measured at 25–50 °C, and the cycle tests of [[4](#ref-4)] at 25 and
40 °C. That the window ranking also holds at room temperature is an extrapolation, supported by
the plateau structure that [[1](#ref-1)] found at 25 °C.

**Extrapolated lifetimes.** Several lifetime figures are extrapolations, not observations: the
LFP cycle lives in [[9](#ref-9)], the high-rate SoC trend in [[10](#ref-10)] and the calendar lifetimes of [[6](#ref-6),
Table 5]. Lam et al. show that constant-activation-energy extrapolation can be substantially wrong
for calendar aging [[7](#ref-7)].

**Where the studies disagree.** Low average SoC was better at C/3 in [[2](#ref-2)] but worse at 4 C in
[[10](#ref-10)], and in the diagnostic conclusion, though not the measured capacities, of [[11](#ref-11)]. Zsoldos et
al. suggest that lithium plating at high current could explain the difference [[2](#ref-2)]; that has not
been tested. Storage SoC mattered
strongly in [[1](#ref-1)] but was "of secondary importance" to temperature in [[6](#ref-6)], a difference that is
largely one of resolution and temperature. For 20 % cycles, the window around 50 % SoC aged
more than the windows around 25 % and 75 % in [[4](#ref-4)]; that effect was largely recoverable [[22](#ref-22)] and did
not appear in realistic profiles, but it means that parking a cycling pack in the middle of its range
is not automatically the gentlest choice.

**Independence.** [[1](#ref-1)], [[3](#ref-3)], [[4](#ref-4)], [[5](#ref-5)] and [[22](#ref-22)] come from one research group, and [[3](#ref-3)], [[4](#ref-4)] and [[5](#ref-5)] share
one data set, so they count as one line of evidence for the calendar-SoC dependence; [[2](#ref-2)] and [[9](#ref-9)]
are independent of it. The reviews [[7](#ref-7)] and [[18](#ref-18)] are not independent evidence for the studies they
relay, and the lifetime factor claimed in [[18](#ref-18)] is not supported by its own citations
([§5.3](#53-holding-a-full-cell-at-constant-voltage)).

**What the sources do not cover.** No reviewed study compares end-of-charge voltages such as
3.50 V and 3.65 V on the same cells at low current and room temperature; if the fade were equal,
the voltage choice would matter little and only the SoC window would. No reviewed source
addresses how often an LFP pack needs a full charge for cell balancing; the cell-voltage spread
of a pack against the time since its last full charge would answer that for a given pack. No
reviewed study tests the combination this charger uses, a partial-charge ceiling held by
load-following with a periodic full charge. And whether the high-SoC penalty of [[2](#ref-2)] persists
beyond 2500 h at room temperature is open; its authors report unpublished hints of recovery.

## References

1. <Ref n="1" />P. Keil, S. F. Schuster, J. Wilhelm, J. Travi, A. Hauser, R. C. Karl, A. Jossen, "Calendar
   aging of lithium-ion batteries. I. Impact of the graphite anode on capacity fade," *Journal of
   The Electrochemical Society* 163(9), A1872–A1880 (2016).
   [doi:10.1149/2.0411609jes](https://doi.org/10.1149/2.0411609jes). Cells: Table I; plateaus:
   Fig. 2 and 5 with text; LFP fade rates: Results, paragraph after Fig. 2.
2. <Ref n="2" />E. S. Zsoldos, D. T. Thompson, W. Black, S. M. Azam, J. R. Dahn, "The operation window of
   lithium iron phosphate/graphite cells affects their lifetime," *Journal of The Electrochemical
   Society* 171(8), 080527 (2024).
   [doi:10.1149/1945-7111/ad6cbd](https://doi.org/10.1149/1945-7111/ad6cbd). Windows and ranking:
   Figs. 2–3 and text; voltage hold: Methods "Voltage hold protocol" and Fig. 4b.
3. <Ref n="3" />M. Naumann, M. Schimpe, P. Keil, H. C. Hesse, A. Jossen, "Analysis and modeling of calendar
   aging of a commercial LiFePO4/graphite cell," *Journal of Energy Storage* 17, 153–169 (2018).
   [doi:10.1016/j.est.2018.01.019](https://doi.org/10.1016/j.est.2018.01.019). SoC dependence:
   §3.1.1, Fig. 2c; 4.7 % at 25 °C: §3.1.2.
4. <Ref n="4" />M. Naumann, F. B. Spingler, A. Jossen, "Analysis and modeling of cycle aging of a commercial
   LiFePO4/graphite cell," *Journal of Power Sources* 451, 227666 (2020).
   [doi:10.1016/j.jpowsour.2019.227666](https://doi.org/10.1016/j.jpowsour.2019.227666). C-rate:
   §3.1.2; depth of cycle: §3.1.3; temperature: §3.1.4; PV profile: Table 4 and §3.3.2.1.
5. <Ref n="5" />M. Naumann, *Techno-economic evaluation of stationary battery energy storage systems with
   special consideration of aging*, Dr.-Ing. dissertation, Technical University of Munich (2018).
   [mediaTUM 1434981](https://mediatum.ub.tum.de/doc/1434981/1434981.pdf). Storage at 0 and 10 °C:
   §4.4; 0.2 C cell: §4.6.2.2; summary: §7.1.
6. <Ref n="6" />S. Grolleau, A. Delaille, H. Gualous, P. Gyan, R. Revel, J. Bernard, E. Redondo-Iglesias,
   J. Peter, "Calendar aging of commercial graphite/LiFePO4 cell – Predicting capacity fade under
   time dependent storage conditions," *Journal of Power Sources* 255, 450–458 (2014).
   [doi:10.1016/j.jpowsour.2013.11.098](https://doi.org/10.1016/j.jpowsour.2013.11.098). Results:
   §3.1; model lifetimes: Table 5.
7. <Ref n="7" />V. N. Lam, X. Cui, F. Stroebl, M. Uppaluri, S. Onori, W. C. Chueh, "A decade of insights:
   Delving into calendar aging trends and implications," *Joule* 9(1), 101796 (2025).
   [doi:10.1016/j.joule.2024.11.013](https://doi.org/10.1016/j.joule.2024.11.013). Cell list:
   Table 1; activation energies: Fig. 4 and text.
8. <Ref n="8" />EVE Power Co., Ltd., *LF280K (3.2V 280Ah) Product Specification*, Version B, effective
   23 March 2021. Parameters: §3; standard charge: §4.2; cycle life: §5.1; storage: §7.
9. <Ref n="9" />Y. Preger, H. M. Barkholtz, A. Fresquez, D. L. Campbell, B. W. Juba, J. Romàn-Kustas,
   S. R. Ferreira, B. Chalamala, "Degradation of commercial lithium-ion cells as a function of
   chemistry and cycling conditions," *Journal of The Electrochemical Society* 167(12), 120532
   (2020). [doi:10.1149/1945-7111/abae37](https://doi.org/10.1149/1945-7111/abae37). Read in the
   accepted manuscript (Sandia report SAND2020-8433J); SoC range: Fig. 5f.
10. <Ref n="10" />D.-I. Stroe, *Lifetime Models for Lithium-ion Batteries used in Virtual Power Plant
    Applications*, PhD thesis, Department of Energy Technology, Aalborg University (2014).
    Test matrix: §5.3, Table 5.2; average-SoC dependence: §7.2.4, Eq. 7.6.
11. <Ref n="11" />J. Kang, G. Yang, Y. Wang, J. V. Wang, Q. Wang, G. Zhu, "Study of aging mechanisms in
    LiFePO4 batteries with various SOC levels using the zero-sum pulse method," *iScience*
    27(7), 110287 (2024).
    [doi:10.1016/j.isci.2024.110287](https://doi.org/10.1016/j.isci.2024.110287). Protocol:
    Table 2; findings: Conclusion.
12. <Ref n="12" />Z. Cao, W. Gao, Y. Fu, C. Turchiano, N. Vosoughi Kurdkandi, J. Gu, C. Mi, "Second-life
    assessment of commercial LiFePO4 batteries retired from EVs," *Batteries* 10(9), 306 (2024).
    [doi:10.3390/batteries10090306](https://doi.org/10.3390/batteries10090306). Cell data:
    Table 1; aging results: §3.1.1 and Table 4; recommendation: Conclusions 1–4.
13. <Ref n="13" />E. Bretscher, "Charging marine lithium battery banks," Nordkyn Design, 21 February 2021,
    last updated 17 April 2022.
    [nordkyndesign.com](https://nordkyndesign.com/charging-marine-lithium-battery-banks/).
    Practitioner article without cited sources.
14. <Ref n="14" />S. Yi, B. Wang, Z. Chen, R. Wang, D. Wang, "The difference in aging behaviors and mechanisms
    between floating charge and cycling of LiFePO4/graphite batteries," *Ionics* 25(5),
    2139–2145 (2019).
    [doi:10.1007/s11581-018-2607-2](https://doi.org/10.1007/s11581-018-2607-2). Results
    paragraphs on Figs. 1–3.
15. <Ref n="15" />M. Azzam, D. U. Sauer, C. Endisch, M. Lewerenz, "Comprehensive analysis of float current
    behavior and calendar aging mechanisms in lithium-ion batteries," *Batteries & Supercaps*
    9(1), e202500349 (2026; published online 2025).
    [doi:10.1002/batt.202500349](https://doi.org/10.1002/batt.202500349). Float voltages:
    Table 1; capacity-loss rates: §3.1; current decomposition: §3.3, Fig. 12a.
16. <Ref n="16" />Z. Wei, G. Zhong, W. Su, W. Wang, Y. Zhang, K. Liu, H. Liu, "Float-charging characteristics
    of lithium iron phosphate battery based on direct-current power supply system in
    substation," *Journal of Energy Engineering* 142(1), 04015016 (2016).
    [doi:10.1061/(ASCE)EY.1943-7897.0000273](https://doi.org/10.1061/%28ASCE%29EY.1943-7897.0000273).
    Abstract and Conclusion.
17. <Ref n="17" />M. Takahashi, T. Shodai, "Float charging performance of lithium ion batteries with LiFePO4
    cathode," *Electrochemistry* 78(5), 342–344 (2010).
    [doi:10.5796/electrochemistry.78.342](https://doi.org/10.5796/electrochemistry.78.342).
18. <Ref n="18" />M. S. Khan, S. Maddipatla, M. Pecht, "A review of float charging in lithium-ion batteries:
    Degradation mechanisms, influencing factors, and optimization strategies," *Journal of Power
    Sources* 674, 239788 (2026).
    [doi:10.1016/j.jpowsour.2026.239788](https://doi.org/10.1016/j.jpowsour.2026.239788).
    Float window and lifetime factor: §5.1.
19. <Ref n="19" />S. S. Zhang, "The effect of the charging protocol on the cycle life of a Li-ion battery,"
    *Journal of Power Sources* 161(2), 1385–1391 (2006).
    [doi:10.1016/j.jpowsour.2006.06.040](https://doi.org/10.1016/j.jpowsour.2006.06.040).
20. <Ref n="20" />T. Rauhala, K. Jalkanen, T. Romann, E. Lust, N. Omar, T. Kallio, "Low-temperature aging
    mechanisms of commercial graphite/LiFePO4 cells cycled with a simulated electric vehicle load
    profile — A post-mortem study," *Journal of Energy Storage* 20, 344–356 (2018).
    [doi:10.1016/j.est.2018.10.007](https://doi.org/10.1016/j.est.2018.10.007). Read in the
    accepted manuscript; cycle life: Table 2 (data from Omar et al., its ref. [50]).
21. <Ref n="21" />M. Petzl, M. Kasper, M. A. Danzer, "Lithium plating in a commercial lithium-ion battery –
    A low-temperature aging study," *Journal of Power Sources* 275, 799–807 (2015).
    [doi:10.1016/j.jpowsour.2014.11.065](https://doi.org/10.1016/j.jpowsour.2014.11.065).
    Protocol: §2.2.
22. <Ref n="22" />F. B. Spingler, M. Naumann, A. Jossen, "Capacity recovery effect in commercial LiFePO4 /
    graphite cells," *Journal of The Electrochemical Society* 167(4), 040526 (2020).
    [doi:10.1149/1945-7111/ab7900](https://doi.org/10.1149/1945-7111/ab7900). Abstract.
23. <Ref n="23" />T. Sasaki, Y. Ukyo, P. Novák, "Memory effect in a lithium-ion battery," *Nature Materials*
    12(6), 569–575 (2013). [doi:10.1038/nmat3623](https://doi.org/10.1038/nmat3623). Abstract.
