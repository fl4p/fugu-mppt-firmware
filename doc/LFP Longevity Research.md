*this document is an LLM generated placeholder*

# LiFePO4 charging and cycle management for longevity

Research date: 2026-09-05. Decision-grade tier (multi-source, quantitative). Every material finding
carries a source ID (S-number, table in §7), an exact locator, an evidence state, and a
direct-evidence / inference mark. Number tags follow origin · transformation · status.

The question: how should a solar charger (this firmware's `charger.conf`: `cv_eoc`, `cv_float`,
`tail_c_rate`, `recharge_dod`, `ibat_max`, temperature cutouts) treat an LFP/graphite pack so the
pack lasts longest?

## 1. Bottom line

1. **Time spent at high state of charge is the best-evidenced lifetime lever for LFP/graphite.**
   Capacity fade is loss of cyclable lithium to SEI growth on the graphite, and the graphite is
   most reactive when most lithiated. Calendar fade steps into its fastest plateau above a
   cell-specific SoC that was 73 % for the LFP 18650 in S1 (57 % for its NCA cell; the position
   moves with anode oversizing). In the one study that varied the operating window (S8: 240 mAh
   lab pouch cells, C/3, 40 and 55 °C), cycling 75–100 % SoC aged faster than 0–100 %, which aged
   faster than 0–25 %, and average SoC outweighed the other factors *that study varied* (40 vs
   55 °C, DoD, salt, graphite). Charge rate was not varied there, and 25 °C was not tested.
   Keeping the pack off the top when it is not needed is worth more than voltage fine-tuning.
2. **Do not hold a full LFP pack at its end-of-charge voltage.** A terminated pack has nothing to
   gain from a hold (LFP self-discharge is ≤3 %/month at 25 °C and 30–50 % SoC, S3) and sits in
   the fastest calendar-aging plateau (S1, S7). A hold at 3.65 V and 60 °C for 1000 h, with a C/3
   check cycle every 100 h, caused no iron dissolution (S8), so the harm of a top-of-charge hold
   is ordinary high-SoC SEI growth, not a special float mechanism. A *low* float (around 3.4 V) is
   weakly evidenced either way: below 45 °C Yi et al. found float aging milder than cycling (S10,
   second-hand), and Azzam et al. saw a cathode-side float current appear only above ≈3.38 V
   (S10, second-hand). The defensible rule is: no sustained hold at or near the EoC voltage.
3. **Recharge hysteresis alone does not lower the average SoC; a partial-charge target would.**
   With `recharge_dod` = 0.2 the pack is released for recharge at 80 % SoC and charged back to
   100 %, so it lives in the 80–100 % window, which is the worst one in S8. A larger
   `recharge_dod` reduces the number of full-charge events and the time at 100 %, which is
   still worth having, but the pack never leaves the top quarter. To move the average SoC down,
   the charger would have to stop short of full (an Ah-counted ceiling such as 70–80 % of `bat_c`
   from the last full charge) and go to 100 % only periodically for balancing. That feature does
   not exist in this firmware today. Caveat from S5/S7: shallow cycles (10–20 % DoD) parked around
   50 % SoC showed a strong but partly reversible dip, and the journal version ranks that 50 %
   window worse than 25 % or 75 % for 20 % DoD cycles; mechanism deferred to a follow-up, absent
   under moving-SoC profiles.
4. **End-of-charge voltage: 3.65 V is the vendor's standard CV target, not merely a ceiling.** The
   LF280K standard charge is 0.5 C to 3.65 V, held to 0.05 C (S3). No inspected primary compares
   a 3.50–3.55 V cutoff with 3.65 V on the same cells, and nothing inspected establishes how much
   capacity a lower cutoff with tail termination leaves on the table. A lower cutoff is therefore
   an *untested experiment*, arguably motivated by the ≈3.38 V float-current onset relayed in
   S10, not a sourced result.
5. **Temperature: never charge below 0 °C; keep the pack cool.** 0 °C is the datasheet charge
   floor (S3). Cycling at −18 °C cut cycle life to under 10 % of room temperature through lithium
   plating, while 0 °C still gave ~90 % (figures S9 relays from its ref. [50]; the sub-zero
   warning is S9's own conclusion). Calendar and cycle fade of LFP both rise with temperature
   above room temperature (S2, S7); the datasheet cycle life drops from ≥6000 to ≥2500 cycles from
   25 °C to 45 °C (S3). Pure cycle aging between 25 and 40 °C was small once calendar aging was
   subtracted, with 80 % DoD curves agreeing to ~8000 FEC and 100 % DoD curves diverging after
   ~4000 FEC (S7), so the temperature penalty is mostly calendar aging, which favours a cool
   *resting* pack more than a cool *charging* pack.
6. **Charge current is a weak lever at solar rates, on the evidence available.** The only
   inspected charge-rate variation is S7 (0.2 / 0.5 / 1 C, 3 Ah 26650, 40 °C, 80 % DoD around
   50 % SoC): fade per full-equivalent cycle was only weakly rate-dependent, and at 0.2 C the
   cycle contribution was so small that calendar aging dominated. S2 varied *discharge* rate only
   (charge fixed at 0.5 C) and found little LFP dependence. Transfer to a 280 Ah prismatic pack is
   an assumption. Stay at or below the 0.5 C standard charge (S3). A cold pack plus a high charge
   rate is the one combination that plates lithium (S9).

## 2. Findings with evidence

### 2.1 Calendar aging: where the pack rests

| finding | source · locator | state | mark | tags |
|---|---|---|---|---|
| Calendar capacity fade does not rise smoothly with SoC; it has plateaus 20–30 % wide. For LFP the step into the high-fade plateau is at about 70 % SoC (NCA/NMC: ~60 %). The transition follows the central graphite peak (≈50 % graphite lithiation), which sits at 73 % SoC in the LFP cell studied. | S1, Results ¶ on Fig. 2 and Fig. 5; Conclusions | inspected | direct | measured · direct · characterization (18650 cells, 9–10 months, 25/40/50 °C) |
| LFP calendar fade rate after 9 months ≈ 0.2 %-points/month at 25 °C, ≈ 0.5 at 50 °C (derivative stated by the authors). | S1, Results ¶ after Fig. 2 | inspected | direct | measured · author-derived slope · typical |
| For LFP, calendar aging "correlates entirely with the anode potential"; no extra fade toward 100 % SoC (unlike NMC at 100 %). Resistance rise of LFP was the lowest and "largely independent" of storage SoC. | S1, Results; Conclusions | inspected | direct | measured · direct |
| Sony/Murata US26650FTC1 (3 Ah): calendar fade ∝ √t, Arrhenius in T; fade "stronger with higher SOC" but flat from 37.5 % to 62.5 % SoC; storage at 0 °C and 10 °C showed "almost no aging". SoC factor fitted as cubic in (SoC − 0.5). | S7 §4.6.1.1 (p. 56–58), §5.1.2 (Eq. 5.1, 5.3, 5.12) | inspected (ch. 4.5–5.2) | direct | measured · fitted model · characterization (17 test points, 885 days) |
| Same cell stored at 50 % SoC, 25 °C lost ≈ 4.7 % capacity in 885 days. | S7 §4.6.1.2 | inspected | direct | measured · direct |
| Across many chemistries, LFP calendar fade follows t^0.5 over T and SoC; capacity-fade activation energy decreases with increasing SoC (temperature matters relatively less at high SoC, where fade is already high). | S11 Introduction ¶2; §"activation energies" (Fig. 4) | partial (intro, Ea section) | direct | review synthesis · fitted |
| Vendor: long-term storage 0–35 °C; ship/store at 30–50 % SoC; self-discharge ≤3 %/month at 25 °C, 30–50 % SoC. | S3 §3 rows 9, 11; §6 | inspected | direct | vendor spec · direct · guaranteed limit |

### 2.2 Cycle aging: the operating window

| finding | source · locator | state | mark | tags |
|---|---|---|---|---|
| LFP/graphite pouch cells, C/3, 25 % DoD windows at 40 °C and 55 °C: 75–100 % window fades fastest, 0–25 % slowest, 0–100 % in between; "average SOC was found to be the most critical factor … over temperature, depth of discharge, electrolyte salt or graphite". After 2500 h best cells 97 %, worst 76 %. | S8 Results (Fig. 2, 3); Conclusions | inspected (methods, results, discussion, conclusions) | direct | measured · direct · characterization; **conditions 40/55 °C only, 240 mAh lab pouch cells** |
| Mechanism: lithiated graphite reactivity rises with SoC even though graphite potential is nearly flat; at high SoC additive depletion → lithium alkoxides → Fe dissolution → Fe deposition on graphite → more LLI. | S8 Fig. 5, Fig. 7–8 discussion | inspected | direct | measured (microcalorimetry, XRF, ICP) |
| 1000 h voltage hold at 3.0 V or 3.65 V, 60 °C, interrupted by one C/3 cycle every 100 h: no Fe deposition above background — Fe dissolution needs *cycling*, not high-SoC *storage*. | S8 Methods "Voltage hold protocol"; Results ¶ on Fig. 4b | inspected | direct | measured · direct |
| Sony 26650 cycled at 40 °C, 1 C: pure cycle fade ∝ √(FEC); C-rate factor linear in C-rate; DoD factor cubic in (DoD − 0.6); the 25 °C vs 40 °C influence on pure cycle aging was small enough to be dropped from the model: 80 % DoD curves agree to ~8000 FEC, 100 % DoD curves diverge after ~4000 FEC. | S7 §4.6.2.1, §5.2.2.1–5.2.2.4, Table 4.6; S5 §3.1.2–3.1.4, Conclusions | inspected | direct | measured · fitted |
| Same study, SoC-range test points (20 % DoD around 25 / 50 / 75 % SoC, 1 C, 40 °C): the dissertation reads "cycles around SOC = 25 % lead to less aging than higher SOC-ranges" but "no clear relation" for the first 5000 FEC; the journal version concludes "cycling with DOCs = 20 % around SOC = 50 % lead to higher aging than lower and higher SOC-ranges". The C-rate "showed only small influence". | S7 §4.6.2.1 (Fig. 4.11); S5 Conclusions (Table 1 TP12/TP13) | inspected | direct | measured · direct |
| Same study: 0.2 C/0.2 C, 80 % DoD cells lost 14.5 % in 885 days, "dominated by calendar aging"; higher C-rates raise fade per *day* but lower it per *FEC*. | S7 §4.6.2.1, §4.6.2.2 | inspected | direct | measured · direct |
| Same study: shallow cycles (10–20 % DoD) around 50 % SoC showed the strongest early fade, then recovered and held flat to 7000 FEC; the journal version calls it "unexpectedly strong, but partly reversible capacity loss … with shallow cycles at medium states of charge", notes the effect "did not occur" in the dynamic load profiles with moving SoC ranges, and defers the mechanism to a follow-up. | S7 §4.6.2.1 (Fig. 4.10); S5 abstract, §3.1.3, Conclusions | inspected | direct | measured · direct |
| Sandia matrix, 18650 cells (LFP = A123 APR18650M1A 1.1 Ah), 0.5 C charge, discharge rate varied: LFP 80 %-EOL lifetimes of 2500–9000 EFC vs 250–1500 (NCA) and 200–2500 (NMC); most LFP cells had not reached 80 % at study end, so their figures are linear extrapolations of the then-current fade rate. | S2 Results ¶ on Fig. 1–2 and the extrapolation ¶ | inspected | direct | measured + extrapolated (authors') · range |
| Same: LFP capacity-fade rate *increased* with temperature in 15–35 °C (NMC decreased); fade increased with DoD for all chemistries, but SoC range had "little effect" on LFP % capacity at 200 EFC (Fig. 5f); discharge-rate dependence for LFP "appears low". | S2 "Temperature dependence", "Depth of discharge dependence", Fig. 5 ¶ | inspected | direct | measured · ANOVA at 200 EFC |
| Second-life review: cells with 2.8–3.6 V limits reached up to 9600 cycles; the review's summary advice is mid-range SoC, limited DoD via voltage cutoffs, moderate current, stable temperature. It relays (Cao et al.) that re-use is supported "if the working voltage range remains between 2.80 V and 3.55 V". | S12 abstract; §1.1 ¶ on Cao et al.; §1.3.7 summary | partial (intro, §1.3.7, abstract) | direct for S12's own words; the 2.8–3.55 V claim is second-hand (S16) | author assertion · review |
| Vendor cycle life: ≥6000 cycles to 80 % at 25 °C, 0.5 C/0.5 C, 2.5–3.65 V, 300 kgf clamp; ≥2500 at 45 °C. Recommended SoC scope 10–90 %. | S3 §5.1 rows 4–5; §3 row 6 | inspected (page images checked) | direct | vendor spec · guaranteed limit |

### 2.3 Float / constant-voltage holds

| finding | source · locator | state | mark | tags |
|---|---|---|---|---|
| Yi et al.: LFP/graphite, 1 C, 2.2–3.56 V, float vs cycling at 25–65 °C: cycling lost more capacity than float; float below 45 °C "relatively mild" (LLI only); above 55 °C float triggers electrolyte decomposition and SEI thickening; <65 % retention above 65 °C after 200 days. | S10 §4 ¶ on Yi et al.; Table 3 | S10 inspected (relevant sections); S15 second-hand via S10 | second-hand | measured (per review) · intermediary-extracted |
| Azzam et al.: A123 18650 LFP floated at 3.2–3.6 V, 5–50 °C: SEI current rises across the range; a cathode-lithiation current appears above ≈3.38 V (max ≈5 µA); review concludes LFP "should avoid sustained voltages above approximately 3.4 V". | S10 §3 ¶ on Azzam et al.; Table 2 | second-hand via S10 | second-hand | measured (per review) · intermediary-extracted |
| Review's own recommendation: LFP float window 3.35–3.45 V/cell, ~3.4 V "widely regarded as ideal"; lowering float 100–300 mV "has been shown to extend cycle life by a factor of two to five" (review cites [11,36], not inspected). | S10 §5.1 | inspected | direct for the review's words; the 2–5× figure is second-hand and unverified | author assertion |
| Takahashi & Shodai: prismatic cells with a Mn-substituted (7 % Mn) LFP cathode floated at **4.0 V**: 70 % after 24 months at 25 °C, 60 % after 1 month at 55 °C; anode degradation from Mn deposition. | S14 Results | inspected | direct | measured · direct; **not commensurable** (4.0 V, Mn-doped cathode) |

### 2.4 Temperature and rate

| finding | source · locator | state | mark | tags |
|---|---|---|---|---|
| Vendor: charge 0–55 °C, discharge −20–55 °C, standard 0.5 C, max continuous 1 C, pulse 2 C/30 s, standard CV 3.65 V with 0.05 C cutoff. | S3 §3 rows 4–8; §4.2 | inspected (page images) | direct | vendor spec · guaranteed |
| Commercial graphite/LFP cells, BEV profile, 1 C CC to 3.6 V then CV: cycle life at −18 °C < 10 % of room temperature (severe Li plating, graphite disordering); at 0 °C ≈ 90 % of room temperature. The cycle-life ratios are relayed by S9 from its ref. [50], where the cycling itself was published. "Charging at sub-zero temperatures should be avoided in all applications" and the post-mortem findings are S9's own. | S9 abstract; §2 procedure; §3.1 ¶ on cycle life | inspected (abstract, methods, cycle-life ¶, conclusions) | ratios second-hand (via S9 from its ref. [50]); post-mortem and warning direct | measured · direct (accepted manuscript) |
| Below ~25 °C the dominant cycle-aging mechanism in graphite cells shifts from SEI growth to Li plating; for LFP the literature tipping point is reported at 5–10 °C (Preger citing ref. 26, not inspected). | S2 "Temperature dependence" | inspected; the 5–10 °C figure is second-hand | second-hand | author assertion |

## 3. Application to this charger

Mapped onto `charger.conf` and the termination logic in `doc/LFP Charging.md`.

| lever | recommendation | evidence strength |
|---|---|---|
| `cv_eoc` | 3.65 V/cell is the vendor's standard CV target held to 0.05 C (S3). A 3.50–3.55 V cutoff with tail termination is an untested experiment: no same-cell comparison was inspected and the capacity it forgoes is not quantified. | none for a lower cutoff (untested); high for 3.65 V as the vendor target |
| `cv_float` | Keep as the zero-current end of the termination line only. No sustained hold at or near the EoC voltage after termination. A low float (~3.4 V) is neither supported nor refuted by inspected primaries. | high for "no EoC hold" (S1, S7, S8); low for anything about a low float |
| `tail_c_rate` | 0.05 matches the vendor's standard-charge cutoff (S3). A larger value terminates earlier and shortens the CV dwell at the top; how much capacity that forgoes was not measured in any inspected source. | medium for the vendor value; the "earlier is fine" part is inference |
| `recharge_dod` | Keep or raise (0.2–0.3): fewer full-charge events and less time at 100 %. It does **not** lower the average SoC: with 0.2 the pack cycles 80–100 %, the worst window in S8. | medium |
| partial-charge target (new) | The evidence-backed way to lower average SoC is to stop charging at an Ah-counted ceiling (e.g. 70–80 % of `bat_c` since the last full) and top to 100 % only on a periodic balancing schedule. The exact ceiling is cell-specific (S1: 57–73 % across cells; S3 recommends 10–90 %); it is not measured for the LF280K. | high for the direction (S1, S8); the number is a guess |
| balancing / full charge | Full charge is still required periodically for BMS balancing on LFP's flat curve; make it periodic (weekly to monthly) rather than daily. This is engineering inference; no primary on balancing cadence was inspected. | inference |
| `ibat_max` | ≤ 0.5 C (S3); typical solar rates ≤ 0.2 C are in the regime where, for the 26650 cell in S7, cycle aging was a small addition to calendar aging. Transfer to a 280 Ah prismatic cell is assumed. | medium |
| temperature | Block charging below 0 °C (S3, S9). Derate or stop above ~45 °C (S3 cycle life, S2 trend, S10 → Yi). Long-term storage 0–35 °C (S3). | high for 0 °C floor; medium for the 45 °C derate |
| storage | If the pack will idle for weeks, leave it at 30–50 % SoC (S3), cool (S1, S7: 0–10 °C storage showed almost no aging). | high |

The firmware's existing structure (absorption → termination → DoD-gated recharge, no float) is
consistent with the evidence as far as it goes, but it always charges to full, so the pack lives
in the top window regardless of `recharge_dod`. The change the evidence points at is a
partial-charge ceiling with periodic full charges. Independently of that, any path that re-tops
the pack daily when the load is small (the periodic re-sweep gating noted in the
`project_recharge_after_full_periodic_sweep` memory) works against the cells.

## 4. Conflicts and dependencies

- **Low-SoC vs high-SoC cycling.** Zsoldos (S8) found lower average SoC always better at C/3.
  Stroe et al. (S17, second-hand via S8) found the opposite at 4 C/4 C, 42 °C, 35 % DoD; S8
  attributes that to lithium plating at 12× higher current. The iScience zero-sum-pulse study
  (S13) also found the highest lithium-inventory loss at 30 % SoC, but its method is 4 C symmetric
  pulses, so it sits in the same high-rate regime and does not transfer to ≤0.5 C solar charging.
  For this application the low-rate result stands.
- **Shallow cycling at mid SoC.** Naumann (S5/S7) saw an unexpectedly strong early fade for 10–20 %
  DoD cycles around 50 % SoC that later recovered, and the journal conclusion ranks the 50 % SoC
  window *worse* than both 25 % and 75 % for 20 % DoD cycles. That is the one inspected result
  where a higher window did not age faster. It is partly reversible, absent under dynamic
  profiles, at 40 °C and 1 C, and its mechanism was left to a follow-up (S5). Preger (S2) saw
  little SoC-range effect on LFP at 200 EFC. Neither overturns the high-SoC penalty from S1 and
  S8, but the exact idle-SoC target is cell-specific and mid-SoC parking of a *cycling* pack is
  not automatically benign.
- **Dependencies.** S1 (Keil), S4/S5/S7 (Naumann), S18 (Spingler, not fetched) share the TUM
  group and, for S4/S5/S7, one data set; they count as one experimental line for the calendar-SoC
  plateau finding. S8 (Dahn group) is independent of TUM. S2 (Sandia) is independent of both. S10,
  S11, S12 are reviews and are not independent evidence of anything they relay.
- **Temperature range.** S8 tested only 40 °C and 55 °C; S1 25/40/50 °C; S7 0–60 °C storage but
  25/40 °C cycling. The room-temperature ordering of SoC windows is an extrapolation from S8
  supported by S1's 25 °C plateau data.

## 5. What would discriminate

- **3.55 V vs 3.65 V cutoff:** a same-cell, same-DoD comparison at ≤0.5 C and 25 °C with
  tail-current termination. If fade is equal, the voltage choice is free and only the SoC window
  matters. No inspected source does this.
- **Balancing cadence:** logs of cell-voltage spread on this pack versus days since last full
  charge would set how rarely a full charge is needed.
- **Room-temperature SoC-window ordering:** the S8 authors note unpublished hints that high-SoC
  cells "could recover in later cycles"; a longer (>2500 h) 25 °C repeat would settle whether the
  75–100 % penalty persists.

## 6. Provenance and search record

Retrieval date 2026-09-05. Browser: Playwright MCP (Chrome 152 UA, headed-capable), control
proven on `about:blank` by DOM read before the first fetch.

Archive: `/Users/fab/dev/pv/ee/lit/bat/` (authorized by the owner on 2026-09-05), index
`bat/INDEX.md`, manifest `bat/SHA256SUMS` (15 files, all verify, coverage checked by set
comparison). Owner-supplied licensed PDFs (S4, S5) are listed in `bat/.gitignore` and must not be
committed or redistributed. SHA-256 (first 7 hex) of archived files: S1 7f8bc24 · S2 0bf52eb ·
S3 1b7c026 (OCR companion 514dafe) · S4 e105623 · S5 fa954fa · S7 58dc5ef · S8 057f609 · S9
ee0d5de · S10 stub 001b13f · S11 stub e00deec · S12 9882b16 · S13 stub 909d135 + saved page
e01da0b · S14 8bbe7a3. Not archived: S6 (not obtained), S15–S18 (second-hand only).

Challenge searches (one per conclusion cluster, Serper web search, results used only for discovery):

1. *High SoC is worst* — "LiFePO4 graphite cycling low SOC window more capacity fade than high SOC
   window Stroe partial cycling": surfaced Stroe (via S8) and the iScience study (S13); both are
   high-rate regimes, handled in §4.
2. *Charge rate* — "LiFePO4 charge C-rate effect on cycle life 0.2C 0.5C 1C": surfaced only
   low-temperature (−10/−20 °C) rate studies and Wang 2011 (not fetched); no room-temperature
   refutation of the weak-rate-dependence finding.
3. *Float harmful* — "LiFePO4 float charging constant voltage hold aging study": surfaced S10
   (review), S14 (4.0 V Mn-LFP), vendor blogs (discarded). S10 → Yi shows float is milder than
   cycling below 45 °C, which bounds the claim: float is not catastrophic, it is unnecessary.
4. *Sub-zero charging* — "LiFePO4 low temperature charging lithium plating 0°C aging study":
   surfaced S9 (fetched) and Petzl 2015 (not fetched; not relied on).
5. *Cutoff voltage* — "LFP graphite cell charge cut-off voltage 3.65 V vs 3.5 V cycle life":
   surfaced S8 and S12 (with the Cao et al. 2.8–3.55 V relay). No direct 3.55-vs-3.65 primary
   found; the recommendation is marked as inference.

No absence claim is made in this document.

## 6a. Independent review

An adversarial Codex review (gpt-5.6-sol, reasoning xhigh, network and a headed browser over
CDP, 2026-09-05; transcript in `doc/reviews/2026-09-05-codex-lfp-longevity-research.md`)
returned twelve findings. All twelve were checked against the primaries and applied. The
substantive corrections, with the previous statement:

- **`recharge_dod` arithmetic (critical).** Previously: size `recharge_dod` 0.2–0.3 "so idle SoC
  sits ≤ ~70 %". Wrong: a 0.2 hysteresis releases recharge at 80 % SoC and refills to 100 %, so
  the pack lives 80–100 %. Replaced by the partial-charge-ceiling recommendation in §1 and §3.
- **`cv_eoc`.** Previously: "3.65 V is the datasheet ceiling, not a target" and a 3.50–3.55 V
  cutoff "is enough to reach full". The spec makes 3.65 V the standard CV target; the lower
  cutoff is now labelled an untested experiment.
- **Scope of "average SoC dominates".** Previously ranked above charge rate and 25→40 °C
  temperature; S8 varied neither. Now bounded to S8's own matrix.
- **~70 % boundary.** Now stated as cell-specific (57–73 % in S1) and unmeasured for the LF280K.
- **"Do not float at any voltage".** Narrowed to "no sustained hold at or near EoC"; a low float
  is marked as unevidenced either way.
- **S2 lifetimes** are authors' linear extrapolations, not observed endpoints. **S7 25 vs 40 °C**
  now carries its DoD boundary. **S9's** cycle-life ratios are marked second-hand via its ref.
  [50]. **S8's** hold protocol includes a C/3 cycle every 100 h. **S1's** "largely independent"
  qualifier restored. Vendor name and two DOIs corrected in §7.

Four items the reviewer could not verify remain marked second-hand here (Yi et al. details,
Azzam et al. values, the review's 2–5× float claim, the Cao et al. 2.8–3.55 V relay).

## 7. Source access log

| ID | work (stable identifier, version) | evidence state | attempt history | route | validation | load-bearing |
|---|---|---|---|---|---|---|
| S1 | Keil, Schuster, Wilhelm, Travi, Hauser, Karl, Jossen, "Calendar Aging of Lithium-Ion Batteries", J. Electrochem. Soc. 163(9) A1872 (2016), DOI 10.1149/2.0411609jes, version of record (open access) | inspected (abstract, experimental, results, conclusions) | curl PDF → Radware captcha HTML (200, 14 kB); browser navigate article HTML → OK, innerText saved; PDF later fetched with the browser session's cookies (%PDF, 10 pages, archived 7f8bc24) | raw text (DOM; PDF archived) | validated (live page) | yes |
| S2 | Preger et al., "Degradation of Commercial Lithium-Ion Cells as a Function of Chemistry and Cycling Conditions", J. Electrochem. Soc. 167 120532 (2020), DOI 10.1149/1945-7111/abae37; manifestation used: accepted manuscript SAND2020-8433J, OSTI 1650174 | inspected (methods, temperature/DoD/rate sections, Fig. 5 discussion, conclusions) | curl IOP PDF → Radware captcha; curl https://www.osti.gov/servlets/purl/1650174 → %PDF 8.5 MB; pdftotext coherent | raw text (text-layer PDF) | validated (text coherent; title/authors match IOP landing) | yes |
| S3 | EVE Power Co., Ltd (title page; §1 says "EVE Energy Co., Ltd."), "LF280K (3.2V 280Ah) Product Specification", Version B, effective 2021-03-23; mirror https://www.e-pohon.cz/files/products_files/l/LF280K_%283_2V_280Ah%29_Product_Specification%28_Version_B_%29.pdf | inspected (§3, §4.2, §5.1, §6) | curl → %PDF 536 kB, image-only scan; ocrmypdf → text; pages 1–4 rendered and read visually | OCR / rendered-page vision | validated (every cited value checked against page images) | yes |
| S4 | Naumann, Schimpe, Keil, Hesse, Jossen, "Analysis and modeling of calendar aging of a commercial LiFePO4/graphite cell", J. Energy Storage 17, 153–169 (2018), DOI 10.1016/j.est.2018.01.019 | inspected (§3.1 SoC result, conclusions; identity page) — journal version of the S7 calendar study | sciencedirect abstract page via browser → OK; full text paywalled (not attempted: paywall boundary); Kempten OPUS record → Anubis PoW page (curl); **owner supplied the publisher PDF** (17 pages, title/author match, archived e105623) | raw text (text-layer PDF) | validated | yes, jointly with S7 |
| S5 | Naumann, Spingler, Jossen, "Analysis and modeling of cycle aging of a commercial LiFePO4/graphite cell", J. Power Sources 451, 227666 (2020), DOI 10.1016/j.jpowsour.2019.227666 | inspected (abstract, §2 test points, §3.1.2–3.1.4, conclusions) — journal version of the S7 cycle study | TUM portal page (curl, no file); sciencedirect abstract page via browser → OK; full text paywalled (not attempted: paywall boundary); **owner supplied the publisher PDF** (12 pages, title/author match, archived fa954fa) | raw text (text-layer PDF) | validated | yes, jointly with S7 |
| S6 | Grolleau et al., "Calendar aging of commercial graphite/LiFePO4 cell – Predicting capacity fade under time dependent storage conditions", J. Power Sources 255 (2014), HAL hal-01002804 | de-scoped: no claim depends on it (S1 and S7 cover LFP calendar aging) | curl → Anubis PoW page; browser → PoW page; browser session cookies + curl → landing page OK, but HAL holds no file, only an ISTEX link; ISTEX → Shibboleth institutional login (not attempted: authentication boundary) | route: none | n/a | no |
| S7 | Naumann, "Techno-economic evaluation of stationary battery energy storage systems with special consideration of aging", Dissertation, TU München (2018), mediaTUM 1434981, https://mediatum.ub.tum.de/doc/1434981/1434981.pdf | inspected (§4.1 cell, §4.5 test matrices, §4.6 results, §5.1–5.2 models) | curl → Anubis PoW page (twice); browser → PoW solved by page JS; download via browser-session cookies → %PDF 8.6 MB, 156 pages; pdftotext coherent | raw text (text-layer PDF) | validated (text coherent; title/author from pdfinfo match mediaTUM record) | yes |
| S8 | Zsoldos et al., "The Operation Window of Lithium Iron Phosphate/Graphite Cells Affects their Lifetime", J. Electrochem. Soc. 171 080527 (2024), DOI 10.1149/1945-7111/ad6cbd, version of record (open access) | inspected (methods, results, discussion, conclusions) | browser navigate article HTML → OK, innerText saved; a forum-hosted PDF mirror was fetched (2.8 MB, %PDF) and discarded; IOP PDF later fetched with the browser session's cookies (15 pages, archived 057f609) | raw text (DOM; PDF archived) | validated (live page) | yes |
| S9 | Rauhala, Jalkanen, Romann, Lust, Omar, Kallio, "Low-temperature aging mechanisms of commercial graphite/LiFePO4 cells cycled with a simulated electric vehicle load profile — A post-mortem study", J. Energy Storage 20, 344–356 (2018); manifestation: peer-reviewed accepted manuscript, Aalto University repository https://aaltodoc.aalto.fi/bitstreams/6bfee248-5315-4f7e-893a-f3b5dbe6a95f/download; DOI 10.1016/j.est.2018.10.007 (resolved 2026-09-05 to PII S2352152X18303694, the sciencedirect record of this title) | inspected (abstract, procedure, cycle-life paragraph, conclusions) | curl → %PDF 1.6 MB; pdftotext coherent | raw text | validated | yes (0 °C floor, with S3) |
| S10 | Khan et al., "A review of float charging in lithium-ion batteries: Degradation mechanisms, influencing factors, and optimization strategies", J. Power Sources (2026), PII S0378775326005380 (open access) | inspected (§3 Azzam ¶, §4 Yi ¶, Tables 2–3, §5.1) | curl → 403 ScienceDirect shell; browser navigate → full text OK, innerText saved | raw text (DOM) | validated | no (supporting; its relayed primaries are S15, S16) |
| S11 | Lam, Cui, Stroebl, Uppaluri, Onori, Chueh, "A decade of insights: Delving into calendar aging trends and implications", Joule 9(1) 101796 (2025), DOI 10.1016/j.joule.2024.11.013 (resolved 2026-09-05 to PII S2542435124005105), open access | partial (introduction ¶2, activation-energy section) | curl → Cloudflare "Just a moment" (403); browser navigate → full text OK; showPdf endpoint → 403 even with browser cookies; archived as a stub | raw text (DOM) | validated | no |
| S12 | Aeppli, Hack, Held, "Aging behavior of LiFePO4-based battery cells at stack level: A Second-Life cycling study", J. Energy Storage 129, 117135 (2025), published version, Empa DORA empa:41733 | partial (abstract, §1.1, §1.3.7) | curl → %PDF 5.1 MB; pdftotext coherent | raw text | validated | no |
| S13 | Kang, Yang, Wang et al., "Study of aging mechanisms in LiFePO4 batteries with various SOC levels using the zero-sum pulse method", iScience 27(7) 110287 (2024), DOI 10.1016/j.isci.2024.110287, PMC11292501 | partial (abstract, method, conclusions) | curl PMC → HTML OK | raw text (HTML) | validated | no (conflict entry only) |
| S14 | Takahashi, Shodai, "Float Charging Performance of Lithium Ion Batteries with LiFePO4 Cathode", Electrochemistry 78(5) 342–344 (2010), J-STAGE https://www.jstage.jst.go.jp/article/electrochemistry/78/5/78_5_342/_article | inspected (results) | curl J-STAGE PDF → %PDF 489 kB; landing title verified | raw text | validated | no (not commensurable) |
| S15 | Yi et al., "The difference in aging behaviors and mechanisms between floating charge and cycling of LiFePO4/graphite batteries", Ionics 25, 2139 (2019), DOI 10.1007/s11581-018-2607-2 (landing page title and metadata verified 2026-09-05) | second-hand via S10 (abstract-level only) | Springer landing page (curl) → title/metadata OK; full text paywalled (not attempted: paywall boundary) | raw text (landing metadata only) | validated (metadata) | no |
| S16 | Cao et al., LFP cells over 10 000 cycles within 2.8–3.6 V (identifier unverified — cited as ref. [8] in S12) | second-hand via S12 | not attempted | route: none | n/a | no |
| S17 | Stroe et al., LFP partial cycling at 4 C (refs. 26–27 in S8; identifier unverified) | second-hand via S8 | not attempted | route: none | n/a | no |
| S18 | Azzam, Sauer, Endisch, Lewerenz, LFP/NCA float-current deconvolution (ref. [27] in S10; identifier unverified) | second-hand via S10 | not attempted | route: none | n/a | no |

Rows S15–S18 are the non-clean rows: their content reached this document through a review,
not through the primary. S4 and S5 were upgraded from abstract-only to inspected after the owner
supplied the publisher PDFs; inspecting S5 changed one finding: the dissertation's reading that
cycles around 25 % SoC aged least was replaced by the journal conclusion that the 50 % SoC window
aged most for 20 % DoD cycles (§2.2, §4).
