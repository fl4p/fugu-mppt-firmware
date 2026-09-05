*this document is an LLM generated placeholder*

# Codex review — doc/LFP Longevity Research.md (2026-09-05)

Reviewer: codex-cli 0.153.2, model gpt-5.6-sol, reasoning effort xhigh, sandbox workspace-write with network, Playwright MCP attached over CDP to a headed scratch-profile Chrome 152 on 127.0.0.1:9411. Prompt: scratchpad review_prompt.md (source-fidelity and entailment attack on the load-bearing rows). Log grep for `no connected browsers` / `node_repl/js (failed)`: only the prompt's own text matched; the ACCESS section shows the IOP title fetched through the Playwright MCP browser. All findings were re-verified against the primaries before being applied; see §6a of the reviewed document.

---

## ACCESS

Live CDP query: `Browser = Chrome/152.0.7977.77`; Playwright MCP returned `document.title = "Calendar Aging of Lithium-Ion Batteries: I. Impact of the Graphite Anode on Capacity Fade - IOPscience"`.

## FINDINGS

1. **(c) `recharge_dod` does not produce the claimed idle SoC — Critical.**  
   Doc: [lines 26–29](doc/LFP Longevity Research.md:26), [line 108](doc/LFP Longevity Research.md:108). A recharge DoD of 0.2 starts recharge at 80% SoC; 0.3 starts it at 70%. Following a full charge, the pack therefore spends almost the entire discharge interval above 70%, then returns to 100%; its approximate mean is 90% or 85%, not ≤70%. Moreover, [Naumann S7 §4.6.2.1, pp. 63–65](https://mediatum.ub.tum.de/doc/1434981/1434981.pdf) reports no clear early monotonic DoD relationship and higher final degradation rates for larger DoD. The evidence supports lowering the charge target or making full charges exceptional; it does not support this threshold arithmetic.

2. **(c) The `cv_eoc` recommendation misstates the EVE specification and asserts unmeasured capacity retention — High.**  
   Doc: [lines 33–38](doc/LFP Longevity Research.md:33), [lines 105–107](doc/LFP Longevity Research.md:105). In [EVE LF280K Version B §3 and §4.2](https://www.e-pohon.cz/files/products_files/l/LF280K_%283_2V_280Ah%29_Product_Specification%28_Version_B_%29.pdf), 3.65 V is the standard constant-voltage charge target, held until 0.05 C—not merely a “ceiling, not a target.” Nothing inspected establishes that 3.50–3.55 V “is enough to reach full” or that earlier tail termination costs “tiny” energy. A flat discharge-voltage curve does not establish capacity versus charge cutoff. A lower cutoff may still be a defensible longevity experiment, but not as a sourced result.

3. **(c) “Average SoC dominates charge rate and 25→40 °C” exceeds the experiment — High.**  
   Doc: [lines 15–20](doc/LFP Longevity Research.md:15), [lines 42–46](doc/LFP Longevity Research.md:42). [Zsoldos S8, Experimental and Conclusions](https://iopscience.iop.org/article/10.1149/1945-7111/ad6cbd) ranks SoC over temperature and DoD only within its 240 mAh laboratory-pouch, C/3, 40/55 °C matrix; it did not vary charge rate or test 25 °C. S7’s 25/40 °C observation concerns extracted *pure-cycle* aging after subtracting calendar aging, under particular test points. These sources cannot rank average SoC above total 25→40 °C aging or above charge rate generally.

4. **(c) The ~70% graphite-stage boundary is not transferable to the LF280K as a numerical control point — High.**  
   Doc: [lines 17–20](doc/LFP Longevity Research.md:17), [lines 26–28](doc/LFP Longevity Research.md:26), [line 108](doc/LFP Longevity Research.md:108). [Keil S1, Fig. 5 and Conclusions](https://iopscience.iop.org/article/10.1149/2.0411609jes) found 73% in an A123 1.1 Ah 18650 and explains that graphite oversizing/cell design moves the staging feature. [Preger S2, Conclusions](https://www.osti.gov/servlets/purl/1650174) explicitly warns against extrapolating lifetime behavior between manufacturers—even for similar chemistry and format. The line-108 caveat acknowledges cell specificity but then still uses 70% as a high-confidence LF280K threshold.

5. **(c) “Do not hold at any voltage” is stronger than the float evidence — Medium.**  
   Doc: [lines 21–25](doc/LFP Longevity Research.md:21), [line 106](doc/LFP Longevity Research.md:106). S1 and S7 study storage SoC, not controlled low-voltage float; S8’s hold experiment addresses Fe dissolution, not comparative capacity fade. EVE’s ≤3%/month figure is conditioned on 25 °C and 30–50% SoC and does not prove that voltage regulation “gains nothing.” The opened [Yi primary landing/abstract](https://link.springer.com/article/10.1007/s11581-018-2607-2) reports less loss under floating than cycling. The defensible conclusion is “avoid sustained EOC/high-SoC float”; the categorical ban at *any* voltage is unsupported and conflicts with the document’s own low-voltage-float evidence row.

6. **(a) S2 did not observe all LFP cells reaching 80% at 2500–9000 EFC — Medium.**  
   Doc: [line 77](doc/LFP Longevity Research.md:77). [Preger S2, Results—General Analysis, Fig. 2 and following extrapolation paragraph](https://www.osti.gov/servlets/purl/1650174) says most LFP cells had not reached 80% capacity; their 80%-EOL lifetimes were extrapolated from the then-current linear degradation rate. “Reached 80% at 2500–9000 EFC” incorrectly converts projected endpoints into observed endpoints.

7. **(a) The S7 25-versus-40 °C claim omits the DoD boundary — Medium.**  
   Doc: [lines 44–46](doc/LFP Longevity Research.md:44), [line 73](doc/LFP Longevity Research.md:73). [S7 §5.2.2.4, p. 82](https://mediatum.ub.tum.de/doc/1434981/1434981.pdf) shows good 25/40 °C agreement to about 8000 FEC for the 80% DoD test points; the 100% DoD curves begin diverging after roughly 4000 FEC. The source calls temperature influence on extracted cycle aging *small*, not unconditionally absent.

8. **(b) The weak-charge-rate conclusion assumes discharge-rate and small-cell results transfer — Medium.**  
   Doc: [lines 47–49](doc/LFP Longevity Research.md:47), [line 78](doc/LFP Longevity Research.md:78), [line 110](doc/LFP Longevity Research.md:110). S2 held charge at 0.5 C and varied **discharge** rate, so it provides no charge-rate comparison. S7 varied charge/discharge rate on 3 Ah 26650 cells, chiefly at 40 °C, 50% mean SoC, and 80% DoD. Assuming this weak dependence holds for a 280 Ah prismatic pack at solar temperatures is plausible but unstated. The ≤0.5 C limit itself survives because it is EVE’s standard rate.

9. **(a) S9’s cycle-life numbers are incorrectly marked primary/direct, and its DOI is omitted — Medium.**  
   Doc: [line 95](doc/LFP Longevity Research.md:95), [line 198](doc/LFP Longevity Research.md:198). In [Rauhala S9 §3.1, p. 9/Table 2](https://aaltodoc.aalto.fi/bitstreams/6bfee248-5315-4f7e-893a-f3b5dbe6a95f/download), the −18 °C and 0 °C cycle-life figures are expressly attributed to reference [50], where the detailed cycling was published. Their transcription is correct, but their evidence mark should be second-hand; the sub-zero-charging warning is S9’s direct conclusion. The named work has DOI [10.1016/j.est.2018.10.007](https://doi.org/10.1016/j.est.2018.10.007).

10. **(a) The S8 “1000 h hold” omits periodic cycles — Low.**  
    Doc: [lines 23–24](doc/LFP Longevity Research.md:23), [line 72](doc/LFP Longevity Research.md:72). [S8 Experimental protocol and Fig. 4b discussion](https://iopscience.iop.org/article/10.1149/1945-7111/ad6cbd) held cells at 3.0 or 3.65 V and 60 °C for 1000 h, but performed a C/3 cycle every 100 h. The no-Fe result and authors’ mechanistic conclusion survive; calling it an uninterrupted voltage hold does not.

11. **(a) S1’s resistance result loses a source qualifier — Low.**  
    Doc: [line 60](doc/LFP Longevity Research.md:60). [S1 Conclusions](https://iopscience.iop.org/article/10.1149/2.0411609jes) says LFP resistance increase was lowest and **largely** independent of storage SoC. The document strengthens this to “SoC-independent.”

12. **(a) The metadata log contains minor copied-field omissions — Low.**  
    Doc: [line 192](doc/LFP Longevity Research.md:192), [line 200](doc/LFP Longevity Research.md:200). The LF280K Version B title page names **EVE Power Co., Ltd.**, not “EVE Energy.” S11’s otherwise-correct Joule metadata omits the resolvable DOI [10.1016/j.joule.2024.11.013](https://doi.org/10.1016/j.joule.2024.11.013).

## SURVIVED

- **S1:** The 20–30%-wide plateaus, transition above about 70% for the tested LFP cell, 73% central graphite peak, and approximately 0.2/0.5 percentage-points-per-month slopes at 25/50 °C all match Fig. 2/Fig. 5 and the Results text.
- **S1:** “Correlates entirely with the anode potential” and the absence of an additional 100%-SoC fade rise are faithful.
- **S2:** The fixed 0.5 C charge, 15–35 °C LFP temperature trend, increasing fade with DoD, little SoC-range effect at 200 EFC, and low apparent **discharge-rate** dependence are correctly bounded.
- **S3:** ≥6000 cycles at 25 °C, ≥2500 at 45 °C, 10–90% recommended SoC, 0–55 °C charge, −20–55 °C discharge, ≤3%/month self-discharge, clamp force, rates, and EOL conditions all match Version B.
- **S7:** The 37.5–62.5% calendar-aging plateau, ≈4.7% loss in 885 days, ≈14.5% loss for the low-rate cycle test, and the rate-per-day versus rate-per-FEC statements are faithful.
- **S7:** The `(SoC − 0.5)^3` and `(DoD − 0.6)^3` factors exist as empirical fitted model terms; the document correctly labels them fitted, though they should not be treated as universal laws.
- **S5/S7:** The shallow mid-SoC anomaly, partial recovery, lack of the anomaly in dynamic profiles, and deferred mechanism are faithfully reported.
- **S8:** The window ordering, 97% best/76% worst capacity after 2500 h, and “average SOC … most critical” quotation are correct within the stated 240 mAh, C/3, 40/55 °C matrix.
- **S9:** The `<10%` at −18 °C and `≈90%` at 0 °C figures are transcribed accurately, despite the direct/second-hand classification error.
- **Temperature controls:** Blocking LF280K charging below 0 °C and respecting its 0–55 °C charge range are directly supported by the pack-specific vendor specification.
- **Counter-evidence:** The opened [Stroe dissertation](https://vbn.aau.dk/ws/portalfiles/portal/549543532/Lifetime_Models_for_Lithium_ion_Batteries_used_in_Virtual_Power_Plant_Applications.pdf), §5.3 and §7.2.4, does find worse cycle fade at decreasing mean SoC—but at 4 C, 42.5 °C, 35% cycle depth, with EOL extrapolation. The document already discloses and reasonably scopes this high-rate conflict.
- **Metadata:** S1, S2, and S8 DOIs resolve to the named works; their author lists, journals, volumes, years, and article identifiers are correct. S7’s title, author, institution, year, and mediaTUM identifier also match.
- **Evidence discipline:** The balancing cadence is explicitly marked engineering inference rather than presented as a measured result.

## UNVERIFIED

- S10/S15’s detailed Yi conditions and thresholds—“relatively mild” below 45 °C, mechanisms above 55 °C, and `<65%` retention at 65 °C/200 days—remain second-hand. The primary landing page and abstract opened, but full text required subscription access.
- S10/S18’s Azzam values—≈3.38 V onset, ≈5 µA maximum, and the derived 3.4 V sustained-voltage recommendation—were not checked against the primary.
- S10’s claimed 2–5× lifetime improvement from reducing float voltage was not checked against its cited primaries.
- S12/S16’s relayed 2.8–3.55 V second-life claim was not checked against Cao et al.’s primary.
- No required load-bearing value from S1, S2, S3, S7, or S8 remained inaccessible.

## END

=== codex exit 0 ===
