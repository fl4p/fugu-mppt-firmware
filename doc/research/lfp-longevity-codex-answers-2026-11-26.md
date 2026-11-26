*this document is an LLM generated placeholder*
# ACCESS

Checked on 2026-11-26. Playwright scratch Chrome fetched the MDPI page with `<title>` **Second-Life Assessment of Commercial LiFePO4 Batteries Retired from EVs**. Browser control was proved first by navigating to `about:blank` and reading its DOM.

| Source | Access and inspection |
|---|---|
| Cao [12] | `www.mdpi.com`: complete publisher HTML and tables through Playwright. |
| Petzl [21] | `sciencedirect.com`: scratch Chrome encountered a human check; after the requested file-based grant, fabchrome cleared the checkbox and retrieved the publisher abstract/preview, but full text required subscription access. `eref.uni-bayreuth.de` supplied metadata only. The complete nine-page article, pp. 799–807, was subsequently read in the `studocu.com` document viewer through scratch Playwright. Relevant rendered pages were checked against their text layer; the separate plain-text preview corrupts decimal digits and was not used for numbers. |
| Bretscher [13] | `nordkyndesign.com`: complete article HTML through scratch Playwright. |
| EVE [8] | `e-pohon.cz`: the research record's linked Version B PDF, retrieved with Chrome-UA curl; 11 PDF pages. Specification pages 1–2 were inspected as images. |
| Rauhala [20] | `aaltodoc.aalto.fi`: accepted-manuscript PDF. Curl encountered Anubis; browser navigation cleared it, and an in-page fetch through scratch Playwright retrieved the PDF. Table 2 was checked visually. Fabchrome was also tried following the scratch challenge. |
| Additional manufacturer sources for Q3 | `analog.com`: complete application-note HTML through scratch Playwright. `buya123products.com`: A123 guide, 66-page PDF, retrieved by in-page fetch through granted fabchrome after curl/scratch challenges; title and p. 29 checked visually. The alternative `formula-hybrid.org` link returned an actual Not Found page. `ti.com`: SLUAAR1 PDF through Chrome-UA curl; §2 inspected. |

All fabchrome tabs opened for this run were closed, and `want-fabchrome` was removed. No connect-URL token sighting. No question below rests solely on the unpublished research record.

## 1. Cao: cell identity, stages and inconsistent numbers

**Tags: (a) missing cell identification; (c) overbroad temperature description; (d) the numerical-discrepancy warning and qualified voltage observation survive.**

Section 2.1 explicitly says: **“Cells 01 and 02 were new.”** Cells **03 and 04** were retired from electric forklifts after four years. The four retired BYD bus modules belong to the separate module experiment (§2.2). The cell experiment used CALB cells labelled 100 Ah; the authors adopted **107 Ah as nominal capacity** from the new-cell measurements. Cell 02 rested for 1.5 years before cycling. All three numerical accounts in the question concern **new Cell 01**. [Cao et al., §§2.1–2.2, 3.1.1, Table 4 and §5](https://doi.org/10.3390/batteries10090306).

The stage list is substantially correct:

- **All four cells initially:** 3.10–3.45 V, 60 A charge and 95–100 A discharge (§2.1).
- **Later stages:** examples include 2.80–3.45 V and 2.80–3.60 V, with currents also changed (§2.1).
- **Cell 01 specifically:** 2.6 V discharge cutoff during cycles 5100–5700 and 2.5 V during cycles 8400–8760 (§3.1.1, discussion of Fig. 4a).
- **Temperature exception:** Cells 01, 02 and 04 stayed at 23 °C; Cell 03 also underwent elevated-temperature testing, with temperature raised from 45 to 55 °C during cycles 9680–10,160 (§3.1.1). Thus the page's opening sentence should not describe the entire experiment as being at 23 °C.

The figures are **not fully consistent**, but they are not three independent, irreconcilable results:

| Account for Cell 01 | What the arithmetic establishes |
|---|---|
| §3.1.1: 33.3% loss, 10,390 cycles, approximately 8000 FEC, 3.275% per 1000 cycles | 33.3/10.390 = **3.205**, not 3.275. |
| §5, conclusion 1: 33.9%, 10,000 cycles, 3.26% per 1000 cycles | 33.9/10 = **3.39**. Using 10,390 cycles instead gives **3.263**, explaining the approximate rate without proving the intended cycle count. |
| Table 4: 107.1 → 70.69 Ah, 10,390 cycles, 3.275% per 1000 cycles | Loss is **33.996% of the initial 107.1 Ah**. Normalizing the 36.41 Ah loss to the paper's **107 Ah nominal capacity** gives **3.2751 percentage points per 1000 cycles**: the table's rate is reconcilable. |
| Table 4 also labels the final capacity 66.7% | 70.69 Ah is **66.004% of 107.1 Ah**, or **66.065% of 107 Ah**, not 66.7%. |

Moreover, treating 107 Ah as the baseline gives a final deficit of **33.935%**, consistent with the conclusion's rounded 33.9%. The substantial discrepancy is therefore **33.3%/66.7% versus the Ah endpoint**; the 33.9% versus approximately 34% difference can reflect normalization, and 10,000 may be a coarse summary of 10,390. These are calculations from printed data, not corrections authorized by the authors.

The voltage statement survives **as the authors' observation within this staged test**. Section 3.1.1 says that increasing the charge cutoff within 3.60 V **“does not contribute to an accelerated aging speed.”** The page already correctly warns that this was not a controlled comparison. It does not establish equivalence between 3.50 and 3.65 V, nor a general absence of voltage effects. The second-life recommendation is indeed in **§3.3 and §5, conclusion 3**; Table 1 gives the manufacturer's 10–90% SoC window.

## 2. Petzl: the section numbers are correct

**Tags: (d) both questioned locators and the main findings survive; (b) avoid generalizing the measured lifetime penalty to all temperatures below 0 °C.**

The paper's actual structure is:

| Item | Correct locator |
|---|---|
| 2.5 Ah cylindrical 26650 graphite/LFP cell; 2.0–3.6 V limits | **§2.1, p. 800** |
| −22 °C aging; 1 C or C/2 charge; full or 80% SoC | **§2.2, p. 800**, headed **“Electrochemical aging tests”** |
| Capacity loss and cycle life | **§3.1, p. 801; Fig. 1** |
| Loss of cyclable lithium and shifted electrode operating windows | **§§3.2.1–3.2.2, pp. 801–803; Figs. 3–4** |
| Ohmic resistance and electrolyte consumption | **§3.3, p. 803; Fig. 5** |
| Partial reversibility during subsequent 25 °C cycling | **§3.4, pp. 803–804; Fig. 6** |
| Summary | **Abstract, p. 799; §4, pp. 806–807** |

Thus **“Protocol: §2.2” is right**, and **“Abstract and §3” is right but broad**. Do not renumber them as a correction. The cell specification additionally needs §2.1. The full-text manifestation is the [Petzl article in the Studocu viewer](https://www.studocu.com/de/document/technische-universitat-munchen/applied-electrochemistry/lithium-plating-in-a-commercial-lithium-ion-battery-a-low-temperature-aging-study/29103595), checked against the [publisher's article identity and abstract](https://doi.org/10.1016/j.jpowsour.2014.11.065).

Protocol detail: full charging used CC followed by CV until current fell below C/20; the partial-charge case stopped at 80% of full capacity. Discharge was C/2, and characterization occurred at 25 °C every 20 cycles (§2.2). The reported combinations are **1 C/full, 1 C/80%, and C/2/80%** (Fig. 1; Table 1), not a complete two-by-two matrix.

The abstract explicitly describes **“loss of cyclable lithium”**, **“partly reversible”** capacity fade and **“electrolyte consumption”**. The detailed sections above support the page's mechanism summary. For precision, describe the aging rate as initially rising to a maximum and then declining (§3.1), rather than implying it was greatest at the very first cycle.

**[21] supports more than plating alone.** Section 3.1 explicitly reports **“a lifetime of around 90–140 cycles”**, with 80% capacity retention as the endpoint. It compares this −22 °C result with more than 2000 cycles at 45 °C in a different study, its reference [16]. That is a severe lifetime penalty at the tested conditions, not a matched temperature comparison within Petzl's experiment.

Rauhala [20], Table 2, reports **185 equivalent cycles at −18 °C versus 2071 at room temperature**, with **1850 at 0 °C**, reproducing Omar et al.'s data. Therefore line 527 is defensible as a shorthand for severe losses observed at particular subzero temperatures; it is too broad if read as a universal threshold claim. [Rauhala et al., §3.1 and Table 2](https://doi.org/10.1016/j.est.2018.10.007).

## 3. Uncited end-of-charge explanation

**Tag: (b) prose stronger than the established source support.**

I checked the complete reference list [1]–[23]. **[13] is the identifiable support for a voltage/current termination model**, but not for the sentence as an unqualified electrochemical rule. Under **Charge Termination Condition**, Bretscher describes **“a pair of values for cell voltage and residual charging current”** and interpolates between the manufacturer's termination pair and an assumed fully charged resting voltage. That supports an explicitly attributed description of his model. It does not independently demonstrate that a short upper charge-curve region makes the principal effect of reducing voltage a change in the current defining full charge. [Bretscher, Charge Termination Condition, opening paragraph and following table](https://nordkyndesign.com/charging-marine-lithium-battery-banks/).

A manufacturer source supports a narrower statement: **EVE [8], §4.2, printed p. 2**, specifies 3.65 V and **“0.05C cutoff.”** It does not specify an equivalent termination-current rule at lower voltages. The [A123 Systems guide, document 493005-001E, February 2013, pp. 28–29](https://www.buya123products.com/uploads/vipcase/c9dec23d34292dd9e30058723737edee.pdf) likewise describes CC/CV charging, with the CV hold continuing **“until the current decays to near-zero”**; it does not establish the questioned lower-voltage equivalence.

Additional primary-source checks—the [Analog Devices application note](https://www.analog.com/en/resources/design-notes/charging-lithium-batteries-with-a-nonstandard-termination-voltage.html) and [TI SLUAAR1, §2](https://www.ti.com/lit/pdf/sluaar1)—did not supply that equivalence either. Searches covered LFP cutoff voltage, termination current, capacity and A123/TI charging guidance. **No verified primary citation for the exact causal claim was established in this check.** This is not an exhaustive assertion that no such publication exists. Remove it or use the narrower replacement below; simply appending [13] would conceal the distinction between an attributed model and demonstrated cell behavior.

# FINDINGS

1. **Lines 291–303 — (a), (c), with (d) for the discrepancy warning.** Replace with:

   > Cao et al. cycled four CALB prismatic LFP cells labelled 100 Ah, using 107 Ah as nominal capacity: Cells 01 and 02 were new, while Cells 03 and 04 had retired from electric forklifts after four years. Cells 01, 02 and 04 were tested at 23 °C; Cell 03 also underwent elevated-temperature testing [12, §§2.1 and 3.1.1]. All four initially used 3.10–3.45 V, 60 A charging and 95–100 A discharging. Later stages changed both voltage limits and currents, including 2.80–3.45 V and 2.80–3.60 V windows. Cell 01 also had discharge cutoffs of 2.6 V during cycles 5100–5700 and 2.5 V during cycles 8400–8760 [12, §2.1; §3.1.1, discussion of Fig. 4a].
   >
   > The capacity summaries for initially new Cell 01 are not fully consistent. Section 3.1.1 reports 33.3% loss after 10,390 cycles (about 8000 FEC), whereas conclusion 1 reports 33.9% after 10,000 cycles. Table 4 lists 107.1 Ah initially and 70.69 Ah finally after 10,390 cycles: approximately 34.0% loss, rather than the 33.3% implied by its stated 66.7% retention. The reported 3.275% per 1000 cycles agrees with the table's Ah loss normalized to 107 Ah nominal capacity; the conclusion's 3.26% agrees approximately with 33.9% over 10,390 cycles, rather than 10,000. These differences should not be silently reconciled into a single exact result [12, §3.1.1, Table 4 and §5, conclusion 1].

2. **Lines 305–309 — (d).** The staged-test caveat and attributed voltage observation are correct. No substantive replacement needed. Make the recommendation locator more precise: **“[12, §3.3 and §5, conclusion 3]”.**

3. **Lines 424–428 and 685 — (d); optional precision edits, not locator-error repairs.** Use **“[21, §§2.1–2.2, p. 800]”** after the cell/protocol sentence. Replace the findings sentences with:

   > Plating caused loss of cyclable lithium and shifted the electrode balance, eventually reducing further plating. The aging rate initially rose to a maximum and then declined. Part of the capacity loss was recovered during subsequent cycling at 25 °C; the authors attributed increased ohmic resistance to electrolyte consumption during film formation on plated lithium [21, Abstract; §§3.1–3.4, pp. 801–804].

   The reference note may remain **“Protocol: §2.2.”** An expanded note would be **“Cell and equipment: §2.1; aging protocol: §2.2 (p. 800); findings: §§3.1–3.4.”**

4. **Line 527 — (b) for unbounded temperature wording; (d) for severe lifetime loss at the tested conditions.** Replace the evidence cell with:

   > The manufacturer's lower charge limit [8, §3, row 7]. Severe cycle-life loss was reported at −18 °C [20, §3.1 and Table 2, reproducing Omar et al.] and at −22 °C under Petzl et al.'s tested charging conditions [21, §§2.2 and 3.1]. Acts only with fresh BMS temperature reports.

5. **Lines 311–312 — (b).** Replace with:

   > The manufacturer's standard-charge specification combines a voltage target with a current cutoff: 3.65 V and 0.05 C for the LF280K [8, §4.2]. It does not establish an equivalent termination condition at a lower voltage.

   Alternatively, delete these two lines; the following paragraph already introduces [13]'s practitioner interpolation with appropriate attribution.

# SURVIVED

- **Cao:** Cell 01 really was new; the stated stages and numerical quotations are present in the paper. Its capacity summaries contain a real inconsistency, although nominal-capacity normalization reconciles the 3.275% rate with Table 4. The qualified observation about charge cutoffs up to 3.60 V is faithfully reported.
- **Petzl:** **§2.2 is the protocol; §3 contains the findings.** Plating, lithium-inventory loss, self-limiting behavior, partial reversibility and the electrolyte/resistance explanation are supported. The paper also directly reports short cycle life at −22 °C; calling it evidence of “only plating” would be incorrect.
- **Charge termination:** [8] specifies both voltage and cutoff current; [13] supplies the attributed interpolation. Neither establishes the uncited causal sentence as written.
