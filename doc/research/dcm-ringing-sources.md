*this document is an LLM generated placeholder*

# DCM ringing page: citation check (2026-11-26)

Checked the uncited claims on `website/docs/internals/dcm-ringing.mdx` against the sources.

| Claim on the page (before) | Source finding | Change |
|---|---|---|
| "TI [4] says 3–10 ×" for `C_snub` | SLYT465 gives the halve-`f_r` method and `R = √(L/C)`, no capacitor ratio; it says more capacitance damps more but dissipates more | dropped the 3–10 × attribution; "more damping, more loss" cited to [4] |
| "ADI [1] says 1–4 ×" | ADI: "increasing it by a factor of 1 to 4 times"; example uses 2 × | kept, as the step's value |
| "Severns" (no reference) | Severns, CDE app note: `L_p` from ring period plus a test capacitor; quick design `C_s` = 2 × (switch output + mounting capacitance), `R_s = E_o/I_o` | added as [7] with what it actually says |
| "Trace adds ~10 nH per 25 mm" | ADI [1]: "up to 10 nH for every 25 mm (1 inch)" with poor layout | cited [1], "up to", "with poor layout" |
| "onsemi FAN65008, TI LM5146 do this automatically" | FAN65008B: DCM with pulse skipping, selected by the MODE pin. LM5146: selectable diode emulation (DCM) or FPWM, no pulse skipping in the data sheet | FAN65008B as [8], "when its MODE pin selects it"; LM5146 removed |
| Ref [5] listed, never cited, no author | Walczak, Arch. Electr. Eng. 66(2) 313–323, 2017, DOI 10.1515/aee-2017-0023: the DCM ring can change the effective duty cycle | cited in the DCM-ring bullet; bibliographic data completed |

Still uncited on the page: the 3–10 × `C_sw` range in the DCM "RC snubber" subsection (a design choice there,
which feeds the 9–30 nF and 1.8–5.9 W board C figures).

## Access log

- ADI article: curl hung (HTTP/2 reset), headless and headed scratch Chrome got "Access Denied" (Akamai);
  Fab's Chrome (`fab-chrome-fetch`) returned the article.
- TI SLYT465, LM5146 and FAN65008B data sheets, CDE Severns PDF: curl with a Chrome UA.
- Specter Engineering blog: curl.
- ResearchGate 317525001: DataDome CAPTCHA in Fab's Chrome (known, kb note
  `researchgate-restriction-defeats-users-own-chrome-use-author-host`); bibliographic data and abstract from
  journals.pan.pl (headless Chrome) and the Semantic Scholar API. Full text not read.
