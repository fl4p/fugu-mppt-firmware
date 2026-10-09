*this document is an LLM generated placeholder*

# Notes moved off published doc pages

Unpublished. These notes were removed from pages under `website/docs/` on 2026-10-09 because published pages carry
curated reference text only. The DCM-ringing retraction history went to `doc/research/dcm-ringing-retractions.md`.

## guide/charging/lfp-charging.md

The page ended with this open TODO list (unverified against the code, moved verbatim):

```
## TODO mention here:
* charger robust to ADC gain error (using own Vout measurement to pin voltage, eliminates error)
* robust with multiple chargers without talking to each other
```

The same page and `guide/charging/termination.md` claimed that the default `tail_c_rate=0.05` "will never be
premature on LFP and never over-charges". The code does not support the absolute: `src/charger.h`
(`Li_ChgTerminationCondition::update`) caps the termination line at `cv_eoc` and backstops it with `cv_ceiling`, which
bounds the voltage for any tail rate, and termination.md itself documents a premature termination seen in the field
(July 2026, EOC feedback loop). Both pages now state only the cap and the backstop.

## guide/updating/ble-ota-transports.md

Integration-scope statements removed from the page:

- "ESPHome proxy behavior is unchanged."
- "These host changes leave the converter firmware unmodified and do not authorize flashing a live converter."
- "No live converter was flashed for this integration." (The transport tests in `etc/test_ota_transport.py` use fake
  peers only.)
