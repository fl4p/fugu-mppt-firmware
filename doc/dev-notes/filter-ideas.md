*this document is an LLM generated placeholder*

# Filter ideas (unreviewed)

Moved verbatim from the public Signal Filters page; speculative suggestions, not implemented.

- Drop the notch on Vout unless you can show inverter pickup on the scope channel. The biquad adds ~2-sample group delay and ringing risk
  for a benefit you may not be getting (DC-side ripple at 100 Hz on the battery node is usually tiny — the bulk caps absorb it).
    - One-Euro / "α-β with deadband" would be a better fit than EWMA-2pass for Vout: small α when the signal is quiet, large α when |Δ|
      exceeds a threshold. Same code complexity as EWMA, low latency on steps, low noise at steady state. This is essentially what your anf was
      reaching for but applied properly.
    - 1-state Kalman for Vout: state = Vout, model = random walk + control input ΔD·(dV/dD). With a measurement-noise estimate from your
      existing ewm.std, gains adapt automatically. In steady state it collapses to an EWMA, but during sweeps/transients you get faster
      convergence at the same noise floor. Worth doing only if you also feed it ΔD — otherwise just use one-Euro.
    - Don't multi-pass EWMA on the controller input. EWMA_nPass<2> doubles group delay; for a digital controller you usually want the least
      phase lag at given noise rejection. A single-pass EWMA with α tuned to match the 2-pass noise floor has less lag, or use a Butterworth
      biquad LPF (esp-dsp already gives you dsps_biquad_gen_lpf_f32) for a sharper rolloff at the same delay.
    - Anti-alias separately from "smoothing". The control input should be band-limited to <fs/2 of the controller loop, not the ADC. If your
      ADC runs much faster than the control rate, decimate (block-average to controller rate) then run a smaller LPF. Block-averaging N samples
      reduces noise σ by √N with one sample of group delay, which is hard to beat with any IIR.


If I had to make one change today: swap median→notch order, delete anf, and try EWMA_nPass<1> on Vout with a slightly larger span.
Measure controller behavior before adding Kalman/one-Euro.