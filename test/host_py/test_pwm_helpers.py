"""Host-side unit tests for the pure helpers in etc/mcpwm_gate_verify.py."""
import sys, unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "etc"))
from mcpwm_gate_verify import edges, parse_pwm_dump, hs_grid, ls_grid, safety_check, Result, SAFE_HOST_PATTERN


class FakeReply:
    def __init__(self, text): self.text = text

class FakeConsole:
    def __init__(self, ip_reply, hostname_reply=""):
        self.ip_reply = ip_reply
        self.hostname_reply = hostname_reply
    def command(self, cmd, timeout=3.0):
        if cmd == "hostname": return FakeReply(self.hostname_reply)
        if cmd == "ip": return FakeReply(self.ip_reply)
        return FakeReply("")


class TestEdges(unittest.TestCase):
    def test_basic_square_wave(self):
        v = [3.3] * 4 + [0.0] * 4 + [3.3] * 4 + [0.0] * 4
        rises, falls = edges(v)
        self.assertEqual(rises, [8])
        self.assertEqual(falls, [4, 12])

    def test_starts_high(self):
        v = [3.3] * 4 + [0.0] * 4
        rises, falls = edges(v)
        self.assertEqual(rises, [])
        self.assertEqual(falls, [4])

    def test_below_threshold_no_edges(self):
        v = [0.5, 0.6, 0.7, 0.6, 0.5]
        rises, falls = edges(v)
        self.assertEqual(rises, [])
        self.assertEqual(falls, [])

    def test_empty_input_returns_empty(self):
        self.assertEqual(edges([]), ([], []))


class TestParsePwmDump(unittest.TestCase):
    def test_parses_all_fields(self):
        line = "freq=39062 pwmMax=2048 hs_off=1024 ls_on=1026 ls_off=1280 fault=0 brake=0"
        d = parse_pwm_dump(line)
        self.assertEqual(d["freq"], 39062)
        self.assertEqual(d["pwmMax"], 2048)
        self.assertEqual(d["hs_off"], 1024)
        self.assertEqual(d["ls_on"], 1026)
        self.assertEqual(d["ls_off"], 1280)
        self.assertEqual(d["fault"], 0)
        self.assertEqual(d["brake"], 0)

    def test_ignores_surrounding_noise(self):
        line = "some banter freq=100 pwmMax=200 hs_off=50 ls_on=51 ls_off=120 fault=1 brake=1 OK"
        d = parse_pwm_dump(line)
        self.assertEqual(d["fault"], 1)
        self.assertEqual(d["brake"], 1)


class TestHsGrid(unittest.TestCase):
    def test_pwm_max_2048(self):
        g = hs_grid(2048)
        for x in (0, 1, 2046, 2047, 2048):
            self.assertIn(x, g)
        self.assertIn(1024, g)
        self.assertIn(16, g)
        self.assertEqual(g, sorted(set(g)))

    def test_pwm_max_4096_scales(self):
        g = hs_grid(4096)
        for x in (0, 1, 4094, 4095, 4096):
            self.assertIn(x, g)
        self.assertIn(2048, g)
        self.assertIn(32, g)

    def test_pwm_max_below_2_asserts(self):
        with self.assertRaises(AssertionError):
            hs_grid(1)
        with self.assertRaises(AssertionError):
            hs_grid(0)


class TestLsGrid(unittest.TestCase):
    def test_window_zero_or_one_returns_empty(self):
        self.assertEqual(ls_grid(0), [])
        self.assertEqual(ls_grid(1), [])

    def test_window_1023(self):
        g = ls_grid(1023)
        for x in (0, 1, 1022, 1023):
            self.assertIn(x, g)
        self.assertIn(512, g)
        self.assertEqual(g, sorted(set(g)))


class TestSafetyCheck(unittest.TestCase):
    def test_mock_host_pass(self):
        args = SimpleNamespace(force_host=False)
        con = FakeConsole("", hostname_reply="Hostname: fugu-esp32s3-139C")
        r = safety_check(con, args)
        self.assertTrue(r.pass_)

    def test_bare_fugu_host_pass(self):
        args = SimpleNamespace(force_host=False)
        con = FakeConsole("", hostname_reply="Hostname: fugu")
        r = safety_check(con, args)
        self.assertTrue(r.pass_)

    def test_real_host_fail(self):
        args = SimpleNamespace(force_host=False)
        con = FakeConsole("", hostname_reply="Hostname: fry")
        r = safety_check(con, args)
        self.assertFalse(r.pass_)

    def test_force_host_bypasses(self):
        args = SimpleNamespace(force_host=True)
        con = FakeConsole("", hostname_reply="Hostname: fry")
        r = safety_check(con, args)
        self.assertTrue(r.pass_)
        self.assertIn("force-host", r.detail)


class TestPhaseSignalCheck(unittest.TestCase):
    """Drive phase_signal_check() with two synthetic captures per channel (low + high)."""

    def setUp(self):
        from mcpwm_gate_verify import phase_signal_check
        self.fn = phase_signal_check

    def _run(self, ch_a_low, ch_a_high, ch_b_low, ch_b_high):
        """captures[] cycle — phase_signal_check makes 4 capture() calls in order:
           1. `dc 0` baseline -> read Ch A
           2. `dc 0` baseline -> read Ch B
           3. `dc <pwm_max-1>` -> read Ch A (HS high)
           4. `dc 1 <pwm_max-2>` (with sync forced) -> read Ch B (LS high)
        """
        import mcpwm_gate_verify as m
        seq = [
            (1e-6, [ch_a_low]  * 100, [ch_b_low]  * 100),   # dc 0, read Ch A
            (1e-6, [ch_a_low]  * 100, [ch_b_low]  * 100),   # dc 0, read Ch B
            (1e-6, [ch_a_high] * 100, [ch_b_low]  * 100),   # dc pwm_max-1, read Ch A
            (1e-6, [ch_a_high] * 100, [ch_b_high] * 100),   # dc 1 pwm_max-2, read Ch B
        ]
        idx = [0]
        original = m.capture
        def fake_capture(scope, pc, freq, samples=3000):
            r = seq[idx[0]]; idx[0] += 1; return r
        m.capture = fake_capture
        try:
            class FakeCon:
                def command(self, cmd, timeout=2.0):
                    class R: text = ""
                    return R()
            class FakeScope: pass
            class FakePc: pass
            board = {"pwm_freq": 39000, "pwm_max": 2047, "pwm_hi": 11, "pwm_li": 13}
            return self.fn(FakeCon(), FakeScope(), FakePc(), board)
        finally:
            m.capture = original

    def test_clean_3v3_step_passes(self):
        rows = self._run(0.05, 3.3, 0.05, 3.3)
        self.assertEqual(len(rows), 2)
        self.assertTrue(all(r.pass_ for r in rows), f"expected both PASS, got: {rows}")

    def test_attenuated_channel_fails(self):
        # 50x attenuation: 3.3V appears as 0.066V → step is ~0.02V on Ch A, well below 1.2V.
        rows = self._run(0.0, 0.066, 0.05, 3.3)
        self.assertEqual(len(rows), 2)
        self.assertFalse(rows[0].pass_, f"Ch A should FAIL (atten), got: {rows[0]}")
        self.assertTrue(rows[1].pass_,  f"Ch B should PASS, got: {rows[1]}")

    def test_disconnected_channel_fails(self):
        # disconnected probe noise around 0V; ~0 step.
        rows = self._run(0.0, 0.0, 0.05, 3.3)
        self.assertFalse(rows[0].pass_)
        self.assertTrue(rows[1].pass_)


class TestPhase1Synthetic(unittest.TestCase):
    """Drive phase1_freq_duty() through a fake console + scope to verify the logic
    without hardware. Synthetic waveforms produce predictable PASS/FAIL outcomes."""

    def setUp(self):
        from mcpwm_gate_verify import phase1_freq_duty
        self.phase1 = phase1_freq_duty

    def _make_capture(self, duty, freq, samples=3000, hs_static=None):
        """Return (dt, hs_v, ls_v) — hs_v synthesized as a duty-cycle square wave at `freq`."""
        # 4 periods of `freq` across `samples`.
        dt = 4 / freq / samples
        hs_v = []
        ls_v = [0.0] * samples
        for i in range(samples):
            t = i * dt
            phase = (t * freq) % 1.0
            if hs_static is True:
                hs_v.append(3.3)
            elif hs_static is False:
                hs_v.append(0.0)
            else:
                hs_v.append(3.3 if phase < duty else 0.0)
        return dt, hs_v, ls_v

    def test_static_low_hs0_passes(self):
        from mcpwm_gate_verify import hs_grid
        from types import SimpleNamespace
        pwm_max = 2048
        freq = 40000
        grid = hs_grid(pwm_max)
        captures = {}
        for h in grid:
            if h == 0:
                captures[h] = self._make_capture(0, freq, hs_static=False)
            elif h == pwm_max:
                captures[h] = self._make_capture(0, freq, hs_static=True)
            else:
                captures[h] = self._make_capture(h / pwm_max, freq)

        class FakeScope:
            def close(self): pass
        class FakePc: pass
        class FakeCon:
            def __init__(self): self.calls = []
            def command(self, cmd, timeout=2.0):
                self.calls.append(cmd)
                class R: text = ""
                return R()
            def close(self): pass

        import mcpwm_gate_verify as m
        original_capture = m.capture
        hs_seq = sorted(captures.keys())
        idx = [0]
        def fake_capture(scope, pc, freq, samples=3000):
            hs = hs_seq[idx[0]]
            idx[0] += 1
            return captures[hs]
        m.capture = fake_capture
        try:
            board = {"pwm_freq": 40000, "pwm_max": 2048, "pwm_deadtime_ns": 0,
                     "pwm_hi": 13, "pwm_li": 11, "pwm_fault_pin": -1}
            rows = self.phase1(FakeCon(), FakeScope(), FakePc(), board)
        finally:
            m.capture = original_capture

        names = [r.name for r in rows]
        self.assertTrue(any("static[hs=0]" == r.name and r.pass_ for r in rows),
                        f"expected static[hs=0] PASS, got: {rows}")
        self.assertTrue(any("static[hs=2048]" == r.name and r.pass_ for r in rows),
                        f"expected static[hs=2048] PASS, got: {rows}")
        self.assertTrue(any(r.name.startswith("duty[") and r.pass_ for r in rows))


class _FakeScope:
    def close(self): pass

class _FakePc:
    pass

def _make_fake_con(dump_line):
    class _FakeCon:
        def __init__(self, dl):
            self._dump = dl
        def command(self, cmd, timeout=2.0):
            class R: pass
            r = R()
            r.text = self._dump if cmd == "pwm-dump" else ""
            return r
        def close(self): pass
    return _FakeCon(dump_line)


class TestSetupBoard(unittest.TestCase):
    """setup_board() reads board.conf through `get-config`. The dead-time keys are floats and a
    quantized dead-time is genuinely fractional, so an int-only value pattern would read them as
    absent and silently fall back to the common key — the failure this covers."""

    @staticmethod
    def _con(values: dict):
        class _Con:
            def command(self, cmd, timeout=2.0):
                class R: pass
                r = R()
                if cmd == "pwm-dump":
                    r.text = "freq=39000 pwmMax=4071 hs_off=0 ls_on=0 ls_off=0"
                    return r
                key = cmd.split()[-1]
                v = values.get(key)
                r.text = "" if v is None else f"Conf '/littlefs/conf/board.conf:{key}' = '{v}'"
                return r
        return _Con()

    def test_fractional_deadtime_keys_are_read(self):
        from mcpwm_gate_verify import setup_board
        b = setup_board(self._con({
            "pwm_freq": "39000", "pwm_deadtime_ns": "200",
            "pwm_deadtime_hl_ns": "193.75", "pwm_deadtime_lh_ns": "106.25",
            "pwm_hi": "21", "pwm_li": "20", "pwm_fault_pin": "-1",
        }))
        self.assertAlmostEqual(b["pwm_deadtime_hl_ns"], 193.75)
        self.assertAlmostEqual(b["pwm_deadtime_lh_ns"], 106.25)
        self.assertEqual(b["pwm_freq"], 39000)
        self.assertIsInstance(b["pwm_freq"], int)   # integral keys stay ints
        self.assertEqual(b["pwm_fault_pin"], -1)

    def test_absent_edge_keys_fall_back_to_common(self):
        from mcpwm_gate_verify import setup_board
        b = setup_board(self._con({
            "pwm_freq": "39000", "pwm_deadtime_ns": "200",
            "pwm_hi": "21", "pwm_li": "20", "pwm_fault_pin": "-1",
        }))
        self.assertEqual(b["pwm_deadtime_hl_ns"], 200)
        self.assertEqual(b["pwm_deadtime_lh_ns"], 200)


class TestPhase2Synthetic(unittest.TestCase):
    """Drive phase2_ls_pos_width() via fake console + scope without hardware."""

    def setUp(self):
        from mcpwm_gate_verify import phase2_ls_pos_width
        self.phase2 = phase2_ls_pos_width

    @staticmethod
    def _make_capture_raw(freq, pwm_max, hs_off, ls_on, ls_off):
        """Build (dt, hs_v, ls_v) with 1 sample per tick (8 periods).
        Tick-exact alignment ensures LS edges match pwm-dump ticks within tolerance."""
        n_periods = 8
        samples = n_periods * pwm_max
        dt = 1.0 / (freq * pwm_max)  # 1 sample = 1 tick
        hs_v, ls_v = [], []
        for i in range(samples):
            tick = i % pwm_max
            hs_v.append(3.3 if tick < hs_off else 0.0)
            ls_v.append(3.3 if ls_on <= tick < ls_off else 0.0)
        return dt, hs_v, ls_v

    def _patch_capture(self, m, fn):
        orig = m.capture
        m.capture = fn
        return orig

    def test_hs0_both_static_low_passes(self):
        import mcpwm_gate_verify as m
        pwm_max = 2048
        freq    = 40000
        board   = {"pwm_freq": freq, "pwm_max": pwm_max,
                   "pwm_hi": 13, "pwm_li": 11, "pwm_fault_pin": -1, "pwm_deadtime_ns": 0}
        static_dt = 4.0 / freq / 3000
        orig = self._patch_capture(m, lambda sc, pc, f, samples=3000:
                                   (static_dt, [0.0] * 3000, [0.0] * 3000))
        try:
            rows = self.phase2(_make_fake_con(""), _FakeScope(), _FakePc(), board)
        finally:
            m.capture = orig
        hs0_rows = [r for r in rows if r.name == "both_static_low[hs=0]"]
        self.assertTrue(hs0_rows, "expected both_static_low[hs=0] row")
        self.assertTrue(hs0_rows[0].pass_, f"expected PASS, got: {hs0_rows[0]}")

    def test_hs_at_pwm_max_minus1_no_window_passes(self):
        import mcpwm_gate_verify as m
        pwm_max = 2048
        freq    = 40000
        board   = {"pwm_freq": freq, "pwm_max": pwm_max,
                   "pwm_hi": 13, "pwm_li": 11, "pwm_fault_pin": -1, "pwm_deadtime_ns": 0}
        static_dt = 4.0 / freq / 3000
        orig = self._patch_capture(m, lambda sc, pc, f, samples=3000:
                                   (static_dt, [3.3] * 3000, [0.0] * 3000))
        try:
            rows = self.phase2(_make_fake_con(""), _FakeScope(), _FakePc(), board)
        finally:
            m.capture = orig
        no_win = [r for r in rows if r.name.startswith("no_window[hs=")]
        self.assertTrue(no_win, f"expected no_window row; rows={[r.name for r in rows]}")
        self.assertTrue(all(r.pass_ for r in no_win),
                        f"expected all no_window PASS, got: {no_win}")

    def test_valid_hs_ls_point_pos_and_width_pass(self):
        """For a single (hs, ls) pair, check ls_pos and ls_wid both PASS when the synthetic
        capture exactly matches the dump ticks. Uses a minimal board with only hs=512 in scope
        by patching hs_grid + ls_grid to return a single point each."""
        import mcpwm_gate_verify as m
        pwm_max = 2048
        freq    = 40000
        hs_val  = 512
        ls_val  = 256
        hs_off  = hs_val
        ls_on   = hs_off + 2
        ls_off  = ls_on + ls_val
        board   = {"pwm_freq": freq, "pwm_max": pwm_max,
                   "pwm_hi": 13, "pwm_li": 11, "pwm_fault_pin": -1, "pwm_deadtime_ns": 0}
        dump_line = (f"freq={freq} pwmMax={pwm_max} hs_off={hs_off} "
                     f"ls_on={ls_on} ls_off={ls_off} fault=0 brake=0")

        orig_hs_grid = m.hs_grid
        orig_ls_grid = m.ls_grid
        m.hs_grid = lambda n: [hs_val]
        m.ls_grid = lambda n: [ls_val]
        orig_capture = m.capture

        def fake_capture(scope, pc, f, samples=3000):
            return self._make_capture_raw(freq, pwm_max, hs_off, ls_on, ls_off)

        m.capture = fake_capture
        try:
            rows = self.phase2(
                _make_fake_con(dump_line), _FakeScope(), _FakePc(), board
            )
        finally:
            m.hs_grid = orig_hs_grid
            m.ls_grid = orig_ls_grid
            m.capture = orig_capture

        pos_rows = [r for r in rows if r.name.startswith("ls_pos[")]
        wid_rows = [r for r in rows if r.name.startswith("ls_wid[")]
        self.assertTrue(pos_rows, f"expected ls_pos row; rows={[r.name for r in rows]}")
        self.assertTrue(wid_rows, f"expected ls_wid row; rows={[r.name for r in rows]}")
        self.assertTrue(pos_rows[0].pass_, f"ls_pos FAIL: {pos_rows[0]}")
        self.assertTrue(wid_rows[0].pass_, f"ls_wid FAIL: {wid_rows[0]}")


class TestPhase3Synthetic(unittest.TestCase):
    """Drive phase3_deadtime() via fake console + scope without hardware.
    Uses 1 sample = 1 tick (samples = 4 * pwm_max) to avoid rounding fudge."""

    def setUp(self):
        from mcpwm_gate_verify import phase3_deadtime
        self.phase3 = phase3_deadtime

    @staticmethod
    def _make_capture(freq, pwm_max, hs_off, ls_rise_tick, lh_ticks=0, ls_fall_tick=None):
        """Build (dt, hs_v, ls_v): 4 periods, 1 sample per tick. The timer period is
        pwm_max + lh_ticks (the LS->HS band is reserved out of pwm_max, so it lives past
        the commandable span). HS is high in [0, hs_off), LS in [ls_rise_tick, ls_fall_tick)."""
        period = pwm_max + lh_ticks
        if ls_fall_tick is None:
            ls_fall_tick = period
        samples = 4 * period
        dt = 1.0 / (freq * pwm_max)
        hs_v, ls_v = [], []
        for i in range(samples):
            tick = i % period
            hs_v.append(3.3 if tick < hs_off else 0.0)
            ls_v.append(3.3 if ls_rise_tick <= tick < ls_fall_tick else 0.0)
        return dt, hs_v, ls_v

    @staticmethod
    def _commanded(pwm_max):
        """The duty the dt_lh capture drives: `dc hs (pwm_max - hs - 1)`, LS at its maximum so the
        wrap band carries only one count of commanded slack."""
        hs = pwm_max // 2
        return hs, pwm_max - hs - 1

    def _run_phase3(self, board, hs_off, ls_rise_tick, lh_ticks=0, ls_fall_tick=None):
        import mcpwm_gate_verify as m
        freq    = board["pwm_freq"]
        pwm_max = board["pwm_max"]

        def fake_capture(scope, pc, f, samples=3000):
            return self._make_capture(freq, pwm_max, hs_off, ls_rise_tick, lh_ticks, ls_fall_tick)

        class _Con:
            def command(self, cmd, timeout=2.0):
                class R: text = ""
                return R()

        orig = m.capture
        m.capture = fake_capture
        try:
            return self.phase3(_Con(), _FakeScope(), _FakePc(), board)
        finally:
            m.capture = orig

    @staticmethod
    def _board(freq, pwm_max, hl_ns, lh_ns):
        return {"pwm_freq": freq, "pwm_max": pwm_max, "pwm_deadtime_ns": hl_ns,
                "pwm_deadtime_hl_ns": hl_ns, "pwm_deadtime_lh_ns": lh_ns,
                "pwm_hi": 13, "pwm_li": 11, "pwm_fault_pin": -1}

    def test_dt0_passes_and_no_shoot_through(self):
        """Dead-time 0/0: LS rises exactly 1 sample after HS falls (degenerate gap of 1 tick).
        dt_hl PASS (gap=1 tick, expected=0, tol=1 tick), dt_lh PASS and shoot_through PASS."""
        pwm_max = 256
        freq    = 40000
        board   = self._board(freq, pwm_max, 0, 0)
        hs_cmd, ls_cmd = self._commanded(pwm_max)
        hs_off       = hs_cmd                # HS falls at tick hs_off
        ls_rise_tick = hs_off + 1            # LS rises 1 tick later (gap = 1 tick)

        rows = self._run_phase3(board, hs_off, ls_rise_tick,
                                ls_fall_tick=hs_cmd + ls_cmd)

        dt_row = next((r for r in rows if r.name == "dt_hl"), None)
        lh_row = next((r for r in rows if r.name == "dt_lh"), None)
        st_row = next((r for r in rows if r.name == "shoot_through"), None)
        self.assertIsNotNone(dt_row, f"dt_hl row missing; rows={[r.name for r in rows]}")
        self.assertIsNotNone(lh_row, f"dt_lh row missing; rows={[r.name for r in rows]}")
        self.assertIsNotNone(st_row, f"shoot_through row missing; rows={[r.name for r in rows]}")
        self.assertTrue(dt_row.pass_, f"dt_hl FAIL: {dt_row}")
        self.assertTrue(lh_row.pass_, f"dt_lh FAIL: {lh_row}")
        self.assertTrue(st_row.pass_, f"shoot_through FAIL: {st_row}")

    def test_dt80ns_passes(self):
        """Symmetric 80 ns: LS rise positioned at the right sample. dt_hl PASS within ±1 tick."""
        pwm_max      = 256
        freq         = 40000
        dt_ns_exp    = 80
        tick_s       = 1.0 / (freq * pwm_max)
        dt_ticks     = round(dt_ns_exp * 1e-9 / tick_s)  # ticks for 80 ns gap
        board        = self._board(freq, pwm_max, dt_ns_exp, dt_ns_exp)
        hs_cmd, ls_cmd = self._commanded(pwm_max)
        hs_off       = hs_cmd
        ls_rise_tick = hs_off + dt_ticks

        rows = self._run_phase3(board, hs_off, ls_rise_tick,
                                lh_ticks=dt_ticks, ls_fall_tick=hs_cmd + ls_cmd)

        dt_row = next((r for r in rows if r.name == "dt_hl"), None)
        lh_row = next((r for r in rows if r.name == "dt_lh"), None)
        self.assertIsNotNone(dt_row, f"dt_hl row missing; rows={[r.name for r in rows]}")
        self.assertTrue(dt_row.pass_, f"dt_hl FAIL: {dt_row}")
        self.assertTrue(lh_row.pass_, f"dt_lh FAIL: {lh_row}")

    def test_split_deadtime_rows_track_their_own_edge(self):
        """Asymmetric hl=50 ns / lh=200 ns: each row must match its own key. A verifier that
        reused one value for both edges fails at least one of these."""
        pwm_max  = 256
        freq     = 40000
        tick_s   = 1.0 / (freq * pwm_max)
        hl_ticks = round(50e-9 / tick_s)
        lh_ticks = round(200e-9 / tick_s)
        self.assertNotEqual(hl_ticks, lh_ticks)
        board    = self._board(freq, pwm_max, round(hl_ticks * tick_s * 1e9),
                               round(lh_ticks * tick_s * 1e9))
        hs_cmd, ls_cmd = self._commanded(pwm_max)

        rows = self._run_phase3(board, hs_cmd, hs_cmd + hl_ticks,
                                lh_ticks=lh_ticks, ls_fall_tick=hs_cmd + ls_cmd)

        dt_row = next((r for r in rows if r.name == "dt_hl"), None)
        lh_row = next((r for r in rows if r.name == "dt_lh"), None)
        self.assertTrue(dt_row.pass_, f"dt_hl FAIL: {dt_row}")
        self.assertTrue(lh_row.pass_, f"dt_lh FAIL: {lh_row}")


class TestPhase4Synthetic(unittest.TestCase):
    """Drive phase4_fault_brake() via fake console + scope without hardware."""

    def setUp(self):
        from mcpwm_gate_verify import phase4_fault_brake
        self.phase4 = phase4_fault_brake

    @staticmethod
    def _switching_capture(freq, samples=3000):
        """4-period square wave at 50% duty."""
        dt = 4.0 / freq / samples
        hs_v = []
        for i in range(samples):
            phase = (i * dt * freq) % 1.0
            hs_v.append(3.3 if phase < 0.5 else 0.0)
        return dt, hs_v, [0.0] * samples

    @staticmethod
    def _quiet_capture(samples=3000):
        """Both gates low."""
        return (1e-6, [0.0] * samples, [0.0] * samples)

    def _make_board(self, fault_pin):
        return {"pwm_freq": 40000, "pwm_max": 2048, "pwm_deadtime_ns": 0,
                "pwm_hi": 13, "pwm_li": 11, "pwm_fault_pin": fault_pin}

    def test_no_fault_pin_returns_failure_row(self):
        import mcpwm_gate_verify as m

        class _Con:
            def command(self, cmd, timeout=2.0):
                class R: text = ""
                return R()

        board = self._make_board(-1)
        rows = self.phase4(_Con(), _FakeScope(), _FakePc(), board, drv_pin=14)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0].name, "fault_pin_configured")
        self.assertFalse(rows[0].pass_)

    def test_brake_quiet_and_recover_passes(self):
        import mcpwm_gate_verify as m

        class _Con:
            def command(self, cmd, timeout=2.0):
                class R: text = ""
                return R()

        freq = 40000
        call_idx = [0]
        def fake_capture(scope, pc, f, samples=3000):
            i = call_idx[0]
            call_idx[0] += 1
            if i == 0:
                return TestPhase4Synthetic._switching_capture(freq, samples)
            elif i == 1:
                return TestPhase4Synthetic._quiet_capture(samples)
            else:
                return TestPhase4Synthetic._switching_capture(freq, samples)

        board = self._make_board(10)
        orig = m.capture
        m.capture = fake_capture
        try:
            rows = self.phase4(_Con(), _FakeScope(), _FakePc(), board, drv_pin=14)
        finally:
            m.capture = orig

        self.assertEqual(len(rows), 3, f"expected 3 rows, got: {[r.name for r in rows]}")
        for r in rows:
            self.assertTrue(r.pass_, f"expected PASS for {r.name}: {r}")

    def test_brake_not_quiet_fails(self):
        import mcpwm_gate_verify as m

        class _Con:
            def command(self, cmd, timeout=2.0):
                class R: text = ""
                return R()

        freq = 40000
        call_idx = [0]
        def fake_capture(scope, pc, f, samples=3000):
            i = call_idx[0]
            call_idx[0] += 1
            # Both pre-brake and "braked" captures have switching HS edges
            return TestPhase4Synthetic._switching_capture(freq, samples)

        board = self._make_board(10)
        orig = m.capture
        m.capture = fake_capture
        try:
            rows = self.phase4(_Con(), _FakeScope(), _FakePc(), board, drv_pin=14)
        finally:
            m.capture = orig

        quiet_row = next((r for r in rows if r.name == "brake_gates_quiet"), None)
        self.assertIsNotNone(quiet_row, f"brake_gates_quiet row missing; rows={[r.name for r in rows]}")
        self.assertFalse(quiet_row.pass_, f"expected FAIL for brake_gates_quiet: {quiet_row}")


class TestFormatSummary(unittest.TestCase):
    def setUp(self):
        from mcpwm_gate_verify import format_summary
        self.fmt = format_summary

    def test_all_pass(self):
        s = [("0", 0), ("1", 0), ("2", 0), ("3", 0), ("4", 0)]
        line, n = self.fmt(s)
        self.assertEqual(n, 0)
        self.assertIn("exit=0", line)
        for label in ("0", "1", "2", "3", "4"):
            self.assertIn(f"{label} PASS", line)

    def test_mixed_fail_and_skip(self):
        s = [("0", 0), ("1", 0), ("2", 3), ("3", 0), ("4", "SKIP")]
        line, n = self.fmt(s)
        self.assertEqual(n, 3)
        self.assertIn("2 FAIL(3)", line)
        self.assertIn("4 SKIP", line)
        self.assertIn("exit=3", line)

    def test_empty_summary(self):
        line, n = self.fmt([])
        self.assertEqual(n, 0)
        self.assertIn("exit=0", line)
        self.assertTrue(line.startswith("SUMMARY  "))


if __name__ == "__main__":
    unittest.main()
