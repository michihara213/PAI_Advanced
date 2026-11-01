"""Synthetic tests only. No real bag or Gazebo integration is exercised here."""
import unittest
import numpy as np
from wave_ai import named_positions, hold, clock_times, resample_episode, fit, predict, make_path


class Tests(unittest.TestCase):
    def test_joint_order_and_rejection(self):
        np.testing.assert_array_equal(named_positions(['b', 'a'], [2, 1], ['a', 'b']), [1, 2])
        with self.assertRaises(ValueError):
            named_positions(['a'], [float('nan')], ['a'])
        with self.assertRaises(ValueError):
            named_positions(['a'], [1], ['b'])

    def test_causal_hold(self):
        q = np.array([[0], [10], [20]])
        np.testing.assert_array_equal(hold(np.array([0, 1, 2]), q, [0, .9, 1, 2]).ravel(), [0, 0, 10, 20])
        with self.assertRaises(ValueError):
            hold(np.array([0, 1, 2]), q, [-1])

    def test_sim_clock_not_wall_duration(self):
        base = 1793564800320411339
        records = [(base, [0]), (base + 1000000000, [1]), (base + 2000000000, [2])]
        t, q = clock_times(records, [(base, 10), (base + 2000000000, 11)])
        np.testing.assert_allclose(t, [10, 10.5, 11])
        with self.assertRaises(ValueError):
            clock_times(records, [(base, 10), (base + 2000000000, 9)])

    def test_trim_preserves_return_and_interior_pause(self):
        t = np.arange(0, 20, .05)
        arm = np.zeros((len(t), 4))
        arm[(t >= 5) & (t < 12), 0] = .3
        grip = np.zeros((len(t), 1))
        streams = [(t, arm), (t, grip), (t, np.column_stack([arm, grip]))]
        q, _, report = resample_episode(streams)
        self.assertGreater(report['trim_start_s'], 4)
        self.assertGreater(report['active_sim_s'], 8)
        self.assertEqual(q[0, 0], 0)
        self.assertEqual(q[-1, 0], 0)
        self.assertEqual(q[:, 0].max(), .3)
        arm[:] = 0
        with self.assertRaises(ValueError):
            resample_episode(streams)

    def test_observed_millisecond_clock_reversals(self):
        sim = [4787.494, 4787.493, 4815.450, 4815.448, 4815.451, 4815.449, 4815.452]
        stamps = np.arange(len(sim), dtype=np.int64) * 1000000000
        records = [(int(t), [i]) for i, t in enumerate(stamps)]
        clock = list(zip(stamps, sim))
        before = list(clock)
        t, q = clock_times(records, clock)
        np.testing.assert_allclose(t, [4787.494, 4815.450, 4815.451, 4815.452])
        np.testing.assert_array_equal(q.ravel(), [1, 3, 5, 6])
        self.assertTrue(np.all(np.diff(t) > 0))
        self.assertEqual(clock, before)

    def test_clock_rewind_limit_uses_high_water_mark(self):
        records = [(0, [0]), (1000000000, [1]), (2000000000, [2])]
        # Each individual reversal is small, but cumulative rewind is too large.
        with self.assertRaises(ValueError):
            clock_times(records, [(0, 10), (1000000000, 9.994), (2000000000, 9.988)])
        with self.assertRaises(ValueError):
            clock_times(records, [(0, 10), (1000000000, 9.98), (2000000000, 10.1)])
        with self.assertRaises(ValueError):
            clock_times(records, [(0, 10), (2000000000, float('nan'))])

    def test_phase_wave_fit(self):
        p = np.linspace(0, 1, 201)
        wave = np.zeros((201, 5))
        wave[:, 0] = .3 * np.sin(np.pi * p) ** 2
        wave[:, 2] = .4
        y = np.repeat(wave[None], 8, axis=0)
        centers, w = fit(y)
        result = predict(p, centers, w, y.min((0, 1)), y.max((0, 1)))
        self.assertLess(float(np.sqrt(np.mean((result-wave)**2))), .002)
        self.assertGreater(result[100, 0], .29)
        self.assertLess(result[-1, 0], .01)
        self.assertTrue(np.all(result[:, 2] == .4))
        with self.assertRaises(ValueError):
            predict([-1], centers, w, y.min((0, 1)), y.max((0, 1)))

    def test_path_speed_and_start(self):
        current = np.zeros(5)
        wave = np.array([[1, 0, 0, 0, 0], [-1, 0, 0, 0, 0], [0, 0, 0, 0, 0]])
        q, t = make_path(current, wave, 1)
        np.testing.assert_array_equal(q[0], current)
        np.testing.assert_array_equal(q[-1], wave[-1])
        speeds = abs(np.diff(q, axis=0)) / np.diff(t)[:, None]
        self.assertLessEqual(speeds.max(), .300001)
        self.assertTrue(np.all(np.diff(t) > 0))


if __name__ == '__main__':
    unittest.main()
