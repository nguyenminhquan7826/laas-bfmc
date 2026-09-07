import unittest
from pathlib import Path


class ParkingPoseBenchUartPolicyTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.repo = Path(__file__).resolve().parents[2]

        cls.main_cpp = (
            cls.repo / "src" / "app" / "main.cpp"
        ).read_text(encoding="utf-8")

        cls.safety_cpp = (
            cls.repo
            / "src"
            / "functional"
            / "safety"
            / "ParkingSafetyFilter.cpp"
        ).read_text(encoding="utf-8")

    def test_pose_bench_enables_uart_rx(self):
        self.assertIn(
            'std::getenv("LAAS_PARKING_POSE_BENCH")',
            self.main_cpp,
        )
        self.assertIn(
            "config.runtime.enable_uart = true;",
            self.main_cpp,
        )

    def test_pose_bench_hard_disables_uart_tx(self):
        self.assertIn(
            "config.runtime.enable_uart_tx = false;",
            self.main_cpp,
        )

    def test_pose_bench_enables_real_pose_estimator(self):
        self.assertIn(
            "config.parking.enable_pose_estimator = true;",
            self.main_cpp,
        )

    def test_legacy_parking_bench_still_disables_uart(self):
        self.assertIn(
            "config.runtime.enable_uart = false;",
            self.main_cpp,
        )

    def test_safety_filter_blocks_tx_not_rx(self):
        self.assertIn(
            "if (config_.runtime.enable_uart_tx)",
            self.safety_cpp,
        )
        self.assertIn(
            'return stop("UART_TX_MUST_BE_DISABLED_IN_BENCH");',
            self.safety_cpp,
        )

    def test_old_rx_blocking_gate_is_gone(self):
        self.assertNotIn(
            'if (config_.runtime.enable_uart) '
            'return stop("UART_MUST_BE_DISABLED_IN_BENCH");',
            self.safety_cpp,
        )


if __name__ == "__main__":
    unittest.main()
