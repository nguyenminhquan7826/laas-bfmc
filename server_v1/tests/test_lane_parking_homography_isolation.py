import unittest
from pathlib import Path


class LaneParkingHomographyIsolationTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        repo = Path(__file__).resolve().parents[2]
        cls.executive_cpp = (
            repo / "src" / "execution_control" / "Executive.cpp"
        ).read_text(encoding="utf-8")
        cls.parking_cpp = (
            repo
            / "src"
            / "functional"
            / "perception"
            / "ParkingPerceptionModule.cpp"
        ).read_text(encoding="utf-8")

    def test_debug_stream_uses_lane_detector_bird_eye(self):
        self.assertIn(
            "yolo_.sendDebugFrame(lane.bird_eye_view, 80);",
            self.executive_cpp,
        )

    def test_parking_homography_never_warps_control_image(self):
        self.assertNotIn("renderBirdEye", self.executive_cpp)
        self.assertNotIn("cv::warpPerspective", self.parking_cpp)

    def test_parking_homography_still_maps_detection_footpoints(self):
        self.assertIn("imageToGround(foot_u, foot_v", self.parking_cpp)


if __name__ == "__main__":
    unittest.main()
