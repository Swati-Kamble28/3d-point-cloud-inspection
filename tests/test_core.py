import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from inspection.core import transform, rigid_fit, synthetic, clean, register_numpy, evaluate, pose_error


class InspectionTests(unittest.TestCase):
    def test_rigid_fit_recovers_known_transform(self):
        rng = np.random.default_rng(1)
        points = rng.normal(size=(100, 3))
        truth = np.eye(4)
        truth[:3, :3] = Rotation.from_euler('xyz', [20, 40, -10], degrees=True).as_matrix()
        truth[:3, 3] = [2, -3, 7]
        recovered = rigid_fit(points, transform(points, truth))
        np.testing.assert_allclose(recovered, truth, atol=1e-10)
        self.assertAlmostEqual(np.linalg.det(recovered[:3, :3]), 1.)

    def test_zero_overlap_reports_null_rmse(self):
        a = np.zeros((30, 3))
        metrics, _ = evaluate(a+100, a, threshold=1)
        self.assertEqual(metrics['fitness'], 0)
        self.assertIsNone(metrics['inlier_rmse_mm'])

    def test_registration_and_defect_contrast(self):
        rates = []
        for defect in [False, True]:
            ref, scan, truth, _ = synthetic(42, 5000, defect)
            ref, scan = clean(ref), clean(scan)
            estimated = register_numpy(scan, ref, threshold=2)
            errors = pose_error(estimated, truth)
            self.assertLess(errors['rotation_error_deg'], .8)
            self.assertLess(errors['translation_error_mm'], .4)
            metrics, _ = evaluate(transform(scan, estimated), ref)
            rates.append(metrics['fraction_above_tolerance'])
        self.assertGreater(rates[1], rates[0]+.003)

    def test_reject_invalid_cloud(self):
        with self.assertRaises(ValueError):
            clean(np.zeros((5, 2)))


if __name__ == '__main__':
    unittest.main()
