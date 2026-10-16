"""Registration and inspection primitives. Coordinates are millimetres in the demo."""
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from itertools import product


def transform(points, matrix):
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


def rigid_fit(source, target):
    """Least-squares rigid transform: source -> target (no scale fitting)."""
    a, b = source.mean(0), target.mean(0)
    u, _, vt = np.linalg.svd((source-a).T @ (target-b))
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1] *= -1
        r = vt.T @ u.T
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = r, b-r@a
    return t


def clean(points, voxel=0.7, neighbors=20, std_ratio=2.5):
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3:
        raise ValueError('Expected an N x 3 point cloud')
    p = p[np.isfinite(p).all(1)]
    if len(p) < 30:
        raise ValueError('At least 30 finite points are required')
    _, ix = np.unique(np.floor(p/voxel).astype(np.int64), axis=0, return_index=True)
    p = p[np.sort(ix)]
    k = min(neighbors+1, len(p))
    distances = cKDTree(p).query(p, k=k)[0][:, 1:].mean(1)
    p = p[distances <= distances.mean()+std_ratio*distances.std()]
    if len(p) < 30:
        raise ValueError('Too few points after preprocessing')
    return p


def icp(source, target, initial=None, threshold=5., iterations=70, trim=0.9):
    """Trimmed point-to-point ICP. Metrics computed AFTER the final update."""
    t = np.eye(4) if initial is None else initial.copy()
    tree = cKDTree(target)
    previous = np.inf
    for step in range(iterations):
        moved = transform(source, t)
        d, ix = tree.query(moved)
        valid = d < threshold
        if valid.sum() < 6:
            break
        valid &= d <= np.quantile(d[valid], trim)
        if valid.sum() < 6:
            break
        update = rigid_fit(moved[valid], target[ix[valid]])
        t = update @ t
        error = float(np.sqrt(np.mean(d[valid]**2)))
        if abs(previous-error) < 1e-7:
            break
        previous = error
    return t


def register_numpy(source, target, threshold=4.):
    """PCA sign hypotheses + trimmed ICP; suitable for asymmetric, overlapping scans.

    This is a lightweight baseline, not FPFH/RANSAC or point-to-plane ICP.
    """
    def axes(p):
        _, vectors = np.linalg.eigh(np.cov(p.T))
        return vectors[:, ::-1]
    s, r = axes(source), axes(target)
    candidates = []
    for signs in product([-1, 1], repeat=3):
        rot = r @ np.diag(signs) @ s.T
        if np.linalg.det(rot) < 0:
            continue
        t = np.eye(4)
        t[:3, :3], t[:3, 3] = rot, target.mean(0)-rot@source.mean(0)
        candidates.append(t)
    tree = cKDTree(target)
    results = []
    for initial in candidates:
        t = icp(source, target, initial, threshold=max(threshold, 8.), iterations=90)
        t = icp(source, target, t, threshold=threshold, iterations=90)
        distances = tree.query(transform(source, t))[0]
        # Reward overlap, then residual: penalizes tiny accidental alignments.
        score = np.minimum(distances, threshold)**2
        results.append((float(score.mean()), t))
    return min(results, key=lambda x:x[0])[1]


def register_open3d(source, target, voxel=0.7, threshold=4., seed=42):
    """FPFH + RANSAC global alignment, followed by point-to-plane ICP."""
    try:
        import open3d as o3d
    except ImportError as exc:
        raise RuntimeError('Install requirements-open3d.txt to use --backend open3d') from exc
    o3d.utility.random.seed(seed)
    reg = o3d.pipelines.registration
    def cloud(p):
        c = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p))
        c.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel*3, max_nn=40))
        return c
    src, ref = cloud(source), cloud(target)
    def feature(c):
        return reg.compute_fpfh_feature(c, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel*6, max_nn=100))
    coarse = reg.registration_ransac_based_on_feature_matching(
        src, ref, feature(src), feature(ref), True, voxel*2.5,
        reg.TransformationEstimationPointToPoint(False), 3,
        [reg.CorrespondenceCheckerBasedOnEdgeLength(0.9),
         reg.CorrespondenceCheckerBasedOnDistance(voxel*2.5)],
        reg.RANSACConvergenceCriteria(100000, 0.999))
    if coarse.fitness == 0:
        raise RuntimeError('RANSAC found no alignment; inspect overlap and voxel size')
    fine = reg.registration_icp(src, ref, threshold, coarse.transformation,
        reg.TransformationEstimationPointToPlane(), reg.ICPConvergenceCriteria(max_iteration=100))
    return np.asarray(fine.transformation)


def evaluate(aligned, reference, threshold=2., tolerance=0.75):
    """Unsigned nearest-neighbour POINT distances, not signed CAD deviations."""
    forward = cKDTree(reference).query(aligned)[0]
    backward = cKDTree(aligned).query(reference)[0]
    inliers = forward < threshold
    metrics = dict(
        fitness=float(inliers.mean()),
        inlier_rmse_mm=float(np.sqrt(np.mean(forward[inliers]**2))) if inliers.any() else None,
        mean_deviation_mm=float(forward.mean()),
        median_deviation_mm=float(np.median(forward)),
        p95_deviation_mm=float(np.quantile(forward, .95)),
        max_deviation_mm=float(forward.max()),
        fraction_above_tolerance=float((forward > tolerance).mean()),
        symmetric_mean_distance_mm=float((forward.mean()+backward.mean())/2),
        correspondence_threshold_mm=threshold, tolerance_mm=tolerance,
        scan_points=len(aligned), reference_points=len(reference))
    return metrics, forward


def pose_error(estimated, truth):
    delta = estimated[:3, :3] @ truth[:3, :3].T
    return dict(rotation_error_deg=float(Rotation.from_matrix(delta).magnitude()*180/np.pi),
                translation_error_mm=float(np.linalg.norm(estimated[:3, 3]-truth[:3, 3])))


def synthetic(seed=42, count=7000, defect=True):
    """Asymmetric ellipsoid, localized inward dent, rigid pose, noise, and outliers.

    Corresponding reference sampling isolates the pipeline from scan-density changes;
    real independently sampled scans are more difficult.
    """
    rng = np.random.default_rng(seed)
    u = rng.normal(size=(count, 3))
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    reference = u*np.array([45., 28., 18.])
    reference[:, 0] += 3*np.sin(reference[:, 1]/9)*u[:, 2]**2
    scan = reference.copy()
    mask = ((reference[:, 0]-8)/12)**2 + ((reference[:, 1]-3)/9)**2 < 1
    mask &= reference[:, 2] > 10
    if defect:
        radial = ((reference[mask, 0]-8)/12)**2 + ((reference[mask, 1]-3)/9)**2
        scan[mask, 2] -= 2.5*(1-radial)
    else:
        mask[:] = False
    scan += rng.normal(0, .10, scan.shape)
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler('xyz', [18, -23, 32], degrees=True).as_matrix()
    pose[:3, 3] = [15., -11., 8.]
    scan = transform(scan, pose)
    outliers = rng.uniform([-90, -70, -55], [90, 70, 55], (count//100, 3))
    return reference, np.vstack([scan, outliers]), np.linalg.inv(pose), mask
