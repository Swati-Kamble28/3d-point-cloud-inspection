# 3D Point Cloud Registration and Dimensional Inspection

A Python portfolio project for learning 3D registration and visual inspection. Align a scanned object to a reference, visualize unsigned deviations, and evaluate alignment against a known synthetic transformation.

**This repository is an educational prototype, not a ZEISS product or a certified metrology system.** All included results are synthetic experiments; no claim of real industrial deployment is made.

## Why this project

Relevant to 3D computer vision internships involving geometric perception, visual inspection, and dimensional measurement. The implementation covers preprocessing, registration, reproducible experiments, metrics, and result visualization. It does not implement a learned classifier or ROS2 integration.

## Quick start (Windows PowerShell)

Use Python 3.11 or 3.12. Extract this folder and open a terminal inside it.

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m inspection
python -m unittest discover -s tests -v
```

If PowerShell blocks activation, use the virtual environment interpreter directly:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m inspection
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

## Quick start (Linux / macOS)

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m inspection
python -m unittest discover -s tests -v
```

The default backend needs only NumPy, SciPy, and Matplotlib. No GPU or external dataset is required. Charts are saved as PNGs; no interactive desktop window opens.

## Pipeline

1. Generate an asymmetric reference shape in millimetres.
2. Add a localized inward dent, Gaussian noise, a known rigid transformation, and 1% extra outlier points.
3. Remove invalid points, voxel-downsample, and filter statistical outliers.
4. Estimate a scan-to-reference rigid transform using one of the backends below.
5. Calculate unsigned nearest-neighbour distances from the aligned, cleaned scan to the cleaned reference.
6. Export heatmaps, a histogram, per-point distances, a 4 x 4 transformation, and JSON metrics.
7. For synthetic inputs, compare the estimated transform to the exact inverse of the applied pose.

### Backends

| Backend | Initial alignment | Refinement | Status |
| --- | --- | --- | --- |
| `numpy` (default) | PCA principal axes with proper-rotation sign hypotheses | Trimmed point-to-point ICP using SVD | Executed in the supplied demo and tests |
| `open3d` | FPFH features and RANSAC | Point-to-plane ICP using estimated normals | See VALIDATION.md for execution status |

PCA is a practical baseline for asymmetric objects with substantial overlap. It is not a replacement for feature matching on arbitrary objects. Eigenvalue ambiguity, symmetry, partial overlap, and outliers can cause failure.

### Optional Open3D backend

```bash
python -m pip install -r requirements-open3d.txt
python -m inspection --backend open3d --output results/open3d
```

Normals use a radius of 3 times the voxel size; FPFH uses 6 times the voxel size. RANSAC uses a 2.5-voxel correspondence threshold, mutual feature matching, and edge-length/distance checkers. ICP uses `--threshold`. Adjust these for input density and object scale. Open3D wheels depend on your Python/platform; use Python 3.11/3.12 when possible. Feature matching is stochastic and may fail even on geometrically similar inputs; inspect the output rather than trusting a single score.

## Experiments

### Dent versus no dent

```bash
python -m inspection --seed 42 --output results/dent
python -m inspection --seed 42 --no-defect --output results/no_defect
```

Compare the distance distributions and fraction above threshold. A threshold crossing is a candidate anomaly, not proof of a manufacturing defect. Residual outliers, sampling differences, noise, and misalignment also raise distances.

### Reproducibility and density

```bash
python -m inspection --seed 7 --points 10000 --output results/seed7
python -m inspection --voxel 1.0 --threshold 2.0 --tolerance 0.75 --output results/coarser
```

The seed controls synthetic data and Open3D random sampling, but bitwise-identical Open3D results across platforms/threads are not guaranteed.

### Your own scans

CSV files must contain exactly three numeric columns, no header: x, y, z. NPY files must be N x 3 numeric arrays. PLY/PCD inputs require Open3D even if you use the NumPy registration backend.

```bash
python -m inspection --reference data/reference.csv --scan data/scan.csv --output results/custom
python -m inspection --reference data/reference.ply --scan data/scan.ply --backend open3d --scale-to-mm 1000 --voxel 0.7 --threshold 2 --output results/custom_ply
```

The second example assumes PLY coordinates are in metres; multiplying by 1000 converts them to millimetres. The program cannot infer units from coordinates. Both inputs must use the same units and represent rigidly related versions of the same object. No scale estimation is performed. Custom scans have no known pose, so ground-truth pose errors are omitted.

## Outputs

The ZIP includes a completed run in `results/demo/`.

| File | Content |
| --- | --- |
| `inspection.png` | Before/after overlays and unsigned deviation heatmap |
| `deviation_histogram.png` | Distance distribution and exploratory tolerance threshold |
| `metrics.json` | Post-registration residual and pose metrics |
| `scan_to_reference.csv` | Homogeneous 4 x 4 transform mapping scan coordinates into the reference frame |
| `deviations.csv` | Cleaned aligned XYZ, unsigned distance, threshold flag |
| `aligned_scan.npy` | Cleaned transformed scan |
| `reference_raw.npy`, `scan_raw.npy` | Input clouds before preprocessing |
| `ground_truth_scan_to_reference.csv` | Synthetic ground-truth transform only |
| `synthetic_defect_mask.npy` | Synthetic defect labels for the original reference sampling, before cleanup |

The raw defect mask does not index the cleaned scan and excludes appended outliers. It is provided for dataset inspection, not as a ready-made segmentation evaluation label array.

## Metric definitions

For scan point s_i and reference point set R, d_i = min over r in R of ||s_i - r||.

- **Fitness:** proportion of cleaned scan points with d_i below the correspondence threshold. This is scan-to-reference overlap, not classification accuracy.
- **Inlier RMSE:** square root of the mean squared distances for those inliers. It is `null` if no inliers exist. Low inlier RMSE can hide excluded defects/outliers, so inspect fitness and all-point metrics too.
- **Mean, median, 95th percentile, maximum:** computed across all cleaned scan points, including residual outliers.
- **Fraction above tolerance:** share of points exceeding the user-selected threshold. It is not physical defective area or a precision/recall score.
- **Symmetric mean distance:** average of scan-to-reference and reference-to-scan mean nearest-neighbour distances (unsquared).
- **Rotation error:** angle of the relative estimated/ground-truth rotation.
- **Translation error:** Euclidean difference between estimated and ground-truth translation vectors, in the shared reference frame.

Nearest-neighbour point distances depend on sampling density. They are unsigned, cannot distinguish inward/outward displacement, and are not calibrated signed CAD-surface measurements. With partial scans the reverse distance is especially sensitive to unseen reference regions.

## Implementation map

- `inspection/core.py`: rigid transforms, SVD fitting, preprocessing, registration, metrics, synthetic generator.
- `inspection/__main__.py`: CLI, input loading, plots, and file exports.
- `tests/test_core.py`: four regression tests for exact transform recovery, invalid inputs, no-overlap behaviour, and synthetic alignment/defect contrast.
- `requirements.txt`: lightweight dependencies.
- `requirements-open3d.txt`: optional geometric feature registration dependency.
- `VALIDATION.md`: recorded execution results and limitations.

## Limitations and useful next steps

- Synthetic scan and reference start from corresponding samples, making this easier than independent real scans. Noise, cleanup, and the dent still disturb correspondences.
- The baseline has no robust guarantee for partial overlap or symmetric geometry. ICP is locally optimized and depends on initialization.
- Statistical filtering leaves some outliers and can remove real defects. Try several parameters and show failure cases.
- Registration uses the potentially defective surface; deformation can bias alignment. A later improvement is robust registration on known stable regions.
- The demo evaluates distance contrast, not defect segmentation accuracy. Add independent scans and reliable point labels before reporting precision/recall.
- There is no sensor calibration, uncertainty budget, traceable reference, CAD fitting, or real industrial validation.
- Next: independent mesh resampling, missing-view experiments, point-to-mesh signed distances on watertight references, robust ICP, parameter sweeps, and real scan validation.

## Interview talking points

Explain the difference between global registration and ICP, why ICP needs an initial pose, why normals are needed for point-to-plane ICP, how voxel size trades detail for runtime, why symmetry creates ambiguity, and why distance thresholds are not measurement accuracy.

## Resume wording after you have run and understood the project

**3D Point Cloud Registration and Inspection | Python, Open3D, NumPy**

- Implemented point-cloud preprocessing and rigid registration, including a PCA/ICP baseline and an optional FPFH/RANSAC and point-to-plane ICP pipeline.
- Visualized unsigned geometric deviations using 3D heatmaps and reported alignment, overlap, and distance metrics.
- Validated the baseline with controlled synthetic transformations, noise, outliers, and a localized surface deformation.

Only list Open3D as hands-on experience after installing and running that backend yourself. Do not claim industrial accuracy, real scanned-data validation, or a production deployment from this synthetic demonstration.

## References

- Open3D global registration: https://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html
- Open3D ICP: https://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html
- SciPy cKDTree: https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.cKDTree.html

## License

MIT; see LICENSE. You can modify and publish this project. Review the code and reproduce the results before using it in an application or interview.
