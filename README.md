# 3D Point Cloud Registration and Inspection

A Python project that aligns a simulated 3D scan with a reference object, visualizes geometric deviations, and evaluates registration accuracy using a known transformation.

Built to explore 3D computer vision, geometric inspection, and reproducible evaluation.

![Open3D registration and deviation heatmap](results/open3d_voxel15/inspection.png)

## Features

- Synthetic reference and scan generation with noise, outliers, and an optional dent
- Voxel downsampling and statistical outlier filtering
- Two registration pipelines:
  - NumPy/SciPy: PCA initialization and trimmed point-to-point ICP
  - Open3D: FPFH features, RANSAC, and point-to-plane ICP
- Unsigned nearest-neighbour deviation heatmaps
- Registration and distance metrics exported as JSON
- Automated tests for core registration behaviour

## Technologies

Python, NumPy, SciPy, Open3D, Matplotlib, unittest

## How It Works

1. Generate an asymmetric reference object.
2. Create a simulated scan with a known rotation and translation.
3. Add noise, outliers, and optionally a localized dent.
4. Downsample and filter both point clouds.
5. Estimate the scan-to-reference transformation.
6. Measure nearest-neighbour distances after alignment.
7. Save plots, point clouds, transformations, and evaluation metrics.

Synthetic coordinates use millimetres.

## Setup

The NumPy pipeline was run successfully on Windows with Python 3.13.
The Open3D experiments used the updated dependency requirement
`open3d==0.20.0`.

### Windows Command Prompt

```cmd
py -3.13 -m venv .venv
.venv\Scripts\activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

For the Open3D backend:

```cmd
python -m pip install -r requirements-open3d.txt
```

Ensure `requirements-open3d.txt` contains:

```text
-r requirements.txt
open3d==0.20.0
```

## Run the Project

### NumPy baseline

```cmd
python -m inspection
```

### Open3D registration

The 1.5 mm voxel setting recovered the correct pose in the recorded experiment:

```cmd
python -m inspection --backend open3d --voxel 1.5 --threshold 2 --output results/open3d_voxel15
```

### No-dent comparison

```cmd
python -m inspection --backend open3d --voxel 1.5 --threshold 2 --no-defect --output results/open3d_voxel15_no_defect
```

### Run tests

```cmd
python -m unittest discover -s tests -v
```

Four automated tests cover known-transform recovery, invalid input,
zero overlap, and synthetic registration with defect contrast.
They test the NumPy baseline and shared functions, not the Open3D backend.

## Experimental Results

Recorded on synthetic data with seed 42 and 7,000 reference samples.

| Configuration | Rotation error (°) | Translation error (mm) | Inlier RMSE (mm) | Points above 0.75 mm |
|---|---:|---:|---:|---:|
| NumPy, dent, voxel 0.7 mm | 0.0054 | 0.0023 | 0.2871 | 2.89% |
| Open3D, dent, voxel 0.7 mm | 179.9864 | 35.9832 | 0.8028 | 43.25% |
| Open3D, dent, voxel 1.5 mm | 0.0556 | 0.0405 | 0.4898 | 16.35% |
| Open3D, no dent, voxel 1.5 mm | 0.0158 | 0.0076 | 0.4744 | 15.37% |

### Findings

- The NumPy baseline recovered the known pose closely in this controlled example.
- Open3D at 0.7 mm voxel size converged to an approximately 180° incorrect orientation despite high registration fitness.
- The object's approximate symmetry makes incorrect alignments plausible.
- Increasing the voxel size to 1.5 mm recovered the correct orientation in the recorded Open3D run.
- At matching Open3D settings, the dent increased the fraction above tolerance by approximately 0.98 percentage points.
- This small difference demonstrates deviation contrast, but does not establish reliable defect detection.

Changing voxel size also changes sampling density, so distance metrics across
different voxel settings are not directly equivalent.

## Understanding the Metrics

- **Fitness:** Fraction of scan points within the correspondence threshold of a reference point. It is not classification accuracy.
- **Inlier RMSE:** Root mean square nearest-neighbour distance for points inside that threshold.
- **Mean deviation:** Average unsigned nearest-neighbour distance across all cleaned scan points.
- **95th-percentile deviation:** Distance at or below which 95% of scan-point distances fall.
- **Fraction above tolerance:** Proportion of scan points exceeding the selected distance threshold.
- **Rotation and translation errors:** Differences between the estimated pose and the known synthetic ground truth.

A high fitness score alone does not guarantee correct alignment.

## Output Files

Each run saves its outputs in the selected results folder.

| File | Description |
|---|---|
| `inspection.png` | Before/after alignment and deviation heatmap |
| `deviation_histogram.png` | Distribution of unsigned distances |
| `metrics.json` | Registration and deviation metrics |
| `scan_to_reference.csv` | Estimated 4 × 4 rigid transformation |
| `deviations.csv` | Aligned coordinates, distances, and threshold flags |
| `aligned_scan.npy` | Cleaned, aligned scan |
| `reference_raw.npy` | Original reference cloud |
| `scan_raw.npy` | Original scan cloud |
| `ground_truth_scan_to_reference.csv` | Known synthetic transformation |
| `synthetic_defect_mask.npy` | Defect labels before preprocessing |

The synthetic defect mask indexes the original reference samples.
It does not directly index the cleaned scan or appended outliers.

## Use Your Own Data

CSV inputs must contain three numeric columns without a header: X, Y, Z.
NPY inputs must contain an N × 3 numeric array.

```cmd
python -m inspection --reference data/reference.csv --scan data/scan.csv --output results/custom
```

PLY and PCD inputs require Open3D.

If input coordinates are in metres, use `--scale-to-mm 1000`.
Both clouds must use the same units. The pipeline estimates rigid motion,
not scale.

## Project Structure

```text
3d-point-cloud-inspection/
├── inspection/
│   ├── __init__.py
│   ├── __main__.py
│   └── core.py
├── tests/
│   └── test_core.py
├── results/
├── requirements.txt
├── requirements-open3d.txt
├── README.md
├── VALIDATION.md
├── LICENSE
└── .gitignore
```

## Limitations

- Evaluation uses synthetic data, not real industrial scans.
- Reference and scan originate from shared samples, making registration easier than independently acquired scans.
- Approximate symmetry can cause incorrect registration.
- Nearest-neighbour distances depend on sampling density, noise, and alignment.
- Distances are unsigned and do not distinguish inward from outward displacement.
- Outliers remain after filtering; maximum deviation is not equivalent to dent depth.
- Threshold crossings are not validated defect labels or physical defective area.
- No sensor calibration, measurement uncertainty analysis, or certified metrology validation is included.

## Future Improvements

- Evaluate independently sampled surfaces and real scans.
- Test partial overlap and missing views.
- Add more distinctive geometry to study registration ambiguity.
- Separate coarse registration from dense-cloud inspection.
- Compare multiple seeds and parameter settings.
- Evaluate defect precision and recall using labels mapped to inspected points.

## References

- [Open3D Global Registration](https://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html)
- [Open3D ICP Registration](https://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html)
- [SciPy cKDTree](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.cKDTree.html)
