# Validation

Executed on 2026-10-16 with Python 3.12.14, NumPy 2.3.5, SciPy 1.17.0, and Matplotlib 3.10.8.

- `python -m unittest discover -s tests -v`: all four tests passed.
- `python -m inspection`: completed; output included in results/demo.
- `python -m inspection --no-defect --output results/no_defect`: completed.
- Optional Open3D backend: not executed because Open3D is not installed in this environment.

## Synthetic baseline results (seed 42, 7000 reference samples)

| Metric | Dent | No dent |
| --- | --- | --- |
| Rotation error (degrees) | 0.005433 | 0.005232 |
| Translation error (mm) | 0.002334 | 0.002777 |
| Inlier RMSE (mm) | 0.287064 | 0.246930 |
| Fraction above 0.75 mm | 0.028877 | 0.016071 |

These results verify this controlled example only. The larger threshold-crossing fraction in the dent run indicates distance contrast, not validated defect classification. Some outliers remain after filtering; maximum distances are approximately 12.37 mm in both runs. See README.md for limitations and metric definitions.
