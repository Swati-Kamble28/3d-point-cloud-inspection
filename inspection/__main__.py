import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from .core import clean, transform, synthetic, register_numpy, register_open3d, evaluate, pose_error


def read_cloud(path):
    if path.suffix.lower() == '.npy':
        return np.load(path, allow_pickle=False)
    if path.suffix.lower() == '.csv':
        return np.loadtxt(path, delimiter=',')
    try:
        import open3d as o3d
    except ImportError as exc:
        raise RuntimeError('PLY/PCD files require requirements-open3d.txt') from exc
    return np.asarray(o3d.io.read_point_cloud(str(path)).points)


def plot(reference, scan, aligned, distances, tolerance, output):
    fig = plt.figure(figsize=(17, 5.8))
    fig.subplots_adjust(left=.045, right=.96, bottom=.1, top=.90, wspace=.12)
    limit = max(tolerance*2, float(np.quantile(distances, .99)))
    for i, title in enumerate(['Before registration', 'After registration', 'Unsigned surface deviation']):
        ax = fig.add_subplot(1, 3, i+1, projection='3d')
        if i < 2:
            ax.scatter(*reference[::3].T, s=1, alpha=.3, c='#2166ac', label='Reference')
            p = scan if i == 0 else aligned
            ax.scatter(*p[::3].T, s=1, alpha=.45, c='#e87928', label='Scan')
            ax.legend(loc='upper right', fontsize=8)
        else:
            im = ax.scatter(*aligned.T, c=distances, s=2, cmap='inferno', vmin=0, vmax=limit)
            fig.colorbar(im, ax=ax, shrink=.6, label='Nearest reference point (mm)')
        ax.set(title=title, xlabel='X (mm)', ylabel='Y (mm)', zlabel='Z (mm)')
        ax.tick_params(labelsize=8, pad=0)
        ax.set_box_aspect((1.6, 1, .7), zoom=.85)
        ax.view_init(elev=28, azim=55)
    fig.savefig(output/'inspection.png', dpi=160, bbox_inches='tight', pad_inches=.25)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 4), layout='constrained')
    ax.hist(distances, bins=70, color='#2166ac')
    ax.axvline(tolerance, color='#bc2727', linestyle='--', label=f'Threshold: {tolerance:g} mm')
    ax.set(xlabel='Unsigned nearest-neighbour distance (mm)', ylabel='Scan points', title='Deviation distribution')
    ax.legend()
    fig.savefig(output/'deviation_histogram.png', dpi=180)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description='3D registration and unsigned point-distance inspection')
    p.add_argument('--backend', choices=['numpy', 'open3d'], default='numpy')
    p.add_argument('--reference', type=Path)
    p.add_argument('--scan', type=Path)
    p.add_argument('--output', type=Path, default=Path('results/demo'))
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--points', type=int, default=7000)
    p.add_argument('--voxel', type=float, default=.7, help='Millimetres')
    p.add_argument('--threshold', type=float, default=2., help='Correspondence distance in mm')
    p.add_argument('--tolerance', type=float, default=.75, help='Exploratory deviation threshold in mm')
    p.add_argument('--scale-to-mm', type=float, default=1., help='Multiply input coordinates by this value')
    p.add_argument('--no-defect', action='store_true')
    args = p.parse_args()
    if bool(args.reference) != bool(args.scan):
        p.error('--reference and --scan must be supplied together')
    if min(args.voxel, args.threshold, args.tolerance, args.scale_to_mm) <= 0 or args.points < 100:
        p.error('Distances/scale must be positive and --points must be at least 100')
    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    truth = None
    if args.reference:
        reference = read_cloud(args.reference)*args.scale_to_mm
        scan = read_cloud(args.scan)*args.scale_to_mm
    else:
        reference, scan, truth, mask = synthetic(args.seed, args.points, not args.no_defect)
        np.save(out/'synthetic_defect_mask.npy', mask)
        np.savetxt(out/'ground_truth_scan_to_reference.csv', truth, delimiter=',')
    np.save(out/'reference_raw.npy', reference)
    np.save(out/'scan_raw.npy', scan)
    ref, src = clean(reference, args.voxel), clean(scan, args.voxel)
    if args.backend == 'open3d':
        matrix = register_open3d(src, ref, args.voxel, args.threshold, args.seed)
    else:
        matrix = register_numpy(src, ref, args.threshold)
    aligned = transform(src, matrix)
    metrics, distances = evaluate(aligned, ref, args.threshold, args.tolerance)
    metrics.update(backend=args.backend, seed=args.seed, voxel_mm=args.voxel,
                   input_reference_points=len(reference), input_scan_points=len(scan),
                   description='Unsigned nearest-neighbour point distances after preprocessing')
    if truth is not None:
        metrics.update(pose_error(matrix, truth))
    with (out/'metrics.json').open('w') as f:
        json.dump(metrics, f, indent=2, allow_nan=False)
    np.savetxt(out/'scan_to_reference.csv', matrix, delimiter=',')
    np.save(out/'aligned_scan.npy', aligned)
    np.savetxt(out/'deviations.csv', np.column_stack([aligned, distances, distances > args.tolerance]),
               delimiter=',', header='x_mm,y_mm,z_mm,unsigned_distance_mm,above_threshold', comments='')
    plot(ref, src, aligned, distances, args.tolerance, out)
    print(json.dumps(metrics, indent=2))
    print(f'Results: {out.resolve()}')


if __name__ == '__main__':
    main()
