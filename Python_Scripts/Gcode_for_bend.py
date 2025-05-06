import pandas as pd
import matplotlib.pyplot as plt
import argparse
import os
import glob
import math

# Define groups of line IDs for simultaneous configuration
groups = [
    [1, 2, 3, 4],
    [5, 6, 7],
    [8],
    [9, 10, 11, 12, 13, 14],
    [15],
    [16],
    [17],
    [18],
    [19],
    [20],
]


def load_coordinates(file_path):
    df = pd.read_excel(file_path)
    df = df.rename(columns={
        'Start X': 'x_start',
        'Start Y': 'y_start',
        'Finish X': 'x_end',
        'Finish Y': 'y_end'
    })
    required = ['x_start', 'y_start', 'x_end', 'y_end']
    if not all(c in df.columns for c in required):
        missing = set(required) - set(df.columns)
        raise ValueError(f"Missing columns in Excel: {missing}")
    df['line_id'] = range(1, len(df) + 1)
    return df


def visualize_lines(df):
    plt.figure(figsize=(8, 6))
    for _, row in df.iterrows():
        xs, ys = [row.x_start, row.x_end], [row.y_start, row.y_end]
        plt.plot(xs, ys, marker='o')
        mid_x = (row.x_start + row.x_end) / 2
        mid_y = (row.y_start + row.y_end) / 2
        plt.text(mid_x, mid_y, str(row.line_id), fontsize=12, color='red')
    plt.xlabel('X (mm)')
    plt.ylabel('Y (mm)')
    plt.title('Bend Lines Overview')
    plt.axis('equal')
    plt.grid(True)
    plt.show()


def interactive_adjustment(df):
    df['angle_deg'] = None
    df['radius_mm'] = None
    df['passes'] = None
    df['q1'] = None

    for grp in groups:
        print(f"\nConfigure bends for lines: {grp}")
        angle = input("  Bend angle [deg] (default 90): ") or "90"
        radius = input("  Bend radius [mm] (default 2): ") or "2"
        passes = input("  Laser passes (default 42): ") or "42"
        q1 = input("  Q1 parameter (default 70): ") or "70"
        df.loc[df.line_id.isin(grp), ['angle_deg', 'radius_mm', 'passes', 'q1']] = [
            float(angle), float(radius), int(passes), int(q1)
        ]
    return df


def generate_gcode(df, travel_speed, bend_speed, dwell_time, extension, output_file):
    """
    Generate G-code where group [1,2,3,4] passes are interleaved by rotation;
    other lines processed sequentially. Extension applies at line ends.
    Alternates direction each pass: odd passes from start->end, even passes end->start.
    """
    first_group = groups[0]
    other_lines = [lid for lid in df.line_id if lid not in first_group]

    # Precompute row data
    rows = {row.line_id: row for _, row in df.iterrows()}

    def write_pass(r, pass_idx, f):
        # Base direction unit
        dx = r.x_end - r.x_start
        dy = r.y_end - r.y_start
        length = math.hypot(dx, dy)
        ux, uy = dx/length, dy/length
        # Extended endpoints
        sx0 = r.x_start - ux * extension
        sy0 = r.y_start - uy * extension
        ex0 = r.x_end   + ux * extension
        ey0 = r.y_end   + uy * extension
        # Offset normal
        nx, ny = -uy, ux
        step = r.radius_mm / r.passes if r.passes and r.radius_mm else 0
        max_off = step * (r.passes - 1)
        off = max_off - step * pass_idx
        # Compute offset endpoints
        sx = sx0 + nx * off
        sy = sy0 + ny * off
        ex = ex0 + nx * off
        ey = ey0 + ny * off
        # Alternate direction
        if pass_idx % 2 == 1:
            sx, sy, ex, ey = ex, ey, sx, sy
        # Move to start of this pass
        f.write(f"G1 X{sx:.2f} Y{sy:.2f} F{travel_speed} "
                f"(Line {r.line_id} pass {pass_idx+1}, off {off:.2f}mm)\n")
        f.write("M3 (Laser on)\n")
        # Bend move to the other point
        f.write(f"G1 X{ex:.2f} Y{ey:.2f} Q1={r.q1} F{bend_speed} "
                f"(Line {r.line_id} end)\n")
        f.write("M5 (Laser off)\n")
        f.write(f"G4 F{dwell_time} (Dwell)\n")

    with open(output_file, 'w') as f:
        f.write("G90 (Absolute positioning)\n")
        # Interleaved passes for first group
        max_passes = max(rows[lid].passes for lid in first_group)
        for p in range(max_passes):
            for lid in first_group:
                r = rows[lid]
                if p < r.passes:
                    write_pass(r, p, f)
        # Sequential for others
        for lid in other_lines:
            r = rows[lid]
            for p in range(r.passes):
                write_pass(r, p, f)
    print(f"G-code written to {output_file}")


def main():
    p = argparse.ArgumentParser(description="Laser bend G-code generator")
    p.add_argument('-i','--input', help="Excel file")
    p.add_argument('-o','--output', help="Output .nc file")
    p.add_argument('--travel_speed', type=float, default=4050)
    p.add_argument('--bend_speed',   type=float, default=3000)
    p.add_argument('--dwell_time',   type=float, default=6.0)
    p.add_argument('--extension',    type=float, default=0.5,
                   help="Extend each bend line (mm) at both ends")
    args = p.parse_args()

    if not args.input:
        lst = glob.glob('*.xlsx')
        args.input = lst[0] if len(lst)==1 else input("Excel path: ")
    if not os.path.isfile(args.input): raise FileNotFoundError
    if not args.output:
        args.output = os.path.splitext(args.input)[0] + "_ext.nc"

    df = load_coordinates(args.input)
    visualize_lines(df)
    df = interactive_adjustment(df)
    generate_gcode(df, args.travel_speed, args.bend_speed,
                   args.dwell_time, args.extension, args.output)

if __name__=='__main__': main()
