import serial
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# serial setup for reading scan output from the board
COM_LINK   = 'COM4'
UART_SPEED = 115200
MOVE_STEP_MM = 100   # 10 cm gap between scan positions

print(f"Connecting to {COM_LINK} at {UART_SPEED} baud...")
serial_pipe = serial.Serial(COM_LINK, UART_SPEED, timeout=10)
print("Connected! Press push button #1 (PM0) to start data aquistion.")
print("Waiting for data...\n")

x_pts = []
y_pts = []
z_pts = []

try:
    while True:
        raw_line = serial_pipe.readline().decode('utf-8', errors='ignore').strip()

        if not raw_line:
            continue

        print(f"Received: {raw_line}")

        # check for the final message before parsing the normal csv lines
        if "ALL SCANS COMPLETE" in raw_line:
            print("\nAll scans received! Generating 3D plot...")
            break

        # skip any status text lines becuse those are not point data
        if any(ch.isalpha() for ch in raw_line.split(',')[0]):
            continue

        point_parts = raw_line.split(',')
        if len(point_parts) != 3:
            continue

        try:
            scan_id     = int(point_parts[0])
            angle_store = int(point_parts[1])
            dist_mm_val = int(point_parts[2])
        except ValueError:
            continue

        # ignore bad readings so the graph doesent get fake points
        if dist_mm_val <= 0 or dist_mm_val > 4000:
            continue

        # convert saved angle into degreses first then radians for trig
        angle_deg_val = angle_store / 100.0
        angle_rad_val = math.radians(angle_deg_val)

        # converts polar scan data into y-z coordinates
        y_coord = dist_mm_val * math.cos(angle_rad_val)
        z_coord = dist_mm_val * math.sin(angle_rad_val)

        # x position depends on which scan number it came from
        x_coord = (scan_id - 1) * MOVE_STEP_MM

        x_pts.append(x_coord)
        y_pts.append(y_coord)
        z_pts.append(z_coord)

except KeyboardInterrupt:
    print("\nInterrupted.")

finally:
    serial_pipe.close()
    print("Serial port closed.")

# make the final 3D plot after all data is collected
if len(x_pts) == 0:
    print("No data received.")
else:
    fig = plt.figure(figsize=(10, 7))
    ax  = fig.add_subplot(111, projection='3d')

    scan_colors = ['blue', 'red', 'green']
    for scan_idx in range(1, 4):
        # pull out only the points from one scan position
        xs_now = [x_pts[i] for i in range(len(x_pts)) if x_pts[i] == (scan_idx - 1) * MOVE_STEP_MM]
        ys_now = [y_pts[i] for i in range(len(y_pts)) if x_pts[i] == (scan_idx - 1) * MOVE_STEP_MM]
        zs_now = [z_pts[i] for i in range(len(z_pts)) if x_pts[i] == (scan_idx - 1) * MOVE_STEP_MM]

        if xs_now:
            ax.scatter(xs_now, ys_now, zs_now,
                       c=scan_colors[scan_idx - 1],
                       label=f'Scan {scan_idx} (x={(scan_idx - 1) * MOVE_STEP_MM}mm)',
                       s=20)

            # if there are many points connect them so shape is easier too see
            if len(xs_now) > 1:
                xs_wrap = xs_now + [xs_now[0]]
                ys_wrap = ys_now + [ys_now[0]]
                zs_wrap = zs_now + [zs_now[0]]
                ax.plot(xs_wrap, ys_wrap, zs_wrap,
                        c=scan_colors[scan_idx - 1], alpha=0.5, linewidth=1)

    ax.set_xlabel('X - Displacement (mm)')
    ax.set_ylabel('Y - Distance (mm)')
    ax.set_zlabel('Z - Height (mm)')
    ax.set_title('3D Spatial Map')
    ax.legend()

    plt.tight_layout()
    plt.savefig('scan_3d.png', dpi=150)
    print("Plot saved as scan_3d.png")
    plt.show()