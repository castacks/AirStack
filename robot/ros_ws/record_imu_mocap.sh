#!/bin/bash

# Get current timestamp for folder name
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
SAVE_DIR="/data/sync_record_${TIMESTAMP}"

# Create main directory
mkdir -p "$SAVE_DIR"
echo "========================================="
echo "Data will be saved to: $SAVE_DIR"

# Get precise nanosecond clock offset before recording starts
OFFSET_NS=$(python3 -c "import time; print(int((time.time() - time.monotonic()) * 1e9))")
echo "Calculated Clock Offset: $OFFSET_NS ns"
echo "========================================="

# Define cleanup and auto-sync function
cleanup() {
    # ARMOR: Ignore all subsequent Ctrl+C presses to prevent killing the sync process
    trap "" SIGINT

    echo ""
    echo "Caught Ctrl+C! Stopping recordings gracefully..."
    kill -SIGINT $VOXL_PID $ROS_PID
    
    # Wait for processes to exit
    wait $VOXL_PID
    wait $ROS_PID
    
    # ARMOR: Force OS to flush all disk buffers physically
    sync
    sleep 0.5
    
    echo "Processing data and aligning timestamps... (DO NOT press Ctrl+C)"
    
    # Locate the nested data.csv and define output targets
    IMU_CSV=$(find "${SAVE_DIR}/imu_log" -name "data.csv" | head -n 1)
    SYNCED_FRD_CSV="${SAVE_DIR}/imu_synced_epoch.csv"
    SYNCED_FLU_CSV="${SAVE_DIR}/imu_synced_flu.csv"
    
    if [ -n "$IMU_CSV" ] && [ -f "$IMU_CSV" ]; then
        # ARMOR: Python script to handle single-pass dual-file generation
        python3 -c '
import sys, csv

infile_path = sys.argv[1]
out_frd_path = sys.argv[2]
out_flu_path = sys.argv[3]
offset = int(sys.argv[4])

success_count = 0
error_count = 0

with open(infile_path, "r") as infile, \
     open(out_frd_path, "w") as out_frd, \
     open(out_flu_path, "w") as out_flu:
     
    reader = csv.reader(infile)
    writer_frd = csv.writer(out_frd)
    writer_flu = csv.writer(out_flu)
    
    header = next(reader, None)
    if header:
        writer_frd.writerow(header)
        writer_flu.writerow(header)
            
    for row in reader:
        try:
            # Ensure row has enough columns (i, ts, batch, AX, AY, AZ, GX, GY, GZ, T)
            if row and len(row) > 8:
                # 1. Update the timestamp to UNIX Epoch
                synced_ts = str(int(row[1]) + offset)
                row[1] = synced_ts
                
                # Write the standard FRD row
                writer_frd.writerow(row)
                
                # 2. Convert to FLU (Invert AY, AZ, GY, GZ)
                row_flu = list(row)
                row_flu[4] = f"{-float(row[4]):.6f}"
                row_flu[5] = f"{-float(row[5]):.6f}"
                row_flu[7] = f"{-float(row[7]):.6f}"
                row_flu[8] = f"{-float(row[8]):.6f}"
                
                # Write the FLU row
                writer_flu.writerow(row_flu)
                
                success_count += 1
        except Exception:
            error_count += 1
            continue

print(f"    -> Sync result: {success_count} rows processed, {error_count} skipped.")
' "$IMU_CSV" "$SYNCED_FRD_CSV" "$SYNCED_FLU_CSV" "$OFFSET_NS"

        echo "Auto-Sync and FRD->FLU conversion complete!"
        echo "Output 1 (FRD): $SYNCED_FRD_CSV"
        echo "Output 2 (FLU): $SYNCED_FLU_CSV"
    else
        echo "Error: Could not find raw data.csv."
    fi
    
    echo "All done! Safe to copy data."
    exit 0
}

# Bind cleanup function to Ctrl+C
trap cleanup SIGINT

# ---------------------------------------------------------
# Start background recordings
# ---------------------------------------------------------
echo "Starting voxl-logger (IMU @ ~500Hz)..."
voxl-logger -i imu_apps -d "${SAVE_DIR}/imu_log/" &
VOXL_PID=$!

echo "Starting ros2 bag record (Mocap)..."
ros2 bag record -o "${SAVE_DIR}/mocap_bag" /modalai/pose &
ROS_PID=$!

echo "------------------------------------------------------"
echo "RECORDING IN PROGRESS..."
echo "Press [Ctrl+C] to stop, save, and auto-sync."
echo "------------------------------------------------------"

# Wait for user interruption
wait