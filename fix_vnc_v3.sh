#!/bin/bash
# Multi-approach VNC fix

echo "=== Approach 1: Find Xvnc in extracted package ==="
find /home/veni/tigervnc -name "*vnc*" -type f 2>/dev/null
find /home/veni/tigervnc -name "Xvnc" 2>/dev/null
ls -la /home/veni/tigervnc/usr/bin/ 2>/dev/null

echo ""
echo "=== Approach 2: Try x11vnc with WSLg display (:0) ==="
# Kill existing
pkill -f 'x11vnc' 2>/dev/null || true
sleep 1

# Try with DISPLAY=:0 (WSLg)
# Unset WAYLAND_DISPLAY to prevent Wayland detection
export DISPLAY=:0
export WAYLAND_DISPLAY=""
# Also try removing the wayland socket temporarily
WAYLAND_SOCKET="/run/user/$(id -u)/wayland-0"
if [ -e "$WAYLAND_SOCKET" ]; then
    echo "Wayland socket exists at $WAYLAND_SOCKET"
    # Don't remove it, just try x11vnc anyway
fi

# Try x11vnc with -display :0
timeout 5 x11vnc -display :0 -forever -nopw -rfbport 5900 -shared -noxdamage 2>&1 | head -10
X11VNC_PID=$(pgrep x11vnc)
if [ -n "$X11VNC_PID" ]; then
    echo "x11vnc running on :0 (PID: $X11VNC_PID)"
else
    echo "x11vnc on :0 failed"

    echo ""
    echo "=== Approach 3: Use Xvnc from apt directly ==="
    # Install to a temporary location
    cd /tmp
    apt download tigervnc-standalone-server 2>/dev/null
    DEB_FILE=$(ls tigervnc-standalone-server*.deb 2>/dev/null | head -1)
    if [ -n "$DEB_FILE" ]; then
        echo "Extracting $DEB_FILE..."
        mkdir -p /tmp/xvnc_extract
        dpkg -x "$DEB_FILE" /tmp/xvnc_extract
        echo "Contents:"
        find /tmp/xvnc_extract -name "Xvnc" -o -name "vncserver" 2>/dev/null
        ls /tmp/xvnc_extract/usr/bin/ 2>/dev/null

        XVNC_BIN=$(find /tmp/xvnc_extract -name "Xvnc" -type f 2>/dev/null | head -1)
        if [ -n "$XVNC_BIN" ]; then
            echo "Found Xvnc: $XVNC_BIN"
            # Kill Xvfb
            pkill -f 'Xvfb :99' 2>/dev/null || true
            sleep 1
            # Start Xvnc
            $XVNC_BIN :99 -geometry 1920x1080 -depth 24 -SecurityTypes None -rfbport 5900 &
            sleep 2
            if pgrep -f "Xvnc :99" > /dev/null; then
                echo "Xvnc started on port 5900!"
                export DISPLAY=:99
            fi
        fi
    fi
fi

echo ""
echo "=== Approach 4: Use WSLg directly (DISPLAY=:0) ==="
# If VNC still not working, try WSLg
if ! ss -tlnp 2>/dev/null | grep -q 5900; then
    echo "VNC not available. Using WSLg directly (DISPLAY=:0)"
    echo "GUI apps will appear as Windows windows."

    # Kill RViz that's on :99
    pkill -f 'rviz2' 2>/dev/null || true
    sleep 1

    # Start RViz on WSLg display
    source /opt/ros/humble/setup.bash
    source /home/veni/puppy_ws/install/setup.bash
    export ROS_DOMAIN_ID=0
    export ROS_LOCALHOST_ONLY=1
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export DISPLAY=:0
    export WAYLAND_DISPLAY=wayland-0

    RVIZ_CONFIG="/home/veni/puppy_ws/install/puppy_bringup/share/puppy_bringup/rviz/navigation.rviz"
    nohup rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true > /home/veni/rviz.log 2>&1 &
    sleep 5

    if pgrep -f 'rviz2' > /dev/null; then
        echo "RViz started on WSLg (DISPLAY=:0)"
        echo "RViz window should appear in Windows!"
    else
        echo "RViz failed on WSLg. Log:"
        tail -20 /home/veni/rviz.log 2>/dev/null
    fi
fi

echo ""
echo "=== Final Status ==="
echo "Port 5900:"
ss -tlnp 2>/dev/null | grep 5900 || echo "  Not listening"
echo ""
echo "Processes:"
ps aux | grep -E 'Xvnc|x11vnc|Xvfb|rviz2|gzserver' | grep -v grep | awk '{print $2, $11, $12, $13}'
