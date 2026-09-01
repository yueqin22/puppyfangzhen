#!/bin/bash
# Fix VNC without sudo - download and extract TigerVNC

echo "=== Checking WSLg display ==="
echo "DISPLAY=$DISPLAY"
echo "WAYLAND_DISPLAY=$WAYLAND_DISPLAY"
ls -la /tmp/.X11-unix/ 2>/dev/null
ls -la /mnt/wslg/ 2>/dev/null | head -10

# Try approach 1: Use WSLg directly (no VNC needed)
echo ""
echo "=== Trying WSLg display ==="
# WSLg usually sets DISPLAY to :0 or similar
# Try running a simple X app to test
if [ -n "$DISPLAY" ] && [ "$DISPLAY" != ":99" ]; then
    echo "WSLg DISPLAY=$DISPLAY detected. Testing..."
    xset q 2>/dev/null && echo "WSLg X server is working!" || echo "WSLg X server not responding"
fi

# Try approach 2: Download and extract TigerVNC
echo ""
echo "=== Downloading TigerVNC (no sudo needed) ==="
cd /tmp
apt download tigervnc-standalone-server 2>/dev/null
if ls tigervnc-standalone-server*.deb 1>/dev/null 2>&1; then
    echo "Downloaded TigerVNC package"
    mkdir -p /home/veni/tigervnc
    dpkg -x tigervnc-standalone-server*.deb /home/veni/tigervnc
    XVNC_PATH=$(find /home/veni/tigervnc -name "Xvnc" -type f 2>/dev/null | head -1)
    if [ -n "$XVNC_PATH" ]; then
        echo "Found Xvnc at: $XVNC_PATH"

        # Kill Xvfb and x11vnc
        pkill -f 'Xvfb :99' 2>/dev/null || true
        pkill -f 'x11vnc' 2>/dev/null || true
        sleep 1

        # Start Xvnc
        $XVNC_PATH :99 -geometry 1920x1080 -depth 24 -SecurityTypes None -rfbport 5900 &
        XVNC_PID=$!
        sleep 2

        if kill -0 $XVNC_PID 2>/dev/null; then
            echo "Xvnc started successfully on port 5900 (PID: $XVNC_PID)"
            export DISPLAY=:99
        else
            echo "Xvnc failed to start"
        fi
    else
        echo "Xvnc binary not found in package"
    fi
else
    echo "Failed to download TigerVNC package"
    echo "Trying x11vnc with -rawfb..."

    # Try approach 3: x11vnc with rawfb
    # Get the Xvfb framebuffer info
    export DISPLAY=:99
    export WAYLAND_DISPLAY=""
    export XDG_SESSION_TYPE=""

    # Try to use x11vnc with -rawfb to capture Xvfb directly
    # The -rawfb option bypasses Wayland detection
    xdpyinfo 2>/dev/null | grep -E "dimensions|depth"

    # Kill existing x11vnc
    pkill -f 'x11vnc' 2>/dev/null || true
    sleep 1

    # Start x11vnc with -rawfb
    # Format: rawfb:WxH:BPL:DEPTH:ADDR
    # For Xvfb :99 with 1920x1080x24:
    x11vnc -rawfb display:99 -forever -nopw -rfbport 5900 -shared 2>&1 &
    X11_PID=$!
    sleep 3

    if kill -0 $X11_PID 2>/dev/null; then
        echo "x11vnc with -rawfb started (PID: $X11_PID)"
    else
        echo "x11vnc -rawfb also failed"
        # Last resort: try x11vnc -create
        x11vnc -create -forever -nopw -rfbport 5900 -shared 2>&1 &
        sleep 3
        echo "x11vnc -create attempted"
    fi
fi

# Check if any VNC server is running on port 5900
echo ""
echo "=== Port 5900 check ==="
ss -tlnp 2>/dev/null | grep 5900 || echo "Port 5900 not listening"

# Start RViz
echo ""
echo "=== Starting RViz ==="
source /opt/ros/humble/setup.bash
source /home/veni/puppy_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export DISPLAY=:99

RVIZ_CONFIG="/home/veni/puppy_ws/install/puppy_bringup/share/puppy_bringup/rviz/navigation.rviz"
if [ ! -f "$RVIZ_CONFIG" ]; then
    RVIZ_CONFIG="/home/veni/puppy_ws/install/puppy_nav/share/puppy_nav/rviz/navigation.rviz"
fi
echo "Using RViz config: $RVIZ_CONFIG"
echo "DISPLAY=$DISPLAY"

nohup rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true > /home/veni/rviz.log 2>&1 &
RVIZ_PID=$!
sleep 5

if kill -0 $RVIZ_PID 2>/dev/null; then
    echo "RViz started (PID: $RVIZ_PID)"
else
    echo "RViz failed. Log:"
    tail -10 /home/veni/rviz.log 2>/dev/null
fi

# Final status
echo ""
echo "=== Final Status ==="
ps aux | grep -E 'Xvnc|x11vnc|Xvfb|rviz2|gzserver' | grep -v grep | awk '{print $2, $11, $12, $13}'
echo ""
ss -tlnp 2>/dev/null | grep 5900
