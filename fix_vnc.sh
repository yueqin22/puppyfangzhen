#!/bin/bash
# Fix VNC and start RViz

# Kill existing VNC processes
pkill -f 'x11vnc' 2>/dev/null || true
pkill -f 'Xvnc' 2>/dev/null || true
sleep 1

# Check available VNC tools
echo "=== Checking VNC tools ==="
which Xvnc 2>/dev/null && echo "Xvnc found" || echo "Xvnc not found"
which x11vnc 2>/dev/null && echo "x11vnc found" || echo "x11vnc not found"
which vncserver 2>/dev/null && echo "vncserver found" || echo "vncserver not found"

# Check if Xvfb is running on :99
echo ""
echo "=== Xvfb status ==="
ps aux | grep 'Xvfb :99' | grep -v grep

# Try approach 1: x11vnc with -rawfb
echo ""
echo "=== Trying x11vnc with -rawfb ==="
export DISPLAY=:99
export WAYLAND_DISPLAY=""
export XDG_SESSION_TYPE=""
export GDK_BACKEND=x11

# Get the Xvfb display dimensions
# Try rawfb approach - capture Xvfb directly
# First, let's try the simple approach: just force x11vnc to use the X display
x11vnc -display :99 -forever -nopw -rfbport 5900 -shared -noxdamage -noxrecord -noxfixes -nosel -noclipboard 2>&1 &
X11VNC_PID=$!
sleep 3

if kill -0 $X11VNC_PID 2>/dev/null; then
    echo "x11vnc started successfully (PID: $X11VNC_PID) on port 5900"
else
    echo "x11vnc failed, trying alternative approaches..."

    # Try approach 2: Xvnc (TigerVNC)
    if which Xvnc 2>/dev/null; then
        echo "Trying Xvnc..."
        # Kill Xvfb first
        pkill -f 'Xvfb :99' 2>/dev/null || true
        sleep 1
        # Start Xvnc
        Xvnc :99 -geometry 1920x1080 -depth 24 -SecurityTypes None -rfbport 5900 &
        XVNC_PID=$!
        sleep 2
        if kill -0 $XVNC_PID 2>/dev/null; then
            echo "Xvnc started successfully (PID: $XVNC_PID) on port 5900"
            export DISPLAY=:99
        else
            echo "Xvnc also failed!"
        fi
    else
        echo "Xvnc not available. Installing tigervnc-standalone-server..."
        sudo apt-get install -y tigervnc-standalone-server 2>/dev/null
        if which Xvnc 2>/dev/null; then
            pkill -f 'Xvfb :99' 2>/dev/null || true
            sleep 1
            Xvnc :99 -geometry 1920x1080 -depth 24 -SecurityTypes None -rfbport 5900 &
            sleep 2
            echo "Xvnc installed and started on port 5900"
            export DISPLAY=:99
        else
            echo "Failed to install Xvnc. Trying x11vnc with -create..."
            x11vnc -create -forever -nopw -rfbport 5900 -shared -display :99 2>&1 &
            sleep 3
            echo "x11vnc -create attempted"
        fi
    fi
fi

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

nohup rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true > /home/veni/rviz.log 2>&1 &
RVIZ_PID=$!
sleep 5

if kill -0 $RVIZ_PID 2>/dev/null; then
    echo "RViz started successfully (PID: $RVIZ_PID)"
else
    echo "RViz failed to start. Check /home/veni/rviz.log"
    tail -20 /home/veni/rviz.log 2>/dev/null
fi

# Final status
echo ""
echo "=== Final Process Status ==="
ps aux | grep -E 'x11vnc|Xvnc|Xvfb|rviz2|gzserver' | grep -v grep | awk '{print $2, $11, $12, $13}'

echo ""
echo "=== VNC port check ==="
ss -tlnp 2>/dev/null | grep 5900 || netstat -tlnp 2>/dev/null | grep 5900 || echo "Port 5900 not found"
