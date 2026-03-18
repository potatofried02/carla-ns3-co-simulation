#!/bin/bash

# Detect Windows Host IP (from WSL2 perspective)
# Method 1: Get default gateway from ip route (More reliable for direct connection)
export HOST_IP=$(ip route show | grep default | awk '{print $3}')
# Fallback if ip route fails
if [ -z "$HOST_IP" ]; then
    export HOST_IP=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')
fi
export CARLA_HOST=$HOST_IP
echo "Detected Windows Host IP: $HOST_IP"

# Locate Carla PythonAPI egg file
# Expected location: inside current directory under PythonAPI/carla/dist/
EGG_FILE=$(find $(pwd)/PythonAPI/carla/dist/ -name "carla-*-py*-linux-x86_64.egg" 2>/dev/null | head -n 1)

if [ -z "$EGG_FILE" ]; then
    echo "⚠️  WARNING: Carla PythonAPI egg file not found!"
    echo "   Please download 'CARLA_0.9.15_RSS_Linux.tar.gz' (or similar), extract the egg file,"
    echo "   and place it in: $(pwd)/PythonAPI/carla/dist/"
    echo "   If you installed carla via pip manually, you can ignore this."
else
    export PYTHONPATH=$PYTHONPATH:$EGG_FILE
    echo "✅ Found Carla egg: $EGG_FILE"
fi

# Activate Conda environment (Python 3.7)
# Note: Source conda.sh to enable 'conda activate'
# Assuming miniconda is installed in /home/ctd/miniconda3
CONDA_BASE=$(conda info --base)
source "$CONDA_BASE/etc/profile.d/conda.sh"

echo "🐍 Activating Conda environment: carla_py38..."
conda activate carla_py38

if [ $? -ne 0 ]; then
    echo "❌ Failed to activate conda environment 'carla_py38'."
    echo "   Please run: conda create -n carla_py38 python=3.8 -y && conda activate carla_py38 && pip install -r requirements.txt"
    exit 1
fi

RUN_ARGS=("$@")
if [ -n "$1" ] && [ -n "$2" ]; then
    case "$1" in
        1|s1|S1)
            RUN_ARGS=(--scene=california_urban --v0-leader=24.44 --v0-follower=24.44 --gap=18 --brake-leader=1.0 --brake-follower=1.0 --method="$2")
            shift 2
            RUN_ARGS+=("$@")
            echo "📋 Preset S1 (California Urban): v0=24.44 gap=18 method=$2"
            ;;
        2|s2|S2)
            RUN_ARGS=(--scene=california_highway --v0-leader=31.39 --v0-follower=31.39 --gap=23 --brake-leader=1.0 --brake-follower=1.0 --method="$2")
            shift 2
            RUN_ARGS+=("$@")
            echo "📋 Preset S2 (California Highway): v0=31.39 gap=23 method=$2"
            ;;
        3|s3|S3)
            RUN_ARGS=(--scene=default --v0-leader=24.44 --v0-follower=24.44 --gap=9 --brake-leader=1.0 --brake-follower=1.0 --method="$2")
            shift 2
            RUN_ARGS+=("$@")
            echo "📋 Preset S3 (Close-following): v0=24.44 gap=9 method=$2"
            ;;
        4|s4|S4)
            RUN_ARGS=(--scene=california_urban --v0-leader=24.44 --v0-follower=24.44 --gap=18 --brake-leader=1.0 --brake-follower=0.6 --method="$2")
            shift 2
            RUN_ARGS+=("$@")
            echo "📋 Preset S4 (Adverse Weather): v0=24.44 gap=18 brake-F=0.6 method=$2"
            ;;
        *)
            ;;
    esac
fi

# Run the simulation (preset-expanded args or pass-through all args)
echo "🚀 Starting simulation..."
python main.py "${RUN_ARGS[@]}"
