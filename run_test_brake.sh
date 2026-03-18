
export HOST_IP=$(ip route show | grep default | awk '{print $3}')
if [ -z "$HOST_IP" ]; then
    export HOST_IP=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')
fi
export CARLA_HOST=$HOST_IP

EGG_FILE=$(find $(pwd)/PythonAPI/carla/dist/ -name "carla-*-py*-linux-x86_64.egg" 2>/dev/null | head -n 1)
[ -n "$EGG_FILE" ] && export PYTHONPATH=$PYTHONPATH:$EGG_FILE

CONDA_BASE=$(conda info --base)
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate carla_py38

echo "Running test_brake.py (Task 3.1)..."
python test_brake.py
