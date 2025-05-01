## [English](../../README.md) | Chinese Instructions

lerobot_alohamini, compared to the original lerobot, significantly enhances debugging capabilities and is adapted for AlohaMini robot hardware.

For debugging capabilities, see:
[Debug Command Summary](3_debug命令汇总.md)

## Preface
lerobot_alohamini is a branch forked from the lerobot repository. It retains all original lerobot code and adds a debug directory, AlohaMini-specific configuration files, and tutorial documentation.

Please note this tutorial uses AlohaMini Solo (1 master, 1 follower) as an example.

## Getting Started (Ubuntu)

*** It is strongly recommended to follow these steps in order ***

### 1. Preparation: Network Environment Test
```bash
curl https://www.google.com
curl https://huggingface.co
```

### 2. Clone the lerobot_alohamini repository
```bash
cd ~
git clone https://github.com/liyitenga/lerobot_alohamini.git
```

### 3. Serial Port Permissions
By default, you cannot access serial ports. To permanently add your user to the dialout group:
```bash
whoami
sudo usermod -a -G dialout <username>
sudo reboot
```

### 4. Install conda3 and Environment Dependencies

#### Install conda3
```bash
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh \
  -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh
~/miniconda3/bin/conda init bash
source ~/.bashrc
```

#### Initialize conda3
```bash
conda create -y -n lerobot_alohamini python=3.10
conda activate lerobot_alohamini
```

#### Install dependencies
```bash
cd ~/lerobot_alohamini
pip install -e ".[feetech]"

conda install -y -c conda-forge ffmpeg
pip uninstall -y opencv-python
conda install -y -c conda-forge "opencv>=4.10.0"

pip install -e ".[aloha, pusht]"
```

### 5. Configure Robot Arm Port
```bash
cd ~/lerobot_alohamini
python lerobot/scripts/find_motors_bus_port.py

# or list devices
ls /dev/ttyACM*
```

Edit `lerobot/common/robot_devices/robots/configs.py`, locate `So100RobotConfig`, and update the `port` value.

### 6. Configure Camera Port

1. Capture images to detect camera indices:
    ```bash
    python lerobot/common/robot_devices/cameras/opencv.py \
      --images-dir outputs/images_from_opencv_cameras
    ```
2. Edit `configs.py`:
    - Set `So100RobotConfig` camera indices.
    - Add additional cameras if needed.

### 7. Teleoperation Calibration and Testing

#### 7.1 Set Robot to Mid Position
```bash
python lerobot/debug/motors.py reset_motors_to_midpoint \
  --port /dev/ttyACM0
```

#### 7.2 Calibration

- **Factory Calibration** (recommended):
  ```bash
  mv ~/.cache/calibration/am_solo_bk ~/.cache/calibration/am_solo
  ```
- **Manual Calibration**:
  ```bash
  python lerobot/scripts/control_robot.py \
    --robot.type=so100 \
    --robot.cameras='{}' \
    --control.type=calibrate
  ```

#### 7.3 Teleoperation Test
```bash
python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --control.type=teleoperate
```
Without camera:
```bash
python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --robot.cameras='{}' \
  --control.type=teleoperate
```

### 8. Local Evaluation Test

**CPU:**
```bash
python lerobot/scripts/eval.py \
  --policy.path=lerobot/diffusion_pusht \
  --env.type=pusht \
  --eval.batch_size=10 \
  --eval.n_episodes=10 \
  --use_amp=false \
  --device=cpu
```

**CUDA:**
```bash
python lerobot/scripts/eval.py \
  --policy.path=lerobot/diffusion_pusht \
  --env.type=pusht \
  --eval.batch_size=10 \
  --eval.n_episodes=10 \
  --use_amp=false \
  --device=cuda
```

### 9. Collect Training Dataset

#### 9.1 Register and Configure Hugging Face Token
```bash
HF_USER=$(huggingface-cli whoami | head -n 1)
echo $HF_USER
git config --global credential.helper store
huggingface-cli login --token <your_token>
```

#### 9.2 Record Dataset
```bash
python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --control.type=record \
  --control.fps=30 \
  --control.single_task="so100_pick_taffy" \
  --control.repo_id=$HF_USER/so100_pick_taffy10 \
  --control.tags='["so100","so100_pick"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=60 \
  --control.reset_time_s=10 \
  --control.num_episodes=22 \
  --control.push_to_hub=true \
  --control.resume=false
```

### 10. Visualization
```bash
python lerobot/scripts/visualize_dataset_html.py \
  --repo-id $HF_USER/so100_bi_test
```

### 11. Replay Dataset
```bash
python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --control.type=replay \
  --control.fps=30 \
  --control.repo_id=$HF_USER/so100_bi_test \
  --control.episode=0
```

### 12. Local Training

**Act Policy:**
```bash
python lerobot/scripts/train.py \
  --dataset.repo_id=liyitenga/so100_pick_taffy10 \
  --policy.type=act \
  --output_dir=outputs/train/so100_pick_taffy10_act \
  --job_name=so100_pick_taffy10_act \
  --policy.device=cuda \
  --wandb.enable=false
```

**Diffusion Policy:**
```bash
python lerobot/scripts/train.py \
  --dataset.repo_id=liyitenga/so100_pick_taffy10 \
  --policy.type=diffusion \
  --output_dir=outputs/train/so100_pick_taffy10_diffusion \
  --policy.device=cuda \
  --wandb.enable=false
```

### 13. Remote Training
```bash
conda init
# restart shell
conda create -y -n lerobot python=3.10
conda activate lerobot
source /etc/network_turbo

git clone https://github.com/liyitenga/lerobot_alohamini.git
cd lerobot_alohamini
pip install -e ".[feetech,aloha,pusht]"

python lerobot/scripts/train.py \
  --dataset.repo_id=liyitenga/so100_pick_taffy10 \
  --policy.type=act \
  --output_dir=outputs/train/so100_pick_taffy10_act \
  --job_name=so100_pick_taffy10_act \
  --policy.device=cuda \
  --wandb.enable=false

sudo apt install filezilla -y
```

### 14. Evaluate Trained Model
```bash
python lerobot/scripts/control_robot.py \
  --robot.type=so100 \
  --control.type=record \
  --control.fps=30 \
  --control.single_task="Grasp a faffy and put it in the bin." \
  --control.repo_id=liyitenga/eval_so100_pick_taffy10_3 \
  --control.tags='["tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=60 \
  --control.reset_time_s=30 \
  --control.num_episodes=2 \
  --control.push_to_hub=false \
  --control.policy.path=outputs/train/so100_pick_taffy10_act/checkpoints/100000/pretrained_model
```