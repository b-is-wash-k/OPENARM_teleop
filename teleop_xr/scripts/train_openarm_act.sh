#!/usr/bin/env bash
# Train ACT on both OpenArm datasets sequentially.
# Run with the lerobot venv already activated:
#
#   source ~/lerobot/.venv/bin/activate
#   cd ~/lerobot
#   bash ~/OPEN_ARM/teleop_xr/scripts/train_openarm_act.sh
#
# Logs saved to:
#   ~/lerobot/outputs/train/act_openarm_cube_pickup/train.log
#   ~/lerobot/outputs/train/act_openarm_cube_pickup_right/train.log
#
# Models pushed to HF when done:
#   20-wasa/act_openarm_cube_pickup
#   20-wasa/act_openarm_cube_pickup_right

set -eo pipefail

LEROBOT_TRAIN="$(dirname "$(which python)")/lerobot-train"
HF_USER="20-wasa"
STEPS=50000
BATCH=8
SAVE_FREQ=5000

# ── helper ───────────────────────────────────────────────────────────────────
run_training() {
    local name="$1"
    local repo_id="$2"
    local out_dir="$3"
    local model_repo="$4"

    local log="${out_dir}/train.log"

    echo ""
    echo "════════════════════════════════════════════════════════════"
    echo "  ${name}"
    echo "  dataset  : ${repo_id}"
    echo "  output   : ${out_dir}"
    echo "  hub push : ${model_repo}"
    echo "  log      : ${log}"
    echo "════════════════════════════════════════════════════════════"
    echo "  Started : $(date)"
    echo ""

    "$LEROBOT_TRAIN" \
        --dataset.repo_id="${repo_id}" \
        --policy.type=act \
        --policy.device=cuda \
        --batch_size=${BATCH} \
        --steps=${STEPS} \
        --save_freq=${SAVE_FREQ} \
        --job_name="${name}" \
        --output_dir="${out_dir}" \
        --policy.push_to_hub=true \
        --policy.repo_id="${model_repo}" \
        2>&1 | tee "${log}"

    echo ""
    echo "  Finished : $(date)"
    echo "  Model    : https://huggingface.co/${model_repo}"
    echo ""
}

# ── training runs ─────────────────────────────────────────────────────────────

START=$(date +%s)

run_training \
    "1/2  bimanual (16-joint)" \
    "${HF_USER}/openarm-cube-pickup" \
    "outputs/train/act_openarm_cube_pickup" \
    "${HF_USER}/act_openarm_cube_pickup"

run_training \
    "2/2  right-arm only (8-joint)" \
    "${HF_USER}/openarm-cube-pickup-right" \
    "outputs/train/act_openarm_cube_pickup_right" \
    "${HF_USER}/act_openarm_cube_pickup_right"

END=$(date +%s)
ELAPSED=$(( (END - START) / 60 ))

echo "════════════════════════════════════════════════════════════"
echo "  ALL DONE  —  total time: ${ELAPSED} min"
echo "  20-wasa/act_openarm_cube_pickup        (bimanual)"
echo "  20-wasa/act_openarm_cube_pickup_right  (right-arm only)"
echo "════════════════════════════════════════════════════════════"
