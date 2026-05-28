# OpenArm LeRobot Recording Guide

Complete walkthrough of recording, fixing, and training on bimanual OpenArm datasets via XR teleop.

---

## 1. Parallel Dataset Recording

**Problem:** Only doing right-arm pick-and-place but wanted to record both a full bimanual dataset AND a right-arm-only dataset simultaneously without running two separate scripts.

**Solution:** Added `--repo-id-right` and `--repo-id-left` flags to `lerobot_recorder.py`. Every `_tick()` call writes to all active datasets at the same time under a shared lock. Right-only uses joint index slicing (`_RIGHT_IDX`) to extract the 8 joints from the 16-joint state/action arrays.

**Command:**
```bash
python scripts/lerobot_recorder.py \
    --repo-id 20-wasa/openarm-cube-pickup \
    --repo-id-right 20-wasa/openarm-cube-pickup-right \
    --fps 30 \
    --root ~/lerobot_datasets \
    --push-to-hub
```

---

## 2. Camera Shape Mismatch

**Problem:**
```
ValueError: feature 'observation.images.head' shape '(480, 640, 3)'
does not match expected '(720, 1280, 3)'
```

**Solution:** V4L2 silently delivers 480×640 regardless of what was declared. Removed separate `HEAD_H, HEAD_W` variables — all three cameras now use `IMG_H, IMG_W = 480, 640`.

---

## 3. HuggingFace Dataset Viewer Not Working

**Problem:** After `push_to_hub()`, the HF Dataset Viewer showed `SplitsNotFoundError` / `Parquet magic bytes not found`.

**Solution:** HuggingFace xet storage stores a tiny pointer blob instead of real parquet bytes for some files. Re-uploading via `api.upload_file()` bypasses xet and stores real content. Added `_reupload_parquets()` which runs automatically after every `push_to_hub()`, covering:
- `data/chunk-*/file-*.parquet`
- `meta/episodes/chunk-*/file-*.parquet`
- `meta/tasks.parquet`

---

## 4. lerobot Space Visualizer "Episode 0 not found in metadata"

**Problem:** `https://huggingface.co/spaces/lerobot/visualize_dataset` showed "Episode 0 not found in metadata" even after the Dataset Viewer was fixed.

**Root cause:** The Space (a Next.js app) reads `meta/episodes/chunk-000/file-000.parquet` directly via HTTP. When that file was an xet pointer, parsing silently failed and no episodes were found.

**Solution:** Two fixes applied:
1. Re-upload `meta/episodes/chunk-000/file-000.parquet` as real bytes (covered by `_reupload_parquets()` above).
2. Also generate and upload a flat `meta/episodes.parquet` (older lerobot Space compatibility). This is now done automatically inside `_reupload_parquets()` after every push.

---

## 5. Resuming Recording into an Existing Dataset

**Problem:** After recording 20 episodes and pushing, wanted to add more episodes without losing the existing ones.

**Solution:** Added `--resume` flag. Uses `LeRobotDataset.resume()` which appends new episodes starting from where the dataset left off.

**Command:**
```bash
python scripts/lerobot_recorder.py \
    --repo-id 20-wasa/openarm-cube-pickup \
    --repo-id-right 20-wasa/openarm-cube-pickup-right \
    --resume \
    --push-to-hub \
    --root ~/lerobot_datasets
```

---

## 6. Save / Discard / Failure Choice After Each Episode

**Problem:** No way to discard a bad demo or label a failure episode — every recording was saved unconditionally.

**Solution:** `stop_episode()` now only stops recording without saving. After stopping, the loop asks:

```
  What to do with this episode?
    s / ENTER  — save (success)
    f          — save as FAILURE (keeps data, marks task)
    d          — discard (bad demo, throw away)

  Choice [s/f/d]:
```

- **s / ENTER** → `save_episode()` — saved normally
- **f** → saved with task prefixed `"FAILURE: <task>"` — data kept, clearly labelled
- **d** → `discard_episode()` → `clear_episode_buffer()` on all datasets — nothing written

---

## 7. Deleting a Bad Episode from an Existing Dataset

**Problem:** Episode 1 was a bad recording and needed to be removed from both datasets before training.

**Solution:** Used lerobot's built-in `lerobot-edit-dataset` tool (re-encodes videos, re-indexes all remaining episodes):

```bash
# In the lerobot venv
.venv/bin/lerobot-edit-dataset \
    --repo_id 20-wasa/openarm-cube-pickup \
    --root ~/lerobot_datasets/20-wasa/openarm-cube-pickup \
    --operation.type delete_episodes \
    --operation.episode_indices "[1]"
```

The edited dataset saves to `~/.cache/huggingface/lerobot/`. Copy it back:

```bash
rsync -a --delete \
    ~/.cache/huggingface/lerobot/20-wasa/openarm-cube-pickup/ \
    ~/lerobot_datasets/20-wasa/openarm-cube-pickup/
```

Then re-upload everything (parquets + videos + meta + README) via `api.upload_file()` to avoid xet corruption on the updated files.

---

## 8. Training with ACT

**Problem:** `lerobot-train` on the laptop (RTX 4050, 6 GB) gave OOM errors with default batch size.

**Solution:** Moved training to the workstation (RTX 6000 Blackwell). Used `batch_size=32`.

---

### 8a. xet: `lerobot-train` crashes with "Parquet magic bytes not found"

**Problem:** On the workstation, `lerobot-train` failed immediately:
```
ArrowInvalid: Parquet magic bytes not found in footer.
```
HF's `snapshot_download` pulled stale xet pointer blobs instead of real parquet bytes.

**Solution:** Download datasets using direct HTTP (bypasses xet), then pass `--dataset.root`:

```bash
# Update download_datasets.py with new repo names, then:
python ~/OPEN_ARM/teleop_xr/scripts/download_datasets.py
```

Always use `--dataset.root` pointing to the local download when training.

---

### 8b. xet: recorder crashes after push with "Parquet magic bytes not found"

**Problem:** After 61 episodes, `lerobot_recorder.py` crashed at the end of `_reupload_parquets()`:
```
pyarrow.lib.ArrowInvalid: Parquet magic bytes not found in footer
```
After `push_to_hub()`, the local `meta/episodes/chunk-000/file-000.parquet` gets overwritten with an xet pointer blob. The recorder then tried to read it to build the flat `meta/episodes.parquet` and crashed.

**Solution:** Changed `_reupload_parquets()` to read from `dataset.meta.episodes` (in-memory HF Dataset object) instead of the local file — no disk read needed.

---

### 8c. xet: "Episode 0 not found in metadata" in lerobot Space after push

**Problem:** Even after fixing 8b, the lerobot Space visualizer showed "Episode 0 not found in metadata" for the bimanual dataset. The right-only worked because its re-upload sent genuinely new bytes; the bimanual's re-upload was skipped ("No files have been modified since last commit") because HF saw identical bytes (both were xet pointers).

**Root cause:** HF xet serves real bytes on CDN-cached requests but pointer blobs on origin. The Space's JS fetch hits origin → gets pointer → fails to parse.

**Solution:** Re-serialise the parquet through pyarrow with snappy compression before uploading. This produces different bytes from the original (different compression = different hash) → forces HF to accept a new commit → file stored as fresh content. Now done automatically in `_reupload_parquets()` for every file.

---

### 8d. xet: `lerobot-doctor` fails with "Could not read schema"

**Problem:**
```
Error creating dataset. Could not read schema from 'data/chunk-000/file-000.parquet'.
Parquet magic bytes not found in footer.
```
`data/chunk-000/file-000.parquet` (the main frames parquet) was also stored as an xet pointer on HF — only the episodes parquet had been fixed.

**Solution:** Re-serialise and re-upload `data/chunk-000/file-000.parquet` the same way (pyarrow snappy re-encode → upload). Fixed for existing datasets manually; new recordings handled automatically by the updated `_reupload_parquets()`.

---

### 8e. Training commands (use these — always with `--dataset.root`)

**Update `download_datasets.py`** with the new repo names before downloading:
```python
DATASETS = [
    "20-wasa/openarm-cube-pickup-20260528",
    "20-wasa/openarm-cube-pickup-right-20260528",
]
```

#### Workstation (RTX 6000 Blackwell — recommended)

```bash
conda activate teleop_xr
cd ~/Biswash/lerobot

# First time: download datasets
python ~/OPEN_ARM/teleop_xr/scripts/download_datasets.py   # update DATASETS list first

# Train both (runs sequentially, pushes each to HF when done)
lerobot-train \
    --dataset.repo_id=20-wasa/openarm-cube-pickup-20260528 \
    --dataset.root=~/Biswash/lerobot_datasets/20-wasa/openarm-cube-pickup-20260528 \
    --policy.type=act \
    --policy.device=cuda \
    --batch_size=32 \
    --steps=50000 \
    --save_freq=5000 \
    --job_name=act_openarm_cube_pickup_20260528 \
    --output_dir=outputs/train/act_openarm_cube_pickup_20260528 \
    --policy.push_to_hub=true \
    --policy.repo_id=20-wasa/act_openarm_cube_pickup_20260528 && \
lerobot-train \
    --dataset.repo_id=20-wasa/openarm-cube-pickup-right-20260528 \
    --dataset.root=~/Biswash/lerobot_datasets/20-wasa/openarm-cube-pickup-right-20260528 \
    --policy.type=act \
    --policy.device=cuda \
    --batch_size=32 \
    --steps=50000 \
    --save_freq=5000 \
    --job_name=act_openarm_cube_pickup_right_20260528 \
    --output_dir=outputs/train/act_openarm_cube_pickup_right_20260528 \
    --policy.push_to_hub=true \
    --policy.repo_id=20-wasa/act_openarm_cube_pickup_right_20260528
```

#### Laptop (RTX 4050 6 GB — use batch_size=4 only)

```bash
source ~/lerobot/.venv/bin/activate
cd ~/lerobot

lerobot-train \
    --dataset.repo_id=20-wasa/openarm-cube-pickup-20260528 \
    --dataset.root=~/lerobot_datasets/20-wasa/openarm-cube-pickup-20260528 \
    --policy.type=act \
    --policy.device=cuda \
    --batch_size=4 \
    --steps=50000 \
    --save_freq=5000 \
    --job_name=act_openarm_cube_pickup_20260528 \
    --output_dir=outputs/train/act_openarm_cube_pickup_20260528 \
    --policy.push_to_hub=true \
    --policy.repo_id=20-wasa/act_openarm_cube_pickup_20260528 && \
lerobot-train \
    --dataset.repo_id=20-wasa/openarm-cube-pickup-right-20260528 \
    --dataset.root=~/lerobot_datasets/20-wasa/openarm-cube-pickup-right-20260528 \
    --policy.type=act \
    --policy.device=cuda \
    --batch_size=4 \
    --steps=50000 \
    --save_freq=5000 \
    --job_name=act_openarm_cube_pickup_right_20260528 \
    --output_dir=outputs/train/act_openarm_cube_pickup_right_20260528 \
    --policy.push_to_hub=true \
    --policy.repo_id=20-wasa/act_openarm_cube_pickup_right_20260528
```

Run overnight in tmux:
```bash
tmux new -s train
# activate env + cd, paste commands above
# Ctrl+B then D to detach
# tmux attach -t train to check tomorrow
```

---

## Quick Reference

| Goal | Command |
|---|---|
| Record new dataset | `python scripts/lerobot_recorder.py --repo-id ... --repo-id-right ... --push-to-hub` |
| Add more episodes | add `--resume` to the command above |
| Delete bad episode | `lerobot-edit-dataset --operation.type delete_episodes --operation.episode_indices "[N]"` |
| Download for training | `python download_datasets.py` |
| Train both models | use the `&&` chained `lerobot-train` commands above |
| Check training | `tmux attach -t train` |
| Visualize dataset | `https://huggingface.co/spaces/lerobot/visualize_dataset?path=20-wasa/openarm-cube-pickup` |

---

## Files

| File | Purpose |
|---|---|
| `scripts/lerobot_recorder.py` | Main recorder — parallel datasets, save/discard/failure, resume, push |
| `scripts/download_datasets.py` | Direct HTTP download to bypass xet corruption |
| `scripts/train_openarm_act.sh` | Sequential ACT training script (laptop, uses lerobot venv) |
| `~/lerobot_datasets/20-wasa/openarm-cube-pickup/` | Local bimanual dataset |
| `~/lerobot_datasets/20-wasa/openarm-cube-pickup-right/` | Local right-only dataset |
