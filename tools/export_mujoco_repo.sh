#!/bin/bash
# Export the MuJoCo version as a stand-alone repository tree (github.com/mastns/quad-bounding-mujoco).
#
#   tools/export_mujoco_repo.sh DEST      (DEST may be an existing checkout; its files are replaced)
#
# Paths stay as in this folder, so every exported file is identical to its copy here:
#   mujoco/                              model, Webots-API shim, runner, video HUD
#   Webots_FINAL*/controllers/<name>/    the two controllers it runs, unchanged
#   Webots_FINAL_2/worlds/textures/      the textures quad.xml uses
#   plots/, media/gif/                   the MuJoCo figures, the comparison with Webots, the README GIFs
# The README is this folder's mujoco/README.md behind a short introduction, with its links made
# relative to the new root.
set -euo pipefail
src=$(cd "$(dirname "$0")/.." && pwd)
dest=$(realpath -m "${1:?usage: export_mujoco_repo.sh DEST}")
mkdir -p "$dest"

rsync -a --delete --exclude data/ --exclude __pycache__/ "$src/mujoco/" "$dest/mujoco/"
for c in Webots_FINAL_2/controllers/controller_webots743_Oct_THALIS Webots_FINAL/controllers/controller_webots730_Feb_FINAL; do
  mkdir -p "$dest/$c"
  rsync -a --delete --exclude data/ --exclude __pycache__/ "$src/$c/" "$dest/$c/"
done
mkdir -p "$dest/Webots_FINAL_2/worlds/textures" "$dest/plots/comparisons" "$dest/media/gif"
cp "$src"/Webots_FINAL_2/worlds/textures/{studio_sky_*.png,floor_tiles.png,lane_5m.png,alum.png,ntua3.png} \
   "$dest/Webots_FINAL_2/worlds/textures/"
rsync -a --delete "$src/plots/mujoco_oct2014/" "$dest/plots/mujoco_oct2014/"
rsync -a --delete "$src/plots/mujoco_feb2013/" "$dest/plots/mujoco_feb2013/"
cp "$src"/plots/comparisons/webots_vs_mujoco.{png,pdf} "$dest/plots/comparisons/"
cp "$src"/media/gif/mujoco_*.gif "$dest/media/gif/"

{
  cat <<'EOF'
# Quadruped bounding on MuJoCo 4.0

The NTUA Control Systems Lab quadruped of the 2012–2014 bounding simulations (an MSc thesis, Webots 7
with MATLAB controllers) as a MuJoCo 4.0 model. It runs the original controllers, ported to Python
and unchanged from the Webots R2025a port at
[quad-bounding-webots](https://github.com/mastns/quad-bounding-webots), through a small Webots-API
shim: the same devices, frames and physics-plugin behaviour, so the controllers cannot tell the
engines apart.

| Oct 2014 controller (desired speed 0.9 m/s) | Feb 2013 thesis controller (0.7 m/s) |
|---|---|
| ![Oct 2014 controller bounding on MuJoCo](media/gif/mujoco_oct2014.gif) | ![Feb 2013 controller bounding on MuJoCo](media/gif/mujoco_feb2013.gif) |

| Folder | Contents |
|---|---|
| `mujoco/` | the model (`make_model.py` → `quad.xml`), the Webots-API shim, the runner and the video HUD; its [README](mujoco/README.md) is reproduced below |
| `Webots_FINAL_2/controllers/controller_webots743_Oct_THALIS/`, `Webots_FINAL/controllers/controller_webots730_Feb_FINAL/` | the two controllers it runs |
| `Webots_FINAL_2/worlds/textures/` | the textures the model shares with the Webots showcase worlds |
| `plots/` | the figures of the two MuJoCo runs and the Webots-versus-MuJoCo comparison |

The paths are those of the Webots port, so every file here is identical to its copy there; the tree
is exported from it by `tools/export_mujoco_repo.sh`.

EOF
  sed -e '1s/^# /## /' \
      -e '/^| Oct 2014 controller (desired speed 0.9 m\/s) |/,/^| !\[Oct 2014 controller bounding on MuJoCo\]/d' \
      -e 's#`\.\./plots/#`plots/#g' \
      -e 's#`\.\./Webots_FINAL_2/worlds/textures`#`Webots_FINAL_2/worlds/textures`#g' \
      "$src/mujoco/README.md"
} > "$dest/README.md"
printf '__pycache__/\n*.pyc\nmujoco/data/\n**/controllers/*/data/\n' > "$dest/.gitignore"
echo "exported to $dest"
