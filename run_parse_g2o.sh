#!/usr/bin/env bash
set -e

G2O="/home/cyw/CYW/mapping/MR_SLAM/Mapping/src/global_manager/log/full_graph.g2o"
OUTDIR="/media/cyw/KESU/mapping_data_formal/results/group5/MR_SLAM"
PY="/home/cyw/CYW/mapping/MR_SLAM/parse_g2o_to_3_tum.py"


if [[ ! -d "${OUTDIR}" ]]; then
  echo "[ERROR] Output directory does not exist: ${OUTDIR}"
  exit 1
fi

# 1) 触发保存（需要的话取消注释）
# rostopic pub -1 /map_saving std_msgs/Bool "data: true"

# 2) 等待 g2o 文件生成（最多等 60 秒）
for i in $(seq 1 60); do
  [[ -f "${G2O}" ]] && break
  sleep 1
done
[[ -f "${G2O}" ]]


####### 解析位姿并输出为 TUM / KITTI 格式 #########

# 输出格式：kitti / tum / both
FORMAT="kitti"

# # 3) 解析并输出
# python3 "${PY}" \
#   --g2o "${G2O}" \
#   --outdir "${OUTDIR}" \
#   --symbols a,b,c \
#   --format "${FORMAT}" \
#   --timestamp index \
#   --kitti_order index


## # 3) 解析并输出（带时间戳版本）通過 keyframes 文件夾裡的時間戳來匹配

PY="/home/cyw/CYW/mapping/MR_SLAM/g2o_to_3robots_tum_with_stamp.py"
KEYFRAMES_DIR="/home/cyw/CYW/mapping/MR_SLAM/Mapping/src/global_manager/log/Keyframes"

python3 "${PY}" \
  --g2o "${G2O}" \
  --keyframes_dir "${KEYFRAMES_DIR}" \
  --outdir "${OUTDIR}" \
  --symbols a,b,c \
  --on_missing_stamp skip  # skip / index / seq

  
