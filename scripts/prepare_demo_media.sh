#!/usr/bin/env bash
# Offline documentation media only. Does not use ROS or contact the robot.
set -euo pipefail
input_dir="${1:?Usage: prepare_demo_media.sh VIDEO_DIR OUTPUT_DIR}"
output_dir="${2:?Usage: prepare_demo_media.sh VIDEO_DIR OUTPUT_DIR}"
mkdir -p "$output_dir"
encode() {
  local input="$1" output="$2" speed="$3"
  ffmpeg -hide_banner -loglevel error -n -threads 2 -i "$input_dir/$input" \
    -an -vf "setpts=(PTS-STARTPTS)/${speed},fps=30,scale=1440:-2,drawtext=text='${speed}x playback':x=20:y=20:fontsize=28:fontcolor=white:box=1:boxcolor=black@0.65:boxborderw=10" \
    -c:v libx264 -threads 2 -preset medium -crf 26 -pix_fmt yuv420p \
    -movflags +faststart -metadata comment="Playback speed: ${speed}x; source preserved" \
    "$output_dir/$output"
}
encode '狗子建图1.mp4' mapping-10x.mp4 10
encode '狗子定位+快速打点.mp4' localization-3x.mp4 3
encode '狗子打点巡航.mp4' patrol-10x.mp4 10
# A short visual preview; full video remains a separate release asset.
ffmpeg -hide_banner -loglevel error -n -threads 2 -ss 40 \
  -i "$input_dir/狗子打点巡航.mp4" -t 6 \
  -filter_complex "[0:v]setpts=(PTS-STARTPTS)/10,fps=8,scale=800:-2,drawtext=text='10x playback':x=12:y=12:fontsize=18:fontcolor=white:box=1:boxcolor=black@0.65:boxborderw=6,split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer" \
  -loop 0 "$output_dir/patrol-preview-10x.gif"
for item in '狗子建图1.mp4:mapping' '狗子定位+快速打点.mp4:localization' '狗子打点巡航.mp4:patrol'; do
  ffmpeg -hide_banner -loglevel error -n -threads 2 -ss 60 \
    -i "$input_dir/${item%:*}" -frames:v 1 -vf 'scale=1440:-2' \
    -q:v 3 "$output_dir/${item##*:}.jpg"
done
