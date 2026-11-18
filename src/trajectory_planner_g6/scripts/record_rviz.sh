#!/bin/bash
# =============================================================================
#  record_rviz.sh — Enregistre la simulation RViz2 en video MP4 + captures PNG
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
RESULTS_DIR="${WORKSPACE_DIR}/kinova_trajectory_matlab/results"
mkdir -p "$RESULTS_DIR"

VIDEO_OUT="${RESULTS_DIR}/rviz2_simulation.mp4"
DURATION=22 # Duree d'enregistrement en secondes

echo "=========================================================="
echo "  Enregistrement automatique de RViz2 (MP4 + Snapshots)"
echo "=========================================================="

if ! command -v ffmpeg &> /dev/null; then
    echo "Installation de ffmpeg requise..."
    sudo apt update && sudo apt install -y ffmpeg
fi

echo "[1/3] Enregistrement video de l'ecran RViz2 pendant ${DURATION}s..."

ffmpeg -y -f x11grab -video_size 1920x1080 -i :0.0 \
       -t "$DURATION" -r 30 -c:v libx264 -pix_fmt yuv420p "$VIDEO_OUT" 2>/dev/null || \
ffmpeg -y -f x11grab -video_size 1280x720 -i :0.0 \
       -t "$DURATION" -r 30 -c:v libx264 -pix_fmt yuv420p "$VIDEO_OUT"

echo "   ✔ Video RViz2 enregistree : $VIDEO_OUT"

echo "[2/3] Extraction de captures PNG haute resolution..."
ffmpeg -y -i "$VIDEO_OUT" -ss 00:00:05 -vframes 1 "${RESULTS_DIR}/rviz2_debut.png" 2>/dev/null || true
ffmpeg -y -i "$VIDEO_OUT" -ss 00:00:12 -vframes 1 "${RESULTS_DIR}/rviz2_milieu.png" 2>/dev/null || true
ffmpeg -y -i "$VIDEO_OUT" -ss 00:00:19 -vframes 1 "${RESULTS_DIR}/rviz2_fin_rose.png" 2>/dev/null || true

echo "   ✔ Captures PNG extraites dans : $RESULTS_DIR"
echo "=========================================================="
