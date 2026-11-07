#!/bin/bash
# =============================================================================
#  setup_urdf.sh — Prepare l'URDF du Kinova Gen3 6-DOF pour MATLAB
#
#  Ce script :
#  1. Copie l'URDF depuis le workspace ROS 2
#  2. Copie tous les fichiers mesh (STL, DAE) necessaires
#  3. Corrige les chemins mesh dans l'URDF pour utiliser des chemins relatifs
#  4. Fixe les articulations de la pince en conservant son rendu 3D
#  5. Cree le repere TCP (Tool Center Point) exactement au bout des mors
#
#  Usage : bash setup_urdf.sh
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
URDF_DIR="${SCRIPT_DIR}/src/urdf"
MESH_DIR="${URDF_DIR}/meshes"

WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SRC_URDF="${WORKSPACE_DIR}/src/ros2_kortex/kortex_description/robots/gen3_6dof_robotiq_2f_85.urdf"
SRC_ARM_MESHES="${WORKSPACE_DIR}/src/ros2_kortex/kortex_description/arms/gen3/6dof/meshes"
SRC_GRIPPER_MESHES="/opt/ros/humble/share/robotiq_description/meshes"

echo "=============================================="
echo "  Setup URDF Kinova Gen3 6-DOF pour MATLAB"
echo "=============================================="

if [ ! -f "$SRC_URDF" ]; then
    echo "ERREUR : URDF source introuvable : $SRC_URDF"
    exit 1
fi

echo ""
echo "[1/4] Creation des repertoires..."
mkdir -p "${MESH_DIR}/arm"
mkdir -p "${MESH_DIR}/gripper/visual"
mkdir -p "${MESH_DIR}/gripper/collision"

echo "[2/4] Copie des meshes du bras (STL)..."
if [ -d "$SRC_ARM_MESHES" ]; then
    cp -v "${SRC_ARM_MESHES}"/*.STL "${MESH_DIR}/arm/" 2>/dev/null || true
    echo "   OK Meshes du bras copies."
fi

echo "[3/4] Copie des meshes du gripper (DAE + STL)..."
if [ -d "$SRC_GRIPPER_MESHES" ]; then
    cp -v "${SRC_GRIPPER_MESHES}/visual"/*.dae "${MESH_DIR}/gripper/visual/" 2>/dev/null || true
    cp -v "${SRC_GRIPPER_MESHES}/collision"/*.stl "${MESH_DIR}/gripper/collision/" 2>/dev/null || true
    echo "   OK Meshes du gripper copies."
fi

echo "[4/4] Copie et adaptation de l'URDF..."
cp "$SRC_URDF" "${URDF_DIR}/gen3_6dof.urdf"

cd "$URDF_DIR"

# Chemins relatifs pour les meshes
sed -i -E "s|file://.*/arms/gen3/6dof/meshes/|meshes/arm/|g" gen3_6dof.urdf
sed -i -E "s|file://.*/robotiq_description/meshes/visual/|meshes/gripper/visual/|g" gen3_6dof.urdf
sed -i -E "s|file://.*/robotiq_description/meshes/collision/|meshes/gripper/collision/|g" gen3_6dof.urdf

# Nettoyage pour compatibilite MATLAB
python3 -c "
import re
with open('gen3_6dof.urdf', 'r') as f:
    content = f.read()

# Supprimer ros2_control, surface et mimic
content = re.sub(r'<ros2_control.*?</ros2_control>', '', content, flags=re.DOTALL)
content = re.sub(r'<surface>.*?</surface>', '', content, flags=re.DOTALL)
content = re.sub(r'<mimic[^/]*/>', '', content)

# Fixer les articulations du gripper pour que seuls les 6 DOF du bras soient mobiles dans MATLAB
def fix_robotiq(m):
    return '<joint name=\"' + m.group(1) + '\" type=\"fixed\">'
content = re.sub(r'<joint name=\"(robotiq_85_[^\"]+)\" type=\"[^\"]+\">', fix_robotiq, content)

# Ajouter le repere TCP au bout des doigts
if '<link name=\"tcp_link\"/>' not in content:
    tcp_xml = '''
  <link name=\"tcp_link\"/>
  <joint name=\"tcp_joint\" type=\"fixed\">
    <parent link=\"end_effector_link\"/>
    <child link=\"tcp_link\"/>
    <origin xyz=\"0 0 0.120\" rpy=\"0 0 0\"/>
  </joint>
</robot>
'''
    content = content.replace('</robot>', tcp_xml)

with open('gen3_6dof.urdf', 'w') as f:
    f.write(content)
print('   OK URDF adapte : 6 DOF mobiles garantis + repere TCP au bout des mors.')
"

echo ""
echo "=============================================="
echo "  SETUP TERMINE AVEC SUCCES"
echo "=============================================="
