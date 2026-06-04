#!/usr/bin/env python3
"""
批量简化 eMeetArm STL 网格，降低 Gazebo 渲染和碰撞检测的计算量。
原始文件自动备份为 *.STL.bak，可随时还原。

用法：
    python3 simplify_meshes.py              # 简化到默认目标面数
    python3 simplify_meshes.py --restore    # 从备份还原原始文件
    python3 simplify_meshes.py --faces 2000 # 自定义目标面数
"""

import argparse
import shutil
from pathlib import Path

MESH_DIR = Path(__file__).parent.parent / 'eMeetArm_models' / 'meshes'

# 各 Link 的目标面数（原始约 10~13 万面，降低约 97%）
TARGET_FACES = {
    'base_link.STL': 3000,
    'Link1.STL':     3000,
    'Link2.STL':     2000,
    'Link3.STL':     2000,
    'Link4.STL':     1000,
    'Link5.STL':     800,
    'Link6.STL':     800,
}


def simplify(target_faces_override: int | None = None):
    try:
        import pymeshlab
    except ImportError:
        print('请先安装：pip install pymeshlab')
        return

    for filename, default_faces in TARGET_FACES.items():
        src = MESH_DIR / filename
        bak = MESH_DIR / (filename + '.bak')

        if not src.exists():
            print(f'[跳过] {filename} 不存在')
            continue

        if not bak.exists():
            shutil.copy2(src, bak)
            print(f'[备份] {filename} → {filename}.bak')

        target = target_faces_override or default_faces
        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(str(src))

        original = ms.current_mesh().face_number()
        ms.meshing_decimation_quadric_edge_collapse(
            targetfacenum=target,
            qualitythr=0.3,
            preserveboundary=True,
            preservenormal=True,
        )
        result = ms.current_mesh().face_number()
        ms.save_current_mesh(str(src))

        print(f'[简化] {filename}: {original:,} → {result:,} 面 ({result/original*100:.1f}%)')

    print('\n完成。重新编译：colcon build --packages-select arm')


def restore():
    for filename in TARGET_FACES:
        bak = MESH_DIR / (filename + '.bak')
        dst = MESH_DIR / filename
        if bak.exists():
            shutil.copy2(bak, dst)
            print(f'[还原] {filename}')
        else:
            print(f'[跳过] {filename}.bak 不存在')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--restore', action='store_true', help='从备份还原原始网格')
    parser.add_argument('--faces', type=int, default=None, help='统一指定目标面数')
    args = parser.parse_args()

    if args.restore:
        restore()
    else:
        simplify(args.faces)
