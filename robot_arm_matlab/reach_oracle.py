"""
reach_oracle.py — 在 CPython 里做机械臂"定姿态可达域"判定。

消费 MATLAB 侧 eMeetArm_workspace_aim.m 导出的 eMeetArm_reach_model.mat，
无需改动任何 MATLAB 代码、运行时也不需要 MATLAB。判定逻辑与 MATLAB 版
reachIsReachable.m / reachProject.m 完全一致（同一套 corner 原点 + floor 分桶约定）。

依赖:
    判定核心(供上层智能体 import): numpy, scipy   ->  pip install numpy scipy
    GUI(python reach_oracle.py):   额外需 matplotlib + tkinter(标准库)

运行:
    python reach_oracle.py              # 打开 GUI，手动输入 X/Y/Z 判定
    python reach_oracle.py --selftest   # 命令行自测(不弹窗)

用法(作为模块，仅需 numpy/scipy):
    from reach_oracle import ReachOracle
    ora = ReachOracle("eMeetArm_reach_model.mat")
    ora.is_reachable([0.18, -0.03, 0.43])      # -> True/False
    ora.is_reachable([[x1,y1,z1],[x2,y2,z2]])  # 批量 -> np.ndarray[bool]
    ora.project([0.43, 0.22, 0.58])            # 不可达则拉回最近可达点
    ora.in_ellipsoid([x,y,z])                  # 椭球护栏内(保证可达)

说明: 可达域对应"相机保持固定朝向(光轴朝前+画面水平)"，且已按 safetyMargin
      把外边界整体内收(默认 5cm)。栅格级判定，执行前建议仍用 IK 精确确认。
"""

import os
import numpy as np
from scipy.io import loadmat

# 模型默认与本脚本同目录，这样从任何工作目录运行都能找到
_DEFAULT_MAT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "eMeetArm_reach_model.mat")


class ReachOracle:
    def __init__(self, mat_path=_DEFAULT_MAT):
        m = loadmat(mat_path, squeeze_me=True, struct_as_record=False)["model"]
        self.vRes = float(m.vRes)
        self.origin = np.asarray(m.origin, dtype=float).reshape(3)      # corner 原点
        self.dims = np.asarray(m.dims, dtype=int).reshape(3)            # 栅格尺寸
        self.occ = np.asarray(m.occ).astype(bool)                      # 占据栅格 (nx,ny,nz)
        self.occCenters = np.asarray(m.occCenters, dtype=float).reshape(-1, 3)  # 可达体素中心
        self.c = np.asarray(m.c, dtype=float).reshape(3)               # 椭球中心
        self.A = np.asarray(m.A, dtype=float).reshape(3, 3)            # 椭球矩阵
        # 记录性字段（可选）
        self.aStar = np.asarray(getattr(m, "aStar", [1, 0, 0]), dtype=float).reshape(3)
        self.tolOri_deg = float(getattr(m, "tolOri_deg", np.nan))
        self.safetyMargin = float(getattr(m, "safetyMargin", 0.0))

    @staticmethod
    def _as2d(P):
        P = np.asarray(P, dtype=float)
        single = P.ndim == 1
        return (P.reshape(1, 3) if single else P), single

    def is_reachable(self, P):
        """P: (3,) 或 (N,3)。返回 bool 或 (N,) bool。栅格查表判定。"""
        P2, single = self._as2d(P)
        # 与建图同一套约定: idx0 = floor((P - origin)/vRes)   (0-based)
        idx = np.floor((P2 - self.origin) / self.vRes).astype(int)
        tf = np.zeros(P2.shape[0], dtype=bool)
        inb = np.all((idx >= 0) & (idx < self.dims), axis=1)
        if inb.any():
            ii = idx[inb]
            tf[inb] = self.occ[ii[:, 0], ii[:, 1], ii[:, 2]]
        return bool(tf[0]) if single else tf

    def project(self, P):
        """不可达的点拉回最近可达体素中心；已可达的原样返回。返回同形状 + 位移。"""
        P2, single = self._as2d(P)
        tf = np.atleast_1d(self.is_reachable(P2))
        out = P2.copy()
        moved = np.zeros(P2.shape[0])
        bad = ~tf
        if bad.any():
            # 到所有可达体素中心的距离，取最近
            d2 = ((P2[bad][:, None, :] - self.occCenters[None, :, :]) ** 2).sum(axis=2)
            k = d2.argmin(axis=1)
            out[bad] = self.occCenters[k]
            moved[bad] = np.sqrt(d2[np.arange(len(k)), k])
        if single:
            return out[0], float(moved[0])
        return out, moved

    def in_ellipsoid(self, P):
        """椭球护栏判据 (x-c)'A(x-c) <= 1；内部保证可达，可作可微约束。"""
        P2, single = self._as2d(P)
        d = P2 - self.c
        val = np.einsum("ni,ij,nj->n", d, self.A, d)  # (x-c)' A (x-c)
        tf = val <= 1.0
        return bool(tf[0]) if single else tf


# =====================================================================
# GUI：手动输入 X/Y/Z 判定可达（tkinter/matplotlib 的 import 放函数内，
# 这样把本文件当模块 import(供上层智能体调用) 时只依赖 numpy/scipy）。
# =====================================================================
def run_gui(mat_path=_DEFAULT_MAT):
    import tkinter as tk
    from tkinter import ttk, messagebox
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import (
        FigureCanvasTkAgg, NavigationToolbar2Tk)

    class _GUI:
        def __init__(self, root):
            self.root = root
            root.title("机械臂定姿态可达域 · 判定器")
            self.ora = ReachOracle(mat_path)

            C = self.ora.occCenters
            step = max(1, len(C) // 4000)
            self.cloud = C[::step]
            self.lo, self.hi = C.min(axis=0), C.max(axis=0)

            self._build_left()
            self._build_right()
            self._draw(query=None)

        def _build_left(self):
            f = ttk.Frame(self.root, padding=10)
            f.grid(row=0, column=0, sticky="ns")
            aim = self.ora.aStar
            txt = (f"光轴 = [{aim[0]:.2f} {aim[1]:.2f} {aim[2]:.2f}]，画面水平\n"
                   f"姿态容差 ±{self.ora.tolOri_deg:.0f}°   安全收边 {self.ora.safetyMargin*100:.0f} cm\n"
                   f"可达范围 (m):\n"
                   f"  X [{self.lo[0]:.2f}, {self.hi[0]:.2f}]\n"
                   f"  Y [{self.lo[1]:.2f}, {self.hi[1]:.2f}]\n"
                   f"  Z [{self.lo[2]:.2f}, {self.hi[2]:.2f}]")
            ttk.Label(f, text="模型信息", font=("", 11, "bold")).grid(
                row=0, column=0, columnspan=2, sticky="w")
            ttk.Label(f, text=txt, justify="left", foreground="#444").grid(
                row=1, column=0, columnspan=2, sticky="w", pady=(2, 12))
            ttk.Label(f, text="输入目标位置 (世界系, m)", font=("", 11, "bold")).grid(
                row=2, column=0, columnspan=2, sticky="w")

            self.ent = {}
            for i, (name, dflt) in enumerate((("X", "0.30"), ("Y", "0.00"), ("Z", "0.45"))):
                ttk.Label(f, text=name).grid(row=3 + i, column=0, sticky="e", padx=(0, 6), pady=2)
                e = ttk.Entry(f, width=12)
                e.insert(0, dflt)
                e.grid(row=3 + i, column=1, sticky="w", pady=2)
                e.bind("<Return>", lambda ev: self.judge())
                self.ent[name] = e

            btns = ttk.Frame(f)
            btns.grid(row=7, column=0, columnspan=2, pady=10, sticky="w")
            ttk.Button(btns, text="判定", command=self.judge).pack(side="left")
            ttk.Button(btns, text="不可达则拉回",
                       command=lambda: self.judge(do_project=True)).pack(side="left", padx=6)

            self.result = tk.Label(f, text="等待输入…", justify="left", anchor="w",
                                   font=("", 11), width=34, height=6,
                                   relief="groove", bg="#f5f5f5")
            self.result.grid(row=8, column=0, columnspan=2, sticky="we", pady=(4, 0))

        def _build_right(self):
            f = ttk.Frame(self.root)
            f.grid(row=0, column=1, sticky="nsew")
            self.root.columnconfigure(1, weight=1)
            self.root.rowconfigure(0, weight=1)
            self.fig = Figure(figsize=(6.5, 6), dpi=100)
            self.ax = self.fig.add_subplot(111, projection="3d")
            self.canvas = FigureCanvasTkAgg(self.fig, master=f)
            self.canvas.get_tk_widget().pack(fill="both", expand=True)
            NavigationToolbar2Tk(self.canvas, f).update()

        def _draw(self, query=None, proj=None, reach=None):
            ax = self.ax
            ax.clear()
            ax.scatter(self.cloud[:, 0], self.cloud[:, 1], self.cloud[:, 2],
                       s=2, c="0.8", label="可达域(收边后)")
            ax.scatter([0], [0], [0], c="r", marker="*", s=140, label="基座")
            if query is not None:
                col = "#1a9e1a" if reach else "#d62728"
                ax.scatter([query[0]], [query[1]], [query[2]], c=col, marker="o",
                           s=90, edgecolor="k",
                           label="查询点(可达)" if reach else "查询点(不可达)")
                if proj is not None:
                    ax.scatter([proj[0]], [proj[1]], [proj[2]], c="#1f77b4", marker="^",
                               s=90, edgecolor="k", label="拉回点")
                    ax.plot([query[0], proj[0]], [query[1], proj[1]],
                            [query[2], proj[2]], "b--", lw=1)
            ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
            rng = self.hi - self.lo
            ax.set_box_aspect(rng if np.all(rng > 0) else (1, 1, 1))
            ax.legend(loc="upper left", fontsize=8)
            self.canvas.draw()

        def judge(self, do_project=False):
            try:
                p = np.array([float(self.ent[k].get()) for k in ("X", "Y", "Z")])
            except ValueError:
                messagebox.showwarning("输入错误", "X/Y/Z 必须是数字（单位 m）。")
                return
            reach = self.ora.is_reachable(p)
            in_ell = self.ora.in_ellipsoid(p)
            lines = [f"查询点: [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}] m",
                     f"可达: {'是 ✓' if reach else '否 ✗'}",
                     f"椭球护栏内(必可达): {'是' if in_ell else '否'}"]
            proj = None
            if not reach and do_project:
                proj, moved = self.ora.project(p)
                lines.append(f"拉回点: [{proj[0]:.3f}, {proj[1]:.3f}, {proj[2]:.3f}]")
                lines.append(f"移动: {moved*100:.1f} cm")
            self.result.config(text="\n".join(lines),
                               bg="#e6f5e6" if reach else "#fdeaea")
            self._draw(query=p, proj=proj, reach=reach)

    root = tk.Tk()
    _GUI(root)
    root.mainloop()


def _selftest():
    ora = ReachOracle()
    print(f"vRes={ora.vRes}  dims={ora.dims.tolist()}  "
          f"tolOri=±{ora.tolOri_deg}°  收边={ora.safetyMargin*100:.0f}cm")
    tests = np.array([[0.18, -0.03, 0.43], [0.43, 0.22, 0.58], [0.08, 0.02, 0.63]])
    tf = ora.is_reachable(tests)
    proj, moved = ora.project(tests)
    for p, t, q, mv in zip(tests, tf, proj, moved):
        print(f"  {p}  可达={int(t)}  ->  拉回{np.round(q, 2)} (移动{mv*100:.1f}cm)")


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        _selftest()
    else:
        run_gui()
