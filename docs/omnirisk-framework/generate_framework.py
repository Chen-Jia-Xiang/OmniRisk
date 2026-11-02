#!/usr/bin/env python3
"""Render source-checked OmniRisk diagrams. Run from any working directory."""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


OUT = Path(__file__).resolve().parent / "assets"
FONT = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
matplotlib.rcParams.update({"svg.fonttype": "none", "pdf.fonttype": 42})
C = {"ink": "#172B43", "muted": "#5E7085", "blue": "#2876B8",
     "teal": "#087F83", "orange": "#C46B28", "purple": "#7655B5",
     "line": "#A7B7C8", "paper": "#F6F8FC"}


class Canvas:
    def __init__(self, width, height, title, subtitle):
        self.width, self.height = width, height
        self.fig, self.ax = plt.subplots(figsize=(width / 100, height / 100))
        self.fig.subplots_adjust(0, 0, 1, 1)
        self.ax.set(xlim=(0, width), ylim=(height, 0))
        self.ax.axis("off")
        self.fig.patch.set_facecolor("white")
        self.text(64, 60, title, 27, weight="bold")
        self.text(64, 104, subtitle, 13, color=C["muted"])
        self.texts = []

    def text(self, x, y, text, size=14, color=None, weight="normal", ha="left"):
        artist = self.ax.text(x, y, text, fontsize=size, fontproperties=FONT,
                              color=color or C["ink"], weight=weight,
                              ha=ha, va="center", linespacing=1.6, zorder=5)
        return artist

    def panel(self, x, y, w, h, title, color, fill):
        self.ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=18",
                                        facecolor=fill, edgecolor=color, linewidth=1.1, zorder=0))
        self.text(x + 24, y + 35, title, 17, color=color, weight="bold")

    def box(self, x, y, w, h, title, lines, color="blue", foot=None, size=14):
        color = C.get(color, color)
        self.ax.add_patch(FancyBboxPatch((x + 3, y + 5), w, h,
                                        boxstyle="round,pad=0,rounding_size=12",
                                        facecolor="#E5EBF3", edgecolor="none", alpha=.5, zorder=1))
        self.ax.add_patch(FancyBboxPatch((x, y), w, h,
                                        boxstyle="round,pad=0,rounding_size=12",
                                        facecolor="white", edgecolor=color, linewidth=1.5, zorder=2))
        self.ax.plot([x + 18, x + 18], [y + 20, y + 48], color=color, lw=3.5, zorder=3)
        artists = [self.text(x + 32, y + 34, title, 17, color=color, weight="bold")]
        for i, line in enumerate(lines):
            artists.append(self.text(x + 25, y + 78 + i * 31, line, size))
        if foot:
            artists.append(self.text(x + 25, y + h - 24, foot, 11, color=C["muted"]))
        self.texts.extend((a, (x, y, w, h)) for a in artists)

    def arrow(self, points, color="line", dashed=False, label=None, label_pos=None):
        color = C.get(color, color)
        for a, b in zip(points[:-2], points[1:-1]):
            self.ax.plot([a[0], b[0]], [a[1], b[1]], color=color, lw=1.6,
                         linestyle="--" if dashed else "-", zorder=3)
        self.ax.add_patch(FancyArrowPatch(points[-2], points[-1], arrowstyle="-|>",
                                         mutation_scale=15, linewidth=1.6, color=color,
                                         linestyle="--" if dashed else "-", zorder=3))
        if label:
            self.text(*label_pos, label, 12, color=color, ha="center")

    def save(self, name):
        # Check text against its owning card; catches font/size changes that clip labels.
        self.fig.canvas.draw()
        renderer = self.fig.canvas.get_renderer()
        for artist, (x, y, w, h) in self.texts:
            bbox = artist.get_window_extent(renderer).transformed(self.ax.transData.inverted())
            if bbox.x0 < x or bbox.x1 > x + w or bbox.y0 > y + h or bbox.y1 < y:
                raise ValueError(f"Text outside card: {artist.get_text()}")
        OUT.mkdir(parents=True, exist_ok=True)
        for suffix in ["svg", "png", "pdf"]:
            self.fig.savefig(OUT / f"{name}.{suffix}", dpi=140, facecolor="white")
        plt.close(self.fig)


def overview():
    c = Canvas(2400, 1580, "OmniRisk 项目框架｜离线风险学习 → 在线全向避障",
               "依据：2609.18191v2，§III–V / Fig. 2；结合当前仓库源码核对。在线主图采用默认 LiDAR + pointcloud 输入路径。")
    xs, w = [80, 650, 1220, 1790], 500
    c.panel(48, 150, 2304, 355, "01   离线训练  /  仿真真值仅用于构造训练信号", C["orange"], "#FFF8F0")
    c.box(xs[0], 225, w, 225, "仿真场景与危险相遇采样",
          ["静态地图 + 动态球体 / 圆柱", "逆 CPA：由最近接近距离与时刻反推场景", "保存距离、物体 ID、位姿和速度真值"], "orange",
          "Simulator/src/src/dataset_generator.cpp")
    c.box(xs[1], 225, w, 225, "数据对齐与当前候选轨迹",
          ["Dataset 构造 D、M、V 与状态 / 目标", "网络预测 36 组终端状态", "由起终状态重建轨迹，随训练持续更新"], "orange",
          "policy/dataset.py · policy/trainer.py")
    c.box(xs[2], 225, w, 225, "解析代价与三项训练损失",
          ["ESDF 静态净空 + jerk / 加速度 / 引导", "非对称动态场：沿轨迹采样、跨障碍累加", "终端状态优化 + 静态评分回归 + 风险回归"], "orange",
          "loss/ · trainer.forward_and_compute_loss()")
    c.box(xs[3], 225, w, 225, "联合训练与模型部署",
          ["评分标签 detach；动态标签取 log1p", "AdamW 同时更新两个编码器与预测头", "保存 .pth；可选 TensorRT 导出"], "orange",
          "train.py · policy/trainer.py · trt_transfer.py")
    for a, b in zip(xs, xs[1:]):
        c.arrow([(a + w, 337), (b, 337)], "orange")
    c.arrow([(2040, 450), (2040, 540), (1470, 540), (1470, 650)], "orange", True,
            "部署同一双分支网络的训练权重", (1570, 522))

    c.panel(48, 570, 2304, 710, "02   在线闭环  /  感知 → 输入构建 → 一次前向 → 选轨 → 控制", C["teal"], "#F2FAFA")
    y, h = 650, 235
    c.box(xs[0], y, w, h, "A  传感与动静态感知",
          ["MID-360 + IMU → FAST-LIO", "M-detector 分割、聚类、确认与跟踪", "输出当前云、静态背景云、动态 tracks"], "teal",
          "ros_ws/src/FAST_LIO · M-detector")
    c.box(xs[1], y, w, h, "B  同步、坐标与全景输入",
          ["最新完整帧；里程计插值与目标时刻外推", "球面投影 + 静态背景融合 + 动态栅格化", "仅 D 补洞；96×360 → 96×384"], "teal",
          "perception_sync.py · dynamic_inputs.py · node")
    c.box(xs[2], y, w, h, "C  双分支网络  ·  一次前向",
          ["D → 共享距离编码；[M,V] → 动态编码", "静态头：36 组终端状态 + 36 静态评分", "动态头：36 动态风险评分"], "blue",
          "policy/network.py · models/backbone.py · head.py")
    c.box(xs[3], y, w, h, "D  模式与候选选择",
          ["HOVER / CRUISE / DODGE", "S = S_static + λ · S_dynamic", "端点连续性代价 + 切换滞回"], "purple",
          "runtime_modes.py · trajectory_selection.py")
    for a, b in zip(xs, xs[1:]):
        c.arrow([(a + w, y + 118), (b, y + 118)], "teal")

    # Two secondary inputs occupy the gutter between the two online rows.
    c.text(100, 944, "状态：FAST-LIO + MAVROS IMU → EKF → p / q / v；a 由节点估计或控制状态提供", 13, color=C["muted"])
    c.text(100, 977, "意图：RViz / RC → goal；状态与目标归一化后投影到 3×12 个基元坐标系，供两个预测头使用", 13, color=C["muted"])
    c.arrow([(900, 927), (900, 885)], "muted")
    c.arrow([(1470, 927), (1470, 885)], "muted")

    y2, h2 = 1030, 205
    c.box(xs[3], y2, w, h2, "E  闭式重建所选轨迹",
          ["终端状态解码并转换到世界系", "以提交时最新状态为轨迹起点", "x / y / z 三轴五次多项式"], "blue",
          "policy/state_transform.py · poly_solver.py")
    c.box(xs[2], y2, w, h2, "F  50 Hz 参考指令发布",
          ["轨迹采样 → p / v / a / jerk / yaw", "PositionCommand → /so3_control/pos_cmd", "轨迹超时：锁定当前位置并请求悬停"], "blue",
          "omnirisk_node.control_pub() · control_state.py")
    c.box(xs[1], y2, w, h2, "G  轨迹跟踪与执行",
          ["实机：px4ctrl → MAVROS → PX4", "仿真：NetworkControl → 刚体动力学", "控制器利用实时 odom 计算姿态 / 推力"], "teal",
          "ros_ws/src/px4ctrl · Controller/src/")
    c.box(xs[0], y2, w, h2, "H  飞行平台与环境反馈",
          ["执行机体运动，改变相对位置与速度", "实机传感器 / 仿真传感器生成新观测", "下一帧再次完成感知、评分与重规划"], "teal",
          "Simulator/src/ · 实机四旋翼")
    c.arrow([(2040, 885), (2040, 1030)], "purple")
    for a, b in zip(reversed(xs[1:]), reversed(xs[:-1])):
        c.arrow([(a, y2 + 103), (b + w, y2 + 103)], "teal")
    c.arrow([(80, 1133), (64, 1133), (64, 768), (80, 768)], "teal")

    c.panel(48, 1320, 2304, 172, "03   读图边界", C["line"], C["paper"])
    c.text(80, 1392, "橙色虚线 = 离线权重迁移；实线 = 在线 / 离线数据流。动态风险积分、ESDF 查询均在训练阶段完成。", 14)
    c.text(80, 1432, "论文规划目标 20 Hz；代码为异步工作线程、吞吐依赖输入与计算时间。50 Hz 是本节点参考指令定时器频率。", 14)
    c.text(64, 1535, "来源：论文 PDF p.3–6；当前仓库 a0a89a4。完整源码索引、公式与实现差异见同目录 report.md。", 12, color=C["muted"])
    c.save("omnirisk-framework")


def network():
    c = Canvas(2400, 1460, "OmniRisk 网络结构｜共享距离特征，隔离动态信息对候选几何的直接输入",
               "张量采用 PyTorch 的 B×C×H×W 顺序；B 为批量。五通道是概念表示，forward 实际接收 D、M、V、obs 四个参数。")
    c.panel(48, 150, 2304, 1090, "一次前向输出 36 组候选：12 个方位 × 3 个俯仰方向", C["blue"], "#F4F8FD")
    c.box(80, 250, 410, 200, "全景距离 D",
          ["B×1×96×384", "距离归一化；仅此通道补洞"], "blue", "输入几何与净空信息")
    c.box(80, 560, 410, 200, "动态掩码 M + 速度图 V",
          ["M: B×1×96×384", "V: B×3×96×384", "拼接 → B×4×96×384"], "orange")
    c.box(80, 900, 410, 235, "当前状态与目标 obs",
          ["[v_x,v_y,v_z, a_x,a_y,a_z, g_x,g_y,g_z]", "速度 / 加速度 / 目标归一化", "转入各基元系 → B×9×3×12", "state_backbone 为恒等映射"], "purple", size=12)

    c.box(590, 250, 450, 200, "共享距离编码器",
          ["ResNet-18；1 通道输入", "32 倍下采样 → B×64×3×12", "距离特征供静态头与动态头共用"], "blue",
          "image_backbone = StaticBackbone(64)")
    c.box(590, 560, 450, 200, "动态信息编码器",
          ["ResNet-18；4 通道输入", "32 倍下采样 → B×64×3×12", "该特征只送入动态评分头"], "orange",
          "dynamic_backbone = DynamicBackbone(64)")
    c.arrow([(490, 350), (590, 350)], "blue")
    c.arrow([(490, 660), (590, 660)], "orange")

    c.box(1240, 250, 440, 235, "静态头  Static Head",
          ["拼接：64 距离 + 9 状态 = 73 通道", "1×1 Conv: 73 → 256 → 256 → 10", "前 9 通道 tanh → 终端状态参数", "第 10 通道 softplus → 静态评分"], "blue", size=13)
    c.box(1240, 610, 440, 235, "动态头  Dynamic Head",
          ["拼接：64 距离 + 64 动态 + 9 状态", "1×1 Conv: 137 → 256 → 256 → 1", "softplus → 每候选一个非负风险值", "训练拟合 log1p 压缩后的动态代价"], "orange", size=13)

    c.arrow([(1040, 350), (1240, 350)], "blue")
    c.arrow([(1100, 350), (1100, 690), (1240, 690)], "blue")
    c.text(1093, 525, "共享", 12, color=C["blue"], ha="center")
    c.arrow([(1040, 660), (1140, 660), (1140, 755), (1240, 755)], "orange")
    c.arrow([(490, 1015), (1185, 1015), (1185, 420), (1240, 420)], "purple")
    c.arrow([(1185, 1015), (1185, 810), (1240, 810)], "purple")
    c.text(675, 986, "同一状态 / 目标特征送入两个头", 13, color=C["purple"])

    c.box(1790, 240, 500, 265, "36 组终端状态与静态评分",
          ["endstate: B×9×3×12 → 36×(p_f,v_f,a_f)", "前 3 个原始输出解码为角偏移与半径", "经 StateTransform 得到实际 p / v / a", "S_static: B×3×12", "静态评分已包含目标进展和平滑代价"], "blue", size=13)
    c.box(1790, 625, 500, 200, "36 个动态风险评分",
          ["S_dynamic: B×3×12", "动态头改变候选排名", "给定相同 D / obs，M / V 不改变终端状态"], "orange", size=13)
    c.arrow([(1680, 367), (1790, 367)], "blue")
    c.arrow([(1680, 727), (1790, 727)], "orange")
    c.box(1790, 975, 500, 210, "融合与重建",
          ["S_static + λ · S_dynamic + 连续性代价", "加切换滞回选出一个候选", "解码终端状态 → 闭式五次轨迹", "评分是代价代理，不能解释为碰撞概率"], "teal", size=13)
    c.arrow([(2040, 505), (2330, 505), (2330, 1080), (2290, 1080)], "blue")
    c.arrow([(2040, 825), (2040, 975)], "orange")

    c.text(80, 1290, "训练关系：终端几何损失不含动态风险；静态 / 动态评分目标均 detach。动态评分损失仍更新共享距离编码器。", 14)
    c.text(80, 1335, "代码细节：有 padding 的卷积在宽、高两轴都做 circular padding；论文只以方位角环绕解释该设计。", 14)
    c.text(64, 1415, "来源：policy/network.py:47 · models/backbone.py:7 · models/head.py:5 · state_transform.py:30 · trainer.py:169", 12, color=C["muted"])
    c.save("omnirisk-network")


if __name__ == "__main__":
    overview()
    network()
    print(f"Rendered PNG / SVG / PDF diagrams to {OUT}")
