# -*- coding: utf-8 -*-
"""Pure Pursuit 路径跟踪仿真 v2（纯标准库）
修正：终止条件改为"沿路径推进到终点"，避免绕圈
"""
import math

# ---------- 1. 参考路径 ----------
def ref_path(n=600):
    pts = []
    for i in range(n):
        x = 40.0 * i / (n - 1)
        pts.append((x, 3.0 * math.sin(x / 6.0)))
    return pts

REF = ref_path()

# ---------- 2. 车辆参数 ----------
L  = 2.6      # 轴距 [m]
V  = 5.0      # 车速 [m/s]
DT = 0.02     # 步长 [s]
LD = 6.0      # 前视距离 [m]（比 4.0 更稳，抑制蛇形）

def nearest_index(x, y):
    best, idx = 1e18, 0
    for i, (px, py) in enumerate(REF):
        d = (px - x) ** 2 + (py - y) ** 2
        if d < best:
            best, idx = d, i
    return idx

# 初始航向对齐路径切线，避免起步猛打方向
x, y = REF[0]
yaw = math.atan2(REF[1][1] - REF[0][1], REF[1][0] - REF[0][0])
traj = [(x, y)]

for step in range(4000):
    i = nearest_index(x, y)
    if i >= len(REF) - 2:          # 已推进到终点，收工
        break

    target, found = REF[-1], False
    for j in range(i, len(REF)):
        if math.hypot(REF[j][0] - x, REF[j][1] - y) >= LD:
            target, found = REF[j], True
            break
    if not found:
        target = REF[-1]

    alpha = math.atan2(target[1] - y, target[0] - x) - yaw
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))   # 归一化到 [-pi, pi]
    delta = math.atan2(2.0 * L * math.sin(alpha), LD)      # 纯跟踪转向角
    delta = max(-0.6, min(0.6, delta))                     # 前轮限幅 ±0.6 rad

    x += V * math.cos(yaw) * DT
    y += V * math.sin(yaw) * DT
    yaw += V / L * math.tan(delta) * DT
    traj.append((x, y))

# ---------- 3. 误差统计 ----------
errs = [min(math.hypot(rx - tx, ry - ty) for rx, ry in REF) for tx, ty in traj]
print("轨迹点数   :", len(traj))
print("行驶距离   : %.1f m" % (V * DT * (len(traj) - 1)))
print("终点残差   : %.3f m" % math.hypot(traj[-1][0] - REF[-1][0], traj[-1][1] - REF[-1][1]))
print("最大横向误差: %.3f m" % max(errs))
print("平均横向误差: %.3f m" % (sum(errs) / len(errs)))

# ---------- 4. 手写 SVG ----------
W, H = 1000, 520
ML, MR, MT, MB = 80, 40, 60, 70

xs = [p[0] for p in REF] + [p[0] for p in traj]
ys = [p[1] for p in REF] + [p[1] for p in traj]
xmin, xmax = min(xs), max(xs)
ymin, ymax = min(ys), max(ys)
padx = (xmax - xmin) * 0.04
pady = (ymax - ymin) * 0.12 + 0.5
xmin, xmax = xmin - padx, xmax + padx
ymin, ymax = ymin - pady, ymax + pady

s = min((W - ML - MR) / (xmax - xmin), (H - MT - MB) / (ymax - ymin))
ox = ML + ((W - ML - MR) - (xmax - xmin) * s) / 2
oy = MT + ((H - MT - MB) - (ymax - ymin) * s) / 2

def px(v): return ox + (v - xmin) * s
def py(v): return H - (oy + (v - ymin) * s)

def polyline(pts, color, width, dash=None):
    d = " ".join("%.2f,%.2f" % (px(a), py(b)) for a, b in pts)
    da = ' stroke-dasharray="%s"' % dash if dash else ""
    return ('<polyline points="%s" fill="none" stroke="%s" stroke-width="%s" '
            'stroke-linejoin="round" stroke-linecap="round"%s/>' % (d, color, width, da))

p = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" '
     'font-family="Microsoft YaHei, sans-serif">' % (W, H, W, H)]
p.append('<rect width="%d" height="%d" fill="#ffffff"/>' % (W, H))

for k in range(6):
    gx = xmin + (xmax - xmin) * k / 5
    p.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#e6eaf0"/>'
             % (px(gx), py(ymin), px(gx), py(ymax)))
    p.append('<text x="%.1f" y="%.1f" font-size="12" fill="#8a929c" text-anchor="middle">%.0f</text>'
             % (px(gx), H - MB + 24, gx))
for k in range(5):
    gy = ymin + (ymax - ymin) * k / 4
    p.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#e6eaf0"/>'
             % (px(xmin), py(gy), px(xmax), py(gy)))
    p.append('<text x="%.1f" y="%.1f" font-size="12" fill="#8a929c" text-anchor="end">%.1f</text>'
             % (px(xmin) - 12, py(gy) + 4, gy))

p.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#c3cad3" stroke-width="1.5"/>'
         % (px(xmin), py(ymin), px(xmax), py(ymin)))
p.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#c3cad3" stroke-width="1.5"/>'
         % (px(xmin), py(ymin), px(xmin), py(ymax)))

p.append(polyline(REF, "#2b2f36", 2.4, "10 7"))
p.append(polyline(traj, "#d95b1e", 2.6))

p.append('<text x="%.1f" y="34" font-size="22" font-weight="bold" fill="#0f2a4a" '
         'text-anchor="middle">Pure Pursuit 路径跟踪仿真</text>' % (W / 2))
p.append('<text x="%.1f" y="%.1f" font-size="13" fill="#555c66" text-anchor="middle">x [m]</text>'
         % (W / 2, H - 14))

lx, ly = W - MR - 270, MT - 26
p.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#2b2f36" stroke-width="2.4" '
         'stroke-dasharray="10 7"/>' % (lx, ly, lx + 34, ly))
p.append('<text x="%.1f" y="%.1f" font-size="13" fill="#2b2f36">目标轨迹</text>' % (lx + 42, ly + 5))
p.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#d95b1e" stroke-width="2.6"/>'
         % (lx + 132, ly, lx + 166, ly))
p.append('<text x="%.1f" y="%.1f" font-size="13" fill="#d95b1e">仿真轨迹</text>' % (lx + 174, ly + 5))
p.append('<text x="%.1f" y="%.1f" font-size="12" fill="#8a929c" text-anchor="end">'
         '最大误差 %.2f m / 平均 %.2f m</text>' % (W - MR, H - 20, max(errs), sum(errs) / len(errs)))

p.append("</svg>")
with open("pure_pursuit_result.svg", "w", encoding="utf-8") as f:
    f.write("\n".join(p))
print("已生成 pure_pursuit_result.svg")
