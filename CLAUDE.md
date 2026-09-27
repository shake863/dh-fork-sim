# dh-fork-sim

速降（DH）自行车前叉压缩原理的交互演示，给用户自己学习用。单文件网页，发布在 GitHub Pages：
https://shake863.github.io/dh-fork-sim/ （仓库 `shake863/dh-fork-sim`，**公开仓库**，提交的任何内容都会公开）。

- 与用户交流用中文。
- 学习资料见 [docs/knowledge.md](docs/knowledge.md)，每轮开发记录见 [docs/devlog.md](docs/devlog.md)。

## 约束

- **单文件、零构建**：全部 HTML/CSS/JS 在 `index.html`，浏览器直接打开即可用。不要引入打包工具或框架。
- 唯一外部依赖：Tabler Icons webfont（jsdelivr CDN），只用 outline 图标 `ti ti-*`。
- 颜色走 `:root` 里的 CSS 变量（`--text-*`、`--surface-*`、`--border*` 等），带 `prefers-color-scheme: dark` 覆盖；图形里的彩色用固定中间色（如 `#378ADD`、`#1D9E75`、`#D85A30`、`#D4537E`、`#7F77DD`），深浅模式都可读。canvas 文字颜色每 60 帧从 CSS 变量重新读取。
- 窄屏（≤640px）改为单列布局，改布局时注意别破坏。
- 所有显示的数字都要取整或 `toFixed`。
- 页面底部有模型免责声明；涉及 Fox 等真实产品的描述要保守，不确定的规格不要写成事实。

## index.html 结构

按顺序：`<style>` → 控制按钮三行（结构 / 阻尼 / 地形）→ 左列 SVG 前叉剖面 + 图例 + 结构说明 → 右列 5 个指标卡、当前回路、三个 canvas（弹簧力-行程、阻尼力-轴速、行程-时间）→ 滑块 → `<script>`。

脚本关键部分：

| 名称 | 作用 |
|---|---|
| `P` | 所有可调参数（体重、气压、垫片、GRIP2 四旋钮 `lsc/hsc/lsr/hsr`、GRIP 两旋钮 `c1/r1`） |
| `MODE` / `DAMP` / `HL` | 正置 `n` / 倒置 `i`；阻尼 `g1`=GRIP、`g2`=GRIP2；是否高亮簧下部分 |
| `pp` `pn` `Fs` | 正气室、负气室压力（psi）和净弹簧力（N） |
| `D()` `FdP(v,d)` `invReb` | 当前阻尼参数、分段线性阻尼力、车轮离地时由弹簧力反解回弹速度 |
| `ROAD` `brk` | 各地形的路面高度函数；急刹的额外下压力包络 |
| `reset(sc)` `step(dt)` | 场景重置（同时把上一次运行存为灰线 `ghost`）；物理积分 |
| `drawFork` `drawSp` `drawDa` `drawTr` `readouts` | 每帧渲染 |
| `band()` | 阻尼图灰带：最近拖动的旋钮从最小到最大的曲线范围 |

## 物理模型（简化）

- 单一簧上质量 `M = (体重 + 16 kg) × 0.4`，车轮无质量、轮胎刚性，行程 200 mm。
- 空气弹簧：多变指数 `G = 1.25`，活塞面积 `A = 8.04e-4 m²`，正气室等效长度 `320 − 25 × 垫片数` mm，负气室 60 mm，正负气室在全伸展时压力相等（净力为 0）。
- 阻尼：压缩和回弹各自分段线性（一个开启点，之前低速斜率、之后高速斜率）。
  - GRIP2：`cl=150+60·lsc`，`kc=0.25+0.05·hsc`，`ch=40+55·hsc`，`crl=250+90·lsr`，`kr=0.3+0.03·hsr`，`crh=150+120·hsr`
  - GRIP：`cl=150+100·c1`，`kc=0.35`，`ch=60+35·c1`，`crl=250+170·r1`，`kr=0.4`，`crh=0.6·crl`
- 车轮离地：前叉以“弹簧力 = 回弹阻尼力”对应的速度自由伸展，对车架作用力为 0。回弹太慢时会出现 packing。
- 触底：超过 200 mm 后加刚度 4e5 N/m 的缓冲。
- 积分：半隐式欧拉，子步长 0.4 ms。倒置结构只改变画面，物理完全相同（因为车轮无质量）。

默认参数（80 kg、100 psi、2 个垫片）下 sag 约 41 mm（20%）。落差场景 `H = 0.7 m` 的调参目标是：默认参数刚好不触底，去掉垫片会触底。改物理参数后要重新核对这些目标，方法见 devlog。

## 开发与验证

- 本地预览：`.claude/launch.json` 里的 `fork-sim`（`python3 -m http.server 8765`），打开 `http://localhost:8765/`。
- 改完至少检查：切换正置/倒置、GRIP/GRIP2，每个地形各跑一次，拖动前叉，控制台无报错；需要时再切到窄屏和深色模式看一眼。
- 发布：push 到 `main`，GitHub Pages（legacy，来源 `main` 分支根目录）大约 1 分钟后更新。push 前先和用户确认。
