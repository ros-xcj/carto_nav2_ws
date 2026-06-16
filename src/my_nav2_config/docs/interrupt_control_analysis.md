# 下位机导航中断控制逻辑 — 场景深度分析报告

**日期**: 2026-05-09  
**适用系统**: 下位机 Jetson Orin Nano (ROS2 Humble) + 上位机 Jetson Orin Nano (ROS2 Humble) / WiFi  
**相关文件**: `waypoint_commint_patrol.py`

---

## 一、上位机信号的本质语义

首先必须明确两个话题的真实语义，这是所有设计决策的基础：

| 话题 | 类型 | 发送时机 | 语义 |
|------|------|----------|------|
| `control/target_angle` | `Float32` | **仅在语音唤醒/声源定位时发送** | "检测到人声方向，请朝向该方向" |
| `control/mode` | `UInt8` | **上位机有任何活动时持续/变化** | `1`=我正忙，请停止导航 / `0`=我空闲，请继续 |

> **关键发现**：这两个话题是**完全独立的信号**，不存在必然的时序耦合关系。`target_angle` 是一个**即时事件触发**，`mode` 是一个**持续状态指示**。把二者在同一个 `modeCallback` 中耦合处理，是现有逻辑最核心的设计缺陷。

---

## 二、应用场景完整拆分

### 场景 A：语音唤醒 → 对话

```
时序：
  [唤醒词] → 上位机检测到声源方向
      ↓
  target_angle = 90.0  (立即发送，告知声源在右侧90°)
      ↓  (可能间隔 0~500ms，也可能更长)
  mode = 1             (上位机开始处理语音，需要下位机停止)
      ↓  (对话进行中，mode 持续为 1)
  mode = 0             (对话结束，下位机恢复)
```

**下位机预期行为**：
1. 收到 `target_angle=90` → **立即**进入停止+转向（无需等 `mode=1` 到来）
2. 收到 `mode=1` → 若已在执行转向则维持；若尚未转向则停止原地等待
3. 收到 `mode=0` → 恢复导航（含时间补偿）

---

### 场景 B：机器人到达指定点 → 上位机播放音乐/投影

```
时序：
  下位机发布 reached_goal_id = 2
      ↓ (上位机收到，触发内容播放)
  mode = 1             (上位机开始播放，无声源，无角度)
      ↓  (播放过程中，mode 持续为 1，无 target_angle)
  mode = 0             (播放结束)
```

**下位机预期行为**：
1. 收到 `mode=1`（**无任何 `target_angle`**）→ 原地停止，不转向
2. 停留期间持续发零速，不恢复导航
3. 收到 `mode=0` → 恢复导航

---

### 场景 C：语音唤醒 → 短暂对话 → 立刻播放音乐

```
时序：
  target_angle = 135.0   (唤醒，声源定位)
  mode = 1               (停止)
  mode = 0               (对话结束，但…)
  mode = 1               (紧接着音乐播放，无新 target_angle)
```

**下位机预期行为**：
1. `target_angle=135` → 停止+转向（现场保护）
2. `mode=0` → 满足 min_interaction_duration 后恢复导航
3. `mode=1` 再次到来（无 angle）→ 再次停止，**不转向**（没有新的声源）
4. `mode=0` → 再次恢复

---

### 场景 D：唤醒 → 让机器人播放音乐（角度 + 随后长时停止）

```
时序：
  target_angle = 60.0    (唤醒，声源定位)
  mode = 1               (上位机处理"播放音乐"指令)
  -- 处理完毕，开始播放 --
  mode = 1               (持续，上位机正在播放，不会再发 angle)
  -- 播放结束 --
  mode = 0
```

**下位机预期行为**：
1. `target_angle=60` → 停止+转向
2. 转向完成后，因为 `mode=1` 持续有效 → 保持停止状态（不恢复导航）
3. `mode=0` → 恢复

---

### 场景 E：唤醒后立刻打断（min_interaction_duration 保护）

```
时序：
  target_angle = 45.0
  mode = 1
  mode = 0   (1.5s 后，人走开了)
  -- 下位机正在转向中 --
```

**下位机预期行为**：
1. 转向动作不因 `mode=0` 到来太快而被打断（`pending_resume` 机制）
2. 满足 `min_interaction_duration=4s` 后才恢复导航

---

## 三、现有逻辑的缺陷诊断

### 缺陷 1：角度与模式被强耦合在 `modeCallback` 中

```
现有逻辑 (modeCallback):
  收到 mode=1
    → 在 modeCallback 里查看 latest_angle
    → 判断是否转向

问题：
  target_angle 可能比 mode=1 早到 500ms~数秒
  也可能比 mode=1 晚到（上位机先发模式，再计算声源）
  → "2秒新鲜度窗口"是一个脆弱的时间假设，在复杂场景下必然误判
```

### 缺陷 2：`target_angle` 没有自己的独立触发逻辑

```
现有 _angle_callback: 仅记录值和时间戳
  → target_angle 到来时，节点不做任何实际动作
  → 必须等 mode=1 触发才能执行转向
  → 如果 target_angle 在 mode=1 之前 2s+ 到达 → 被"新鲜度"过滤丢弃
```

### 缺陷 3：场景 B 无法区分"需要角度"和"不需要角度"

```
场景 B 中：
  仅收到 mode=1，无 target_angle
  现有逻辑: latest_angle=-1 → 走"停止"分支 ✅ (偶然正确)

  但如果：
  前次唤醒的 latest_angle 因某种原因没被重置
  → 下次 mode=1 到来时被误用为"有效角度" → 错误转向 ⚠️
```

---

## 四、改进后的状态机设计

### 核心设计原则

> **`target_angle` 是独立的即时命令，收到即执行转向；`mode` 是状态信号，控制导航的停止与恢复。两者解耦，互不等待。**

### 新信号处理逻辑

```
_angle_callback(angle):
  ├─ angle >= 0（有效角度）:
  │    ├─ 若当前 state != INTERACTING:
  │    │    → 现场保护 (previous_state = state)
  │    │    → state = INTERACTING
  │    │    → sound_start_time = now
  │    → 执行 _rotate_in_place(angle)   ← 立即行动，不等 mode
  │    → latest_angle = -1               ← 使用后立即重置
  └─ angle < 0: 忽略（哨兵值，不应由上位机发送）

_mode_callback(mode):
  mode == 0（恢复）:
  ├─ 若 state == INTERACTING:
  │    ├─ elapsed < min_interaction_duration → pending_resume=True
  │    └─ elapsed >= threshold → _do_resume()

  mode == 1（停止）:
  ├─ 若 state != INTERACTING:
  │    → 现场保护
  │    → state = INTERACTING
  │    → sound_start_time = now（若尚未设置）
  → 若 is_rotating == False:        ← 仅在不转向时发零速
       _publish_zero_vel()           ← 不取消转向动作！
  （不查询 latest_angle，不主动触发转向）
```

### 新状态转移图

```
                    ┌──────────────────────────────┐
                    │           INIT               │
                    └──────────────┬───────────────┘
                                   │ 启动
                    ┌──────────────▼───────────────┐
              ┌────►│         NAVIGATING            │◄────────┐
              │     └──────────────┬───────────────┘         │
              │                    │ 距离达到                  │
              │     ┌──────────────▼───────────────┐         │
              │     │       REACHED_WAITING         │         │
              │     └──────────────┬───────────────┘         │
              │                    │                          │
              │    mode=1 / target_angle(>=0)                 │
              │     ┌──────────────▼───────────────┐  mode=0  │
              │     │          INTERACTING          ├──────────┘
              │     │                               │
              │     │  ┌──────────┐ ┌────────────┐ │
              └─────┤  │ STOPPING │ │  ROTATING  │ │
                    │  │(零速循环)│ │(goToPose)  │ │
                    │  └──────────┘ └────────────┘ │
                    └───────────────────────────────┘

触发条件：
  → INTERACTING: mode=1 OR target_angle(>=0) 任一到来即触发
  → STOPPING 子状态: mode=1 到来且 is_rotating=False
  → ROTATING 子状态: target_angle(>=0) 到来（无论 mode 是否同时到来）
  → 恢复: mode=0 且 elapsed >= min_interaction_duration
```

---

## 五、各场景验证（新逻辑）

### 场景 A 验证

| 时序 | 新逻辑处理 | 状态 |
|------|-----------|------|
| `target_angle=90` 到来 | 立即现场保护 + `_rotate_in_place(90)` | → INTERACTING/ROTATING |
| `mode=1` 到来（0.5s后）| 已在INTERACTING，`is_rotating=True` → 不发零速 | INTERACTING/ROTATING |
| 转向完成 | `is_rotating=False` | INTERACTING/STOPPING |
| `mode=0` | `_do_resume()` → 恢复 | → NAVIGATING/REACHED_WAITING |

✅ **正确**

---

### 场景 B 验证

| 时序 | 新逻辑处理 | 状态 |
|------|-----------|------|
| `mode=1` 到来（无angle）| 现场保护 + state=INTERACTING | → INTERACTING |
| controlLoop 10Hz | `is_rotating=False` → 发零速 | INTERACTING/STOPPING |
| `mode=0` | 恢复 | → NAVIGATING |

✅ **正确，不会触发任何转向**

---

### 场景 C 验证

| 时序 | 新逻辑处理 | 状态 |
|------|-----------|------|
| `target_angle=135` | 现场保护 + 转向 | INTERACTING/ROTATING |
| `mode=0` (2s后) | elapsed < 4s → `pending_resume=True` | INTERACTING（等待） |
| 转向完成(3s) | `is_rotating=False` | INTERACTING/STOPPING |
| 满4s | `pending_resume` 触发 → 恢复 | → NAVIGATING |
| `mode=1` (1s后，音乐) | 现场保护 + INTERACTING（**无angle**）| INTERACTING/STOPPING |
| `mode=0` | 恢复 | → NAVIGATING |

✅ **正确，两次中断各自独立处理**

---

### 场景 D 验证

| 时序 | 新逻辑处理 | 状态 |
|------|-----------|------|
| `target_angle=60` | 转向 | INTERACTING/ROTATING |
| `mode=1` | 已在INTERACTING，不重置sound_start_time | INTERACTING/ROTATING |
| 转向完成 | `is_rotating=False` | INTERACTING/STOPPING |
| 音乐播放中，`mode=1` 持续 | controlLoop 发零速 | INTERACTING/STOPPING |
| `mode=0` | 恢复 | → 恢复 |

✅ **正确**

---

### 场景 E 验证

| 时序 | 新逻辑处理 | 状态 |
|------|-----------|------|
| `target_angle=45` (t=0) | 转向开始 | INTERACTING/ROTATING |
| `mode=0` (t=1.5s) | elapsed=1.5s < 4s → `pending_resume=True` | INTERACTING |
| t=3s，转向完成 | `is_rotating=False` | INTERACTING/STOPPING |
| t=4s（pending触发）| `_do_resume()` | → 恢复 |

✅ **正确，转向完整执行**

---

## 六、对现有 `waypoint_commint_patrol.py` 的改动建议

### 改动 1：`_angle_callback` — 增加即时行动逻辑

```python
def _angle_callback(self, msg: Float32):
    angle = msg.data
    if angle < 0.0:
        return  # 哨兵值，忽略
    
    self.get_logger().info(f'[angleCallback] 收到有效角度 {angle:.2f}°，立即执行转向')
    
    # 若尚未进入 INTERACTING，先做现场保护
    if self.state != PatrolState.INTERACTING:
        self.previous_state = self.state
        self.state = PatrolState.INTERACTING
        self.pending_resume = False
        self.sound_start_time = self.get_clock().now()
        self.get_logger().info(
            f'[angleCallback] 中断 {self.previous_state.name}，进入 INTERACTING')
    
    # 立即执行转向（不等 mode=1）
    self.is_rotating = True
    self._rotate_in_place(angle)
    # latest_angle 不再需要存储，使用后即丢弃
```

### 改动 2：`_mode_callback` — 剥离角度判断，仅负责状态控制

```python
def _mode_callback(self, msg: UInt8):
    mode = msg.data

    if mode == 0:
        # 恢复逻辑不变
        if self.state == PatrolState.INTERACTING:
            elapsed = self._elapsed_sec(self.sound_start_time)
            if elapsed < self.min_interaction_duration:
                self.pending_resume = True
                return
            self._do_resume(elapsed)
        return

    # mode != 0: 停止模式
    if self.state != PatrolState.INTERACTING:
        self.previous_state = self.state
        self.state = PatrolState.INTERACTING
        self.pending_resume = False
        self.is_rotating = False
        self.sound_start_time = self.get_clock().now()
    
    # ★ 不再查询 latest_angle，不主动触发转向
    # ★ 转向只由 _angle_callback 触发
    # 仅在非旋转状态下确保停止
    if not self.is_rotating:
        self.navigator.cancelTask()
        self._publish_zero_vel(count=1)
```

### 改动 3：移除 `latest_angle` / `angle_update_time` / `angle_freshness_window`

这三个字段与"2秒新鲜度"机制一起可以完全移除，因为角度在 `_angle_callback` 中被即时消费、即时丢弃，不再需要存储和过期判断。

---

## 七、改动后的变量精简对比

| 字段 | 原逻辑 | 新逻辑 |
|------|--------|--------|
| `latest_angle` | ✅ 需要（跨回调传递） | ❌ 可移除 |
| `angle_update_time` | ✅ 需要（新鲜度判断） | ❌ 可移除 |
| `angle_freshness_window` | ✅ 参数 | ❌ 可移除 |
| `is_rotating` | ✅ 需要 | ✅ 保留 |
| `pending_resume` | ✅ 需要 | ✅ 保留 |
| `sound_start_time` | ✅ 需要 | ✅ 保留 |
| `previous_state` | ✅ 需要 | ✅ 保留 |
| `min_interaction_duration` | ✅ 参数 | ✅ 保留 |

---

## 八、结论

现有的"2秒新鲜度窗口"方案在单一场景（唤醒后立刻发模式）下可以工作，但在真实应用场景中存在**根本性设计缺陷**：将独立的事件信号（角度）与状态信号（模式）强耦合，依赖脆弱的时序假设。

**推荐改进方向**：

> `target_angle` = **即时命令**，收到即在 `_angle_callback` 中执行，无需等待 `mode`  
> `control/mode` = **状态信号**，仅控制导航的停止/恢复，不涉及角度判断

这样设计的核心优势：
- **零时序依赖**：angle 和 mode 的到达顺序、时间差完全不影响结果
- **逻辑简化**：消除 `latest_angle`、`angle_update_time`、`freshness_window` 三个脆弱字段
- **场景覆盖完整**：5个典型场景均可正确处理
- **可维护性更高**：每个回调只负责一件事，职责单一

---

*本报告建议在下一次代码迭代中将 `waypoint_commint_patrol.py` 按第六节所述改动进行重构。*
