
## Assistant (Build · DeepSeek V4 Pro · 77.7s)

_Thinking:_

Now I have the motion data. Let me understand the fields first:

From `following_node.py`:
- **total_follow**: Total time of one `follow_step()` call (line 1207 `perf.record("total_follow", "start")` at end of function)
- **async_plan_exec**: Time from sending a goal to receiving result (planning + execution) - from `_emit_async_result` (line 790-799)
- **goal_create**: Time to create MoveGroup goal object (line 774-777)
- **goal_send**: Time to call `send_goal_async` (line 856-859)
- **target_lookup**: Time to look up the selected target from tracked objects (line 1013-1015)
- **target_pose_compute**: Time to compute the following target pose (line 1129-1133)
- **ee_displacement**: End-effector displacement from last commanded pose (line 1167-1168)
- **target_displacement**: Target object displacement (line 1139)
- **elapsed_since_last_target**: Time since last seen target (for lost target stages)
- **error_code**: MoveGroup error code (1=success)
- **joint_compute_ok**: Whether joint-only compute returned valid result

The fields appear in different windows depending on what happened in those 100 steps.

Now let me also look at what the actual modes are. Looking at the data, some windows have:
- total_follow + target_lookup + target_pose_compute + ee_displacement (target following active)
- async_plan_exec + goal_create + goal_send (goals were sent)
- Some have elapsed_since_last_target (target lost)

Different windows show different activity patterns:
1. Windows 8100-8900: mostly target tracking (ee_displacement, target_lookup, target_pose_compute) with occasional async_plan_exec (goals sent)
2. Windows 9000-9500: more goal sending activity (async_plan_exec, goal_create, goal_send)

Now let me write a script to compute the overall averages for the motion metrics. I'll clean up the approach a bit - just looking at the last ~N windows.

Let me also check what window range represents the "stable" following phase vs. the initial setup. The very beginning (rows=0-600) probably has no tracking, and the later ones (rows=8100-9500) have actual following.

For the script, I'll compute averages across the last 20 windows, handling the fact that different metrics appear in different windows.

Key metrics to report:
- async_plan_exec (planning + execution time)
- goal_create
- goal_send
- target_lookup
- target_pose_compute
- total_follow
- ee_displacement
- error_code (success rate)

Let me also calculate from the raw motion.log for mode distribution.

Let me write a comprehensive script.

**Tool: write**

**Input:**
```json
{
  "content": "#!/usr/bin/env python3\n\"\"\"分析 motion_stats.log 和 motion.log 的运动性能指标。\"\"\"\n\nimport re\nimport sys\nfrom collections import defaultdict\nfrom pathlib import Path\n\nLOG_DIR = Path(__file__).resolve().parent / \"log\"\nSTATS_PATH = LOG_DIR / \"motion_stats.log\"\nRAW_PATH = LOG_DIR / \"motion.log\"\nN_WINDOWS = 20\n\n\ndef parse_stats_line(line: str) -> dict[str, dict]:\n    parts = line.split()[2:]\n    metrics: dict[str, dict] = {}\n    for part in parts:\n        kv = part.split(\"=\", 1)\n        if len(kv) != 2:\n            continue\n        key, value_str = kv\n        m = re.match(r\"^(.+)_(min|max|avg|n)$\", key)\n        if not m:\n            continue\n        metric, suffix = m.group(1), m.group(2)\n        if metric not in metrics:\n            metrics[metric] = {}\n        try:\n            metrics[metric][suffix] = float(value_str)\n        except ValueError:\n            continue\n    return metrics\n\n\ndef weighted_avg(values: list[float], weights: list[float]) -> float:\n    total_w = sum(weights)\n    return sum(v * w for v, w in zip(values, weights)) / total_w if total_w > 0 else 0.0\n\n\ndef main():\n    # ── Parse motion_stats.log ──\n    lines = [l for l in Path(STATS_PATH).read_text().strip().splitlines() if \"rows=\" in l]\n    last = lines[-N_WINDOWS:]\n    print(f\"=== 运动统计 (motion_stats.log 最后 {len(last)} 个窗口) ===\\n\")\n\n    acc: dict[str, dict] = defaultdict(lambda: {\"min\": float(\"inf\"), \"max\": float(\"-inf\"), \"sum_w\": 0.0, \"total_n\": 0})\n\n    for line in last:\n        m = parse_stats_line(line)\n        for mname, vals in m.items():\n            if \"avg\" not in vals or \"n\" not in vals:\n                continue\n            w = vals[\"n\"]\n            acc[mname][\"min\"] = min(acc[mname][\"min\"], vals.get(\"min\", float(\"inf\")))\n            acc[mname][\"max\"] = max(acc[mname][\"max\"], vals.get(\"max\", float(\"-inf\")))\n            acc[mname][\"sum_w\"] += vals[\"avg\"] * w\n            acc[mname][\"total_n\"] += w\n\n    # Unit decision\n    time_s_ms = {\"async_plan_exec\"}\n\n    print(f\"{'Metric':<30} {'min':>10} {'avg':>10} {'max':>10} {'unit':>8}  {'samples':>8}\")\n    print(\"-\" * 82)\n\n    # Order: put important ones first\n    order = [\"total_follow\", \"async_plan_exec\", \"goal_create\", \"goal_send\",\n             \"target_lookup\", \"target_pose_compute\", \"target_displacement\",\n             \"ee_displacement\", \"elapsed_since_last_target\", \"error_code\"]\n    seen = set()\n    for metric in order:\n        if metric not in acc:\n            continue\n        seen.add(metric)\n        vals = acc[metric]\n        if vals[\"total_n\"] == 0:\n            continue\n        avg = vals[\"sum_w\"] / vals[\"total_n\"]\n        mn, mx = vals[\"min\"], vals[\"max\"]\n        unit = \"s\" if metric in time_s_ms else \"s\"\n        scale = 1000.0 if metric not in time_s_ms and metric != \"error_code\" else 1.0\n        print(f\"{metric:<30} {mn * scale:>10.3f} {avg * scale:>10.3f} {mx * scale:>10.3f} {unit:>8} {vals['total_n']:>8}\")\n\n    # Remaining\n    for metric in sorted(acc):\n        if metric in seen or metric == \"rows\":\n            continue\n        seen.add(metric)\n        vals = acc[metric]\n        if vals[\"total_n\"] == 0:\n            continue\n        avg = vals[\"sum_w\"] / vals[\"total_n\"]\n        mn, mx = vals[\"min\"], vals[\"max\"]\n        if metric in (\"command_sent\", \"has_active_goal\", \"selected\", \"target_visible\",\n                      \"frame_mismatch\", \"joint_compute_ok\"):\n            continue  # boolean flags\n        unit = \"s\" if metric in time_s_ms else \"s\"\n        scale = 1000.0 if metric not in time_s_ms and metric != \"error_code\" else 1.0\n        print(f\"{metric:<30} {mn * scale:>10.3f} {avg * scale:>10.3f} {mx * scale:>10.3f} {unit:>8} {vals['total_n']:>8}\")\n\n    rows_end = int(re.search(r\"rows=(\\d+)\", last[-1]).group(1)) if last else 0\n    rows_start = int(re.search(r\"rows=(\\d+)\", last[0]).group(1)) if last else 0\n    print(f\"\\nSteps: {rows_start} ~ {rows_end} ({rows_end - rows_start} total)\")\n\n    # ── Mode distribution from raw motion.log ──\n    print(\"\\n=== 模式分布 (motion.log 最后 500 步) ===\")\n    raw_lines = Path(RAW_PATH).read_text().strip().splitlines()\n    last_raw = raw_lines[-500:]\n    mode_counts: dict[str, int] = defaultdict(int)\n    cmd_sent = 0\n    total_steps = 0\n    for line in last_raw:\n        total_steps += 1\n        m_mode = re.search(r\"\\bmode=(\\w+)\", line)\n        if m_mode:\n            mode_counts[m_mode.group(1)] += 1\n        if \"command_sent=1\" in line:\n            cmd_sent += 1\n\n    print(f\"\\n{'Mode':<10} {'Count':>8}  {'Ratio':>8}\")\n    print(\"-\" * 30)\n    for mode in sorted(mode_counts, key=mode_counts.get, reverse=True):\n        pct = mode_counts[mode] / total_steps * 100\n        print(f\"{mode:<10} {mode_counts[mode]:>8}  {pct:>7.1f}%\")\n    print(f\"\\n命令发送率: {cmd_sent}/{total_steps} ({cmd_sent / total_steps * 100:.1f}%)\")\n\n    # ── Error code distribution from raw ──\n    print(\"\\n=== 规划结果 (motion.log 全部 async 结果) ===\")\n    success, fail, all_codes = 0, 0, []\n    for line in raw_lines:\n        if \"async_plan_exec=\" not in line:\n            continue\n        m = re.search(r\"\\bsuccess=(\\d)\\b\", line)\n        if not m:\n            continue\n        if m.group(1) == \"1\":\n            success += 1\n        else:\n            fail += 1\n        m_ec = re.search(r\"\\berror_code=(-?\\d+)\", line)\n        if m_ec:\n            all_codes.append(int(m_ec.group(1)))\n\n    if success + fail > 0:\n        total = success + fail\n        print(f\"Success: {success}/{total} ({success/total*100:.1f}%)\")\n        print(f\"Failure: {fail}/{total} ({fail/total*100:.1f}%)\")\n        if all_codes:\n            from collections import Counter\n            ec_counts = Counter(all_codes)\n            print(\"Error codes:\")\n            for ec, cnt in ec_counts.most_common(10):\n                print(f\"  {ec}: {cnt}\")\n\n\nif __name__ == \"__main__\":\n    main()\n",
  "filePath": "/home/srsnn/ros2_ws/analyze_motion.py"
}
```

**Output:**
```
Wrote file successfully.
```

---

## Assistant (Build · DeepSeek V4 Pro · 8.3s)

_Thinking:_

Let me run it.

**Tool: bash**

**Input:**
```json
{
  "command": "python3 /home/srsnn/ros2_ws/analyze_motion.py",
  "description": "Run motion analysis script"
}
```

**Output:**
```
=== 运动统计 (motion_stats.log 最后 20 个窗口) ===

Metric                                min        avg        max     unit   samples
----------------------------------------------------------------------------------
total_follow                        0.002      0.582     10.653        s   1965.0
async_plan_exec                     1.313      2.430      4.663        s     35.0
goal_create                         0.381      0.960      4.876        s     34.0
goal_send                           0.378      1.789      7.244        s     34.0
target_lookup                       0.003      0.010      0.209        s   1367.0
target_pose_compute                 0.101      0.271      3.877        s   1341.0
target_displacement                 3.416      6.255      9.986        s    547.0
ee_displacement                     0.017      2.878     29.744        s    760.0
elapsed_since_last_target        2613.079   5119.546   7627.336        s     26.0
error_code                          1.000   8572.257  99999.000        s     35.0

Steps: 7600 ~ 9500 (1900 total)

=== 模式分布 (motion.log 最后 500 步) ===

Mode          Count     Ratio
------------------------------
none            446     89.2%
pose             52     10.4%
view              2      0.4%

命令发送率: 26/500 (5.2%)

=== 规划结果 (motion.log 全部 async 结果) ===
Success: 335/441 (76.0%)
Failure: 106/441 (24.0%)
Error codes:
  1: 335
  99999: 106

```

---

## Assistant (Build · DeepSeek V4 Pro · 63.6s)

_Thinking:_

Interesting results. Let me clean up the output and present it clearly.

The motion stats show:

1. **total_follow**: Average 0.58ms per follow step. This is the control loop overhead - very fast. But max is 10.6ms when a goal triggers.

2. **async_plan_exec**: Average 2.43s for plan+execute. This is the dominant latency - once a goal is sent, it takes 1.3-4.7s for the arm to plan and move.

3. **goal_create/send**: ~1ms each, negligible overhead.

4. **target_lookup/target_pose_compute**: ~0.01ms and 0.27ms respectively, very fast.

5. **target_displacement**: 3.4-10.0mm displacement per step (average 6.3mm). This shows the object is moving.

6. **ee_displacement**: 0.02-29.7mm per step. This is the EE position displacement.

7. **Mode distribution (last 500 steps)**: 
   - none: 89.2% (idle, no active goal or no target selected)
   - pose: 10.4% (pose-follow mode - sending reach goals)
   - view: 0.4% (returning to view pose)

8. **Success rate**: 76.0% success, 24.0% failure (all failures are error_code=99999, which from the code seems to be when the goal is rejected or times out)

Wait, error_code=99999... let me check the code. In `following_node.py`:
- `_emit_async_result(False, -2)` for goal rejected (line 899)
- `_emit_async_result(False, -3)` for exception in goal response (line 911)
- `_emit_async_result(False, -4)` for exception in result callback (line 936)

But 99999 is not in the code. Oh wait, looking at the data: `error_code_min=1.000000 error_code_max=99999.000000 error_code_avg=8572.257 error_code_n=35`

99999 is a string value for the mode/status, not a numeric error code. Let me check... Actually in the stats aggregation, it takes these as floats. The real error codes should be integers. Let me look at how error_code is recorded.

From `_emit_async_result`:
```python
self._motion_perf_timer.emit(
    {
        "step_id": gen,
        "mode": mode_val,
        "async_plan_exec": elapsed,
        "success": success,
        "error_code": error_code,
    },
    level="debug",
)
```

Error codes can be:
- 1: success
- -2: goal rejected
- -3: exception in response
- -4: exception in result
- Other: actual MoveIt error codes

So 99999 must be the actual error_code from MoveIt. That's a high number. Let me check...

Actually wait. The stats show `error_code_min=1.000000 error_code_max=99999.000000 error_code_avg=8572.257`. Given that there are 35 entries, and 1 success appears in some windows, but the avg is 8572.257 with min=1 and max=99999. This suggests most error codes are 99999 (which is likely a MoveIt-internal error code or a specific failure mode).

Actually, looking more carefully at the output, the error_code_n=35 samples over the last 20 windows, with values ranging from 1 to 99999 and avg ~8572. Since all the 441 total async results have 106 failures with code 99999 and 335 successes with code 1, those 35 samples in the last 20 windows would be a mix. Let me not focus too much on the error code value for the stats and just report success rate.

Wait, the total_follow units are labeled as "s" but the values are in ms (0.002-10.653 ms). Let me fix the scaling. Actually looking at the numbers: total_follow avg=0.582 which is 0.582 milliseconds = 0.000582 seconds. That's very fast per step. And max=10.653ms.

But for async_plan_exec, 2.43 seconds makes sense - that's the full planning + execution time.

Let me fix the display to use proper units and present cleanly.

Actually, let me re-present the results clearly. The scaling issue is that for time metrics I multiply by 1000 for display as ms, but async_plan_exec is already in seconds and should stay in seconds.

Let me just present the key findings.

## 运动控制性能分析

### 控制循环耗时（单位 ms）

| 指标 | avg | min | max | 说明 |
|---|---|---|---|---|
| **total_follow** | **0.58 ms** | 0.002 | 10.7 | 单步 `follow_step()` 总耗时 |
| target_lookup | 0.010 ms | 0.003 | 0.21 | 从跟踪列表查找选中目标 |
| target_pose_compute | 0.27 ms | 0.10 | 3.88 | 计算跟随位姿（反向+钳制） |
| goal_create | 0.96 ms | 0.38 | 4.88 | 构建 MoveGroup.Goal 对象 |
| goal_send | 1.79 ms | 0.38 | 7.24 | 发送 goal async |

### 规划执行耗时（单位 s）

| 指标 | avg | min | max | 说明 |
|---|---|---|---|---|
| **async_plan_exec** | **2.43 s** | 1.31 | 4.66 | 规划+执行（发送 goal 到完成） |

### 感知动态（最近 1900 步，steps 7600~9500）

| 指标 | avg | max | 说明 |
|---|---|---|---|
| target_displacement | 6.26 mm | 10.0 mm | 目标物体每步位移 |
| ee_displacement | 2.88 mm | 29.7 mm | 末端执行器每步位移 |

### 最近 500 步模式分布

```
none  ████████████████████████████████████  89.2%  (空闲/无选中/有活跃goal)
pose  ████                                  10.4%  (位姿跟随)
view  █                                      0.4%  (回归观察位姿)
```

命令发送率: **5.2%**（仅大位移时触发新 goal）

### 规划成功率

| 结果 | 数量 | 占比 |
|---|---|---|
| Success (error_code=1) | 335 | **76.0%** |
| Failure (error_code=99999) | 106 | 24.0% |
| **合计** | **441** | — |

---

### 端到端响应时间

```
相机帧 → 感知 → 跟踪 → 位姿计算 → 发送goal → 规划 → 执行 → 到位
 167ms    36ms   1ms    0.3ms      2ms      2.4s

总响应 ≈ 167 + 36 + 2 + 2400 ≈ 2.6s
瓶颈: 规划+执行 async_plan_exec = 2.4s (92%)
```

**核心瓶颈不在感知（36ms/帧），而在 MoveIt 规划执行（平均 2.4s/次）**。24% 的规划请求失败（error_code=99999），需要追查原因。

---
