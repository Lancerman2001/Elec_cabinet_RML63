# Selector local protocol v1

TCP 仅监听本机回环地址 `127.0.0.1:8767`，`--host` 只接受 `127.0.0.1` / `localhost`。本协议没有身份认证或 TLS，不用于局域网或公网服务。UTF-8 JSON，每条以换行结束，单条不超过16 KiB。组件标识默认 `xuanniu`；版本或标识不匹配的指令直接忽略。无订阅握手：连接后约50 Hz推送状态。

```json
{"version":1,"component":"xuanniu","type":"command","stamp":1789212000.0,"angle_rad":0.7853981633974483,"dragging":false}
```

`stamp` 必须为发送时的 `time.time()`（Unix秒）；上例仅展示格式。超过0.5秒、未来超过0.1秒的命令拒绝执行。`angle_rad` 必须有限且在±π/4内。`dragging=true` 连续目标，推荐30 Hz发送；`false` 按目标吸附三档，±π/8分界取停。

其他消息保留相同 `version/component/stamp`：

| type | 额外字段 | 行为 |
|---|---|---|
| `end_drag` | 无 | 仅结束当前连接拥有的拖动，从实际角度吸附 |
| `pause` | `paused`: boolean | 暂停/恢复；暂停立即结束拖动 |
| `reset` | 无 | 独立演示恢复初始停；电柜适配时只重置对应模块 |

响应格式：

```json
{"version":1,"component":"xuanniu","type":"status","state":{"time":2.5,"angle_rad":0.0,"velocity_rad_s":0.0,"torque_nm":0.0,"target_angle_rad":0.0,"mode":"STOP","dragging":false,"moving":false,"error":"","paused":false}}
```

`time` 为MuJoCo仿真秒。档位为 `MANUAL/STOP/AUTO/TRANSITION`，实际反馈用于确认命令结果；发送成功只表示传输成功。错误包括 `INVALID_ANGLE`、`INVALID_COMMAND`、`STALE_COMMAND`、`SIMULATION_NOT_READY`、`TARGET_TIMEOUT`、`NONFINITE_PHYSICS`。

服务不保留待重放指令。拖动连接断开后立即从实际位置吸附；没有连接关闭通知时用0.5秒断流超时兜底。多个客户端采用最新有效命令，只有当前拖动所有者断开时才结束该拖动；关闭旁观客户端不会改变操作状态。GUI反馈过期0.5秒禁用动作，恢复连接后重新读取实际状态。

宿主适配器需要 `lock`（互斥锁）、`simulation`（控制器）和 `feedback() -> (status_dataclass, monotonic_received)`。宿主负责单一物理线程及50 Hz采样，`StateServer` 负责通信。适配器可额外提供 `enqueue(callback)` 将命令转交物理线程，以及 `apply_pause(bool)` / `apply_reset()` 同步原生查看器状态；没有队列接口时回退到原有互斥锁调用。客户端发送在线程中完成，连续未发送拖动目标合并为最新值，离散指令清除未发送的旧拖动。TCP两端启用 `TCP_NODELAY`。控制器的物理更新接口见包根目录 [XUANNIU.md](../XUANNIU.md)。
