"""Complete bilingual UI catalogue; also the bundled font's glyph source."""
TEXT = {
    'subtitle': ('三档自保持转换开关', 'Three-position maintained selector'),
    'MANUAL': ('手动', 'Manual'), 'STOP': ('停', 'Stop'), 'AUTO': ('自动', 'Auto'),
    'TRANSITION': ('切换中', 'Moving'), 'waiting': ('等待仿真反馈', 'Waiting for simulation'),
    'left': ('左转一档', 'Turn left'), 'stop': ('回到停', 'Return to stop'), 'right': ('右转一档', 'Turn right'),
    'actual': ('实际角度', 'Actual angle'), 'target': ('目标角度', 'Target angle'),
    'connected': ('已连接 Gazebo', 'Connected to Gazebo'),
    'stale': ('仿真暂停或连接中断', 'Simulation paused or disconnected'),
    'dragging': ('正在拖动，松手后入档', 'Dragging; release to select a detent'),
    'hint': ('拖动黑色拨杆，或使用按钮切换。正面观察：左转为手动，右转为自动。', 'Drag the black handle or use the buttons. Front view: left is Manual, right is Auto.'),
    'legend': ('实心拨杆：实际位置    虚线：拖动目标', 'Solid handle: actual position    Dashed line: drag target'),
    'INVALID_ANGLE': ('指令角度无效或超出范围', 'Invalid or out-of-range angle'),
    'INVALID_COMMAND': ('指令无效', 'Invalid command'),
    'STALE_COMMAND': ('已忽略过期指令', 'Stale command ignored'),
    'SIMULATION_NOT_READY': ('仿真未就绪', 'Simulation not ready'),
    'TARGET_TIMEOUT': ('切换超时，请回停重试', 'Motion timed out; return to stop'),
    'NONFINITE_PHYSICS': ('物理状态异常，请重置仿真', 'Invalid physics; reset simulation'),
}
