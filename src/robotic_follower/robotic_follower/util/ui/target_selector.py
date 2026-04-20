#!/usr/bin/env python3
"""目标选择器 UI 逻辑"""

import tkinter as tk
from collections.abc import Callable
from tkinter import ttk

from robotic_follower.util.handler import NodeHandler


class TargetSelectorUI(NodeHandler):
    """目标选择器 UI"""

    # 列表刷新最小间隔（毫秒），避免刷新过快导致选中丢失
    REFRESH_INTERVAL_MS = 200

    def __init__(
        self,
        on_select: Callable[[int], None] | None = None,
        parent_node=None,
    ):
        """初始化 UI

        Args:
            on_select: 选中目标时的回调函数，参数为 track_id (int)
            parent_node: ROS2 节点，用于日志输出
        """
        super().__init__(parent_node)
        self._on_select = on_select
        self._tracked_objects: list[dict] = []
        self._selected_track_id: int | None = None
        self._running = True
        self._refresh_scheduled = False
        self._last_display_items: list[str] = []

        self._create_gui()

    def _create_gui(self):
        """创建 GUI 界面"""
        self.root = tk.Tk()
        self.root.title("目标跟随 - 目标选择器")
        self.root.geometry("400x350")
        self.root.resizable(True, True)

        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 标题
        title_label = ttk.Label(main_frame, text="跟踪目标列表", font=("", 12, "bold"))
        title_label.pack(pady=(0, 10))

        # 目标列表框架
        list_frame = ttk.Frame(main_frame)
        list_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

        # 列表框 + 滚动条
        scrollbar = ttk.Scrollbar(list_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.listbox = tk.Listbox(
            list_frame,
            yscrollcommand=scrollbar.set,
            font=("", 10),
            height=8,
        )
        self.listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.listbox.yview)

        # 绑定点击事件
        self.listbox.bind("<Double-Button-1>", self._on_item_double_click)

        # 当前状态
        status_frame = ttk.Frame(main_frame)
        status_frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(status_frame, text="当前跟随:").pack(side=tk.LEFT)
        self.status_label = ttk.Label(
            status_frame, text="无", foreground="red", font=("", 10, "bold")
        )
        self.status_label.pack(side=tk.LEFT, padx=(5, 0))

        # 按钮框架
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X)

        self.follow_btn = ttk.Button(
            button_frame, text="开始跟随", command=self._on_follow_click
        )
        self.follow_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 5))

        self.cancel_btn = ttk.Button(
            button_frame, text="取消跟随", command=self._on_cancel_click
        )
        self.cancel_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 5))

        self.refresh_btn = ttk.Button(
            button_frame, text="刷新", command=self._refresh_list
        )
        self.refresh_btn.pack(side=tk.LEFT, expand=True, fill=tk.X)

        # 关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def update_targets(self, targets: list[dict]):
        """更新目标列表

        Args:
            targets: 目标列表，每个元素包含 track_id, label, position, size
        """
        self._tracked_objects = targets
        self._schedule_refresh()

    def _schedule_refresh(self):
        """安排刷新，避免过于频繁导致选中状态丢失。"""
        if self._refresh_scheduled:
            return
        self._refresh_scheduled = True
        self.root.after(self.REFRESH_INTERVAL_MS, self._do_scheduled_refresh)

    def set_selected(self, track_id: int | None):
        """设置当前选中的目标（由 ROS 回调触发，需要节流刷新）。"""
        self._selected_track_id = track_id
        self._refresh_list()

    def _force_refresh(self):
        """强制立即刷新列表（选中操作时调用，确保用户反馈即时）。"""
        if not hasattr(self, "listbox"):
            return
        # 取消待执行的节流刷新
        self._refresh_scheduled = False
        self._do_refresh_now()

    def _do_refresh_now(self):
        """立即重建列表内容。"""
        # 记住当前选中位置
        selected_indices = self.listbox.curselection()

        self.listbox.delete(0, tk.END)

        new_items = []
        for i, obj in enumerate(self._tracked_objects):
            track_id = obj["track_id"]
            label = obj["label"]
            x, y, z = obj["position"]
            dx, dy, dz = obj.get("size", (0, 0, 0))
            display = f"#{track_id} {label} ({dx:.2f}x{dy:.2f}x{dz:.2f}) @ ({x:.2f}, {y:.2f}, {z:.2f})"
            self.listbox.insert(tk.END, display)

            # 高亮当前选中的目标
            if track_id == self._selected_track_id:
                self.listbox.itemconfig(i, {"fg": "blue", "selectforeground": "blue"})

            new_items.append(display)

        self._last_display_items = new_items

        # 恢复选中状态
        if selected_indices:
            for idx in selected_indices:
                if idx < self.listbox.size():
                    self.listbox.selection_set(idx)

        self._update_status()

    def _refresh_list(self):
        """刷新目标列表"""
        if not hasattr(self, "listbox"):
            return
        self._schedule_refresh()

    def _do_scheduled_refresh(self):
        """定时刷新回调：只在内容实际变化时才重建列表。"""
        self._refresh_scheduled = False

        # 生成新的显示文本列表用于对比
        new_items = []
        for obj in self._tracked_objects:
            track_id = obj["track_id"]
            label = obj["label"]
            x, y, z = obj["position"]
            dx, dy, dz = obj.get("size", (0, 0, 0))
            new_items.append(
                f"#{track_id} {label} ({dx:.2f}x{dy:.2f}x{dz:.2f}) @ ({x:.2f}, {y:.2f}, {z:.2f})"
            )

        # 内容没有变化时跳过重建，保留当前选中状态
        if new_items == self._last_display_items:
            return

        self._do_refresh_now()

    def _update_status(self):
        """更新状态显示"""
        if self._selected_track_id is not None:
            for obj in self._tracked_objects:
                if obj["track_id"] == self._selected_track_id:
                    self.status_label.config(
                        text=f"#{obj['track_id']} {obj['label']}",
                        foreground="green",
                    )
                    return
            self.status_label.config(
                text=f"[{self._selected_track_id}]",
                foreground="orange",
            )
        else:
            self.status_label.config(text="无", foreground="red")

    def _on_item_double_click(self, event):
        """列表项双击事件"""
        selection = self.listbox.curselection()
        if not selection:
            return

        index = selection[0]
        if index < len(self._tracked_objects):
            obj = self._tracked_objects[index]
            if obj["track_id"] is not None:
                self._select_target(obj["track_id"])

    def _on_follow_click(self):
        """点击开始跟随按钮"""
        selection = self.listbox.curselection()
        if not selection:
            return

        index = selection[0]
        if index < len(self._tracked_objects):
            obj = self._tracked_objects[index]
            if obj["track_id"] is not None:
                self._select_target(obj["track_id"])

    def _on_cancel_click(self):
        """点击取消跟随按钮"""
        self._select_target(-1)

    def _select_target(self, track_id: int):
        """选中目标"""
        if track_id > 0:
            self._selected_track_id = track_id
        else:
            self._selected_track_id = None

        # 用户操作需要即时反馈，使用强制刷新
        self._force_refresh()

        if self._on_select:
            self._on_select(track_id)

    def _on_close(self):
        """窗口关闭事件"""
        self._running = False

    @property
    def running(self) -> bool:
        """检查是否在运行"""
        return self._running

    def update(self):
        """更新 GUI（处理事件）"""
        self.root.update()

    def destroy(self):
        """销毁窗口"""
        self.root.destroy()
