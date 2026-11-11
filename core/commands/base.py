"""
Command Pattern Engine (`core/commands/base.py`).
Provides infinite Undo/Redo history with command coalescing (merge_with) for smooth gizmo dragging.
"""

from __future__ import annotations

import time
import uuid
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional
from core.schema.scene_graph import SceneGraphDocument


class Command(ABC):
    """Abstract base class for all Scene Graph mutations."""

    def __init__(self, label: str, coalesce_key: Optional[str] = None) -> None:
        self.id: str = f"cmd_{uuid.uuid4().hex[:8]}"
        self.label: str = label
        self.timestamp: float = time.time()
        self.coalesce_key: Optional[str] = coalesce_key

    @abstractmethod
    def execute(self, doc: SceneGraphDocument) -> None:
        """Apply mutation to the SceneGraphDocument."""

    @abstractmethod
    def undo(self, doc: SceneGraphDocument) -> None:
        """Revert mutation on the SceneGraphDocument."""

    def merge_with(self, newer: "Command") -> bool:
        """
        Optionally merge a consecutive command into `self` (e.g., while dragging a gizmo)
        so Undo reverts the entire drag stroke at once.
        """
        return False

    def to_summary(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "label": self.label,
            "timestamp": self.timestamp,
        }


class CommandManager:
    """Manages execution, coalescing, and infinite Undo/Redo stacks on a SceneGraphDocument."""

    def __init__(self, doc: Optional[SceneGraphDocument] = None, max_history: int = 500) -> None:
        self.doc: SceneGraphDocument = doc or SceneGraphDocument.create_default()
        self.undo_stack: List[Command] = []
        self.redo_stack: List[Command] = []
        self.max_history: int = max_history

    def execute(self, command: Command, allow_coalesce: bool = True) -> None:
        command.execute(self.doc)
        if allow_coalesce and self.undo_stack:
            top = self.undo_stack[-1]
            if (
                top.coalesce_key
                and top.coalesce_key == command.coalesce_key
                and (command.timestamp - top.timestamp) < 1.25
                and top.merge_with(command)
            ):
                top.timestamp = command.timestamp
                self.redo_stack.clear()
                return

        self.undo_stack.append(command)
        if len(self.undo_stack) > self.max_history:
            self.undo_stack.pop(0)
        self.redo_stack.clear()

    def can_undo(self) -> bool:
        return len(self.undo_stack) > 0

    def can_redo(self) -> bool:
        return len(self.redo_stack) > 0

    def undo(self) -> Optional[Command]:
        if not self.undo_stack:
            return None
        cmd = self.undo_stack.pop()
        cmd.undo(self.doc)
        self.redo_stack.append(cmd)
        return cmd

    def redo(self) -> Optional[Command]:
        if not self.redo_stack:
            return None
        cmd = self.redo_stack.pop()
        cmd.execute(self.doc)
        self.undo_stack.append(cmd)
        return cmd

    def clear_history(self) -> None:
        self.undo_stack.clear()
        self.redo_stack.clear()

    def get_history_state(self) -> Dict[str, Any]:
        return {
            "can_undo": self.can_undo(),
            "can_redo": self.can_redo(),
            "undo_count": len(self.undo_stack),
            "redo_count": len(self.redo_stack),
            "last_action": self.undo_stack[-1].label if self.undo_stack else None,
            "next_redo": self.redo_stack[-1].label if self.redo_stack else None,
        }

