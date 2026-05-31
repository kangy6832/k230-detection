# Project Guide

This workspace is a small K230 camera and photo-capture project. Treat the repository as a skeleton until source files are added.

Start with [memory-bank/demand.md](memory-bank/demand.md) and [memory-bank/rules.md](memory-bank/rules.md). Use [memory-bank/progress.md](memory-bank/progress.md), [memory-bank/questions.md](memory-bank/questions.md), and [memory-bank/summary.md](memory-bank/summary.md) to track work as you go.

Keep changes minimal and local. Do not delete files or modify anything you do not understand.

Preserve the existing layout. Store captured images in [photos/](photos/). If application code is added, keep the entry point close to the workspace root unless a package layout is clearly needed.

Prefer clear, small Python modules. If you touch Python entry or launch scripts, run `python3 -m py_compile` on the edited files before finishing.

If requirements are unclear, record the question in [memory-bank/questions.md](memory-bank/questions.md) instead of guessing.

When you complete a task, update [memory-bank/progress.md](memory-bank/progress.md) and add a short recap to [memory-bank/summary.md](memory-bank/summary.md).

No build or test commands are defined yet in this workspace. Discover them from project files before inventing new ones.