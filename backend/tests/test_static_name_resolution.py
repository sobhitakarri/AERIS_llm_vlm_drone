"""Catches names that only fail when a rarely-taken branch runs.

Hardware paths (LiteWing takeoff/land) are not exercised by unit tests, so a
missing import there stays invisible until the drone is in the air. This walks
every backend module's AST and checks each global name it loads actually
resolves, which is what the interpreter would do at that line.
"""
import ast
import builtins
import importlib
import pkgutil
from pathlib import Path

import pytest

BACKEND = Path(__file__).resolve().parents[1]

SKIP = {"backend.tests", "backend.drone.matlab_interface"}


def _module_names() -> list[str]:
    mods = []
    for info in pkgutil.walk_packages([str(BACKEND)], prefix="backend."):
        if any(info.name.startswith(s) for s in SKIP):
            continue
        mods.append(info.name)
    return sorted(mods)


def _unresolved(module) -> list[str]:
    src = Path(module.__file__).read_text(encoding="utf-8-sig")
    tree = ast.parse(src)

    # Names bound locally (params, assignments, comprehensions, handlers) are
    # not globals, so collect them per-scope and exclude them.
    local: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            local.add(node.name)  # nested defs are visible to their siblings
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            args = node.args
            for a in [*args.posonlyargs, *args.args, *args.kwonlyargs]:
                local.add(a.arg)
            if args.vararg:
                local.add(args.vararg.arg)
            if args.kwarg:
                local.add(args.kwarg.arg)
        elif isinstance(node, ast.Name) and isinstance(node.ctx, (ast.Store, ast.Del)):
            local.add(node.id)
        elif isinstance(node, ast.ExceptHandler) and node.name:
            local.add(node.name)
        elif isinstance(node, ast.Lambda):
            for a in [*node.args.posonlyargs, *node.args.args, *node.args.kwonlyargs]:
                local.add(a.arg)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            for alias in node.names:
                local.add((alias.asname or alias.name).split(".")[0])
        elif isinstance(node, ast.Global):
            local.update(node.names)

    loaded = {
        n.id
        for n in ast.walk(tree)
        if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)
    }
    return sorted(
        name
        for name in loaded - local
        if not hasattr(module, name) and not hasattr(builtins, name)
    )


@pytest.mark.parametrize("module_name", _module_names())
def test_module_global_names_resolve(module_name):
    try:
        module = importlib.import_module(module_name)
    except Exception as exc:  # optional third-party backend not installed
        pytest.skip(f"{module_name} not importable: {exc}")
    if not getattr(module, "__file__", None):
        pytest.skip("namespace package")
    missing = _unresolved(module)
    assert not missing, f"{module_name} references undefined name(s): {missing}"
