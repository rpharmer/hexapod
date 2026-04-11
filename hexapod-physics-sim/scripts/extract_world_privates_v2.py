#!/usr/bin/env python3
"""
Out-line World private member bodies from world.hpp into src/core/world_private_methods.cpp.

Keeps nested aggregates (struct/class/enum) in the header unchanged; converts member
function definitions to declarations + cpp definitions (including World::ManifoldManager::Process).
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "include" / "minphys3d" / "core" / "world.hpp"
OUT_CPP = ROOT / "src" / "core" / "world_private_methods.cpp"

# Inclusive 1-based: first private inline through ClearAccumulators closing brace.
FIRST = 254
LAST = 3061


def strip_line_comment(line: str) -> str:
    in_s = False
    in_c = False
    out = []
    i = 0
    while i < len(line):
        c = line[i]
        nxt = line[i + 1] if i + 1 < len(line) else ""
        if not in_s and not in_c and c == "/" and nxt == "/":
            break
        if not in_s and c == '"':
            in_s = not in_s
        out.append(c)
        i += 1
    return "".join(out).rstrip()


def is_declaration_line(line: str) -> bool:
    s = strip_line_comment(line).strip()
    if not s or s.startswith("#"):
        return False
    if s.endswith(");"):
        return True
    return False


def brace_match_in_text(s: str, open_brace_index: int) -> int:
    """Return index of matching `}` for `{` at open_brace_index; string/char/comment aware."""
    i = open_brace_index
    depth = 0
    state = "code"  # code, line_comment, block_comment, str, chr
    while i < len(s):
        c = s[i]
        nxt = s[i + 1] if i + 1 < len(s) else ""

        if state == "code":
            if c == "/" and nxt == "/":
                state = "line_comment"
                i += 2
                continue
            if c == "/" and nxt == "*":
                state = "block_comment"
                i += 2
                continue
            if c == '"':
                state = "str"
                i += 1
                continue
            if c == "'":
                state = "chr"
                i += 1
                continue
            if c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0:
                    return i
            i += 1
            continue

        if state == "line_comment":
            if c == "\n":
                state = "code"
            i += 1
            continue

        if state == "block_comment":
            if c == "*" and nxt == "/":
                state = "code"
                i += 2
                continue
            i += 1
            continue

        if state == "str":
            if c == "\\":
                i += 2
                continue
            if c == '"':
                state = "code"
            i += 1
            continue

        if state == "chr":
            if c == "\\":
                i += 2
                continue
            if c == "'":
                state = "code"
            i += 1
            continue

    raise RuntimeError("unbalanced braces in extracted fragment")


def skip_cpp_directive_block(lines: list[str], start: int) -> int:
    j = start
    while j < len(lines) and lines[j].lstrip().startswith("#"):
        j += 1
    return j


def looks_member_fn_head(line: str) -> bool:
    if not line.startswith("    ") or line.startswith("        "):
        return False
    if line.lstrip().startswith("#"):
        return False
    s = line.strip()
    banned = (
        "if ",
        "if(",
        "for ",
        "for(",
        "while ",
        "while(",
        "else",
        "switch ",
        "case ",
        "default:",
        "return ",
        "break;",
        "continue;",
        "struct ",
        "class ",
        "enum ",
        "union ",
        "namespace ",
        "static_assert",
        "friend ",
        "#",
    )
    for b in banned:
        if s.startswith(b):
            return False
    if "(" not in line:
        return False
    return True


def accumulate_until_body_brace(lines: list[str], start: int) -> tuple[int, str] | None:
    """
    From lines[start], accumulate lines until a line ends the parameter list with '{'.
    Returns (end_line_index_inclusive_for_signature_part, full_signature_text_including_brace_line)
    or None if this is not a function (e.g. hits declaration ');' first).
    """
    parts: list[str] = []
    j = start
    while j < len(lines):
        ln = lines[j]
        if j > start and ln.startswith("    ") and not ln.startswith("        ") and looks_member_fn_head(ln):
            # new top-level member started before we found '{'
            return None
        if is_declaration_line(ln) and j == start:
            return None
        if j == start and strip_line_comment(ln).strip().endswith(");"):
            return None
        parts.append(ln)
        joined = "".join(parts)
        last_line = strip_line_comment(ln).rstrip()
        if re.search(r"\)\s*(const\s*)?(noexcept\s*)?\{\s*$", last_line):
            st = ln.strip()
            if st.startswith(("if ", "if(", "for ", "for(", "while ", "while(", "switch ", "else")):
                j += 1
                continue
            return j, joined
        j += 1
        if j >= len(lines):
            return None
    return None


def qualify_world_method(sig_core: str) -> str:
    """sig_core: from return type through closing ')' optional ' const', no '{'."""
    sig_core = sig_core.strip()
    if sig_core.startswith("static "):
        sig_core = sig_core[7:].lstrip()
    open_paren = sig_core.find("(")
    if open_paren < 0:
        raise ValueError(sig_core)
    pre = sig_core[:open_paren].rstrip()
    post = sig_core[open_paren:]
    tokens = pre.split()
    if len(tokens) < 2:
        raise ValueError(pre)
    ret = " ".join(tokens[:-1])
    name = tokens[-1]
    return f"{ret} World::{name}{post}"


def sig_to_declaration(sig_head: str) -> str:
    """sig_head: signature through the line containing opening '{'."""
    lines = sig_head.splitlines(keepends=True)
    if not lines:
        return ""
    last = lines[-1]
    ob = last.rfind("{")
    if ob < 0:
        raise ValueError("expected { in signature head")
    lines[-1] = last[:ob].rstrip() + ";\n"
    return "".join(lines)


def main() -> int:
    all_lines = HEADER.read_text(encoding="utf-8").splitlines(keepends=True)
    slice_lo = FIRST - 1
    slice_hi = LAST
    chunk = all_lines[slice_lo:slice_hi]

    header_out: list[str] = []
    cpp_out: list[str] = [
        '#include "minphys3d/core/world.hpp"\n',
        "\n",
        "namespace minphys3d {\n",
        "\n",
    ]

    i = 0
    while i < len(chunk):
        line = chunk[i]

        if line.lstrip().startswith("#"):
            block_end = skip_cpp_directive_block(chunk, i)
            for k in range(i, block_end):
                header_out.append(chunk[k])
            i = block_end
            continue

        if is_declaration_line(line) or (line.strip() == ""):
            header_out.append(line)
            i += 1
            continue

        if not looks_member_fn_head(line) and not line.strip().startswith("void Process("):
            # e.g. continuation-only line shouldn't happen at column 4
            header_out.append(line)
            i += 1
            continue

        acc = accumulate_until_body_brace(chunk, i)
        if acc is None:
            header_out.append(line)
            i += 1
            continue
        j, sig_head = acc
        ob = sig_head.rfind("{")
        sig_core = sig_head[:ob].rstrip()
        body_open = ob
        joined = sig_head
        k = j + 1
        while True:
            try:
                close_idx = brace_match_in_text(joined, body_open)
                break
            except RuntimeError:
                if k >= len(chunk):
                    print(
                        f"brace_match failed at slice line {i + 1} (abs {i + FIRST}), partial len={len(joined)}",
                        file=sys.stderr,
                    )
                    print(joined[:500], file=sys.stderr)
                    raise RuntimeError("unbalanced braces (end of chunk)")
                joined += chunk[k]
                k += 1
        body = joined[body_open + 1 : close_idx]

        # ManifoldManager::Process (nested; 8-space indent)
        if chunk[i].strip().startswith("void Process("):
            m = re.search(r"void\s+Process\s*\((.*)\)\s*const", sig_core, re.S)
            if not m:
                raise RuntimeError(f"bad Process sig: {sig_core!r}")
            cpp_out.append(f"void World::ManifoldManager::Process({m.group(1)}) const {{\n")
            cpp_out.append(body)
            cpp_out.append("}\n\n")
            header_out.append("        void Process(Manifold& manifold, const Manifold* previous) const;\n")
            i = k
            continue

        # ManageManifoldContacts and similar one-liners / short — still handled here
        try:
            qual = qualify_world_method(sig_core)
        except Exception as e:
            raise RuntimeError(f"at slice line {i + FIRST}: {e}\n{sig_core!r}") from e

        cpp_out.append(qual + " {\n")
        cpp_out.append(body)
        cpp_out.append("}\n\n")

        # declaration: re-indent to 4 spaces from original sig_core lines
        decl = sig_to_declaration(sig_head)
        header_out.append(decl)

        i = k

    cpp_out.append("} // namespace minphys3d\n")

    OUT_CPP.write_text("".join(cpp_out), encoding="utf-8")

    new_file = all_lines[:slice_lo] + header_out + all_lines[slice_hi:]
    HEADER.write_text("".join(new_file), encoding="utf-8")

    print(f"Updated {HEADER}")
    print(f"Wrote {OUT_CPP}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
