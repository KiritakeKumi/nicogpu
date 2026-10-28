#!/usr/bin/env python3
"""run_trace_check.py -- model-vs-RTL trace comparison driver.

Pipeline per scenario:
  1. gen_scenarios.py writes <work>/scen/<name>/{scen.cfg,code.hex}
  2. sp_trace_gen (functional model) writes <scen>/model.trace
  3. tb_sp_sched_trace (xsim) writes <scen>/rtl.trace
  4. diff the two line by line

Requires: a prebuilt sp_trace_gen.exe (build_model_trace.sh, mingw g++) and
the Vivado xsim binaries.  Python >= 3.8 stdlib only.
"""
from __future__ import annotations

import argparse
import filecmp
import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]  # rtl/gpu/tb/model_trace -> repo root
RTL = REPO / "rtl" / "gpu"


def run(cmd, cwd=None, env=None):
    print("+ " + " ".join(str(c) for c in cmd))
    return subprocess.run([str(c) for c in cmd], cwd=str(cwd) if cwd else None,
                          env=env, capture_output=True, text=True)


def tool_env():
    """PATH extended with the mingw runtime DLLs (sp_trace_gen links them)."""
    env = dict(os.environ)
    mingw = Path("C:/msys64/mingw64/bin")
    if mingw.exists():
        env["PATH"] = str(mingw) + os.pathsep + env.get("PATH", "")
    return env


def per_warp(lines):
    """Group trace lines by slot (field 1), keeping each warp's own order."""
    d = {}
    for ln in lines:
        f = ln.split()
        if len(f) >= 2:
            d.setdefault(f[1], []).append(ln)
    return d


def warp_compare(al, bl):
    """ warp-selective issue changes the global commit interleaving, so the
    comparison is per warp.  Model trace `al`, RTL trace `bl`.  A warp that traps
    (T line) must match exactly; every other warp of a trapping scenario may stop
    early (RTL commit list a prefix of the model's, trap precision is per warp);
    without any trap every warp must match exactly.  Returns a list of messages."""
    am, bm = per_warp(al), per_warp(bl)
    trapped = any(ln.startswith("T ") for ln in al)
    msgs = []
    for slot in sorted(set(am) | set(bm)):
        x, y = am.get(slot, []), bm.get(slot, [])
        has_t = any(ln.startswith("T ") for ln in x)
        if x == y:
            continue
        if trapped and not has_t and y == x[:len(y)]:
            continue
        if trapped and not has_t and x == y[:len(x)]:
            continue   # RTL ran this warp a little further than the model's cut at the trap
        for i in range(max(len(x), len(y))):
            xa = x[i] if i < len(x) else "<eof>"
            ya = y[i] if i < len(y) else "<eof>"
            if xa != ya:
                msgs.append("  slot %s line %d: model=%r rtl=%r" % (slot, i + 1, xa, ya))
                break
    return msgs


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default="C:/AMDDesignTools/2026.1/Vivado/bin")
    ap.add_argument("--work", default="build/gpu/tb")
    ap.add_argument("--scenarios", default=None,
                    help="comma-separated subset (default: all)")
    ap.add_argument("--gen", default=None,
                    help="prebuilt sp_trace_gen executable")
    args = ap.parse_args()

    work = Path(args.work).resolve()
    work.mkdir(parents=True, exist_ok=True)

    # 1. scenarios
    gen_cmd = [sys.executable, str(HERE / "gen_scenarios.py"), "--out-dir",
               str(work / "scen")]
    if args.scenarios:
        gen_cmd += ["--only", args.scenarios]
    r = run(gen_cmd)
    if r.returncode != 0:
        print(r.stdout, r.stderr)
        return 1
    names = sorted(p.name for p in (work / "scen").iterdir() if p.is_dir())
    if args.scenarios:
        names = [n for n in names if n in args.scenarios.split(",")]
    print("scenarios: %s" % ", ".join(names))

    # 2. model traces
    gen_exe = Path(args.gen) if args.gen else work / "model_trace_gen" / "sp_trace_gen.exe"
    if not gen_exe.exists():
        # fall back to a sibling build location used by the run script
        alt = REPO / "build" / "gpu" / "modelrun" / "sp_trace_gen.exe"
        if alt.exists():
            gen_exe = alt
    if not gen_exe.exists():
        print("model runner not found (%s); build it with build_model_trace.sh"
              % gen_exe)
        return 1
    for n in names:
        r = run([gen_exe, str(work / "scen" / n)], env=tool_env())
        if r.returncode != 0:
            print("model run failed for %s (rc=%d):\n%s%s" % (n, r.returncode, r.stdout, r.stderr))
            return 1

    # 3. RTL traces: one xvlog/xelab per scenario (the scenario path is
    #    baked in through tb_sp_sched_trace_scen.svh because xsim.bat on
    #    Windows cannot forward valued plusargs).
    tb = RTL / "tb"
    tbsrc = tb / "tb_sp_sched_trace.sv"
    inc = work / "tb_sp_sched_trace_scen.svh"
    ok = True
    for n in names:
        scen_abs = (work / "scen" / n).resolve().as_posix()
        with open(inc, "w", encoding="ascii", newline="\n") as f:
            f.write('`define NG_TRACE_SCEN "%s"\n' % scen_abs)
        xvlog = [Path(args.bin) / "xvlog.bat", "-sv", "-d", "NG_TRACE_SCEN_FILE",
                 "-i", str(work), "-i", str(tb),
                 str(RTL / "sp_sched" / "ng_sp_sched_pkg.sv"),
                 str(RTL / "sp_sched" / "ng_sp_predec.sv"),
                 str(RTL / "sp_sched" / "ng_sp_sched.sv"),
                 str(RTL / "prim" / "ng_sp_prim_rr_arb.sv"),
                 str(tbsrc)]
        r = run(xvlog, cwd=work)
        if r.returncode != 0:
            print(r.stdout + r.stderr)
            return 1
        xelab = [Path(args.bin) / "xelab.bat", "-debug", "off",
                 "tb_sp_sched_trace", "-s", "tb_trace"]
        r = run(xelab, cwd=work)
        if r.returncode != 0:
            print(r.stdout + r.stderr)
            return 1
        # xsim interactive via stdin: "run -all; quit"
        xsim = Path(args.bin) / "xsim.bat"
        p = subprocess.run([str(xsim), "tb_trace"], cwd=str(work),
                           input="run -all\nquit\n", capture_output=True,
                           text=True)
        out = (work / ("xsim_trace_%s.out" % n))
        with open(out, "w", encoding="utf-8", newline="\n") as f:
            f.write(p.stdout + p.stderr)
        if "NG_SP_SCHED_TRACE_TB_PASS" not in p.stdout:
            print("RTL trace run FAILED for %s (see %s)" % (n, out))
            ok = False
            continue

        # 4. diff
        a = work / "scen" / n / "model.trace"
        b = work / "scen" / n / "rtl.trace"
        al = a.read_text().splitlines()
        bl = b.read_text().splitlines()
        if filecmp.cmp(str(a), str(b), shallow=False):
            print("TRACE MATCH %s (%d lines)" % (n, len(al)))
        else:
            msgs = warp_compare(al, bl)
            if not msgs:
                print("TRACE MATCH %s (%d lines, per-warp order)" % (n, len(al)))
            else:
                print("TRACE MISMATCH %s" % n)
                for m in msgs:
                    print(m)
                ok = False
    inc.unlink(missing_ok=True)
    print("TRACE_CHECK_%s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
