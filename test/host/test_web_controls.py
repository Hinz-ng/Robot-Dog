"""Compile both actual embedded pages and check their JavaScript and button routing."""
import importlib.util
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def main():
    runner = ROOT / "test/host/run.py"
    spec = importlib.util.spec_from_file_location("host_compiler", runner)
    module = importlib.util.module_from_spec(spec)
    exec(compile(runner.read_text(encoding="utf-8"), str(runner), "exec"), module.__dict__)
    cache = ROOT / ".pio/web-controls-test"
    cache.mkdir(parents=True, exist_ok=True)
    source = cache / "dump.cpp"
    source.write_text('#include <cstdio>\n#define PROGMEM\n#include "' +
                      (ROOT / 'master/src/web_page.h').as_posix() +
                      '"\nint main(){std::fputs(WEB_PAGE,stdout);}\n', encoding="utf-8")
    env = os.environ.copy()
    env["TEMP"] = env["TMP"] = str(cache)
    env.setdefault("ZIG_GLOBAL_CACHE_DIR", str(cache / "zig-cache"))
    for profile in ("normal", "leg"):
        exe = cache / (profile + (".exe" if os.name == "nt" else ""))
        command = module.compiler() + ["-std=c++11", "-I", str(ROOT / "test/host/mock")]
        if profile == "leg":
            command += ["-DM_LEG_BENCH=1"]
        command += [str(source), "-o", str(exe)]
        result = subprocess.run(command, env=env, capture_output=True, text=True)
        if result.returncode:
            print(result.stderr, file=sys.stderr)
            return result.returncode
        html = subprocess.check_output([str(exe)]).decode("utf-8")
        page, js = cache / (profile + ".html"), cache / (profile + ".js")
        page.write_text(html, encoding="utf-8")
        js.write_text(re.search(r"<script>([\s\S]*?)</script>", html)[1], encoding="utf-8")
        subprocess.run(["node", "--check", str(js)], check=True)
        subprocess.run(["node", str(ROOT / "test/host/test_web_controls.js"), str(page), profile], check=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
