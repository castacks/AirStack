"""Run a model-serving module under the interpreter that can actually import it.

`ros2 run` executes console_scripts with SYSTEM python. That interpreter has
torch and transformers, but NOT fastapi, uvicorn, bitsandbytes or accelerate —
`Dockerfile.robot` installs those into `/opt/lvlm-venv` alone, deliberately, so
the serving stack cannot drag a numpy 2 or a shadowed cv2 into the ROS
environment (see the dependency-trap table in that Dockerfile).

The consequence is that `ros2 run search_baselines vlm_server` dies on import
while `/opt/lvlm-venv/bin/python -m search_baselines.vlm_server` works — two
invocations for the same server, and the wrong one fails in a way that reads
like a broken package rather than a wrong interpreter.

So the entry point hands off: if the imports resolve here, run in-process; if
they do not and the venv exists, re-exec there. Either invocation works, and
`ros2 run` behaves the way a ROS user expects.
"""

import importlib
import os
import sys

VENV_PYTHON = '/opt/lvlm-venv/bin/python'


def run(module_name, required):
    """Run `module_name`'s main(), re-execing into the venv if needed.

    `required` is what the module cannot start without. Checked by import name
    rather than by pip metadata, because what matters is whether THIS
    interpreter can import it.
    """
    missing = []
    for mod in required:
        try:
            importlib.import_module(mod)
        except ImportError:
            missing.append(mod)

    if not missing:
        importlib.import_module(module_name).main()
        return

    if os.path.exists(VENV_PYTHON) and os.environ.get('_SEARCH_BASELINES_REEXEC') != '1':
        # Guard against a loop if the venv is ALSO missing something: the child
        # sees the flag set and reports the real error instead of re-execing
        # forever.
        env = dict(os.environ, _SEARCH_BASELINES_REEXEC='1')
        sys.stderr.write(
            f'[search_baselines] {module_name} needs {", ".join(missing)}, which '
            f'this interpreter lacks; re-execing under {VENV_PYTHON}\n')
        os.execve(VENV_PYTHON,
                  [VENV_PYTHON, '-m', module_name] + sys.argv[1:], env)

    raise SystemExit(
        f'[search_baselines] cannot run {module_name}: missing {", ".join(missing)}.\n'
        f'  Those live in {VENV_PYTHON}, which is '
        f'{"present but already tried" if os.path.exists(VENV_PYTHON) else "NOT present"}.\n'
        f'  Run directly with: {VENV_PYTHON} -m {module_name}')


def vlm_server():
    run('search_baselines.vlm_server',
        ['fastapi', 'uvicorn', 'transformers', 'bitsandbytes', 'accelerate'])


def itm_server():
    run('search_baselines.itm_server', ['fastapi', 'uvicorn', 'transformers'])
