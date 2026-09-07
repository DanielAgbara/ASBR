"""Validate local Markdown links in the proposed Git file set."""
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote

root = Path(__file__).resolve().parents[1]
files = subprocess.check_output(['git', 'ls-files', '--cached', '--others', '--exclude-standard'], cwd=root, text=True).splitlines()
failures = []
for name in set(files):
    p = root / name
    if p.suffix != '.md':
        continue
    for target in re.findall(r'\]\(([^)]+)\)', p.read_text(encoding='utf-8')):
        if target.startswith(('http:', 'https:', 'mailto:', '#')):
            continue
        target = unquote(target.split('#')[0])
        if not (p.parent / target).exists():
            failures.append(f'{name}: {target}')
if failures:
    raise SystemExit('\n'.join(failures))
print('All local Markdown link targets exist.')
