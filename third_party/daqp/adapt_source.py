"""Apply the documented Cortex-M4F adaptations to an upstream DAQP copy.

Copy include/, src/ and LICENSE from the pinned commit into this directory first.
Keep src/HelmDaqpMath.h and run this script once. Repeated runs are idempotent.
"""

from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent
FLOAT_TOKEN = re.compile(
    r'/\*[\s\S]*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|'
    r'(?<![A-Za-z_0-9.])(?:(?:\d+\.\d*|\.\d+)(?:[eE][+-]?\d+)?|'
    r'\d+[eE][+-]?\d+)[fFlL]?'
)


def float_literal(match):
    token = match[0]
    if token[0] in '/\"\'' or token[-1] in 'fFlL':
        return token
    return token + 'f'


for path in [*(ROOT / 'src').glob('*.c'), *(ROOT / 'include').glob('*.h')]:
    source = path.read_text(encoding='utf-8')
    if path.suffix == '.c' and '#include "HelmDaqpMath.h"' not in source:
        source = source.replace(
            '#include <math.h>', '#include <math.h>\n#include "HelmDaqpMath.h"', 1
        )
    source = FLOAT_TOKEN.sub(float_literal, source)
    path.write_text(source, encoding='utf-8', newline='\n')
