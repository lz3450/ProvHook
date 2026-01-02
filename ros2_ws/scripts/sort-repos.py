#!/usr/bin/env python
#
# sort-repos.py
#

import sys
import yaml
from pathlib import Path

if len(sys.argv) != 2:
    print("Usage: python sort_yaml.py <yaml_file>")
    sys.exit(1)

path = Path(sys.argv[1])
if not path.exists():
    print(f"Error: {path} not found.")
    sys.exit(1)

with path.open("r", encoding="utf-8") as f:
    data = yaml.safe_load(f)

with path.open("w", encoding="utf-8") as f:
    yaml.safe_dump(data, f, sort_keys=True, allow_unicode=True)
