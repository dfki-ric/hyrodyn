# Python Bindings

HyRoDyn provides optional Python bindings that can be enabled during the CMake build.  
It is recommended to create a Python virtual environment first and then build the bindings against that environment.
---
## Installation
> [!IMPORTANT]
> Requires Python 3.10 or later.

1.. [Install uv](https://docs.astral.sh/uv/getting-started/installation/), a faster alternative to `pip`
2. Create a virtual environment: `uv venv --python 3.12`
3. Activate it: `source .venv/bin/activate`
5. Verify hyrodyn: `python -c "import hyrodyn"`
6. Install dependencies with: `uv pip install -e .`

---

## Quick Test

Run the tutorial script:

```bash
python scripts/tutorial_hyrodyn.py
```

This will load a robot model and demonstrate the basic functionality of the Python API.

---
