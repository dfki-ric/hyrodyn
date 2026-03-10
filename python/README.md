## Python Bindings

HyRoDyn provides optional Python bindings that can be enabled during the CMake build.  
It is recommended to create a Python virtual environment first and then build the bindings against that environment.

---
## Installation
> [!IMPORTANT]
> The Python virtual environment must be created before building the Python bindings so that CMake can detect the correct Python interpreter.

1. [Install uv](https://docs.astral.sh/uv/getting-started/installation/), a faster alternative to `pip`
2. From the root of the repository, create a virtual environment: `uv venv --python 3.8`
3. Activate it: `source .venv/bin/activate`
4. Build HyRoDyn with Python bindings enabled:
```bash
mkdir build && cd build
cmake .. -DBUILD_PYTHON_BINDINGS=ON
make -j
```
5. Verify hyrodyn: `python -c "import hyrodyn"`
6. Navigate to this folder `cd python`
7. Install dependencies with: `uv pip install -e .`

---

## Quick Test

Run the tutorial script:

```bash
python scripts/tutorial_hyrodyn.py
```

This will load a robot model and demonstrate the basic functionality of the Python API.

### Play with HyRoDyn

```bash
python scripts/play_hyrodyn.py
```

## Demo

The following video demonstrates interactive robot visualization using the Python bindings and Viser.

<video src="../docs/slider_gui_hyrodyn.mp4" controls width="800"></video>
---
