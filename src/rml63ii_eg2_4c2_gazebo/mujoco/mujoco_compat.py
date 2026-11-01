"""MuJoCo 2.3.7 compatibility, without modifying the installed SDK.

Model conversion preserves PD forces and textured mesh topology. Native helpers
are compiled against the very same wheel that supplies the Python bindings.
"""
from __future__ import annotations

import ctypes
import fcntl
import hashlib
import os
from pathlib import Path
import subprocess
import threading
import xml.etree.ElementTree as ET


def _configure_render_environment(environ=None, kernel_release=None):
    """Apply the launcher's explicit policy before importing any GL backend.

    Direct Python entrypoints also need the WSL software default.  Only this
    process environment is changed; explicit hardware remains available.
    """
    environ = os.environ if environ is None else environ
    mode = environ.get("ARM_MUJOCO_RENDERER", "auto")
    if mode not in ("auto", "software", "hardware"):
        raise ValueError("ARM_MUJOCO_RENDERER must be auto, software, or hardware")
    if kernel_release is None:
        try:
            kernel_release = Path("/proc/sys/kernel/osrelease").read_text()
        except OSError:
            kernel_release = ""
    is_wsl = bool(environ.get("WSL_DISTRO_NAME") or environ.get("WSL_INTEROP")
                  or "microsoft" in kernel_release.lower())
    if mode == "software" or (mode == "auto" and is_wsl):
        environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
        environ["GALLIUM_DRIVER"] = "llvmpipe"
        environ.setdefault("LP_NUM_THREADS", "2")
    elif mode == "hardware":
        # Match render_env.sh: the public hardware switch overrides inherited
        # software flags. No hardware context is opened by this function.
        environ.pop("LIBGL_ALWAYS_SOFTWARE", None)
        environ.pop("GALLIUM_DRIVER", None)


_configure_render_environment()

import mujoco as _mj
import numpy as np

_native = None


def __getattr__(name):
    return getattr(_mj, name)


def equality_active(model, data):
    return data.eq_active if hasattr(data, "eq_active") else model.eq_active


class Renderer(_mj.Renderer):
    """Provide deterministic cleanup also for the 2.3.7 renderer."""

    def close(self):
        if hasattr(_mj.Renderer, "close"):
            return super().close()
        context = getattr(self, "_gl_context", None)
        if context is not None:
            context.make_current()
            self._mjr_context.free()
            context.free()
            self._gl_context = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def _library():
    global _native
    if _native is not None:
        return _native
    wheel = Path(_mj.__file__).resolve().parent
    source = Path(__file__).with_name("mujoco_compat.c")
    signature = hashlib.sha256(source.read_bytes() + str(wheel).encode() + _mj.__version__.encode()).hexdigest()[:16]
    cache = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "arm_sim" / signature
    cache.mkdir(parents=True, exist_ok=True)
    output = cache / "libcabinet_mujoco_compat.so"
    with (cache / "build.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if not output.exists():
            library = wheel / ("libmujoco.so." + _mj.__version__)
            temporary = output.with_suffix(".tmp.so")
            subprocess.run(["cc", "-shared", "-fPIC", "-O2", "-I", str(wheel / "include"),
                            str(source), str(library), "-Wl,-rpath," + str(wheel), "-o", str(temporary)], check=True)
            temporary.replace(output)
    native = ctypes.CDLL(str(output))
    native.cabinet_version.restype = ctypes.c_int
    if native.cabinet_version() != _mj.mj_version():
        raise RuntimeError("MuJoCo compatibility helper ABI does not match the Python wheel")
    native.cabinet_distance.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int,
                                      ctypes.c_int, ctypes.c_double, ctypes.c_void_p]
    native.cabinet_distance.restype = ctypes.c_double
    native.cabinet_copy_data.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
    native.cabinet_copy_data.restype = None
    _native = native
    return native


def mj_geomDistance(model, data, first, second, distmax, fromto=None):
    if hasattr(_mj, "mj_geomDistance"):
        return _mj.mj_geomDistance(model, data, first, second, distmax, fromto)
    if not (0 <= first < model.ngeom and 0 <= second < model.ngeom):
        raise IndexError("geometry index out of range")
    buffer = np.zeros(6, dtype=np.float64)
    result = _library().cabinet_distance(model._address, data._address, int(first), int(second),
                                         float(distmax), buffer.ctypes.data)
    if not np.isfinite(result):
        raise ValueError("Unsupported geometry pair in distance query")
    if fromto is not None:
        fromto[:] = buffer
    return result


def mj_copyData(destination, model, source):
    if hasattr(_mj, "mj_copyData"):
        return _mj.mj_copyData(destination, model, source)
    _library().cabinet_copy_data(destination._address, model._address, source._address)


def normalize_tree(root, directory):
    """Normalize an independent XML tree and return virtual OBJ assets."""
    directory = Path(directory)
    assets = {}
    compiler = root.find('compiler')
    assetdir = compiler.attrib.pop('assetdir', '') if compiler is not None else ''
    meshdir = compiler.attrib.pop('meshdir', assetdir) if compiler is not None else ''
    texturedir = compiler.attrib.pop('texturedir', assetdir) if compiler is not None else ''
    for element in root.iter():
        if element.get("file"):
            relative = meshdir if element.tag == 'mesh' else texturedir if element.tag == 'texture' else ''
            element.set("file", str((directory / relative / element.get("file")).resolve()))
        if element.tag == "mesh" and element.get("vertex") and element.get("texcoord"):
            vertices = element.attrib.pop("vertex").split()
            uv = element.attrib.pop("texcoord").split()
            faces = list(map(int, element.attrib.pop("face").split()))
            if len(vertices) // 3 != len(uv) // 2:
                raise ValueError("Expected one texture coordinate per inline mesh vertex")
            lines = ["v " + " ".join(vertices[i:i+3]) for i in range(0, len(vertices), 3)]
            # MuJoCo flips OBJ V coordinates on import, unlike inline MJCF
            # texcoord. Undo that here so both paths load the same UV mapping.
            lines += [f"vt {uv[i]} {1.0 - float(uv[i+1]):.15g}"
                      for i in range(0, len(uv), 2)]
            lines += ["f " + " ".join(f"{j+1}/{j+1}" for j in faces[i:i+3])
                      for i in range(0, len(faces), 3)]
            filename = "cabinet_compat_" + element.get("name") + ".obj"
            assets[filename] = ("\n".join(lines) + "\n").encode()
            element.set("file", filename)
        if element.tag == "camera":
            element.attrib.pop("resolution", None)
        for newer, older in (("actuatorfrcrange", "actuatorforcerange"),
                              ("actuatorfrclimited", "actuatorforcelimited")):
            if newer in element.attrib:
                element.set(older, element.attrib.pop(newer))
        if element.tag == "position" and "kv" in element.attrib:
            kp = float(element.attrib.pop("kp", "1"))
            kv = float(element.attrib.pop("kv"))
            element.tag = "general"
            element.set("gaintype", "fixed")
            element.set("gainprm", f"{kp:.15g}")
            element.set("biastype", "affine")
            element.set("biasprm", f"0 {-kp:.15g} {-kv:.15g}")
    return assets


def load_model(path):
    path = Path(path).resolve()
    root = ET.parse(path).getroot()
    if root.tag == 'robot':
        # URDF resolves stripped mesh basenames beside the input file. Loading
        # it as a string loses that directory and breaks model regeneration.
        return _mj.MjModel.from_xml_path(str(path))
    def expand(tree, filename, parents):
        assets = normalize_tree(tree, filename.parent)
        for parent in list(tree.iter()):
            for child in list(parent):
                if child.tag != 'include':
                    continue
                included = Path(child.get('file')).resolve()
                if included in parents:
                    raise ValueError(f'Circular MJCF include: {included}')
                subtree = ET.parse(included).getroot()
                assets.update(expand(subtree, included, parents | {included}))
                index = list(parent).index(child)
                parent.remove(child)
                for node in reversed(list(subtree)):
                    parent.insert(index, node)
        return assets
    assets = expand(root, path, {path})
    return _mj.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"), assets)


def write_compatible_model(path):
    """Generator output: portable XML plus equivalent OBJ assets."""
    path = Path(path).resolve()
    root = ET.parse(path).getroot()
    if root.tag != "mujoco":
        return
    assets = normalize_tree(root, path.parent)
    destination = path.parent / "compat_meshes"
    for name, content in assets.items():
        destination.mkdir(exist_ok=True)
        (destination / name).write_bytes(content)
    for element in root.iter():
        filename = element.get("file")
        if filename:
            resource = destination / filename if filename in assets else Path(filename)
            element.set("file", os.path.relpath(resource, path.parent))
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def sync_viewer(viewer):
    viewer.sync()


class PassiveViewer:
    """Stop sync as soon as close is requested, not after native destruction.

    MuJoCo 2.3.7 Handle.is_running() remains true for exitrequest == 1;
    its Sync() does not guard against a freed scene state during shutdown.
    Serialize application close/sync and observe the native exit request.
    """
    def __init__(self, handle):
        self._handle = handle
        self._closed = False
        self._lifecycle_lock = threading.RLock()

    def __getattr__(self, name):
        return getattr(self._handle, name)

    def is_running(self):
        if self._closed:
            return False
        getter = getattr(self._handle, '_get_sim', None) or getattr(self._handle, '_sim', None)
        native = getter() if getter else None
        if native is not None and getattr(native, 'exitrequest', 0) != 0:
            return False
        return self._handle.is_running()

    def sync(self):
        with self._lifecycle_lock:
            if self.is_running():
                self._handle.sync()

    def close(self):
        with self._lifecycle_lock:
            self._closed = True
            self._handle.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def launch_passive(model, data, **kwargs):
    import inspect
    from mujoco import viewer
    supported = inspect.signature(viewer.launch_passive).parameters
    return PassiveViewer(viewer.launch_passive(model, data, **{k: v for k, v in kwargs.items() if k in supported}))
