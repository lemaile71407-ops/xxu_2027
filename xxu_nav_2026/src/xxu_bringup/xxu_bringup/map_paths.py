"""Resolve writable map paths."""
import os
from pathlib import Path
import warnings


def default_map_save_path(bringup_share=None):
    """Resolve the writable map prefix independently of the launch cwd."""
    from ament_index_python.packages import get_package_share_directory
    configured = os.environ.get("XXU_MAP_DIR")
    if configured:
        folder = Path(configured).expanduser()
        if not folder.is_absolute():
            raise ValueError("XXU_MAP_DIR must be an absolute directory")
        return str(folder / "auto_map")
    share = Path(bringup_share or get_package_share_directory("xxu_bringup"))
    anchor = share / "maps" / "auto_map.yaml"
    # A symlink installation points at writable source maps. A copied install
    # is a deploy artifact; never choose it as the automatic write destination.
    if anchor.is_symlink() and anchor.exists():
        folder = anchor.resolve().parent
        if os.access(folder, os.W_OK):
            return str(folder / "auto_map")
    fallback = Path.home() / ".ros" / "xxu_maps" / "auto_map"
    warnings.warn(f"No writable source map link; saving maps to {fallback}", stacklevel=2)
    return str(fallback)
