# apply_patches.py
from pathlib import Path
import subprocess, sys, os
from SCons.Script import Import

# Import the construction environment provided by PlatformIO.  Note that
# Import() returns None; it *injects* the requested variables into the local
# namespace.  Therefore we must NOT assign its return value.
Import("env")

# ----------------------------------------------------------------------
# 1. Locate the Adafruit-nRF52 framework directory in every situation
# ----------------------------------------------------------------------
try:
    # Works when PlatformIO has already attached the platform object
    platform_obj = env.PioPlatform()
except AttributeError:
    platform_obj = None

# If we still don't have a platform object we can build the framework path
# manually from PlatformIO's packages directory.  This avoids relying on
# internal (and version-specific) PlatformIO APIs such as PlatformFactory.
if platform_obj is not None:
    fw_dir = Path(platform_obj.get_package_dir("framework-arduinoadafruitnrf52"))
else:
    # $PIOPACKAGES_DIR always exists at this point (after package install).
    packages_dir = Path(env.subst("$PIOPACKAGES_DIR"))
    fw_dir = packages_dir / "framework-arduinoadafruitnrf52"

if not fw_dir.exists():
    sys.stderr.write("Cannot locate Adafruit-nRF52 framework folder\n")
    env.Exit(1)

print("Framework folder:", fw_dir)

# ----------------------------------------------------------------------
# 2. Apply your patches
# ----------------------------------------------------------------------
project_dir = Path(env["PROJECT_DIR"])
patches = [
    (project_dir / "firmware/patches/blehid_generic.patch", fw_dir),
]

for patch_file, cwd in patches:
    if not patch_file.exists():
        sys.stderr.write(f"Patch {patch_file} not found\n")
        env.Exit(1)

    print(f"Applying {patch_file.name} ...")
    # Use --batch to avoid interactive prompts if patch cannot find a file.
    cmd = [
      "patch", "--batch",
      "-p1",      # only removes leading a/ or b/
      "-N", "-r", "-",
      "-d", str(cwd),
      "-i", str(patch_file),
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)

    # return codes: 0 = applied, 1 = already applied, >1 = error
    if result.returncode not in (0, 1):
        print(result.stdout)
        sys.stderr.write(result.stderr)
        env.Exit(result.returncode)