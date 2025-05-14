Import("env")
from os.path import join

platform = env.PioPlatform()
FRAMEWORK_DIR = platform.get_package_dir("framework-arduinoadafruitnrf52")
env.Append(
    LIBPATH=[join(FRAMEWORK_DIR, "libraries", "Adafruit_nRFCrypto", "src", "cortex-m4", "fpv4-sp-d16-hard")]
)