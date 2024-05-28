# Copyright 2014-present PlatformIO <contact@platformio.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
from platform import system
from os import makedirs
from os.path import isdir, join
from pathlib import Path

from platformio.project.config import ProjectConfig

from SCons.Script import (
    ARGUMENTS,
    COMMAND_LINE_TARGETS,
    AlwaysBuild,
    Builder,
    Default,
    DefaultEnvironment,
)

env = DefaultEnvironment()
platform = env.PioPlatform()

platform_version  = str(int(''.join(['{0:0=3d}'.format(int(x)) for x in platform.version.split('.')]))) + 'UL'
#framework_version = str(int(''.join(['{0:0=3d}'.format(int(x)) for x in platform.get_package_version('framework-arduino-mcui').split('.')]))) + 'UL'

if "BOARD" not in env:
    raise(Exception("No board in configuration"))

board = env.BoardConfig()
target_mcu = board.get("build.mcu").upper()

env.Replace(
    AR="arm-none-eabi-ar",
    AS="arm-none-eabi-as",
    CC="arm-none-eabi-gcc",
    CXX="arm-none-eabi-g++",
    GDB="arm-none-eabi-gdb",
    OBJCOPY="arm-none-eabi-objcopy",
    RANLIB="arm-none-eabi-ranlib",
    SIZETOOL="arm-none-eabi-size",

    ARFLAGS=[],

    ASFLAGS=[],
    CCFLAGS=[],
    CXXFLAGS=[],
    CPPDEFINES=[("F_CPU", "$BOARD_F_CPU")],
    LINKFLAGS=[],
    LIBS=[],

    SIZEPROGREGEXP=r"^(?:\.vectors|\.text|\.ramcode|\.data|\.rodata|\.text.align|\.ARM.exidx)\s+(\d+).*",
    SIZEDATAREGEXP=r"^(?:\.vectors|\.ramcode|\.data|\.bss|\.noinit)\s+(\d+).*",
    SIZECHECKCMD="$SIZETOOL -A -d $SOURCES",
    SIZEPRINTCMD='$SIZETOOL -B -d $SOURCES',

    PROGSUFFIX=".elf"
)

# Allow user to override via pre:script
if env.get("PROGNAME", "program") == "program":
    env.Replace(PROGNAME="firmware")

env.Append(
    CCFLAGS=[
        "-mcpu=%s" % board.get("build.cpu")
    ],
    LINKFLAGS=[
        "-mcpu=%s" % board.get("build.cpu")
    ]
)

env.Append(
    ASFLAGS=env.get("CCFLAGS", [])[:],

    BUILDERS=dict(
        ElfToBin=Builder(
            action=env.VerboseAction(" ".join([
                "$OBJCOPY",
                "-O",
                "binary",
                "$SOURCES",
                "$TARGET"
            ]), "Building $TARGET"),
            suffix=".bin"
        ),
        ElfToHex=Builder(
            action=env.VerboseAction(" ".join([
                "$OBJCOPY",
                "-O",
                "ihex",
                "-R",
                ".eeprom",
                "$SOURCES",
                "$TARGET"
            ]), "Building $TARGET"),
            suffix=".hex"
        )
    )
)

env.Append(
    ARFLAGS=["rc"],
    ASFLAGS=["-x", "assembler-with-cpp"],

    CFLAGS=[
        "-std=gnu11",
        #"-flto"
    ],

    CCFLAGS=[
        "-Os",  # optimize for size
        "-Wall",  # show warnings
        "-ffunction-sections",  # place each function in its own section
        "-fdata-sections",
        "-fsingle-precision-constant",
        "-mthumb",
        "-fno-fat-lto-objects"
    ],

    CXXFLAGS=[
        "-std=gnu++17",
        "-fno-rtti",
        "-fno-exceptions",
        "-fno-use-cxa-atexit",
        "-fno-common",
        "-fno-threadsafe-statics",
        #"-flto"
    ],

    LINKFLAGS=[
        "-Wl,--as-needed,--gc-sections,--relax",
        #"-flto",
        "-Wall",  # show warnings
        "-ffunction-sections",  # place each function in its own section
        "-fdata-sections",
        "-fsingle-precision-constant",
        "-mthumb",
        "--specs=nano.specs",
        "--specs=nosys.specs"
        #"-u_printf_float"
    ],

    LIBS=[]
)

mcu_interface_name = {
        "lpc1768" : "LPC176x5x",
        "lpc1769" : "LPC176x5x",
        "lpc4078" : "LPC408x7x",
    }[board.get("build.mcu")]

env.Append(
    CPPDEFINES= [mcu_interface_name]
)

platform_path = Path(platform.manifest_path).parents[0]

is_bootloader = False
device_link_script = join(platform_path, "mcu_interface", "src", "device", mcu_interface_name, "build", "gcc_arm.ld")

try:
    if board.get("build.is_bootloader") == "yes":
        board.update("upload.maximum_size", "16384")
        board.update("upload.offset_address", "0")
        device_link_script = join(platform_path, "mcu_interface", "src", "device", mcu_interface_name, "build", "gcc_arm_bootloader.ld")
        is_bootloader = True
        env.Append(
            CPPDEFINES= ["LPC_USB_STACK_ENABLED=Disabled"]
        )
except:
  pass

if env.get("LDSCRIPT_PATH") == None:
    env.Replace(LDSCRIPT_PATH=device_link_script)

#to pull library configs from the project include folder it needs added here
env.Append(CPPPATH=[env['PROJECT_INCLUDE_DIR']])

env.Append(CPPPATH=[join(platform_path, "mcu_interface", "src")])
env.Append(CPPPATH=[join(platform_path, "mcu_interface", "src", "device", mcu_interface_name)])

env.Append(CPPPATH=[join(platform_path, "external_libs", "lpc_usb")])
env.Append(CPPPATH=[join(platform_path, "external_libs", "chanff-R0.15")])

env.BuildSources(join("$BUILD_DIR", "PlatformMCUI", mcu_interface_name), join(join(platform_path, "mcu_interface", "src", "device", mcu_interface_name)))
env.BuildSources(join("$BUILD_DIR", "PlatformMCUI", "driver"), join(join(platform_path, "mcu_interface", "src", "driver")))

env.BuildSources(join("$BUILD_DIR", "PlatformMCUI", "external", "lpc_usb"), join(join(platform_path, "external_libs", "lpc_usb", "usb")))
env.BuildSources(join("$BUILD_DIR", "PlatformMCUI", "external", "chanfs"), join(join(platform_path, "external_libs", "chanff-R0.15", "chanfs")))

# This should work but has issues losing symbols because of the -Wl, --start-group?, even though there is a 'strong' symbol as well
libs = []
# libs.append(env.BuildLibrary( join("$BUILD_DIR", "libmcuinterface"), join(platform_path, "mcu_interface", "src")))
# libs.append(env.BuildLibrary( join("$BUILD_DIR", "libchanff"), join(platform_path, "external_libs", "chanff-R0.15", "source")))
# libs.append(env.BuildLibrary( join("$BUILD_DIR", "libtinyusb"), join(platform_path, "external_libs", "tinyusb-0.15.0", "src")))
env.Append(LIBS=libs)

#
# Target: Build executable and linkable firmware
#

target_elf = None
if "nobuild" in COMMAND_LINE_TARGETS:
    target_firm = join("$BUILD_DIR", "${PROGNAME}.bin")
else:
    target_elf = env.BuildProgram()
    target_firm = env.ElfToBin(join("$BUILD_DIR", "${PROGNAME}"), target_elf)

AlwaysBuild(env.Alias("nobuild", target_firm))
target_buildprog = env.Alias("buildprog", target_firm, target_firm)

#
# Target: Print binary size
#
target_size = env.Alias(
    "size", target_elf,
    env.VerboseAction("$SIZEPRINTCMD", "Calculating size $SOURCE"))
AlwaysBuild(target_size)

#
# Target: Upload by default .bin file
#

upload_protocol = env.subst("$UPLOAD_PROTOCOL")
debug_server = board.get("debug.tools", {}).get(
    upload_protocol, {}).get("server")
upload_actions = []

if upload_protocol == "mbed":
    upload_actions = [
        env.VerboseAction(env.AutodetectUploadPort, "Looking for upload disk..."),
        env.VerboseAction(env.UploadToDisk, "Uploading $SOURCE")
    ]

elif upload_protocol.startswith("jlink"):
    def _jlink_cmd_script(env, source):
        build_dir = env.subst("$BUILD_DIR")
        if not isdir(build_dir):
            makedirs(build_dir)
        script_path = join(build_dir, "upload.jlink")
        commands = ["h", "loadbin %s,%s" % (source, hex(int(board.get("upload.offset_address", "0x0")))), "r", "q"]
        with open(script_path, "w") as fp:
            fp.write("\n".join(commands))
        return script_path

    env.Replace(
        __jlink_cmd_script=_jlink_cmd_script,
        UPLOADER="JLink.exe" if system() == "Windows" else "JLinkExe",
        UPLOADERFLAGS=[
            "-device", board.get("debug", {}).get("jlink_device"),
            "-speed", "4000",
            "-if", ("jtag" if upload_protocol == "jlink-jtag" else "swd"),
            "-autoconnect", "1"
        ],
        UPLOADCMD="$UPLOADER $UPLOADERFLAGS -CommanderScript \"${__jlink_cmd_script(__env__, SOURCE)}\""
    )
    upload_actions = [env.VerboseAction("$UPLOADCMD", "Uploading $SOURCE")]

elif upload_protocol.startswith("blackmagic"):
    env.Replace(
        UPLOADER="$GDB",
        UPLOADERFLAGS=[
            "-nx",
            "--batch",
            "-ex", "target extended-remote $UPLOAD_PORT",
            "-ex", "monitor %s_scan" %
            ("jtag" if upload_protocol == "blackmagic-jtag" else "swdp"),
            "-ex", "attach 1",
            "-ex", "load",
            "-ex", "compare-sections",
            "-ex", "kill"
        ],
        UPLOADCMD="$UPLOADER $UPLOADERFLAGS $BUILD_DIR/${PROGNAME}.elf"
    )
    upload_actions = [
        env.VerboseAction(env.AutodetectUploadPort, "Looking for BlackMagic port..."),
        env.VerboseAction("$UPLOADCMD", "Uploading $SOURCE")
    ]

elif debug_server and debug_server.get("package") == "tool-pyocd":
    env.Replace(
        UPLOADER=join(platform.get_package_dir("tool-pyocd") or "",
                      "pyocd-flashtool.py"),
        UPLOADERFLAGS=debug_server.get("arguments", [])[1:],
        UPLOADCMD='"$PYTHONEXE" "$UPLOADER" $UPLOADERFLAGS $SOURCE'
    )
    upload_actions = [
        env.VerboseAction("$UPLOADCMD", "Uploading $SOURCE")
    ]

# custom upload tool
elif "UPLOADCMD" in env:
    upload_actions = [env.VerboseAction("$UPLOADCMD", "Uploading $SOURCE")]

else:
    sys.stderr.write("Warning! Unknown upload protocol %s\n" % upload_protocol)

AlwaysBuild(env.Alias("upload", target_firm, upload_actions))

#
# Default targets
#

Default([target_buildprog, target_size])
