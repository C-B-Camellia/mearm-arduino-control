"""Build an isolated synthetic fully-enabled Uno image. NEVER upload this image."""
from pathlib import Path
import shutil
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
work = root / ".pio" / "synthetic-capacity-check"
work.mkdir(parents=True, exist_ok=True)
for name in ("src", "include"):
    shutil.copytree(root / name, work / name, dirs_exist_ok=True)
ini = (root / "platformio.ini").read_text(encoding="utf-8-sig")
ini += "\n    -DMEARM_BUTTON_RX=2\n    -DMEARM_BUTTON_TX=3\n    -DMEARM_BUTTON_BAUD=9600\n"
(work / "platformio.ini").write_text(ini, encoding="utf-8")
shutil.copytree(root / ".pio" / "libdeps" / "uno", work / ".pio" / "libdeps" / "uno", dirs_exist_ok=True)
(work / "DO_NOT_UPLOAD.txt").write_text("SYNTHETIC TEST MEASUREMENTS ONLY. DO NOT UPLOAD.\n", encoding="utf-8")
config = work / "src" / "hardware_config.cpp"
text = config.read_text(encoding="utf-8")
text = text.replace("HardwareConfig hardwareConfig{};", """HardwareConfig hardwareConfig={
    {true,10,170,90}, {true,20,100,50},
    {true,ToolModel::DistalLink,1,90,5,30,3,2,4},
    true,80,30,true,{{90,120,95,50}},2,
    {{{90,115,100,50}},{{90,120,95,50}}}
};""")
config.write_text(text, encoding="utf-8")
pick = work / "src" / "pick_place.cpp"
text = pick.read_text(encoding="utf-8")
program = """{true, {{{90,120,95,80}},{{90,115,95,80}},{{90,115,95,30}},
{{90,120,95,30}},{{110,120,95,30}},{{110,115,95,30}},
{{110,115,95,80}},{{110,120,95,80}}}, {100,100,200,100,100,100,200,100}}"""
text = text.replace("const PickProgram programs[3] PROGMEM={};", "const PickProgram programs[3] PROGMEM={" + ",".join([program]*3) + "};")
pick.write_text(text, encoding="utf-8")
print("CAPACITY TEST ONLY: all measured data and UART pins are synthetic.", flush=True)
sys.exit(subprocess.call(["pio", "run", "--project-dir", str(work), "-e", "uno"]))
