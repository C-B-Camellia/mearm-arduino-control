"""Build an isolated synthetic fully-enabled Uno image. NEVER upload this image."""
from pathlib import Path
import shutil
import subprocess
import sys
import argparse
import hashlib
import json

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--label',help='Archive ELF/Map/size evidence below .pio/resource-optimization')
args=parser.parse_args()
if args.label and not all(c.isalnum() or c in '-_' for c in args.label): parser.error('invalid label')

root = Path(__file__).resolve().parents[1]
work = root / ".pio" / "synthetic-capacity-check"
work.mkdir(parents=True, exist_ok=True)
for name in ("src", "include"):
    shutil.copytree(root / name, work / name, dirs_exist_ok=True)
ini = (root / "platformio.ini").read_text(encoding="utf-8-sig")
ini += "\n    -DMEARM_BUTTON_RX=2\n    -DMEARM_BUTTON_TX=3\n    -DMEARM_BUTTON_BAUD=9600\n"
ini += '    -Wl,-Map,'+(work/'firmware.map').as_posix()+'\n'
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
pio=Path(sys.executable).with_name('pio.exe')
result=subprocess.run([str(pio) if pio.exists() else 'pio', "run", "--project-dir", str(work), "-e", "uno"],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace')
print(result.stdout,flush=True)
if result.returncode: sys.exit(result.returncode)
if args.label:
    evidence=root/'.pio/resource-optimization'/args.label
    evidence.mkdir(parents=True,exist_ok=True)
    elf=work/'.pio/build/uno/firmware.elf'
    shutil.copy2(elf,evidence/'firmware.elf')
    shutil.copy2(work/'firmware.map',evidence/'firmware.map')
    (evidence/'build.log').write_text(result.stdout,encoding='utf-8')
    avr=Path.home()/'.platformio/packages/toolchain-atmelavr/bin'
    for executable,options,name in [('avr-size.exe',['-A'],'sections.txt'),('avr-nm.exe',['--print-size','--size-sort','--radix=d','-C'],'symbols.txt'),('avr-objdump.exe',['-d','-C'],'disassembly.txt')]:
        output=subprocess.check_output([str(avr/executable),*options,str(elf)],text=True,encoding='utf-8')
        (evidence/name).write_text(output,encoding='utf-8')
    metadata={'head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),'elf_sha256':hashlib.sha256(elf.read_bytes()).hexdigest(),'synthetic_only':True,
        'source_sha256':{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for folder in ['src','include'] for p in (root/folder).glob('*') if p.is_file()}}
    (evidence/'metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
