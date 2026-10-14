"""Desktop-only historical executor with explicit scenario timing overrides."""
from pathlib import Path
import subprocess, os
root=Path(__file__).resolve().parents[2]
work=root/'.pio/minimal-legacy';work.mkdir(parents=True,exist_ok=True)
for folder in ['include','src']:
    (work/folder).mkdir(exist_ok=True)
    files=subprocess.check_output(['git','ls-tree','--name-only','20335f2',folder+'/'],cwd=root,text=True).splitlines()
    for name in files:
        (work/name).write_bytes(subprocess.check_output(['git','show','20335f2:'+name],cwd=root))
env=dict(os.environ);env['PATH']='C:/msys64/ucrt64/bin;'+env['PATH']
original=(work/'include/config.h').read_text()
for step,delay in [(5,10),(10,20)]:
    (work/'include/config.h').write_text(original.replace('LINE_STEP_MM = 5.0f','LINE_STEP_MM = '+str(step)+'.0f').replace('WAYPOINT_DELAY_MS = 1000','WAYPOINT_DELAY_MS = '+str(delay)))
    exe=work/'legacy.exe'
    subprocess.run(['C:/msys64/ucrt64/bin/g++.exe','-std=c++11','-O2','-I'+str(root/'test/native'),'-I'+str(work/'include'),'-I'+str(work/'src'),str(root/'test/minimal_architecture/legacy_trace.cpp'),str(work/'src/arm.cpp'),str(work/'src/kinematics.cpp'),'-o',str(exe)],check=True,env=env)
    print('SCENARIO step=',step,'delay=',delay,flush=True)
    for args in [[],['cost5']]: subprocess.run([str(exe),*args],check=True,env=env)
