"""Compare captured native Servo.write streams and every loaded segment plan."""
from pathlib import Path
import re
import hashlib
root=Path(__file__).resolve().parents[2]
evidence=root/'docs/fast70_evidence'
def read(name):
    data=(evidence/name).read_bytes()
    return data.decode('utf-16' if data.startswith(b'\xff\xfe') else 'utf-8-sig').splitlines()
a=read('fast35-output.log'); control=read('fast35-control.log'); b=read('fast70-output.log')
for prefix in ('OUT ', 'KNOT ', 'HOLD ', 'METRIC '):
    assert [s for s in a if s.startswith(prefix)]==[s for s in control if s.startswith(prefix)],prefix
for kind in (0,1):
    assert [s for s in a if s.startswith(f'OUT {kind} ')]==[s for s in b if s.startswith(f'OUT {kind} ')]
assert [s for s in a if s.startswith(('KNOT ','HOLD '))]==[s for s in b if s.startswith(('KNOT ','HOLD '))]
x=read('plans35.log'); y=read('plans70.log')
nonstroke=lambda rows:[s for s in rows if s.startswith('PLAN ') and 'stroke=0 ' in s]
assert nonstroke(x)==nonstroke(y)
for kind in (0,1):
    assert [s for s in x if s.startswith(f'PLAN kind={kind} ')]==[s for s in y if s.startswith(f'PLAN kind={kind} ')]
# Non-stroke output profile times are relative to each stage; total task clocks differ.
for kind in (2,3):
    total=max(int(re.search(r'index=(\d+)',s).group(1)) for s in x if s.startswith(f'PLAN kind={kind} '))
    def profiles(rows):
        result={}
        for s in rows:
            if not s.startswith(f'OUT {kind} '):continue
            f=s.split();time,index=int(f[2]),int(f[3])
            if 4<=index<total-2:continue
            result.setdefault(index,[]).append((time,tuple(f[4:])))
        return {key:[(time-values[0][0],pose) for time,pose in values] for key,values in result.items()}
    assert profiles(a)==profiles(b),kind
out=['Original committed Fast35 matches override35: integer output, knots, holds and metrics.',
     'LINE/SHAPE: integer output streams with timestamps and all segment plans identical.',
     f'Non-stroke: {len(nonstroke(x))} identical segment plans; five-point integer output stage profiles identical.',
     'Five mandatory integer knot targets identical; POLY 3x80ms holds, CURVE no holds.']
for name in ('fast35-output.log','fast70-output.log'):
    out += [name]+[s for s in read(name) if s.startswith(('METRIC','INTERRUPT','PASS'))]
for name in ('plans35.log','plans70.log'):
    for kind in (2,3):
        times=[int(re.search(r' ms=(\d+)',s).group(1)) for s in read(name) if s.startswith(f'PLAN kind={kind} ') and 'stroke=1 ' in s]
        out.append(f'{name} kind={kind} all stroke segment ms sum={sum(times)} nodes={len(times)}')
p=root/'.pio/build/uno_nano_five_fast/firmware.hex'
out.append('Fast70 HEX SHA256 '+hashlib.sha256(p.read_bytes()).hexdigest())
(evidence/'comparison.txt').write_text('\n'.join(out)+'\n',encoding='utf-8')
print('\n'.join(out))
