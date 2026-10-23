#!/usr/bin/env python3
"""Read-only export of likely EGO planner FSM source for P2 debugging.

No ROS launch. Scans source directories only (skips build/devel).
Does not change or delete any source/log files.
"""
from pathlib import Path
import os
import zipfile
from datetime import datetime

root=Path.home()/'contest1_uav'/'runtime'
out=Path.home()/'Desktop'/('demo11_p2_fsm_sources_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.zip')
search_roots=[root/'Fast-Drone-250',root/'ego_uav_ws'/'src',root/'sunray_ws'/'src']
needs=('ego_replan_fsm','replan_fsm','ego_planner_fsm','traj_server','planner_manager')
tokens=(b'MANUAL FULL-GOAL',b'MANUAL goal accepted',b'GEN_NEW_TRAJ',b'EXEC_TRAJ',b'PREVIEW REQUEST')
selected=[]
for base in search_roots:
    if not base.exists():
        continue
    for directory, subdirs, files in os.walk(base):
        subdirs[:]=[x for x in subdirs if x not in ('.git','build','devel','install','log','.cache','node_modules')]
        for name in files:
            p=Path(directory)/name
            if p.suffix.lower() not in ('.cpp','.cc','.hpp','.h'):
                continue
            try:
                if p.is_symlink() or p.stat().st_size>1000000:
                    continue
                if any(n in name.lower() for n in needs):
                    selected.append(p)
                    continue
                data=p.read_bytes()
                if any(t in data for t in tokens):
                    selected.append(p)
            except (OSError,PermissionError):
                continue
selected=sorted(set(selected))[:100]
with zipfile.ZipFile(out,'w',compression=zipfile.ZIP_DEFLATED) as z:
    z.writestr('README.txt','Read-only relevant planner source. No ROS master or flight command was run.\n')
    for p in selected:
        try:
            z.write(p,arcname=str(p.relative_to(root)))
        except (OSError,ValueError):
            pass
print('[FOUND]',len(selected),'candidate C++ source files')
print('[OUTPUT]',out)
if not selected:
    print('[NOTE] No FSM source was found under the standard runtime source directories. A compiled/installed planner may not include its original source.')
