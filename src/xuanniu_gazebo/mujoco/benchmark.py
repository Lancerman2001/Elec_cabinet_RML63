#!/usr/bin/env python3
"""Measure physical real-time factor and command/feedback latency in both launch modes.
Usage: python mujoco/benchmark.py docs/performance/after.json
"""
import json,sys,time,tempfile,subprocess,statistics
from pathlib import Path
root=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(root/'mujoco'))
from transport import RemoteConnection
reports=[]
for headless in (True,False):
 with tempfile.TemporaryDirectory() as folder:
  ready=Path(folder)/'ready.json'
  cmd=[sys.executable,str(root/'mujoco/xuanniu_mujoco.py'),'--port','0','--ready-file',str(ready)]
  if headless:cmd+=['--headless']
  p=subprocess.Popen(cmd,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True)
  remote=None
  try:
   deadline=time.monotonic()+15
   while not ready.exists() and time.monotonic()<deadline:time.sleep(.02)
   remote=RemoteConnection(port=json.loads(ready.read_text())['port'])
   deadline=time.monotonic()+20
   while time.monotonic()<deadline:
    st,received=remote.feedback()
    if st is not None and time.monotonic()-received<.1:break
    time.sleep(.05)
   time.sleep(2)
   t0=time.monotonic();s0=remote.feedback()[0].time
   commands={};latency=[];samples=[];enqueue=[];prev_received=0;n=0
   while time.monotonic()-t0<5:
    now=time.monotonic()
    if n<150 and now-t0>=n/30:
     angle=round((n+1)*.002,6);commands[angle]=now;remote.send(angle,True);enqueue.append(time.monotonic()-now);n+=1
    state,received=remote.feedback()
    if received!=prev_received:
     samples.append(received);prev_received=received
     sent=commands.pop(round(state.target_angle_rad,6),None)
     if sent is not None:latency.append(now-sent)
    time.sleep(.002)
   elapsed=time.monotonic()-t0;sim_elapsed=remote.feedback()[0].time-s0
   percentile=lambda values,p:sorted(values)[min(len(values)-1,int((len(values)-1)*p))]
   report={'headless':headless,'wall_seconds':elapsed,'sim_seconds':sim_elapsed,'realtime_factor':sim_elapsed/elapsed,'feedback_hz':len(samples)/elapsed,'command_ack_ms_p50':percentile(latency,.5)*1000,'command_ack_ms_p95':percentile(latency,.95)*1000,'max_feedback_gap_ms':max(b-a for a,b in zip(samples,samples[1:]))*1000,'commands_observed':len(latency),'max_client_enqueue_ms':max(enqueue)*1000}
   reports.append(report);print(report,flush=True);Path(sys.argv[1]).write_text(json.dumps(reports,indent=2)+'\n')
  finally:
   if remote:remote.close()
   p.terminate()
   try:_,err=p.communicate(timeout=8)
   except subprocess.TimeoutExpired:p.kill();_,err=p.communicate()
   if err:print(err)
Path(sys.argv[1]).write_text(json.dumps(reports,indent=2)+'\n')
