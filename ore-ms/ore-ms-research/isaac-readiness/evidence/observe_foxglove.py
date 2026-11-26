import socket,json,struct,os,time,collections
s=socket.create_connection(('127.0.0.1',8766),8)
s.sendall(b'GET / HTTP/1.1\r\nHost: localhost:8766\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Protocol: foxglove.sdk.v1\r\n\r\n')
f=s.makefile('rb'); status=f.readline().decode().strip()
while f.readline().strip(): pass
topics={}; stats={}; start=time.monotonic()
while time.monotonic()-start<12:
 try:
  a,b=f.read(2); n=b&127
 except (TimeoutError,ValueError): break
 if n>=126: n=int.from_bytes(f.read(2 if n==126 else 8),'big')
 d=f.read(n)
 if a&15==1:
  m=json.loads(d)
  if m['op']!='advertise': continue
  subs=[]
  for ch in m['channels']:
   t=ch['topic']
   if t in ('/clock','/tf','/tf_static','/gcs/sim_ground','/diagnostics','/robot_1/vdb_mapping/vdb_map_visualization') or t.endswith(('/odometry','/mavros/global_position/global','/left/image_rect','/left/camera_info','/right/depth_ground_truth','/mavros/state','/ouster/point_cloud_raw')):
    i=len(topics)+1; topics[i]=t; subs.append(dict(id=i,channelId=ch['id']))
  if not subs: continue
  p=json.dumps(dict(op='subscribe',subscriptions=subs)).encode(); k=os.urandom(4)
  h=bytes([129,128|len(p)]) if len(p)<126 else b'\x81\xfe'+struct.pack('>H',len(p))
  s.sendall(h+k+bytes(x^k[i%4] for i,x in enumerate(p)))
 elif a&15==2 and d and d[0]==1:
  t=topics.get(struct.unpack_from('<I',d,1)[0])
  if t:
   now=time.monotonic()
   row=stats.setdefault(t,dict(count=0,payload_bytes=0,max_gap_s=0))
   if row['count']: row['max_gap_s']=max(row['max_gap_s'],now-row['last_receipt'])
   row['last_receipt']=now
   row['count']+=1
   row['payload_bytes']+=len(d)
elapsed=time.monotonic()-start
for row in stats.values(): row['observed_hz']=row['count']/elapsed
print(json.dumps(dict(handshake=status,elapsed_s=elapsed,subscribed=list(topics.values()),streams=stats),indent=2))
