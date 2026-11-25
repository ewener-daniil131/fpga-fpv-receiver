import os, termios, time, re, sys, numpy as np
SP = os.path.dirname(os.path.abspath(__file__))
SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 14.0
FS = 20e6
fd = os.open('/dev/ttyUSB1', os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
a = termios.tcgetattr(fd); a[0]=a[1]=a[2]=a[3]=0
a[4]=a[5]=termios.B115200; a[2] |= (termios.CS8|termios.CLOCAL|termios.CREAD)
termios.tcsetattr(fd, termios.TCSANOW, a); termios.tcflush(fd, termios.TCIOFLUSH)
t=time.time(); buf=b''
while time.time()-t < SECS:
    try:
        d=os.read(fd,65536)
        if d: buf+=d
    except BlockingIOError: time.sleep(0.004)
os.close(fd)
open(SP+'/last.txt','wb').write(buf)
txt=buf.decode('ascii','replace')
rs=[int(x,16) for x in re.findall(r'RS=([0-9A-F]{4})',txt)]
lk=re.findall(r'LK=(\d)',txt)
print('%.0f s, %d kB, %d dumps' % (SECS, len(buf)//1000, txt.count('IQDUMP')))
if rs: print('RSSI min=%d avg=%.0f max=%d   LK=1: %d/%d' % (min(rs),sum(rs)/len(rs),max(rs),lk.count('1'),len(lk)))
best=None
for bi,b in enumerate(txt.split('IQDUMP')[1:]):
    tk=re.findall(r'\b([0-9A-F]{4})\b', b)
    if len(tk)<20000: continue
    w=np.array([int(x,16) for x in tk[:32768]],dtype=np.uint16)
    I=((w>>8)&0xFF).astype(np.int8).astype(float); Q=(w&0xFF).astype(np.int8).astype(float)
    z=I+1j*Q; amp=np.abs(z).mean()
    clip=((np.abs(I)>=127)|(np.abs(Q)>=127)).mean()
    f=np.angle(z[1:]*np.conj(z[:-1]))/(2*np.pi)*FS
    sm=np.convolve(f,np.ones(16)/16,mode='valid'); x=sm-sm.mean()
    ac=np.correlate(x,x,'full')[len(x)-1:]; ac/=ac[0]
    lo,hi=int(55e-6*FS),int(75e-6*FS); j=lo+int(np.argmax(ac[lo:hi]))
    print('  block %2d |z|=%5.1f clip=%5.2f%% lag=%.2fus r=%.3f'%(bi,amp,clip*100,j/FS*1e6,ac[j]))
    if best is None or ac[j]>best[0]: best=(ac[j],f,amp,clip)
if best and best[0]>0.5:
    r,f,amp,clip=best
    v=np.convolve(f,np.ones(3)/3,mode='same')
    thr=np.percentile(v,8); below=v<thr
    e=np.where((~below[:-1])&(below[1:]))[0]
    keep=[e[0]]
    for x_ in e[1:]:
        if x_-keep[-1]>1271*0.7: keep.append(x_)
    A=int(9.5e-6*FS); B=int(62.0e-6*FS)
    rows=[v[k+A:k+B][:int(52.5e-6*FS)] for k in keep if k+B<len(v)]
    L=min(len(r_) for r_ in rows); img=np.array([r_[:L] for r_ in rows])
    b_,w_=np.percentile(img,[5,97]); im=np.clip((img-b_)/(w_-b_)*255,0,255)
    cs=np.nanmean([np.corrcoef(im[i],im[i+1])[0,1] for i in range(len(im)-1)])
    det=np.mean([np.std(np.diff(r_)) for r_ in im])
    d=np.diff(keep)
    print('\nVIDEO  lines=%d  line-corr=%.3f  horiz-detail=%.2f  intervals within +-8 of 1271: %d/%d'
          % (len(im), cs, det, (np.abs(d-1271)<=8).sum(), len(d)))
    np.save(SP+'/last_img.npy', im.astype(np.uint8))
    ramp=' .:-=+*#%@'
    for r_ in im[:22,::8]: print('  '+''.join(ramp[int(np.clip(p/255*9,0,9))] for p in r_))
else:
    print('\nNO VIDEO in this capture (carrier absent or too weak)')
