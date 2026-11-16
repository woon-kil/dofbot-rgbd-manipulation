import pty,os,time,select,json,sys
pid,fd=pty.fork()
if pid==0:
    os.chdir('/Users/woonkil/projects/dofbot-rgbd-manipulation'); os.execv('/Users/woonkil/projects/dofbot-rgbd-manipulation/.venv/bin/python',['python','-I','/tmp/pty_child.py'])
out=b''; t0=time.time(); sent=[False,False,False]
while True:
    r,_,_=select.select([fd],[],[],0.5)
    if r:
        try: d=os.read(fd,4096)
        except OSError: break
        if not d: break
        out+=d
    el=time.time()-t0
    if el>2 and not sent[0]: os.write(fd,b'14\n'); sent[0]=True        # 55초 전 첫 입력(R16 2404와 같은 상황)
    if el>6 and not sent[1]: os.write(fd,b'14\n'); sent[1]=True        # 정지 중 두 번째 입력(버퍼에 남을 입력)
    if b'\xec\xa2\x85\xed\x95\xa9 \xed\x99\x95\xec\x9d\xb8 \xec\xa7\x88\xeb\xac\xb8?' in out and not sent[2]:   # 종합 확인 질문?
        time.sleep(0.5); os.write(fd,b'y\n'); sent[2]=True
    if el>120: break
time.sleep(1)
print('결과',open('/tmp/pty_result.json').read() if os.path.exists('/tmp/pty_result.json') else '결과 없음', '| 출력에 안내문:', '다시 입력하지 마세요' in out.decode('utf-8','ignore'))
