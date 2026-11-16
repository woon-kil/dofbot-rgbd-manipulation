import sys,time,json
sys.path.insert(0,'scripts')
import g01_5b4_s42_test as T
class A: fake=False
plan={"set":"24","hold_poses":[2404],"hold_max_s":180,"gemini_photo":True,"hold_min_mm":{"2404":12}}
rec={}; T.hold_for_measure(plan,2404,A(),rec,None)
r=T.ask("종합 확인 질문?")
json.dump({"user_entry":rec.get("user_entry"),"ask_result":r},open('/tmp/pty_result.json','w'))
