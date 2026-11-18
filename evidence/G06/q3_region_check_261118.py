"""Q3 영역 재현 스크립트(참고용 사후 분석, 결정 근거 아님). 러너 플랜 §2 R8·P9의 수치를 한 번에 다시 계산한다.
실행: .venv/bin/python -I evidence/G06/q3_region_check_261118.py   (data/g04 사진 필요, gitignore 폴더)
base_shift는 scripts/g01_e0g_analyze.py:48을 import해 (1)(2)에만 쓰고(420×100 영역은 뷰도 결정적), 영역 비교 (3)(4)는 연속 복사 bs()를 쓴다."""
import importlib.util, itertools, cv2, numpy as np
spec = importlib.util.spec_from_file_location('e0g', 'scripts/g01_e0g_analyze.py'); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
G = 'data/g04/'; P = 'data/g04/pa/pa_261118-090521_'
rd = lambda n, d=G: cv2.imread(d + n + ('.png' if d == G else ''))
OLD = (480, 0, 900, 100); NEW = (800, 0, 900, 100)
def bs(a, b, box=OLD):
    x0, y0, x1, y1 = box; g = lambda i: np.ascontiguousarray(cv2.cvtColor(i, cv2.COLOR_BGR2GRAY).astype(np.float32)[y0:y1, x0:x1])   # 호출마다 새 배열 필수: phaseCorrelate는 입력 배열을 제자리에서 수정한다(뷰든 복사본이든; DFT 최적 크기 100·160·1280은 변경, 420은 변경 안 됨, 2026-11-18 확인)
    (dx, dy), r = cv2.phaseCorrelate(g(a), g(b), cv2.createHanningWindow((x1 - x0, y1 - y0), cv2.CV_32F)); return dx, dy, r
# (1) 3회차 P-A 정지 사진: 현행 영역(m.base_shift = BASE_BOX)
ref = cv2.imread(P + 'snap01.color.png')
st = [m.base_shift(ref, cv2.imread(P + n + '.color.png')) for n in ('snap02', 'snap10', 'snap11', 'snap20', 'base_a1', 'base_b2')]
print('(1) P-A 정지 쌍(현행 BASE_BOX) 이동 최대 %.3f px, 응답 %.3f~%.3f' % (max(max(abs(a), abs(b)) for a, b, _ in st), min(r for *_, r in st), max(r for *_, r in st)))
# (2) K1 대 접촉 자세 사진 4쌍(플랜 R8 문구의 출처)
pr = [('p2kref_261117-093753', 'd1_261117-143127_31040'), ('p2kref_261117-093753', 'd1_261117-143127_31041'), ('p2kpost_261117-143127', 'd1_261117-153705_31040'), ('p2kpost_261117-143127', 'd1_261117-153705_31041')]
r2 = [m.base_shift(rd(a), rd(b)) for a, b in pr]
print('(2) K1 대 접촉 4쌍(현행) 응답 %.3f~%.3f, 이동 최대 %.3f' % (min(r for *_, r in r2), max(r for *_, r in r2), max(max(abs(a), abs(b)) for a, b, _ in r2)))
# (3) 42쌍: 실행별 {K1 전, 하강 3101~3104(+41), K1 후}의 모든 쌍
runs = {'R3': ['p2kpost_261117-125333'] + ['d1_261117-143127_310%d' % k for k in (10, 20, 30, 40, 41)] + ['p2kpost_261117-143127'],
        'R4': ['p2kpost_261117-143127'] + ['d1_261117-153705_310%d' % k for k in (10, 20, 30, 40, 41)] + ['p2kpost_261117-153705']}
im = {r: [rd(n) for n in ns] for r, ns in runs.items()}
for nm, box in (('현행 (480,0,900,100)', OLD), ("(나') (800,0,900,100)", NEW)):
    rs = []; ss = []
    for L in im.values():
        for a, b in itertools.combinations(L, 2):
            dx, dy, r = bs(a, b, box); rs.append(r); ss.append(max(abs(dx), abs(dy)))
    print('(3) 42쌍 %-22s 응답 최소 %.2f 중앙 %.2f, 이동 최대 %.2f, 응답<0.3 %d/%d' % (nm, min(rs), float(np.median(rs)), max(ss), sum(r < 0.3 for r in rs), len(rs)))
# (4) 양성 대조: 둘째 사진 이동 주입
def shift(i, sx, sy): return cv2.warpAffine(i, np.float32([[1, 0, sx], [0, 1, sy]]), (i.shape[1], i.shape[0]), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
R = 'd1_261117-143127_'
for pn, (a, b) in {'팔정지 31040→31041': (R + '31040', R + '31041'), '팔이동 31020→31030': (R + '31020', R + '31030'), '팔이동 31010→31040': (R + '31010', R + '31040')}.items():
    A, B = rd(a), rd(b)
    for nm, box in (('현행', OLD), ("(나')", NEW)):
        e = []; rr = []
        for s in (1, 2, 3):
            dx, dy, r = bs(A, shift(B, s, 0), box); e.append(abs(dx - bs(A, B, box)[0] - s)); rr.append(r)
        print('(4) %s %-4s 주입 1/2/3 px 오차(기준 대비) %s, 응답 %.2f~%.2f, 무주입 응답 %.2f 이동 (%.2f,%.2f)' % (pn, nm, ['%.2f' % x for x in e], min(rr), max(rr), bs(A, B, box)[2], bs(A, B, box)[0], bs(A, B, box)[1]))
