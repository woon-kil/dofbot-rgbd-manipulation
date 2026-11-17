"""설계 문서 지도 생성기(tools/doc_map.py) 단위 테스트. 임시 폴더에 가짜 문서를 만들어 읽기 전용 추출만 시험한다."""
import importlib.util
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("doc_map", ROOT / "tools" / "doc_map.py")
DM = importlib.util.module_from_spec(spec)
spec.loader.exec_module(DM)


def make(tmp):
    g = tmp / "docs" / "gates"; g.mkdir(parents=True)
    (tmp / "docs" / "adr").mkdir()
    (tmp / "docs" / "adr" / "ADR-003-x.md").write_text("# ADR3\n", encoding="utf-8")
    (g / "roadmap.md").write_text("# 로드맵\n", encoding="utf-8")
    (g / "X-설계-초안.md").write_text(
        "# X 설계\n결정 [x] 확정 [~] 부분 [ ] 미확인 [ ] 또\n"
        "인용 `Y-설계-초안.md#앵커` `docs/adr/ADR-003-x.md` `docs/gates/roadmap.md` `없는파일.md` `X-설계-초안.md` `R-결과.md`\n", encoding="utf-8")
    (g / "Y-설계-초안.md").write_text("# Y 설계\n칸 없음\n", encoding="utf-8")
    (g / "R-결과.md").write_text("# 결과\n", encoding="utf-8")
    return tmp, g


class DocMapTest(unittest.TestCase):
    def test_extract_counts_and_resolution(self):
        with tempfile.TemporaryDirectory() as t:
            root, g = make(Path(t))
            docs = {d["name"]: d for d in DM.analyze(root=root, gates=g, include=[])}
            x = docs["X-설계-초안.md"]
            self.assertEqual(x["boxes"], {"x": 1, "~": 1, " ": 2})
            self.assertEqual(x["cites"]["scope"], {"docs/gates/Y-설계-초안.md"})   # 앵커 제거, 자기 인용 제외
            self.assertEqual(x["cites"]["adr"], {"docs/adr/ADR-003-x.md"})
            self.assertEqual(x["cites"]["roadmap"], {"docs/gates/roadmap.md"})
            self.assertEqual(x["cites"]["other"], {"docs/gates/R-결과.md"})        # 파일명만 적은 인용도 docs/gates에서 해석
            self.assertEqual(x["unresolved"], ["없는파일.md"])
            self.assertEqual(sum(docs["Y-설계-초안.md"]["boxes"].values()), 0)

    def test_drawn_edges_equal_extracted(self):
        with tempfile.TemporaryDirectory() as t:
            root, g = make(Path(t))
            docs = DM.analyze(root=root, gates=g, include=[])
            e = DM.edges(docs)
            self.assertEqual(len(e), 3)                       # 범위 1 + ADR 1 + roadmap 1, 'other'는 선 없음
            svg = DM.render_svg(docs)                         # 내부 assert가 그린 선 == 추출한 선을 확인
            self.assertIn("<svg", svg)
            self.assertIn("결정 칸 없음", svg)
            self.assertIn("미해결 인용", DM.render_md(docs, "2026-11-17"))

    def test_include_list_adds_file_without_keyword(self):
        with tempfile.TemporaryDirectory() as t:
            root, g = make(Path(t))
            names = DM.scope_files(g, ["R-결과.md", "없음.md"])
            self.assertIn("R-결과.md", names)
            self.assertNotIn("없음.md", names)


if __name__ == "__main__":
    unittest.main()
