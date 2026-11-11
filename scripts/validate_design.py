from pathlib import Path
import json,subprocess,sys,datetime
root=Path(__file__).resolve().parents[1]
required=['README.md','docs/architecture/system-design.md','docs/architecture/data-contracts.md','docs/adr/ADR-001-hardware-and-dof.md','docs/adr/ADR-002-calibration.md','docs/adr/ADR-003-motion-and-safety.md','docs/gates/roadmap.md','docs/validation/external-source-review.md','docs/validation/risk-register.md','configs/project.example.json','tests/test_se3.py','tests/test_dual_camera.py','docs/adr/ADR-004-dual-camera.md','docs/architecture/calibration-protocol.md','docs/validation/acceptance-matrix.md','docs/agent-work-order.md','demos/demo-plan.md','docs/adr/ADR-005-pose-and-datum.md']
missing=[s for s in required if not (root/s).is_file()]
import re
readme=(root/'README.md').read_text()
missing+=[f'README link broken: {m}' for m in re.findall(r'`(docs/[^`\s]+\.md)`',readme) if not (root/m).is_file()]
cfg=json.loads((root/'configs/project.example.json').read_text())
interlock=cfg['execution']['dry_run'] and not cfg['execution']['hardware_enabled'] and cfg['execution']['requires_operator_approval'] and cfg['execution'].get('calibration_motion_approved') is False
dual= set(cfg.get('cameras',{}))=={'gemini_335l','dabai'} and all(x.get('required') for x in cfg['cameras'].values()) and not any(cfg['calibration'].get(k) for k in ('gemini_base_verified','dabai_hand_eye_verified','cross_camera_verified'))
result=subprocess.run([sys.executable,'-m','unittest','discover','-s','tests','-v'],cwd=root,capture_output=True,text=True)
report={'timestamp_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'required_files_missing':missing,'default_interlock_safe':interlock,'dual_camera_contract_safe':dual,'unit_tests_pass':result.returncode==0,'unit_test_log':result.stdout+result.stderr,'hardware_tests':'NOT_RUN','overall':'PASS_STATIC_ONLY' if not missing and interlock and dual and result.returncode==0 else 'FAIL'}
(root/'docs/validation/validation-report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='unit_test_log'},ensure_ascii=False,indent=2))
sys.exit(0 if report['overall']=='PASS_STATIC_ONLY' else 1)
