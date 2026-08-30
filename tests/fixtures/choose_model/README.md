# Choose Model Fixtures

These simulator screenshots are an opt-in regression set for comparing local
YOLO drone models before changing detector settings or connecting detection
results to the UI.

Run the comparison from the repository root:

```powershell
$env:DRONTECT_RUN_MODEL_SELECTION_TESTS="1"
.\.venv\Scripts\python.exe -m pytest tests\test_choose_model.py -q
```

The test compares:

- `models\drone.pt` at `imgsz=960`, confidence `0.25`
- `models\best.pt` at `imgsz=960`, confidence `0.25`

The saved expectations are intentionally count-based. Use the generated report
and visual inspection when deciding whether a detection is a correct drone,
partial drone, duplicate, bird false positive, or random background false
positive.

The latest JSON report is written to:

```text
artifacts\choose_model_last_report.json
```
