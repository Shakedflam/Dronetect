# Model licenses and provenance

The checkpoints `best.pt`, `drone.pt`, and the root-level `../yolo11n.pt` contain
Ultralytics metadata declaring AGPL-3.0. The license text is included in
`../licenses/AGPL-3.0.txt`; see https://www.ultralytics.com/license.

This records the checkpoints' embedded declarations, not independent verification
of their authorship or redistribution rights. All three files were added in commit
`46dcdf48311cb4b16a7b99d9b178a18d5c5b0727` without source URLs or accompanying
model cards.

| File | Embedded Ultralytics version | Embedded training date | Provenance |
| --- | --- | --- | --- |
| `best.pt` | 8.4.45 | 2026-05-01 | Custom checkpoint; original source and author unverified. |
| `drone.pt` | 8.3.183 | 2025-08-29 | Custom checkpoint; original source and author unverified. |
| `../yolo11n.pt` | 8.2.100 | 2024-09-25 | Metadata references Ultralytics' COCO configuration; exact download source unverified. |

Before declaring redistribution cleared, record the original download URLs,
upstream authors and notices, and any additional applicable terms. If trained
locally, document the base weights, training code/configuration, and dataset
provenance, and provide corresponding source as required by the applicable license.
An embedded license label does not grant rights the checkpoint distributor lacked.
