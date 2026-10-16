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
| `best.pt` | 8.4.45 | 2026-05-01 | Exact SHA-256 match to `sapoepsilon/yolov11s-drone-detector`, `best.pt`; see license discrepancy below. |
| `drone.pt` | 8.3.183 | 2025-08-29 | Exact SHA-256 match to `marie-kjelberg/drone-detector`, `yolo11n_drone.pt` (also `train/weights/last.pt`); upstream declares AGPL-3.0. |
| `../yolo11n.pt` | 8.2.100 | 2024-09-25 | Metadata references Ultralytics' COCO configuration; exact download source unverified. |

## Verified upstream checkpoints

Verified on 2026-10-16 by hashing the local files and comparing them with Hugging
Face's LFS SHA-256 metadata. This establishes byte identity, rather than merely
similar names or model architectures.

- `drone.pt`: Marie Kjelberg's [drone-detector](https://huggingface.co/marie-kjelberg/drone-detector/tree/3616624e16362e35649ef1447cbc871e535d9c13).
  Upstream file: `yolo11n_drone.pt`; SHA-256:
  `311b8bea0a5a9f2b2dd407ade666a91831bbcb4dcd9d4b6580dbe33aac3123be`.
  Upstream model card declares AGPL-3.0 and credits Maciej Pawelczyk and Marek
  Wojtyra for the dataset. Preserve the upstream attribution and applicable terms.
- `best.pt`: sapoepsilon's [yolov11s-drone-detector](https://huggingface.co/sapoepsilon/yolov11s-drone-detector/tree/58897d6d6fadda9dfec979fd5dd7ad9040e73365).
  Upstream file: `best.pt`; SHA-256:
  `337390e1018eaf815474c02e172a224425095466d4a5cd2dbd11301b0aa01b78`.
  Upstream model card declares Apache-2.0 and identifies Ultralytics YOLO11s base
  weights and `pathikg/drone-detection-dataset` as training inputs.

## License discrepancy and remaining checks

For `best.pt`, the upstream Apache-2.0 declaration differs from the checkpoint's
embedded AGPL-3.0 declaration. Both license texts are included in `../licenses/`.
Neither this repository nor its maintainers can resolve that discrepancy merely
by choosing a license label. Ultralytics states that its code and trained models
are AGPL-3.0 by default or require a separate enterprise license for proprietary
use. Retain both declarations and obtain clarification before relying on
Apache-only rights for this checkpoint.

The default runtime detector uses `drone.pt`, whose upstream model card and embedded
license declarations agree on AGPL-3.0. `best.pt` is retained for model comparison.
The root `yolo11n.pt` has not yet been matched to a specific upstream release asset.
Provide corresponding source and preserve upstream notices as required by the
applicable terms. Exact file identity does not independently verify ownership of
all training inputs or establish that every license obligation has been satisfied.
