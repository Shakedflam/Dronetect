# Model licenses and provenance

The checkpoints `drone.pt` and the root-level `../yolo11n.pt` contain
Ultralytics metadata declaring AGPL-3.0. The license text is included in
`../licenses/AGPL-3.0.txt`; see https://www.ultralytics.com/license.

This records the checkpoints' embedded declarations, not independent verification
of their authorship or redistribution rights. The files were originally added without source URLs or accompanying model cards.

| File | Embedded Ultralytics version | Embedded training date | Provenance |
| --- | --- | --- | --- |
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

## Remaining checks

The default runtime detector uses `drone.pt`, whose upstream model card and embedded
license declarations agree on AGPL-3.0.
The root `yolo11n.pt` has not yet been matched to a specific upstream release asset.
Provide corresponding source and preserve upstream notices as required by the
applicable terms. Exact file identity does not independently verify ownership of
all training inputs or establish that every license obligation has been satisfied.
