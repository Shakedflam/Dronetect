# Licensing

Original Dronetect code outside `app/` is licensed under the GNU Affero General
Public License, version 3 only (AGPL-3.0-only), except for the third-party material
identified below. The full text is in `LICENSE` and `licenses/AGPL-3.0.txt`.

Original code in `app/` is licensed under MIT; see `app/LICENSE`. The Android
client communicates with the cloud services over network protocols. This license
choice does not license DJI's SDK, dependencies, or other third-party material.

Copyright (c) 2026 Dronetect contributors. Contributors retain copyright in their
respective contributions. These grants cover only rights held by the contributors.

## Third-party material

| Material | License / terms | Notices |
| --- | --- | --- |
| `controllers/autonomous_drone/autonomous_drone.py` | Apache-2.0 | Adapted from Cyberbotics Webots R2025a; existing copyright and adaptation notices remain in the file. See `licenses/Apache-2.0.txt`. |
| `app/android/gradle/wrapper/gradle-wrapper.jar`, `app/android/gradlew`, `app/android/gradlew.bat` | Apache-2.0 | Gradle wrapper; see script headers and `licenses/Apache-2.0.txt`. |
| `models/best.pt`, `models/drone.pt`, `yolo11n.pt` | Embedded metadata declares AGPL-3.0 | See `models/README.md` for provenance limitations. This is not a new license grant by Dronetect for these files. |
| Ultralytics Python dependency | AGPL-3.0, unless separately licensed | https://www.ultralytics.com/license |
| DJI Mobile SDK | DJI SDK EULA | Downloaded as a Gradle dependency, not licensed under this project's AGPL or MIT grant. See `app/NOTICE.md`. |
| Webots PROTO assets | Upstream asset-specific terms | Referenced by URL in `worlds/`; obtain licenses with the referenced upstream assets. |
| Other installed dependencies | Their respective upstream licenses | Package managers install these separately. Preserve their license and notice files when distributing builds or dependencies. |

## Source availability

Project source is at https://github.com/Shakedflam/Drontect. When distributing
AGPL-covered builds, provide the corresponding source for that build, including
required build and installation material, and retain license and copyright notices.
Where AGPL section 13 applies to a modified network service, prominently offer its
users access to its corresponding source. A link to an unrelated or newer revision
does not substitute for the source of the version actually deployed.

## Publication review status

The two custom drone checkpoints have been matched by SHA-256 to their upstream
Hugging Face repositories; see `models/README.md` for pinned sources and credits.
The default `drone.pt` model's upstream and embedded declarations agree on AGPL-3.0.
The comparison model `best.pt` has conflicting Apache-2.0 model-card and AGPL-3.0
checkpoint declarations; that discrepancy remains unresolved. The root `yolo11n.pt`
has not been matched to a specific upstream release asset. License texts and byte
identity do not independently verify ownership or complete every redistribution
obligation; include additional upstream notices and corresponding source as required.

The Android client and the cloud services have separate license grants. This does
not assert that every possible combined distribution is compatible with DJI's SDK
terms. Do not place DJI SDK binaries under this project's open-source licenses.

These files document licensing; they do not certify the security of a live deployment.
