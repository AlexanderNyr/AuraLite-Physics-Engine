# Platform verification matrix (2026-07-16)
| Platform | Compile | Tests executed | Status/blocker |
|---|---:|---:|---|
| Linux x86-64 GNU | yes, release+debug | yes, 133 unit tests | **Verified** |
| Linux ARM64 | yes | no | Configured target guidance only |
| Windows x86-64 | yes | no | Configured in CI |
| macOS x86-64/ARM64 | yes | no | Configured in CI |
| Android ARM64 | yes | no | Script/guide only |
| iOS ARM64 | yes | no | Script/guide only |

Linux x86-64 is the primary verified target. SSE2 SIMD and ThreadPool scheduler are verified on this platform.
