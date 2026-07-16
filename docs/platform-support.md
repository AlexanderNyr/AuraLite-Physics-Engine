# Platform verification matrix (2026-07-16)
| Platform | Compile | Tests executed | Status/blocker |
|---|---:|---:|---|
| Linux x86-64 GNU | yes, release+debug | yes, 16 unit + doctests | **Verified** in Arena container |
| Linux ARM64 | no | no | Configured target guidance only; cross linker/sysroot absent |
| Windows x86-64/ARM64 | no | no | Configured in CI; Windows host/MSVC SDK absent |
| macOS x86-64/ARM64 | no | no | Configured in CI; Apple host/Xcode absent |
| Android ARM64 | no | no | Script/guide only; NDK absent |
| iOS ARM64/simulator | no | no | Script/guide only; macOS/Xcode/signing absent |

No unavailable target is claimed verified. GPU, sanitizer, Miri, and fuzz have not been run.
