# Risk register
| Risk | Likelihood | Impact | Mitigation/status |
|---|---|---|---|
| Scope exceeds one execution session | Certain | Critical | Honest continuity records; never claim completion; resume M1/M2 |
| Robust convex collision degeneracy | High | High | Bounded algorithms + differential tests planned |
| Floating-point cross-platform drift | High | High | Tiered guarantees; stable ordering; no Tier C claim |
| FFI memory safety | Medium | Critical | Isolated unsafe, C conformance/fuzz required |
| Mobile/GPU SDK unavailable | Certain here | Medium | Configure CI/docs; mark unverified |
| Performance architecture premature | Medium | High | Maintain reference paths and benchmark before optimization |
