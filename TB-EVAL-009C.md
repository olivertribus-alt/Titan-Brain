# TB-EVAL-009C: Multi-Sensor Fusion Envelope Specification

## Overview
TB-EVAL-009C implements a bounded $O(1)$ worst-case safety envelope union evaluator fusing heterogeneous sensor streams (LIDAR, Depth Camera, Sonar).

## Key Safety Invariants
1. **Conservative Worst-Case Union:**
   $$d_{\text{fusion\_min}} = \min_{k \in S_{\text{valid}}} \left( d_{\text{min}}^{(k)} \right)$$
2. **Fail-Closed Critical Sensor Guard:**
   If any critical sensor enters `STALE` state (exceeding `stale_timeout_s`), the evaluator immediately forces $d_{\text{fusion\_min}} = 0.0\text{ m}$ and signals emergency stop.
3. **Bounded Memory & Execution:**
   Per-evaluation processing is bounded to $K_{\max} = 16$ sensor inputs, enforcing strict $O(1)$ latency.
