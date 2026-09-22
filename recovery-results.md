# MPC and PID Recovery Results

This page presents the final trajectory-analysis results for learned-dynamics model predictive control (MPC) and the PID-labelled baseline. Each complete trajectory is evaluated from 0 to 2.99 s with an upright target of 0°.

> **Scope:** These are illustrative simulation records. They support a transparent control-performance comparison, not a claim of validated physical-robot performance.

## How to read the results

- **Recovered / settling ±2°:** The trajectory enters the ±2° band and stays there for at least 0.5 s through the observed end of the run. The time shown is the first qualifying entry.
- **Settling ±1°:** The same sustained-settling rule, using the stricter ±1° band.
- **RMSE and IAE:** Overall angular-error measures. Lower values are better.
- **Opposite overshoot:** Largest excursion past upright in the opposite direction. Lower is better.
- **Final 0.5 s bias:** Mean angular error over the final 0.5 s. Values closer to 0° indicate better late regulation.
- **Final 0.5 s SD:** Variation during the final 0.5 s; it is within-trajectory variation, not repeated-trial uncertainty.
- **Temporary recovery then exit:** The trajectory entered the ±2° band but did not remain there. The associated timing table describes that first temporary episode.

## Recovery and accuracy summary

| Controller | Initial angle | Recovery within ±2° | Settle ±2° (s) | Settle ±1° (s) | RMSE (°) | IAE (°·s) | Opposite overshoot (°) | Overshoot / initial | Final 0.5 s bias (°) | Final 0.5 s SD (°) |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| MPC | 5° | Not settled by 2.99 s | — | — | 1.526 | 4.333 | 2.008 | 40.2% | -1.791 | 0.118 |
| PID | 5° | Recovered | 0.116 | 0.164 | 0.693 | 0.940 | 6.056 | 121.1% | -0.032 | 0.006 |
| MPC | 10° | Recovered | 0.200 | — | 1.820 | 4.528 | 1.799 | 18.0% | -1.406 | 0.101 |
| PID | 10° | Recovered | 0.597 | 0.867 | 1.519 | 2.336 | 12.007 | 120.1% | -0.095 | 0.020 |
| MPC | 15° | Recovered | 0.300 | — | 2.532 | 5.219 | 1.694 | 11.3% | -1.560 | 0.116 |
| PID | 15° | Recovered | 0.861 | 1.034 | 2.672 | 4.538 | 17.786 | 118.6% | -0.207 | 0.045 |
| MPC | 20° | Recovered | 0.330 | — | 3.302 | 5.811 | 1.495 | 7.5% | -1.200 | 0.021 |
| PID | 20° | Recovered | 0.985 | 1.110 | 4.215 | 7.518 | 23.296 | 116.5% | -0.359 | 0.080 |
| MPC | 30° | Not settled by 2.99 s | — | — | 6.053 | 9.192 | 2.038 | 6.8% | -1.824 | 0.148 |
| PID | 30° | Recovered | 1.079 | 2.416 | 7.367 | 13.376 | 33.213 | 110.7% | -0.640 | 0.146 |
| MPC | 40° | Recovered | 0.610 | — | 9.214 | 13.368 | 1.799 | 4.5% | -1.531 | 0.206 |
| PID | 40° | Recovered | 1.124 | 2.264 | 7.886 | 13.449 | 40.534 | 101.3% | -0.568 | 0.121 |
| MPC | 50° | Recovered | 0.680 | — | 11.903 | 16.900 | 1.881 | 3.8% | -1.610 | 0.211 |
| PID | 50° | Recovered | 1.510 | 2.049 | 11.427 | 15.122 | 44.961 | 89.9% | +0.251 | 0.092 |
| PID | 55° | Recovered | 2.216 | — | 21.385 | 36.560 | 68.804 | 125.1% | -1.143 | 0.232 |
| MPC | 60° | Not settled by 2.99 s | — | — | 13.424 | 19.122 | 2.018 | 3.4% | -1.553 | 0.201 |
| PID | 60° | Failed / truncated | — | — | — | — | — | — | — | — |
| MPC | 70° | Recovered | 0.690 | — | 13.606 | 18.625 | 1.838 | 2.6% | -1.434 | 0.092 |
| PID | 70° | Failed / truncated | — | — | — | — | — | — | — | — |

## Temporary ±2° recovery episodes

These cases entered the ±2° band but exited before qualifying as sustained recovery. PID at 60° and 70° is not assessed because the logs are truncated.

| Controller | Initial angle | First temporary start (s) | Duration (s) | First observed exit (s) |
| --- | ---: | ---: | ---: | ---: |
| MPC | 5° | 0.090 | 2.890 | 2.990 |
| MPC | 30° | 0.470 | 2.440 | 2.920 |
| PID | 55° | 1.281 | 0.506 | 1.790 |
| MPC | 60° | 0.720 | 2.260 | 2.990 |

## Brief interpretation

Across the matched complete runs, MPC generally reaches sustained ±2° recovery sooner and has much lower opposite-direction overshoot than PID. However, it does not attain sustained ±1° settling in any available MPC run, and it retains a negative late-window bias of roughly -1.2° to -1.8°.

PID typically takes longer to recover and overshoots more strongly, but it reaches sustained ±1° settling in its complete 5°–50° runs and has a smaller absolute final bias in every complete matched comparison. These results motivate investigation of a hybrid control strategy—MPC for larger-angle recovery, followed by PID near upright—but do not validate a switching threshold or establish real-time computational performance.

## Data source

The full-precision source data are available in [`outputs/mpc_pid_paper/data/metrics.csv`](../outputs/mpc_pid_paper/data/metrics.csv). The recovery criterion and metric definitions are documented in the repository README.
