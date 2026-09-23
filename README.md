# Self-Balancing Robot Control: PID and Learned-Dynamics MPC

An open-source, illustrative simulation study comparing a conventional PID-labelled controller with learned-dynamics Model Predictive Control (MPC) for a self-balancing robot. The project examines the trade-off between rapid recovery from an initial tilt, accurate near-upright regulation, residual error, and recorded control effort. It also proposes a hybrid controller for future testing: MPC for large-angle recovery, followed by PID near equilibrium.

> **Important:** The numerical results in this repository are supplied illustrative simulation data for research-writing and reproducible-analysis purposes. They are not independently validated experiments and must not be presented as physical-robot results.

## Research question

Within the supplied simulation records, how do PID and learned-dynamics MPC differ in recovery speed, angular accuracy, residual bias, and control demand? Do those differences justify investigating a switch to PID near upright?

## Methodology

### Controllers

The project compares two controller labels used in the supplied data:

- **PID:** The prior controller formulation combines an angle loop and a position loop. The reported integral gains are zero, so the historical implementation is technically a proportional-derivative (PD) controller, while the `PID` label is retained to match the source workbook.
- **Learned-dynamics MPC:** A recurrent neural-network dynamics model is used to predict future system states. At each time step, MPC optimizes a sequence of future inputs against predicted behavior and applies the first input before re-solving at the next step.

The reported state representation is `[cos(theta), sin(theta), theta_dot, position, position_dot]`. Historical documentation describes an LSTM predictor trained from PID-generated trajectories, but the trained model, training data, normalization, and runtime logs are not included here; these details should therefore be treated as unverified background rather than reproducible controller code.

### Test conditions and data processing

The supplied trajectories cover initial-angle recovery tests at multiple initial tilts. Nine conditions are shared between PID and MPC, with one additional PID-only condition. Each complete trajectory is analyzed over a common 0–2.99 s window. Values at the 2.49 s and 2.99 s boundaries are interpolated only when needed to align the observation windows.

For each complete run, the analysis computes:

- root mean squared error (RMSE)
- integrated absolute error (IAE)
- opposite-direction overshoot
- late-window bias and variation
- mean absolute input and mean squared input
- peak absolute input and squared-input integral
- sustained settling behavior within ±1° and ±2° bands

Truncated PID runs are preserved but are not assigned full-window metrics. The 0.5 s standard deviation reported in the analysis represents within-trajectory variation, not repeated-trial uncertainty.

### Interpretation framework

The comparison separates three questions that are often conflated:

1. **Recovery:** How quickly does the robot enter and remain in a specified angular tolerance band?
2. **Regulation:** How close does it remain to upright late in the run?
3. **Input demand:** How large are the recorded controller inputs?

Recorded input demand is not treated as measured energy efficiency, and no computational-runtime conclusion is made because execution-time measurements were not supplied.

## Main observations

In the supplied records, MPC shows faster recovery in the matched cases that settle within the ±2° criterion and lower recorded input demand. PID has smaller absolute late-window bias in every complete matched comparison. MPC does not meet the stricter ±1° settling criterion in the available runs, and several trajectories re-exit the ±2° band after initially reaching it.

These patterns motivate—rather than validate—a hybrid strategy: use MPC to recover from larger deviations, then transfer to PID near upright. A 5° switching threshold is a candidate for evaluation, not an established optimum. Validating that strategy requires new hybrid simulations or robot experiments, timing measurements, and a clearly defined switching rule.

## Repository contents

```text
models/
  selfBalancingRobot_Recovery.slx       # Simulink recovery-test model
  selfBalancingRobot_Demonstration.slx  # Simulink demonstration model
  selfBalancingRobot_LargeDisturbance.slx # Simulink large-disturbance model
  learnedDynamicsLSTM.mat               # Trained LSTM dynamics model
src/
  trainLearnedDynamicsModel.m           # LSTM training workflow
  extractTrainingData.m                 # Training-data extraction
  resampleTrainingData.m                # Data resampling
  runLearnedDynamicsMPC.m               # MPC controller implementation
  extractPIDRecoveryData.m              # PID recovery-data export
  playTrajectoryInSimulink.m            # Trajectory playback utility
data/
  recovery_05deg_trajectory.csv         # Example 5° recovery trajectory
outputs/
  mpc_pid_comparison/        # Original trajectory exports and comparison workbook
  mpc_pid_paper/             # LaTeX manuscript, processed data, figures, and tables
  MPC_PID_Overleaf/          # Self-contained Overleaf-ready manuscript project
work/
  prepare.py                 # Analysis-preparation workflow
  paper/                     # Analysis and quality-assurance scripts
```

## MATLAB and Simulink implementation

The MATLAB/Simulink files implement the data-preparation, LSTM-model training, trajectory playback, and controller workflows used to support the study. Place these files in the suggested `models/`, `src/`, and `data/` folders above when publishing the repository.

| Current filename | Purpose |
| --- | --- |
| `selfBalancingRobot_Demonstration.slx` | Demonstration model (confirm the full original filename before renaming). |
| `selfBalancingRobot_LargeDisturbance.slx` | Large-disturbance scenario model (confirm the full original filename before renaming). |
| `selfBalancingRobot_Recovery.slx` | Initial-angle recovery scenario model. |
| `learnedDynamicsLSTM.mat` | Saved trained LSTM dynamics model. |
| `extractTrainingData.m` | Extracts data for model training or analysis. |
| `trainLearnedDynamicsModel.m` | Trains the learned-dynamics LSTM model. |
| `resampleTrainingData.m` | Resamples trajectories to the required time base. |
| `playTrajectoryInSimulink.m` | Loads and plays a trajectory in Simulink. |
| `extractPIDRecoveryData.m` | Extracts PID recovery results. |
| `runLearnedDynamicsMPC.m` | Runs the learned-dynamics MPC workflow. |


### Suggested workflow

1. Open a Simulink model from `models/` and make sure MATLAB can find `src/`, `data/`, and `models/`.
2. Run `extractTrainingData.m` and `resampleTrainingData.m` when rebuilding the LSTM training set.
3. Run `trainLearnedDynamicsModel.m` to create or replace `learnedDynamicsLSTM.mat`.
4. Run `runLearnedDynamicsMPC.m` to simulate MPC behavior, then use `playTrajectoryInSimulink.m` to inspect trajectories.
5. Run `extractPIDRecoveryData.m` to export PID recovery data for comparison.

The exact function arguments, Simulink block parameters, and model dependencies should be documented after the files are uploaded and checked. Do not commit any private paths, local machine-specific settings, credentials, or proprietary toolbox files.


## Limitations and responsible use

- The proposed hybrid controller has not been simulated or experimentally evaluated in this repository.
- Results should be cited as an illustrative analysis unless replaced with authenticated experimental or simulation records.

## Future work

Useful next steps include publishing reproducible controller implementations, documenting the robot model and all controller settings, testing explicit hybrid switching logic across disturbances and initial angles, measuring real-time computational cost, and validating the approach on physical hardware.

