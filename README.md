# neothunderism

[Open the website](https://neothunderism.pages.dev) or [download the Windows app](https://github.com/Leo-TY-H/wt-em-calc/releases/latest/download/neothunderism-windows-x64.zip).

On the website, add aircraft, choose flight conditions, and click **Calculate diagram**. Hover over the plot to inspect results; use the download buttons to save charts or data.

The **Missile simulator** tab offers starter scenarios, missile search and a live setup preview. Enter the launcher aircraft and target aircraft states at release, using km/h for speed and metres for position. Aircraft face their travel direction, with zero roll. The launcher uses Launch angle; the target uses Climb angle. The simulator applies the selected missile's release processing and ignition/guidance timing, plots the engagement, supports timeline playback and exports JSON with SI units. The target follows constant velocity; ideal visibility and radar support retain geometric seeker and flight limits. The model is experimental, and a point-proximity event does not assert aircraft damage.

After a simulation, hover over the missile or its traveled path for speed, Mach, altitude, range and force data. Expand **Propulsion & aerodynamic plots** to select thrust, mass, propellant consumed, drag/lift, drag coefficient, angle of attack, dynamic pressure or net acceleration. Click a chart point to move the 3D timeline there. Chart toolbars save PNGs; **Flight data JSON** includes the numerical telemetry. Missing dynamics at an exact proximity event remain blank.

Missiles are listed as **Display name [missile_id]**, excluding DEFAULT profiles and duplicate display names. Use the same search-and-add menu as the EM aircraft picker and click **Add** beside a missile to compare up to eight missiles with the same aircraft setup. Up to four independent flights calculate concurrently and share a color-coded 3D view, timeline and telemetry plots. Use **Inspect missile result** or a colored legend button for each outcome. Comparison JSON includes every completed flight and identifies incomplete or cancelled comparisons. The target is visual only: its path and marker do not show hover data or attract selection.

The missile solver uses optimized state handling and optional compiled helpers while preserving the original simulation results. The Windows and Docker build scripts include the helpers; a source checkout can build them with `python scripts/build_missile_backend.py` when Cython and a C compiler are available. Without them, the faster Python path runs automatically. `WT_MISSILE_BACKEND=reference` selects the original implementation for comparison, and `WT_MISSILE_WORKERS` limits concurrent jobs to 1–4.

Optional Rust kernels accelerate aerodynamic lift/drag, rotated polar forces,
aircraft force/moment assembly, missile atmosphere and quaternion updates. With
Rust/Cargo installed, build them using `python scripts/build_rust_backend.py`.
The library has no Python ABI dependency and no third-party Rust dependencies.
Docker builds include it; source and Windows installations can build it locally.

`WT_NUMERIC_BACKEND=auto` (the default) uses a verified local library for Python
kernels and missile atmosphere/orientation, while keeping the existing Cython
EM kernels preferred. Missing or stale Rust builds fall back automatically.
`WT_NUMERIC_BACKEND=python` disables Rust; `WT_NUMERIC_BACKEND=rust` requires the
library and also opts compiled EM modules into the Rust calls. Selecting
`WT_MISSILE_BACKEND=reference` continues to bypass all missile acceleration.
No compiler runs during startup. Rebuild after editing native sources.

Run `python scripts/test_rust_backend.py --benchmark --benchmark-flight` to check exact reference
parity and measure call costs including Python marshalling. Local Windows
measurements found approximately 8.6× faster missile quaternion updates, 8×
faster atmosphere evaluation, 2.8× faster moment assembly and 1.4× faster polar
force calls. The optional batch API (`rust_backend.polar_batch`) evaluates a
361-angle sweep about 8× faster, but is not yet used by the adaptive EM solver.
A complete 10-second AIM-9L flight improved from a median 0.637 s to 0.604 s
across seven runs per backend (about 5.2% less runtime), with identical output.
These compare against the Python path, not the existing Cython build; full EM
diagram and altitude-job speedups have not been measured.

For Windows 10/11 x64:

1. Extract the entire Windows ZIP into a writable folder.
2. Run **Launch EM Plotter.cmd** or **Launch Altitude Plotter.cmd**. Keep its console window open while using the app.
3. Launchers open using bundled data without checking for updates or requiring internet access. Run **Update Game Data.cmd** to refresh aircraft data when GitHub is accessible.

No Python installation is needed. To update the app, close it and extract the new download into a new folder.
