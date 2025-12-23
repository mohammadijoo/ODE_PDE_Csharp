<div style="font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; line-height: 1.6;">

  <h1 align="center" style="margin-bottom: 0.2em;">ODE &amp; PDE Solvers in C# (2D Heat Equation + Inverted Pendulum SMC)</h1>

  <p style="font-size: 0.98rem; max-width: 980px; margin: 0.2rem auto 0;">
    A .NET repository focused on <strong>numerical simulation</strong> and <strong>visualization</strong> of
    <strong>partial differential equations (PDEs)</strong> and <strong>ordinary differential equations (ODEs)</strong> using <strong>C#</strong>.
    The project contains two complete end-to-end examples:
  </p>

  <ul style="max-width: 980px; margin: 0.6rem auto 0.2rem; padding-left: 1.2rem;">
    <li>
      <strong>PDE:</strong> 2D heat-conduction (diffusion) equation solved with an explicit finite-difference method (FTCS),
      producing <strong>high-resolution heatmap snapshots</strong>, summary plots, and a CSV log.
    </li>
    <li>
      <strong>ODE:</strong> a nonlinear cart–pole (inverted pendulum) system stabilized using
      <strong>Sliding Mode Control (SMC)</strong> with injected disturbances, producing
      <strong>high-FPS animation frames</strong>, an optional MP4 video via <strong>FFmpeg</strong>, plots, and a CSV log.
    </li>
  </ul>

  <p align="center" style="font-size: 1rem; color: #666; margin: 0.45rem auto 0;">
    Platform target: <strong>.NET 8</strong> • IDE: <strong>Visual Studio 2022</strong> • Plotting: <strong>ScottPlot (v5)</strong> •
    Rendering: <strong>System.Drawing</strong> • Optional video encoding: <strong>FFmpeg</strong>
  </p>

</div>

<hr />

<!-- ========================================================= -->
<!-- Table of Contents                                        -->
<!-- ========================================================= -->

<ul style="list-style: none; padding-left: 0; font-size: 0.97rem;">
  <li> <a href="#about-this-repository">About this repository</a></li>
  <li> <a href="#what-are-odes-and-pdes">What are ODEs and PDEs?</a></li>
  <li> <a href="#example-1-pde-2d-heat-equation">Example 1 (PDE): 2D Heat Equation</a></li>
  <li> <a href="#example-2-ode-inverted-pendulum-with-smc">Example 2 (ODE): Inverted Pendulum with Sliding Mode Control</a></li>
  <li> <a href="#sliding-mode-control-theory">Sliding Mode Control (SMC) theory</a></li>
  <li> <a href="#numerical-methods-used-in-this-repo">Numerical methods used in this repo</a></li>
  <li> <a href="#plots-and-animations-scottplot-systemdrawing-ffmpeg">Plots and animations (ScottPlot, System.Drawing, FFmpeg)</a></li>
  <li> <a href="#dependencies-and-installation">Dependencies and installation</a></li>
  <li> <a href="#repository-file-guide">Repository file guide (full explanation)</a></li>
  <li> <a href="#running-the-projects-and-generating-results">Running the projects and generating results</a></li>
  <li> <a href="#operating-system-guides-windows-macos-linux">Operating system guides (Windows / macOS / Linux)</a></li>
  <li> <a href="#troubleshooting">Troubleshooting</a></li>
  <li> <a href="#implementation-tutorial-video">Implementation tutorial video</a></li>
</ul>

---

<a id="about-this-repository"></a>

## About this repository

This repository is designed as a practical reference for implementing numerical solvers in modern C# (.NET 8):

- A <strong>PDE workflow</strong> that starts from a physical model (heat conduction), discretizes it, runs a time-marching solver,
  and produces professional plots and image snapshots.
- An <strong>ODE + control workflow</strong> that implements a nonlinear plant (cart–pole), injects disturbances, runs a robust controller
  (Sliding Mode Control), generates simulation plots, and produces an animation/video.

The goal is not only to “get results,” but also to show common engineering practices:

- stable time stepping (explicit stability constraints),
- reproducible logging to CSV,
- high-resolution figure export (including 300 DPI metadata),
- animation frames for debugging and communication,
- optional video encoding through FFmpeg,
- a clean Visual Studio solution structure (one solution, multiple projects).

### Quick map of the two projects

- <code>Heat2D</code> → solves the 2D heat equation and writes results into <code>output/heat2d/</code>
- <code>PendulumSlidingMode</code> → simulates a cart–pole and writes results into <code>output/pendulum_sliding_mode/</code>

---

<a id="what-are-odes-and-pdes"></a>

## What are ODEs and PDEs?

### Ordinary Differential Equations (ODEs)

An ODE describes the evolution of one or more state variables with respect to a single independent variable (usually time):

$$
\dot{\mathbf{x}}(t) = \mathbf{f}(\mathbf{x}(t), \mathbf{u}(t), t),
$$

where:

- $\mathbf{x}(t)$ is the state (e.g., cart position, pole angle),
- $\mathbf{u}(t)$ is an input (e.g., a control force),
- $\mathbf{f}(\cdot)$ describes the nonlinear dynamics.

In this repository, the inverted pendulum example is a nonlinear ODE system integrated in time using a 4th-order Runge–Kutta (RK4) scheme with sub-stepping per rendered frame.

### Partial Differential Equations (PDEs)

A PDE involves partial derivatives with respect to two or more independent variables (e.g., time and space). A canonical example is the
heat equation (diffusion equation):

$$
\frac{\partial T}{\partial t} = \alpha \left( \frac{\partial^2 T}{\partial x^2} + \frac{\partial^2 T}{\partial y^2} \right),
$$

where:

- $T(x,y,t)$ is temperature,
- $\alpha$ is thermal diffusivity,
- $(x,y)$ are spatial coordinates,
- $t$ is time.

In this repository, the heat equation is discretized in space using finite differences and integrated forward in time using an explicit FTCS method.

---

<a id="example-1-pde-2d-heat-equation"></a>

## Example 1 (PDE): 2D Heat Equation

### Physical model

The PDE solved in <code>ODE_PDE_Csharp/Heat2D/Program.cs</code> is:

$$
\frac{\partial T}{\partial t} = \alpha \left( \frac{\partial^2 T}{\partial x^2} + \frac{\partial^2 T}{\partial y^2} \right).
$$

Interpretation:

- conduction in a 2D plate,
- no internal heat sources,
- constant isotropic diffusivity $\alpha$,
- fixed temperature boundaries (Dirichlet BCs).

Boundary conditions:

- left boundary held at $T=100$,
- right/top/bottom held at $T=0$,

which produces a diffusion front moving from the left edge into the interior.

### Discretization summary (what the code computes)

The solver uses:

- uniform grid in $x$ and $y$,
- second-order central differences for $\partial^2 T/\partial x^2$ and $\partial^2 T/\partial y^2$,
- explicit time stepping (Forward Euler in time).

At interior grid nodes $(i,j)$:

$$
T^{n+1}_{i,j} = T^{n}_{i,j} + \alpha\,\Delta t\left(
\frac{T^n_{i+1,j}-2T^n_{i,j}+T^n_{i-1,j}}{\Delta x^2}
+
\frac{T^n_{i,j+1}-2T^n_{i,j}+T^n_{i,j-1}}{\Delta y^2}
\right).
$$

### Stability constraint (explicit diffusion)

For the 2D explicit diffusion scheme, a commonly used sufficient stability constraint is:

$$
\Delta t \le \frac{1}{2\alpha \left(\frac{1}{\Delta x^2} + \frac{1}{\Delta y^2}\right)}.
$$

The code computes <code>dtStable</code> and uses a conservative factor (<code>0.80</code>) to avoid running near the limit.

### Output artifacts

When you run <code>Heat2D</code> you will find:

- heatmap snapshots: <code>output/heat2d/heat_t*.png</code>
- final centerline profile: <code>output/heat2d/centerline_final.png</code>
- center temperature vs time: <code>output/heat2d/center_point_vs_time.png</code>
- CSV log: <code>output/heat2d/heat2d_log.csv</code>

This combination is typical for PDE workflows:

- images for qualitative verification,
- plots for engineering interpretation,
- CSV for reproducibility and post-processing in Python/Matlab/Excel.

---

<a id="example-2-ode-inverted-pendulum-with-smc"></a>

## Example 2 (ODE): Inverted Pendulum with Sliding Mode Control

The second example (<code>ODE_PDE_Csharp/PendulumSlidingMode/Program.cs</code>) simulates a nonlinear cart–pole system with disturbances and stabilizes it using Sliding Mode Control (SMC).

### State variables and conventions

The state vector is:

$$
\mathbf{x} =
\begin{bmatrix}
x & \dot{x} & \theta & \dot{\theta}
\end{bmatrix}^T
$$

- $x$ is cart position (m),
- $\theta$ is pole angle (rad) with $\theta=0$ upright,
- angle wrapping keeps $\theta \in [-\pi,\pi]$ to avoid numeric drift.

### Disturbances (exactly two)

Two external disturbance pulses are injected as an external torque about the pole pivot:

- pulse 1: starts at $t=0.5\,s$ (positive torque; “push right”)
- pulse 2: starts at $t=5.0\,s$ (negative torque; “push left”)

Each pulse lasts 0.5 seconds and uses a smooth half-sine profile:

$$
\tau_{ext}(t) =
\begin{cases}
+\tau_{amp}\sin\left(\pi\frac{t-t_1}{d}\right) & t\in[t_1,t_1+d] \\
-\tau_{amp}\sin\left(\pi\frac{t-t_2}{d}\right) & t\in[t_2,t_2+d] \\
0 & \text{otherwise}
\end{cases}
$$

where $d=0.5\,s$.

For visualization and logging, the code also reports an “equivalent bob force”:

$$
F_{eq}(t) = \frac{\tau_{ext}(t)}{L}.
$$

An on-screen arrow is drawn for 0.5 seconds after each disturbance start time. The arrow length is constant (only direction changes) to provide a clear direction indicator.

### Simulation and rendering pipeline

This example is intentionally end-to-end:

- integrate the nonlinear dynamics (RK4),
- render each frame using the current integrated state (System.Drawing),
- save frames as PNG images,
- optionally encode MP4 using FFmpeg,
- generate plots (ScottPlot),
- save a CSV log.

Outputs include:

- <code>output/pendulum_sliding_mode/frames/frame_000000.png</code> ... <code>frame_005999.png</code>
- <code>output/pendulum_sliding_mode/pendulum_smc_10s_6000f.mp4</code> (if FFmpeg is available)
- plots: <code>cart_position.png</code>, <code>pole_angle.png</code>, <code>control_force.png</code>, etc.
- <code>output/pendulum_sliding_mode/cartpole_log.csv</code>

---

<a id="sliding-mode-control-theory"></a>

## Sliding Mode Control (SMC) theory

Sliding Mode Control is a robust nonlinear control method that enforces a chosen “sliding manifold” (sliding surface) in the state space, even in the presence of disturbances and model uncertainty.

### 1) Sliding surface design

For a general second-order system, a common sliding surface is:

$$
s = \dot{e} + \lambda e,
$$

where $e$ is the tracking error. Enforcing $s \to 0$ yields stable error dynamics.

In this repository, the “error” is the pendulum’s deviation from upright ($\theta$), plus a cart stabilization term. The implemented sliding surface is:

$$
s(\mathbf{x}) = \dot{\theta} + \lambda_{\theta}\,\theta + \alpha\left(\dot{x} + \lambda_x\,x\right).
$$

Interpretation:

- $\lambda_{\theta}$ penalizes angle error,
- $\alpha$ couples cart motion into the balancing objective (cart acceleration is how the pendulum is “caught”),
- $\lambda_x$ adds a soft tendency toward cart centering.

### 2) Reaching condition (Lyapunov argument)

Define a Lyapunov candidate:

$$
V(s) = \frac{1}{2}s^2.
$$

A sufficient reaching condition is:

$$
\dot{V} = s\dot{s} \le -\eta|s|,\quad \eta>0,
$$

which implies that $|s|$ decreases and reaches zero in finite time.

A classical SMC choice is:

$$
\dot{s} = -k\,\mathrm{sign}(s),\quad k>0.
$$

This produces strong corrective action and robust disturbance rejection (particularly for “matched” disturbances).

### 3) Boundary layer to reduce chattering

The ideal sign function can cause chattering (high-frequency switching). A common mitigation is to replace $\mathrm{sign}(\cdot)$ with a continuous saturation function:

$$
\mathrm{sat}(z)=
\begin{cases}
-1 & z<-1 \\
z  & |z|\le 1 \\
+1 & z>1
\end{cases}
$$

and enforce:

$$
\dot{s} = -k\,\mathrm{sat}\left(\frac{s}{\phi}\right),
$$

where $\phi>0$ defines the boundary-layer thickness. In this repository:

- <code>K</code> corresponds to $k$,
- <code>Phi</code> corresponds to $\phi$,
- the helper <code>Sat()</code> implements the saturation map.

### 4) How the code computes the control input

For the cart–pole, an explicit closed-form mapping $u \mapsto \dot{s}$ can be inconvenient to derive and maintain. The code therefore uses a pragmatic numeric linearization:

1. Compute the desired sliding surface derivative:  
   $$\dot{s}_{des} = -k\,\mathrm{sat}\left(\frac{s}{\phi}\right).$$
2. Approximate $\dot{s}(u)$ locally as an affine function:  
   $$\dot{s}(u)\approx a u + b.$$
3. Estimate $a$ and $b$ numerically using two evaluations of the nominal dynamics (ignoring the disturbance torque inside the control law):  
   $$a \approx \dot{s}(1)-\dot{s}(0), \qquad b \approx \dot{s}(0).$$
4. Solve for the control:  
   $$u_{smc} = \frac{\dot{s}_{des}-b}{a}.$$
5. Apply saturation:  
   $$u = \mathrm{clamp}(u_{smc}+u_{hold}, -u_{max}, u_{max}).$$

This approach is frequently used in practice when you want an SMC-like behavior without hard-coding a brittle symbolic derivation.

### 5) Cart-centering term with gating

To avoid cart drift, a centering term is added:

$$
u_{hold} = g(\theta)\left(-k_p x - k_d \dot{x}\right),
$$

with a gate:

$$
g(\theta)=\mathrm{clamp}\left(1-\frac{|\theta|}{\theta_{gate}}, 0, 1\right).
$$

This means:

- when $|\theta|$ is large, $g(\theta)\approx 0$: prioritize catching/balancing,
- near upright, $g(\theta)\approx 1$: softly return the cart toward $x=0$.

---

<a id="numerical-methods-used-in-this-repo"></a>

## Numerical methods used in this repo

### PDE solver: explicit finite differences (FTCS)

The heat solver is a textbook example of an explicit diffusion integrator:

- Space: central differences (2nd order)
- Time: forward Euler (1st order)

Strengths:

- straightforward implementation,
- inexpensive per time step,
- natural baseline for later improvements.

Limitations:

- stability constraint can force small $\Delta t$ for fine grids.

A typical extension is an implicit method (e.g., Crank–Nicolson or ADI) to relax/remove the explicit stability constraint.

### ODE solver: RK4 with sub-stepping

The cart–pole uses classical Runge–Kutta 4th order (RK4). The simulation produces 6000 frames in 10 seconds (600 FPS), so the code uses substeps per frame:

- frame time step: $\Delta t_{frame} = T_{video}/N_{frames}$
- physics step: $\Delta t_{physics} = \Delta t_{frame}/N_{substeps}$

Sub-stepping improves stability and ensures the rendered frames correspond to physically integrated states.

---

<a id="plots-and-animations-scottplot-systemdrawing-ffmpeg"></a>

## Plots and animations (ScottPlot, System.Drawing, FFmpeg)

### ScottPlot (plots and heatmaps)

This repository uses <a href="https://scottplot.net/" target="_blank">ScottPlot</a> (v5) for:

- heatmaps (temperature fields),
- time-history plots (signals vs time),
- final cross-sections (centerline temperature).

Export behavior:

- images are saved at high pixel resolution (e.g., 2400×1800 or 2600×1600),
- the PNG is rewritten to embed <strong>300 DPI metadata</strong> safely (no cropping) for print-quality export.

### System.Drawing (frame rendering)

The inverted pendulum animation frames are drawn using:

- <code>Bitmap</code> as an offscreen canvas,
- <code>Graphics</code> drawing primitives (lines, filled shapes),
- anti-aliasing and high-quality interpolation settings.

Frames are saved as PNGs into:

- <code>output/pendulum_sliding_mode/frames/</code>

### FFmpeg (optional MP4 encoding)

If <code>ffmpeg</code> is available on your PATH, the program automatically encodes:

- <code>pendulum_smc_10s_6000f.mp4</code>

The FFmpeg invocation is designed to avoid “hung encoding” scenarios by <strong>not</strong> redirecting stdout/stderr and by using a fast encoder preset.

---

<a id="dependencies-and-installation"></a>

## Dependencies and installation

### Required software

- <strong>.NET SDK 8</strong> (for build/run with <code>dotnet</code>)
- <strong>Visual Studio 2022</strong> (recommended on Windows)
- NuGet packages:
  - <strong>ScottPlot</strong> (v5)
  - <strong>System.Drawing.Common</strong> (for Bitmap/Graphics types used in rendering and DPI metadata)

### Optional (recommended)

- <strong>FFmpeg</strong> for MP4 encoding
  - without FFmpeg, you still get PNG frames and can encode manually.

### NuGet packages used

Both projects use the same package set (as configured in the <code>.csproj</code> files):

<ul>
  <li><strong>ScottPlot</strong> (<code>Version=5.*</code>)</li>
  <li><strong>System.Drawing.Common</strong> (<code>Version=8.0.0</code>)</li>
</ul>

---

<a id="repository-file-guide"></a>

## Repository file guide (full explanation)

This section explains every important file and its role.

### <code>ODE_PDE_Csharp/Heat2D/Program.cs</code>

A self-contained PDE pipeline:

- defines the physical parameters and grid (<code>nx</code>, <code>ny</code>, <code>dx</code>, <code>dy</code>),
- computes an explicit stability time step (<code>dtStable</code>) and chooses a conservative <code>dt</code>,
- stores the 2D temperature field in a flattened 1D array for cache efficiency,
- updates interior nodes using the FTCS scheme,
- enforces Dirichlet boundary conditions on every time step,
- logs temperature at the grid center over time,
- periodically saves heatmap snapshots,
- produces summary plots:
  - final centerline temperature profile,
  - center temperature versus time,
- saves a CSV log (<code>heat2d_log.csv</code>).

Plot/export details:

- <code>SavePlotPng300Dpi()</code> exports at high resolution then rewrites PNG metadata to embed 300 DPI, using a pixel-exact clone to avoid DPI cropping artifacts.

### <code>ODE_PDE_Csharp/Heat2D/Heat2D.csproj</code>

Project configuration for the heat solver:

- targets <code>net8.0-windows</code>,
- enables WinForms types (<code>&lt;UseWindowsForms&gt;true&lt;/UseWindowsForms&gt;</code>) for consistent System.Drawing behavior in this repository,
- references:
  - ScottPlot v5
  - System.Drawing.Common

### <code>ODE_PDE_Csharp/PendulumSlidingMode/Program.cs</code>

A complete ODE + control + rendering pipeline:

- defines a physically motivated cart–pole model including:
  - rod+bob mass distribution,
  - center-of-mass location,
  - pivot inertia and derived inertia factor,
  - cart and pole damping,
- defines the control strategy:
  - sliding surface <code>s</code>,
  - boundary-layer saturation,
  - numeric linearization to compute <code>u</code>,
  - actuator saturation,
  - gated cart-centering,
- defines disturbances:
  - two half-sine torque pulses,
  - 0.5 second arrow display window,
- integrates dynamics using RK4 with substeps per frame,
- renders frames into a <code>Bitmap</code> using <code>Graphics</code>,
- saves frames and logs signals,
- saves plots and CSV,
- optionally encodes MP4 via FFmpeg (when available).

### <code>ODE_PDE_Csharp/PendulumSlidingMode/PendulumSlidingMode.csproj</code>

Project configuration for the cart–pole simulation:

- targets <code>net8.0-windows</code>,
- enables WinForms types for optional preview (<code>--preview</code>),
- references:
  - ScottPlot v5
  - System.Drawing.Common

---

<a id="running-the-projects-and-generating-results"></a>

## Running the projects and generating results

### Important note: where the <code>output/</code> folder is created

Both programs write to relative paths like <code>output/heat2d</code>. The exact location depends on the <strong>working directory</strong>:

- When you run from Visual Studio Debug, the default working directory is often:
  <code>.../bin/Debug/net8.0-windows/</code>
  so the output folder may appear under that directory.
- If you want <code>output/</code> at the repository root, set the project working directory to the solution folder (steps shown in the Windows guide).

### Run Heat2D

- Visual Studio: select <code>Heat2D</code> as Startup Project → Run
- CLI:
  <pre><code>dotnet run --project ODE_PDE_Csharp/Heat2D -c Release</code></pre>

Expected outputs:

- <code>output/heat2d/heat_t*.png</code>
- <code>output/heat2d/centerline_final.png</code>
- <code>output/heat2d/center_point_vs_time.png</code>
- <code>output/heat2d/heat2d_log.csv</code>

### Run PendulumSlidingMode

- Visual Studio: select <code>PendulumSlidingMode</code> as Startup Project → Run
- CLI:
  <pre><code>dotnet run --project ODE_PDE_Csharp/PendulumSlidingMode -c Release</code></pre>

Optional live preview window:

<pre><code>dotnet run --project ODE_PDE_Csharp/PendulumSlidingMode -c Release -- --preview</code></pre>

Expected outputs:

- <code>output/pendulum_sliding_mode/frames/frame_*.png</code>
- plots in <code>output/pendulum_sliding_mode/</code>
- CSV: <code>output/pendulum_sliding_mode/cartpole_log.csv</code>
- MP4: <code>output/pendulum_sliding_mode/pendulum_smc_10s_6000f.mp4</code> (if FFmpeg is installed)

### Manual MP4 encoding (if needed)

If FFmpeg is installed but you want to encode manually (or customize quality):

<pre><code>ffmpeg -y -framerate 600 -i output/pendulum_sliding_mode/frames/frame_%06d.png ^
  -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p ^
  output/pendulum_sliding_mode/pendulum_smc_10s_6000f.mp4</code></pre>

(Use <code>\</code> line continuations on macOS/Linux.)

---

<a id="operating-system-guides-windows-macos-linux"></a>

## Operating system guides (Windows / macOS / Linux)

### Windows (Visual Studio 2022) — full setup from a blank solution

This section matches the intended “one solution, two projects” structure.

#### 1) Install prerequisites

- Visual Studio 2022
  - Workload: <strong>“.NET desktop development”</strong>
- .NET SDK 8 (if not already included)
- Optional: FFmpeg

#### 2) Create a blank solution

1. <strong>File → New → Project</strong>  
2. Choose <strong>“Blank Solution”</strong>  
3. Name: <strong>ODE_PDE_Csharp</strong>  
4. Location: your desired folder  
5. Create

#### 3) Add the Heat2D project

1. Right-click the solution → <strong>Add → New Project</strong>  
2. Choose <strong>Console App</strong> (C#)  
3. Name: <strong>Heat2D</strong>  
4. Framework: <strong>.NET 8</strong>  
5. Create  
6. Replace the generated <code>Program.cs</code> with <code>ODE_PDE_Csharp/Heat2D/Program.cs</code> content in this repository.
7. Right-click <strong>Heat2D</strong> → <strong>Manage NuGet Packages</strong>:
   - Install: <strong>ScottPlot</strong> (5.*)
   - Install: <strong>System.Drawing.Common</strong> (8.0.0)
8. Ensure the project file matches <code>Heat2D.csproj</code> (TargetFramework and UseWindowsForms).

#### 4) Add the PendulumSlidingMode project

1. Right-click the solution → <strong>Add → New Project</strong>  
2. Choose <strong>Console App</strong> (C#)  
3. Name: <strong>PendulumSlidingMode</strong>  
4. Framework: <strong>.NET 8</strong>  
5. Create  
6. Replace the generated <code>Program.cs</code> with <code>ODE_PDE_Csharp/PendulumSlidingMode/Program.cs</code> content in this repository.
7. Right-click <strong>PendulumSlidingMode</strong> → <strong>Manage NuGet Packages</strong>:
   - Install: <strong>ScottPlot</strong> (5.*)
   - Install: <strong>System.Drawing.Common</strong> (8.0.0)
8. Ensure the project file matches <code>PendulumSlidingMode.csproj</code>.

#### 5) Set the output folder to the repository root (recommended)

If you want the <code>output/</code> folder to appear at the repo root (not under <code>bin/Debug</code>):

1. Right-click a project (Heat2D or PendulumSlidingMode) → <strong>Properties</strong>  
2. Go to <strong>Debug</strong>  
3. Set <strong>Working directory</strong> to:
   <code>$(SolutionDir)</code>

Repeat for both projects.

#### 6) Install FFmpeg (optional)

You can install FFmpeg via winget:

<pre><code>winget install --id Gyan.FFmpeg</code></pre>

Then open a new terminal and verify:

<pre><code>ffmpeg -version</code></pre>

Run <code>PendulumSlidingMode</code> again; MP4 encoding should occur automatically.

---

### macOS / Linux

#### Current repository configuration (important)

Both projects target <code>net8.0-windows</code> and rely on <code>System.Drawing</code> for frame rendering and DPI metadata operations. On non-Windows platforms, <code>System.Drawing.Common</code> is not supported as a general-purpose graphics API in modern .NET and may not work reliably.

You have two practical options:

1) <strong>Run on Windows</strong> (recommended for the repository in its current form), or  
2) <strong>Port the rendering layer</strong> to a cross-platform backend (recommended if you need native macOS/Linux execution).

#### Option A: Run via Windows environment

- Use a Windows machine, a Windows VM, or a CI runner that supports Windows.
- Run with Visual Studio or the <code>dotnet</code> CLI.

#### Option B: Port to cross-platform rendering (high-level plan)

To run natively on macOS/Linux, the typical engineering approach is:

- change <code>TargetFramework</code> to <code>net8.0</code>,
- remove <code>&lt;UseWindowsForms&gt;true&lt;/UseWindowsForms&gt;</code>,
- remove <code>System.Drawing.Common</code>,
- render frames using a cross-platform library (for example):
  - <a href="https://github.com/mono/SkiaSharp" target="_blank">SkiaSharp</a>, or
  - <a href="https://github.com/SixLabors/ImageSharp" target="_blank">SixLabors.ImageSharp</a>.
- keep ScottPlot for plots (ScottPlot v5 is cross-platform), and save images directly to PNG.

Once ported, you can build and run with:

<pre><code>dotnet build -c Release
dotnet run --project ODE_PDE_Csharp/Heat2D -c Release
dotnet run --project ODE_PDE_Csharp/PendulumSlidingMode -c Release</code></pre>

FFmpeg installation (macOS via Homebrew):

<pre><code>brew install ffmpeg</code></pre>

FFmpeg installation (Ubuntu/Debian):

<pre><code>sudo apt-get update
sudo apt-get install -y ffmpeg</code></pre>

---

<a id="troubleshooting"></a>

## Troubleshooting

### I cannot find the <code>output/</code> folder

The output is created relative to the program’s <strong>working directory</strong>. In Visual Studio, the default is often:

<code>.../bin/Debug/net8.0-windows/</code>

Fix: set <strong>Project Properties → Debug → Working directory = $(SolutionDir)</strong>.

### MP4 encoding is slow or appears “stuck”

- Encoding 6000 PNG frames is CPU-intensive, and disk IO can dominate runtime.
- Ensure you are running in <strong>Release</strong> mode.
- Ensure FFmpeg is installed and accessible:
  <pre><code>ffmpeg -version</code></pre>
- The repository’s FFmpeg invocation avoids pipe stalls by not redirecting progress output; you should see ongoing FFmpeg progress lines.

### MP4 not created

- FFmpeg is optional. If it is not installed, the program still saves PNG frames.
- Encode manually (see “Manual MP4 encoding” section).

### Plot images look cropped or labels appear missing

This repository exports plots at high pixel resolutions and then rewrites PNG DPI metadata by cloning pixel data 1:1 (no scaling). If you still see issues:

- verify you are viewing the PNG at “actual size” (some viewers auto-scale),
- try opening the PNG in a different viewer (some applications mishandle DPI metadata),
- increase plot image dimensions (e.g., 3200×2000) for extreme cases.

---

<a id="implementation-tutorial-video"></a>

## Implementation tutorial video

When the repository is running end-to-end, you can watch the full implementation and walkthrough on YouTube.

<!-- Replace YOUR_VIDEO_ID with your uploaded tutorial video ID. -->
<a href="https://www.youtube.com/watch?v=YOUR_VIDEO_ID" target="_blank">
  <img
    src="https://i.ytimg.com/vi/YOUR_VIDEO_ID/maxresdefault.jpg"
    alt="ODE/PDE in C# - Implementation Tutorial"
    style="max-width: 100%; border-radius: 10px; box-shadow: 0 6px 18px rgba(0,0,0,0.18); margin-top: 0.5rem;"
  />
</a>
