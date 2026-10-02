# 3-DOF Helicopter Virtual Lab

Stand-alone browser lab for the Quanser 3-DOF helicopter: enter a controller (PID gains, LQR-I weights, own code or a block diagram), run a test instantly or live in 3-D, and check it against specifications and a linear analysis. Optional AI tutor with the student's own Groq key.

Static site, no build step: `index.html` loads `sim-core.js`, `sim-design.js`, `studio.js` and `tutor-core.js` (plus three.js, Tailwind, KaTeX and Lucide from public CDNs).

Run locally: `python -m http.server 8000` in this folder, then open http://localhost:8000/.

Deployment: Cloudflare Pages, framework preset "None", no build command, output directory `/`.

(c) Afshin Rahimi, University of Windsor. All rights reserved until a licence is added.
