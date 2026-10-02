/*
 * sim-design.js — design-and-check tools for the 3-DOF helicopter virtual lab.
 * Pure functions (no DOM); runs in the browser and in Node, so every number the
 * Design Studio shows can be verified headlessly.
 *
 *   Linear algebra ........ inv, solve, eigvals (Hessenberg + shifted QR), complex solve
 *   Optimal control ....... care() via the matrix sign function, lqr(), lqriGain()
 *   Controllers ........... built-in PID / LQR-I with student parameters,
 *                           student code (sandboxed function), block diagrams
 *   Tests ................. scenarios (reference steps/ramps, disturbances, presets)
 *   Checks ................ per-event metrics, specification pass/fail
 *   Linear analysis ....... closed-loop poles and loop margins of ANY controller,
 *                           by linearising the 1 kHz closed-loop step map
 */
(function (root, factory) {
    const S = (typeof module === 'object' && module.exports) ? require('./sim-core.js') : root.Sim3DOF;
    const api = factory(S);
    if (typeof module === 'object' && module.exports) module.exports = api;
    else root.Sim3DOFDesign = api;
}(typeof self !== 'undefined' ? self : this, function (S) {
    'use strict';
    const DEG = Math.PI / 180;
    const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

    // =========================================================== linear algebra
    const zeros = (n, m) => Array.from({ length: n }, () => new Array(m).fill(0));
    const eye = (n) => { const I = zeros(n, n); for (let i = 0; i < n; i++) I[i][i] = 1; return I; };
    const T = (A) => A[0].map((_, j) => A.map(r => r[j]));
    const mul = (A, B) => A.map(r => B[0].map((_, j) => r.reduce((s, a, k) => s + a * B[k][j], 0)));
    const add = (A, B, b = 1) => A.map((r, i) => r.map((v, j) => v + b * B[i][j]));
    const scal = (A, c) => A.map(r => r.map(v => v * c));
    const copy = (A) => A.map(r => r.slice());

    // Gauss-Jordan with partial pivoting; returns { X, logdet } solving A X = B
    function solveGJ(A, B) {
        const n = A.length, m = B[0].length;
        const M = A.map((r, i) => r.concat(B[i]));
        let logdet = 0, sign = 1;
        for (let c = 0; c < n; c++) {
            let p = c;
            for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
            if (Math.abs(M[p][c]) < 1e-300) throw new Error('singular matrix');
            if (p !== c) { [M[p], M[c]] = [M[c], M[p]]; sign = -sign; }
            const piv = M[c][c]; logdet += Math.log(Math.abs(piv)); if (piv < 0) sign = -sign;
            for (let j = c; j < n + m; j++) M[c][j] /= piv;
            for (let r = 0; r < n; r++) if (r !== c && M[r][c] !== 0) {
                const f = M[r][c];
                for (let j = c; j < n + m; j++) M[r][j] -= f * M[c][j];
            }
        }
        return { X: M.map(r => r.slice(n)), logdet, sign };
    }
    const solve = (A, B) => solveGJ(A, B).X;
    const inv = (A) => solve(A, eye(A.length));

    // Eigenvalues of a real square matrix: Householder reduction to Hessenberg form,
    // then the Francis double-shift QR iteration (after JAMA / EISPACK hqr).
    function eigvals(A0) {
        const nn = A0.length, H = copy(A0);
        const low = 0, high = nn - 1;
        const ort = new Array(nn).fill(0);
        for (let m = low + 1; m <= high - 1; m++) {
            let scale = 0;
            for (let i = m; i <= high; i++) scale += Math.abs(H[i][m - 1]);
            if (scale !== 0) {
                let h = 0;
                for (let i = high; i >= m; i--) { ort[i] = H[i][m - 1] / scale; h += ort[i] * ort[i]; }
                let g = Math.sqrt(h); if (ort[m] > 0) g = -g;
                h -= ort[m] * g; ort[m] -= g;
                for (let j = m; j < nn; j++) {
                    let f = 0; for (let i = high; i >= m; i--) f += ort[i] * H[i][j]; f /= h;
                    for (let i = m; i <= high; i++) H[i][j] -= f * ort[i];
                }
                for (let i = 0; i <= high; i++) {
                    let f = 0; for (let j = high; j >= m; j--) f += ort[j] * H[i][j]; f /= h;
                    for (let j = m; j <= high; j++) H[i][j] -= f * ort[j];
                }
                ort[m] *= scale; H[m][m - 1] = scale * g;
            }
        }
        const d = new Array(nn).fill(0), e = new Array(nn).fill(0);
        let n = nn - 1, exshift = 0, p = 0, q = 0, r = 0, s = 0, z = 0, t, w, x, y;
        const eps = Math.pow(2, -52);
        let norm = 0;
        for (let i = 0; i < nn; i++) for (let j = Math.max(i - 1, 0); j < nn; j++) norm += Math.abs(H[i][j]);
        let iter = 0;
        while (n >= low) {
            let l = n;
            while (l > low) {
                s = Math.abs(H[l - 1][l - 1]) + Math.abs(H[l][l]);
                if (s === 0) s = norm;
                if (Math.abs(H[l][l - 1]) < eps * s) break;
                l--;
            }
            if (l === n) {
                d[n] = H[n][n] + exshift; e[n] = 0; n--; iter = 0;
            } else if (l === n - 1) {
                w = H[n][n - 1] * H[n - 1][n];
                p = (H[n - 1][n - 1] - H[n][n]) / 2;
                q = p * p + w; z = Math.sqrt(Math.abs(q));
                x = H[n][n] + exshift;
                if (q >= 0) {
                    z = p >= 0 ? p + z : p - z;
                    d[n - 1] = x + z; d[n] = d[n - 1]; if (z !== 0) d[n] = x - w / z;
                    e[n - 1] = 0; e[n] = 0;
                } else { d[n - 1] = x + p; d[n] = x + p; e[n - 1] = z; e[n] = -z; }
                n -= 2; iter = 0;
            } else {
                x = H[n][n]; y = 0; w = 0;
                if (l < n) { y = H[n - 1][n - 1]; w = H[n][n - 1] * H[n - 1][n]; }
                if (iter === 10) {
                    exshift += x;
                    for (let i = low; i <= n; i++) H[i][i] -= x;
                    s = Math.abs(H[n][n - 1]) + Math.abs(H[n - 1][n - 2]);
                    x = y = 0.75 * s; w = -0.4375 * s * s;
                }
                if (iter === 30) {
                    s = (y - x) / 2; s = s * s + w;
                    if (s > 0) {
                        s = Math.sqrt(s); if (y < x) s = -s;
                        s = x - w / ((y - x) / 2 + s);
                        for (let i = low; i <= n; i++) H[i][i] -= s;
                        exshift += s; x = y = w = 0.964;
                    }
                }
                if (++iter > 500) throw new Error('eigvals: no convergence');
                let m = n - 2;
                while (m >= l) {
                    z = H[m][m]; r = x - z; s = y - z;
                    p = (r * s - w) / H[m + 1][m] + H[m][m + 1];
                    q = H[m + 1][m + 1] - z - r - s;
                    r = H[m + 2][m + 1];
                    s = Math.abs(p) + Math.abs(q) + Math.abs(r);
                    p /= s; q /= s; r /= s;
                    if (m === l) break;
                    if (Math.abs(H[m][m - 1]) * (Math.abs(q) + Math.abs(r)) <
                        eps * (Math.abs(p) * (Math.abs(H[m - 1][m - 1]) + Math.abs(z) + Math.abs(H[m + 1][m + 1])))) break;
                    m--;
                }
                for (let i = m + 2; i <= n; i++) { H[i][i - 2] = 0; if (i > m + 2) H[i][i - 3] = 0; }
                for (let k = m; k <= n - 1; k++) {
                    const notlast = (k !== n - 1);
                    if (k !== m) {
                        p = H[k][k - 1]; q = H[k + 1][k - 1]; r = notlast ? H[k + 2][k - 1] : 0;
                        x = Math.abs(p) + Math.abs(q) + Math.abs(r);
                        if (x === 0) continue;
                        p /= x; q /= x; r /= x;
                    }
                    s = Math.sqrt(p * p + q * q + r * r); if (p < 0) s = -s;
                    if (s !== 0) {
                        if (k !== m) H[k][k - 1] = -s * x;
                        else if (l !== m) H[k][k - 1] = -H[k][k - 1];
                        p += s; x = p / s; y = q / s; z = r / s; q /= p; r /= p;
                        for (let j = k; j < nn; j++) {
                            p = H[k][j] + q * H[k + 1][j];
                            if (notlast) { p += r * H[k + 2][j]; H[k + 2][j] -= p * z; }
                            H[k][j] -= p * x; H[k + 1][j] -= p * y;
                        }
                        for (let i = 0; i <= Math.min(n, k + 3); i++) {
                            p = x * H[i][k] + y * H[i][k + 1];
                            if (notlast) { p += z * H[i][k + 2]; H[i][k + 2] -= p * r; }
                            H[i][k] -= p; H[i][k + 1] -= p * q;
                        }
                    }
                }
            }
        }
        return d.map((re, i) => ({ re, im: e[i] }));
    }

    // Complex helpers (numbers as [re, im]) and complex linear solve
    const cadd = (a, b) => [a[0] + b[0], a[1] + b[1]];
    const csub = (a, b) => [a[0] - b[0], a[1] - b[1]];
    const cmul = (a, b) => [a[0] * b[0] - a[1] * b[1], a[0] * b[1] + a[1] * b[0]];
    const cdiv = (a, b) => { const d = b[0] * b[0] + b[1] * b[1]; return [(a[0] * b[0] + a[1] * b[1]) / d, (a[1] * b[0] - a[0] * b[1]) / d]; };
    const cabs = (a) => Math.hypot(a[0], a[1]);
    function csolve(A, B) {   // A: n x n complex, B: n x m complex
        const n = A.length, m = B[0].length;
        const M = A.map((r, i) => r.concat(B[i]).map(c => c.slice()));
        for (let c = 0; c < n; c++) {
            let p = c;
            for (let r = c + 1; r < n; r++) if (cabs(M[r][c]) > cabs(M[p][c])) p = r;
            [M[p], M[c]] = [M[c], M[p]];
            const piv = M[c][c];
            for (let j = c; j < n + m; j++) M[c][j] = cdiv(M[c][j], piv);
            for (let r = 0; r < n; r++) if (r !== c) {
                const f = M[r][c];
                if (f[0] === 0 && f[1] === 0) continue;
                for (let j = c; j < n + m; j++) M[r][j] = csub(M[r][j], cmul(f, M[c][j]));
            }
        }
        return M.map(r => r.slice(n));
    }

    // =========================================================== optimal control
    // Stabilising solution of A'P + PA - P B R^-1 B' P + Q = 0 by the matrix sign
    // function of the Hamiltonian, with determinant scaling (Roberts; Byers).
    function care(A, B, Q, R) {
        const n = A.length;
        const G = mul(mul(B, inv(R)), T(B));
        let Z = zeros(2 * n, 2 * n);
        for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
            Z[i][j] = A[i][j]; Z[i][j + n] = -G[i][j];
            Z[i + n][j] = -Q[i][j]; Z[i + n][j + n] = -A[j][i];
        }
        for (let it = 0; it < 100; it++) {
            const { X: Zi, logdet } = solveGJ(Z, eye(2 * n));
            const c = Math.exp(-logdet / (2 * n));
            const Zn = add(scal(Z, 0.5 * c), scal(Zi, 0.5 / c));
            let diff = 0, nz = 0;
            for (let i = 0; i < 2 * n; i++) for (let j = 0; j < 2 * n; j++) { diff += Math.abs(Zn[i][j] - Z[i][j]); nz += Math.abs(Zn[i][j]); }
            Z = Zn;
            if (diff <= 1e-13 * nz) break;
        }
        // [Z12; Z22 + I] P = -[Z11 + I; Z21]  (least squares via normal equations)
        const Mt = zeros(2 * n, n), Nt = zeros(2 * n, n);
        for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
            Mt[i][j] = Z[i][j + n]; Mt[i + n][j] = Z[i + n][j + n] + (i === j ? 1 : 0);
            Nt[i][j] = -(Z[i][j] + (i === j ? 1 : 0)); Nt[i + n][j] = -Z[i + n][j];
        }
        let P = solve(mul(T(Mt), Mt), mul(T(Mt), Nt));
        P = scal(add(P, T(P)), 0.5);
        // polish with Newton-Kleinman steps: (A-GP)'X + X(A-GP) = -(Q + P G P)
        for (let it = 0; it < 3; it++) {
            const Acl = add(A, mul(G, P), -1);
            const rhs = add(Q, mul(mul(P, G), P));
            const Xn = lyap(Acl, rhs);
            if (!Xn) break;
            P = scal(add(Xn, T(Xn)), 0.5);
        }
        return P;
    }
    // Solves Ac' X + X Ac + W = 0 via the Kronecker form (small n only)
    function lyap(Ac, W) {
        const n = Ac.length, N = n * n, M = zeros(N, N), b = zeros(N, 1);
        for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
            const r = i * n + j; b[r][0] = -W[i][j];
            for (let k = 0; k < n; k++) { M[r][k * n + j] += Ac[k][i]; M[r][i * n + k] += Ac[k][j]; }
        }
        try { const x = solve(M, b); const X = zeros(n, n); for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) X[i][j] = x[i * n + j][0]; return X; }
        catch (e) { return null; }
    }
    function lqr(A, B, Q, R) {
        const P = care(A, B, Q, R);
        return { K: mul(inv(R), mul(T(B), P)), P };
    }
    // Augmented model used by the LQR-I law: z = [x; int(theta-theta_r); int(psi-psi_r)]
    function augmentedModel(P) {
        const Vop = S.hoverVoltage(P);
        const { A, B } = S.linearizeFD(P, [0, 0, 0, 0, 0, 0], [Vop, Vop]);
        const Aa = zeros(8, 8), Ba = zeros(8, 2);
        for (let i = 0; i < 6; i++) { for (let j = 0; j < 6; j++) Aa[i][j] = A[i][j]; Ba[i] = B[i].slice(); }
        Aa[6][0] = 1; Aa[7][2] = 1;
        return { A: Aa, B: Ba, Vop };
    }
    const diag = (v) => { const D = zeros(v.length, v.length); v.forEach((x, i) => D[i][i] = x); return D; };
    function lqriGain(P, qdiag, rdiag) {
        const { A, B } = augmentedModel(P);
        return lqr(A, B, diag(qdiag), diag(rdiag)).K;
    }

    // =========================================================== student controllers
    // Expression fields accept numbers or small expressions such as "20*deg" or "pi/9".
    function evalExpr(v) {
        if (typeof v === 'number') return v;
        const sTxt = String(v).trim();
        if (!/^[0-9eE+\-*/().\s,a-z_]*$/.test(sTxt)) throw new Error('invalid expression: ' + sTxt);
        const f = new Function('deg', 'pi', 'sqrt', 'exp', '"use strict"; return (' + sTxt + ');');
        const r = f(DEG, Math.PI, Math.sqrt, Math.exp);
        if (!Number.isFinite(r)) throw new Error('expression is not a finite number: ' + sTxt);
        return r;
    }

    function builtinPID(P, g) {
        const gg = {};
        for (const k of Object.keys(g)) gg[k] = evalExpr(g[k]);
        const c = S.makePID(P, gg); c.kind = 'pid'; return c;
    }
    function builtinLQRI(P, spec) {
        let K;
        if (spec.mode === 'K') K = spec.K.map(r => r.map(evalExpr));
        else K = lqriGain(P, spec.Q.map(evalExpr), spec.R.map(evalExpr));
        const c = S.makeLQRI(P, K, { e: evalExpr(spec.int_lim_e ?? 1), t: evalExpr(spec.int_lim_t ?? 2) });
        c.kind = 'lqri'; c.K = K; return c;
    }

    // Helpers available to student code as `lib`
    function makeLib(P) {
        return {
            deg: DEG, clamp, wrap: S.wrap, hoverVoltage: (th, ph) => S.hoverVoltage(P, th || 0, ph || 0),
            lqr, lqriGain: (q, r) => lqriGain(P, q, r), linearize: () => augmentedModel(P), eigvals,
            mat: { mul, add, T, inv, solve, eye, zeros, diag }
        };
    }
    // Flatten / restore the numeric leaves of a plain object (controller memory)
    function flatten(o, out = []) {
        if (typeof o === 'number') out.push(o);
        else if (Array.isArray(o)) o.forEach(v => flatten(v, out));
        else if (o && typeof o === 'object') Object.keys(o).sort().forEach(k => flatten(o[k], out));
        return out;
    }
    function unflatten(o, vals, idx = { i: 0 }) {
        if (Array.isArray(o)) { for (let k = 0; k < o.length; k++) { if (typeof o[k] === 'number') o[k] = vals[idx.i++]; else unflatten(o[k], vals, idx); } }
        else if (o && typeof o === 'object') { for (const k of Object.keys(o).sort()) { if (typeof o[k] === 'number') o[k] = vals[idx.i++]; else unflatten(o[k], vals, idx); } }
        return o;
    }

    /* Student code: defines
         function init(P, lib) { return { ...memory... }; }        (optional)
         function step(y, ref, dt, mem, P, lib) { return [Vf, Vb]; }
       y = [theta, phi, psi, dtheta, dphi, dpsi] (rad, rad/s), ref = {theta, psi} (rad). */
    function compileCode(src, P) {
        let mod;
        try {
            mod = new Function('"use strict";\n' + src + '\n;return { init: (typeof init === "function") ? init : null, step: (typeof step === "function") ? step : null };')();
        } catch (e) { throw new Error('Syntax error: ' + e.message); }
        if (!mod.step) throw new Error('The code must define function step(y, ref, dt, mem, P, lib).');
        const lib = makeLib(P);
        let mem = {};
        const c = {
            name: 'Student code', kind: 'code',
            reset() { mem = mod.init ? (mod.init(P, lib) || {}) : {}; },
            getState() { return flatten(mem); },
            setState(v) { unflatten(mem, v); },
            step(y, ref, dt) {
                const u = mod.step(y.slice(), { theta: ref.theta, psi: ref.psi }, dt, mem, P, lib);
                if (!Array.isArray(u) || u.length !== 2 || !Number.isFinite(u[0]) || !Number.isFinite(u[1]))
                    throw new Error('step() must return [Vf, Vb] as two finite numbers; got ' + JSON.stringify(u));
                return u;
            }
        };
        c.reset();
        return c;
    }

    // ---------------------------------------------------------------- block diagrams
    const SIGNALS = ['theta', 'phi', 'psi', 'dtheta', 'dphi', 'dpsi', 'theta_ref', 'psi_ref', 'Vop', 'time'];
    const BLOCK_TYPES = {
        source:     { nin: 0, nout: 1, params: { signal: 'theta' }, label: b => b.params.signal },
        const:      { nin: 0, nout: 1, params: { value: 0 }, label: b => String(b.params.value) },
        gain:       { nin: 1, nout: 1, params: { k: 1 }, label: b => '× ' + b.params.k },
        sum:        { nin: null, nout: 1, params: { signs: '+-' }, label: b => 'Σ ' + b.params.signs },
        product:    { nin: 2, nout: 1, params: {}, label: () => '×' },
        integrator: { nin: 1, nout: 1, params: { lim: 1, x0: 0 }, label: () => '∫' },
        derivative: { nin: 1, nout: 1, params: { wc: 50 }, label: b => 'd/dt (' + b.params.wc + ')' },
        saturation: { nin: 1, nout: 1, params: { lo: '-20*deg', hi: '20*deg' }, label: b => 'sat' },
        wrap:       { nin: 1, nout: 1, params: {}, label: () => 'wrap ±π' },
        mixer:      { nin: 2, nout: 2, params: {}, label: () => 'mixer' },
        output:     { nin: 1, nout: 0, params: { channel: 'Vf' }, label: b => b.params.channel }
    };
    const nIn = (b) => b.type === 'sum' ? String(b.params.signs).length : BLOCK_TYPES[b.type].nin;
    const nOut = (b) => BLOCK_TYPES[b.type].nout;

    function compileDiagram(diag, P) {
        const blocks = diag.blocks, byId = {};
        const errors = [];
        for (const b of blocks) {
            if (!BLOCK_TYPES[b.type]) errors.push(`Block ${b.id}: unknown type ${b.type}`);
            byId[b.id] = b;
        }
        const inputOf = {};   // `${id}:${port}` -> {id, port}
        for (const w of diag.wires) {
            const k = w.to.id + ':' + w.to.port;
            if (!byId[w.from.id] || !byId[w.to.id]) { errors.push('Wire to a missing block'); continue; }
            if (inputOf[k]) errors.push(`Block ${w.to.id} input ${w.to.port + 1} has two wires`);
            inputOf[k] = w.from;
        }
        for (const b of blocks) for (let p = 0; p < nIn(b); p++)
            if (!inputOf[b.id + ':' + p]) errors.push(`Block ${b.id} (${b.type}) input ${p + 1} is not connected`);
        const outs = blocks.filter(b => b.type === 'output');
        for (const ch of ['Vf', 'Vb']) {
            const n = outs.filter(b => b.params.channel === ch).length;
            if (n !== 1) errors.push(`The diagram needs exactly one ${ch} output block (found ${n})`);
        }
        const par = {};
        for (const b of blocks) {
            par[b.id] = {};
            for (const [k, v] of Object.entries(b.params)) {
                if (k === 'signal' || k === 'channel' || k === 'signs') { par[b.id][k] = v; continue; }
                try { par[b.id][k] = evalExpr(v); } catch (e) { errors.push(`Block ${b.id}: ${e.message}`); }
            }
            if (b.type === 'source' && !SIGNALS.includes(b.params.signal)) errors.push(`Block ${b.id}: unknown signal ${b.params.signal}`);
            if (b.type === 'sum' && !/^[+-]+$/.test(b.params.signs)) errors.push(`Block ${b.id}: signs must be a string of + and -`);
        }
        // Evaluation order: integrator outputs are states, so wires into integrators do not
        // create dependencies. Anything left over is an algebraic loop.
        const deps = {}; for (const b of blocks) deps[b.id] = new Set();
        for (const b of blocks) if (b.type !== 'integrator')
            for (let p = 0; p < nIn(b); p++) { const src = inputOf[b.id + ':' + p]; if (src) deps[b.id].add(src.id); }
        const order = [], done = new Set();
        let progress = true;
        while (order.length < blocks.length && progress) {
            progress = false;
            for (const b of blocks) if (!done.has(b.id) && [...deps[b.id]].every(d => done.has(d))) { order.push(b); done.add(b.id); progress = true; }
        }
        if (order.length < blocks.length) errors.push('Algebraic loop through blocks: ' + blocks.filter(b => !done.has(b.id)).map(b => b.id).join(', ') + ' (put an integrator in the loop)');
        if (errors.length) { const e = new Error(errors.join('\n')); e.list = errors; throw e; }

        // numeric slots for every output port, and per-block input slot lists
        const slot = {}; let ns = 0;
        for (const b of blocks) for (let p = 0; p < nOut(b); p++) slot[b.id + ':' + p] = ns++;
        const prog = order.map(b => {
            const ins = []; for (let p = 0; p < nIn(b); p++) { const s0 = inputOf[b.id + ':' + p]; ins.push(slot[s0.id + ':' + s0.port]); }
            const outs = []; for (let p = 0; p < nOut(b); p++) outs.push(slot[b.id + ':' + p]);
            const sg = b.type === 'sum' ? Array.from(par[b.id].signs).map(ch => ch === '-' ? -1 : 1) : null;
            return { b, type: b.type, p: par[b.id], ins, outs, sg, sigIdx: b.type === 'source' ? SIGNALS.indexOf(b.params.signal) : -1, ch: b.params.channel };
        });
        const integ = prog.filter(q => q.type === 'integrator'), derivB = prog.filter(q => q.type === 'derivative');
        const st = {};   // per-block state (kept as an object so it can be flattened for linear analysis)
        const reset = () => {
            for (const q of integ) st[q.b.id] = { v: q.p.x0 };
            for (const q of derivB) st[q.b.id] = { u: 0, v: 0, init: 0 };
        };
        reset();
        let tNow = 0;
        const val = new Float64Array(ns), sig = new Float64Array(SIGNALS.length);
        const c = {
            name: diag.name || 'Block diagram', kind: 'diagram',
            reset() { reset(); tNow = 0; },
            getState() { return flatten(st); },
            setState(v) { unflatten(st, v); },
            step(y, ref, dt) {
                sig[0] = y[0]; sig[1] = y[1]; sig[2] = y[2]; sig[3] = y[3]; sig[4] = y[4]; sig[5] = y[5];
                sig[6] = ref.theta; sig[7] = ref.psi; sig[8] = S.hoverVoltage(P, y[0], y[1]); sig[9] = tNow;
                let Vf = NaN, Vb = NaN;
                for (const q of prog) {
                    const I = q.ins, O = q.outs, p = q.p;
                    switch (q.type) {
                        case 'source': val[O[0]] = sig[q.sigIdx]; break;
                        case 'const': val[O[0]] = p.value; break;
                        case 'gain': val[O[0]] = p.k * val[I[0]]; break;
                        case 'sum': { let s0 = 0; for (let i = 0; i < I.length; i++) s0 += q.sg[i] * val[I[i]]; val[O[0]] = s0; break; }
                        case 'product': val[O[0]] = val[I[0]] * val[I[1]]; break;
                        case 'integrator': val[O[0]] = st[q.b.id].v; break;
                        case 'derivative': {
                            const u = val[I[0]], s0 = st[q.b.id];
                            if (s0.init) { const a = Math.exp(-p.wc * dt); s0.v = a * s0.v + (1 - a) * (u - s0.u) / dt; }
                            s0.u = u; s0.init = 1; val[O[0]] = s0.v; break;
                        }
                        case 'saturation': val[O[0]] = clamp(val[I[0]], p.lo, p.hi); break;
                        case 'wrap': val[O[0]] = S.wrap(val[I[0]]); break;
                        case 'mixer': { const vs = val[I[0]], vd = val[I[1]]; val[O[0]] = (vs + vd) / 2; val[O[1]] = (vs - vd) / 2; break; }
                        case 'output': if (q.ch === 'Vf') Vf = val[I[0]]; else Vb = val[I[0]]; break;
                    }
                }
                // integrator states advance after the outputs are formed (forward Euler)
                for (const q of integ) { const l = q.p.lim; st[q.b.id].v = clamp(st[q.b.id].v + val[q.ins[0]] * dt, -l, l); }
                tNow += dt;
                return [Vf, Vb];
            }
        };
        return c;
    }

    // =========================================================== scenarios
    /* scenario = { name, T, theta0 (deg), preset: 'nominal'|'identified', hw: bool,
                    refs: [{ t, theta (deg), psi (deg), ramp (s, 0 = step) }],
                    dists: [{ t, dur, axis: 'theta'|'phi'|'psi', tau (N m) }],
                    faults: [see Sim3DOF.makeFaults] } */
    function scenarioFns(sc) {
        const refs = sc.refs.slice().sort((a, b) => a.t - b.t);
        const ref = (t) => {
            let prev = { theta: refs[0].theta, psi: refs[0].psi }, cur = prev;
            for (let i = 0; i < refs.length; i++) {
                const r = refs[i];
                if (t + 1e-12 < r.t) break;
                const ramp = r.ramp || 0;
                prev = cur;
                if (ramp > 0 && t < r.t + ramp) {
                    const a = (t - r.t) / ramp;
                    cur = { theta: prev.theta + a * (r.theta - prev.theta), psi: prev.psi + a * (r.psi - prev.psi) };
                } else cur = { theta: r.theta, psi: r.psi };
            }
            return { theta: cur.theta * DEG, psi: cur.psi * DEG };
        };
        const ax = { theta: 0, phi: 1, psi: 2 };
        const dist = (sc.dists && sc.dists.length) ? (t) => {
            const d = [0, 0, 0];
            for (const q of sc.dists) if (t + 1e-12 >= q.t && t < q.t + q.dur) d[ax[q.axis]] += q.tau;
            return d;
        } : null;
        return { ref, dist };
    }

    const SCENARIOS = {
        takeoff_steps: { name: 'Take-off, elevation then travel step', T: 60, theta0: -27.5, preset: 'nominal', hw: false,
            refs: [{ t: 0, theta: -27.5, psi: 0 }, { t: 1, theta: 0, psi: 0 }, { t: 15, theta: 10, psi: 0 }, { t: 30, theta: 10, psi: 30 }], dists: [] },
        verification: { name: 'Paper verification sequence', T: 60, theta0: 0, preset: 'nominal', hw: false,
            refs: [{ t: 0, theta: 0, psi: 0 }, { t: 1, theta: 10, psi: 0 }, { t: 10, theta: 10, psi: 20 }, { t: 40, theta: -10, psi: 0 }], dists: [] },
        big_travel: { name: 'Large travel step (90°)', T: 30, theta0: 0, preset: 'nominal', hw: false,
            refs: [{ t: 0, theta: 0, psi: 0 }, { t: 1, theta: 0, psi: 90 }], dists: [] },
        ramp_tracking: { name: 'Travel ramp tracking', T: 50, theta0: 0, preset: 'nominal', hw: false,
            refs: [{ t: 0, theta: 0, psi: 0 }, { t: 2, theta: 5, psi: 0, ramp: 3 }, { t: 8, theta: 5, psi: 120, ramp: 25 }], dists: [] },
        disturbance: { name: 'Disturbance rejection (elevation and travel)', T: 40, theta0: 0, preset: 'nominal', hw: false,
            refs: [{ t: 0, theta: 5, psi: 0 }],
            dists: [{ t: 10, dur: 0.5, axis: 'theta', tau: -0.3 }, { t: 25, dur: 0.5, axis: 'psi', tau: 0.1 }] },
        rotor_fault: { name: 'Front rotor fault (30 %, abrupt)', T: 50, theta0: 0, preset: 'nominal', hw: false,
            refs: [{ t: 0, theta: 5, psi: 0 }, { t: 10, theta: 5, psi: 20 }, { t: 35, theta: 5, psi: 0 }], dists: [],
            faults: [{ type: 'rotor', target: 'front', t: 25, size: 0.3, ramp: 0 }] },
        degradation: { name: 'Gradual thrust loss (accelerated ageing)', T: 120, theta0: 0, preset: 'nominal', hw: false,
            refs: [{ t: 0, theta: 5, psi: 0 }, { t: 20, theta: 5, psi: 20 }, { t: 50, theta: 5, psi: 0 }, { t: 80, theta: 5, psi: 20 }], dists: [],
            faults: [{ type: 'rotor', target: 'both', t: 10, size: 0.5, ramp: 100 }] }
    };

    // =========================================================== running and checking
    function runScenario(controller, sc, opts = {}) {
        const P = Object.assign({}, S.PRESETS[sc.preset || 'nominal']);
        const { ref, dist } = scenarioFns(sc);
        const hw = sc.hw ? { encoders: true, motorTau: 0.05 } : {};
        let error = null, tFail = null, calls = 0;
        const safe = {
            reset() { controller.reset(); calls = 0; },
            step(y, r, h) {
                const tk = (calls++) * h;
                if (error) return [0, 0];
                try { return controller.step(y, r, h); }
                catch (e) { error = e.message; tFail = tk; return [0, 0]; }
            }
        };
        const x0 = [(sc.theta0 ?? 0) * DEG, 0, 0, 0, 0, 0];
        const t0 = (typeof performance !== 'undefined' ? performance : Date).now();
        const monitor = opts.monitor === false ? null : makeHealthMonitor(sc, opts.healthCfg);
        const log = S.simulate({ P, T: sc.T, h: 1e-3, x0, controller: safe, ref, dist, hw, faults: sc.faults, monitor, logEvery: opts.logEvery || 10 });
        const wall = (typeof performance !== 'undefined' ? performance : Date).now() - t0;
        const run = { scenario: JSON.parse(JSON.stringify(sc)), controllerName: controller.name, kind: controller.kind,
                      t: log.t, x: log.x, V: log.V, ref: log.ref, error, tFail, wallMs: wall, P };
        if (log.y) Object.assign(run, { y: log.y, eta: log.eta, r: log.r, s: log.s, mlive: log.mlive, g: log.g, alarm: log.alarm });
        return run;
    }

    // =========================================================== health monitoring (diagnosis and prognosis)
    const HEALTH_DEFAULTS = { thrScale: 1, dwell: 0.25, KO: 5, KOhw: 1, tauS: 0.1, tauShw: 0.3, eolEta: 0.6, rulWindow: 20 };
    // The monitor always uses the NOMINAL model (it does not know which plant set is simulated),
    // with the actuator lag it assumes when hardware effects are on.
    function makeHealthMonitor(sc, cfg = {}) {
        const c = Object.assign({}, HEALTH_DEFAULTS, cfg);
        const hw = !!(sc && sc.hw);
        // with encoder quantisation and filtered rates, a slower observer keeps the healthy residual below threshold
        return S.makeMonitor(S.NOMINAL, { KO: hw ? c.KOhw : c.KO, tauS: hw ? c.tauShw : c.tauS, dwell: c.dwell, thr: [0.02, 0.005, 0.01].map(v => v * c.thrScale * (hw ? 1.5 : 1)),
                                          motorTau: hw ? 0.05 : 0, rateFilter: hw ? 50 : 0 });
    }

    /* Post-processing of a run's monitor log:
       detection (first alarm), isolation (least squares of the filtered residual torques on the
       fault signatures of each rotor and of travel friction), size estimates, a running thrust-health
       estimate eta_hat(t) and a remaining-useful-life prediction from its trend. */
    function healthAnalysis(run, cfg = {}) {
        if (!run.r || !run.t.length) return null;
        const c = Object.assign({}, HEALTH_DEFAULTS, cfg);
        const t = run.t, n = t.length, dt = n > 1 ? t[1] - t[0] : 0.01, KO = run.scenario.hw ? c.KOhw : c.KO;
        const faults = (run.scenario.faults || []).filter(f => f && f.type);
        const tFault = faults.length ? Math.min(...faults.map(f => f.t)) : null;
        // regressors filtered with the observer dynamics so that they line up with r
        const a = Math.exp(-KO * dt);
        const G = []; let gf = null;
        for (let k = 0; k < n; k++) {
            const g = run.g[k];
            if (!run.mlive[k] || !g) { gf = null; G.push(null); continue; }
            const flat = [g[0][0], g[0][1], g[1][0], g[1][1], g[2][0], g[2][1], run.y[k][5]];
            if (!gf) gf = flat.map(() => 0);
            gf = gf.map((v, i) => a * v + (1 - a) * flat[i]);
            G.push(gf.slice());
        }
        // least squares over a window: r = [g_th; g_ph; g_ps] * [df, db] + [0; 0; -psidot] * dD
        function fitWindow(k0, k1) {
            const AtA = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], Atb = [0, 0, 0]; let rr = 0, m = 0;
            const sc = [1 / 0.02, 1 / 0.005, 1 / 0.01];   // weight axes by their nominal threshold
            for (let k = k0; k <= k1; k++) {
                const g = G[k]; if (!g) continue;
                const rows = [[g[0], g[1], 0], [g[2], g[3], 0], [g[4], g[5], -g[6]]];
                for (let i = 0; i < 3; i++) {
                    const w = sc[i], row = rows[i].map(v => v * w), b = run.r[k][i] * w;
                    for (let p = 0; p < 3; p++) { Atb[p] += row[p] * b; for (let q = 0; q < 3; q++) AtA[p][q] += row[p] * row[q]; }
                    rr += b * b; m++;
                }
            }
            if (m < 30) return null;
            for (let p = 0; p < 3; p++) AtA[p][p] += 1e-9;
            const th = solve(AtA, Atb.map(v => [v])).map(v => v[0]);
            let res = 0;
            for (let k = k0; k <= k1; k++) {
                const g = G[k]; if (!g) continue;
                const rows = [[g[0], g[1], 0], [g[2], g[3], 0], [g[4], g[5], -g[6]]];
                for (let i = 0; i < 3; i++) { const e = (run.r[k][i] - rows[i].reduce((s2, v, q) => s2 + v * th[q], 0)) * sc[i]; res += e * e; }
            }
            return { dF: th[0], dB: th[1], dD: th[2], fit: rr > 0 ? 1 - res / rr : 1 };
        }
        const alarm = run.alarm ?? null;
        const out = { tFault, alarm, detected: alarm !== null, falseAlarm: alarm !== null && (tFault === null || alarm < tFault - 1e-9),
                      delay: (alarm !== null && tFault !== null && alarm >= tFault) ? alarm - tFault : null,
                      isolatedAs: null, estimate: null, rul: null };
        if (alarm !== null) {
            const winS = Math.max(2, 4 / KO);   // slower observer -> longer window
            const k0 = t.findIndex(v => v >= alarm), k1 = Math.min(n - 1, k0 + Math.round(winS / dt));
            const fit = fitWindow(k0, k1);
            if (fit) {
                out.estimate = fit;
                const big = Math.max(Math.abs(fit.dF), Math.abs(fit.dB));
                if (fit.fit < 0.5) out.isolatedAs = 'unexplained (sensor fault or unmodelled effect)';
                else if (big >= 0.02) {
                    const ratio = Math.min(Math.abs(fit.dF), Math.abs(fit.dB)) / big;
                    out.isolatedAs = (ratio > 0.6 && fit.dF * fit.dB > 0) ? 'both rotors' : (Math.abs(fit.dF) > Math.abs(fit.dB) ? 'front rotor' : 'back rotor');
                } else if (fit.dD > 0.005) out.isolatedAs = 'travel friction';
                else out.isolatedAs = 'unexplained (sensor fault or unmodelled effect)';
            }
        }
        // running thrust-health estimate over a sliding window and a linear-trend RUL prediction
        const W = Math.round(Math.max(2, 4 / KO) / dt), etaHat = new Array(n).fill(null);
        // each window estimate describes the thrust about half a window plus the observer lag earlier
        const lag = (W * dt) / 2 + 1 / KO;
        for (let k = W; k < n; k += 5) {
            const f = fitWindow(k - W, k); if (!f) continue;
            etaHat[k] = 1 - (f.dF + f.dB) / 2;
        }
        const rul = [];
        const Wr = c.rulWindow;
        for (let k = 0; k < n; k += 50) {
            const pts = []; for (let q = 0; q <= k; q++) if (etaHat[q] !== null && t[q] - lag >= t[k] - Wr) pts.push([t[q] - lag, etaHat[q]]);
            if (pts.length < 10 || pts[pts.length - 1][0] - pts[0][0] < 0.5 * Wr) continue;   // need half a window of history
            const mt = pts.reduce((s2, p) => s2 + p[0], 0) / pts.length, me = pts.reduce((s2, p) => s2 + p[1], 0) / pts.length;
            let sxx = 0, sxy = 0; for (const p of pts) { sxx += (p[0] - mt) ** 2; sxy += (p[0] - mt) * (p[1] - me); }
            const slope = sxy / sxx, now = me + slope * (t[k] - mt);
            let sse = 0; for (const p of pts) sse += (p[1] - me - slope * (p[0] - mt)) ** 2;
            const sd = Math.sqrt(sse / pts.length);
            // predict only for a steady downward trend (an abrupt drop is a fault, not wear)
            const tstat = -slope / (sd / Math.sqrt(sxx) + 1e-12);   // significance of the downward slope
            const pred = (slope < -5e-4 && sd < 0.01 && tstat > 20) ? Math.max(0, (now - c.eolEta) / -slope) : null;
            rul.push({ t: t[k], eta: now, slope, rul: pred });
        }
        // true end of life for a ramped thrust loss, if any
        let tEol = null;
        for (let k = 0; k < n; k++) { const e = run.eta[k]; if (Math.min(e[0], e[1]) <= c.eolEta + 1e-9) { tEol = t[k]; break; } }
        out.etaHat = etaHat; out.lag = lag; out.rulTrace = rul; out.tEol = tEol; out.eolEta = c.eolEta;
        // prognosis summary: a sustained trend is needed; with a known end of life, the alpha = 10 % prognostic horizon
        const preds = rul.filter(q => q.rul !== null);
        out.prog = { n: preds.length, sustained: preds.length >= 5, tAccurate: null, horizon: null, last: preds.length ? preds[preds.length - 1] : null };
        if (out.prog.sustained && tEol !== null) {
            const before = preds.filter(q => q.t < tEol);
            for (let i = 0; i < before.length; i++) {
                if (before.slice(i).every(q => Math.abs(q.rul - (tEol - q.t)) <= 0.1 * (tEol - q.t) + 0.5)) { out.prog.tAccurate = before[i].t; out.prog.horizon = tEol - before[i].t; break; }
            }
        }
        out.maxStatBeforeFault = Math.max(0, ...run.s.filter((_, k) => tFault === null || t[k] < tFault));
        return out;
    }

    // Metrics per reference event (one per axis change) and per disturbance, plus run-level measures.
    function computeMetrics(run, cfg = {}) {
        const band = (cfg.settleBandPct ?? 2) / 100, floorDeg = cfg.settleFloorDeg ?? 0.2;
        const sc = run.scenario, t = run.t, x = run.x, V = run.V;
        const refs = sc.refs.slice().sort((a, b) => a.t - b.t);
        const events = [];
        for (const axis of ['theta', 'psi']) {
            const idx = axis === 'theta' ? 0 : 2;
            const changes = [];
            for (let i = 1; i < refs.length; i++) if (refs[i][axis] !== refs[i - 1][axis]) changes.push({ t: refs[i].t, from: refs[i - 1][axis], to: refs[i][axis], ramp: refs[i].ramp || 0 });
            changes.forEach((c, j) => {
                const tEnd = j + 1 < changes.length ? changes[j + 1].t : sc.T;
                const ii = []; for (let k = 0; k < t.length; k++) if (t[k] >= c.t - 1e-9 && t[k] < tEnd - 1e-9) ii.push(k);
                if (ii.length < 3) return;
                const span = c.to - c.from;
                const yv = ii.map(k => x[k][idx] / DEG);
                const n = yv.map(v => (v - c.from) / span);
                const peak = Math.max(...n);
                const tol = Math.max(band, floorDeg / Math.abs(span));
                let ts = null; for (let q = n.length - 1; q >= 0; q--) if (Math.abs(n[q] - 1) > tol) { ts = q + 1 < n.length ? t[ii[q + 1]] - c.t : null; break; }
                if (ts === null && Math.abs(n[n.length - 1] - 1) <= tol && n.every(v => Math.abs(v - 1) <= tol)) ts = 0;
                const i10 = n.findIndex(v => v >= 0.1), i90 = n.findIndex(v => v >= 0.9);
                const lastS = Math.min(1.0, (tEnd - c.t) / 4);
                const tail = ii.filter(k => t[k] >= tEnd - lastS - 1e-9).map(k => Math.abs(x[k][idx] / DEG - c.to));
                let iae = 0; for (let q = 1; q < ii.length; q++) iae += Math.abs(run.ref[ii[q]][idx === 0 ? 0 : 1] / DEG - yv[q]) * (t[ii[q]] - t[ii[q - 1]]);
                events.push({ axis, type: c.ramp > 0 ? 'ramp' : 'step', t: c.t, from: c.from, to: c.to,
                    overshootPct: c.ramp > 0 ? null : Math.max(0, (peak - 1) * 100),
                    riseTime: (c.ramp > 0 || i10 < 0 || i90 < 0) ? null : t[ii[i90]] - t[ii[i10]],
                    settlingTime: ts, steadyStateErrDeg: tail.reduce((a, b) => a + b, 0) / Math.max(1, tail.length), iaeDegS: iae });
            });
        }
        for (const q of (sc.dists || [])) {
            const idx = { theta: 0, phi: 1, psi: 2 }[q.axis];
            const k0 = t.findIndex(v => v >= q.t - 1e-9); if (k0 < 0) continue;
            const base = x[k0][idx];
            let dev = 0, krec = null;
            for (let k = k0; k < t.length; k++) { const dv = Math.abs(x[k][idx] - base) / DEG; if (dv > dev) dev = dv; }
            for (let k = t.length - 1; k >= k0; k--) if (Math.abs(x[k][idx] - base) / DEG > Math.max(floorDeg, 0.1 * dev)) { krec = k; break; }
            events.push({ axis: q.axis, type: 'disturbance', t: q.t, maxDeviationDeg: dev,
                recoveryTime: krec === null ? 0 : (krec + 1 < t.length ? t[krec + 1] - q.t : null) });
        }
        const P = run.P;
        let vmax = 0, sat = 0, pitchStop = 0, upperStop = 0, lowerStop = 0, left = false;
        for (let k = 0; k < t.length; k++) {
            vmax = Math.max(vmax, Math.abs(V[k][0]), Math.abs(V[k][1]));
            if (Math.abs(V[k][0]) >= P.Vmax - 1e-9 || Math.abs(V[k][1]) >= P.Vmax - 1e-9) sat++;
            if (Math.abs(x[k][1]) >= P.phMax - 1e-9) pitchStop++;
            if (x[k][0] >= P.thMax - 1e-9) upperStop++;
            if (x[k][0] > P.thMin + 1e-6) left = true; else if (left) lowerStop++;
        }
        const dtLog = t.length > 1 ? t[1] - t[0] : 0;
        const health = run.r ? healthAnalysis(run, cfg.health) : null;
        return { health, events, vmax, satPct: 100 * sat / t.length, pitchStopTime: pitchStop * dtLog,
                 upperStopTime: upperStop * dtLog, landingTime: lowerStop * dtLog, failed: !!run.error };
    }

    // Specification rows: metric over all events of an axis (worst case) compared with a limit.
    const DEFAULT_SPECS = [
        { id: 'th_os', label: 'Elevation overshoot', axis: 'theta', metric: 'overshootPct', unit: '%', limit: 25, on: true },
        { id: 'th_ts', label: 'Elevation settling time', axis: 'theta', metric: 'settlingTime', unit: 's', limit: 10, on: true },
        { id: 'th_ess', label: 'Elevation steady-state error', axis: 'theta', metric: 'steadyStateErrDeg', unit: 'deg', limit: 0.5, on: true },
        { id: 'ps_os', label: 'Travel overshoot', axis: 'psi', metric: 'overshootPct', unit: '%', limit: 25, on: true },
        { id: 'ps_ts', label: 'Travel settling time', axis: 'psi', metric: 'settlingTime', unit: 's', limit: 25, on: true },
        { id: 'ps_ess', label: 'Travel steady-state error', axis: 'psi', metric: 'steadyStateErrDeg', unit: 'deg', limit: 1, on: true },
        { id: 'dist', label: 'Disturbance peak deviation', axis: '*', metric: 'maxDeviationDeg', unit: 'deg', limit: 5, on: true },
        { id: 'vmax', label: 'Peak motor voltage', axis: 'run', metric: 'vmax', unit: 'V', limit: 20, on: true },
        { id: 'pstop', label: 'Time on pitch stop', axis: 'run', metric: 'pitchStopTime', unit: 's', limit: 0, on: true },
        { id: 'ustop', label: 'Time on upper elevation stop', axis: 'run', metric: 'upperStopTime', unit: 's', limit: 0, on: true },
        { id: 'fdd_delay', label: 'Fault detection delay', axis: 'health', metric: 'delay', unit: 's', limit: 1, on: true },
        { id: 'fdd_false', label: 'False alarms before the fault', axis: 'health', metric: 'falseAlarm', unit: '', limit: 0, on: true }
    ];
    function checkSpecs(metrics, specs = DEFAULT_SPECS) {
        return specs.filter(s => s.on).map(s => {
            let value;
            if (s.axis === 'health') {
                const hm = metrics.health;
                if (!hm || hm.tFault === null) return { ...s, value: null, pass: null, note: 'only checked in tests with a fault' };
                value = s.metric === 'falseAlarm' ? (hm.falseAlarm ? 1 : 0) : (hm.delay === null ? Infinity : hm.delay);
            } else if (s.axis === 'run') value = metrics[s.metric];
            else {
                const ev = metrics.events.filter(e => (s.axis === '*' || e.axis === s.axis) && e[s.metric] !== undefined);
                if (!ev.length) return { ...s, value: null, pass: null, note: 'not exercised by this test' };
                const vals = ev.map(e => e[s.metric]);
                value = vals.some(v => v === null) ? Infinity : Math.max(...vals);   // null = never settled
            }
            const pass = metrics.failed ? false : value <= s.limit + 1e-12;
            return { ...s, value, pass };
        });
    }

    // =========================================================== linear analysis
    /* Linearises the closed loop of ANY controller with getState/setState around hover:
       one 1 kHz step maps (x, c) -> (x', c'); its Jacobian Phi gives the discrete poles,
       mapped to s = ln(lambda)/h. Breaking the loop at the plant input gives loop
       transfer functions in collective / cyclic voltage, from which the classical
       margins are computed. */
    function linearAnalysis(makeCtrl, presetKey = 'nominal', opts = {}) {
        const P = Object.assign({}, S.PRESETS[presetKey]);
        const h = 1e-3, ref = { theta: 0, psi: 0 };
        // 1) operating point: fly the controller to hover and let it settle
        const ctrl = makeCtrl(P); ctrl.reset();
        let x = [0, 0, 0, 0, 0, 0];
        const Tset = opts.Tsettle || 60, N = Math.round(Tset / h);
        let xPrev = null, cPrev = null;
        for (let k = 0; k < N; k++) {
            const u = ctrl.step(x.slice(), ref, h);
            x = S.rk4(x, [clamp(u[0], -P.Vmax, P.Vmax), clamp(u[1], -P.Vmax, P.Vmax)], P, h);
            if (k === N - 1001) { xPrev = x.slice(); cPrev = ctrl.getState(); }
        }
        const cs = ctrl.getState();
        const drift = Math.max(...x.map((v, i) => Math.abs(v - xPrev[i])), ...cs.map((v, i) => Math.abs(v - cPrev[i])));
        const nx = 6; let nc = cs.length;
        // 2) Jacobians of the plant step (inputs u) and the controller step
        const step = (xx, cc) => { ctrl.setState(cc.slice()); const u = ctrl.step(xx.slice(), ref, h); return { u, c: ctrl.getState() }; };
        const op = step(x, cs); const u0 = op.u;
        const plantNext = (xx, uu) => S.rk4(xx, uu, P, h);
        const eps = (v) => 1e-6 * Math.max(1, Math.abs(v));
        const Ad = zeros(nx, nx), Bd = zeros(nx, 2);
        for (let j = 0; j < nx; j++) {
            const e = eps(x[j]); const xp = x.slice(), xm = x.slice(); xp[j] += e; xm[j] -= e;
            const fp = plantNext(xp, u0), fm = plantNext(xm, u0);
            for (let i = 0; i < nx; i++) Ad[i][j] = (fp[i] - fm[i]) / (2 * e);
        }
        for (let j = 0; j < 2; j++) {
            const e = eps(u0[j]); const up = u0.slice(), um = u0.slice(); up[j] += e; um[j] -= e;
            const fp = plantNext(x, up), fm = plantNext(x, um);
            for (let i = 0; i < nx; i++) Bd[i][j] = (fp[i] - fm[i]) / (2 * e);
        }
        const Ac = zeros(nc, nc), Bc = zeros(nc, nx), Cc = zeros(2, nc), Dc = zeros(2, nx);
        for (let j = 0; j < nc; j++) {
            const e = eps(cs[j]); const cp = cs.slice(), cm = cs.slice(); cp[j] += e; cm[j] -= e;
            const a = step(x, cp), b = step(x, cm);
            for (let i = 0; i < nc; i++) Ac[i][j] = (a.c[i] - b.c[i]) / (2 * e);
            for (let i = 0; i < 2; i++) Cc[i][j] = (a.u[i] - b.u[i]) / (2 * e);
        }
        for (let j = 0; j < nx; j++) {
            const e = eps(x[j]); const xp = x.slice(), xm = x.slice(); xp[j] += e; xm[j] -= e;
            const a = step(xp, cs), b = step(xm, cs);
            for (let i = 0; i < nc; i++) Bc[i][j] = (a.c[i] - b.c[i]) / (2 * e);
            for (let i = 0; i < 2; i++) Dc[i][j] = (a.u[i] - b.u[i]) / (2 * e);
        }
        ctrl.setState(cs);
        // Memory entries that never change and are not driven by the plant (for example a
        // gain matrix kept in mem) are parameters, not states: remove them.
        const keep = [];
        for (let i = 0; i < nc; i++) {
            const isConst = Ac[i].every((v, j) => Math.abs(v - (i === j ? 1 : 0)) < 1e-8) && Bc[i].every(v => Math.abs(v) < 1e-8);
            if (!isConst) keep.push(i);
        }
        if (keep.length < nc) {
            const pick = (M, rows, cols) => rows.map(i => cols.map(j => M[i][j]));
            const all6 = [0, 1, 2, 3, 4, 5];
            const Ac2 = pick(Ac, keep, keep), Bc2 = pick(Bc, keep, all6), Cc2 = pick(Cc, [0, 1], keep);
            Ac.length = 0; Ac.push(...Ac2); Bc.length = 0; Bc.push(...Bc2); Cc.forEach((r, i) => { Cc[i] = Cc2[i]; });
        }
        const nParams = nc - keep.length;
        nc = keep.length;
        // Closed loop: x' = Ad x + Bd (Cc c + Dc x) ; c' = Ac c + Bc x  (u uses the updated c
        // inside step(); the Jacobians above already capture that)
        const n = nx + nc, Phi = zeros(n, n);
        const BdDc = mul(Bd, Dc), BdCc = nc ? mul(Bd, Cc) : null;
        for (let i = 0; i < nx; i++) {
            for (let j = 0; j < nx; j++) Phi[i][j] = Ad[i][j] + BdDc[i][j];
            for (let j = 0; j < nc; j++) Phi[i][nx + j] = BdCc[i][j];
        }
        for (let i = 0; i < nc; i++) {
            for (let j = 0; j < nx; j++) Phi[nx + i][j] = Bc[i][j];
            for (let j = 0; j < nc; j++) Phi[nx + i][nx + j] = Ac[i][j];
        }
        const lam = eigvals(Phi);
        const poles = lam.map(l => {
            const r = Math.hypot(l.re, l.im), th = Math.atan2(l.im, l.re);
            const s = { re: Math.log(r) / h, im: th / h, mag: r };
            const wn = Math.hypot(s.re, s.im);
            return { ...s, wn, zeta: wn > 0 ? -s.re / wn : 1, lambda: l };
        }).sort((a, b) => b.re - a.re);
        const stable = lam.every(l => Math.hypot(l.re, l.im) < 1 - 1e-12);

        // 3) loop transfer functions at the plant input, channels v = [collective, cyclic]
        //    u = Tm v with Tm = [[1, 1], [1, -1]]
        const loop = (w) => {
            const z = [Math.cos(w * h), Math.sin(w * h)];
            const zI = (M) => M.map((r, i) => r.map((v, j) => (i === j ? csub(z, [v, 0]) : [-v, 0])));
            const G = csolve(zI(Ad), Bd.map(r => r.map(v => [v, 0])));           // nx x 2
            let K = Dc.map(r => r.map(v => [v, 0]));                               // 2 x nx
            if (nc) {
                const X = csolve(zI(Ac), Bc.map(r => r.map(v => [v, 0])));          // nc x nx
                K = K.map((r, i) => r.map((v, j) => { let s = v; for (let q = 0; q < nc; q++) s = cadd(s, cmul([Cc[i][q], 0], X[q][j])); return s; }));
            }
            // M = K G (2 x 2) : u_in -> u_out ; transform to channels
            const M = [0, 1].map(i => [0, 1].map(j => { let s = [0, 0]; for (let q = 0; q < nx; q++) s = cadd(s, cmul(K[i][q], G[q][j])); return s; }));
            const Tm = [[1, 1], [1, -1]], Ti = [[0.5, 0.5], [0.5, -0.5]];
            const Mv = [0, 1].map(i => [0, 1].map(j => { let s = [0, 0]; for (let a = 0; a < 2; a++) for (let b = 0; b < 2; b++) s = cadd(s, cmul([Ti[i][a] * Tm[b][j], 0], M[a][b])); return s; }));
            // break one channel with the other closed; negative-feedback convention L = -(...)
            const one = [1, 0];
            const L1 = scalC(cadd(Mv[0][0], cdiv(cmul(Mv[0][1], Mv[1][0]), csub(one, Mv[1][1]))), -1);
            const L2 = scalC(cadd(Mv[1][1], cdiv(cmul(Mv[1][0], Mv[0][1]), csub(one, Mv[0][0]))), -1);
            return [L1, L2];
        };
        const wmin = opts.wmin || 1e-2, wmax = Math.PI / h * 0.95, NW = opts.nw || 1200;
        const ws = Array.from({ length: NW }, (_, i) => wmin * Math.pow(wmax / wmin, i / (NW - 1)));
        const resp = ws.map(loop);
        const channels = ['collective (elevation)', 'cyclic (pitch and travel)'].map((name, ch) => {
            const mag = resp.map(r => cabs(r[ch]));
            let ph = resp.map(r => Math.atan2(r[ch][1], r[ch][0]) * 180 / Math.PI);
            for (let i = 1; i < ph.length; i++) { while (ph[i] - ph[i - 1] > 180) ph[i] -= 360; while (ph[i] - ph[i - 1] < -180) ph[i] += 360; }
            // shift so the low-frequency phase lies in (-360, 0]
            const off = Math.ceil(ph[0] / 360) * 360; ph = ph.map(p => p - off);
            const gms = [], pms = [];
            // exact evaluation between grid points, phase kept continuous with the grid
            const Lat = (w) => loop(w)[ch];
            const phNear = (w, ref) => { const c = Lat(w); let p = Math.atan2(c[1], c[0]) * 180 / Math.PI; while (p - ref > 180) p -= 360; while (p - ref < -180) p += 360; return p; };
            const bisect = (lo, hi, f) => { let a = Math.log(lo), b = Math.log(hi), fa = f(lo); for (let it = 0; it < 60; it++) { const m = 0.5 * (a + b), fm = f(Math.exp(m)); if ((fm > 0) === (fa > 0)) { a = m; fa = fm; } else b = m; } return Math.exp(0.5 * (a + b)); };
            for (let i = 1; i < ws.length; i++) {
                if (((mag[i - 1] - 1) * (mag[i] - 1)) <= 0 && mag[i - 1] !== mag[i]) {
                    const wc = bisect(ws[i - 1], ws[i], (w) => Math.log(cabs(Lat(w))));
                    const pc = phNear(wc, 0.5 * (ph[i - 1] + ph[i]));
                    let pm = ((pc + 180) % 360 + 360) % 360; if (pm > 180) pm -= 360;
                    pms.push({ w: wc, pmDeg: pm });
                }
                const k1 = Math.floor((ph[i - 1] + 180) / 360), k2 = Math.floor((ph[i] + 180) / 360);
                if (k1 !== k2) {
                    const target = Math.max(k1, k2) * 360 - 180;
                    const mid = 0.5 * (ph[i - 1] + ph[i]);
                    const wp = bisect(ws[i - 1], ws[i], (w) => phNear(w, mid) - target);
                    gms.push({ w: wp, gmDb: -20 * Math.log10(cabs(Lat(wp))) });
                }
            }
            const pm = pms.length ? pms.reduce((a, b) => Math.abs(b.pmDeg) < Math.abs(a.pmDeg) ? b : a) : null;
            // present the phase so that it lies in (-360, 0] at the gain crossover
            if (pm) {
                const i = ws.findIndex(w => w >= pm.w); const k = Math.floor((ph[i] + 360) / 360 + 1e-9);
                if (k !== 0) { ph = ph.map(p => p - k * 360); }
            }
            // gain-increase margin (crossings with |L| < 1) and gain-reduction margin (|L| > 1,
            // typical of loops around double integrators such as this plant)
            const up = gms.filter(g => g.gmDb > 0), down = gms.filter(g => g.gmDb < 0);
            const gmUp = up.length ? up.reduce((a, b) => b.gmDb < a.gmDb ? b : a) : null;
            const gmDown = down.length ? down.reduce((a, b) => b.gmDb > a.gmDb ? b : a) : null;
            return { name, w: ws, magDb: mag.map(m => 20 * Math.log10(m)), phaseDeg: ph, pm, gmUp, gmDown, crossings: { pms, gms } };
        });
        return { stable, poles, channels, operatingPoint: { x, u: u0, c: cs, drift }, Phi, nc, nParams, P: presetKey };
    }
    const scalC = (a, c) => [a[0] * c, a[1] * c];

    // =========================================================== templates
    const CODE_TEMPLATES = {
        pid: (() => { const g = S.GAINS.pid, f = (v) => +v.toPrecision(10); return `// Cascaded PID (same law and gains as the built-in controller).
// y = [theta, phi, psi, dtheta, dphi, dpsi] in rad and rad/s; ref = {theta, psi} in rad.
// Return [Vf, Vb] in volts. Keep anything that must persist between steps in mem.

function init(P, lib) {
  return { iTh: 0, iPs: 0 };
}

function step(y, ref, dt, mem, P, lib) {
  const g = { kp_e: ${f(g.kp_e)}, ki_e: ${f(g.ki_e)}, kd_e: ${f(g.kd_e)},   // elevation loop
              kp_p: ${f(g.kp_p)}, kd_p: ${f(g.kd_p)},                    // pitch loop
              kp_t: ${f(g.kp_t)}, ki_t: ${f(g.ki_t)}, kd_t: ${f(g.kd_t)}, // travel loop -> pitch reference
              phiMax: 20 * lib.deg };
  const eTh = ref.theta - y[0];
  const ePs = lib.wrap(ref.psi - y[2]);
  mem.iTh = lib.clamp(mem.iTh + eTh * dt, -1, 1);
  mem.iPs = lib.clamp(mem.iPs + ePs * dt, -2, 2);
  const Vs = 2 * lib.hoverVoltage(y[0], y[1]) + g.kp_e * eTh + g.ki_e * mem.iTh - g.kd_e * y[3];
  const phiRef = lib.clamp(-(g.kp_t * ePs + g.ki_t * mem.iPs - g.kd_t * y[5]), -g.phiMax, g.phiMax);
  const Vd = g.kp_p * (phiRef - y[1]) - g.kd_p * y[4];
  return [(Vs + Vd) / 2, (Vs - Vd) / 2];
}
`; })(),
        lqri: `// LQR with integral action. K is computed once in init() from your Q and R.
// Augmented state z = [theta-theta_r, phi, psi-psi_r, dtheta, dphi, dpsi, int e_theta, int e_psi]

function init(P, lib) {
  const Q = [100, 1, 10, 0, 0, 2, 10, 0.1];   // edit the weights
  const R = [0.05, 0.05];
  return { K: lib.lqriGain(Q, R), iTh: 0, iPs: 0 };
}

function step(y, ref, dt, mem, P, lib) {
  const eTh = y[0] - ref.theta, ePs = lib.wrap(y[2] - ref.psi);
  mem.iTh = lib.clamp(mem.iTh + eTh * dt, -1, 1);
  mem.iPs = lib.clamp(mem.iPs + ePs * dt, -2, 2);
  const z = [eTh, y[1], ePs, y[3], y[4], y[5], mem.iTh, mem.iPs];
  const ff = lib.hoverVoltage(y[0], y[1]);
  const u = mem.K.map(row => ff - row.reduce((s, k, i) => s + k * z[i], 0));
  return u;
}
`,
        blank: `// Write your own control law. Inputs: y (state), ref (targets), dt (s), mem (your memory).
function init(P, lib) {
  return {};
}

function step(y, ref, dt, mem, P, lib) {
  const Vop = lib.hoverVoltage(y[0], y[1]);   // per-motor hover voltage
  return [Vop, Vop];
}
`
    };

    // Block-diagram template: the cascaded PID of the built-in controller
    function diagramPID(g = S.GAINS.pid) {
        const r = (v) => +v.toPrecision(10);
        const B = (id, type, x, y, params) => ({ id, type, x, y, params: Object.assign({}, BLOCK_TYPES[type].params, params || {}) });
        const blocks = [
            B('thr', 'source', 20, 40, { signal: 'theta_ref' }), B('th', 'source', 20, 100, { signal: 'theta' }),
            B('eth', 'sum', 160, 60, { signs: '+-' }),
            B('kpe', 'gain', 300, 20, { k: r(g.kp_e) }), B('ie', 'integrator', 300, 90, { lim: 1 }), B('kie', 'gain', 440, 90, { k: r(g.ki_e) }),
            B('dth', 'source', 20, 170, { signal: 'dtheta' }), B('kde', 'gain', 300, 170, { k: r(g.kd_e) }),
            B('vop', 'source', 20, 230, { signal: 'Vop' }), B('two', 'gain', 300, 230, { k: 2 }),
            B('vs', 'sum', 600, 120, { signs: '+++-' }),
            B('psr', 'source', 20, 320, { signal: 'psi_ref' }), B('ps', 'source', 20, 380, { signal: 'psi' }),
            B('eps', 'sum', 160, 340, { signs: '+-' }), B('wr', 'wrap', 300, 340),
            B('kpt', 'gain', 440, 300, { k: r(g.kp_t) }), B('it', 'integrator', 440, 370, { lim: 2 }), B('kit', 'gain', 580, 370, { k: r(g.ki_t) }),
            B('dps', 'source', 20, 450, { signal: 'dpsi' }), B('kdt', 'gain', 440, 450, { k: r(g.kd_t) }),
            B('st', 'sum', 720, 360, { signs: '++-' }), B('neg', 'gain', 850, 360, { k: -1 }),
            B('sat', 'saturation', 980, 360, { lo: '-20*deg', hi: '20*deg' }),
            B('ph', 'source', 980, 450, { signal: 'phi' }), B('ep', 'sum', 1110, 380, { signs: '+-' }),
            B('kpp', 'gain', 1240, 380, { k: r(g.kp_p) }),
            B('dph', 'source', 980, 520, { signal: 'dphi' }), B('kdp', 'gain', 1240, 470, { k: r(g.kd_p) }),
            B('vd', 'sum', 1370, 410, { signs: '+-' }),
            B('mix', 'mixer', 1370, 160), B('of', 'output', 1500, 130, { channel: 'Vf' }), B('ob', 'output', 1500, 200, { channel: 'Vb' })
        ];
        const W = (a, ap, b, bp) => ({ from: { id: a, port: ap }, to: { id: b, port: bp } });
        const wires = [
            W('thr', 0, 'eth', 0), W('th', 0, 'eth', 1), W('eth', 0, 'kpe', 0), W('eth', 0, 'ie', 0), W('ie', 0, 'kie', 0),
            W('dth', 0, 'kde', 0), W('vop', 0, 'two', 0),
            W('two', 0, 'vs', 0), W('kpe', 0, 'vs', 1), W('kie', 0, 'vs', 2), W('kde', 0, 'vs', 3),
            W('psr', 0, 'eps', 0), W('ps', 0, 'eps', 1), W('eps', 0, 'wr', 0), W('wr', 0, 'kpt', 0), W('wr', 0, 'it', 0), W('it', 0, 'kit', 0),
            W('dps', 0, 'kdt', 0), W('kpt', 0, 'st', 0), W('kit', 0, 'st', 1), W('kdt', 0, 'st', 2), W('st', 0, 'neg', 0), W('neg', 0, 'sat', 0),
            W('sat', 0, 'ep', 0), W('ph', 0, 'ep', 1), W('ep', 0, 'kpp', 0), W('dph', 0, 'kdp', 0), W('kpp', 0, 'vd', 0), W('kdp', 0, 'vd', 1),
            W('vs', 0, 'mix', 0), W('vd', 0, 'mix', 1), W('mix', 0, 'of', 0), W('mix', 1, 'ob', 0)
        ];
        return { name: 'Cascaded PID (blocks)', blocks, wires };
    }

    return { DEG, mul, add, T, inv, solve, eye, zeros, diag, eigvals, csolve, care, lqr, lqriGain, augmentedModel,
             evalExpr, builtinPID, builtinLQRI, compileCode, compileDiagram, BLOCK_TYPES, SIGNALS, nIn, nOut,
             scenarioFns, SCENARIOS, runScenario, computeMetrics, DEFAULT_SPECS, checkSpecs, linearAnalysis,
             HEALTH_DEFAULTS, makeHealthMonitor, healthAnalysis,
             CODE_TEMPLATES, diagramPID, flatten, unflatten, makeLib };
}));
