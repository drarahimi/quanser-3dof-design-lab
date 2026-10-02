/*
 * sim-core.js — physics and control core of the 3-DOF helicopter virtual laboratory.
 * Pure functions, no DOM. Loaded by 3dof-quanser-simulator.html and usable from Node
 * (require('./sim-core.js')) for headless verification.
 *
 * Notation (as in the UI): theta = elevation, phi = pitch, psi = travel.
 * State x = [theta, phi, psi, dtheta, dphi, dpsi] (rad, rad/s). Input V = [Vf, Vb] (volts).
 *
 *   Je*theta'' = Kf*La*cos(phi)*(Vf+Vb) - m*g*La*cos(theta) - De*theta'
 *   Jp*phi''   = Kf*Lh*(Vf-Vb)                              - Dp*phi'
 *   Jt*psi''   = -Kf*La*cos(theta)*sin(phi)*(Vf+Vb)         - Dt*psi'
 *
 * Parameter sources: see PRESETS below and the paper's Table of parameters.
 */
(function (root, factory) {
    const api = factory();
    if (typeof module === 'object' && module.exports) module.exports = api;
    else root.Sim3DOF = api;
}(typeof self !== 'undefined' ? self : this, function () {
    'use strict';
    const DEG = Math.PI / 180;
    const VERSION = '2.2.0';

    // Preset A: vendor nominal values as reproduced in Li et al., IEEE TIE 2015, and
    // Wang et al., arXiv:2008.10817 (Table I). Jt = Je + Jp is derived from the mass
    // distribution (counterweight + two motors about the vertical axis at theta = 0).
    // Limits: pitch +/-32 deg and 63.5 deg elevation range from the Quanser data sheet;
    // the lower elevation stop (-27.5 deg) from Wang et al., IEEE/ASME TMECH 2022.
    const NOMINAL = {
        name: 'Quanser nominal',
        Je: 1.0348, Jp: 0.0451, Jt: 1.0348 + 0.0451,
        La: 0.66, Lh: 0.178, m: 0.094, g: 9.81, Kf: 0.1188,
        De: 0.0, Dp: 0.0, Dt: 0.0,
        Vmax: 24.0,
        thMin: -27.5 * DEG, thMax: 36.0 * DEG, phMax: 32.0 * DEG,
        encCounts: { theta: 4096, phi: 4096, psi: 8192 }   // theta resolution assumed = pitch
    };
    // Preset B: inertias and travel friction identified on a physical unit (Ruf, Chalmers
    // MSc thesis, 2014). Used as a "second rig" to teach robustness to model mismatch.
    const IDENTIFIED = Object.assign({}, NOMINAL, {
        name: 'Identified (Ruf 2014)',
        Je: 1.1555, Jp: 0.0398, Jt: 1.1785, Dt: 0.0247 * 1.1785
    });
    const PRESETS = { nominal: NOMINAL, identified: IDENTIFIED };

    // Gains computed offline by src/design.py on the NOMINAL linear model
    // (LQR-I: Q = diag(100,1,10,0,0,2,10,0.1), R = 0.05 I; PID: pole placement).
    const GAINS = {
        lqri: [
            [38.055419, 10.326044, -11.914248, 22.410744, 4.692854, -20.974648, 10.000000, -1.000000],
            [38.055419, -10.326044, 11.914248, 22.410744, -4.692854, 20.974648, 10.000000, 1.000000]
        ],
        pid: {
            kp_e: 29.694674, kd_e: 35.633609, ki_e: 11.135503,
            kp_p: 34.124012, kd_p: 13.649605,
            kp_t: 0.638771, kd_t: 1.916313, ki_t: 0.038326,
            phi_ref_max: 20 * DEG
        }
    };

    const hoverVoltage = (P, th = 0, ph = 0) => P.m * P.g * Math.cos(th) / (2 * P.Kf * Math.cos(ph));
    const wrap = (a) => { let d = a % (2 * Math.PI); if (d > Math.PI) d -= 2 * Math.PI; if (d < -Math.PI) d += 2 * Math.PI; return d; };
    const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

    // d = optional external disturbance torques [tau_theta, tau_phi, tau_psi] in N m
    function derivs(x, V, P, d) {
        const th = x[0], ph = x[1], dth = x[3], dph = x[4], dps = x[5];
        const Vs = V[0] + V[1], Vd = V[0] - V[1];
        const de = d ? d[0] : 0, dp = d ? d[1] : 0, dt = d ? d[2] : 0;
        return [
            dth, dph, dps,
            (P.Kf * P.La * Math.cos(ph) * Vs - P.m * P.g * P.La * Math.cos(th) - P.De * dth + de) / P.Je,
            (P.Kf * P.Lh * Vd - P.Dp * dph + dp) / P.Jp,
            (-P.Kf * P.La * Math.cos(th) * Math.sin(ph) * Vs - P.Dt * dps + dt) / P.Jt
        ];
    }

    function rk4(x, V, P, h, d) {
        const k1 = derivs(x, V, P, d);
        const x2 = x.map((v, i) => v + 0.5 * h * k1[i]);
        const k2 = derivs(x2, V, P, d);
        const x3 = x.map((v, i) => v + 0.5 * h * k2[i]);
        const k3 = derivs(x3, V, P, d);
        const x4 = x.map((v, i) => v + h * k3[i]);
        const k4 = derivs(x4, V, P, d);
        return x.map((v, i) => v + (h / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
    }

    // Integrator used by v1 of the simulator (semi-implicit Euler, one step per frame).
    // Kept only so the paper can quantify what the change to fixed-step RK4 buys.
    function semiImplicitEuler(x, V, P, h, dist) {
        const d = derivs(x, V, P, dist);
        const v = [x[3] + h * d[3], x[4] + h * d[4], x[5] + h * d[5]];
        return [x[0] + h * v[0], x[1] + h * v[1], x[2] + h * v[2], v[0], v[1], v[2]];
    }

    // Mechanical stops (same restitution rule as v1).
    function applyLimits(x, P) {
        if (x[1] > P.phMax) { x[1] = P.phMax; x[4] *= -0.2; }
        else if (x[1] < -P.phMax) { x[1] = -P.phMax; x[4] *= -0.2; }
        if (x[0] > P.thMax) { x[0] = P.thMax; x[3] *= -0.2; }
        else if (x[0] < P.thMin) { x[0] = P.thMin; if (x[3] < 0) x[3] = 0; }
        return x;
    }

    const quant = (a, counts) => { const q = 2 * Math.PI / counts; return Math.round(a / q) * q; };

    // Measurement model: ideal (true state) or encoder angles + filtered derivative,
    // as on the hardware, where rates are obtained by filtering the encoder signal.
    function makeSensor(P, hw) {
        const wc = (hw && hw.derivBandwidth) || 50;   // rad/s, first-order derivative filter
        let prev = null, rate = [0, 0, 0];
        return {
            reset() { prev = null; rate = [0, 0, 0]; },
            measure(x, h, fs) {
                let y;
                if (!hw || !hw.encoders) y = x.slice();
                else {
                    const a = [quant(x[0], P.encCounts.theta), quant(x[1], P.encCounts.phi), quant(x[2], P.encCounts.psi)];
                    if (fs) for (let i = 0; i < 3; i++) { a[i] += fs.bias[i]; if (fs.stuck[i] !== null) a[i] = fs.stuck[i]; }
                    if (prev) {
                        const f = Math.exp(-wc * h);
                        for (let i = 0; i < 3; i++) rate[i] = f * rate[i] + (1 - f) * (a[i] - prev[i]) / h;
                    }
                    prev = a;
                    return [a[0], a[1], a[2], rate[0], rate[1], rate[2]];
                }
                if (fs) for (let i = 0; i < 3; i++) {   // ideal rate sensing: a stuck channel reads constant, rate 0
                    y[i] += fs.bias[i];
                    if (fs.stuck[i] !== null) { y[i] = fs.stuck[i]; y[i + 3] = 0; }
                }
                return y;
            }
        };
    }

    // ----- Faults (for diagnosis and prognosis exercises) -----
    // faults: [{ type: 'rotor', target: 'front'|'back'|'both', t, size (fraction of thrust lost), ramp (s, 0 = abrupt) },
    //          { type: 'friction', t, size (extra travel damping, N m s/rad), ramp },
    //          { type: 'bias', target: 'theta'|'phi'|'psi', t, size (deg), ramp },
    //          { type: 'stuck', target: 'theta'|'phi'|'psi', t }]
    // A rotor fault scales that rotor's thrust: F = eta * Kf * V with eta = 1 - size * progress.
    const AXIS = { theta: 0, phi: 1, psi: 2 };
    function makeFaults(list) {
        const faults = (list || []).filter(f => f && f.type);
        const stuckAt = [null, null, null];
        const progress = (f, t) => t + 1e-12 < f.t ? 0 : (f.ramp > 0 ? Math.min(1, (t - f.t) / f.ramp) : 1);
        return {
            active: faults.length > 0,
            list: faults,
            reset() { stuckAt[0] = stuckAt[1] = stuckAt[2] = null; },
            // state at time t; x (true state) is used to latch a stuck sensor's value at onset
            at(t, x) {
                const fs = { eta: [1, 1], dDt: 0, bias: [0, 0, 0], stuck: [null, null, null] };
                for (const f of faults) {
                    const a = progress(f, t); if (a <= 0) continue;
                    if (f.type === 'rotor') {
                        const loss = clamp((f.size || 0) * a, 0, 1);
                        if (f.target !== 'back') fs.eta[0] *= 1 - loss;
                        if (f.target !== 'front') fs.eta[1] *= 1 - loss;
                    } else if (f.type === 'friction') fs.dDt += (f.size || 0) * a;
                    else if (f.type === 'bias') fs.bias[AXIS[f.target]] += (f.size || 0) * a * DEG;
                    else if (f.type === 'stuck') {
                        const i = AXIS[f.target];
                        if (stuckAt[i] === null && x) stuckAt[i] = x[i];
                        fs.stuck[i] = stuckAt[i];
                    }
                }
                return fs;
            }
        };
    }

    // ----- Health monitor: generalized-momentum residual against the nominal model -----
    // For each axis i, r_i estimates the unknown torque (fault or disturbance) acting on it:
    //   r = K_O [ J w(t) - J w(t0) - integral (tau_model(y, V) + r) dt ],  i.e. r' = K_O (tau_unknown - r).
    // No differentiation of measurements is needed. The monitor knows only the nominal model,
    // the commanded voltages and the measured outputs. Stop contacts are not modelled, so the
    // monitor is inhibited while an angle is at a stop and re-initialised afterwards.
    function makeMonitor(P, cfg = {}) {
        const KO = cfg.KO ?? 5;                       // observer bandwidth, rad/s
        const J = [P.Je, P.Jp, P.Jt];
        const thr = cfg.thr || [0.02, 0.005, 0.01];   // N m, detection thresholds per axis
        const dwell = cfg.dwell ?? 0.25;              // s above threshold before an alarm
        const tauM = cfg.motorTau || 0;               // actuator lag assumed by the monitor
        const tauS = cfg.tauS ?? 0.1;                 // s, smoothing of the detection statistic only
        const wcR = cfg.rateFilter || 0;
        const hold = cfg.hold ?? 0.5;                 // s, monitor inhibited at start-up and after a stop contact              // rad/s: rates come from a filtered derivative, so filter the model torque the same way
        let r, rs, I, p0, act, inhibit, above, alarm, tf;
        const reset = () => { r = [0, 0, 0]; rs = [0, 0, 0]; I = [0, 0, 0]; p0 = null; act = null; inhibit = hold; above = 0; alarm = null; tf = null; };
        reset();
        const tolStop = cfg.stopTol ?? 0.5 * DEG;     // encoder quantisation can hide exact contact
        const atStop = (y) => Math.abs(y[1]) >= P.phMax - tolStop || y[0] >= P.thMax - tolStop || y[0] <= P.thMin + tolStop;
        return {
            reset,
            get alarm() { return alarm; },
            // y: measured outputs, V: commanded voltages applied this step, h: step
            step(t, y, V, h) {
                const Vc = [clamp(V[0], -P.Vmax, P.Vmax), clamp(V[1], -P.Vmax, P.Vmax)];   // same order as the plant: clamp, then lag
                if (!act) act = tauM > 0 ? [0, 0] : Vc.slice();   // motors start at rest, as in the plant
                if (tauM > 0) { const a = 1 - Math.exp(-h / tauM); act[0] += a * (Vc[0] - act[0]); act[1] += a * (Vc[1] - act[1]); }
                else { act[0] = Vc[0]; act[1] = Vc[1]; }
                const Va = act.slice();
                if (atStop(y)) { inhibit = hold; p0 = null; r = [0, 0, 0]; rs = [0, 0, 0]; above = 0; }
                else if (inhibit > 0) { inhibit -= h; p0 = null; r = [0, 0, 0]; rs = [0, 0, 0]; above = 0; }
                const live = inhibit <= 0 && !atStop(y);
                if (live && p0 === null) { p0 = [J[0] * y[3], J[1] * y[4], J[2] * y[5]]; I = [0, 0, 0]; r = [0, 0, 0]; rs = [0, 0, 0]; }
                // model torque, passed through the same filter as the measured rates (runs continuously)
                const acc = derivs(y, Va, P);
                const tm = [J[0] * acc[3], J[1] * acc[4], J[2] * acc[5]];
                if (wcR > 0) {
                    const af = 1 - Math.exp(-wcR * h);
                    if (!tf) tf = tm.slice(); else for (let i = 0; i < 3; i++) tf[i] += af * (tm[i] - tf[i]);
                } else tf = tm;
                // regressors for isolation: unknown torques produced by losing a fraction of each rotor's thrust
                const c = Math.cos(y[1]), Lc = P.Kf * P.La;
                const g = [[-Lc * c * Va[0], -Lc * c * Va[1]], [-P.Kf * P.Lh * Va[0], P.Kf * P.Lh * Va[1]],
                           [Lc * Math.cos(y[0]) * Math.sin(y[1]) * Va[0], Lc * Math.cos(y[0]) * Math.sin(y[1]) * Va[1]]];
                const out = { r: r.slice(), live, g, s: 0, alarm: null };
                if (!live) return out;
                for (let i = 0; i < 3; i++) {
                    I[i] += (tf[i] + r[i]) * h;
                    r[i] = KO * (J[i] * y[3 + i] - p0[i] - I[i]);
                }
                const as = tauS > 0 ? 1 - Math.exp(-h / tauS) : 1;
                for (let i = 0; i < 3; i++) rs[i] += as * (r[i] - rs[i]);
                const s = Math.max(Math.abs(rs[0]) / thr[0], Math.abs(rs[1]) / thr[1], Math.abs(rs[2]) / thr[2]);
                if (s > 1) { above += h; if (alarm === null && above >= dwell) alarm = t; } else above = 0;
                out.r = r.slice(); out.s = s; out.alarm = alarm;
                return out;
            }
        };
    }

    // ----- Controllers. Each returns { name, reset(), step(y, ref, h) -> [Vf, Vb],
    //       getState() -> number[], setState(number[]) }. The state accessors let the
    //       design tools linearise any controller, including student-written ones. -----
    function makeLQRI(P, K = GAINS.lqri, lim = { e: 1, t: 2 }) {
        let iTh = 0, iPs = 0;
        return {
            name: 'LQR-I',
            reset() { iTh = 0; iPs = 0; },
            getState() { return [iTh, iPs]; },
            setState(s) { iTh = s[0]; iPs = s[1]; },
            step(y, ref, h) {
                const eTh = y[0] - ref.theta, ePs = wrap(y[2] - ref.psi);
                iTh = clamp(iTh + eTh * h, -lim.e, lim.e);
                iPs = clamp(iPs + ePs * h, -lim.t, lim.t);
                const z = [eTh, y[1], ePs, y[3], y[4], y[5], iTh, iPs];
                const ff = hoverVoltage(P, y[0], y[1]);   // nonlinear gravity feedforward
                const u = [0, 1].map(r => ff - K[r].reduce((s, k, i) => s + k * z[i], 0));
                return u;
            }
        };
    }

    function makePID(P, g = GAINS.pid) {
        let iTh = 0, iPs = 0;
        const limE = g.int_lim_e ?? 1, limT = g.int_lim_t ?? 2;
        return {
            name: 'Cascaded PID',
            reset() { iTh = 0; iPs = 0; },
            getState() { return [iTh, iPs]; },
            setState(s) { iTh = s[0]; iPs = s[1]; },
            step(y, ref, h) {
                const eTh = ref.theta - y[0], ePs = wrap(ref.psi - y[2]);
                iTh = clamp(iTh + eTh * h, -limE, limE);
                iPs = clamp(iPs + ePs * h, -limT, limT);
                const Vs = 2 * hoverVoltage(P, y[0], y[1]) + g.kp_e * eTh + g.ki_e * iTh - g.kd_e * y[3];
                // psi'' = -bt*phi, so a positive travel error needs a negative pitch reference
                const phiRef = clamp(-(g.kp_t * ePs + g.ki_t * iPs - g.kd_t * y[5]), -g.phi_ref_max, g.phi_ref_max);
                const Vd = g.kp_p * (phiRef - y[1]) - g.kd_p * y[4];
                return [(Vs + Vd) / 2, (Vs - Vd) / 2];
            }
        };
    }

    // ----- One fixed-step plant/controller update (used by both the UI loop and simulate) -----
    function makeStepper(P, opts = {}) {
        const hw = opts.hw || {};
        const sensor = makeSensor(P, hw);
        const integ = opts.integrator === 'semi-implicit-euler' ? semiImplicitEuler : rk4;
        const act = [0, 0];               // motor (actuator) state, volts
        return {
            sensor, act,
            reset(x0) { sensor.reset(); act[0] = 0; act[1] = 0; },
            step(x, Vcmd, h, dist, fs) {
                let Vc = [clamp(Vcmd[0], -P.Vmax, P.Vmax), clamp(Vcmd[1], -P.Vmax, P.Vmax)];
                if (hw.motorTau && hw.motorTau > 0) {
                    const a = 1 - Math.exp(-h / hw.motorTau);
                    act[0] += a * (Vc[0] - act[0]); act[1] += a * (Vc[1] - act[1]);
                    Vc = [act[0], act[1]];
                } else { act[0] = Vc[0]; act[1] = Vc[1]; }
                let Pf = P;
                if (fs) {   // thrust is linear in voltage, so a loss of effectiveness scales the effective voltage
                    Vc = [Vc[0] * fs.eta[0], Vc[1] * fs.eta[1]];
                    if (fs.dDt) Pf = Object.assign({}, P, { Dt: P.Dt + fs.dDt });
                }
                const xn = integ(x, Vc, Pf, h, dist);
                return opts.limits === false ? xn : applyLimits(xn, P);
            }
        };
    }

    /**
     * Headless simulation.
     *  opts: { P, T, h=1e-3, x0, controller: 'open'|'pid'|'lqri'| controller object,
     *          input(t)->[Vf,Vb], ref(t)->{theta,psi}, dist(t)->[tau_th,tau_ph,tau_ps],
     *          hw:{encoders,motorTau}, integrator, limits, logEvery }
     * Controller and plant both run at h (zero-order hold on the command).
     */
    function simulate(opts) {
        const P = opts.P || NOMINAL, h = opts.h || 1e-3, T = opts.T;
        const N = Math.round(T / h), logEvery = opts.logEvery || 1;
        const stepper = makeStepper(P, opts);
        const ctrl = (opts.controller && typeof opts.controller === 'object') ? opts.controller
                   : opts.controller === 'lqri' ? makeLQRI(P, opts.K)
                   : opts.controller === 'pid' ? makePID(P, opts.pidGains) : null;
        if (ctrl && ctrl.reset) ctrl.reset();
        let x = (opts.x0 || [0, 0, 0, 0, 0, 0]).slice();
        const faults = makeFaults(opts.faults); faults.reset();
        const mon = opts.monitor || null; if (mon) mon.reset();
        const log = { t: [], x: [], V: [], ref: [] };
        if (faults.active || mon) Object.assign(log, { y: [], eta: [], r: [], s: [], mlive: [], g: [] });
        for (let k = 0; k <= N; k++) {
            const t = k * h;
            const ref = opts.ref ? opts.ref(t) : { theta: 0, psi: 0 };
            const fs = faults.active ? faults.at(t, x) : null;
            const y = stepper.sensor.measure(x, h, fs);
            let V;
            if (ctrl) V = ctrl.step(y, ref, h);
            else V = opts.input(t);
            V = [clamp(V[0], -P.Vmax, P.Vmax), clamp(V[1], -P.Vmax, P.Vmax)];
            const m = mon ? mon.step(t, y, V, h) : null;
            if (k % logEvery === 0) {
                log.t.push(t); log.x.push(x.slice()); log.V.push(V); log.ref.push([ref.theta, ref.psi]);
                if (log.y) { log.y.push(y); log.eta.push(fs ? fs.eta.slice() : [1, 1]);
                             log.r.push(m ? m.r : [0, 0, 0]); log.s.push(m ? m.s : 0); log.mlive.push(m ? m.live : false); log.g.push(m ? m.g : null); }
            }
            if (k < N) x = stepper.step(x, V, h, opts.dist ? opts.dist(t) : null, fs);
        }
        if (mon) log.alarm = mon.alarm;
        return log;
    }

    // Central-difference Jacobians of derivs at (x0, V0); used to verify the code
    // against the analytic linear model.
    function linearizeFD(P, x0, V0, eps = 1e-6) {
        const A = [], B = [];
        for (let i = 0; i < 6; i++) A.push(new Array(6).fill(0));
        for (let i = 0; i < 6; i++) B.push([0, 0]);
        for (let j = 0; j < 6; j++) {
            const xp = x0.slice(), xm = x0.slice(); xp[j] += eps; xm[j] -= eps;
            const fp = derivs(xp, V0, P), fm = derivs(xm, V0, P);
            for (let i = 0; i < 6; i++) A[i][j] = (fp[i] - fm[i]) / (2 * eps);
        }
        for (let j = 0; j < 2; j++) {
            const vp = V0.slice(), vm = V0.slice(); vp[j] += eps; vm[j] -= eps;
            const fp = derivs(x0, vp, P), fm = derivs(x0, vm, P);
            for (let i = 0; i < 6; i++) B[i][j] = (fp[i] - fm[i]) / (2 * eps);
        }
        return { A, B };
    }

    // Step-response metrics for automated feedback (also the AI-assistant tool output).
    function stepMetrics(t, y, r0, r1, tol = 0.02) {
        const span = r1 - r0; if (Math.abs(span) < 1e-9) return null;
        let peak = -Infinity, ts = 0, tr10 = null, tr90 = null;
        for (let k = 0; k < t.length; k++) {
            const n = (y[k] - r0) / span;
            if (n > peak) peak = n;
            if (tr10 === null && n >= 0.1) tr10 = t[k];
            if (tr90 === null && n >= 0.9) tr90 = t[k];
            if (Math.abs(n - 1) > tol) ts = t[k];
        }
        return { overshootPct: Math.max(0, (peak - 1) * 100), riseTime: (tr90 ?? NaN) - (tr10 ?? NaN),
                 settlingTime: ts, finalError: r1 - y[y.length - 1] };
    }

    return { VERSION, DEG, PRESETS, NOMINAL, IDENTIFIED, GAINS, hoverVoltage, wrap, clamp, derivs, rk4,
             semiImplicitEuler, applyLimits, makeSensor, makeLQRI, makePID, makeStepper, makeFaults, makeMonitor,
             simulate, linearizeFD, stepMetrics };
}));
