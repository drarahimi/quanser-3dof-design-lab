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
    const VERSION = '2.1.0';

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
            measure(x, h) {
                if (!hw || !hw.encoders) return x.slice();
                const y = [quant(x[0], P.encCounts.theta), quant(x[1], P.encCounts.phi), quant(x[2], P.encCounts.psi)];
                if (prev) {
                    const a = Math.exp(-wc * h);
                    for (let i = 0; i < 3; i++) rate[i] = a * rate[i] + (1 - a) * (y[i] - prev[i]) / h;
                }
                prev = y;
                return [y[0], y[1], y[2], rate[0], rate[1], rate[2]];
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
            step(x, Vcmd, h, dist) {
                let Vc = [clamp(Vcmd[0], -P.Vmax, P.Vmax), clamp(Vcmd[1], -P.Vmax, P.Vmax)];
                if (hw.motorTau && hw.motorTau > 0) {
                    const a = 1 - Math.exp(-h / hw.motorTau);
                    act[0] += a * (Vc[0] - act[0]); act[1] += a * (Vc[1] - act[1]);
                    Vc = [act[0], act[1]];
                } else { act[0] = Vc[0]; act[1] = Vc[1]; }
                const xn = integ(x, Vc, P, h, dist);
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
        const log = { t: [], x: [], V: [], ref: [] };
        for (let k = 0; k <= N; k++) {
            const t = k * h;
            const ref = opts.ref ? opts.ref(t) : { theta: 0, psi: 0 };
            let V;
            if (ctrl) V = ctrl.step(stepper.sensor.measure(x, h), ref, h);
            else V = opts.input(t);
            V = [clamp(V[0], -P.Vmax, P.Vmax), clamp(V[1], -P.Vmax, P.Vmax)];
            if (k % logEvery === 0) { log.t.push(t); log.x.push(x.slice()); log.V.push(V); log.ref.push([ref.theta, ref.psi]); }
            if (k < N) x = stepper.step(x, V, h, opts.dist ? opts.dist(t) : null);
        }
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
             semiImplicitEuler, applyLimits, makeSensor, makeLQRI, makePID, makeStepper,
             simulate, linearizeFD, stepMetrics };
}));
