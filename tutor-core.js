/*
 * tutor-core.js — simulation-grounded AI tutor for the 3-DOF helicopter virtual lab.
 * Calls any OpenAI-compatible chat-completions endpoint (default: Groq) with the
 * student's own API key, from the browser or from Node (benchmark harness).
 * The tutor can call tools that run the verified simulator, so the effect of every
 * suggestion it makes can be checked before it is given.
 */
(function (root, factory) {
    const isNode = typeof module === 'object' && module.exports;
    const S = isNode ? require('./sim-core.js') : root.Sim3DOF;
    const D = isNode ? require('./sim-design.js') : root.Sim3DOFDesign;
    const api = factory(S, D);
    if (isNode) module.exports = api; else root.Sim3DOFTutor = api;
}(typeof self !== 'undefined' ? self : this, function (S, D) {
    'use strict';
    const DEG = Math.PI / 180;
    const DEFAULTS = { baseUrl: 'https://api.groq.com/openai/v1', model: 'openai/gpt-oss-120b', maxRounds: 5, maxSims: 6, temperature: 0.2, maxTokens: 1500 };
    // Groq free-tier chat models with tool use (Oct 2026). The endpoint accepts any model id typed in the UI.
    const MODELS = ['openai/gpt-oss-120b', 'openai/gpt-oss-20b', 'qwen/qwen3.8-27b'];

    const SYSTEM = (opt) => `You are a teaching assistant in an undergraduate control lab using a model of the Quanser 3-DOF helicopter (elevation theta, pitch phi, travel psi; motor voltages Vf, Vb, |V| <= 24 V).
Je*theta'' = Kf*La*cos(phi)*(Vf+Vb) - m*g*La*cos(theta); Jp*phi'' = Kf*Lh*(Vf-Vb); Jt*psi'' = -Kf*La*cos(theta)*sin(phi)*(Vf+Vb). Travel is driven only through pitch; pitch stops at +/-32 deg.
Rules:
- Use the student's data and quote measured numbers.
- ${opt.grounded ? 'Never state the effect of a change unless you tested it with simulate_variant or linear_analysis in this conversation; say what you tested and what it returned. Test at most two or three variants.' : 'You cannot run the simulator; say that your suggestions are untested.'}
- Explain the physical reason (saturation, stop contact, damping, wind-up, loop bandwidth) and ask a guiding question.
- ${opt.allowValues ? 'You may give specific parameter values.' : 'Do not give a full set of tuned gains; suggest the direction and rough size of one or two changes.'}
- At most about 150 words unless asked for more.`;

    // ------------------------------------------------------------------ tools
    const TOOL_SPECS = [
        { type: 'function', function: { name: 'get_design', description: 'Return the controller design currently entered by the student (type and parameters, code, or block diagram).', parameters: { type: 'object', properties: {} } } },
        { type: 'function', function: { name: 'get_run', description: 'Summary of a test run: test, specification checks, per-event metrics, saturation and stop contact; optionally a coarse time history (1 sample every 2 s).', parameters: { type: 'object', properties: { run_id: { type: 'integer', description: 'Run number; omit for the selected run.' }, include_history: { type: 'boolean', description: 'Add the coarse time history (costs tokens).' } } } } },
        { type: 'function', function: { name: 'simulate_variant', description: 'Run the current design with some parameters changed, on the current test or a standard test, and return the specification checks and key metrics. Use it to verify a suggestion before giving it.',
            parameters: { type: 'object', properties: {
                changes: { type: 'object', description: 'PID: {"pid": {"kp_e": 20, "kd_p": 15, "phi_ref_max": "15*deg", ...}}. LQR-I: {"lqri": {"Q": [8 numbers], "R": [2 numbers]}}. Block diagram: {"blocks": {"<block id>": {"k": 0.5}}}. Plant/test options: {"preset": "nominal"|"identified", "hw": true|false}.' },
                test: { type: 'string', description: 'Optional standard test: ' + Object.keys(D.SCENARIOS).join(', ') + '. Omit to use the student\'s current test.' } }, required: ['changes'] } } },
        { type: 'function', function: { name: 'linear_analysis', description: 'Linearise the closed loop at hover for the current design (optionally with changes) and return stability, the slowest/least-damped poles and the loop margins.',
            parameters: { type: 'object', properties: { changes: { type: 'object', description: 'Same format as simulate_variant.changes; may be empty.' } } } } }
    ];

    function applyChanges(design, changes = {}) {
        const d = JSON.parse(JSON.stringify(design));
        if (changes.pid) { if (d.type !== 'pid') throw new Error('The current design is not the built-in PID.'); Object.assign(d.pid, changes.pid); }
        if (changes.lqri) {
            if (d.type !== 'lqri') throw new Error('The current design is not the built-in LQR-I.');
            if (changes.lqri.Q) { d.lqri.Q = changes.lqri.Q; d.lqri.mode = 'QR'; }
            if (changes.lqri.R) { d.lqri.R = changes.lqri.R; d.lqri.mode = 'QR'; }
            if (changes.lqri.K) { d.lqri.K = changes.lqri.K; d.lqri.mode = 'K'; }
        }
        if (changes.blocks) {
            if (d.type !== 'diagram') throw new Error('The current design is not a block diagram.');
            for (const [id, pr] of Object.entries(changes.blocks)) {
                const b = d.diagram.blocks.find(x => x.id === id); if (!b) throw new Error('No block with id ' + id);
                Object.assign(b.params, pr);
            }
        }
        if (d.type === 'code' && (changes.pid || changes.lqri || changes.blocks)) throw new Error('Student code cannot be changed by the tutor.');
        return d;
    }
    function factoryFor(d) {
        switch (d.type) {
            case 'pid': return (P) => D.builtinPID(P, d.pid);
            case 'lqri': return (P) => D.builtinLQRI(P, d.lqri);
            case 'code': return (P) => D.compileCode(d.code, P);
            case 'diagram': return (P) => D.compileDiagram(d.diagram, P);
        }
        throw new Error('unknown design type');
    }
    const r3 = (v) => (v === null || v === undefined) ? null : (Number.isFinite(v) ? +(+v).toFixed(3) : String(v));
    function summarizeChecks(checks) {
        return checks.map(c => ({ spec: c.label, measured: r3(c.value), limit: c.limit, unit: c.unit, pass: c.pass }));
    }
    function summarizeRun(run, specs, metricCfg, withHistory = true) {
        const m = run.metrics || D.computeMetrics(run, metricCfg);
        const checks = run.checks || D.checkSpecs(m, specs);
        const r2 = (v) => (v === null || v === undefined) ? null : (Number.isFinite(v) ? +(+v).toFixed(2) : 'not settled');
        const out = {
            test: `${run.scenario.name}; ${run.scenario.T} s; plant ${run.scenario.preset}${run.scenario.hw ? ' + hardware effects' : ''}; refs ` +
                  run.scenario.refs.map(q => `t=${q.t}:theta=${q.theta},psi=${q.psi}${q.ramp ? ' ramp ' + q.ramp + 's' : ''}`).join(' | ') +
                  ((run.scenario.dists || []).length ? '; disturbances ' + run.scenario.dists.map(q => `t=${q.t} ${q.axis} ${q.tau}Nm for ${q.dur}s`).join(' | ') : ''),
            error: run.error || undefined,
            specs: checks.map(c => `${c.label}: ${c.value === null ? 'n/a' : r2(c.value)} ${c.unit} (limit ${c.limit}) ${c.pass === null ? 'n/a' : c.pass ? 'PASS' : 'FAIL'}`),
            events: m.events.map(e => e.type === 'disturbance'
                ? `${e.axis} disturbance t=${e.t}: peak dev ${r2(e.maxDeviationDeg)} deg, recovery ${r2(e.recoveryTime)} s`
                : `${e.axis} ${e.type} t=${e.t} ${e.from}->${e.to} deg: OS ${r2(e.overshootPct)}%, rise ${r2(e.riseTime)} s, settle ${r2(e.settlingTime)} s, ss err ${r2(e.steadyStateErrDeg)} deg`),
            run: `peak |V| ${r2(m.vmax)} V, saturated ${r2(m.satPct)}% of time, pitch stop ${r2(m.pitchStopTime)} s, upper elevation stop ${r2(m.upperStopTime)} s`
        };
        const hm = m.health;
        if (hm && (run.scenario.faults || []).length) {   // only for tests with injected faults, so fault-free prompts are unchanged
            out.health_monitor = `faults injected: ${run.scenario.faults.map(f => `${f.type}${f.target ? ' ' + f.target : ''} size ${f.size} at t=${f.t}${f.ramp ? ' ramp ' + f.ramp + 's' : ''}`).join(' | ')}; ` +
                (hm.alarm === null ? 'no alarm' : `alarm at ${r2(hm.alarm)} s (delay ${r2(hm.delay)} s), isolated as ${hm.isolatedAs}` +
                 (hm.estimate ? `, estimated thrust loss front ${r2(hm.estimate.dF * 100)}% back ${r2(hm.estimate.dB * 100)}%` : ''));
        }
        if (withHistory) {
            const step = Math.max(1, Math.round(2 / (run.t[1] - run.t[0])));   // one sample per 2 s keeps prompts small
            out.history_2s = { t: [], theta_deg: [], psi_deg: [], phi_deg: [], Vf: [], Vb: [], theta_ref: [], psi_ref: [] };
            for (let k = 0; k < run.t.length; k += step) {
                const H = out.history_2s;
                H.t.push(r3(run.t[k])); H.theta_deg.push(r3(run.x[k][0] / DEG)); H.psi_deg.push(r3(run.x[k][2] / DEG)); H.phi_deg.push(r3(run.x[k][1] / DEG));
                H.Vf.push(r3(run.V[k][0])); H.Vb.push(r3(run.V[k][1])); H.theta_ref.push(r3(run.ref[k][0] / DEG)); H.psi_ref.push(r3(run.ref[k][1] / DEG));
            }
        }
        return out;
    }
    function summarizeDesign(d) {
        if (d.type === 'pid') return { type: 'built-in cascaded PID', gains: d.pid,
            law: 'Vs = 2*Vop(theta,phi) + kp_e*e_theta + ki_e*int(e_theta) - kd_e*dtheta; phi_ref = clamp(-(kp_t*e_psi + ki_t*int(e_psi) - kd_t*dpsi), +/-phi_ref_max); Vd = kp_p*(phi_ref - phi) - kd_p*dphi; Vf=(Vs+Vd)/2, Vb=(Vs-Vd)/2' };
        if (d.type === 'lqri') return { type: 'built-in LQR with integral action', mode: d.lqri.mode, Q: d.lqri.Q, R: d.lqri.R, K: d.lqri.K,
            state: 'z = [theta-theta_r, phi, psi-psi_r, dtheta, dphi, dpsi, int(theta-theta_r), int(psi-psi_r)]; u = Vop - K z' };
        if (d.type === 'code') return { type: 'student JavaScript code', code: d.code.slice(0, 4000) };
        return { type: 'block diagram', blocks: d.diagram.blocks.map(b => ({ id: b.id, type: b.type, params: b.params })),
                 wires: d.diagram.wires.map(w => `${w.from.id}.${w.from.port} -> ${w.to.id}.${w.to.port}`) };
    }

    /* ctx = { getDesign(), getRun(id?), getScenario(), specs, metricCfg } */
    function makeToolRunner(ctx, opt = {}) {
        let sims = 0;
        const maxSims = opt.maxSims || DEFAULTS.maxSims;
        const run = {
            get_design: () => summarizeDesign(ctx.getDesign()),
            get_run: (a) => { const r = ctx.getRun(a && a.run_id); if (!r) return { error: 'No run available. Ask the student to press Simulate first.' }; return summarizeRun(r, ctx.specs, ctx.metricCfg, !!(a && a.include_history)); },
            simulate_variant: (a) => {
                if (++sims > maxSims) return { error: `Simulation budget of ${maxSims} reached for this question.` };
                const d = applyChanges(ctx.getDesign(), a.changes || {});
                let sc = a.test && D.SCENARIOS[a.test] ? JSON.parse(JSON.stringify(D.SCENARIOS[a.test])) : JSON.parse(JSON.stringify(ctx.getScenario()));
                if (a.changes && a.changes.preset) sc.preset = a.changes.preset;
                if (a.changes && a.changes.hw !== undefined) sc.hw = !!a.changes.hw;
                const P = Object.assign({}, S.PRESETS[sc.preset || 'nominal']);
                const r = D.runScenario(factoryFor(d)(P), sc);
                const out = summarizeRun(r, ctx.specs, ctx.metricCfg, false);
                delete out.test;
                out.changes_applied = a.changes; out.simulations_used = sims + '/' + maxSims;
                return out;
            },
            linear_analysis: (a) => {
                const d = applyChanges(ctx.getDesign(), (a && a.changes) || {});
                const an = D.linearAnalysis(factoryFor(d), (ctx.getScenario().preset) || 'nominal');
                const slow = an.poles.slice(0, 6).map(p => ({ re: r3(p.re), im: r3(p.im), wn: r3(p.wn), zeta: r3(p.zeta) }));
                const lowZ = an.poles.filter(p => p.im > 1e-6).sort((a1, b1) => a1.zeta - b1.zeta).slice(0, 3).map(p => ({ re: r3(p.re), im: r3(p.im), wn: r3(p.wn), zeta: r3(p.zeta) }));
                return { stable: an.stable, slowest_poles: slow, least_damped_poles: lowZ,
                         margins: an.channels.map(c => ({ loop: c.name, phase_margin_deg: c.pm ? r3(c.pm.pmDeg) : null, crossover_rad_s: c.pm ? r3(c.pm.w) : null,
                                                          gain_increase_margin_dB: c.gmUp ? r3(c.gmUp.gmDb) : 'infinite', gain_reduction_margin_dB: c.gmDown ? r3(c.gmDown.gmDb) : null })) };
            }
        };
        return {
            call(name, args) {
                try { if (!run[name]) return { error: 'unknown tool ' + name }; return run[name](args || {}); }
                catch (e) { return { error: e.message }; }
            },
            get sims() { return sims; }
        };
    }

    // ------------------------------------------------------------------ chat loop
    // Free-tier pacing: remember the rate-limit headers of the last response and wait for the
    // token window to refill before the next request; retry 429 responses after the advertised delay.
    const RL = { remainingTokens: null, resetTokensMs: 0, at: 0 };
    const sleep = (ms) => new Promise(r => setTimeout(r, ms));
    const parseDur = (v) => { if (!v) return 0; const m = String(v).match(/([\d.]+)\s*(ms|s|m)?/); if (!m) return 0; const x = +m[1]; return m[2] === 'ms' ? x : m[2] === 'm' ? x * 60000 : x * 1000; };
    async function chat({ apiKey, baseUrl = DEFAULTS.baseUrl, model = DEFAULTS.model, messages, tools, temperature = DEFAULTS.temperature, fetchImpl, onWait = () => {}, maxTokens = DEFAULTS.maxTokens }) {
        const f = fetchImpl || fetch;
        const body = { model, messages, temperature, max_completion_tokens: maxTokens };
        if (/gpt-oss/.test(model)) body.reasoning_effort = 'low';
        if (tools && tools.length) { body.tools = tools; body.tool_choice = 'auto'; }
        const est = Math.ceil(JSON.stringify(body).length / 3.5) + maxTokens;
        for (let attempt = 0; attempt < 5; attempt++) {
            if (RL.remainingTokens !== null && RL.remainingTokens < est) {
                const wait = Math.max(0, RL.resetTokensMs - (Date.now() - RL.at)) + 500;
                if (wait > 0) { onWait(wait); await sleep(wait); }
            }
            const res = await f(baseUrl.replace(/\/$/, '') + '/chat/completions', {
                method: 'POST', headers: { 'Content-Type': 'application/json', 'Authorization': 'Bearer ' + apiKey }, body: JSON.stringify(body) });
            const hdr = (k) => (res.headers && res.headers.get) ? res.headers.get(k) : null;
            const remTok = hdr('x-ratelimit-remaining-tokens');
            if (remTok !== null && remTok !== undefined) { RL.remainingTokens = +remTok; RL.resetTokensMs = parseDur(hdr('x-ratelimit-reset-tokens')); RL.at = Date.now(); }
            const txt = await res.text();
            let js; try { js = JSON.parse(txt); } catch (e) { js = null; }
            if (res.ok) return js;
            const msg = (js && js.error && js.error.message) || txt.slice(0, 300);
            if (res.status === 429 && !/per day|TPD|RPD/i.test(msg) && attempt < 4) {
                const m = msg.match(/try again in ([\d.]+m)?([\d.]+s)?/i);
                const wait = (parseDur(hdr('retry-after')) || ((m ? (parseDur(m[1]) + parseDur(m[2])) : 0)) || 15000) + 500;
                onWait(wait); await sleep(wait); continue;
            }
            const hint = res.status === 401 ? ' Check the API key.' : res.status === 429 ? ' The free-tier limit was reached; try again later.' : res.status === 413 ? ' The request is too large for this model\'s per-minute token limit.' : '';
            const err = new Error(`API error ${res.status}: ${msg}${hint}`); err.status = res.status; err.daily = /per day|TPD|RPD/i.test(msg); throw err;
        }
        throw new Error('API error: retries exhausted');
    }

    /**
     * Runs one tutoring turn: sends the conversation, executes requested tools, and repeats
     * until the model answers in text (or maxRounds). onEvent receives {type:'tool', name, args, result}
     * and {type:'answer', text}. Returns { text, messages, toolCalls }.
     */
    async function tutorTurn({ apiKey, baseUrl, model, history, userText, ctx, grounded = true, allowValues = false, maxRounds = DEFAULTS.maxRounds, onEvent = () => {}, fetchImpl }) {
        const usage = { prompt: 0, completion: 0, requests: 0 };
        const runner = makeToolRunner(ctx);
        const messages = [{ role: 'system', content: SYSTEM({ grounded, allowValues }) }, ...(history || []), { role: 'user', content: userText }];
        if (!history || !history.length) {
            // the design and latest run are given up front in both modes (tools add the ability to test)
            const r = ctx.getRun();
            messages.splice(1, 0, { role: 'user', content: 'Context (design and latest run):\n' + JSON.stringify({ design: summarizeDesign(ctx.getDesign()), run: r ? summarizeRun(r, ctx.specs, ctx.metricCfg, false) : null }) });
        }
        const toolCalls = [];
        for (let round = 0; round < maxRounds; round++) {
            const js = await chat({ apiKey, baseUrl, model, messages, tools: grounded ? TOOL_SPECS : null, fetchImpl, onWait: (ms) => onEvent({ type: 'wait', ms }) });
            usage.requests++; if (js.usage) { usage.prompt += js.usage.prompt_tokens || 0; usage.completion += js.usage.completion_tokens || 0; }
            const msg = js.choices && js.choices[0] && js.choices[0].message;
            if (!msg) throw new Error('Empty response from the model.');
            const calls = msg.tool_calls || [];
            if (!calls.length) {
                const text = msg.content || '';
                messages.push({ role: 'assistant', content: text });
                onEvent({ type: 'answer', text });
                return { text, messages: messages.slice(1), toolCalls, usage };
            }
            messages.push({ role: 'assistant', content: msg.content || '', tool_calls: calls });
            for (const c of calls) {
                let args = {};
                try { args = c.function.arguments ? JSON.parse(c.function.arguments) : {}; } catch (e) { args = {}; }
                const result = runner.call(c.function.name, args);
                toolCalls.push({ name: c.function.name, args, result });
                onEvent({ type: 'tool', name: c.function.name, args, result });
                messages.push({ role: 'tool', tool_call_id: c.id, content: JSON.stringify(result).slice(0, 6000) });
            }
        }
        const text = '(The tutor used its tool budget without a final answer. Try a narrower question.)';
        onEvent({ type: 'answer', text });
        return { text, messages: messages.slice(1), toolCalls, usage };
    }

    return { DEFAULTS, MODELS, SYSTEM, TOOL_SPECS, applyChanges, factoryFor, summarizeRun, summarizeDesign, makeToolRunner, chat, tutorTurn };
}));
