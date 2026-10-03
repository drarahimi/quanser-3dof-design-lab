/*
 * studio.js — Design Studio for the 3-DOF helicopter virtual lab.
 * Students enter a controller (built-in PID or LQR-I parameters, their own code, or a
 * block diagram), choose a test, run it instantly or live in the 3-D view, and check the
 * result against specifications, other runs and a linear analysis.
 * Depends on sim-core.js (Sim3DOF) and sim-design.js (Sim3DOFDesign).
 */
(function () {
    'use strict';
    const S = window.Sim3DOF, D = window.Sim3DOFDesign, DEG = Math.PI / 180;
    const COLORS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#4a3aa7', '#e34948'];
    const $ = (sel, el = document) => el.querySelector(sel);
    const h = (tag, attrs = {}, ...kids) => {
        const e = document.createElement(tag);
        for (const [k, v] of Object.entries(attrs || {})) {
            if (k === 'class') e.className = v;
            else if (k === 'style') e.setAttribute('style', v);
            else if (k.startsWith('on')) e.addEventListener(k.slice(2), v);
            else if (v !== undefined && v !== null && v !== false) e.setAttribute(k, v === true ? '' : v);
        }
        for (const c of kids.flat()) if (c !== null && c !== undefined && c !== false) e.append(c.nodeType ? c : document.createTextNode(String(c)));
        return e;
    };
    const fmt = (v, d = 2) => (v === null || v === undefined) ? 'n/a' : (!Number.isFinite(v) ? 'not settled' : (+v).toFixed(d));
    const store = {
        get(k, d) { try { const v = localStorage.getItem('3dof-studio-' + k); return v ? JSON.parse(v) : d; } catch (e) { return d; } },
        set(k, v) { try { localStorage.setItem('3dof-studio-' + k, JSON.stringify(v)); } catch (e) { /* storage unavailable */ } }
    };
    const download = (name, text, type = 'application/json') => {
        const a = document.createElement('a');
        a.href = URL.createObjectURL(new Blob([text], { type })); a.download = name; a.click();
        setTimeout(() => URL.revokeObjectURL(a.href), 1000);
    };
    const pickFile = (cb) => {
        const i = h('input', { type: 'file', accept: '.json,application/json' });
        i.addEventListener('change', () => { const f = i.files[0]; if (!f) return; f.text().then(cb); });
        i.click();
    };

    // ------------------------------------------------------------------ state
    const g0 = S.GAINS.pid;
    const DEFAULT_DESIGN = {
        type: 'pid',
        pid: { kp_e: +g0.kp_e.toFixed(4), ki_e: +g0.ki_e.toFixed(4), kd_e: +g0.kd_e.toFixed(4), kp_p: +g0.kp_p.toFixed(4), kd_p: +g0.kd_p.toFixed(4),
               kp_t: +g0.kp_t.toFixed(5), ki_t: +g0.ki_t.toFixed(6), kd_t: +g0.kd_t.toFixed(5), phi_ref_max: '20*deg', int_lim_e: 1, int_lim_t: 2 },
        lqri: { mode: 'QR', Q: [100, 1, 10, 0, 0, 2, 10, 0.1], R: [0.05, 0.05], K: S.GAINS.lqri.map(r => r.map(v => +v.toFixed(4))), int_lim_e: 1, int_lim_t: 2 },
        code: D.CODE_TEMPLATES.pid,
        diagram: D.diagramPID()
    };
    const st = {
        tab: 'controller',
        design: Object.assign(JSON.parse(JSON.stringify(DEFAULT_DESIGN)), store.get('design', {})),
        scenario: store.get('scenario', JSON.parse(JSON.stringify(D.SCENARIOS.takeoff_steps))),
        specs: store.get('specs', JSON.parse(JSON.stringify(D.DEFAULT_SPECS))),
        metricCfg: store.get('metricCfg', { settleBandPct: 2, settleFloorDeg: 0.2 }),
        runs: [], runCounter: 0, selectedRun: null, analysis: null,
        diagSel: null, diagZoom: null,   // null = fit the diagram to the available width
        ai: Object.assign({ key: '', remember: false, model: window.Sim3DOFTutor ? window.Sim3DOFTutor.DEFAULTS.model : 'openai/gpt-oss-120b', baseUrl: 'https://api.groq.com/openai/v1', grounded: true, allowValues: false },
                          store.get('ai', {}), { key: store.get('ai-key', '') || '', history: [], transcript: [], busy: false })
    };
    // a model saved by an older version may no longer be offered (e.g. retired from the free tier): fall back to the default
    if (window.Sim3DOFTutor && !window.Sim3DOFTutor.MODELS.includes(st.ai.model)) st.ai.model = window.Sim3DOFTutor.DEFAULTS.model;
    st.ai.remember = !!st.ai.key;
    for (const d of D.DEFAULT_SPECS) if (!st.specs.some(x => x.id === d.id)) st.specs.push(JSON.parse(JSON.stringify(d)));   // specs added in later versions
    st.metricCfg.health = Object.assign({ thrScale: 1, dwell: 0.25, eolEta: 0.6 }, st.metricCfg.health || {});
    const save = () => { store.set('design', st.design); store.set('scenario', st.scenario); store.set('specs', st.specs); store.set('metricCfg', st.metricCfg); };

    const designLabel = () => ({ pid: 'PID (built-in)', lqri: 'LQR-I (built-in)', code: 'Code', diagram: 'Block diagram' })[st.design.type];
    function controllerFactory() {
        const d = JSON.parse(JSON.stringify(st.design));
        switch (d.type) {
            case 'pid': return (P) => D.builtinPID(P, d.pid);
            case 'lqri': return (P) => D.builtinLQRI(P, d.lqri);
            case 'code': return (P) => D.compileCode(d.code, P);
            case 'diagram': return (P) => D.compileDiagram(d.diagram, P);
        }
    }

    // ------------------------------------------------------------------ shell
    let root, body, msgBar, live, lastFocus = null;
    function build() {
        root = h('div', { id: 'studio', role: 'dialog', 'aria-modal': 'true', 'aria-labelledby': 'studio-title', class: 'fixed inset-0 z-[300] hidden bg-slate-900/40 backdrop-blur-sm pointer-events-auto' });
        const panel = h('div', { class: 'studio-panel absolute inset-3 bg-white dark:bg-slate-900 rounded-2xl shadow-2xl border border-slate-200 dark:border-slate-700 flex flex-col overflow-hidden text-slate-800 dark:text-slate-100' });
        const tabs = [['controller', '1 · Controller'], ['test', '2 · Test'], ['specs', '3 · Specifications'], ['results', '4 · Results'], ['analysis', '5 · Linear analysis'], ['tutor', '6 · AI tutor']];
        const tabBar = h('div', { class: 'studio-tabs flex items-center gap-1', role: 'tablist', 'aria-label': 'Design Studio steps' }, tabs.map(([k, label]) =>
            h('button', { 'data-tab': k, id: 'studio-tab-' + k, role: 'tab', 'aria-controls': 'studio-body', class: 'studio-tab px-3 py-1.5 rounded-lg text-sm font-semibold', onclick: () => { st.tab = k; render(); } }, label)));
        // arrow keys move between tabs (WAI-ARIA tabs pattern, automatic activation)
        tabBar.addEventListener('keydown', (e) => {
            const keys = tabs.map(t => t[0]); let i = keys.indexOf(st.tab);
            if (e.key === 'ArrowRight') i = (i + 1) % keys.length; else if (e.key === 'ArrowLeft') i = (i + keys.length - 1) % keys.length;
            else if (e.key === 'Home') i = 0; else if (e.key === 'End') i = keys.length - 1; else return;
            e.preventDefault(); st.tab = keys[i]; render(); $('#studio-tab-' + st.tab).focus();
        });
        const header = h('div', { class: 'studio-header flex items-center justify-between gap-3 px-4 py-2.5 border-b border-slate-200 dark:border-slate-700 bg-slate-50 dark:bg-slate-800/60' },
            h('div', { class: 'studio-left flex items-center gap-4' },
                h('div', { class: 'studio-title' }, h('h2', { id: 'studio-title', class: 'text-sm font-bold whitespace-nowrap' }, 'Design Studio'),
                    h('div', { class: 'studio-sub text-[11px] text-slate-500 whitespace-nowrap' }, 'design · test · check · revise')),
                tabBar),
            h('div', { class: 'studio-actions flex items-center gap-2' },
                h('span', { id: 'studio-design-chip', class: 'text-xs font-semibold px-2 py-1 rounded bg-indigo-50 text-indigo-700 dark:bg-indigo-900/40 dark:text-indigo-200' }),
                h('button', { class: 'px-3 py-1.5 rounded-lg text-sm font-semibold bg-indigo-600 text-white hover:bg-indigo-700', onclick: () => runBatch(), title: 'Run the whole test instantly (1 kHz, headless) and check it' }, '▶ Simulate'),
                h('button', { class: 'px-3 py-1.5 rounded-lg text-sm font-semibold bg-emerald-600 text-white hover:bg-emerald-700', onclick: () => flyLive(), title: 'Run the same test in real time in the 3-D view' }, '✈ Fly in 3-D'),
                h('button', { class: 'px-2.5 py-1.5 rounded-lg text-sm font-semibold bg-slate-200 dark:bg-slate-700 hover:bg-slate-300', onclick: close, title: 'Close (Esc)', 'aria-label': 'Close Design Studio' }, '✕')));
        msgBar = h('div', { id: 'studio-msg', class: 'hidden px-4 py-2 text-sm border-b' });
        live = h('div', { id: 'studio-live', class: 'sr-only', role: 'status', 'aria-live': 'polite', 'aria-atomic': 'true' });
        body = h('div', { id: 'studio-body', class: 'flex-1 overflow-auto p-4', role: 'tabpanel' });
        panel.append(header, msgBar, live, body);
        root.append(panel);
        document.body.append(root);
        window.addEventListener('keydown', (e) => {
            if (root.classList.contains('hidden')) return;
            if (e.key === 'Escape') close();
            if ((e.key === 'Delete' || e.key === 'Backspace') && st.tab === 'controller' && st.design.type === 'diagram' && !/INPUT|TEXTAREA|SELECT/.test(document.activeElement.tagName)) {
                deleteDiagSelection(); e.preventDefault();
            }
        });
    }
    // modal behaviour: everything behind the studio is inert while it is open; focus moves in and is restored on close
    const setInert = (on) => [...document.body.children].forEach(c => { if (c !== root && c.id !== 'lab-tip' && c.tagName !== 'SCRIPT') c.inert = on; });
    function open(tab) {
        if (!root) build(); if (tab) st.tab = tab;
        const wasHidden = root.classList.contains('hidden');
        if (wasHidden) lastFocus = document.activeElement;
        root.classList.remove('hidden'); setInert(true); render();
        if (wasHidden) { const t = $('#studio-tab-' + st.tab); t && t.focus(); }
    }
    function close() {
        if (!root || root.classList.contains('hidden')) return;
        root.classList.add('hidden'); setInert(false);
        if (lastFocus && document.contains(lastFocus)) lastFocus.focus(); lastFocus = null;
    }
    function message(text, kind = 'info') {
        const cls = { info: 'bg-sky-50 text-sky-800 border-sky-200', ok: 'bg-emerald-50 text-emerald-800 border-emerald-200', err: 'bg-rose-50 text-rose-800 border-rose-200' }[kind];
        msgBar.className = 'px-4 py-2 text-sm border-b whitespace-pre-wrap ' + cls;
        msgBar.textContent = text;
        live.textContent = ''; setTimeout(() => { live.textContent = text; }, 50);   // announce to screen readers
    }
    function clearMessage() { msgBar.className = 'hidden'; }

    function render() {
        root.querySelectorAll('.studio-tab').forEach(b => {
            b.className = 'studio-tab px-3 py-1.5 rounded-lg text-sm font-semibold ' +
                (b.dataset.tab === st.tab ? 'bg-indigo-600 text-white' : 'text-slate-600 dark:text-slate-300 hover:bg-slate-200 dark:hover:bg-slate-700');
            const on = b.dataset.tab === st.tab;
            b.setAttribute('aria-selected', on ? 'true' : 'false'); b.tabIndex = on ? 0 : -1;
        });
        body.setAttribute('aria-labelledby', 'studio-tab-' + st.tab);
        $('#studio-design-chip').textContent = 'Design: ' + designLabel();
        // the body is rebuilt on every change: remember which control had focus and put focus back afterwards
        const FOC = 'button, input, select, textarea, a[href], [tabindex]:not([tabindex="-1"])';
        const a = document.activeElement, had = a && body.contains(a);
        const key = had ? { block: a.getAttribute('data-block'), label: a.getAttribute('aria-label'), text: (a.textContent || '').trim(), tag: a.tagName,
                            idx: [...body.querySelectorAll(FOC)].indexOf(a) } : null;
        body.innerHTML = '';
        ({ controller: renderController, test: renderTest, specs: renderSpecs, results: renderResults, analysis: renderAnalysis, tutor: renderTutor })[st.tab]();
        if (key) {
            const all = [...body.querySelectorAll(FOC)];
            const t = (key.block && body.querySelector(`[data-block="${key.block}"]`)) ||
                      (key.label && all.find(e => e.getAttribute('aria-label') === key.label)) ||
                      (key.text && all.find(e => e.tagName === key.tag && (e.textContent || '').trim() === key.text)) || all[Math.min(key.idx, all.length - 1)];
            if (t) t.focus({ preventScroll: true });
        }
    }

    // ------------------------------------------------------------------ small UI kit
    const card = (title, ...kids) => h('div', { class: 'rounded-xl border border-slate-200 dark:border-slate-700 p-3 bg-white dark:bg-slate-900' },
        title ? h('div', { class: 'text-xs font-bold uppercase tracking-wide text-slate-500 mb-2' }, title) : null, ...kids);
    const btn = (label, onclick, kind = 'plain', title) => h('button', {
        class: 'px-2.5 py-1 rounded-md text-sm font-semibold border ' + ({
            plain: 'bg-white dark:bg-slate-800 border-slate-300 dark:border-slate-600 hover:bg-slate-50',
            primary: 'bg-indigo-600 text-white border-indigo-600 hover:bg-indigo-700',
            danger: 'bg-white text-rose-700 border-rose-300 hover:bg-rose-50' })[kind], onclick, title,
        type: 'button', 'aria-label': (title && typeof label === 'string' && !/[A-Za-z]{2}/.test(label)) ? title : undefined }, label);
    const numField = (label, value, onchange, unit = '', hint = '') => h('label', { class: 'flex items-center justify-between gap-2 text-sm py-0.5' },
        h('span', { class: 'text-slate-600 dark:text-slate-300', title: hint }, label),
        h('span', { class: 'flex items-center gap-1' },
            h('input', { class: 'w-28 px-1.5 py-0.5 rounded border border-slate-300 dark:border-slate-600 bg-white dark:bg-slate-800 font-mono text-right text-sm', value: String(value),
                onchange: (e) => onchange(e.target.value) }),
            h('span', { class: 'w-12 text-xs text-slate-500' }, unit)));
    const parseVal = (v) => { const n = Number(v); return Number.isFinite(n) && String(v).trim() !== '' ? n : String(v).trim(); };

    // ------------------------------------------------------------------ 1. controller
    function renderController() {
        const types = [['pid', 'Cascaded PID', 'Edit the gains of the built-in cascaded PID.'],
                       ['lqri', 'LQR with integral action', 'Choose Q and R (K is solved in the browser) or type K.'],
                       ['code', 'Your own code', 'Write any control law in JavaScript.'],
                       ['diagram', 'Block diagram', 'Wire gains, sums, integrators and limits.']];
        const picker = h('div', { class: 'grid grid-cols-4 gap-2 mb-3' }, types.map(([k, t, d]) =>
            h('button', { class: 'text-left rounded-xl border p-2.5 ' + (st.design.type === k ? 'border-indigo-500 ring-2 ring-indigo-200 bg-indigo-50 dark:bg-indigo-900/30' : 'border-slate-200 dark:border-slate-700 hover:bg-slate-50'),
                onclick: () => { st.design.type = k; save(); clearMessage(); render(); } },
                h('div', { class: 'text-sm font-bold' }, t), h('div', { class: 'text-xs text-slate-500' }, d))));
        body.append(picker);
        ({ pid: renderPID, lqri: renderLQRI, code: renderCode, diagram: renderDiagram })[st.design.type]();
    }

    function renderPID() {
        const p = st.design.pid;
        const set = (k) => (v) => { p[k] = parseVal(v); save(); };
        const f = (k, label, unit, hint) => numField(label, p[k], set(k), unit, hint);
        const g = S.GAINS.pid;
        body.append(h('div', { class: 'grid grid-cols-3 gap-3' },
            card('Elevation loop → collective voltage',
                f('kp_e', 'K_p,θ', 'V/rad'), f('ki_e', 'K_i,θ', 'V/(rad s)'), f('kd_e', 'K_d,θ', 'V s/rad'), f('int_lim_e', 'integrator limit', 'rad s'),
                h('div', { class: 'text-xs text-slate-500 mt-1' }, 'V_s = 2 V_op(θ,φ) + K_p,θ e_θ + K_i,θ ∫e_θ − K_d,θ θ̇')),
            card('Travel loop → pitch reference',
                f('kp_t', 'K_p,ψ', 'rad/rad'), f('ki_t', 'K_i,ψ', '1/s'), f('kd_t', 'K_d,ψ', 's'), f('phi_ref_max', '|φ_r| limit', 'rad', 'e.g. 20*deg'), f('int_lim_t', 'integrator limit', 'rad s'),
                h('div', { class: 'text-xs text-slate-500 mt-1' }, 'φ_r = −(K_p,ψ e_ψ + K_i,ψ ∫e_ψ − K_d,ψ ψ̇), limited')),
            card('Pitch loop → cyclic voltage',
                f('kp_p', 'K_p,φ', 'V/rad'), f('kd_p', 'K_d,φ', 'V s/rad'),
                h('div', { class: 'text-xs text-slate-500 mt-1' }, 'V_d = K_p,φ (φ_r − φ) − K_d,φ φ̇;  V_f = (V_s+V_d)/2,  V_b = (V_s−V_d)/2'),
                h('div', { class: 'mt-3 flex gap-2' }, btn('Reset to design values', () => { st.design.pid = JSON.parse(JSON.stringify(DEFAULT_DESIGN.pid)); save(); render(); })),
                h('div', { class: 'text-xs text-slate-500 mt-2' }, `Design values place the closed-loop poles at ω = 1.5 (elevation), 4 (pitch) and 0.6 rad/s (travel). Fields accept expressions such as 20*deg.`))));
        void g;
    }

    function renderLQRI() {
        const q = st.design.lqri;
        const zN = ['θ−θ_r', 'φ', 'ψ−ψ_r', 'θ̇', 'φ̇', 'ψ̇', '∫e_θ', '∫e_ψ'];
        const modeSel = h('div', { class: 'flex gap-3 text-sm mb-2' },
            ...[['QR', 'Choose weights Q, R (K solved by Riccati)'], ['K', 'Enter the gain matrix K directly']].map(([m, t]) =>
                h('label', { class: 'flex items-center gap-1' }, h('input', { type: 'radio', name: 'lqmode', checked: q.mode === m, onchange: () => { q.mode = m; save(); render(); } }), t)));
        const qGrid = h('div', { class: 'grid grid-cols-4 gap-x-4' }, zN.map((n, i) => numField('Q[' + n + ']', q.Q[i], (v) => { q.Q[i] = parseVal(v); save(); })));
        const rGrid = h('div', { class: 'grid grid-cols-4 gap-x-4' }, ['V_f', 'V_b'].map((n, i) => numField('R[' + n + ']', q.R[i], (v) => { q.R[i] = parseVal(v); save(); })));
        const Kt = h('table', { class: 'text-xs font-mono' },
            h('tr', {}, h('th', {}, ''), zN.map(n => h('th', { class: 'px-1 text-slate-500 font-semibold' }, n))),
            ['V_f', 'V_b'].map((rn, r) => h('tr', {}, h('td', { class: 'pr-2 text-slate-500 font-semibold' }, rn),
                q.K[r].map((v, c) => h('td', {}, h('input', { class: 'w-20 px-1 py-0.5 border rounded text-right ' + (q.mode === 'K' ? 'bg-white' : 'bg-slate-100 text-slate-500'),
                    value: String(v), readonly: q.mode !== 'K', onchange: (e) => { q.K[r][c] = parseVal(e.target.value); save(); } }))))));
        const computeK = () => {
            try {
                const K = D.lqriGain(S.NOMINAL, q.Q.map(D.evalExpr), q.R.map(D.evalExpr));
                q.K = K.map(r => r.map(v => +v.toFixed(5))); save(); render();
                message('K solved from Q and R on the nominal linear model (augmented with the two integrators).', 'ok');
            } catch (e) { message('Could not solve the Riccati equation: ' + e.message + '\nQ must be ≥ 0 and R > 0.', 'err'); }
        };
        body.append(h('div', { class: 'grid grid-cols-3 gap-3' },
            h('div', { class: 'col-span-2 space-y-3' },
                card('Mode', modeSel),
                q.mode === 'QR' ? card('State weights Q (diagonal)', qGrid) : null,
                q.mode === 'QR' ? card('Input weights R (diagonal)', rGrid, h('div', { class: 'mt-2' }, btn('Solve for K', computeK, 'primary'))) : null,
                card('Gain matrix K  (u = V_op·1 − K z)', Kt)),
            card('About this controller',
                h('div', { class: 'text-sm space-y-2 text-slate-600 dark:text-slate-300' },
                    h('p', {}, 'The augmented state adds the integrals of the elevation and travel errors, so constant disturbances are rejected.'),
                    h('p', {}, 'In Q/R mode, K is recomputed from your weights when you simulate. Larger Q entries penalise that error more; larger R makes the controller use less voltage.'),
                    h('p', {}, 'Tip: compare a design with a large travel weight on the 90° travel test. Does it hit the ±32° pitch stop?')),
                numField('∫e_θ limit', q.int_lim_e, (v) => { q.int_lim_e = parseVal(v); save(); }, 'rad s'),
                numField('∫e_ψ limit', q.int_lim_t, (v) => { q.int_lim_t = parseVal(v); save(); }, 'rad s'))));
    }

    function renderCode() {
        const ta = h('textarea', { id: 'studio-code', spellcheck: 'false', class: 'w-full h-[52vh] font-mono text-[13px] leading-5 p-3 rounded-lg border border-slate-300 dark:border-slate-600 bg-slate-50 dark:bg-slate-950',
            oninput: (e) => { st.design.code = e.target.value; save(); },
            onkeydown: (e) => { if (e.key === 'Tab') { e.preventDefault(); const t = e.target, s0 = t.selectionStart; t.setRangeText('  ', s0, t.selectionEnd, 'end'); st.design.code = t.value; save(); } } });
        ta.value = st.design.code;
        const check = () => {
            try {
                const c = D.compileCode(st.design.code, S.NOMINAL);
                const u = c.step([0, 0, 0, 0, 0, 0], { theta: 0, psi: 0 }, 1e-3);
                message(`Code compiles. At hover with zero error it returns Vf = ${u[0].toFixed(3)} V, Vb = ${u[1].toFixed(3)} V (hover needs ${S.hoverVoltage(S.NOMINAL).toFixed(3)} V each). Memory has ${c.getState().length} numeric entries.`, 'ok');
            } catch (e) { message(e.message, 'err'); }
        };
        const tplSel = h('select', { class: 'border rounded px-1 py-0.5 text-sm', onchange: (e) => { if (!e.target.value) return; if (confirm('Replace your code with the template?')) { st.design.code = D.CODE_TEMPLATES[e.target.value]; save(); render(); } } },
            h('option', { value: '' }, 'Load template…'), h('option', { value: 'pid' }, 'Cascaded PID'), h('option', { value: 'lqri' }, 'LQR-I (Q, R → K)'), h('option', { value: 'blank' }, 'Blank'));
        body.append(h('div', { class: 'grid grid-cols-3 gap-3' },
            h('div', { class: 'col-span-2' }, h('div', { class: 'flex items-center gap-2 mb-2' }, btn('Check code', check, 'primary'), tplSel,
                h('span', { class: 'text-xs text-slate-500' }, 'Runs in your browser at 1 kHz. step() must return [Vf, Vb] in volts.')), ta),
            card('Reference',
                h('div', { class: 'text-xs leading-5 space-y-2 text-slate-600 dark:text-slate-300' },
                    h('div', {}, h('b', {}, 'function init(P, lib)'), ' runs once per test and returns your memory object (numbers or arrays of numbers).'),
                    h('div', {}, h('b', {}, 'function step(y, ref, dt, mem, P, lib)'), ' runs every 1 ms.'),
                    h('ul', { class: 'list-disc pl-4' },
                        h('li', {}, 'y = [θ, φ, ψ, θ̇, φ̇, ψ̇] (rad, rad/s); with hardware effects on, angles are encoder-quantised and rates are filtered'),
                        h('li', {}, 'ref = { theta, psi } in rad'), h('li', {}, 'dt = 0.001 s'),
                        h('li', {}, 'P: Je, Jp, Jt, La, Lh, m, g, Kf, Vmax, phMax, thMin, thMax')),
                    h('div', {}, h('b', {}, 'lib'), ': deg, clamp(v,lo,hi), wrap(a), hoverVoltage(θ,φ), lqriGain(Q,R), lqr(A,B,Q,R), linearize(), eigvals(M), mat.{mul,add,T,inv,solve,eye,zeros,diag}'),
                    h('div', {}, 'Keep every value that must persist (integrators, filters, previous errors) in mem. The linear analysis uses mem as the controller state.')))));
    }

    // ---------------------------------------------------------------- block diagram editor
    const BW = 120, BH = 34, PORT = 14;
    const blockH = (b) => Math.max(BH, 12 + Math.max(D.nIn(b), D.nOut(b)) * PORT);
    const portY = (b, i, n) => b.y + (blockH(b) / (n + 1)) * (i + 1);
    function renderDiagram() {
        const dg = st.design.diagram;
        const palette = h('div', { class: 'flex flex-wrap gap-1.5 mb-2' },
            Object.keys(D.BLOCK_TYPES).map(t => btn('+ ' + t, () => addBlock(t), 'plain', 'Add a ' + t + ' block')),
            h('span', { class: 'mx-2 border-l' }),
            btn('Validate', validateDiagram, 'primary'),
            btn('PID template', () => { if (confirm('Replace the diagram with the cascaded PID template?')) { st.design.diagram = D.diagramPID(); st.diagSel = null; save(); render(); } }),
            btn('Clear', () => { if (confirm('Remove all blocks?')) { st.design.diagram = blankDiagram(); st.diagSel = null; save(); render(); } }),
            btn('Export', () => download('diagram.json', JSON.stringify(st.design.diagram, null, 1))),
            btn('Import', () => pickFile(txt => { try { st.design.diagram = JSON.parse(txt); save(); render(); } catch (e) { message('Not a diagram file: ' + e.message, 'err'); } })),
            btn('−', () => { st.diagZoom = Math.max(0.3, diagZoomNow() - 0.1); render(); }, 'plain', 'Zoom out'), btn('+', () => { st.diagZoom = Math.min(1.5, diagZoomNow() + 0.1); render(); }, 'plain', 'Zoom in'),
            btn('Fit', () => { st.diagZoom = null; render(); }, 'plain', 'Fit the diagram to the width'));
        const W = 1700, Hh = 620;
        function diagZoomNow() {
            if (st.diagZoom != null) return st.diagZoom;
            const avail = (body.clientWidth - 40) * (window.innerWidth > 1100 ? 0.75 : 1);
            return Math.min(1.5, Math.max(0.3, avail / W));
        }
        const zoom = diagZoomNow();
        const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
        svg.setAttribute('viewBox', `0 0 ${W} ${Hh}`);
        svg.setAttribute('width', W * zoom); svg.setAttribute('height', Hh * zoom);
        svg.setAttribute('class', 'bg-[radial-gradient(circle,#e2e8f0_1px,transparent_1px)] [background-size:20px_20px] select-none');
        svg.id = 'studio-diagram'; svg.setAttribute('role', 'group'); svg.setAttribute('aria-label', 'Block diagram editor. Tab to a block to select or move it.');
        const NS = 'http://www.w3.org/2000/svg';
        const el = (t, a) => { const e = document.createElementNS(NS, t); for (const [k, v] of Object.entries(a)) e.setAttribute(k, v); return e; };
        const byId = Object.fromEntries(dg.blocks.map(b => [b.id, b]));
        // wires
        dg.wires.forEach((w, wi) => {
            const a = byId[w.from.id], b = byId[w.to.id]; if (!a || !b) return;
            const x1 = a.x + BW, y1 = portY(a, w.from.port, D.nOut(a)), x2 = b.x, y2 = portY(b, w.to.port, D.nIn(b));
            const mx = Math.max(x1 + 20, (x1 + x2) / 2);
            const sel = st.diagSel && st.diagSel.wire === wi;
            const path = el('path', { d: `M${x1},${y1} C${mx},${y1} ${Math.min(x2 - 20, mx)},${y2} ${x2},${y2}`, fill: 'none', stroke: sel ? '#e34948' : '#64748b', 'stroke-width': sel ? 3 : 1.6 });
            const hit = el('path', { d: path.getAttribute('d'), fill: 'none', stroke: 'transparent', 'stroke-width': 10, style: 'cursor:pointer' });
            hit.addEventListener('mousedown', (e) => { e.stopPropagation(); st.diagSel = { wire: wi }; render(); });
            svg.append(path, hit);
        });
        // blocks
        const colorOf = { source: '#2a78d6', const: '#2a78d6', output: '#1baf7a', mixer: '#1baf7a', integrator: '#4a3aa7', derivative: '#4a3aa7', saturation: '#eb6834', wrap: '#eb6834' };
        for (const b of dg.blocks) {
            const g = el('g', { transform: `translate(${b.x},${b.y})`, style: 'cursor:move', tabindex: 0, role: 'button', 'data-block': b.id,
                                'aria-pressed': st.diagSel && st.diagSel.block === b.id ? 'true' : 'false',
                                'aria-label': `Block ${b.id}, ${b.type}: ${D.BLOCK_TYPES[b.type].label(b)}. Enter selects it; arrow keys move it; connect inputs in the inspector.` });
            g.addEventListener('keydown', (e) => {
                const mv = { ArrowLeft: [-10, 0], ArrowRight: [10, 0], ArrowUp: [0, -10], ArrowDown: [0, 10] }[e.key];
                if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); st.diagSel = { block: b.id }; render(); }
                else if (mv) { e.preventDefault(); b.x = Math.max(0, b.x + mv[0] * (e.shiftKey ? 5 : 1)); b.y = Math.max(0, b.y + mv[1] * (e.shiftKey ? 5 : 1)); st.diagSel = { block: b.id }; save(); render(); }
            });
            const sel = st.diagSel && st.diagSel.block === b.id;
            const bh = blockH(b), col = colorOf[b.type] || '#52514e';
            g.append(el('rect', { width: BW, height: bh, rx: 6, fill: sel ? '#eef2ff' : '#ffffff', stroke: sel ? '#4f46e5' : col, 'stroke-width': sel ? 2.5 : 1.5 }));
            const lab = el('text', { x: BW / 2, y: bh / 2 + 4, 'text-anchor': 'middle', 'font-size': 12, 'font-family': 'Inter, sans-serif', fill: '#0f172a' });
            lab.textContent = D.BLOCK_TYPES[b.type].label(b);
            const idt = el('text', { x: 4, y: -4, 'font-size': 9, 'font-family': 'monospace', fill: '#94a3b8' }); idt.textContent = b.id;
            g.append(lab, idt);
            g.addEventListener('mousedown', (e) => startDrag(e, b, svg));
            svg.append(g);
            for (let i = 0; i < D.nIn(b); i++) {
                const c = el('circle', { cx: b.x, cy: portY(b, i, D.nIn(b)), r: 5, fill: '#fff', stroke: col, 'stroke-width': 1.5, style: 'cursor:crosshair', 'data-in': b.id + ':' + i });
                svg.append(c);
                if (b.type === 'sum') { const t = el('text', { x: b.x + 8, y: portY(b, i, D.nIn(b)) + 4, 'font-size': 11, fill: '#475569' }); t.textContent = b.params.signs[i]; svg.append(t); }
            }
            for (let i = 0; i < D.nOut(b); i++) {
                const c = el('circle', { cx: b.x + BW, cy: portY(b, i, D.nOut(b)), r: 5, fill: col, stroke: '#fff', 'stroke-width': 1.5, style: 'cursor:crosshair' });
                c.addEventListener('mousedown', (e) => startWire(e, b, i, svg));
                svg.append(c);
                if (b.type === 'mixer') { const t = el('text', { x: b.x + BW - 22, y: portY(b, i, 2) + 4, 'font-size': 10, fill: '#475569' }); t.textContent = i ? 'V_b' : 'V_f'; svg.append(t); }
            }
            if (b.type === 'mixer') ['V_s', 'V_d'].forEach((n, i) => { const t = el('text', { x: b.x + 8, y: portY(b, i, 2) + 4, 'font-size': 10, fill: '#475569' }); t.textContent = n; svg.append(t); });
        }
        svg.addEventListener('mousedown', (e) => { if (e.target === svg) { st.diagSel = null; render(); } });
        const canvasWrap = h('div', { class: 'overflow-auto border rounded-lg border-slate-200 dark:border-slate-700 h-[56vh] bg-slate-50' }, svg);
        body.append(palette, h('div', { class: 'grid grid-cols-4 gap-3' }, h('div', { class: 'col-span-3' }, canvasWrap,
            h('div', { class: 'text-xs text-slate-500 mt-1' }, 'Drag blocks to move them. Drag from an output port (filled) to an input port (open) to connect. Click a block or wire to select it; Delete removes it. Keyboard: Tab to a block, Enter selects it, arrow keys move it (Shift for larger steps), and the inspector connects its inputs. Integrators break feedback loops. Units: rad, rad/s, volts.')),
            renderBlockInspector()));
    }
    function blankDiagram() {
        return { name: 'My diagram', blocks: [
            { id: 'mix', type: 'mixer', x: 1300, y: 200, params: {} },
            { id: 'of', type: 'output', x: 1480, y: 170, params: { channel: 'Vf' } },
            { id: 'ob', type: 'output', x: 1480, y: 240, params: { channel: 'Vb' } }],
            wires: [{ from: { id: 'mix', port: 0 }, to: { id: 'of', port: 0 } }, { from: { id: 'mix', port: 1 }, to: { id: 'ob', port: 0 } }] };
    }
    function addBlock(type) {
        const dg = st.design.diagram;
        let n = 1; while (dg.blocks.some(b => b.id === type.slice(0, 3) + n)) n++;
        const id = type.slice(0, 3) + n;
        dg.blocks.push({ id, type, x: 80 + (dg.blocks.length * 37) % 600, y: 40 + (dg.blocks.length * 53) % 500, params: JSON.parse(JSON.stringify(D.BLOCK_TYPES[type].params)) });
        st.diagSel = { block: id }; save(); render();
    }
    function deleteDiagSelection() {
        const dg = st.design.diagram, s0 = st.diagSel; if (!s0) return;
        if (s0.wire !== undefined) dg.wires.splice(s0.wire, 1);
        if (s0.block) { dg.blocks = dg.blocks.filter(b => b.id !== s0.block); dg.wires = dg.wires.filter(w => w.from.id !== s0.block && w.to.id !== s0.block); }
        st.diagSel = null; save(); render();
    }
    const svgPoint = (svg, e) => { const p = svg.createSVGPoint(); p.x = e.clientX; p.y = e.clientY; return p.matrixTransform(svg.getScreenCTM().inverse()); };
    function startDrag(e, b, svg) {
        e.stopPropagation();
        const p0 = svgPoint(svg, e), bx = b.x, by = b.y; let moved = false;
        const mm = (ev) => { const p = svgPoint(document.getElementById('studio-diagram') || svg, ev); b.x = Math.round((bx + p.x - p0.x) / 10) * 10; b.y = Math.round((by + p.y - p0.y) / 10) * 10; moved = true; render(); };
        const mu = () => { window.removeEventListener('mousemove', mm); window.removeEventListener('mouseup', mu); st.diagSel = { block: b.id }; save(); render(); };
        window.addEventListener('mousemove', mm); window.addEventListener('mouseup', mu);
        if (!moved) { st.diagSel = { block: b.id }; }
    }
    function startWire(e, b, port, svg) {
        e.stopPropagation();
        const NS = 'http://www.w3.org/2000/svg';
        const x1 = b.x + BW, y1 = portY(b, port, D.nOut(b));
        const line = document.createElementNS(NS, 'line');
        line.setAttribute('x1', x1); line.setAttribute('y1', y1); line.setAttribute('x2', x1); line.setAttribute('y2', y1);
        line.setAttribute('stroke', '#4f46e5'); line.setAttribute('stroke-width', 2); line.setAttribute('stroke-dasharray', '5 4'); line.setAttribute('pointer-events', 'none');
        svg.append(line);
        const mm = (ev) => { const p = svgPoint(svg, ev); line.setAttribute('x2', p.x); line.setAttribute('y2', p.y); };
        const mu = (ev) => {
            window.removeEventListener('mousemove', mm); window.removeEventListener('mouseup', mu);
            const tgt = document.elementFromPoint(ev.clientX, ev.clientY);
            const key = tgt && tgt.getAttribute && tgt.getAttribute('data-in');
            line.remove();
            if (key) {
                const [id, p] = key.split(':'); const dg = st.design.diagram;
                dg.wires = dg.wires.filter(w => !(w.to.id === id && w.to.port === +p));
                dg.wires.push({ from: { id: b.id, port }, to: { id, port: +p } });
                save(); render();
            }
        };
        window.addEventListener('mousemove', mm); window.addEventListener('mouseup', mu);
    }
    function renderBlockInspector() {
        const dg = st.design.diagram, s0 = st.diagSel;
        const b = s0 && s0.block ? dg.blocks.find(x => x.id === s0.block) : null;
        if (!b) return card('Selected block', h('div', { class: 'text-sm text-slate-500' }, 'Select a block to edit its parameters.'),
            h('div', { class: 'text-xs text-slate-500 mt-3 leading-5' }, 'Signals available from source blocks: ' + D.SIGNALS.join(', ') + '. Vop is the per-motor hover voltage V_op(θ,φ).'));
        const fields = Object.keys(D.BLOCK_TYPES[b.type].params).map(k => {
            if (k === 'signal') return h('label', { class: 'flex justify-between text-sm py-0.5' }, 'signal',
                h('select', { class: 'border rounded px-1', onchange: (e) => { b.params.signal = e.target.value; save(); render(); } }, D.SIGNALS.map(s0 => h('option', { value: s0, selected: s0 === b.params.signal }, s0))));
            if (k === 'channel') return h('label', { class: 'flex justify-between text-sm py-0.5' }, 'channel',
                h('select', { class: 'border rounded px-1', onchange: (e) => { b.params.channel = e.target.value; save(); render(); } }, ['Vf', 'Vb'].map(s0 => h('option', { value: s0, selected: s0 === b.params.channel }, s0))));
            return numField(k, b.params[k], (v) => {
                if (k === 'signs') { b.params.signs = String(v).replace(/[^+-]/g, '') || '+'; const n = b.params.signs.length; dg.wires = dg.wires.filter(w => !(w.to.id === b.id && w.to.port >= n)); }
                else b.params[k] = parseVal(v);
                save(); render();
            });
        });
        // keyboard alternative to dragging wires: choose the source of each input port
        const outs = []; dg.blocks.forEach(x => { for (let i = 0; i < D.nOut(x); i++) outs.push(x.id + ':' + i); });
        const inputs = D.nIn(b) ? h('div', { class: 'mt-2 pt-2 border-t border-slate-200' }, h('div', { class: 'text-xs font-bold uppercase tracking-wide text-slate-500 mb-1' }, 'Inputs'),
            Array.from({ length: D.nIn(b) }, (_, i) => {
                const w = dg.wires.find(w => w.to.id === b.id && w.to.port === i), cur = w ? w.from.id + ':' + w.from.port : '';
                const nm = b.type === 'mixer' ? ['V_s', 'V_d'][i] : b.type === 'sum' ? (b.params.signs[i] || '+') + ' input ' + (i + 1) : 'input ' + (i + 1);
                return h('label', { class: 'flex justify-between items-center gap-2 text-sm py-0.5' }, nm,
                    h('select', { class: 'border rounded px-1 font-mono text-xs', 'aria-label': `Block ${b.id} ${nm}: connected from`,
                        onchange: (e) => { dg.wires = dg.wires.filter(x => !(x.to.id === b.id && x.to.port === i));
                            if (e.target.value) { const [id, p] = e.target.value.split(':'); dg.wires.push({ from: { id, port: +p }, to: { id: b.id, port: i } }); }
                            save(); render(); } },
                        h('option', { value: '', selected: !cur }, '(not connected)'),
                        outs.map(o => h('option', { value: o, selected: o === cur }, o.endsWith(':0') && D.nOut(dg.blocks.find(x => x.id === o.split(':')[0])) === 1 ? o.split(':')[0] : o))));
            })) : null;
        return card('Block ' + b.id + ' · ' + b.type, ...fields, inputs,
            h('div', { class: 'mt-2 flex gap-2' }, btn('Delete block', () => deleteDiagSelection(), 'danger')),
            h('div', { class: 'text-xs text-slate-500 mt-2' }, ({
                integrator: 'Output is the state; the input is integrated after the step (forward Euler) and clamped to ±lim.',
                derivative: 'Filtered derivative with bandwidth wc (rad/s).', sum: 'signs: one + or − per input, e.g. "+-+".',
                mixer: 'V_f = (V_s + V_d)/2, V_b = (V_s − V_d)/2.', saturation: 'Clamp between lo and hi (expressions allowed, e.g. -20*deg).'
            })[b.type] || ''));
    }
    function validateDiagram() {
        try { D.compileDiagram(st.design.diagram, S.NOMINAL); message('The diagram is valid: every input is connected, there is one Vf and one Vb output, and there are no algebraic loops.', 'ok'); }
        catch (e) { message('The diagram has problems:\n' + e.message, 'err'); }
    }

    // ------------------------------------------------------------------ 2. test
    function renderTest() {
        const sc = st.scenario;
        const set = (k, f = parseFloat) => (v) => { sc[k] = f(v); save(); render(); };
        const presetSel = h('select', { class: 'border rounded px-1 py-0.5 text-sm', 'aria-label': 'Load a standard test', onchange: (e) => { if (!e.target.value) return; st.scenario = JSON.parse(JSON.stringify(D.SCENARIOS[e.target.value])); save(); render(); } },
            h('option', { value: '' }, 'Load a standard test…'), Object.entries(D.SCENARIOS).map(([k, v]) => h('option', { value: k }, v.name)));
        const refLbl = { t: 'time (s)', theta: 'elevation reference (deg)', psi: 'travel reference (deg)', ramp: 'ramp duration (s)' };
        const refRows = sc.refs.map((r, i) => h('tr', {},
            ['t', 'theta', 'psi', 'ramp'].map(k => h('td', {}, h('input', { class: 'w-20 px-1 py-0.5 border rounded text-right font-mono text-sm', value: r[k] ?? 0,
                'aria-label': `Reference change ${i + 1}: ${refLbl[k]}`,
                onchange: (e) => { r[k] = parseFloat(e.target.value) || 0; save(); renderTestPreview(); } }))),
            h('td', {}, i > 0 ? btn('✕', () => { sc.refs.splice(i, 1); save(); render(); }, 'danger', `Remove reference change ${i + 1}`) : '')));
        const distRows = (sc.dists || []).map((q, i) => h('tr', {},
            h('td', {}, h('input', { class: 'w-16 px-1 py-0.5 border rounded text-right font-mono text-sm', value: q.t, 'aria-label': `Disturbance ${i + 1}: start time (s)`, onchange: (e) => { q.t = parseFloat(e.target.value) || 0; save(); } })),
            h('td', {}, h('input', { class: 'w-16 px-1 py-0.5 border rounded text-right font-mono text-sm', value: q.dur, 'aria-label': `Disturbance ${i + 1}: duration (s)`, onchange: (e) => { q.dur = parseFloat(e.target.value) || 0; save(); } })),
            h('td', {}, h('select', { class: 'border rounded px-1', 'aria-label': `Disturbance ${i + 1}: axis`, onchange: (e) => { q.axis = e.target.value; save(); } }, ['theta', 'phi', 'psi'].map(a => h('option', { value: a, selected: a === q.axis }, a)))),
            h('td', {}, h('input', { class: 'w-16 px-1 py-0.5 border rounded text-right font-mono text-sm', value: q.tau, 'aria-label': `Disturbance ${i + 1}: torque (N m)`, onchange: (e) => { q.tau = parseFloat(e.target.value) || 0; save(); } })),
            h('td', {}, btn('✕', () => { sc.dists.splice(i, 1); save(); render(); }, 'danger', `Remove disturbance ${i + 1}`))));
        const FT = { rotor: { label: 'Rotor thrust loss', targets: ['front', 'back', 'both'], unit: '% of thrust', scale: 100 },
                     friction: { label: 'Travel friction increase', targets: null, unit: 'N m s/rad', scale: 1 },
                     bias: { label: 'Encoder bias', targets: ['theta', 'phi', 'psi'], unit: 'deg', scale: 1 },
                     stuck: { label: 'Encoder stuck', targets: ['theta', 'phi', 'psi'], unit: '', scale: 1 } };
        const inp = (w, v, on, label) => h('input', { class: 'px-1 py-0.5 border rounded text-right font-mono text-sm', style: 'width:' + w, value: v, 'aria-label': label, onchange: (e) => { on(parseFloat(e.target.value) || 0); save(); renderTestPreview(); } });
        const faultRows = (sc.faults || []).map((f, i) => {
            const ft = FT[f.type] || FT.rotor;
            return h('tr', {},
                h('td', {}, h('select', { class: 'border rounded px-1 text-sm', 'aria-label': `Fault ${i + 1}: type`, onchange: (e) => { const ty = e.target.value; Object.assign(f, { type: ty, target: FT[ty].targets ? FT[ty].targets[0] : undefined,
                    size: { rotor: 0.3, friction: 0.05, bias: 2, stuck: 0 }[ty], ramp: 0 }); save(); render(); } },
                    Object.entries(FT).map(([k, v]) => h('option', { value: k, selected: k === f.type }, v.label)))),
                h('td', {}, ft.targets ? h('select', { class: 'border rounded px-1 text-sm', 'aria-label': `Fault ${i + 1}: where`, onchange: (e) => { f.target = e.target.value; save(); } }, ft.targets.map(a => h('option', { value: a, selected: a === f.target }, a))) : ''),
                h('td', {}, inp('3.2rem', f.t, v => { f.t = v; }, `Fault ${i + 1}: start time (s)`)),
                h('td', {}, f.type === 'stuck' ? '' : h('span', { class: 'whitespace-nowrap' }, inp('3.6rem', +(f.size * ft.scale).toFixed(4), v => { f.size = v / ft.scale; }, `Fault ${i + 1}: size (${ft.unit})`), h('span', { class: 'text-[11px] text-slate-500 ml-1' }, ft.unit === '% of thrust' ? '%' : ft.unit))),
                h('td', {}, f.type === 'stuck' ? '' : inp('3.2rem', f.ramp || 0, v => { f.ramp = v; }, `Fault ${i + 1}: ramp duration (s)`)),
                h('td', {}, btn('✕', () => { sc.faults.splice(i, 1); save(); render(); }, 'danger', `Remove fault ${i + 1}`)));
        });
        body.append(h('div', { class: 'grid grid-cols-3 gap-3' },
            h('div', { class: 'space-y-3' },
                card('Test', h('div', { class: 'mb-2' }, presetSel),
                    h('label', { class: 'flex justify-between text-sm py-0.5' }, 'Name', h('input', { class: 'w-44 px-1 border rounded text-sm', value: sc.name, onchange: (e) => { sc.name = e.target.value; save(); } })),
                    numField('Duration', sc.T, set('T'), 's'), numField('Initial elevation θ₀', sc.theta0, set('theta0'), 'deg', '−27.5 = resting on the lower stop'),
                    h('label', { class: 'flex justify-between text-sm py-0.5' }, 'Plant parameters',
                        h('select', { class: 'border rounded px-1', onchange: (e) => { sc.preset = e.target.value; save(); } },
                            h('option', { value: 'nominal', selected: sc.preset === 'nominal' }, 'Quanser nominal'), h('option', { value: 'identified', selected: sc.preset === 'identified' }, 'Identified rig'))),
                    h('label', { class: 'flex justify-between text-sm py-0.5' }, 'Hardware effects', h('input', { type: 'checkbox', checked: !!sc.hw, onchange: (e) => { sc.hw = e.target.checked; save(); } })))),
            h('div', { class: 'space-y-3' },
                card('Reference changes', h('table', { class: 'text-sm' },
                    h('tr', { class: 'text-xs text-slate-500' }, h('th', {}, 't (s)'), h('th', {}, 'θ_r (deg)'), h('th', {}, 'ψ_r (deg)'), h('th', {}, 'ramp (s)'), h('th', {})), refRows),
                    h('div', { class: 'mt-2' }, btn('+ Add change', () => { const l = sc.refs[sc.refs.length - 1]; sc.refs.push({ t: Math.min(sc.T, l.t + 10), theta: l.theta, psi: l.psi, ramp: 0 }); save(); render(); })),
                    h('div', { class: 'text-xs text-slate-500 mt-1' }, 'ramp = 0 gives a step; otherwise the reference moves linearly over that many seconds.')),
                card('Disturbance torques', h('table', { class: 'text-sm' },
                    h('tr', { class: 'text-xs text-slate-500' }, h('th', {}, 't (s)'), h('th', {}, 'for (s)'), h('th', {}, 'axis'), h('th', {}, 'τ (N m)'), h('th', {})), distRows),
                    h('div', { class: 'mt-2' }, btn('+ Add disturbance', () => { (sc.dists = sc.dists || []).push({ t: 10, dur: 0.5, axis: 'theta', tau: -0.3 }); save(); render(); }))),
                card('Faults (diagnosis and prognosis)', h('div', { class: 'overflow-x-auto' }, h('table', { class: 'text-sm' },
                    h('tr', { class: 'text-xs text-slate-500' }, h('th', {}, 'fault'), h('th', {}, 'where'), h('th', {}, 'from t (s)'), h('th', {}, 'size'), h('th', {}, 'ramp (s)'), h('th', {})), faultRows)),
                    h('div', { class: 'mt-2' }, btn('+ Add fault', () => { (sc.faults = sc.faults || []).push({ type: 'rotor', target: 'front', t: Math.round(sc.T / 2), size: 0.3, ramp: 0 }); save(); render(); })),
                    h('div', { class: 'text-xs text-slate-500 mt-1' }, 'ramp = 0 gives an abrupt fault; a ramp makes it grow linearly over that many seconds (degradation). A health monitor runs in every test; see Results.'))),
            card('Reference preview', h('canvas', { id: 'studio-test-preview', class: 'w-full', style: 'height:280px' }))));
        renderTestPreview();
    }
    function renderTestPreview() {
        const cv = $('#studio-test-preview'); if (!cv) return;
        const { ref } = D.scenarioFns(st.scenario);
        const t = [], th = [], ps = [];
        for (let k = 0; k <= 400; k++) { const tt = st.scenario.T * k / 400; const r = ref(tt); t.push(tt); th.push(r.theta / DEG); ps.push(r.psi / DEG); }
        plot(cv, [{ panel: 0, x: t, y: th, color: COLORS[3], label: 'θ_r' }, { panel: 1, x: t, y: ps, color: COLORS[0], label: 'ψ_r' }], ['θ_r (deg)', 'ψ_r (deg)'],
             { vlines: (st.scenario.faults || []).map(f => ({ x: f.t, color: '#dc2626', label: 'fault' })) });
    }

    // ------------------------------------------------------------------ 3. specs
    function renderSpecs() {
        const rows = st.specs.map(s0 => h('tr', { class: 'border-b border-slate-100 dark:border-slate-800' },
            h('td', { class: 'py-1' }, h('input', { type: 'checkbox', checked: s0.on, 'aria-label': `Check ${s0.label}`, onchange: (e) => { s0.on = e.target.checked; save(); } })),
            h('td', { class: 'pr-4 text-sm' }, s0.label),
            h('td', { class: 'text-sm text-slate-500 pr-2' }, '≤'),
            h('td', {}, h('input', { class: 'w-24 px-1 py-0.5 border rounded text-right font-mono text-sm', value: s0.limit, 'aria-label': `${s0.label} limit${s0.unit ? ' (' + s0.unit + ')' : ''}`, onchange: (e) => { s0.limit = parseFloat(e.target.value); save(); } })),
            h('td', { class: 'text-xs text-slate-500 pl-1' }, s0.unit)));
        body.append(h('div', { class: 'grid grid-cols-3 gap-3' },
            h('div', { class: 'col-span-2' }, card('Specifications (checked after every run)', h('table', {}, rows),
                h('div', { class: 'mt-3 flex gap-2' },
                    btn('Reset defaults', () => { st.specs = JSON.parse(JSON.stringify(D.DEFAULT_SPECS)); save(); render(); }),
                    btn('Export spec sheet', () => download('specs.json', JSON.stringify({ specs: st.specs, metricCfg: st.metricCfg }, null, 1))),
                    btn('Import spec sheet', () => pickFile(txt => { try { const o = JSON.parse(txt); st.specs = o.specs || o; if (o.metricCfg) st.metricCfg = o.metricCfg; save(); render(); message('Spec sheet loaded.', 'ok'); } catch (e) { message('Not a spec file: ' + e.message, 'err'); } }))))),
            card('How metrics are measured',
                numField('Settling band', st.metricCfg.settleBandPct, (v) => { st.metricCfg.settleBandPct = parseFloat(v); save(); }, '% of step'),
                numField('Band floor', st.metricCfg.settleFloorDeg, (v) => { st.metricCfg.settleFloorDeg = parseFloat(v); save(); }, 'deg'),
                h('ul', { class: 'text-xs text-slate-600 dark:text-slate-300 list-disc pl-4 mt-2 space-y-1' },
                    h('li', {}, 'Each reference change on an axis is one event; its window runs to the next change on that axis.'),
                    h('li', {}, 'Overshoot is relative to the step size. Settling time is the last exit from the band (not settled if the response is still outside it at the end of the window).'),
                    h('li', {}, 'Steady-state error is the mean absolute error over the last second of the window.'),
                    h('li', {}, 'Disturbance deviation is the peak departure from the angle held when the torque starts.'),
                    h('li', {}, 'Each spec takes the worst event of the test. Specs a test does not exercise show n/a.'),
                    h('li', {}, 'Instructors can export a spec sheet and share it with students.')),
                h('div', { class: 'text-xs font-bold uppercase tracking-wide text-slate-500 mt-4 mb-1' }, 'Health monitor'),
                numField('Threshold scale', st.metricCfg.health.thrScale, (v) => { st.metricCfg.health.thrScale = parseFloat(v); save(); }, '×', 'Multiplies the residual thresholds (0.02, 0.005, 0.01 N m on elevation, pitch, travel; ×1.5 with hardware effects)'),
                numField('Alarm dwell', st.metricCfg.health.dwell, (v) => { st.metricCfg.health.dwell = parseFloat(v); save(); }, 's', 'The statistic must stay above 1 this long to raise an alarm'),
                numField('End of life', st.metricCfg.health.eolEta * 100, (v) => { st.metricCfg.health.eolEta = parseFloat(v) / 100; save(); }, '% thrust', 'Remaining useful life is predicted to the time the estimated thrust effectiveness reaches this level'),
                h('div', { class: 'text-xs text-slate-500 mt-1' }, 'Detection specs apply only to tests that contain a fault.'))));
    }

    // ------------------------------------------------------------------ running
    function makeRunRecord(run, label) {
        const metrics = D.computeMetrics(run, Object.assign({}, st.metricCfg, { health: st.metricCfg.health }));
        const checks = D.checkSpecs(metrics, st.specs);
        const id = ++st.runCounter;
        return Object.assign(run, { id, label: label || `Run ${id} · ${designLabel()} · ${run.scenario.name}`, design: JSON.parse(JSON.stringify(st.design)),
            metrics, checks, color: COLORS[(id - 1) % COLORS.length], overlay: true });
    }
    function runBatch() {
        let ctrl;
        try { ctrl = controllerFactory()(Object.assign({}, S.PRESETS[st.scenario.preset || 'nominal'])); }
        catch (e) { message('The controller could not be built:\n' + e.message, 'err'); return; }
        const run = D.runScenario(ctrl, st.scenario, { healthCfg: st.metricCfg.health });
        const rec = makeRunRecord(run);
        st.runs.push(rec); st.selectedRun = rec.id;
        const np = rec.checks.filter(c => c.pass === true).length, nt = rec.checks.filter(c => c.pass !== null).length;
        if (run.error) message(`The controller stopped with an error at t = ${fmt(run.tFail, 3)} s:\n${run.error}`, 'err');
        else message(`${rec.label}: ${np}/${nt} specifications met.${run.alarm != null ? ` Health monitor alarm at ${fmt(run.alarm, 2)} s.` : ''} ${st.scenario.T} s simulated in ${run.wallMs.toFixed(0)} ms.`, np === nt ? 'ok' : 'info');
        st.tab = 'results'; render();
    }
    function flyLive() {
        let ctrl;
        try { ctrl = controllerFactory()(Object.assign({}, S.PRESETS[st.scenario.preset || 'nominal'])); }
        catch (e) { message('The controller could not be built:\n' + e.message, 'err'); return; }
        if (!window.Sim3DOFApp || !window.Sim3DOFApp.startLive) { message('The 3-D view is not available.', 'err'); return; }
        close();
        window.Sim3DOFApp.startLive({ controller: ctrl, scenario: JSON.parse(JSON.stringify(st.scenario)), label: designLabel(), healthCfg: st.metricCfg.health,
            onDone: (run) => {
                const rec = makeRunRecord(run, `Run ${st.runCounter + 1} · ${designLabel()} · ${run.scenario.name} (live)`);
                st.runs.push(rec); st.selectedRun = rec.id;
            } });
    }

    // ------------------------------------------------------------------ 4. results
    function renderResults() {
        if (!st.runs.length) { body.append(card(null, h('div', { class: 'text-sm text-slate-500' }, 'No runs yet. Choose a controller and a test, then press ▶ Simulate (instant) or ✈ Fly in 3-D (real time).'))); return; }
        const sel = st.runs.find(r => r.id === st.selectedRun) || st.runs[st.runs.length - 1];
        const list = card('Runs', h('div', { class: 'space-y-1 max-h-[60vh] overflow-auto' }, st.runs.slice().reverse().map(r => {
            const np = r.checks.filter(c => c.pass === true).length, nt = r.checks.filter(c => c.pass !== null).length;
            return h('div', { class: 'flex items-center gap-2 p-1.5 rounded-lg cursor-pointer ' + (r.id === sel.id ? 'bg-indigo-50 dark:bg-indigo-900/30' : 'hover:bg-slate-50'),
                onclick: () => { st.selectedRun = r.id; render(); } },
                h('input', { type: 'checkbox', checked: r.overlay, title: 'Overlay in the plots', 'aria-label': 'Overlay ' + r.label + ' in the plots', onclick: (e) => e.stopPropagation(), onchange: (e) => { r.overlay = e.target.checked; render(); } }),
                h('span', { class: 'inline-block w-3 h-3 rounded-full', style: 'background:' + r.color, 'aria-hidden': 'true' }),
                h('button', { type: 'button', class: 'flex-1 min-w-0 text-left', 'aria-pressed': r.id === sel.id ? 'true' : 'false', onclick: (e) => { e.stopPropagation(); st.selectedRun = r.id; render(); } },
                    h('span', { class: 'block text-xs font-semibold truncate' }, r.label),
                    h('span', { class: 'block text-[11px] text-slate-500' }, r.error ? 'error: ' + r.error.slice(0, 40) : `${np}/${nt} specs met`)),
                h('span', { class: 'text-[11px] font-bold px-1.5 rounded ' + (np === nt && !r.error ? 'bg-emerald-100 text-emerald-700' : 'bg-rose-100 text-rose-700') }, np === nt && !r.error ? 'PASS' : 'FAIL'),
                h('button', { type: 'button', class: 'text-slate-400 hover:text-rose-600 text-xs', title: 'Delete run', 'aria-label': 'Delete ' + r.label, onclick: (e) => { e.stopPropagation(); st.runs = st.runs.filter(x => x !== r); render(); } }, '✕'));
        })), h('div', { class: 'mt-2 flex flex-wrap gap-1.5' },
            btn('Export CSV', () => download(`run${sel.id}.csv`, runCSV(sel), 'text/csv')),
            btn('Export JSON', () => download(`run${sel.id}.json`, JSON.stringify(runJSON(sel), null, 1))),
            btn('Load design', () => { st.design = JSON.parse(JSON.stringify(sel.design)); save(); st.tab = 'controller'; render(); message('Loaded the design used in ' + sel.label + '.', 'ok'); })));
        const cv = h('canvas', { id: 'studio-results-plot', class: 'w-full', style: 'height:520px' });
        const checks = card('Specification check · ' + sel.label, h('table', { class: 'w-full text-sm' },
            h('tr', { class: 'text-xs text-slate-500 text-left' }, h('th', {}, 'Specification'), h('th', { class: 'text-right' }, 'Measured'), h('th', { class: 'text-right' }, 'Limit'), h('th', {})),
            sel.checks.map(c => h('tr', { class: 'border-b border-slate-100 dark:border-slate-800' },
                h('td', { class: 'py-0.5' }, c.label), h('td', { class: 'text-right font-mono' }, fmt(c.value) + (c.value !== null && Number.isFinite(c.value) ? ' ' + c.unit : '')),
                h('td', { class: 'text-right font-mono text-slate-500' }, c.limit + ' ' + c.unit),
                h('td', { class: 'pl-2 font-bold ' + (c.pass === null ? 'text-slate-400' : c.pass ? 'text-emerald-600' : 'text-rose-600') }, c.pass === null ? 'n/a' : c.pass ? '✓' : '✗')))));
        const ev = card('Events', h('table', { class: 'w-full text-xs' },
            h('tr', { class: 'text-slate-500 text-left' }, ['axis', 'type', 't (s)', 'from → to (deg)', 'OS %', 'rise (s)', 'settle (s)', 'ss err (deg)', 'peak dev (deg)'].map(x => h('th', { class: 'pr-2' }, x))),
            sel.metrics.events.map(e => h('tr', {}, h('td', {}, e.axis), h('td', {}, e.type), h('td', {}, fmt(e.t, 1)),
                h('td', {}, e.type === 'disturbance' ? '' : `${e.from} → ${e.to}`), h('td', {}, fmt(e.overshootPct, 1)), h('td', {}, fmt(e.riseTime)),
                h('td', {}, e.type === 'disturbance' ? fmt(e.recoveryTime) : fmt(e.settlingTime)), h('td', {}, fmt(e.steadyStateErrDeg, 3)), h('td', {}, fmt(e.maxDeviationDeg))))),
            h('div', { class: 'text-xs text-slate-500 mt-2' }, `Peak voltage ${fmt(sel.metrics.vmax)} V · saturated ${fmt(sel.metrics.satPct, 1)} % of the time · on pitch stop ${fmt(sel.metrics.pitchStopTime)} s · plant: ${sel.scenario.preset}${sel.scenario.hw ? ' + hardware effects' : ''}`));
        const hc = healthCard(sel);
        body.append(h('div', { class: 'grid grid-cols-4 gap-3' }, list,
            h('div', { class: 'col-span-3 space-y-3' }, card('Time histories (checked runs overlaid; dashed = reference of the selected run)', cv), h('div', { class: 'grid grid-cols-2 gap-3' }, checks, ev), hc ? hc.el : null)));
        if (hc) hc.draw();
        const series = [];
        const runs = st.runs.filter(r => r.overlay);
        for (const r of runs) {
            const t = r.t;
            series.push({ panel: 0, x: t, y: r.x.map(v => v[0] / DEG), color: r.color, w: r.id === sel.id ? 2 : 1.2 });
            series.push({ panel: 1, x: t, y: r.x.map(v => v[2] / DEG), color: r.color, w: r.id === sel.id ? 2 : 1.2 });
            series.push({ panel: 2, x: t, y: r.x.map(v => v[1] / DEG), color: r.color, w: r.id === sel.id ? 2 : 1.2 });
            series.push({ panel: 3, x: t, y: r.V.map(v => v[0]), color: r.color, w: 1 });
            series.push({ panel: 4, x: t, y: r.V.map(v => v[1]), color: r.color, w: 1 });
        }
        series.push({ panel: 0, x: sel.t, y: sel.ref.map(v => v[0] / DEG), color: '#0b0b0b', dash: [5, 4], w: 1 });
        series.push({ panel: 1, x: sel.t, y: sel.ref.map(v => v[1] / DEG), color: '#0b0b0b', dash: [5, 4], w: 1 });
        const P = sel.P;
        plot(cv, series, ['θ elevation (deg)', 'ψ travel (deg)', 'φ pitch (deg)', 'V_f (V)', 'V_b (V)'],
             { hlines: { 2: [P.phMax / DEG, -P.phMax / DEG], 3: [P.Vmax], 4: [P.Vmax] } });
    }
    // Health monitor view: detection statistic, residual torques, thrust-health estimate and RUL
    function healthCard(sel) {
        const hm = sel.metrics.health; if (!hm) return null;
        const faults = (sel.scenario.faults || []);
        const ft = { rotor: 'rotor thrust loss', friction: 'travel friction', bias: 'encoder bias', stuck: 'encoder stuck' };
        const injected = faults.length ? faults.map(f => `${f.target ? f.target + ' ' : ''}${ft[f.type]}` + (f.type === 'rotor' ? ` ${fmt(f.size * 100, 0)} %` : f.type === 'friction' ? ` +${fmt(f.size, 3)} N m s/rad` : f.type === 'bias' ? ` ${fmt(f.size, 2)}°` : '') +
            ` at ${fmt(f.t, 1)} s` + (f.ramp > 0 ? ` (over ${fmt(f.ramp, 0)} s)` : ' (abrupt)')).join('; ') : 'none (healthy run)';
        let est = '';
        if (hm.estimate && hm.isolatedAs) {
            const e = hm.estimate;
            if (/rotor/.test(hm.isolatedAs)) est = `estimated thrust loss: front ${fmt(e.dF * 100, 1)} %, back ${fmt(e.dB * 100, 1)} %`;
            else if (/friction/.test(hm.isolatedAs)) est = `estimated extra travel damping: ${fmt(e.dD, 4)} N m s/rad`;
            else est = `the rotor and friction signatures explain only ${fmt(Math.max(0, e.fit) * 100, 0)} % of the residual`;
        }
        const rulNow = (hm.prog && hm.prog.sustained) ? hm.rulTrace.filter(q => q.rul !== null) : [];
        const rows = [
            ['Injected', injected],
            ['Detection', hm.alarm === null ? 'no alarm' : `alarm at ${fmt(hm.alarm, 2)} s` + (hm.falseAlarm ? ' (false alarm: before any fault)' : hm.delay !== null ? `, ${fmt(hm.delay, 2)} s after the fault` : '')],
            ['Isolation', hm.isolatedAs ? hm.isolatedAs + (est ? ' · ' + est : '') : '—'],
            ['Prognosis', !(hm.prog && hm.prog.sustained) ? 'no sustained degradation trend, so no life prediction' :
                `end of life = ${fmt(hm.eolEta * 100, 0)} % thrust. ` + (hm.tEol !== null
                    ? (hm.prog.tAccurate !== null ? `The predicted remaining life stays within ±10 % of the truth from t = ${fmt(hm.prog.tAccurate, 0)} s, ${fmt(hm.prog.horizon, 0)} s before end of life (true end of life ${fmt(hm.tEol, 1)} s).`
                                                  : `The prediction never settles within ±10 % of the truth (true end of life ${fmt(hm.tEol, 1)} s).`)
                    : `Latest prediction: ${fmt(hm.prog.last.rul, 1)} s remaining at t = ${fmt(hm.prog.last.t, 0)} s.`)]
        ];
        const cvh = h('canvas', { class: 'w-full', style: 'height:' + (rulNow.length ? 420 : 300) + 'px' });
        const el = card('Health monitor · ' + sel.label,
            h('table', { class: 'w-full text-sm mb-2' }, rows.map(([k, v]) => h('tr', {}, h('td', { class: 'pr-3 py-0.5 text-slate-500 align-top whitespace-nowrap' }, k), h('td', {}, v)))),
            sel.scenario.preset !== 'nominal' ? h('div', { class: 'text-xs text-amber-700 bg-amber-50 border border-amber-200 rounded px-2 py-1 mb-2' },
                'This test uses the identified plant set, but the monitor uses the nominal model. Alarms here can come from the model mismatch itself: a monitor is only as good as its model.') : null,
            cvh,
            h('div', { class: 'text-xs text-slate-500 mt-2' }, 'The monitor knows only the nominal model, the commanded voltages and the measured angles and rates. r is the torque on each axis that the model cannot explain (a generalized-momentum observer); the statistic is r over its threshold, and an alarm needs it above 1 for the dwell time. Isolation fits r to the signatures of a thrust loss on each rotor and of extra travel friction. Note how the tracking plots above can look normal while r shows the fault: integral action hides it.'));
        const draw = () => {
            const t = sel.t, series = [
                { panel: 0, x: t, y: sel.s, color: '#4a3aa7', label: 'statistic' },
                { panel: 1, x: t, y: sel.r.map(v => v[0]), color: COLORS[3], label: 'r_θ' }, { panel: 1, x: t, y: sel.r.map(v => v[1] * 4), color: COLORS[6], label: 'r_φ ×4' },
                { panel: 1, x: t, y: sel.r.map(v => v[2] * 2), color: COLORS[0], label: 'r_ψ ×2' }];
            const labels = ['statistic', 'r (N m)'];
            const pts = []; (hm.etaHat || []).forEach((v, k) => { if (v !== null) pts.push(k); });
            series.push({ panel: 2, x: t, y: sel.eta.map(e => (e[0] + e[1]) / 2 * 100), color: '#0b0b0b', dash: [5, 4], w: 1, label: 'true (mean of the rotors)' });
            series.push({ panel: 2, x: pts.map(k => t[k] - (hm.lag || 0)), y: pts.map(k => hm.etaHat[k] * 100), color: '#dc2626', label: 'estimated' });
            labels.push('thrust (%)');
            if (rulNow.length) {
                series.push({ panel: 3, x: rulNow.map(q => q.t), y: rulNow.map(q => Math.min(q.rul, sel.scenario.T)), color: '#dc2626', label: 'predicted RUL' });
                if (hm.tEol !== null) series.push({ panel: 3, x: rulNow.map(q => q.t), y: rulNow.map(q => Math.max(0, hm.tEol - q.t)), color: '#0b0b0b', dash: [5, 4], w: 1, label: 'true' });
                labels.push('RUL (s)');
            }
            const vl = faults.map(f => ({ x: f.t, color: '#dc2626', label: 'fault' }));
            if (hm.alarm !== null) vl.push({ x: hm.alarm, color: '#d97706', label: 'alarm' });
            plot(cvh, series, labels, { hlines: { 0: [1], 2: [hm.eolEta * 100] }, vlines: vl });
        };
        return { el, draw };
    }
    function runCSV(r) {
        const head = 't_s,theta_deg,phi_deg,psi_deg,dtheta_deg_s,dphi_deg_s,dpsi_deg_s,Vf_V,Vb_V,theta_ref_deg,psi_ref_deg';
        return '# ' + r.label + '\n' + head + '\n' + r.t.map((t, k) => [t.toFixed(3), ...r.x[k].map(v => (v / DEG).toFixed(5)), r.V[k][0].toFixed(4), r.V[k][1].toFixed(4), (r.ref[k][0] / DEG).toFixed(4), (r.ref[k][1] / DEG).toFixed(4)].join(',')).join('\n');
    }
    function runJSON(r) {
        return { version: S.VERSION, label: r.label, design: r.design, scenario: r.scenario, specs: st.specs, metricCfg: st.metricCfg,
                 metrics: r.metrics, checks: r.checks.map(c => ({ id: c.id, label: c.label, value: Number.isFinite(c.value) ? c.value : String(c.value), limit: c.limit, unit: c.unit, pass: c.pass })),
                 error: r.error, live: !!r.live };
    }

    // ------------------------------------------------------------------ 5. analysis
    function renderAnalysis() {
        const run = () => {
            try {
                const t0 = performance.now();
                st.analysis = D.linearAnalysis(controllerFactory(), st.scenario.preset || 'nominal');
                st.analysis.ms = performance.now() - t0; st.analysis.design = designLabel();
                clearMessage(); render();
            } catch (e) { message('Linear analysis failed: ' + e.message, 'err'); }
        };
        body.append(h('div', { class: 'flex items-center gap-2 mb-3' }, btn('Analyse current design', run, 'primary'),
            h('span', { class: 'text-xs text-slate-500' }, 'Linearises the closed loop at hover (1 kHz step map, ideal sensing, no saturation) for any controller type, including your code and diagrams.')));
        const a = st.analysis; if (!a) return;
        const poles = a.poles;
        const pz = h('div', { id: 'studio-splane' });
        const tbl = h('table', { class: 'text-xs w-full' }, h('tr', { class: 'text-slate-500 text-left' }, ['Re (1/s)', 'Im (rad/s)', 'ω_n (rad/s)', 'ζ'].map(x => h('th', {}, x))),
            poles.filter(p => p.im >= -1e-9).map(p => h('tr', { class: p.re >= 0 ? 'text-rose-600 font-bold' : '' }, h('td', { class: 'font-mono' }, p.re.toFixed(3)), h('td', { class: 'font-mono' }, Math.abs(p.im) < 1e-9 ? '0' : '±' + Math.abs(p.im).toFixed(3)), h('td', { class: 'font-mono' }, p.wn.toFixed(3)), h('td', { class: 'font-mono' }, p.zeta.toFixed(3)))));
        const mt = h('table', { class: 'text-xs w-full' }, h('tr', { class: 'text-slate-500 text-left' }, ['Loop (broken at)', 'PM (deg) @ ω', 'GM↑ (dB) @ ω', 'GM↓ (dB) @ ω'].map(x => h('th', {}, x))),
            a.channels.map(c => h('tr', {}, h('td', {}, c.name),
                h('td', { class: 'font-mono' }, c.pm ? `${c.pm.pmDeg.toFixed(1)} @ ${c.pm.w.toFixed(2)}` : '—'),
                h('td', { class: 'font-mono' }, c.gmUp ? `${c.gmUp.gmDb.toFixed(1)} @ ${c.gmUp.w.toFixed(2)}` : '∞'),
                h('td', { class: 'font-mono' }, c.gmDown ? `${c.gmDown.gmDb.toFixed(1)} @ ${c.gmDown.w.toFixed(2)}` : '—'))));
        const bode = h('canvas', { id: 'studio-bode', class: 'w-full', style: 'height:420px' });
        body.append(h('div', { class: 'grid grid-cols-3 gap-3' },
            h('div', { class: 'space-y-3' },
                card(`Closed loop: ${a.stable ? 'STABLE' : 'UNSTABLE'} · ${a.design}`,
                    h('div', { class: 'text-sm font-bold ' + (a.stable ? 'text-emerald-600' : 'text-rose-600') }, a.stable ? 'All poles in the left half-plane.' : 'At least one pole is unstable.'),
                    h('div', { class: 'text-xs text-slate-500 mt-1' }, `${a.poles.length} poles = 6 plant + ${a.nc} controller states` + (a.nParams ? ` (${a.nParams} constant memory entries treated as parameters)` : '') +
                        `. Operating point drift ${a.operatingPoint.drift.toExponential(1)}; ${a.ms.toFixed(0)} ms.`),
                    a.operatingPoint.drift > 1e-6 ? h('div', { class: 'text-xs text-amber-700 mt-1' }, 'The controller did not settle to a steady hover in 60 s, so the linearisation point may be inaccurate.') : null),
                card('s-plane', pz), card('Poles', tbl)),
            h('div', { class: 'col-span-2 space-y-3' }, card('Loop margins (one loop broken, the other closed)', mt,
                h('div', { class: 'text-xs text-slate-500 mt-1' }, 'Collective = common voltage (elevation); cyclic = difference voltage (pitch and, through it, travel). GM↓ is the gain-reduction margin typical of loops around double integrators.')),
                card('Loop transfer functions L(jω)', bode))));
        drawSPlane(pz, poles);
        const ser = [];
        a.channels.forEach((c, i) => { ser.push({ panel: 0, x: c.w, y: c.magDb, color: COLORS[i], label: c.name, w: 1.6 }); ser.push({ panel: 1, x: c.w, y: c.phaseDeg, color: COLORS[i], w: 1.6 }); });
        plot(bode, ser, ['|L| (dB)', '∠L (deg)'], { logx: true, hlines: { 0: [0], 1: [-540, -180, 180] }, xlabel: 'ω (rad/s)', xmax: 300 });
    }
    function drawSPlane(container, poles) {
        const W = 360, Hh = 260, NS = 'http://www.w3.org/2000/svg';
        const re = poles.map(p => p.re), im = poles.map(p => p.im);
        const xmin = Math.min(-0.5, ...re.filter(v => v > -50)) * 1.15, xmax = Math.max(0.3, ...re) * 1.15 + 0.2;
        const ymax = Math.max(1, ...im.map(Math.abs).filter(v => v < 50)) * 1.2;
        const sx = (v) => 30 + (v - xmin) / (xmax - xmin) * (W - 40), sy = (v) => Hh / 2 - v / ymax * (Hh / 2 - 15);
        const svg = document.createElementNS(NS, 'svg'); svg.setAttribute('viewBox', `0 0 ${W} ${Hh}`); svg.setAttribute('class', 'w-full');
        svg.setAttribute('role', 'img'); svg.setAttribute('aria-label', `s-plane map of ${poles.length} closed-loop poles, ${poles.filter(p => p.re >= 0).length} in the right half-plane. The poles are listed in the table below.`);
        const el = (t, a) => { const e = document.createElementNS(NS, t); for (const [k, v] of Object.entries(a)) e.setAttribute(k, v); return e; };
        svg.append(el('rect', { x: sx(0), y: 0, width: W - sx(0), height: Hh, fill: '#fde8e8' }));
        svg.append(el('line', { x1: 30, y1: sy(0), x2: W, y2: sy(0), stroke: '#94a3b8' }), el('line', { x1: sx(0), y1: 0, x2: sx(0), y2: Hh, stroke: '#94a3b8' }));
        const tx = (x, y, s0, a = 'middle') => { const t = el('text', { x, y, 'font-size': 10, fill: '#52514e', 'text-anchor': a }); t.textContent = s0; svg.append(t); };
        tx(sx(xmin) + 4, sy(0) - 4, xmin.toFixed(1), 'start'); tx(sx(0) - 3, 12, 'Im ' + ymax.toFixed(1), 'end'); tx(W - 6, sy(0) - 4, 'Re', 'end');
        let off = 0;
        for (const p of poles) {
            if (p.re < -50 || Math.abs(p.im) > 50) { off++; continue; }
            const x = sx(p.re), y = sy(p.im), c = p.re >= 0 ? '#e34948' : '#2a78d6';
            svg.append(el('path', { d: `M${x - 5},${y - 5}L${x + 5},${y + 5}M${x - 5},${y + 5}L${x + 5},${y - 5}`, stroke: c, 'stroke-width': 2 }));
        }
        if (off) tx(W - 6, Hh - 6, `${off} fast pole(s) off scale`, 'end');
        container.append(svg);
    }

    // ------------------------------------------------------------------ plotting (canvas, stacked panels)
    // The canvas bitmap is sized from the canvas's on-screen size. On the first visit to a tab, the Tailwind
    // CDN has not yet generated CSS for that tab's classes (w-full, grid-cols-4, col-span-3, ...) when we draw,
    // so the size read here is wrong and the bitmap is then stretched once the CSS lands. Redraw whenever the
    // canvas's displayed size changes, so the plot always matches its box.
    function plot(cv, series, labels, opt = {}) {
        cv._plotArgs = [series, labels, opt];
        drawPlot(cv, series, labels, opt);
        if (!cv._ro && window.ResizeObserver) {
            let lw = cv.clientWidth, lh = cv.clientHeight;
            cv._ro = new ResizeObserver(() => {
                if (!cv.isConnected) { cv._ro.disconnect(); return; }
                if (cv.clientWidth === lw && cv.clientHeight === lh) return;
                lw = cv.clientWidth; lh = cv.clientHeight;
                drawPlot(cv, ...cv._plotArgs);
            });
            cv._ro.observe(cv);
        }
    }
    function drawPlot(cv, series, labels, opt = {}) {
        const dpr = window.devicePixelRatio || 1, W = cv.clientWidth || 800, Hh = cv.clientHeight || 300;
        cv.width = W * dpr; cv.height = Hh * dpr;
        const ctx = cv.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, W, Hh);
        // text alternative: what is plotted; the numbers are in the adjacent tables and in the CSV/JSON export
        const names = [...new Set(series.filter(s0 => s0.label).map(s0 => s0.label))];
        cv.setAttribute('role', 'img');
        cv.setAttribute('aria-label', 'Plot of ' + labels.join(', ') + ' versus ' + (opt.xlabel || 't (s)') + (names.length ? '; series: ' + names.join(', ') : '') +
            '. The values are listed in the tables on this tab and can be exported as CSV.');
        const np = labels.length, L = 62, R = 12, TOP = 8, BOT = 26, gap = 10, ph = (Hh - TOP - BOT - gap * (np - 1)) / np;
        let x0 = Infinity, x1 = -Infinity;
        for (const s0 of series) for (const v of s0.x) { if (v < x0) x0 = v; if (v > x1) x1 = v; }
        if (opt.logx) { x0 = Math.max(x0, 1e-3); if (opt.xmax) x1 = Math.min(x1, opt.xmax); }
        const fx = opt.logx ? (v) => L + (Math.log10(v) - Math.log10(x0)) / (Math.log10(x1) - Math.log10(x0)) * (W - L - R) : (v) => L + (v - x0) / (x1 - x0 || 1) * (W - L - R);
        const css = getComputedStyle(document.documentElement);
        const ink = '#52514e', grid = '#e4e3df';
        ctx.font = '11px Inter, sans-serif';
        for (let p = 0; p < np; p++) {
            const top = TOP + p * (ph + gap);
            let y0 = Infinity, y1 = -Infinity;
            for (const s0 of series) if (s0.panel === p) s0.y.forEach((v, i) => { if (opt.logx && (s0.x[i] < x0 || s0.x[i] > x1)) return; if (Number.isFinite(v)) { if (v < y0) y0 = v; if (v > y1) y1 = v; } });
            ((opt.hlines || {})[p] || []).forEach(v => { if (v > y0 - 0.3 * (y1 - y0 + 1) && v < y1 + 0.3 * (y1 - y0 + 1)) { y0 = Math.min(y0, v); y1 = Math.max(y1, v); } });
            if (!Number.isFinite(y0)) { y0 = -1; y1 = 1; }
            if (y1 - y0 < 1e-9) { y0 -= 1; y1 += 1; }
            const pad = 0.08 * (y1 - y0); y0 -= pad; y1 += pad;
            const fy = (v) => top + ph - (v - y0) / (y1 - y0) * ph;
            ctx.strokeStyle = grid; ctx.lineWidth = 1; ctx.strokeRect(L, top, W - L - R, ph);
            const step = niceStep((y1 - y0) / 3);
            ctx.fillStyle = ink; ctx.textAlign = 'right';
            for (let v = Math.ceil(y0 / step) * step; v <= y1; v += step) { const yy = fy(v); ctx.beginPath(); ctx.moveTo(L, yy); ctx.lineTo(W - R, yy); ctx.stroke(); ctx.fillText(+v.toFixed(6) + '', L - 4, yy + 4); }
            ((opt.hlines || {})[p] || []).forEach(v => { if (v < y0 || v > y1) return; ctx.save(); ctx.strokeStyle = '#b4b2a8'; ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.moveTo(L, fy(v)); ctx.lineTo(W - R, fy(v)); ctx.stroke(); ctx.restore(); });
            (opt.vlines || []).forEach(m => { if (m.x < x0 || m.x > x1) return; ctx.save(); ctx.strokeStyle = m.color; ctx.lineWidth = 1.2; ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(fx(m.x), top); ctx.lineTo(fx(m.x), top + ph); ctx.stroke();
                if (p === 0 && m.label) { ctx.fillStyle = m.color; ctx.textAlign = 'left'; ctx.fillText(m.label, fx(m.x) + 3, top + ph - 5); } ctx.restore(); });
            ctx.save(); ctx.translate(12, top + ph / 2); ctx.rotate(-Math.PI / 2); ctx.textAlign = 'center'; ctx.fillText(labels[p], 0, 0); ctx.restore();
            ctx.save(); ctx.beginPath(); ctx.rect(L, top, W - L - R, ph); ctx.clip();
            for (const s0 of series) if (s0.panel === p) {
                ctx.strokeStyle = s0.color; ctx.lineWidth = s0.w || 1.5; ctx.setLineDash(s0.dash || []); ctx.beginPath();
                let first = true;
                for (let i = 0; i < s0.x.length; i++) {
                    const xv = s0.x[i]; if (opt.logx && (xv < x0 || xv > x1)) continue;
                    const X = fx(xv), Y = fy(s0.y[i]); if (!Number.isFinite(Y)) continue;
                    if (first) { ctx.moveTo(X, Y); first = false; } else ctx.lineTo(X, Y);
                }
                ctx.stroke();
            }
            ctx.restore();
        }
        ctx.fillStyle = ink; ctx.textAlign = 'center'; ctx.setLineDash([]);
        const yb = Hh - 8;
        if (opt.logx) { for (let e = Math.ceil(Math.log10(x0)); e <= Math.log10(x1); e++) ctx.fillText('10^' + e, fx(Math.pow(10, e)), yb); }
        else { const st0 = niceStep((x1 - x0) / 8); for (let v = Math.ceil(x0 / st0) * st0; v <= x1 + 1e-9; v += st0) ctx.fillText(+v.toFixed(3) + '', fx(v), yb); }
        ctx.textAlign = 'right'; ctx.fillText(opt.xlabel || 't (s)', W - R, yb - 12);
        const leg = series.filter(s0 => s0.label);
        let lx = L + 8; ctx.textAlign = 'left';
        for (const s0 of leg) { ctx.fillStyle = s0.color; ctx.fillRect(lx, TOP + 6, 14, 3); ctx.fillStyle = ink; ctx.fillText(s0.label, lx + 18, TOP + 11); lx += ctx.measureText(s0.label).width + 34; }
        void css;
    }
    function niceStep(raw) { const p = Math.pow(10, Math.floor(Math.log10(Math.abs(raw) || 1))); const f = raw / p; return (f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10) * p; }

    // ------------------------------------------------------------------ 6. AI tutor
    const T = window.Sim3DOFTutor;
    const saveAI = () => {
        const { key, history, transcript, busy, ...cfg } = st.ai; store.set('ai', cfg);
        try { if (st.ai.remember && st.ai.key) localStorage.setItem('3dof-studio-ai-key', JSON.stringify(st.ai.key)); else localStorage.removeItem('3dof-studio-ai-key'); } catch (e) { /* storage unavailable */ }
    };
    const esc = (t) => String(t).replace(/[&<>]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' })[c]);
    const md = (t) => esc(t).replace(/\*\*(.+?)\*\*/g, '<b>$1</b>').replace(/`([^`]+)`/g, '<code class="px-1 bg-slate-100 rounded">$1</code>')
        .replace(/^\s*[-*] (.*)$/gm, '• $1').replace(/\n/g, '<br>');
    function tutorCtx() {
        return { getDesign: () => st.design, getScenario: () => st.scenario, specs: st.specs, metricCfg: st.metricCfg,
                 getRun: (id) => (id ? st.runs.find(r => r.id === id) : (st.runs.find(r => r.id === st.selectedRun) || st.runs[st.runs.length - 1])) };
    }
    function toolSummary(tc) {
        const r = tc.result || {};
        if (r.error) return 'error: ' + r.error;
        if (tc.name === 'simulate_variant' || tc.name === 'get_run') {
            const sp = (r.specs || []).filter(x => / (PASS|FAIL)$/.test(x)); const np = sp.filter(x => / PASS$/.test(x)).length;
            return `${np}/${sp.length} specs met`;
        }
        if (tc.name === 'linear_analysis') return (r.stable ? 'stable' : 'UNSTABLE') + (r.margins ? ', PM ' + r.margins.map(m => m.phase_margin_deg).join(' / ') + ' deg' : '');
        return 'ok';
    }
    async function askTutor(text) {
        const ai = st.ai;
        if (!T) { message('tutor-core.js is not loaded.', 'err'); return; }
        if (!ai.key) { message('Paste your Groq API key first (free at console.groq.com).', 'err'); return; }
        if (!text.trim() || ai.busy) return;
        ai.busy = true;
        const entry = { role: 'assistant', text: '', tools: [], pending: true };
        ai.transcript.push({ role: 'user', text }, entry);
        render();
        try {
            const res = await T.tutorTurn({ apiKey: ai.key, baseUrl: ai.baseUrl, model: ai.model, history: ai.history, userText: text, ctx: tutorCtx(),
                grounded: ai.grounded, allowValues: ai.allowValues,
                onEvent: (ev) => { if (ev.type === 'tool') { entry.tools.push(ev); entry.wait = null; if (st.tab === 'tutor') render(); } if (ev.type === 'wait') { entry.wait = Math.round(ev.ms / 1000); if (st.tab === 'tutor') render(); } } });
            entry.text = res.text; ai.history = res.messages.filter(m => !(m.role === 'user' && String(m.content).startsWith('Context (design and latest run)')));
        } catch (e) { entry.text = ''; entry.error = e.message + (/Failed to fetch|NetworkError/i.test(e.message) ? ' (network or browser blocked the request; check your connection)' : ''); }
        entry.pending = false; ai.busy = false;
        if (st.tab === 'tutor') render();
    }
    function renderTutor() {
        const ai = st.ai;
        const run = tutorCtx().getRun();
        const settings = card('Tutor settings',
            h('label', { class: 'block text-sm mb-1' }, 'Groq API key',
                h('input', { type: 'password', class: 'mt-1 w-full px-2 py-1 border rounded font-mono text-sm', placeholder: 'gsk_…', value: ai.key, autocomplete: 'off',
                    onchange: (e) => { ai.key = e.target.value.trim(); saveAI(); render(); } })),
            h('label', { class: 'flex items-center gap-2 text-xs text-slate-600 mb-2' }, h('input', { type: 'checkbox', checked: ai.remember, onchange: (e) => { ai.remember = e.target.checked; saveAI(); } }), 'Remember the key in this browser'),
            h('div', { class: 'text-xs text-slate-500 mb-2' }, 'Get a free key at ', h('a', { href: 'https://console.groq.com/keys', target: '_blank', class: 'text-indigo-600 underline' }, 'console.groq.com/keys'), '. The free tier limits tokens per minute and per day; the tutor waits automatically when the per-minute limit is reached.'),
            h('label', { class: 'flex justify-between items-center text-sm py-0.5' }, 'Model',
                h('select', { class: 'border rounded px-1 text-sm', onchange: (e) => { ai.model = e.target.value; saveAI(); } },
                    T ? T.MODELS.map(m => h('option', { value: m, selected: m === ai.model }, m)) : null)),
            h('label', { class: 'flex justify-between items-center text-sm py-0.5' }, 'Endpoint',
                h('input', { class: 'w-52 px-1 border rounded text-xs font-mono', value: ai.baseUrl, onchange: (e) => { ai.baseUrl = e.target.value.trim(); saveAI(); } })),
            h('label', { class: 'flex items-center gap-2 text-sm py-1' }, h('input', { type: 'checkbox', checked: ai.grounded, onchange: (e) => { ai.grounded = e.target.checked; saveAI(); } }), 'Let the tutor test its advice in the simulator'),
            h('label', { class: 'flex items-center gap-2 text-sm py-1' }, h('input', { type: 'checkbox', checked: ai.allowValues, onchange: (e) => { ai.allowValues = e.target.checked; saveAI(); } }), 'Allow explicit gain values (instructor setting)'),
            h('div', { class: 'text-xs text-slate-500 mt-2 leading-5 border-t pt-2' },
                'Privacy: the request goes from this browser directly to the endpoint above with your own key. The tutor receives your controller design and run metrics; it does not receive your name or ID. The key is never sent anywhere else.'),
            h('div', { class: 'mt-2 flex gap-2' }, btn('New conversation', () => { ai.history = []; ai.transcript = []; render(); }),
                btn('Export conversation', () => download('tutor_conversation.json', JSON.stringify({ model: ai.model, grounded: ai.grounded, design: st.design, transcript: ai.transcript }, null, 1)))));
        const msgs = h('div', { id: 'tutor-log', class: 'space-y-3 overflow-auto h-[52vh] pr-1' },
            ai.transcript.length ? ai.transcript.map(m => m.role === 'user'
                ? h('div', { class: 'flex justify-end' }, h('div', { class: 'max-w-[75%] rounded-xl px-3 py-2 bg-indigo-600 text-white text-sm whitespace-pre-wrap' }, m.text))
                : h('div', { class: 'max-w-[85%]' },
                    m.tools.length ? h('div', { class: 'mb-1 space-y-1' }, m.tools.map(tc => h('details', { class: 'text-xs bg-slate-50 dark:bg-slate-800 border rounded px-2 py-1' },
                        h('summary', { class: 'cursor-pointer' }, '🔧 ', h('b', {}, tc.name), ' ', JSON.stringify(tc.args).slice(0, 120), ' → ', toolSummary(tc)),
                        h('pre', { class: 'mt-1 max-h-40 overflow-auto text-[11px]' }, JSON.stringify(tc.result, null, 1).slice(0, 4000))))) : null,
                    m.pending ? h('div', { class: 'text-sm text-slate-500 italic' }, (m.wait ? `waiting ${m.wait} s for the free-tier rate limit… ` : 'thinking… ') + '(' + m.tools.length + ' tool calls so far)') : null,
                    m.error ? h('div', { class: 'text-sm text-rose-700 bg-rose-50 border border-rose-200 rounded px-3 py-2' }, m.error) : null,
                    m.text ? (() => { const d0 = h('div', { class: 'rounded-xl px-3 py-2 bg-slate-100 dark:bg-slate-800 text-sm leading-6' }); d0.innerHTML = md(m.text); return d0; })() : null))
            : h('div', { class: 'text-sm text-slate-500' }, 'Ask about your latest run. The tutor reads your design and results, can run the simulator to check an idea, and answers with hints rather than finished gains.'));
        const ta = h('textarea', { id: 'tutor-input', class: 'flex-1 h-16 px-2 py-1 border rounded text-sm', placeholder: 'e.g. Why does my travel overshoot fail? What should I change first?',
            onkeydown: (e) => { if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); askTutor(e.target.value); } } });
        const quick = ['Diagnose my latest run against the specifications.', 'Which single change would most likely fix the failing specification? Test it first.', 'Explain why the pitch angle behaves like this during the travel step.'];
        body.append(h('div', { class: 'grid grid-cols-3 gap-3' }, settings,
            h('div', { class: 'col-span-2' }, card('Conversation · ' + (run ? run.label : 'no run yet') + (ai.grounded ? ' · simulator-grounded' : ' · not grounded'), msgs,
                h('div', { class: 'flex flex-wrap gap-1.5 mt-2' }, quick.map(q => btn(q.length > 48 ? q.slice(0, 46) + '…' : q, () => askTutor(q)))),
                h('div', { class: 'flex gap-2 mt-2' }, ta, h('div', { class: 'flex flex-col gap-1' },
                    btn(ai.busy ? '…' : 'Send', () => askTutor(ta.value), 'primary'))),
                h('div', { class: 'text-[11px] text-slate-500 mt-1' }, 'AI answers can be wrong. Check every claim against the plots and the specification table.')))));
        const log = $('#tutor-log'); if (log) log.scrollTop = log.scrollHeight;
    }

    window.Studio = { open, close, state: st, runBatch, controllerFactory, render, askTutor };
})();
