/*
 * math-ui.js — one way to put math on screen, everywhere in the 3-DOF lab.
 * Strings mark math with $...$ (TeX), e.g. 'gain $K_{p,\theta}$ on $\dot\theta$'.
 *   HTML   : KaTeX (equation objects with MathML for screen readers)   MathUI.node / MathUI.html / [data-tex]
 *   SVG    : KaTeX inside a foreignObject                              MathUI.svgText (string) / MathUI.svgNode (element)
 *   canvas : a small TeX-subset typesetter drawn with the KaTeX fonts  MathUI.drawText / MathUI.measure
 *   text   : Unicode fallback for aria-labels, titles, CSV             MathUI.plain
 * Without KaTeX (offline) everything falls back to the Unicode text, so nothing breaks.
 */
(function () {
    'use strict';
    const GREEK = { alpha: 'α', beta: 'β', gamma: 'γ', delta: 'δ', epsilon: 'ϵ', varepsilon: 'ε', zeta: 'ζ', eta: 'η', theta: 'θ', vartheta: 'ϑ',
        kappa: 'κ', lambda: 'λ', mu: 'μ', nu: 'ν', xi: 'ξ', pi: 'π', rho: 'ρ', sigma: 'σ', tau: 'τ', phi: 'ϕ', varphi: 'φ', chi: 'χ', psi: 'ψ', omega: 'ω',
        Gamma: 'Γ', Delta: 'Δ', Theta: 'Θ', Lambda: 'Λ', Sigma: 'Σ', Pi: 'Π', Phi: 'Φ', Psi: 'Ψ', Omega: 'Ω' };
    const SYM = { pm: '±', mp: '∓', le: '≤', leq: '≤', ge: '≥', geq: '≥', ne: '≠', neq: '≠', approx: '≈', infty: '∞', times: '×', cdot: '⋅',
        int: '∫', angle: '∠', to: '→', rightarrow: '→', leftarrow: '←', uparrow: '↑', downarrow: '↓', circ: '∘', partial: '∂', top: '⊤',
        ldots: '…', cdots: '⋯', sum: '∑', prime: '′', vert: '|', lvert: '|', rvert: '|', mid: '|', ell: 'ℓ', degree: '°' };
    const REL = new Set(['=', '<', '>', '≤', '≥', '≠', '≈', '→', '←']), BIN = new Set(['+', '−', '±', '∓', '×', '⋅']);
    const SPACES = { ',': 0.167, ':': 0.222, ';': 0.278, ' ': 0.333, quad: 1, qquad: 2, '!': -0.167 };

    // ---------------------------------------------------------------- parser (TeX subset -> nodes)
    function parse(src) {
        let i = 0;
        const group = () => {
            while (src[i] === ' ') i++;
            if (src[i] === '{') { i++; const r = seq('}'); if (src[i] === '}') i++; return r; }
            return atom();
        };
        const seq = (end) => { const out = []; while (i < src.length && src[i] !== end) out.push(...atom()); return out; };
        function atom() {
            const c = src[i];
            if (c === undefined) return [];
            if (c === '\\') {
                const m = /^\\([A-Za-z]+|.)/.exec(src.slice(i)); i += m[0].length; const n = m[1];
                if (GREEK[n]) return [{ t: 'ch', ch: GREEK[n], f: /^[A-Z]/.test(n) ? 'main' : 'math' }];
                if (SYM[n]) return [{ t: 'ch', ch: SYM[n], f: 'main' }];
                if (n === 'dot' || n === 'ddot' || n === 'hat' || n === 'bar') return [{ t: 'acc', a: n, body: group() }];
                if (n === 'mathrm' || n === 'text' || n === 'operatorname' || n === 'mathbf' || n === 'textrm') {
                    const b = group(); b.forEach(x => { if (x.t === 'ch') { x.f = n === 'text' ? 'text' : 'main'; if (n === 'mathbf') x.bold = true; x.ord = true; } });
                    return b;
                }
                if (n === 'mathbb' || n === 'mathcal') { const BB = { R: 'ℝ', C: 'ℂ', N: 'ℕ', Z: 'ℤ', Q: 'ℚ' }; return group().map(x => x.t === 'ch' ? Object.assign(x, { ch: BB[x.ch] || x.ch, f: 'main', ord: true }) : x); }
                if (n in SPACES) return [{ t: 'sp', em: SPACES[n] }];
                if (n === 'frac' || n === 'tfrac' || n === 'dfrac') { const a = group(), b = group(); return [{ t: 'frac', a, b }]; }
                if (n === 'left' || n === 'right' || n === 'big' || n === 'Big') { const d = atom(); return d.filter(x => !(x.t === 'ch' && x.ch === '.')); }
                if (n === 'sat' || n === 'sin' || n === 'cos' || n === 'tan' || n === 'max' || n === 'min' || n === 'log' || n === 'exp' || n === 'Re' || n === 'Im')
                    return n.split('').map(ch => ({ t: 'ch', ch, f: 'main', ord: true }));
                return [{ t: 'ch', ch: n, f: 'main' }];
            }
            if (c === '_' || c === '^') { i++; return [{ t: c === '_' ? 'sub' : 'sup', body: group() }]; }
            if (c === '{') return group();
            i++;
            if (c === ' ' || c === '~') return c === '~' ? [{ t: 'sp', em: 0.333 }] : [];
            if (/[A-Za-z]/.test(c)) return [{ t: 'ch', ch: c, f: 'math' }];
            if (c === '-') return [{ t: 'ch', ch: '−', f: 'main' }];
            if (c === "'") return [{ t: 'sup', body: [{ t: 'ch', ch: '′', f: 'main' }] }];
            return [{ t: 'ch', ch: c, f: 'main' }];
        }
        return seq(undefined);
    }
    const cacheParse = new Map();
    const parsed = (tex) => { let p = cacheParse.get(tex); if (!p) { p = parse(tex); cacheParse.set(tex, p); } return p; };

    // segments: [{math:false,s:'text'},{math:true,s:'\\theta'}]; '\$' is a literal dollar
    function segments(str) {
        const out = []; const re = /(^|[^\\])\$([^$]+?)\$/g; let last = 0, m;
        str = String(str);
        while ((m = re.exec(str))) {
            const start = m.index + m[1].length;
            if (start > last) out.push({ math: false, s: str.slice(last, start) });
            out.push({ math: true, s: m[2] });
            last = re.lastIndex;
        }
        if (last < str.length) out.push({ math: false, s: str.slice(last) });
        out.forEach(o => { if (!o.math) o.s = o.s.replace(/\\\$/g, '$'); });
        return out;
    }
    const has = (str) => typeof str === 'string' && /(^|[^\\])\$[^$]+?\$/.test(str);

    // ---------------------------------------------------------------- Unicode fallback
    const SUBS = { 0: '₀', 1: '₁', 2: '₂', 3: '₃', 4: '₄', 5: '₅', 6: '₆', 7: '₇', 8: '₈', 9: '₉', a: 'ₐ', e: 'ₑ', h: 'ₕ', i: 'ᵢ', j: 'ⱼ', k: 'ₖ', l: 'ₗ', m: 'ₘ', n: 'ₙ', o: 'ₒ', p: 'ₚ', r: 'ᵣ', s: 'ₛ', t: 'ₜ', u: 'ᵤ', v: 'ᵥ', x: 'ₓ', '+': '₊', '−': '₋', '=': '₌', '(': '₍', ')': '₎', 'β': 'ᵦ', 'γ': 'ᵧ', 'ρ': 'ᵨ', 'φ': 'ᵩ', 'ϕ': 'ᵩ', 'χ': 'ᵪ' };
    const SUPS = { 0: '⁰', 1: '¹', 2: '²', 3: '³', 4: '⁴', 5: '⁵', 6: '⁶', 7: '⁷', 8: '⁸', 9: '⁹', '+': '⁺', '−': '⁻', n: 'ⁿ', i: 'ⁱ', T: 'ᵀ', '⊤': 'ᵀ', '∘': '°', '′': '′' };
    function plainNodes(nodes) {
        let s = '';
        for (const n of nodes) {
            if (n.t === 'ch') s += n.ch;
            else if (n.t === 'sp') s += n.em >= 0.3 ? ' ' : '';
            else if (n.t === 'acc') s += plainNodes(n.body) + ({ dot: '̇', ddot: '̈', hat: '̂', bar: '̄' })[n.a];
            else if (n.t === 'frac') { const a = plainNodes(n.a), b = plainNodes(n.b); s += (a.length > 1 ? '(' + a + ')' : a) + '/' + (b.length > 1 ? '(' + b + ')' : b); }
            else { const b = plainNodes(n.body), map = n.t === 'sub' ? SUBS : SUPS; const all = [...b].every(ch => map[ch]);
                   s += all ? [...b].map(ch => map[ch]).join('') : (n.t === 'sub' ? '_' : '^') + (b.length > 1 ? '(' + b + ')' : b); }
        }
        return s;
    }
    const plain = (str) => segments(str).map(g => g.math ? plainNodes(parsed(g.s)) : g.s).join('');

    // ---------------------------------------------------------------- HTML (KaTeX)
    const esc = (t) => String(t).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]);
    const cacheTex = new Map();
    function texHTML(tex, display) {
        const key = (display ? 'D' : 'I') + tex;
        let r = cacheTex.get(key);
        if (r === undefined) {
            if (window.katex) {
                try { r = window.katex.renderToString(tex, { displayMode: !!display, throwOnError: false, output: 'htmlAndMathml' }); }
                catch (e) { r = null; }
            } else r = null;
            if (r !== null) { if (cacheTex.size > 2000) cacheTex.clear(); cacheTex.set(key, r); }
        }
        return r === null ? '<span class="mu-fb">' + esc(plainNodes(parsed(tex))) + '</span>' : r;
    }
    const html = (str) => segments(str).map(g => g.math ? '<span class="mu">' + texHTML(g.s) + '</span>' : esc(g.s)).join('');
    function node(str) {
        if (!has(str)) return document.createTextNode(String(str));
        const sp = document.createElement('span'); sp.className = 'mu-wrap'; sp.innerHTML = html(str); return sp;
    }
    // static markup: <span data-tex="\psi">ψ</span> (the content is the offline fallback)
    function renderAll(root) {
        if (!window.katex) return;
        (root || document).querySelectorAll('[data-tex]').forEach(el => {
            if (el.dataset.texDone === el.dataset.tex) return;
            el.innerHTML = '<span class="mu">' + texHTML(el.dataset.tex, el.hasAttribute('data-tex-display')) + '</span>';
            el.dataset.texDone = el.dataset.tex;
        });
    }

    // ---------------------------------------------------------------- SVG (KaTeX in a foreignObject)
    // (x, y) is the text baseline as for <text>; anchor start | middle | end
    function svgText(x, y, str, o = {}) {
        const size = o.size || 12, anchor = o.anchor || 'middle', Wd = o.width || 600, H = Math.round(size * 2.6);
        const fx = anchor === 'middle' ? x - Wd / 2 : anchor === 'end' ? x - Wd : x;
        const fy = y - size * 0.34 - H / 2;
        const just = anchor === 'middle' ? 'center' : anchor === 'end' ? 'flex-end' : 'flex-start';
        return '<foreignObject x="' + fx + '" y="' + fy + '" width="' + Wd + '" height="' + H + '" style="overflow:visible;pointer-events:none">' +
            '<div xmlns="http://www.w3.org/1999/xhtml" class="mu-svg" style="height:' + H + 'px;display:flex;align-items:center;justify-content:' + just +
            ';white-space:nowrap;line-height:1;font:' + (o.weight || 400) + ' ' + size + 'px ' + (o.family || 'Inter,system-ui,sans-serif') +
            ';color:' + (o.fill || '#0f172a') + (o.italic ? ';font-style:italic' : '') + '"><span style="white-space:pre">' + html(str) + '</span></div></foreignObject>';
    }
    function svgNode(x, y, str, o = {}) {
        const g = document.createElementNS('http://www.w3.org/2000/svg', 'g');
        g.innerHTML = svgText(x, y, str, o);
        g.setAttribute('aria-hidden', 'true');
        return g;
    }

    // ---------------------------------------------------------------- canvas typesetter (KaTeX fonts)
    const FAM = { math: 'KaTeX_Math, "Times New Roman", serif', main: 'KaTeX_Main, "Times New Roman", serif' };
    const TALL = /[bdfhkltABCDEFGHIJKLMNOPQRSTUVWXYZθδλβζξ0-9]/;
    // returns {w, items:[{ch,font,x,y}], dots:[{x,y,r}], rules:[{x,y,w,h}]}; y relative to the baseline
    function layoutMath(ctx, nodes, sz, wt, textFont) {
        const items = [], dots = [], rules = []; let x = 0;
        const fontOf = (n, s) => n.f === 'math' ? `italic ${n.bold ? 700 : wt >= 600 ? 600 : 400} ${s}px ${FAM.math}`
            : n.f === 'text' ? `${wt} ${s}px ${textFont}` : `${n.bold || wt >= 600 ? 700 : 400} ${s}px ${FAM.main}`;
        function run(list, s, dy) {
            let prevItalic = false;
            for (let k = 0; k < list.length; k++) {
                const n = list[k];
                if (n.t === 'ch') {
                    const big = n.ch === '∫' ? 1.35 : 1;
                    ctx.font = fontOf(n, s * big);
                    const sp = s === sz && !n.ord ? (REL.has(n.ch) ? 0.278 : BIN.has(n.ch) && k > 0 && list[k - 1].t !== 'sp' && !(list[k - 1].t === 'ch' && (REL.has(list[k - 1].ch) || '([{,'.includes(list[k - 1].ch))) ? 0.222 : 0) : 0;
                    x += sp * s;
                    items.push({ ch: n.ch, font: ctx.font, x, y: dy + (big > 1 ? s * 0.12 : 0) });
                    x += ctx.measureText(n.ch).width + (n.ch === ',' && s === sz ? 0.167 * s : 0) + (n.ch === '∫' ? -0.08 * s : 0);
                    x += sp * s;
                    prevItalic = n.f === 'math';
                } else if (n.t === 'sp') x += n.em * s;
                else if (n.t === 'sub' || n.t === 'sup') {
                    if (prevItalic && n.t === 'sup') x += 0.06 * s;
                    run(n.body, s * 0.7, dy + (n.t === 'sub' ? 0.24 * s : -0.42 * s));
                    prevItalic = false;
                } else if (n.t === 'acc') {
                    const x0 = x; run(n.body, s, dy);
                    const tall = n.body.some(b => b.t === 'ch' && TALL.test(b.ch));
                    const cx = (x0 + x) / 2 + 0.07 * s, cy = dy - (tall ? 0.86 : 0.62) * s;
                    if (n.a === 'dot') dots.push({ x: cx, y: cy, r: 0.062 * s });
                    else if (n.a === 'ddot') { dots.push({ x: cx - 0.12 * s, y: cy, r: 0.058 * s }); dots.push({ x: cx + 0.12 * s, y: cy, r: 0.058 * s }); }
                    else rules.push({ x: cx - 0.2 * s, y: cy - 0.03 * s, w: 0.4 * s, h: 0.06 * s });
                    prevItalic = false;
                } else if (n.t === 'frac') {
                    const s2 = s * 0.72, a = layoutMath(ctx, n.a, s2, wt, textFont), b = layoutMath(ctx, n.b, s2, wt, textFont);
                    const w = Math.max(a.w, b.w) + 0.2 * s, x0 = x + 0.08 * s, axis = dy - 0.27 * s;
                    a.items.forEach(it => items.push(Object.assign({}, it, { x: x0 + (w - a.w) / 2 + it.x, y: axis - 0.2 * s + it.y })));
                    b.items.forEach(it => items.push(Object.assign({}, it, { x: x0 + (w - b.w) / 2 + it.x, y: axis + 0.62 * s + it.y })));
                    a.dots.concat(b.dots).forEach(d => dots.push(d));
                    rules.push({ x: x0, y: axis - 0.03 * s, w, h: Math.max(1, 0.05 * s) });
                    x = x0 + w + 0.08 * s; prevItalic = false;
                }
            }
        }
        run(nodes, sz, 0);
        return { w: x, items, dots, rules };
    }
    const cacheLay = new Map();
    function layout(ctx, str, o) {
        const size = o.size || 12, wt = o.weight || 400, family = o.family || 'Inter, system-ui, sans-serif';
        const key = str + '|' + size + '|' + wt + '|' + family;
        let L = cacheLay.get(key);
        if (L) return L;
        ctx.save();
        const items = [], dots = [], rules = []; let x = 0;
        for (const g of segments(str)) {
            if (g.math) {
                const m = layoutMath(ctx, parsed(g.s), size * 1.12, wt, family);
                m.items.forEach(it => items.push(Object.assign({}, it, { x: it.x + x })));
                m.dots.forEach(d => dots.push(Object.assign({}, d, { x: d.x + x })));
                m.rules.forEach(r => rules.push(Object.assign({}, r, { x: r.x + x })));
                x += m.w + 0.04 * size;
            } else {
                ctx.font = `${wt} ${size}px ${family}`;
                items.push({ ch: g.s, font: ctx.font, x, y: 0 });
                x += ctx.measureText(g.s).width;
            }
        }
        ctx.restore();
        L = { w: x, items, dots, rules };
        if (cacheLay.size > 3000) cacheLay.clear();
        cacheLay.set(key, L);
        return L;
    }
    const measure = (ctx, str, o = {}) => layout(ctx, str, o).w;
    // draws str with its baseline at y (o.baseline 'middle' centres it on y); o.align left | center | right
    function drawText(ctx, str, x, y, o = {}) {
        const L = layout(ctx, str, o), size = o.size || 12;
        const x0 = o.align === 'center' ? x - L.w / 2 : o.align === 'right' ? x - L.w : x;
        const y0 = o.baseline === 'middle' ? y + size * 0.36 : y;
        ctx.save();
        ctx.textAlign = 'left'; ctx.textBaseline = 'alphabetic';
        const paint = (stroke) => {
            for (const it of L.items) { ctx.font = it.font; stroke ? ctx.strokeText(it.ch, x0 + it.x, y0 + it.y) : ctx.fillText(it.ch, x0 + it.x, y0 + it.y); }
            for (const d of L.dots) { ctx.beginPath(); ctx.arc(x0 + d.x, y0 + d.y, d.r, 0, 2 * Math.PI); stroke ? ctx.stroke() : ctx.fill(); }
            for (const r of L.rules) stroke ? ctx.strokeRect(x0 + r.x, y0 + r.y, r.w, r.h) : ctx.fillRect(x0 + r.x, y0 + r.y, r.w, r.h);
        };
        if (o.stroke) { ctx.strokeStyle = o.stroke.color; ctx.lineWidth = o.stroke.width; ctx.lineJoin = 'round'; paint(true); }
        if (o.color) ctx.fillStyle = o.color;
        paint(false);
        ctx.restore();
        return L.w;
    }

    // ---------------------------------------------------------------- styles and font readiness
    const css = document.createElement('style');
    css.textContent = '.mu .katex,.mu-svg .katex{font-size:1.1em;text-transform:none;letter-spacing:0;line-height:1.1}' +
        '.mu{text-transform:none;white-space:nowrap}.mu .katex-html{white-space:nowrap}' +
        '[data-tex]{text-transform:none}.mu-fb{font-family:KaTeX_Main,"Times New Roman",serif}';
    (document.head || document.documentElement).appendChild(css);
    let fontsOK = false;
    const ready = new Promise((resolve) => {
        const go = () => {
            renderAll(document);
            const F = ['italic 16px KaTeX_Math', '16px KaTeX_Main', 'bold 16px KaTeX_Main', 'italic 600 16px KaTeX_Math'];
            const p = (document.fonts && document.fonts.load) ? Promise.all(F.map(f => document.fonts.load(f, 'θxV1'))) : Promise.resolve();
            p.catch(() => null).then(() => { fontsOK = true; cacheLay.clear(); window.dispatchEvent(new Event('mathui-fonts')); resolve(); });
        };
        if (document.readyState === 'complete') go(); else window.addEventListener('load', go);
    });

    window.MathUI = { has, plain, html, node, renderAll, svgText, svgNode, drawText, measure, segments, ready, get fontsReady() { return fontsOK; } };
})();
