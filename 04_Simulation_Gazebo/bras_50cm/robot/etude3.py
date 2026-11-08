"""Implantation retenue : lateraux en x 205,05 (+27,8 devant l'essieu), central en x 190,55 (+13,3), rangee arriere -58,5."""
import numpy as np
import ecartement_capteurs as c

H = 11.4


def simule(s, d, v, ax, am, ar, V, K1, K2, Ki, deux, graine=0, bruit=0.01):
    rng = np.random.default_rng(graine)
    E, dt_ctl, dt = 140.0, 0.01, 0.002
    x = y = th = delta = delta_c = 0.0
    i = im = j = 0
    ed = ef_f = integ = df = dr = th_f = 0.0
    err, cmd, perdu = [], [], 0
    vmax, dmax = np.radians(250), np.radians(45)
    L = ax - ar
    for it in range(int(40 / dt)):
        if it % 5 == 0:
            cc, ss = np.cos(th), np.sin(th)
            eo, i = c.ecart(x + ax * cc, y + ax * ss, i)
            if i >= len(c.LIG) - 40:
                break
            em, im = c.ecart(x + am * cc, y + am * ss, im)
            er, j = c.ecart(x + ar * cc, y + ar * ss, j)

            def lire(ea, eb, der):
                n = np.clip([np.interp(ea + s, d, v) + rng.normal(0, bruit), np.interp(eb, d, v) + rng.normal(0, bruit), np.interp(ea - s, d, v) + rng.normal(0, bruit)], 0, 1)
                t = n.sum()
                return (s * (n[2] - n[0]) / t, True) if t > 0.25 else (np.sign(der) * 1.5 * s, False)
            mf, vu = lire(eo, em, df)
            if vu:
                df = mf
            else:
                perdu += 1
            integ = np.clip(integ + mf * dt_ctl, -60, 60)
            if deux:
                mr, vr = lire(er, er, dr)
                if vr:
                    dr = mr
                th_f += 0.3 * ((mf - mr) / L - th_f)
                kap = -(K1 * mf + K2 * th_f + Ki * integ)
            else:
                ed += 0.25 * ((mf - ef_f) / dt_ctl - ed); ef_f = mf
                kap = -(K1 * mf + K2 * ed + Ki * integ)
            delta_c = np.clip(np.arctan(kap * E), -dmax, dmax)
            err.append(eo); cmd.append(delta_c)
        delta += np.clip(delta_c - delta, -vmax * dt, vmax * dt)
        x += V * dt * np.cos(th); y += V * dt * np.sin(th); th += V * dt * np.tan(delta) / E
    err, cmd = np.array(err), np.array(cmd)
    return dict(rms=float(np.sqrt((err ** 2).mean())), max=float(np.abs(err).max()), acoups=float(np.degrees(np.sqrt((np.diff(cmd) ** 2).mean()) / dt_ctl)), perdu=perdu, ok=i >= len(c.LIG) - 45)


def meilleur(s, d, v, ax, am, ar, V, deux):
    best = None
    for K1 in np.geomspace(2e-4, 4e-3, 9):
        for K2 in (np.geomspace(3e-3, 0.3, 9) if deux else (0.0, 2e-5, 6e-5, 1.5e-4, 3e-4, 6e-4)):
            for Ki in (0.0, 3e-4, 1e-3):
                r = simule(s, d, v, ax, am, ar, V, K1, K2, Ki, deux)
                if r['ok'] and not r['perdu']:
                    co = r['rms'] / 3 + r['acoups'] / 60 + r['max'] / 12
                    if best is None or co < best[0]:
                        best = (co, K1, K2, Ki, r)
    return best


d, v, f, _ = c.reponse(H, True)
print('V | s | rangees | K1 K2 Ki | RMS | max | a-coups')
for V in (200.0, 300.0, 400.0):
    for s, ax, am, nom in ((13, 27.8, 13.3, 'retenu'), (33, 22.0, 22.0, 'actuel 33')):
        for deux in (False, True):
            b = meilleur(s, d, v, ax, am, -58.5, V, deux)
            t = f'{V:.0f} | {nom} s={s} | {"2" if deux else "1"} rang. | '
            if b is None:
                print(t + 'aucun reglage', flush=True); continue
            _, K1, K2, Ki, r = b
            print(t + f'{K1:.1e} {K2:.1e} {Ki:.1e} | {r["rms"]:.2f} | {r["max"]:.1f} | {r["acoups"]:.0f}', flush=True)
