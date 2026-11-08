"""Suite : implantations possibles, pilotage sur 1 ou 2 rangees, correction integrale."""
import numpy as np
import ecartement_capteurs as c

H = 11.3   # hauteur apres descente de 1,6 mm, a corriger si le sol est different


def simule(s, d, v, af, ar, V, K1, K2, Ki, deux, graine=0, bruit=0.01):
    rng = np.random.default_rng(graine)
    E, dt_ctl, dt = 140.0, 0.01, 0.002
    x = y = th = delta = delta_c = 0.0
    i = j = 0
    ef_filt = ed = integ = dern_f = dern_r = 0.0
    th_f = 0.0
    err, cmd, perdu = [], [], 0
    vmax, dmax = np.radians(250), np.radians(45)
    L = af - ar
    for it in range(int(40 / dt)):
        if it % 5 == 0:
            ef, i = c.ecart(x + af * np.cos(th), y + af * np.sin(th), i)
            if i >= len(c.LIG) - 40:
                break
            er, j = c.ecart(x + ar * np.cos(th), y + ar * np.sin(th), j)

            def lire(e, dern):
                n = np.clip([np.interp(e - p, d, v) + rng.normal(0, bruit) for p in (-s, 0, s)], 0, 1)
                t = n.sum()
                if t > 0.25:
                    return s * (n[2] - n[0]) / t, True
                return np.sign(dern) * 1.5 * s, False
            mf, vu = lire(ef, dern_f)
            if vu:
                dern_f = mf
            else:
                perdu += 1
            integ = np.clip(integ + mf * dt_ctl, -60, 60)
            if deux:
                mr, vr = lire(er, dern_r)
                if vr:
                    dern_r = mr
                th_m = (mf - mr) / L
                th_f += 0.3 * (th_m - th_f)
                kap = -(K1 * mf + K2 * th_f + Ki * integ)
            else:
                ed += 0.25 * ((mf - ef_filt) / dt_ctl - ed)
                ef_filt = mf
                kap = -(K1 * mf + K2 * ed + Ki * integ)
            delta_c = np.clip(np.arctan(kap * E), -dmax, dmax)
            err.append(ef); cmd.append(delta_c)
        delta += np.clip(delta_c - delta, -vmax * dt, vmax * dt)
        x += V * dt * np.cos(th); y += V * dt * np.sin(th); th += V * dt * np.tan(delta) / E
    err, cmd = np.array(err), np.array(cmd)
    return dict(rms=float(np.sqrt((err ** 2).mean())), max=float(np.abs(err).max()), acoups=float(np.degrees(np.sqrt((np.diff(cmd) ** 2).mean()) / dt_ctl)), perdu=perdu, ok=i >= len(c.LIG) - 45)


def meilleur(s, d, v, af, ar, V, deux):
    best = None
    K1s = np.geomspace(2e-4, 4e-3, 9)
    K2s = np.geomspace(3e-3, 0.3, 9) if deux else (0.0, 2e-5, 6e-5, 1.5e-4, 3e-4, 6e-4)
    for K1 in K1s:
        for K2 in K2s:
            for Ki in (0.0, 3e-4, 1e-3):
                r = simule(s, d, v, af, ar, V, K1, K2, Ki, deux)
                if not r['ok'] or r['perdu']:
                    continue
                co = r['rms'] / 3 + r['acoups'] / 60 + r['max'] / 12
                if best is None or co < best[0]:
                    best = (co, K1, K2, Ki, r)
    return best


d, v, f, _ = c.reponse(H, True)
print('tache 10-90 % a h =', H, ':', round(f, 1), 'mm')
print('--- pente au centre / erreur pour 1 % de bruit, selon la hauteur du capteur')
for h in (9.5, 11.3, 12.9, 16.0, 20.0):
    dd, vv, ff, _ = c.reponse(h, False)
    print(f'h {h:4.1f} tache {ff:4.1f} : ' + '  '.join(f's{s}: {c.analyse(s, dd, vv)["pente0"]:.2f}/{c.analyse(s, dd, vv)["bruit"]:.2f}' for s in (12, 13, 14, 14.5, 15, 16)), flush=True)
cas = [('33 mm, avant a +22, 1 rangee (actuel)', 33, 22.0, False), ('33 mm, 2 rangees (actuel)', 33, 22.0, True),
       ('14,5 mm, avant a +22, 1 rangee (imposs.)', 14.5, 22.0, False), ('14,5 mm, avant a +7,3, 1 rangee', 14.5, 7.34, False),
       ('14,5 mm, avant a +7,3, 2 rangees', 14.5, 7.34, True), ('14,5 mm, avant a +22, 2 rangees (imposs.)', 14.5, 22.0, True)]
print('V | cas | K1 K2 Ki | RMS | max | a-coups')
for V in (300.0, 500.0):
    for nom, s, af, deux in cas:
        b = meilleur(s, d, v, af, -59.3, V, deux)
        if b is None:
            print(f'{V:.0f} | {nom} | aucun reglage', flush=True); continue
        _, K1, K2, Ki, r = b
        print(f'{V:.0f} | {nom} | {K1:.1e} {K2:.1e} {Ki:.1e} | {r["rms"]:.2f} | {r["max"]:.1f} | {r["acoups"]:.0f}', flush=True)
