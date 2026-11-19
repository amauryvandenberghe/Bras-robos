"""Optimisation de la configuration E (6 capteurs sur la roue) pour les 2 vitesses possibles (5 V et 10 V) a la fois."""
import json, math, random
import sim_ligne as S
P0 = dict(json.load(open('resultats_optimisation.json'))['D_0.6']['P'], v_perdu=150.0)
def cout(P, graines=(0, 1)):
    tot = 0.0
    for v in (0.6, 0.9):
        for g in graines:
            r = S.simuler('E', P, seed=g, v255=v)
            tot += r['temps'] if r['ok'] else 60.0 - 30.0 * r['progres']
    return tot
rnd = random.Random(3)
best, bc = P0, cout(P0)
print('depart', round(bc, 1), flush=True)
for it in range(45):
    P = dict(best)
    for k, lo, hi in (('g', 0.3, 3.0), ('tau', 0.015, 0.2), ('braq_lent', 15, 90), ('v_virage', 80, 255), ('v_perdu', 80, 255), ('v_droite', 150, 255)):
        if rnd.random() < 0.5:
            P[k] = min(hi, max(lo, P[k] * math.exp(rnd.gauss(0, 0.3))))
    P['v_virage'] = min(P['v_virage'], P['v_droite'])
    c = cout(P)
    if c < bc:
        best, bc = P, c
        print(it, 'mieux', round(c, 1), {k: round(best[k], 3) for k in ('g', 'tau', 'braq_lent', 'v_droite', 'v_virage', 'v_perdu')}, flush=True)
for v in (0.6, 0.9):
    nok, tm, res = S.evaluer('E', best, graines=tuple(range(20, 30)), v255=v)
    print('VALIDATION %.1f m/s : %d/10, %.1f s, ecart max %.0f mm' % (v, nok, tm, max(r['maxerr'] for r in res)), flush=True)
json.dump(best, open('meilleur_E.json', 'w'), indent=1)
