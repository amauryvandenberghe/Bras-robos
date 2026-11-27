"""27/11/2026 : 4 capteurs EN FLECHE sur le bloc de direction (tournent avec la roue), barycentre + PID (comme bras_robot.ino).
Geometrie realisable (cartes TCRT5000 de 14,1 x 31,2 mm, element a 3,8 mm d'un bout) :
- paire INTERIEURE : cartes le long du robot, entre les 2 roues motrices, elements a x_in, +-b_in (b_in >= 7,6 : cartes cote a cote) ;
- paire EXTERIEURE : cartes en travers vers l'exterieur, devant les roues (x_out <= 9), +-b_out (b_out >= b_in + 11,4) ;
- recul = x_out - x_in (> 0 : pointe de la fleche vers l'avant, les interieurs devant ; < 0 : V ouvert vers l'avant).
Repere SolidWorks : pivot de direction a x 37,3, le robot avance vers les x decroissants : a = 37,3 - x (devant le pivot).
Etape 1 (geometrie) : chaque disposition avec 4 jeux de PID, 2 modeles de capteur, servo rapide + lent, rayons 230 et 200, 0,3 et 0,45 m/s.
Usage : python3 etude_fleche.py geometrie | pid <fichier json des geometries retenues>
"""
import sys, itertools, json, time
sys.path.insert(0, '.')
from multiprocessing import Pool
import numpy as np
import sim_ligne2 as S

PIVOT_X = 37.3
LENT = dict(sv_retard=0.04, sv_tau=0.08, sv_vmax=250.0)
PIDS = ((1.0, 2.0, 0.05, 0.10), (1.6, 2.0, 0.05, 0.06), (2.5, 3.0, 0.05, 0.03), (0.7, 1.5, 0.04, 0.10))


ACTUEL = [(31.8, 22.0), (40.3, 0.0), (31.8, -22.0)]   # 3 capteurs du robot (26/11 nuit)


def capteurs(b_in, b_out=None, x_out=None, recul=None):
    if b_in == 'actuel':
        return ACTUEL
    x_in = x_out - recul
    a_in, a_out = PIVOT_X - x_in, PIVOT_X - x_out
    return [(a_out, b_out), (a_in, b_in), (a_in, -b_in), (a_out, -b_out)]


def evaluer(geo, pid, h, vitesses=(0.3, 0.45), modeles=('etroite', 'large'), servos=({}, LENT), rayons=(230.0, 200.0), graine=10):
    kp, ki, kd, tau = pid
    res = []
    for v in vitesses:
        P = dict(S.P_DEFAUT, loi='pid', avant_seul=True, kp=kp, ki=ki, kd=kd, tau=tau, vitesse_servo=257.0, v_max=255.0, v_min=255.0, v255_estime=v)
        for m in modeles:
            for sv in servos:
                for r in rayons:
                    conf = dict(n=4, pas=0, h=h, methode='bary2', avant_seul=True, modele=m, capteurs=capteurs(*geo), **sv)
                    res.append(S.simuler(conf, P, graine, r, v, tmax=90.0))
    ok = sum(x['ok'] for x in res)
    return dict(ok=ok, n=len(res), rms=float(np.mean([x['rms'] for x in res])), emax=float(max(x['emax'] for x in res)),
                servo=float(np.mean([x['vit_servo'] for x in res])), pertes=float(np.mean([x['pertes'] for x in res])),
                progres=float(np.mean([x['progres'] for x in res])))


def score(r):   # tout reussi d'abord ; puis ecart RMS (mm) + douceur (1 mm pour 50 deg/s RMS de consigne) + pertes
    if r['ok'] < r['n']:
        return 1000 - 500 * r['progres'] - 10 * r['ok']
    return r['rms'] + r['servo'] / 50.0 + 0.5 * r['pertes'] + 0.05 * r['emax']


def un_geo(args):
    geo, h = args
    meilleur = None
    for pid in PIDS:
        r = evaluer(geo, pid, h)
        r['pid'] = pid; r['score'] = score(r)
        if meilleur is None or r['score'] < meilleur['score']:
            meilleur = r
    meilleur['geo'] = geo; meilleur['h'] = h
    return meilleur


if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) > 1 else 'geometrie'
    if mode == 'chrono':
        t = time.time(); print(evaluer((8.5, 22.0, 5.5, 8.0), PIDS[0], 6.0, vitesses=(0.45,), modeles=('etroite',), servos=({},), rayons=(230.0,))); print('%.1f s' % (time.time() - t))
        sys.exit()
    if mode == 'geometrie':
        geos = []
        for b_in, b_out, x_out, recul in itertools.product((7.6, 8.5, 10.0), (19.5, 22.0, 25.0, 28.0), (5.5, 9.0), (-12.0, 0.0, 8.0, 16.0)):
            if b_out >= b_in + 11.4:
                geos.append((b_in, b_out, x_out, recul))
        taches = [(g, h) for g in geos for h in (6.0, 8.0)]
        # reference : les 3 capteurs actuels (22 mm, h 5) avec le PID du robot
        ref = evaluer(('actuel',), (1.3, 2.0, 0.05, 0.10), 5.0)
        print('REFERENCE robot actuel (3 capteurs, 22 mm, h 5, PID 1,3 / 2 / 0,05 / 0,1) : %d/%d  RMS %.1f  max %.1f  servo %.0f  pertes %.1f  score %.1f' % (
            ref['ok'], ref['n'], ref['rms'], ref['emax'], ref['servo'], ref['pertes'], score(ref)), flush=True)
        print('%d dispositions x 2 hauteurs, %d simulations chacune x %d PID' % (len(geos), 16, len(PIDS)), flush=True)
        t0 = time.time(); out = []
        with Pool(5) as pool:
            for i, r in enumerate(pool.imap_unordered(un_geo, taches)):
                out.append(r)
                print('%3d/%d  b_in %4.1f b_out %4.1f x_out %4.1f recul %+5.1f h %.0f | %2d/%d  RMS %5.1f  max %5.1f  servo %4.0f  pertes %4.1f  PID %s  score %.1f  (%.0f min)' % (
                    i + 1, len(taches), *r['geo'], r['h'], r['ok'], r['n'], r['rms'], r['emax'], r['servo'], r['pertes'], r['pid'], r['score'], (time.time() - t0) / 60), flush=True)
        out.sort(key=lambda r: r['score'])
        json.dump(out, open('etude_fleche_geometrie.json', 'w'), indent=1)
        print('\n=== CLASSEMENT (score = RMS + servo/50 + 0,5 pertes + 0,05 max) ===')
        for r in out[:25]:
            print('b_in %4.1f b_out %4.1f x_out %4.1f recul %+5.1f h %.0f | %2d/%d  RMS %5.1f  max %5.1f  servo %4.0f  pertes %4.1f  PID %s  score %.1f' % (
                *r['geo'], r['h'], r['ok'], r['n'], r['rms'], r['emax'], r['servo'], r['pertes'], r['pid'], r['score']))


def un_pid(args):
    geo, h, pid = args
    r = evaluer(geo, pid, h)
    r['pid'] = pid; r['geo'] = geo; r['h'] = h; r['score'] = score(r)
    return r


def etape_pid():
    """Etape 2 : PID fin pour les dispositions retenues (h 8 mm, capteurs releves)."""
    geos = [(7.6, 19.5, 5.5, 8.0), (7.6, 19.5, 5.5, 16.0), (7.6, 22.0, 5.5, 8.0)]
    grille = list(itertools.product((1.0, 1.3, 1.6, 2.0, 2.5), (1.0, 2.0, 3.5), (0.03, 0.05, 0.08), (0.04, 0.06, 0.1)))
    taches = [(g, 8.0, p) for g in geos for p in grille]
    print('%d dispositions x %d PID' % (len(geos), len(grille)), flush=True)
    out = []
    with Pool(5) as pool:
        for r in pool.imap_unordered(un_pid, taches):
            out.append(r)
    for g in geos:
        rs = sorted([r for r in out if r['geo'] == g], key=lambda r: r['score'])
        print('\n=== b_in %.1f b_out %.1f x_out %.1f recul %+.1f h 8 : 10 meilleurs PID ===' % g)
        for r in rs[:10]:
            print('Kp %.1f Ki %.1f Kd %.2f tau %.2f | %2d/%d  RMS %5.1f  max %5.1f  servo %4.0f  pertes %4.1f  score %.2f' % (
                *r['pid'], r['ok'], r['n'], r['rms'], r['emax'], r['servo'], r['pertes'], r['score']), flush=True)
    json.dump(out, open('etude_fleche_pid.json', 'w'), indent=1)


if __name__ == '__main__' and len(sys.argv) > 1 and sys.argv[1] == 'pid':
    etape_pid()
