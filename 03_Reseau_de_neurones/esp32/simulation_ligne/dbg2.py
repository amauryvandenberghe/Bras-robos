import sim_ligne as S
from sim_ligne import *
# trace interne : on rejoue en instrumentant le regulateur
orig = Regulateur.pas
log = []
def pas(self, t, dt, a, b):
    r = orig(self, t, dt, a, b)
    log.append((t, list(a), list(b), r[0], r[1], self.facteur))
    return r
Regulateur.pas = pas
P = dict(PARAM_DEFAUT); P.update(v_droite=100, v_virage=100, v_demarrage=100, v_perdu=100, braquage_max=40, kp=45)
r = simuler('A', P, seed=0, trace=True)
print(r['cause'], r['progres'], r['temps'])
for i in range(0, len(log), 40):
    t, a, b, br, v, f = log[i]
    print('%.2f' % t, a, b, 'braq %.1f v %.0f f %.2f' % (br, v, f))
