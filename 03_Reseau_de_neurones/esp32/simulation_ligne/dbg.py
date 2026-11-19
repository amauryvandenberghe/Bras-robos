from sim_ligne import *
for bm in (32, 40, 45):
    for cfg in 'ABC':
        P = dict(PARAM_DEFAUT); P.update(braquage_max=bm)
        nok, tm, res = evaluer(cfg, P)
        print('braq max', bm, cfg, '%d/3' % nok, 't %.1f' % tm, [r['cause'] for r in res], 'progres %.2f' % res[0]['progres'], 'maxerr %.0f' % max(r['maxerr'] for r in res))
