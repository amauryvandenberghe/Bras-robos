"""Remplit le recapitulatif PDF avec les resultats du test Gazebo officiel (fin_automatique.sh), puis l'imprime en PDF
avec Chrome sans fenetre. Le modele HTML garde les reperes [[RES_...]] ; un seul PDF, a la racine
du dossier du Bureau.
Usage : python3 finaliser_document.py
"""
import json
import os
import subprocess

ICI = os.path.dirname(os.path.abspath(__file__))
M = os.environ.get('MODELES_EVAL', os.path.join(ICI, 'modeles'))      # autre dossier : pour essayer le script
DEST = '/mnt/c/Users/amaur/Desktop/Bras robotisé'
DOC = os.path.join(DEST, '05_Documentation')
MODELE_HTML = os.path.join(DOC, 'source', 'Recapitulatif_bras_robotise.modele.html')
SORTIE_HTML = os.path.join(DOC, 'source', 'Recapitulatif_bras_robotise.html')
CHROME = '/mnt/c/Program Files/Google/Chrome/Application/chrome.exe'


def bilan(nom, n_balles=3):
    s = json.load(open(os.path.join(M, nom)))['sequences']
    balles = sum(r == 'reussite' for x in s for r in x['resultats'])
    echecs = {}
    for x in s:
        for r in x['resultats']:
            if r != 'reussite':
                echecs[r] = echecs.get(r, 0) + 1
    non_jouees = n_balles * len(s) - sum(len(x['resultats']) for x in s)
    if non_jouees:
        echecs['non jouee'] = non_jouees
    return sum(x['reussite'] for x in s), len(s), balles, n_balles * len(s), echecs


def windows(chemin):
    return 'C:\\' + chemin.split('/mnt/c/', 1)[1].replace('/', '\\')


def main():
    sim, aff = bilan('eval_gazebo_simulateur.json'), bilan('eval_gazebo_affine.json')
    # depuis le 30/10, le reseau officiel est celui du parcours (complement "mission" + affinage Gazebo) : il sait aussi
    # prendre les balles devant le robot. Ses tests sont dans modeles/mission/ (evaluer_mission_gazebo.py).
    mis = bilan('mission/eval_gazebo_officielle.json')
    mis_dev = bilan('mission/eval_gazebo_devant.json', 2)
    off = bilan('mission/eval_gazebo_officielle_politique_mission_affine.json')
    off_dev = bilan('mission/eval_gazebo_devant_politique_mission_affine.json', 2)
    noms_echecs = {'tombee': 'tombée(s)', 'temps': 'temps dépassé', 'collision': 'collision(s)', 'non jouee': 'non jouée(s)',
                   'poussee': 'poussée(s)'}

    def echecs(e):
        return ', '.join(f'{n} {noms_echecs.get(k, k)}' for k, n in sorted(e.items())) or 'aucun'

    court = (f'<b>{off[2]}/{off[3]} balles</b> ({off[0]}/{off[1]} séquences complètes) et, balles devant le robot, '
             f'<b>{off_dev[2]}/{off_dev[3]}</b>')
    ligne = (f'Réseau officiel (parcours, affiné) : {off[0]}/{off[1]} séquences, {off[2]}/{off[3]} balles ; '
             f'balles devant : {off_dev[0]}/{off_dev[1]}, {off_dev[2]}/{off_dev[3]}.')
    table = f'''<table>
<tr><th>Pilote</th><th class="n">Séquences de 3 balles réussies</th><th class="n">Balles déposées</th><th>Échecs (balles)</th></tr>
<tr><td>Expert (référence)</td><td class="n">4 / 4</td><td class="n">12 / 12</td><td>aucun</td></tr>
<tr><td>Ancien réseau (avant les règles définitives)</td><td class="n">6 / 10</td><td class="n">20 / 30</td><td>balles lâchées juste après la prise</td></tr>
<tr><td>Nouveau réseau, phase de corrections (20 dispositions)</td><td class="n">18 / 20</td><td class="n">56 / 60</td><td>1 temps dépassé, 1 tombée, 2 non jouées</td></tr>
<tr><td><b>Test officiel</b> – réseau du simulateur (20 autres dispositions)</td><td class="n">{sim[0]} / {sim[1]}</td><td class="n">{sim[2]} / {sim[3]}</td><td>{echecs(sim[4])}</td></tr>
<tr><td><b>Test officiel</b> – réseau affiné avec Gazebo (mêmes dispositions) : réseau final du 29/10, <b>ne sait pas prendre devant le robot</b></td><td class="n">{aff[0]} / {aff[1]}</td><td class="n">{aff[2]} / {aff[3]}</td><td>{echecs(aff[4])}</td></tr>
<tr><td>Complément « parcours » (balles dans toutes les directions), mêmes dispositions</td><td class="n">{mis[0]} / {mis[1]}</td><td class="n">{mis[2]} / {mis[3]}</td><td>{echecs(mis[4])}</td></tr>
<tr><td><b>Réseau officiel</b> (30/10) : complément « parcours » affiné avec 19 000 corrections Gazebo, mêmes dispositions</td><td class="n"><b>{off[0]} / {off[1]}</b></td><td class="n"><b>{off[2]} / {off[3]}</b></td><td>{echecs(off[4])}</td></tr>
<tr><th colspan="4">Réglette devant le robot (−35° à +35°), 2 balles par séquence vers les petits réceptacles, 20 dispositions</th></tr>
<tr><td>Complément « parcours »</td><td class="n">{mis_dev[0]} / {mis_dev[1]}</td><td class="n">{mis_dev[2]} / {mis_dev[3]}</td><td>{echecs(mis_dev[4])}</td></tr>
<tr><td><b>Réseau officiel</b></td><td class="n"><b>{off_dev[0]} / {off_dev[1]}</b></td><td class="n"><b>{off_dev[2]} / {off_dev[3]}</b></td><td>{echecs(off_dev[4])}</td></tr>
</table>
<p>Réseau officiel : <code>modeles/politique_finale.pt</code> (copie de <code>modeles/mission/politique_mission_affine.pt</code>), exporté pour l'ESP32 ; dans le dossier du Bureau : <code>03_Reseau_de_neurones\\modele\\1_RESEAU_FINAL (a utiliser)\\reseau_final.pt</code>.
L'ancien réseau final (20/20 sur la scène d'origine, mais 0 prise sur 15 droit devant) est gardé en secours : <code>modele\\2_Secours (ancien reseau du 29-10)\\reseau_secours_29oct.pt</code>
(<code>modeles/politique_finale_29oct.pt</code> dans Ubuntu). Le fichier <code>LISEZMOI.txt</code> du dossier <code>modele</code> résume lequel utiliser.
Une séquence ratée s'arrête à la première balle manquée : les balles suivantes ne sont pas jouées et comptent comme non déposées.
« Temps dépassé » : le réseau hésite au-dessus de la balle sans la prendre, pendant les 70 s autorisées. Le parcours demande de prendre devant le robot (station 2) :
c'est pourquoi ce réseau, un peu moins bon sur la scène d'origine, est retenu. Le réseau définitif sera réentraîné sur la vraie disposition du plateau (chapitre 11).</p>'''
    html = open(MODELE_HTML, encoding='utf-8').read()
    html = html.replace('[[RES_GAZEBO_COURT]]', court).replace('[[RES_GAZEBO_LIGNE]]', ligne).replace('[[RES_GAZEBO_TABLE]]', table)
    assert '[[' not in html, 'repere non remplace'
    open(SORTIE_HTML, 'w', encoding='utf-8').write(html)
    pdf = os.path.join(DEST, 'Recapitulatif_bras_robotise.pdf')      # un seul PDF, a la racine du dossier
    subprocess.run([CHROME, '--headless=new', '--disable-gpu', '--no-first-run', '--no-pdf-header-footer',
                    '--user-data-dir=C:\\Users\\amaur\\AppData\\Local\\Temp\\bras50\\chrome_profil',
                    f'--print-to-pdf={windows(pdf)}', 'file:///' + windows(SORTIE_HTML).replace('\\', '/')],
                   check=True, capture_output=True)
    print(ligne, '->', pdf)


if __name__ == '__main__':
    main()
