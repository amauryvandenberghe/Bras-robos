"""Genere le projet MIT App Inventor MAVE2.aia (27/11/2026) : l'application MAVE d'origine (memes boutons, meme joystick,
meme curseur de direction, memes commandes) + un panneau REGLAGES pour changer en direct les constantes du suivi de ligne
(Kp, Ki, Kd, lissage, vitesse du servo, vitesses, v255), les relire et les memoriser dans l'ESP32.

Ce qui change aussi (le "ping" = commandes executees en retard) : l'ancienne appli envoyait 2 messages Bluetooth A CHAQUE
mouvement du doigt sur le joystick et sur le curseur (des dizaines par seconde) ; le telephone les met en file d'attente et le
robot les execute avec du retard. Ici, le joystick et le curseur ne font que noter la commande voulue, et une horloge envoie
la DERNIERE toutes les 80 ms, seulement si elle a change. STOP, H, 1, N partent tout de suite. La direction se termine par ;
(O90;) : l'ESP32 l'execute sans attendre 30 ms.

Usage : python generer_mave2.py  ->  MAVE2.aia (a importer dans ai2.appinventor.mit.edu : Projets > Importer un projet).
"""
import json
import os
import random
import zipfile
from xml.sax.saxutils import escape

ICI = os.path.dirname(os.path.abspath(__file__))
PROJET = 'MAVE2'
PAQUET = 'appinventor/ai_amaury_vandenberghe/' + PROJET
random.seed(27112026)


# ------------------------------------------------------------------ interface (Screen1.scm)
def comp(nom, typ, version, enfants=None, **props):
    d = {'$Name': nom, '$Type': typ, '$Version': str(version), 'Uuid': str(random.randint(10 ** 8, 2 * 10 ** 9))}
    d.update({k: str(v) for k, v in props.items()})
    if enfants is not None:
        d['$Components'] = enfants
    return d


BLANC, ROUGE, VERT, GRIS, BLEU = '&HFFFFFFFF', '&HFFE53935', '&HFF43A047', '&HFFEEEEEE', '&HFF1E88E5'
REGLAGES = [  # (nom de la case, libelle, valeur par defaut affichee avant lecture)
    ('TbKp', 'Kp', '2.0'), ('TbKi', 'Ki', '1.0'), ('TbKd', 'Kd', '0.05'), ('TbTau', 'Lissage (s)', '0.06'),
    ('TbVservo', 'Vit. servo (deg/s)', '257'), ('TbVmax', 'V max (m/s)', '0.3'), ('TbVmin', 'V min (m/s)', '0.2'),
    ('TbBrq', 'Braq. lent (deg)', '63.5'), ('TbEcart', 'Ecart lent (mm)', '33.6'), ('TbV255', 'v255 (m/s)', '1.5')]

table = []
for i, (tb, lib, val) in enumerate(REGLAGES):
    table.append(comp('Lb' + tb[2:], 'Label', 5, Column=0, Row=i, Text=lib, FontSize=13))
    table.append(comp(tb, 'TextBox', 6, Column=1, Row=i, Text=val, NumbersOnly='True', Width=90, FontSize=14, Hint=lib))

ecran = comp('Screen1', 'Form', 27, [
    comp('Barre', 'HorizontalArrangement', 3, [
        comp('ListPicker1', 'ListPicker', 9, Text='Connexion', FontSize=16, Width=150),
        comp('LabelEtat', 'Label', 5, Text='Non connecte', FontSize=14, Width=-2),
        comp('BoutonReglages', 'Button', 6, Text='Reglages', FontSize=15, Width=110),
        comp('Bouton1', 'Button', 6, Text='STOP', FontSize=15, FontBold='True', Width=130, BackgroundColor=ROUGE, TextColor=BLANC),
    ], Width=-2, AlignVertical=2, BackgroundColor=BLANC),
    comp('Arrangement_horizontal1', 'HorizontalArrangement', 3, [
        comp('Arrangement_vertical1', 'VerticalArrangement', 3, [
            comp('LabelDirection', 'Label', 5, Text='Direction', FontSize=13),
            comp('Ascenseur1', 'Slider', 2, MinValue=0, MaxValue=180, ThumbPosition=90, Width=250),
            comp('BoutonCentre', 'Button', 6, Text='Tout droit', FontSize=13, Width=120),
            comp('Boutons', 'HorizontalArrangement', 3, [
                comp('Bouton2', 'Button', 6, Text='H', FontSize=16, Width=60),
                comp('Bouton3', 'Button', 6, Text='1', FontSize=16, Width=60),
                comp('Bouton6', 'Button', 6, Text='N', FontSize=16, Width=60),
            ]),
        ], Width=280, AlignHorizontal=3),
        comp('Cadre2', 'Canvas', 12, [comp('Balle2', 'Ball', 6, Radius=25, X=75, Y=75, PaintColor=BLEU)],
             Width=200, Height=200, BackgroundColor=GRIS),
        comp('PanneauReglages', 'VerticalScrollArrangement', 2, [
            comp('TableReglages', 'TableArrangement', 1, table, Columns=2, Rows=len(REGLAGES)),
            comp('BoutonsReglages1', 'HorizontalArrangement', 3, [
                comp('BoutonLire', 'Button', 6, Text='Lire', FontSize=13),
                comp('BoutonEnvoyer', 'Button', 6, Text='Envoyer', FontSize=13, BackgroundColor=VERT, TextColor=BLANC),
            ]),
            comp('BoutonsReglages2', 'HorizontalArrangement', 3, [
                comp('BoutonMemoriser', 'Button', 6, Text='Memoriser', FontSize=13),
                comp('BoutonCode', 'Button', 6, Text='Valeurs du code', FontSize=13),
            ]),
        ], Width=-2, Height=-2, Visible='False'),
    ], Width=-2, Height=-2, BackgroundColor=BLANC),
    comp('LabelMessage', 'Label', 5, Text='', FontSize=12, Width=-2),
    comp('BluetoothClient1', 'BluetoothClient', 5),
    comp('Clock1', 'Clock', 3, TimerInterval=80),
    comp('Notifier1', 'Notifier', 5),
], AppName='MAVE', Title='M.A.V.E.', ScreenOrientation='landscape', Sizing='Responsive', Theme='Classic',
    ShowListsAsJson='True', BackgroundColor=BLANC, VersionCode=2, VersionName='2.0')

scm = '#|\n$JSON\n' + json.dumps({'authURL': ['ai2.appinventor.mit.edu'], 'YaVersion': '208', 'Source': 'Form',
                                   'Properties': ecran}, ensure_ascii=False) + '\n|#'


# ------------------------------------------------------------------ blocs (Screen1.bky)
def B(typ, inner='', nxt=None, mutation=None):
    m = '<mutation %s></mutation>' % mutation if mutation is not None else ''
    n = '<next>%s</next>' % nxt if nxt else ''
    return '<block type="%s">%s%s%s</block>' % (typ, m, inner, n)


def F(nom, val):
    return '<field name="%s">%s</field>' % (nom, escape(str(val)))


def V(nom, bloc):
    return '<value name="%s">%s</value>' % (nom, bloc)


def S(nom, bloc):
    return '<statement name="%s">%s</statement>' % (nom, bloc) if bloc else ''


def suite(*blocs):
    """Enchaine des blocs d'instructions (chacun sans <next>) : b1 -> b2 -> ..."""
    blocs = [b for b in blocs if b]
    out = ''
    for b in reversed(blocs):
        out = b[:-len('</block>')] + ('<next>%s</next>' % out if out else '') + '</block>'
    return out


def txt(t):
    return B('text', F('TEXT', t))


def num(x):
    return B('math_number', F('NUM', x))


def join(*parts):
    return B('text_join', ''.join(V('ADD%d' % i, p) for i, p in enumerate(parts)), mutation='items="%d"' % len(parts))


def get(comp_type, inst, prop):
    return B('component_set_get', F('COMPONENT_SELECTOR', inst) + F('PROP', prop),
             mutation='component_type="%s" set_or_get="get" property_name="%s" is_generic="false" instance_name="%s"' % (comp_type, prop, inst))


def set_(comp_type, inst, prop, val):
    return B('component_set_get', F('COMPONENT_SELECTOR', inst) + F('PROP', prop) + V('VALUE', val),
             mutation='component_type="%s" set_or_get="set" property_name="%s" is_generic="false" instance_name="%s"' % (comp_type, prop, inst))


def call(comp_type, inst, methode, *args):
    return B('component_method', F('COMPONENT_SELECTOR', inst) + ''.join(V('ARG%d' % i, a) for i, a in enumerate(args)),
             mutation='component_type="%s" method_name="%s" is_generic="false" instance_name="%s"' % (comp_type, methode, inst))


def event(comp_type, inst, ev, corps, x, y):
    b = B('component_event', F('COMPONENT_SELECTOR', inst) + S('DO', corps),
          mutation='component_type="%s" is_generic="false" instance_name="%s" event_name="%s"' % (comp_type, inst, ev))
    return b.replace('<block type="component_event">', '<block type="component_event" x="%d" y="%d">' % (x, y), 1)


def gvar(nom):
    return B('lexical_variable_get', F('VAR', 'global ' + nom))


def gset(nom, val):
    return B('lexical_variable_set', F('VAR', 'global ' + nom) + V('VALUE', val))


def lvar(nom):
    return B('lexical_variable_get', F('VAR', nom))


def lset(nom, val):
    return B('lexical_variable_set', F('VAR', nom) + V('VALUE', val))


def si(test, alors, sinon=None):
    m = 'else="1"' if sinon else None
    return B('controls_if', V('IF0', test) + S('DO0', alors) + (S('ELSE', sinon) if sinon else ''), mutation=m)


def si_sinonsi(cas, sinon=None):
    m = 'elseif="%d"%s' % (len(cas) - 1, ' else="1"' if sinon else '')
    inner = ''.join(V('IF%d' % i, t) + S('DO%d' % i, a) for i, (t, a) in enumerate(cas))
    return B('controls_if', inner + (S('ELSE', sinon) if sinon else ''), mutation=m)


def compare(op, a, b):
    return B('math_compare', F('OP', op) + V('A', a) + V('B', b))


def diff(a, b):   # texte different (bloc logique "different de")
    return B('logic_compare', F('OP', 'NEQ') + V('A', a) + V('B', b))


def connecte():
    return get('BluetoothClient', 'BluetoothClient1', 'IsConnected')


def envoyer(t):
    return call('BluetoothClient', 'BluetoothClient1', 'SendText', t)


def gdecl(nom, val, x, y):
    return '<block type="global_declaration" x="%d" y="%d">%s%s</block>' % (x, y, F('NAME', nom), V('VALUE', val))


def proc(nom, args, corps, x, y):
    m = ''.join('<arg name="%s"></arg>' % a for a in args)
    f = ''.join(F('VAR%d' % i, a) for i, a in enumerate(args))
    return '<block type="procedures_defnoreturn" x="%d" y="%d"><mutation>%s</mutation>%s%s%s</block>' % (x, y, m, F('NAME', nom), f, S('STACK', corps))


def proc_ret(nom, args, expr, x, y):
    m = ''.join('<arg name="%s"></arg>' % a for a in args)
    f = ''.join(F('VAR%d' % i, a) for i, a in enumerate(args))
    return '<block type="procedures_defreturn" x="%d" y="%d"><mutation>%s</mutation>%s%s%s</block>' % (x, y, m, F('NAME', nom), f, V('RETURN', expr))


def appel(nom, *args, noms=None):
    noms = noms or ['x%d' % i for i in range(len(args))]
    m = '<mutation name="%s">%s</mutation>' % (nom, ''.join('<arg name="%s"></arg>' % n for n in noms))
    return '<block type="procedures_callnoreturn">%s%s%s</block>' % (m, F('PROCNAME', nom), ''.join(V('ARG%d' % i, a) for i, a in enumerate(args)))


def appel_ret(nom, *args, noms=None):
    noms = noms or ['x%d' % i for i in range(len(args))]
    m = '<mutation name="%s">%s</mutation>' % (nom, ''.join('<arg name="%s"></arg>' % n for n in noms))
    return '<block type="procedures_callreturn">%s%s%s</block>' % (m, F('PROCNAME', nom), ''.join(V('ARG%d' % i, a) for i, a in enumerate(args)))


def local(nom, val, corps):
    return B('local_declaration_statement', F('VAR0', nom) + V('DECL0', val) + S('STACK', corps), mutation=None).replace(
        '<block type="local_declaration_statement">', '<block type="local_declaration_statement"><mutation><localname name="%s"></localname></mutation>' % nom, 1)


def tb_text(tb):
    return get('TextBox', tb, 'Text')


def pas_connecte():
    return call('Notifier', 'Notifier1', 'ShowAlert', txt('Pas connecte au robot : appuyer sur Connexion'))


def bouton_commande(inst, commande, en_plus=None):
    return si(connecte(), suite(envoyer(txt(commande)), en_plus), pas_connecte())


blocs = []
X0 = 20
y = 20


def ajout(b, h=1):
    global y
    blocs.append(b.replace('x="X"', 'x="%d"' % X0).replace('y="Y"', 'y="%d"' % y))
    y += 60 * h


# variables globales
ajout(gdecl('cmd_v', txt(''), X0, y))          # commande de vitesse voulue (V<n>A / V<n>B), envoyee par l'horloge
ajout(gdecl('envoye_v', txt(''), X0, y))
ajout(gdecl('cmd_o', txt(''), X0, y))          # commande de direction voulue (O<n>;)
ajout(gdecl('envoye_o', txt(''), X0, y))
ajout(gdecl('tampon', txt(''), X0, y))         # texte recu du robot, pas encore decoupe en lignes

# Connexion
ajout(event('ListPicker', 'ListPicker1', 'BeforePicking',
            set_('ListPicker', 'ListPicker1', 'Elements', get('BluetoothClient', 'BluetoothClient1', 'AddressesAndNames')), X0, y), 2)
ajout(event('ListPicker', 'ListPicker1', 'AfterPicking',
            si(call('BluetoothClient', 'BluetoothClient1', 'Connect', get('ListPicker', 'ListPicker1', 'Selection')),
               suite(set_('Label', 'LabelEtat', 'Text', txt('Connecte')), envoyer(txt('L')), envoyer(txt('{?}')),
                     gset('envoye_v', txt('')), gset('envoye_o', txt(''))),
               set_('Label', 'LabelEtat', 'Text', txt('Connexion impossible'))), X0, y), 4)

# Boutons de commande (tout de suite, sans file d'attente)
ajout(event('Button', 'Bouton1', 'Click', bouton_commande('Bouton1', 'S', suite(gset('cmd_v', txt('')), gset('envoye_v', txt('')))), X0, y), 3)
ajout(event('Button', 'Bouton2', 'Click', bouton_commande('Bouton2', 'H'), X0, y), 2)
ajout(event('Button', 'Bouton3', 'Click', bouton_commande('Bouton3', '1'), X0, y), 2)
ajout(event('Button', 'Bouton6', 'Click', bouton_commande('Bouton6', 'N'), X0, y), 2)
ajout(event('Button', 'BoutonCentre', 'Click',
            suite(set_('Slider', 'Ascenseur1', 'ThumbPosition', num(90)), gset('cmd_o', txt('O90;'))), X0, y), 2)

# Direction : le curseur note la commande, l'horloge l'envoie (0..180, comme l'ancienne appli)
ajout(event('Slider', 'Ascenseur1', 'PositionChanged',
            gset('cmd_o', join(txt('O'), B('math_round', F('OP', 'ROUND') + V('NUM', lvar('thumbPosition'))), txt(';'))), X0, y), 2)


# Joystick : memes calculs que l'ancienne appli (centre 75, zone morte 65..85, 1,68 PWM par pixel, 126 au plus)
def vitesse_depuis(ecart):
    return B('math_on_list', F('OP', 'MIN') + V('NUM0', num(126)) +
             V('NUM1', B('math_round', F('OP', 'ROUND') + V('NUM', B('math_multiply', V('NUM0', B('math_single', F('OP', 'ABS') + V('NUM', ecart))) + V('NUM1', num(1.68)), mutation='items="2"')))),
             mutation='items="2"')


ajout(event('Ball', 'Balle2', 'Dragged', suite(
    call('Ball', 'Balle2', 'MoveTo', num(75), lvar('currentY')),
    si_sinonsi([(compare('LTE', lvar('currentY'), num(65)), gset('cmd_v', join(txt('V'), vitesse_depuis(B('math_subtract', V('A', lvar('currentY')) + V('B', num(75)))), txt('A')))),
                (compare('GT', lvar('currentY'), num(85)), gset('cmd_v', join(txt('V'), vitesse_depuis(B('math_subtract', V('A', num(75)) + V('B', lvar('currentY')))), txt('B'))))],
               gset('cmd_v', txt('V0A')))), X0, y), 6)
ajout(event('Ball', 'Balle2', 'TouchUp', suite(
    call('Ball', 'Balle2', 'MoveTo', num(75), num(75)),
    gset('cmd_v', txt('')), gset('envoye_v', txt('')),
    si(connecte(), envoyer(txt('S')))), X0, y), 4)

# Horloge : envoie la derniere commande voulue si elle a change, et lit les messages du robot
ajout(event('Clock', 'Clock1', 'Timer', si(connecte(), suite(
    si(B('logic_operation', F('OP', 'AND') + V('A', diff(gvar('cmd_v'), gvar('envoye_v'))) + V('B', B('logic_negate', V('BOOL', B('text_isEmpty', V('VALUE', gvar('cmd_v'))))))),
       suite(envoyer(gvar('cmd_v')), gset('envoye_v', gvar('cmd_v')))),
    si(B('logic_operation', F('OP', 'AND') + V('A', diff(gvar('cmd_o'), gvar('envoye_o'))) + V('B', B('logic_negate', V('BOOL', B('text_isEmpty', V('VALUE', gvar('cmd_o'))))))),
       suite(envoyer(gvar('cmd_o')), gset('envoye_o', gvar('cmd_o')))),
    si(compare('GT', get('BluetoothClient', 'BluetoothClient1', 'BytesAvailableToReceive'), num(0)), suite(
        gset('tampon', join(gvar('tampon'), call('BluetoothClient', 'BluetoothClient1', 'ReceiveText', get('BluetoothClient', 'BluetoothClient1', 'BytesAvailableToReceive')))),
        appel('decouper'))))), X0, y), 10)

# Decoupe le texte recu en lignes
pos = B('text_starts_at', V('TEXT', gvar('tampon')) + V('PIECE', txt('\\n')))
ajout(proc('decouper', [], B('controls_while', V('TEST', compare('GT', pos, num(0))) + S('DO', local('n', pos, suite(
    appel('traiter_ligne', B('text_trim', V('TEXT', B('text_segment', V('TEXT', gvar('tampon')) + V('START', num(1)) + V('LENGTH', B('math_subtract', V('A', lvar('n')) + V('B', num(1))))))), noms=['ligne']),
    gset('tampon', B('text_segment', V('TEXT', gvar('tampon')) + V('START', B('math_add', V('NUM0', lvar('n')) + V('NUM1', num(1)), mutation='items="2"')) +
                     V('LENGTH', B('math_subtract', V('A', B('text_length', V('VALUE', gvar('tampon')))) + V('B', lvar('n')))))))))), X0, y), 6)

# Une ligne du robot : "R kp ki kd tau vservo vmax vmin brqlent ecalent v255" remplit les cases ; sinon elle s'affiche
item = lambda i: B('lists_select_item', V('LIST', lvar('mots')) + V('NUM', num(i)))
remplir = suite(*[set_('TextBox', tb, 'Text', item(i + 2)) for i, (tb, _, _) in enumerate(REGLAGES)])
ajout(proc('traiter_ligne', ['ligne'], si(
    B('logic_operation', F('OP', 'AND') + V('A', compare('EQ', B('text_starts_at', V('TEXT', lvar('ligne')) + V('PIECE', txt('R '))), num(1))) +
      V('B', compare('GTE', B('lists_length', V('LIST', B('text_split_at_spaces', V('TEXT', lvar('ligne'))))), num(len(REGLAGES) + 1)))),
    local('mots', B('text_split_at_spaces', V('TEXT', lvar('ligne'))), suite(remplir, set_('Label', 'LabelMessage', 'Text', txt('Reglages du robot recus')))),
    si(B('logic_negate', V('BOOL', B('text_isEmpty', V('VALUE', lvar('ligne'))))), set_('Label', 'LabelMessage', 'Text', lvar('ligne')))), X0, y), 8)

# Case vide -> "x" : l'ESP32 s'arrete de lire a cet endroit et garde les valeurs suivantes (jamais de decalage)
ajout(proc_ret('valeur', ['t'], B('controls_choose', V('TEST', B('text_isEmpty', V('VALUE', B('text_trim', V('TEXT', lvar('t')))))) +
                                  V('THENRETURN', txt('x')) + V('ELSERETURN', B('text_trim', V('TEXT', lvar('t'))))), X0, y), 2)


def val(tb):
    return appel_ret('valeur', tb_text(tb), noms=['t'])


# Reglages
ajout(event('Button', 'BoutonReglages', 'Click', set_('VerticalScrollArrangement', 'PanneauReglages', 'Visible',
            B('logic_negate', V('BOOL', get('VerticalScrollArrangement', 'PanneauReglages', 'Visible')))), X0, y), 2)
ajout(event('Button', 'BoutonLire', 'Click', bouton_commande('BoutonLire', '{?}'), X0, y), 2)
ajout(event('Button', 'BoutonMemoriser', 'Click', bouton_commande('BoutonMemoriser', '{W}'), X0, y), 2)
ajout(event('Button', 'BoutonCode', 'Click', bouton_commande('BoutonCode', '{Z}'), X0, y), 2)
envoi_g = join(txt('{G '), val('TbKp'), txt(' '), val('TbKi'), txt(' '), val('TbKd'), txt(' '), val('TbTau'), txt(' '), val('TbVservo'), txt('}'))
envoi_t = join(txt('{T '), val('TbVmax'), txt(' '), val('TbVmin'), txt(' '), val('TbBrq'), txt(' '), val('TbEcart'), txt('}'))
envoi_k = join(txt('{K '), val('TbV255'), txt('}'))
ajout(event('Button', 'BoutonEnvoyer', 'Click', si(connecte(), suite(
    envoyer(envoi_g), envoyer(envoi_t), envoyer(envoi_k), call('Form', 'Screen1', 'HideKeyboard')), pas_connecte()), X0, y), 5)

bky = ('<xml xmlns="http://www.w3.org/1999/xhtml">' + ''.join(blocs) +
       '<yacodeblocks ya-version="208" language-version="33"></yacodeblocks></xml>')

proprietes = """main=appinventor.ai_amaury_vandenberghe.%s.Screen1
name=%s
assets=../assets
source=../src
build=../build
versioncode=2
versionname=2.0
useslocation=False
aname=MAVE
sizing=Responsive
showlistsasjson=True
tutorialurl=
subsetjson=
actionbar=False
theme=Classic
color.primary=&HFF3F51B5
color.primary.dark=&HFF303F9F
color.accent=&HFFFF4081
""" % (PROJET, PROJET)

sortie = os.path.join(ICI, PROJET + '.aia')
with zipfile.ZipFile(sortie, 'w', zipfile.ZIP_DEFLATED) as z:
    z.writestr('youngandroidproject/project.properties', proprietes)
    z.writestr('src/%s/Screen1.scm' % PAQUET, scm)
    z.writestr('src/%s/Screen1.bky' % PAQUET, bky)
# controle : le XML des blocs doit etre bien forme
import xml.dom.minidom
xml.dom.minidom.parseString(bky)
json.loads(scm[len('#|\n$JSON\n'):-len('\n|#')])
print(sortie, os.path.getsize(sortie), 'octets ;', len(blocs), 'blocs de tete')
