#!/usr/bin/env python3
"""Yorum/docstring-kapsamlı ASCII-Almanca -> gerçek umlaut dönüştürücü.
String literal'lere DOKUNMAZ (cv2 metinleri + grep kontratı korunur).
Kullanım: umlaut.py [--apply]   (varsayılan: dry-run)
"""
import os, re, sys, io, tokenize, ast, glob, collections

APPLY = '--apply' in sys.argv
ROOT = os.path.expanduser('~/ros2_ws')
OUR = ['src/mycobot_calibration','src/mycobot_world','src/mycobot_hardware',
       'src/mycobot_demo','src/mycobot_moveit_config','src/vida_vision']
VOWELS=set('aeiouäöüAEIOU')

# --- ß + özel tam-kelime dönüşümleri (elle onaylı) ---
FORCE = {
 'gross':'groß','grosse':'große','grosses':'großes','Grosser':'Großer','grossteils':'großteils',
 'groesste':'größte','groesstem':'größtem','groessten':'größten','groesster':'größter',
 'Groessen':'Größen','Groesster':'Größter',
 'vergroessern':'vergrößern','Boardgroesse':'Boardgröße','Feldgroesse':'Feldgröße','Marker-Groesse':'Marker-Größe',
 'ausser':'außer','ausserdem':'außerdem','ausserhalb':'außerhalb','aussen':'außen',
 'aeusseren':'äußeren','aeusserer':'äußerer','aeusseres':'äußeres',
 'Aussenflaeche':'Außenfläche','Aussenmass':'Außenmaß',
 'schliessen':'schließen','schliesst':'schließt','Schliessen':'Schließen','Schliess-':'Schließ-',
 'anschliessend':'anschließend','anschliesst':'anschließt','umschliessen':'umschließen',
 'ausschliesslich':'ausschließlich','Finger-Schliessachse':'Finger-Schließachse',
 'stoesst':'stößt','stossen':'stoßen','Anstossen':'Anstoßen','anstossen':'anstoßen',
 'angestossen':'angestoßen','Anstoss-Pruefung':'Anstoß-Prüfung',
 'weiss':'weiß','weissen':'weißen','weisser':'weißer',
 'Mass':'Maß','Masse':'Maße','Massbestimmung':'Maßbestimmung','Board-Masse':'Board-Maße',
 'Markermasse':'Markermaße','Massstab':'Maßstab','Kamera-Massstab':'Kamera-Maßstab',
 'massstaeblich':'maßstäblich','massgeblich':'maßgeblich',
 'Ausreisser':'Ausreißer','Ausreisser-Eliminierung':'Ausreißer-Eliminierung','Ausreisser-Filter':'Ausreißer-Filter',
 'MAD-Ausreisser-Eliminierung':'MAD-Ausreißer-Eliminierung','MAD-Ausreisser-':'MAD-Ausreißer-',
 'Tiefen-Ausreisser':'Tiefen-Ausreißer','ausreisserrobust':'ausreißerrobust','ausreisser-robustes':'ausreißer-robustes',
 'Gauss-Newton':'Gauß-Newton',
 'gleichmaessig':'gleichmäßig','gleichmaessiger':'gleichmäßiger',
 'erfahrungsgemaess':'erfahrungsgemäß','Standardmaessig':'Standardmäßig',
 'Muell':'Müll',
 'zuruueckprojizieren':'zurückprojizieren',  # önceden var olan yazım hatası (uue) düzeltildi
}
# İngilizce/özel-ad/zuerst — hiç değişmez
KEEP = {'true','True','value','values','Daemon-Thread','Rodrigues','Rodrigues-Rotationsvektor',
 'Rodrigues-Umformung','Shoemake','Verzeichnungskoeffizienten','zuerst','Zuerst'}
# -uell Latince sıfat/zarf — ue korunur (füllen DEĞİL)
UELL_SKIP = {'aktuell','aktuelle','aktuellem','aktuellen','aktueller','aktuelles','aktuellste','aktuellstes',
 'Aktuelle','Aktuellen','manuell','manuellen','visuell','visuelle','visuellen'}

def conv_ue(w):
    out=[]; i=0
    while i<len(w):
        two=w[i:i+2]
        if two in ('ue','Ue'):
            prev=w[i-1] if i>0 else ''
            if prev in VOWELS or prev in ('q','Q'):
                out.append(w[i]); i+=1; continue
            out.append('ü' if two=='ue' else 'Ü'); i+=2; continue
        out.append(w[i]); i+=1
    return ''.join(out)

def rule(w):
    return conv_ue(w.replace('ae','ä').replace('Ae','Ä').replace('oe','ö').replace('Oe','Ö'))

CHANGES = collections.Counter()
def conv_word(w):
    if w in FORCE: new=FORCE[w]
    elif w in KEEP or w in UELL_SKIP or w.isupper(): new=w   # UPPERCASE=UI-literal referansı, atla
    else: new=rule(w)
    if new!=w: CHANGES[(w,new)]+=1
    return new

WORD=re.compile(r'[A-Za-zÀ-ÿ][A-Za-zÀ-ÿ\-]*')
def conv_text(t):
    return WORD.sub(lambda m: conv_word(m.group(0)), t)

# ---------- .py: tokenize span-replacement (yorum + docstring) ----------
def process_py(src):
    spans=[]  # (start_off, end_off, newtext)
    lines=src.splitlines(keepends=True)
    def off(row,col):
        return sum(len(lines[r]) for r in range(row-1))+col
    try:
        toks=list(tokenize.generate_tokens(io.StringIO(src).readline))
    except Exception as e:
        return src, False
    # docstring string token'larını bul (Module/Class/Func ilk ifade)
    doc_positions=set()
    try:
        tree=ast.parse(src)
        for node in ast.walk(tree):
            if isinstance(node,(ast.Module,ast.ClassDef,ast.FunctionDef,ast.AsyncFunctionDef)):
                body=getattr(node,'body',[])
                if body and isinstance(body[0],ast.Expr) and isinstance(getattr(body[0],'value',None),ast.Constant) and isinstance(body[0].value.value,str):
                    doc_positions.add((body[0].value.lineno, body[0].value.col_offset))
    except Exception:
        pass
    for tok in toks:
        if tok.type==tokenize.COMMENT:
            spans.append((off(*tok.start), off(*tok.end), conv_text(tok.string)))
        elif tok.type==tokenize.STRING and (tok.start[0],tok.start[1]) in doc_positions:
            spans.append((off(*tok.start), off(*tok.end), conv_text(tok.string)))
    if not spans: return src, False
    spans.sort()
    out=[]; last=0; changed=False
    for s,e,new in spans:
        out.append(src[last:s]); 
        if new!=src[s:e]: changed=True
        out.append(new); last=e
    out.append(src[last:])
    return ''.join(out), changed

# ---------- .sh: tam-satır # yorumları ----------
def process_sh(src):
    res=[]; changed=False
    for line in src.splitlines(keepends=True):
        if line.lstrip().startswith('#'):
            nl=conv_text(line)
            if nl!=line: changed=True
            res.append(nl)
        else:
            res.append(line)
    return ''.join(res), changed

# ---------- .yaml: satır-başı ya da boşluk-sonrası # ----------
def process_yaml(src):
    res=[]; changed=False
    for line in src.splitlines(keepends=True):
        m=re.search(r'(^|\s)#', line)
        if m:
            i=m.start()+len(m.group(1))
            head, comment = line[:i], line[i:]
            nc=conv_text(comment)
            nl=head+nc
            if nl!=line: changed=True
            res.append(nl)
        else:
            res.append(line)
    return ''.join(res), changed

# ---------- .cpp: // ve /* */ yorumları ----------
def process_cpp(src):
    out=[]; i=0; n=len(src); changed=False
    while i<n:
        if src[i:i+2]=='//':
            j=src.find('\n', i);  j=n if j<0 else j
            seg=src[i:j]; nseg=conv_text(seg)
            if nseg!=seg: changed=True
            out.append(nseg); i=j
        elif src[i:i+2]=='/*':
            j=src.find('*/', i); j=n if j<0 else j+2
            seg=src[i:j]; nseg=conv_text(seg)
            if nseg!=seg: changed=True
            out.append(nseg); i=j
        elif src[i]=='"':  # string literal — atla
            j=i+1
            while j<n and src[j]!='"':
                if src[j]=='\\': j+=1
                j+=1
            out.append(src[i:j+1]); i=j+1
        elif src[i]=="'":
            j=i+1
            while j<n and src[j]!="'":
                if src[j]=='\\': j+=1
                j+=1
            out.append(src[i:j+1]); i=j+1
        else:
            out.append(src[i]); i+=1
    return ''.join(out), changed

# ---------- .xml/.xacro/.srdf: <!-- --> ----------
def process_xml(src):
    changed=[False]
    def repl(m):
        seg=m.group(0); nseg=conv_text(seg)
        if nseg!=seg: changed[0]=True
        return nseg
    out=re.sub(r'<!--.*?-->', repl, src, flags=re.S)
    return out, changed[0]

def handler(path):
    if path.endswith('.py'): return process_py
    if path.endswith('.sh'): return process_sh
    if path.endswith(('.yaml','.yml')): return process_yaml
    if path.endswith(('.cpp','.hpp','.h','.cc')): return process_cpp
    if path.endswith(('.xacro','.srdf','.urdf','.xml')): return process_xml
    return None

files=[]
for base in OUR:
    for r,dirs,fs in os.walk(os.path.join(ROOT,base)):
        dirs[:] = [d for d in dirs if d not in ('__pycache__','.git')]
        for f in fs:
            if handler(f): files.append(os.path.join(r,f))
for pat in ('*.sh','*.py','*.yaml'):
    files += glob.glob(os.path.join(ROOT,pat))

changed_files=[]
for p in sorted(set(files)):
    h=handler(p); src=open(p,encoding='utf-8').read()
    new,ch=h(src)
    if ch:
        changed_files.append(p)
        if APPLY:
            open(p,'w',encoding='utf-8').write(new)

print(f"{'UYGULANDI' if APPLY else 'DRY-RUN'}: {len(changed_files)} dosya değiş(ecek/ti), {len(CHANGES)} benzersiz kelime dönüşümü, toplam {sum(CHANGES.values())} yerde")
print("=== benzersiz dönüşümler (alfabetik) ===")
for (w,new),c in sorted(CHANGES.items()):
    print(f"{c:4d}  {w}  ->  {new}")
