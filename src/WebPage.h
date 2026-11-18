/*

   WebPage.h

   Page of the settings (WebUi.h), kept in the program. One section per
   subject; each one reads and writes its part of the JSON API.
   Colours and layout as the page of the locomotives of DCC_RMT_cutout.

*/

#pragma once

#include <Arduino.h> // PROGMEM

static const char WEB_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>MFX Track Station · Réglages</title>
<style>
:root{
  --bg:#eef3f7;--panel:#fff;--ink:#17212d;--muted:#617184;--line:#d5e0e8;
  --brand:#116f89;--brand-dark:#153f50;--good:#197742;--good-bg:#dff3e5;
  --bad:#a92d25;--bad-bg:#fff0ee;--shadow:0 12px 30px rgba(26,56,72,.08)
}
*{box-sizing:border-box}
body{margin:0;background:linear-gradient(180deg,#e8eff4,#f7f9fb 54%,#edf3f7);color:var(--ink);
  font-family:Inter,ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif}
button,input{font:inherit}
.shell{max-width:980px;margin:auto;padding:14px 16px 40px}
.header{background:linear-gradient(120deg,var(--brand-dark),#176c82);color:#fff;padding:24px 28px;
  border-radius:22px;box-shadow:var(--shadow)}
.header h1{font-size:clamp(1.6rem,4vw,2.4rem);margin:0 0 6px}
.header p{margin:0;color:#dcebf0}
.card{background:var(--panel);border:1px solid var(--line);border-radius:18px;padding:20px 22px;
  margin-top:18px;box-shadow:var(--shadow)}
.card h2{margin:0 0 4px;font-size:1.25rem}
.hint{color:var(--muted);margin:0 0 16px;font-size:.95rem}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:14px}
label{display:block;font-weight:650;margin-bottom:6px}
label small{display:block;font-weight:400;color:var(--muted)}
input[type=number]{width:100%;padding:10px 12px;border:1px solid var(--line);border-radius:10px}
.actions{display:flex;gap:12px;align-items:center;margin-top:16px;flex-wrap:wrap}
button{background:var(--brand);color:#fff;border:0;border-radius:999px;padding:10px 22px;
  font-weight:700;cursor:pointer}
button:disabled{opacity:.5;cursor:default}
.msg{padding:8px 14px;border-radius:10px;display:none}
.msg.ok{display:block;background:var(--good-bg);color:var(--good)}
.msg.err{display:block;background:var(--bad-bg);color:var(--bad)}
.state{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:12px;margin-top:16px}
.tile{border:1px solid var(--line);border-radius:12px;padding:12px 14px}
.tile b{display:block;font-size:1.2rem;margin-top:4px}
.tile span{color:var(--muted);font-size:.9rem}
table{width:100%;border-collapse:collapse;margin-top:8px}
td,th{text-align:left;padding:8px 6px;border-bottom:1px solid var(--line)}
a{color:var(--brand)}
</style>
</head>
<body>
<div class="shell">
  <div class="header">
    <h1>MFX Track Station</h1>
    <p>Réglages de la centrale</p>
  </div>

  <div class="card" id="l88">
    <h2>Link S88 (L88 60883)</h2>
    <p class="hint">Nombre de modules S88 branchés sur chaque bus du L88 et temps de lecture.
      Un module branché mais non déclaré ne remonte rien.</p>
    <div class="grid">
      <div><label>Bus 1<small>RJ45 (S88-N), 0 à 31</small></label><input id="b1" type="number" min="0" max="31"></div>
      <div><label>Bus 2<small>RJ45 (S88-N), 0 à 31</small></label><input id="b2" type="number" min="0" max="31"></div>
      <div><label>Bus 3<small>6 broches (6088), 0 à 31</small></label><input id="b3" type="number" min="0" max="31"></div>
      <div><label>Temps de cycle<small>en ms, 10 à 1000</small></label><input id="cy" type="number" min="10" max="1000"></div>
    </div>
    <div class="actions">
      <button id="save">Enregistrer</button>
      <div class="msg" id="msg"></div>
    </div>
    <div class="state">
      <div class="tile"><span>L88 trouvé(s)</span><b id="found">–</b></div>
      <div class="tile"><span>Événements S88</span><b id="events">–</b></div>
      <div class="tile"><span>Dernier contact</span><b id="last">–</b></div>
    </div>
  </div>

  <div class="card">
    <h2>Fichiers de configuration</h2>
    <p class="hint" id="fsinfo">Mémoire LittleFS de l'ESP32.</p>
    <table><thead><tr><th>Fichier</th><th>Taille</th><th></th></tr></thead><tbody id="files"></tbody></table>
  </div>
</div>
<script>
const $=id=>document.getElementById(id);
let edited=false;
['b1','b2','b3','cy'].forEach(id=>$(id).addEventListener('input',()=>edited=true));
function show(s){
  if(!edited){$('b1').value=s.bus[0];$('b2').value=s.bus[1];$('b3').value=s.bus[2];$('cy').value=s.cycleMs;}
  $('found').textContent=s.found.length?s.found.join(', '):'aucun';
  $('events').textContent=s.events;
  $('last').textContent=s.events?('n° '+s.lastContact+(s.lastState?' occupé':' libre')):'–';
}
function msg(t,ok){const m=$('msg');m.textContent=t;m.className='msg '+(ok?'ok':'err');}
async function refresh(){
  try{const r=await fetch('/api/l88');show(await r.json());}catch(e){}
}
async function files(){
  try{
    const r=await fetch('/api/files');const f=await r.json();
    $('fsinfo').textContent=f.available?('Mémoire LittleFS : '+f.used+' octets utilisés sur '+f.total+'.'):'LittleFS indisponible : les réglages ne sont pas enregistrés.';
    $('files').innerHTML=Object.entries(f.files).map(([n,s])=>
      '<tr><td>/config/'+n+'.json</td><td>'+s+' o</td><td><a href="/api/config?name='+encodeURIComponent(n)+'">télécharger</a></td></tr>').join('')
      ||'<tr><td colspan="3">aucun fichier pour l\'instant</td></tr>';
  }catch(e){}
}
$('save').onclick=async()=>{
  const body={bus:[+$('b1').value,+$('b2').value,+$('b3').value],cycleMs:+$('cy').value};
  $('save').disabled=true;
  try{
    const r=await fetch('/api/l88',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    const s=await r.json();
    if(!r.ok){msg(s.error||'erreur',false);}
    else{edited=false;show(s);msg(s.saved?'Enregistré et envoyé au L88':'Appliqué, mais non enregistré (LittleFS)',s.saved);files();}
  }catch(e){msg('pas de réponse de la centrale',false);}
  $('save').disabled=false;
};
refresh();files();setInterval(refresh,2000);
</script>
</body>
</html>
)HTML";
