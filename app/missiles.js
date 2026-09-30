(() => {
  'use strict';
  const el = id => document.getElementById(id);
  const root = el('missile-workspace');
  const initial = {missile:'us_aim9l_sidewinder',duration:60,
    launch:{position:[0,5000,0],velocity:[300,0,0],angles:[0,0,0]},
    target:{position:[4000,5000,0],velocity:[200,0,0],angles:[0,0,0]}};
  let meta=null, loading=null, activeJob=null, result=null, playing=false, lastFrame=0, playbackTime=0, downloadUrl=null;
  const vectors = role => [['position','Position · m',['X','Y · altitude','Z']],['velocity','Velocity · m/s',['Vx','Vy','Vz']],['angles','Orientation · degrees',['Heading','Pitch','Roll']]].map(([key,label,axes]) =>
    `<div class="missile-vector"><span>${label}</span><div class="three-inputs">${axes.map((axis,i)=>`<label>${axis}<input id="missile-${role}-${key}-${i}" aria-label="${role} ${axis}" type="number" step="any" required value="${initial[role][key][i]}" ${key==='position'&&i===1?'min="0.001" max="30000"':key==='velocity'?'min="-2000" max="2000"':key==='angles'?`min="${i===1?-90:-360}" max="${i===1?90:360}"`:'min="-200000" max="200000"'}></label>`).join('')}</div></div>`).join('');
  root.innerHTML = `<aside class="missile-sidebar"><form id="missile-form">
    <div class="section-heading"><span class="eyebrow">ENGAGEMENT SETUP</span><button id="missile-reset" class="text-button" type="button">Reset</button></div>
    <label class="field">Missile<select id="missile-select" required><option value="">Loading configurations…</option></select></label>
    <h2>Missile at release</h2>${vectors('launch')}
    <p class="hint">Velocity and body orientation are independent. Values describe the missile immediately after release.</p>
    <h2>Target aircraft</h2>${vectors('target')}
    <p class="hint">Constant velocity and orientation. Changing the target’s heading does not change its velocity or prescribe a turn.</p>
    <label class="field">Maximum simulation time<div class="input-unit"><input id="missile-duration" type="number" min="0.1" max="180" step="any" value="60" required><span>s</span></div></label>
    <details class="missile-details"><summary>Coordinate conventions</summary><p>X and Z are horizontal; Y is altitude. Zero heading points along +X; +90° heading points along +Z. Positive pitch points upward. Roll rotates about the forward axis.</p></details>
    <button id="missile-run" class="primary-button" type="submit" disabled>Simulate engagement <span>↗</span></button>
    <button id="missile-cancel" class="secondary-button" type="button" hidden>Cancel simulation</button>
    <div id="missile-status" role="status" aria-live="polite"></div>
  </form></aside>
  <div class="missile-main">
    <div class="page-heading"><div><span class="eyebrow">MISSILE FLIGHT</span><h1>3D engagement simulator</h1></div><span class="status-badge">EXPERIMENTAL</span></div>
    <p class="missile-model-note">Ideal target visibility and radar support, with geometric seeker limits and recovered flight physics. The missile continues after a miss until interception, a modeled limit, or the selected time limit.</p>
    <div id="missile-error" class="error" role="alert" hidden></div>
    <div id="missile-stale" class="notice" hidden>Inputs changed. Run again to update the trajectories.</div>
    <section class="chart-card"><div class="chart-toolbar"><div class="missile-legend"><span>━ Missile</span><span>━ Target</span></div><button id="missile-view-reset" type="button">Reset 3D view</button></div>
    <div id="missile-chart" aria-label="Interactive 3D missile and target trajectories"><div class="empty"><span class="empty-glyph">↗</span><h2>Set an engagement</h2><p>Enter both initial states, then simulate.</p></div></div>
    <div class="missile-playback"><button id="missile-play" type="button" disabled>Play</button><input id="missile-time" type="range" aria-label="Trajectory time" min="0" max="0" value="0" step="1" disabled><output id="missile-time-label">0.00 s</output><select id="missile-play-speed" aria-label="Playback speed"><option value="1">1×</option><option value="4" selected>4×</option><option value="10">10×</option></select></div></section>
    <p id="missile-frame" class="missile-frame-readout"></p>
    <div class="missile-metrics"><div class="missile-metric"><span>Outcome</span><strong id="missile-outcome">—</strong></div><div class="missile-metric"><span>Closest approach</span><strong id="missile-closest">—</strong></div><div class="missile-metric"><span>Elapsed time</span><strong id="missile-elapsed">—</strong></div></div>
    <div class="exports"><span>EXPORT</span><a id="missile-download" aria-disabled="true">Trajectory JSON</a></div>
    <details class="missile-details"><summary>Model and current limitations</summary><p>Recovered profile baseline: War Thunder 2.59.0.34. Seekers begin with warm-up complete, respecting designation and angle limits. Radar missiles retain their lock-before-launch or lock-after-launch behavior. Ideal radar support moves from the launch position at the entered launch velocity; recovered inertial guidance remains active where configured. Target geometry is a point, so a proximity event does not assert aircraft damage. Full engagements and native collision timing are still being validated.</p><ul id="missile-model-details"></ul></details>
  </div>`;
  const apiBase=String((window.EM_CONFIG||{}).apiBase||'').replace(/\/$/,'');
  async function request(path, options={}) {
    const response=await fetch(apiBase?apiBase+path:'.'+path,options);
    const body=await response.json();
    if(!response.ok) throw new Error(body.error||`Request failed (${response.status})`);
    return body;
  }
  const showError = text => {el('missile-error').textContent=text;el('missile-error').hidden=!text;};
  const pause = () => {playing=false;el('missile-play').textContent='Play';};
  function selectTab(missile) {
    el('em-workspace').hidden=missile;root.hidden=!missile;
    for(const [id,selected] of [['em-tab',!missile],['missile-tab',missile]]) {
      el(id).setAttribute('aria-selected',String(selected));el(id).tabIndex=selected?0:-1;
    }
    if(missile) {
      if(!meta) load();
      if(result) requestAnimationFrame(()=>Plotly.Plots.resize(el('missile-chart')));
    } else {pause(); if(el('chart').data) requestAnimationFrame(()=>Plotly.Plots.resize(el('chart')));}
  }
  el('em-tab').addEventListener('click',()=>selectTab(false));
  el('missile-tab').addEventListener('click',()=>selectTab(true));
  document.querySelector('.tool-tabs').addEventListener('keydown',event=>{
    if(!['ArrowLeft','ArrowRight','Home','End'].includes(event.key)) return;
    event.preventDefault();const missile=event.key==='End'||(event.key!=='Home'&&root.hidden);
    selectTab(missile);el(missile?'missile-tab':'em-tab').focus();
  });
  async function load() {
    if(loading) return loading;
    loading=(async()=>{try{
      showError('');meta=await request('/api/missiles/meta');const select=el('missile-select');select.replaceChildren();
      for(const family of ['optical','radar']) {
        const group=document.createElement('optgroup');group.label=family==='optical'?'Optical / infrared':'Radar';
        for(const item of meta.missiles.filter(x=>x.family===family)) {const option=document.createElement('option');option.value=item.id;option.textContent=item.name;group.append(option);}
        select.append(group);
      }
      select.value=initial.missile;el('missile-run').disabled=false;
    }catch(error){showError('Missile backend unavailable. The server needs the missile simulator update. '+error.message);}
    finally{loading=null;}})();return loading;
  }
  function inputs() {
    const config={missile:el('missile-select').value,duration:Number(el('missile-duration').value)};
    for(const role of ['launch','target']) config[role]=Object.fromEntries(['position','velocity','angles'].map(key=>[key,[0,1,2].map(i=>Number(el(`missile-${role}-${key}-${i}`).value))]));
    return config;
  }
  function busy(value) {el('missile-run').disabled=value||!meta;el('missile-cancel').hidden=!value;el('missile-reset').disabled=value;}
  el('missile-form').addEventListener('input',()=>{if(result)el('missile-stale').hidden=false;});
  el('missile-reset').addEventListener('click',()=>{
    el('missile-select').value=initial.missile;el('missile-duration').value=initial.duration;
    for(const role of ['launch','target'])for(const key of ['position','velocity','angles'])for(let i=0;i<3;i++)el(`missile-${role}-${key}-${i}`).value=initial[role][key][i];
    if(result)el('missile-stale').hidden=false;if(!meta)load();
  });
  el('missile-form').addEventListener('submit',async event=>{
    event.preventDefault();if(activeJob)return;pause();showError('');busy(true);
    const submitted=inputs();el('missile-status').textContent='Submitting engagement…';
    try {
      const job=await request('/api/missiles/jobs',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(submitted)});activeJob=job.id;
      for(;;) {
        const status=await request('/api/missiles/jobs/'+activeJob);
        if(status.status==='complete') {result=status.result;await render();el('missile-status').textContent='Simulation complete.';el('missile-stale').hidden=JSON.stringify(inputs())===JSON.stringify(submitted);break;}
        if(status.status==='failed')throw new Error(status.error);
        if(status.status==='cancelled'){el('missile-status').textContent='Simulation cancelled.';break;}
        const p=status.progress;el('missile-status').textContent=p?`Simulating ${p.time_s.toFixed(1)} / ${p.duration_s.toFixed(1)} s…`:status.status==='queued'?'Waiting for simulation worker…':'Simulating engagement…';
        await new Promise(resolve=>setTimeout(resolve,600));
      }
    }catch(error){showError(error.message);el('missile-status').textContent='Simulation did not complete.';}
    finally{activeJob=null;busy(false);}
  });
  el('missile-cancel').addEventListener('click',async()=>{
    if(!activeJob)return;try{await request('/api/missiles/jobs/'+activeJob+'/cancel',{method:'POST'});}catch(error){showError(error.message);}
  });
  const camera={eye:{x:1.45,y:1.4,z:.8},up:{x:0,y:0,z:1}};
  const trackingLabels={observed:'Target observed',coasting:'Coasting on the previous track',inertial:'Inertial guidance · seeker not tracking',acquiring:'Acquiring / searching for target'};
  async function render() {
    const rows=result.trajectory;
    const trace=(role,color)=>({type:'scatter3d',mode:'lines',name:role==='missile'?'Missile':'Target',x:rows.map(r=>r[role][0]),y:rows.map(r=>r[role][2]),z:rows.map(r=>r[role][1]),customdata:rows.map(r=>r.time_s),line:{color,width:5},hovertemplate:'%{customdata:.2f} s<br>X %{x:.1f} m<br>Z %{y:.1f} m<br>Altitude %{z:.1f} m<extra>%{fullData.name}</extra>'});
    const marker=(name,color)=>({type:'scatter3d',mode:'markers',name,x:[],y:[],z:[],marker:{color,size:5},hoverinfo:'skip',showlegend:false});
    const bounds=[0,2,1].map(i=>{
      const values=rows.flatMap(r=>[r.missile[i],r.target[i]]);
      return [Math.min(...values),Math.max(...values)];
    });
    const span=Math.max(100,...bounds.map(([low,high])=>high-low))*1.12;
    const axis=(title,index)=>{const center=(bounds[index][0]+bounds[index][1])/2;return {title,range:[center-span/2,center+span/2],gridcolor:'#263244',zerolinecolor:'#455568',color:'#a9b8ca',backgroundcolor:'#121925'};};
    await Plotly.newPlot(el('missile-chart'),[trace('missile','#38c9d7'),trace('target','#ffa66b'),marker('Missile now','#9beaf0'),marker('Target now','#ffd0ac')],
      {paper_bgcolor:'#121925',plot_bgcolor:'#121925',font:{color:'#dce5f0'},margin:{l:0,r:0,t:0,b:0},showlegend:false,scene:{xaxis:axis('X · m',0),yaxis:axis('Z · m',1),zaxis:axis('Altitude · m',2),aspectmode:'cube',camera},uirevision:'missile'},
      {responsive:true,displaylogo:false,modeBarButtonsToRemove:['toImage']});
    el('missile-time').max=rows.length-1;el('missile-time').value=0;el('missile-time').disabled=false;el('missile-play').disabled=false;
    el('missile-outcome').textContent=result.outcome.label;el('missile-closest').textContent=result.closest_approach.distance_m.toFixed(2)+' m';el('missile-elapsed').textContent=result.elapsed_s.toFixed(2)+' s';
    el('missile-model-details').replaceChildren(...result.model.limitations.map(text=>{const li=document.createElement('li');li.textContent=text;return li;}));
    if(downloadUrl)URL.revokeObjectURL(downloadUrl);downloadUrl=URL.createObjectURL(new Blob([JSON.stringify(result,null,2)],{type:'application/json'}));
    el('missile-download').href=downloadUrl;el('missile-download').download=result.scenario.missile+'-engagement.json';el('missile-download').setAttribute('aria-disabled','false');
    updateFrame(0);
  }
  function updateFrame(index) {
    if(!result)return;const row=result.trajectory[index];el('missile-time').value=index;playbackTime=row.time_s;
    el('missile-time-label').textContent=row.time_s.toFixed(2)+' s';
    el('missile-frame').textContent=row.kind==='proximity_event'?'Target proximity event · position and time from the model':`Missile speed ${row.speed_mps.toFixed(1)} m/s · ${trackingLabels[row.tracking_state]||(row.tracking?'Tracking target':'Acquiring / geometric track unavailable')}`;
    Plotly.restyle(el('missile-chart'),{x:[[row.missile[0]],[row.target[0]]],y:[[row.missile[2]],[row.target[2]]],z:[[row.missile[1]],[row.target[1]]]},[2,3]);
  }
  el('missile-time').addEventListener('input',()=>{pause();updateFrame(Number(el('missile-time').value));});
  el('missile-view-reset').addEventListener('click',()=>{if(result)Plotly.relayout(el('missile-chart'),{'scene.camera':camera});});
  el('missile-play').addEventListener('click',()=>{
    if(playing){pause();return;}if(!result)return;
    if(Number(el('missile-time').value)>=result.trajectory.length-1)updateFrame(0);
    playing=true;lastFrame=performance.now();el('missile-play').textContent='Pause';requestAnimationFrame(animate);
  });
  function animate(now) {
    if(!playing)return;const wanted=playbackTime+(now-lastFrame)/1000*Number(el('missile-play-speed').value);lastFrame=now;
    let index=Number(el('missile-time').value);while(index<result.trajectory.length-1&&result.trajectory[index+1].time_s<=wanted)index++;
    updateFrame(index);playbackTime=wanted;
    if(index>=result.trajectory.length-1)pause();else requestAnimationFrame(animate);
  }
})();
