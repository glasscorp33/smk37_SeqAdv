  // SPDX-License-Identifier: GPL-3.0-only WITH GPL-3.0-linking-exception
  // SeqAdv sound controls for the simulator page. tools/Build-Web.ps1 inserts
  // this file into sdk/Simulator.js at page-build time (sdk/ is hash-pinned),
  // inside its page closure, so send, start, say, context and muted are in scope.
  // The C side is web/engine/seqadv_sim.c.
  const seqadvSound=(()=>{
    // Two audio blocks: SeqAdv notes reach the simulator's audio about this
    // long after the block that produced them; keep Web MIDI level with it.
    const lead=2*128/44.1;
    let internal=true,access,output,connecting=false;
    const row=document.createElement('div');row.className='sim-file-row';row.id='seqadv-sound';
    row.innerHTML='<strong>SeqAdv sound</strong>'+
      '<label><input id="seqadv-internal" type="checkbox" checked> Simulator instrument</label>'+
      '<button id="seqadv-midi">Connect MIDI output</button>'+
      '<select id="seqadv-port" hidden aria-label="SeqAdv MIDI output port"></select>';
    $('sim-status').after(row);
    const port=$('seqadv-port'),button=$('seqadv-midi');
    if(!navigator.requestMIDIAccess){button.disabled=true;button.title='This browser offers no Web MIDI here. Use desktop Chrome or Edge.';}
    const apply=()=>send('seqadv-sound',{flags:(internal?1:0)|(output?2:0)});
    function allOff(target){
      if(!target)return;
      try{for(let ch=0;ch<16;ch++){target.send([0xb0|ch,64,0]);target.send([0xb0|ch,123,0]);}}catch(_){}
    }
    function choose(id){
      const next=id?access?.outputs.get(id):undefined;
      if(next===output)return;
      allOff(output);output=next;apply();
      if(output)output.open?.().catch(()=>{});
      say(output?'SeqAdv MIDI goes to '+output.name+'. Enter SeqAdv with SHIFT + highest key.':'SeqAdv MIDI output off.');
    }
    function refresh(){
      const selected=output?.id||'';
      if(output&&output.state!=='connected'){output=undefined;apply();say('SeqAdv MIDI output disconnected.');}
      port.replaceChildren();
      const none=document.createElement('option');none.value='';none.textContent='No MIDI output';port.append(none);
      for(const item of access.outputs.values()){
        if(item.state!=='connected')continue;
        const option=document.createElement('option');option.value=item.id;option.textContent=item.name;port.append(option);
      }
      port.value=output?selected:'';
    }
    $('seqadv-internal').onchange=event=>{internal=event.target.checked;apply();};
    port.onchange=()=>choose(port.value);
    button.onclick=()=>action(async()=>{
      if(connecting)return;
      if(!ready)throw Error('The simulator is still loading.');
      connecting=true;button.disabled=true;
      try{
        // The audio thread gives SeqAdv a steady clock; start it like MIDI input does.
        const audio=muted?Promise.resolve():start(true);
        access=await deadline(navigator.requestMIDIAccess({sysex:false}),30000,
          'Web MIDI is not available yet. Allow MIDI for this page in the browser (site settings) and try again.');
        await audio;
        access.onstatechange=refresh;refresh();port.hidden=false;
        say(port.options.length>1?'Choose the SeqAdv MIDI output port (for example the S-1).':'No MIDI output found. Connect the synth by USB, then press Connect MIDI output again.');
      }finally{connecting=false;button.disabled=false;}
    });
    $('sim-panic').addEventListener('click',()=>allOff(output));
    // Packed messages from sim_seqadv_out. time: AudioContext time of the
    // block that sent them (audio thread), or undefined (send at once).
    function out(list,time){
      if(!output||!list)return;
      let at=0;
      if(time!==undefined&&context?.getOutputTimestamp){
        const stamp=context.getOutputTimestamp();
        if(stamp.performanceTime)at=stamp.performanceTime+(time-stamp.contextTime)*1000+lead;
      }
      const late=at<=performance.now();
      for(const value of list){
        const bytes=[value&255,value>>>8&255,value>>>16&255].slice(0,value>>>24);
        try{late?output.send(bytes):output.send(bytes,at);}catch(_){}
      }
    }
    return {out};
  })();
