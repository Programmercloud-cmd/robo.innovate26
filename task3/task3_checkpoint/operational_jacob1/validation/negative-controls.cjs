/* Fault injection proves the independent auditor rejects material/resource faults. */
'use strict';
const fs=require('fs'),path=require('path'),{audit}=require('./audit-run.cjs');
const args=process.argv.slice(2),get=(k,f)=>{const i=args.indexOf(k);return i>=0?args[i+1]:f;};
const manifest=JSON.parse(fs.readFileSync(get('--manifest',path.resolve(__dirname,'../geometry-manifest.json')),'utf8'));
const input=get('--run');
const run=input?JSON.parse(fs.readFileSync(input,'utf8')):require(path.resolve(get('--engine',path.resolve(__dirname,'../engine/operational-engine.js')))).createRun(manifest,JSON.parse(get('--options','{}')));
const baseline=audit(run,manifest),results=[];
function test(name,mutate,expected){const changed=structuredClone(run);mutate(changed);const r=audit(changed,manifest),failed=r.checks.filter(c=>!c.passed).map(c=>c.name);results.push({name,passed:!r.passed&&failed.some(x=>x.includes(expected)),failedChecks:failed});}
test('duplicate brick in a formed layer',r=>{r.layers[0].ids[1]=r.layers[0].ids[0];},'layers contain forty');
test('overlapping exclusive zone owners',r=>{r.zones.push({id:'FAULT:exclusive-buffer',owner:'FAULT:A',start:1,end:5,kind:'bufferA'},{id:'FAULT:exclusive-buffer',owner:'FAULT:B',start:2,end:4,kind:'bufferA'});},'occupied resource calendars');
test('out-of-order pallet layers',r=>{[r.pallets[0].layers[0],r.pallets[0].layers[1]]=[r.pallets[0].layers[1],r.pallets[0].layers[0]];},'completed pallets are fourteen');
test('duplicate identity in a ledger transaction',r=>{const rows=r.transactions||r.ledger,tx=rows.find(x=>x.ids?.length);tx.ids.push(tx.ids[0]);},'ownership transitions');
const result={method:'Independent negative controls mutate a fresh newly generated run; baseline must pass',baselinePassed:baseline.passed,passed:baseline.passed&&results.every(r=>r.passed),results};
const out=get('--out');if(out)fs.writeFileSync(out,JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result,null,2));process.exitCode=result.passed?0:1;
