$ErrorActionPreference='Stop'
$port=9391
try {$pages=Invoke-RestMethod "http://localhost:$port/json/list"} catch {$profile=Join-Path $env:TEMP 'brickflow-qa-9391';Start-Process 'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe' -ArgumentList @('--headless=new','--disable-gpu','--disable-extensions','--no-first-run','--disable-background-timer-throttling','--disable-renderer-backgrounding','--allow-file-access-from-files',"--remote-debugging-port=$port",'--remote-allow-origins=*',"--user-data-dir=$profile",'about:blank');Start-Sleep -Seconds 4;$pages=Invoke-RestMethod "http://localhost:$port/json/list"}
$page=$null;foreach($p in $pages){if($p.type -eq 'page' -and ($p.url -like '*greenfield*' -or $p.url -eq 'about:blank')){$page=$p;break}};if(!$page){foreach($p in $pages){if($p.type -eq 'page'){$page=$p;break}}}
$ws=New-Object System.Net.WebSockets.ClientWebSocket
$ws.ConnectAsync([Uri]$page.webSocketDebuggerUrl,[Threading.CancellationToken]::None).GetAwaiter().GetResult()|Out-Null
$script:serial=0
function CDP($method,$params){$script:serial++;$id=$script:serial;$msg=@{id=$id;method=$method;params=$params}|ConvertTo-Json -Depth 30 -Compress;$bytes=[Text.Encoding]::UTF8.GetBytes($msg);$ws.SendAsync([ArraySegment[byte]]::new($bytes),[Net.WebSockets.WebSocketMessageType]::Text,$true,[Threading.CancellationToken]::None).GetAwaiter().GetResult();do{$ms=New-Object IO.MemoryStream;do{$b=New-Object byte[] 1048576;$r=$ws.ReceiveAsync([ArraySegment[byte]]::new($b),[Threading.CancellationToken]::None).GetAwaiter().GetResult();$ms.Write($b,0,$r.Count)}while(!$r.EndOfMessage);$reply=[Text.Encoding]::UTF8.GetString($ms.ToArray())|ConvertFrom-Json}while($reply.id -ne $id);return $reply}
function JS($expr){$r=CDP 'Runtime.evaluate' @{expression=$expr;returnByValue=$true;awaitPromise=$true};if($r.result.exceptionDetails){Write-Output ($r.result.exceptionDetails|ConvertTo-Json -Depth 8)};return $r.result.result.value}
CDP 'Emulation.setDeviceMetricsOverride' @{width=1600;height=1000;deviceScaleFactor=1;mobile=$false}|Out-Null
CDP 'Page.navigate' @{url='file://wsl.localhost/Ubuntu-22.04/home/vjhannox/hackathon/robo.innovate-hackathon-TUM/artifacts/greenfield_36/index.html'}|Out-Null
CDP 'Page.reload' @{}|Out-Null
Start-Sleep -Seconds 4
for($i=0;$i -lt 240;$i++){Start-Sleep -Milliseconds 500;$state=JS 'JSON.stringify({ready:!!window.CART_DEMO,error:window.CART_ERROR})';if($state -match '"ready":true' -or $state -match '"error":'){break}}
Write-Output $state
Write-Output (JS 'JSON.stringify({complete:CART_DEMO.model.completeInput,rate:CART_DEMO.model.bph,views:CART_DEMO.model.bricks[0].inspectionViews})')
JS "if(document.getElementById('play').textContent==='Pause')document.getElementById('play').click();true"|Out-Null



JS "document.getElementById('inspectionCutaway').click();document.getElementById('routingCutaway').click();true"|Out-Null
$checks=@(
@{name='sensor-base';expr="CART_DEMO.seek(300);CART_DEMO.camera.position.set(27,1.20,-14.0);CART_DEMO.camera.lookAt(27,1.18,-13.20);true"},
@{name='decline';expr="(()=>{const b=CART_DEMO.model.bricks.find(b=>b.c===0&&!b.good&&b.tracks.some(s=>s.kind==='supported-reject-decline')),s=b.tracks.find(s=>s.kind==='supported-reject-decline');let ds=[];for(let i=1;i<s.path.length;i++)ds.push(Math.hypot(...s.path[i].map((v,j)=>v-s.path[i-1][j])));const u=(ds[0]+ds[1]+ds[2]/2)/ds.reduce((a,b)=>a+b,0),t=s.t0+(s.t1-s.t0)*u;CART_DEMO.seek(t);CART_DEMO.camera.position.set(26.3,1.9,-11.2);CART_DEMO.camera.lookAt(27.5,.9,-11.7);return JSON.stringify({brick:b.id,time:t,track:s})})()"},
@{name='mixed-height';expr="(()=>{const m=CART_DEMO.model,p=m.pallets[0];let found=null;for(let h=0;h<12&&!found;h++)for(let q=0;q<40&&!found;q++){const ids=[p.ids[h*40+q],p.ids[(h+1)*40+q],p.ids[(h+2)*40+q]];if(new Set(ids.map(i=>m.rec(i).shade)).size===1)found={ids,h,q}}CART_DEMO.seek(p.t+5);for(const id of found.ids){const r=m.rec(id),g=new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(.251,.073,.073)),new THREE.LineBasicMaterial({color:0xff2754,depthTest:false}));g.position.set(...r.p.slice(0,3));g.renderOrder=30;g.name='AUDIT_THREE_EQUAL_SHADES';CART_DEMO.scene.add(g)}const pos=m.rec(found.ids[1]).p;CART_DEMO.camera.position.set(pos[0]+2.1,1.7,pos[2]-2.0);CART_DEMO.camera.lookAt(pos[0],pos[1],pos[2]);return JSON.stringify({pallet:p.t,...found,position:pos})})()"},
@{name='source-transfer';expr="(()=>{const s=CART_DEMO.model.rec('head-0-0').tracks.find(s=>s.kind==='distribute-2x6-layers');CART_DEMO.seek((s.t0+s.t1)/2);CART_DEMO.view('input');return JSON.stringify({time:(s.t0+s.t1)/2})})()"},
@{name='row-transfer';expr="(()=>{const s=CART_DEMO.model.rec('row-0-0').tracks.find(s=>s.kind==='side-pick-and-transfer-rows');CART_DEMO.seek((s.t0+s.t1)/2);CART_DEMO.view('rows');return JSON.stringify({time:(s.t0+s.t1)/2})})()"}
)
foreach($q in $checks){Write-Output (JS $q.expr);Start-Sleep -Milliseconds 700;$shot=CDP 'Page.captureScreenshot' @{format='png'};[IO.File]::WriteAllBytes("\\wsl.localhost\Ubuntu-22.04\tmp\audit36-$($q.name).png",[Convert]::FromBase64String($shot.result.data))}
