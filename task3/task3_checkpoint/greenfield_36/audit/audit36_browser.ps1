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


$checks=@(
@{name='layer-grip';expr="(()=>{const m=CART_DEMO.model,s=m.rec('stacker-0').tracks.find(s=>s.kind==='lift-complete-layer');CART_DEMO.seek((s.t0+s.t1)/2);CART_DEMO.camera.position.set(47.5,2.35,s.a[2]+1.8);CART_DEMO.camera.lookAt(49,1.58,s.a[2]);return JSON.stringify({stage:'layer-grip',time:(s.t0+s.t1)/2,head:s})})()"},
@{name='centering';expr="(()=>{const s=CART_DEMO.model.rec('jaw-0-1-1').tracks.find(s=>s.kind==='center-placed-layer');CART_DEMO.seek(s.t1+.05);CART_DEMO.camera.position.set(51.8,2.0,-11.6);CART_DEMO.camera.lookAt(53,1.25,-13);return JSON.stringify({stage:'centering',time:s.t1+.05,jaw:s})})()"},
@{name='pallet-fork';expr="(()=>{const s=CART_DEMO.model.rec('dispatch').tracks.find(s=>s.kind==='lift-supported-output-pallet');CART_DEMO.seek(s.t0+.05);CART_DEMO.camera.position.set(55.7,.65,s.a[2]+1.8);CART_DEMO.camera.lookAt(57,.24,s.a[2]+.35);return JSON.stringify({stage:'fork',time:s.t0+.05,head:s})})()"},
@{name='pallet-cart';expr="(()=>{const m=CART_DEMO.model,s=m.rec('dispatch').tracks.find(s=>s.kind==='lower-pallet-on-transport-cart');CART_DEMO.seek(s.t1+.10);CART_DEMO.camera.position.set(s.b[0]-1.4,.70,s.b[2]+1.5);CART_DEMO.camera.lookAt(s.b[0],.35,s.b[2]);return JSON.stringify({stage:'cart',time:s.t1+.1,head:s})})()"},
@{name='input-start';expr="CART_DEMO.seek(0);CART_DEMO.camera.position.set(-18,3.4,7);CART_DEMO.camera.lookAt(-16.75,1,0);true"}
)
foreach($q in $checks){Write-Output (JS $q.expr);Start-Sleep -Milliseconds 700;$shot=CDP 'Page.captureScreenshot' @{format='png'};[IO.File]::WriteAllBytes("\\wsl.localhost\Ubuntu-22.04\tmp\audit36-$($q.name).png",[Convert]::FromBase64String($shot.result.data))}
