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



$dump=JS '(()=>{const a=[],dyn=CART_DEMO.scene.getObjectByName("dispatch").parent,T=THREE,im=new T.Matrix4(),wm=new T.Matrix4();CART_DEMO.scene.updateMatrixWorld(true);CART_DEMO.scene.traverse(o=>{if(!o.isMesh||Array.isArray(o.material)||o.material.visible===false)return;let p=o,skip=false;while(p){if(p===dyn||p.visible===false){skip=true;break}p=p.parent}if(skip)return;const g=o.geometry.parameters;if(!g||g.width!==1||g.height!==1||g.depth!==1)return;const n=o.isInstancedMesh?o.count:1;for(let i=0;i<n;i++){if(o.isInstancedMesh){o.getMatrixAt(i,im);wm.multiplyMatrices(o.matrixWorld,im)}else wm.copy(o.matrixWorld);a.push({m:wm.elements.slice(),color:o.material.color?.getHex()})}});return JSON.stringify(a)})()'
[IO.File]::WriteAllText("\\wsl.localhost\Ubuntu-22.04\tmp\audit36-static-boxes.json",$dump)
Write-Output 'STATIC_BOXES_EXPORTED'
