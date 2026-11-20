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



Write-Output (JS "(()=>{const all=CART_DEMO.scene.children;const well=all.find(o=>o.userData.name?.includes('guided pallet well 2'));CART_DEMO.showEquipment(well);return JSON.stringify({title:document.getElementById('inspectorTitle').textContent,text:document.getElementById('inspectorBody').textContent})})()")
Start-Sleep -Milliseconds 500
$shot=CDP 'Page.captureScreenshot' @{format='png'};[IO.File]::WriteAllBytes("\\wsl.localhost\Ubuntu-22.04\tmp\audit36-well-reference.png",[Convert]::FromBase64String($shot.result.data))
JS "document.getElementById('closeInspector').click();true"|Out-Null
Write-Output (JS "(()=>{const cart=CART_DEMO.scene.children.find(o=>o.userData.name?.includes('covered discard cart'));CART_DEMO.showEquipment(cart);return JSON.stringify({title:document.getElementById('inspectorTitle').textContent,text:document.getElementById('inspectorBody').textContent})})()")
JS "document.getElementById('closeInspector').click();CART_DEMO.seek(700);window.oldModel=CART_DEMO.model;document.getElementById('recipe').value='single';document.getElementById('apply').click();JSON.stringify({unchanged:CART_DEMO.model===oldModel,disabled:document.getElementById('restart').disabled})"|Write-Output
JS 'CART_DEMO.seek(CART_DEMO.model.total);true'|Out-Null
Start-Sleep -Milliseconds 400
JS "document.getElementById('restart').click();true"|Out-Null
for($i=0;$i -lt 100;$i++){Start-Sleep -Milliseconds 500;$state=JS 'JSON.stringify({changed:CART_DEMO.model!==oldModel,error:window.CART_ERROR})';if($state -match '"changed":true'){break}}
Start-Sleep -Seconds 1
Write-Output (JS 'JSON.stringify({nextCart:CART_DEMO.model!==oldModel,mode:CART_DEMO.requests[0]?.mode,pallets:CART_DEMO.model.pallets.length,allSingle:CART_DEMO.model.pallets.every(p=>p.shade!==null),time:document.getElementById("time").value,playButton:document.getElementById("play").textContent,error:window.CART_ERROR})')
