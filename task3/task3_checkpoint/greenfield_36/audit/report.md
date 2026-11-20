# Auditoría independiente de Greenfield 36

Objeto congelado: commit `f173c74`, `artifacts/greenfield_36/index.html`, replays `468422bd7ba526f2`. No se modificó el repositorio y no se auditó la versión 37. Fuentes fijadas en `/tmp/audit36-pinned/`.

## Conclusión

La versión 36 reproduce ambos lotes de 28.800 bricks, conserva material y calcula correctamente los ratios de pallets completos. Sin embargo, no debe describirse como libre de errores visuales o de control. Los problemas principales son la geometría de agarre/centrado, la mezcla en altura del pallet y la ocupación de celdas cuando un brick deja de moverse. Hay otros defectos de apoyo y montaje visibles en primeros planos.

## Hallazgos prioritarios

### P1 — Las garras y el centrador de layers penetran los bricks

Los destinos están separados 75 mm en Z: ancho exterior `9 × .075 + .067 = .742 m`. Los bricks no se compactan después. El EOAT cierra sus pads alrededor de un ancho interior de aproximadamente .668 m y el centrador llega a .670 m. Los pads quedan dentro de los bricks de los extremos; el centrador penetra 36 mm por lado. No se trata de deformación física simulada: las posiciones Z de los bricks no cambian.

Fuentes: `cart_model.js:77,93,100`, `viewer_cart.js:25,128,134`. Evidencia: t=368.1864387 s, `stacker-0` levantando el primer layer; t=377.2615759 s, `jaw-0-1-1` cerrado. `/tmp/audit36-layer-grip.png`, `/tmp/audit36-centering.png`.

Corrección: unificar el ancho formado y el ancho de herramienta, o incorporar una compactación real y visible de los 40 bricks con su fase/interbloqueo. No estrechar sólo las mordazas mientras los bricks conservan el paso de75mm.

### P1 — El algoritmo mixed no evita tres tonos iguales en altura del pallet

`canPlace()` sólo comprueba el grid 4×10 de un layer. Las mismas posiciones de los layers sucesivos se apilan verticalmente sin restricciones contra los dos layers anteriores. En los 42 pallets mixtos aparecen **2.478 ventanas de tres bricks del mismo tono en altura**. Dentro del plano de los layers hay cero triples, de modo que la comprobación anterior de mezcla pasa pero no cubre las caras verticales del pallet. Si el requisito es el feedback del usuario de no tres iguales contiguos en el pallet visible, incumple esa parte.

Fuentes: `cart_model.js:82,93,100`. Ejemplo visible del primer pallet: IDs `[7166,7169,7168]`, primera posición espacial q=3, heights0/1/2, mismo tono. A t=1453.1419237 s están en x70.3675,z−7.8625. `/tmp/audit36-mixed-height.png` añade contornos de auditoría a esos tres bricks; no cambia el modelo.

Corrección: comprobar los vecinos de los layers precedentes de ese pallet y validar después las caras X/Z/altura; la excepción monocolor debe seguir siendo intencionada.

### P1 — La exclusividad de celdas no protege bricks estacionados

`path()` reserva ticks sólo durante el trayecto. Tras la llegada no retiene las celdas del brick mientras espera a completar el layer. La geometría actual hace que bricks vecinos de un mismo row compartan celdas80×40mm. Se encontraron **22.572 pares vecinos** con celdas finales compartidas y llegadas distintas: el primer brick permanece parado mientras la ruta del segundo usa esas celdas. El algoritmo no representa el conflicto entre detener uno y accionar al otro.

Fuentes: `cart_model.js:107–115`, especialmente113–114. Evidencia cuantitativa: IDs22389/22386, lane4/station0, llegadas265.7108741/269.3095470s; el layer no sale hasta350.3315728s. Comparten celdas X577..580,Z138. `/tmp/audit36_cells.py` reproduce el cálculo.

Esto no prueba una colisión geométrica entre los dos cuerpos; prueba una contradicción de ocupación/accionamiento respecto a la exclusividad anunciada. Un lock permanente solo tampoco soluciona el diseño, porque los destinos ya comparten celdas.

Corrección: dimensionar contactos/celdas y posiciones finales conjuntamente, retener la ocupación hasta la salida y/o separar transporte a paso compatible de compactación mecánica final.

### P1 — Los bricks no inclinan su cuerpo al bajar por la faja roja

La última sección pasa de(27.7,1.0455) a(27.2,.74): pendiente31.4°. El centro del brick sigue esa pendiente, pero `setBrick()` sólo aplica yaw; su cara inferior permanece horizontal. En el centro del tramo, el extremo cuesta arriba penetra aproximadamente74.8mm en el plano de la faja (más que los69mm de altura del brick), y el otro extremo queda separado. La tapa oculta parte del defecto pero no lo elimina.

Fuentes: `cart_model.js:80,119`, `viewer_cart.js:50,133`. Ejemplo brick3 a t=8881.8505837 s. `/tmp/audit36-decline.png`. El asentamiento dentro del carrito está declarado como abstracción y no se incluye en este hallazgo.

Corrección: orientar el brick según el plano inclinado y su normal, conservando el semiespesor correcto; no hace falta rigid-body.

## Otros hallazgos confirmados

### P2 — Colisiones en horquillas, pallets y carros

- Las horquillas de salida están correctamente bajo las tablas superiores, pero demasiado cerca de los largueros exteriores: sus intervalosZ invaden30mm los largueros y sus intervalosY se solapan50mm. Ejemplo primer pick a t1423.6259837s. `/tmp/audit36-pallet-fork.png`.
- Al depositarse un pallet en el carro, el fondo del larguero está enY=.3055m y las barras metálicas llegan aY=.345m: penetración39.5mm. Primer depósito t1448.0419237s. `/tmp/audit36-pallet-cart.png`.
- Las ruedas de `cart()` tienen centroY=.08m y radio.17m, de modo que llegan aY=−.09m. El suelo termina enY=−.02m y las vías tienen superficies positivas: ruedas hundidas70mm respecto al suelo.

Fuentes: `cart_model.js:104–106`; `viewer_cart.js:23–24,28,110,126,134`. Corregir las horquillas para entrar en los huecos entre largueros o definir otro apoyo; usar datums comunes para suelo/vías, carros y pallet.

### P2 — Sensores de inspección sin unión al bastidor

El helper `sensor()` coloca postes enZ−245mm. La mesa de inspección mide340mm (bordeZ−170mm); los postes terminan enY1.145m, por encima del suelo, pero lateralmente fuera del bastidor. Los motores vecinos y las placas de zonas tampoco alcanzan su X/Z. Esto afecta a los7sensores ZPA y los2sensores de cámara por lane:54montajes repetidos. Los postes/soportes de los sensores de infeed y de row sí están sobre una mesa ancha y no se incluyen.

Fuentes: `viewer_cart.js:19,21,40,45`. Es necesario un soporte desde el lateral del bastidor o reposicionar el poste; tener un poste no garantiza que esté anclado.

### P2 — La franja amarilla de la faja principal es un obstáculo sólido

`box(10.7,B+.006,z,.025,.012,1.78,...)` sobresale12mm sobre la superficie. Todos los bricks atraviesan esa posición en el indexado común. Un barrido de1.968.950puntosmedios contra3.426cajas estáticas alineadas detectó960penetraciones>5mm contra esta misma pieza y ninguna otra caja del subconjunto.

Fuente: `viewer_cart.js:35`. Ejemplo brick1607 t8349.7997603s, centroX10.6675. `/tmp/audit36-static-results.json`. Si es una marca, debe ser decal/pintura; si es tope, necesita actuador e interbloqueo.

### P2 — Fichas técnicas asignadas al componente incorrecto

- El pozo de pallet560bricks muestra el FestoDFM32/160mm/482N, aunque necesita bajar14×69=966mm y sostener el pallet. Esa referencia es un pequeño cilindro guiado y no sustenta la función/carrera representada. Hay un aviso de custom, pero la ficha y foto principal siguen siendo las del componente inapropiado. `/tmp/audit36-well-reference.png`.
- El carrito de descarte muestra HORSTMANNVWG de transferencia sobre raíles, con supuesto8.000kg, en lugar del `discard_bin` BAUER disponible en el catálogo.

Fuentes: `viewer_cart.js:62,102,149`; `catalog.json` entradasguided_lift/inventory_cart/discard_bin. Corregir los mappings y separar referencias de principio de una selección nominal válida.

### P2/P3 — La superficie de las ruedas está13mm dentro de los bricks

`tiles()` coloca los centros de ruedas enB−.006 con radio.019, por lo que llegan aB+.013; el fondo de los bricks está enB. Existe penetración13mm en todos los tramos de ruedas, incluidos turner y routing. Fuente `viewer_cart.js:22`, `cart_model.js:4,78`. Alinear la superficie efectiva de contacto con el datumB.

## Observaciones y límites, no defectos de la fórmula de caudal

- Las filas de pallets vacíos del dispensador son geometría estática de8unidades. `prepare()` genera otro pallet en su salida; no se anima ni descuenta el pallet físico de esa pila. Es una abstracción visual/inventario de embalaje; no representa un dispensador continuo validado.
- `Start queued cart` carga correctamente el siguiente modelo pero deja el replay en0 con el botónPlay, porque restaura el estado pausado del lote ya terminado. Renombrar aLoad o iniciar la reproducción.
- El texto `bricks in completed layers` es el acumulado formado, no inventario actual en buffers. Puede volver a causar confusión de miles de bricks supuestamente almacenados; sugerencia: `Bricks formed so far`.
- Cada cambio de carro es una nueva ejecución/modelo. No transfiere a la ejecución siguiente los1812aprobados remanentes del mixed o los5172del single. Esto es coherente como comparación de runs, pero no valida una planta continua entre carros ni el cierre de esos pallets parciales.

## Comprobaciones que sí pasaron

- Ambos replays:28.800bricks inspeccionados,25.332aprobados,3.468rechazados; sin duplicación de material en output ni bricks malos enviados a pallets.
- Mixed:42pallets/23.520bricks completos,1.560aprobados en otros layers y252antes de terminar un layer. Total remanente1812.
- Single:36pallets/20.160bricks completos,4.800aprobados en otros layers y372antes de terminar un layer. Todos los pallets completos son de un tono. Total remanente5172.
- Fórmulas correctas:42×560×3600/8901.632217=9511.963bricks/h y42×3600/8901.632217=16.98565pallets/h; single7571.4906bricks/h y13.52052pallets/h. Son ratios modelados del lote, no caudal industrial medido.
- No hay tracks de reserva temporal de colores. Los tres formadores están activos;627layers mixtos,209porformer.
- Orden de llegada lejano→cercano confirmado y movimientos conjuntos de40bricks en cada zona de buffer.
- Interbloqueo de pusher3466ciclos con siguiente brick presente; el siguiente acceso se retiene hasta retorno/home. El carrito coincide con la descarga y los ladrillos descartados caben dentro de su volumen representado durante este lote.
- Cámaras de captura/confirmación antes del rechazo conlatencia configurada; decisión sintética consistente y UI de cámara basada en el mismo replay3D.
- Queue next cart no modifica el modelo actual, bloquea un cambio prematuro y permite cargar el replay single al fin. Cutaways y vistas responden sin excepciones deJavaScript.
- Inspección visual de transferencia36heads, extracción lateral, V1/V2, inspección, rechazo, routing cubierto/abierto, formación, pickup/centrado, outfeed ycarros.

## Cobertura y lo no verificado

No se ejecutó física de contacto/fricción, cálculo estructural, dimensionamiento térmico/cargas/potencia, visión real ni PLC/SDK real; el usuario no pidió que laPoC los resolviera. El barrido estático fue de puntos medios y cajas alineadas; no certifica ausencia de colisión continua contra cilindros, vigas rotadas, herramientas móviles ni entre todos los cuerpos. Los fallos anteriores tienen evidencia geométrica/control concreta independiente de esa limitación. No se buscó información web ni se alteróUnreal. No se auditó37.
