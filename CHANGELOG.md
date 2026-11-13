# Changelog

## Unreleased (últimas actualizaciones)

### Añadido
- **Paneles redimensionables con el ratón** (ancho de columnas izquierda/XML/inspector y altura de Librería/Jerarquía/Doctor Gazebo) con memoria en `localStorage` y ajuste 3D automático (`ResizeObserver`).
- **Barra superior adaptable (`flex-wrap`)**: todos los botones superiores permanecen visibles sin cortarse en ventanas estrechas.
- **Trazador de muros 3D mejorado**: snap magnético automático a esquinas y extremos de muros existentes (`< 0.45 m`, anillo verde) y deshacer última esquina con **`Backspace`**.
- **Controles de Fase 4 en UI**: selector de motor de física (`dart`/`bullet`/`ode`), casillas completas de plugins (`Physics`, `UserCommands`, `SceneBroadcaster`), presets en *Spawn Robot* y reordenación `⬆ Subir` / `⬇ Bajar` de *Waypoints* con `Undo/Redo` (`SwapNodesOrderCommand`).
- **Multiselección fiable**: caja de selección (`🔲 Multi / Caja` y `Shift + Arrastrar`), `Shift/Ctrl + Clic` en el visor y casillas + rango `Shift` en el Outliner.
- **Agrupar / Desagrupar** objetos (`Ctrl+G` / `Ctrl+U`).
- **Imán `⬆️ Encima (B)`**: coloca un objeto sobre la superficie superior del objeto 3D más cercano (ignora carreteras planas).
- **Carreteras y viales**: tramo, curva, cruce, paso de cebra, rotonda y badén.
- **Botones Abrir/Cerrar** con estado en cada sección de la Librería de Elementos.

### Corregido
- **Agrupar mallas CAD (`.stl`/`.obj`)**: `GroupNodesCommand` y `SDFCompiler` conservan ahora la malla 3D (`shape: "mesh"`, URI, escala y orientación) en lugar de convertirlas en cubos.
- **Regla global `.hidden` en CSS**: `#nav2-map-preview-box` y los paneles ocultos se ocultan correctamente.
- `Espacio`/`⬇ Apoyar` ya no manda los objetos siempre al suelo (usa el soporte real debajo).
- **Vectorizador `.PGM`**: los contornos cuadrados ya no se convierten en círculo; se generan muros rectos a 90° y solo se suavizan los tramos curvos reales. Pilares redondos → cilindros.
- Caché del navegador: el servidor envía `Cache-Control: no-store`.
- Contorno de selección 3D ceñido a la geometría (`EdgesGeometry`); se retiró el `Box3Helper` sobredimensionado.

### Conocido
- En algunos objetos rotados el contorno de selección puede verse algo más grande que el objeto (solo visual).

