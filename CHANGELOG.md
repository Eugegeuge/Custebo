# Changelog

## Unreleased (últimas actualizaciones)

### Añadido
- **Multiselección fiable**: caja de selección (`🔲 Multi / Caja` y `Shift + Arrastrar`), `Shift/Ctrl + Clic` en el visor y casillas + rango `Shift` en el Outliner.
- **Agrupar / Desagrupar** objetos (`Ctrl+G` / `Ctrl+U`).
- **Imán `⬆️ Encima (B)`**: coloca un objeto sobre la superficie superior del objeto 3D más cercano (ignora carreteras planas).
- **Carreteras y viales**: tramo, curva, cruce, paso de cebra, rotonda y badén.
- **Botones Abrir/Cerrar** con estado en cada sección de la Librería de Elementos.

### Corregido
- `Espacio`/`⬇ Apoyar` ya no manda los objetos siempre al suelo (usa el soporte real debajo).
- **Vectorizador `.PGM`**: los contornos cuadrados ya no se convierten en círculo; se generan muros rectos a 90° y solo se suavizan los tramos curvos reales. Pilares redondos → cilindros.
- Caché del navegador: el servidor envía `Cache-Control: no-store`.
- Contorno de selección 3D ceñido a la geometría (`EdgesGeometry`); se retiró el `Box3Helper` sobredimensionado.

### Conocido
- En algunos objetos rotados el contorno de selección puede verse algo más grande que el objeto (solo visual).
