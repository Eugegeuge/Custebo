# 🌐 Custebo (Custom + Gazebo)

**Editor Visual de Mundos 3D, Generador de Mapas 2D y Empaquetador para ROS 2 y Gazebo (Harmonic & Classic)**

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble_%7C_Jazzy_%7C_Kilted-22314E?logo=ros)](https://docs.ros.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic_%7C_Classic_11-FF6F00)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python)](https://www.python.org/)

**Custebo** es un entorno de diseño 3D ligero, ergonómico (inspirado en Blender, RViz2 y herramientas CAD modernas) y determinista creado para solucionar la fricción de diseñar mundos de simulación en **Gazebo Harmonic / Ionic (SDF 1.9–1.11)** y **Gazebo Classic 11 (SDF 1.6–1.7)**.

---

## ✨ Características Principales

### 1. Ergonomía 3D Nativamente Robótica (`Z-Up` ENU)
- **Sistema de Coordenadas Estándar ROS (REP-103):** $+X$ delante (Rojo), $+Y$ izquierda (Verde), $+Z$ arriba (Azul), estrictamente en **metros** y **radianes**.
- **Gizmos Estilo Blender:** Traslación (`G`), Rotación (`R`), Escalado (`S`), alternancia de ejes `World / Local` (`X`), *Grid Snapping*, *Angle Snapping* y vista ortográfica cenital 2D (`Numpad 7`).
- **Smart Drop-to-Ground (`Espacio` / `End`) y Snap Cara-a-Cara (`V`):** Apila objetos automáticamente sobre superficies inferiores o acopla tramos de paredes/vallas sin que atraviesen el suelo ni salgan disparados al iniciar la física.
- **Historial `Undo / Redo` Infinito (`Ctrl+Z` / `Ctrl+Y`):** Arquitectura basada en el patrón *Command* sobre el Grafo de Escena JSON puro.

### 2. Bloques Constructivos Paramétricos y Objetos Compuestos
- **Arquitectura:** Paredes paramétricas (`🧱 Pared`), paredes con marco de puerta (`🚪 Pared Puerta`), paredes con ventana (`🪟 Pared Ventana`), esquinas en L (`📐 Esquina L`), pilares (`🏛 Columna`), escaleras paramétricas (`🪜 Escalera`) y rampas (`📐 Rampa`).
- **Mobiliario e Industria:** Estanterías de almacén (`🗄 Estantería`), mesas (`🪑 Mesa`), palés europeos estándar $1.2 \times 0.8\text{ m}$ (`🪵 Palé Euro`), vallas de seguridad (`🚧 Valla`), barreras Jersey (`🚧 Jersey`), conos (`🔺 Cono`), bolardos (`🛑 Bolardo`) y cajas (`📦 Caja Cartón`).
- **Desglose en Piezas Sueltas:** Cualquier objeto compuesto puede editarse paramétricamente o desglosarse en primitivas independientes con un clic.

### 3. Sección de "📂 Objetos Personalizados" e Importador CAD Universal
- Importación directa de mallas **`.stl`, `.obj`, `.dae`, `.glb`** con conversión automática de unidades (`mm -> m`, `cm -> m`) y orientación (`Y-Up -> Z-Up`).
- Guardado de cualquier objeto configurado como prefab reutilizable (`⭐ Guardar en Objetos Personalizados`).
- **Optimizador de `<collision>`:** 6 estrategias por objeto (Caja AABB, Cilindro, Esfera, *Convex Hull*, *Mesh Decimation* al 10% o Malla Completa).
- **Conector Gazebo Fuel y Workspace ROS 2:** Inserción de modelos `<include><uri>` desde Gazebo Fuel y escaneo de `AMENT_PREFIX_PATH`.
- **Importador Round-Trip `.sdf` / `.world`:** Limpia automáticamente los bloques `<state>` residuales de Gazebo y reconstruye el Grafo de Escena editable.

### 4. Auto-Física, Tensores de Inercia y "Doctor Gazebo"
- Cálculo automático de masa y matriz de inercia $3 \times 3$ definida positiva (`ixx, iyy, izz, ixy, ixz, iyz`) a partir del volumen real y la densidad del material (Madera, Aluminio, Acero, Hormigón, Plástico, Goma, Hielo) más el **Teorema de Steiner** para objetos compuestos.
- **Doctor Gazebo en Tiempo Real:** Linter integrado que detecta masas nulas, desigualdades triangulares de inercia violadas o interpenetraciones antes de abrir Gazebo.
- **Vista Dividida XML/SDF en Vivo:** Previsualización determinista del código SDF con resaltado bidireccional y validación oficial mediante `gz sdf -k`.

### 5. Herramientas Procedurales y CAD (Fase 3)
- **Clonadores Procedurales:** Matriz 3D ($N_x \times N_y \times N_z$), clonador en curva Bézier (*Spline Cloner*) y Pincel de Dispersión (*Scatter Brush* con anti-solapamiento AABB).
- **Trazador de Planos 2D $\to$ Muros 3D e Importador `.PGM`:** Calco de planos de planta 2D y reconstrucción automática de paredes 3D a partir de mapas SLAM `.pgm` de ROS 2.
- **Generador de Terrenos Exteriores:** Generación procedural de terrenos (*Colinas*, *Cráter Lunar / Rover*, *Valle*, *Terreno Rugoso*).

---

## 🚀 Inicio Rápido

No requiere compilación ni dependencias pesadas externas (funciona directamente con **Python 3.10+**):

```bash
# 1. Lanzar el servidor de Custebo Studio
python3 server/app.py --host 127.0.0.1 --port 8080
```

Abre en tu navegador: **`http://127.0.0.1:8080`**

### Validación por Línea de Comandos (CLI)

```bash
# Ejecutar suite de tests matemáticos y físicos (Inercia, Steiner, Cuaterniones ENU, AABB)
python3 tools/test_physics_math.py

# Ejecutar validador SDF multi-versión + Doctor Gazebo + gz sdf -k
python3 tools/validate_sdf.py --self-test
```

---

## 📂 Estructura del Proyecto

```text
Custebo/
├── core/
│   ├── schema/                 # Grafo de Escena JSON y librería de arquetipos paramétricos
│   ├── commands/               # Patrón Command con Undo/Redo infinito
│   ├── math_physics/           # Tensores de inercia, Steiner, RPY <-> Quaternions, 3D AABB
│   ├── importers/              # Importador STL/OBJ/DAE/GLB, Optimizador Colisión, Fuel y SDF Parser
│   ├── procedural/             # Clonadores Grid/Spline, Scatter Brush, Floorplan 2D->3D y Terrenos
│   ├── compilers/              # Compilador determinista SDF 1.10 (Harmonic) y SDF 1.7 (Classic)
│   └── validators/             # Linter físico "Doctor Gazebo"
├── server/
│   └── app.py                  # Servidor HTTP/JSON de Custebo Studio
├── frontend/
│   ├── index.html              # Interfaz de estudio oscura (Outliner, Viewport 3D, Split XML, Inspector)
│   ├── css/studio.css          # Estilos del entorno CAD/Robótica
│   └── js/main.js              # Visor Three.js Z-Up, Gizmos, capas debug y herramientas interactivas
├── assets/                     # Mallas CAD y biblioteca de Objetos Personalizados
├── projects/                   # Proyectos nativos (.gproj.json)
├── exports/                    # Mundos (.sdf) exportados para Gazebo Harmonic y Classic
└── tools/                      # Validadores CLI y tests unitarios de física/matemáticas
```

---

## 👤 Autor

- **GitHub:** [@Eugegeuge](https://github.com/Eugegeuge)

