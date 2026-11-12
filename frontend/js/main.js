/**
 * Custebo 3D World Editor — Frontend Application (`frontend/js/main.js`).
 *
 * Layer 3 (Presentation & 3D Viewport):
 * - Strictly ENU Z-Up (`THREE.Object3D.DEFAULT_UP.set(0, 0, 1)`)
 * - Euler intrinsic ZYX (`new THREE.Euler(roll, pitch, yaw, 'ZYX')`) matching Gazebo <pose>
 * - Zero direct state mutation: all actions emit Commands to `/api/command` (Undo/Redo)
 */

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { TransformControls } from "three/addons/controls/TransformControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";

// Enforce ROS 2 / Gazebo ENU Z-Up globally before creating any Three.js objects
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const RAD2DEG = 180.0 / Math.PI;
const DEG2RAD = Math.PI / 180.0;

function rgbaToHex(rgba) {
  if (!Array.isArray(rgba) || rgba.length < 3) return "#3b82f6";
  const r = Math.min(255, Math.max(0, Math.round(rgba[0] * 255)));
  const g = Math.min(255, Math.max(0, Math.round(rgba[1] * 255)));
  const b = Math.min(255, Math.max(0, Math.round(rgba[2] * 255)));
  return "#" + [r, g, b].map((x) => x.toString(16).padStart(2, "0")).join("");
}

function hexToRgba(hex) {
  const clean = hex.replace("#", "");
  const num = parseInt(clean, 16);
  const r = ((num >> 16) & 255) / 255.0;
  const g = ((num >> 8) & 255) / 255.0;
  const b = (num & 255) / 255.0;
  return [Number(r.toFixed(4)), Number(g.toFixed(4)), Number(b.toFixed(4)), 1.0];
}

class CusteboStudioApp {
  constructor() {
    this.sceneGraph = null;
    this.historyState = { can_undo: false, can_redo: false };
    this.sdfPreview = "";
    this.sdfLineMap = {};
    this.doctorAlerts = [];
    this.customObjects = [];
    this.presets = { materials: {}, surfaces: {} };
    this.meshGeometryCache = new Map();
    this.stlLoader = new STLLoader();
    this.objLoader = new OBJLoader();

    this.selectedNodeId = null;
    this.selectedNodeIds = new Set();
    this.showAnglesInDegrees = true;
    this.cameraMode = "perspective"; // "perspective" | "ortho_top"
    this.debugLayers = {
      collision: false,
      inertia: false,
      tf: true,
    };

    // Map of node_id -> THREE.Group in the 3D scene
    this.nodeObjectMap = new Map();
    this.isDraggingGizmo = false;
    this.multiDragStart = null;
    this.boxSelectStart = null;
    this.isBoxSelecting = false;
    this.activeViewportTool = null;
    this.rulerPoints = [];
    this.floorplanPoints = [];
    this.splinePoints = [];
    this.cursorWorldPt = [0, 0, 0];

    this._initThreeViewport();
    this._bindToolbarAndShortcuts();
    this.fetchInitialState();
  }

  // ===========================================================================
  // 1. THREE.JS ENU Z-UP VIEWPORT INITIALIZATION
  // ===========================================================================
  _initThreeViewport() {
    this.mountEl = document.getElementById("canvas-mount");
    const width = this.mountEl.clientWidth || 800;
    const height = this.mountEl.clientHeight || 600;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0f1318);

    // Perspective Camera (Z-Up)
    this.perspCamera = new THREE.PerspectiveCamera(50, width / height, 0.05, 1000);
    this.perspCamera.position.set(6.5, -6.5, 4.8);
    this.perspCamera.up.set(0, 0, 1);

    // Orthographic Top-Down Camera (Numpad 7)
    const aspect = width / height;
    const frustumSize = 16;
    this.orthoCamera = new THREE.OrthographicCamera(
      (-frustumSize * aspect) / 2,
      (frustumSize * aspect) / 2,
      frustumSize / 2,
      -frustumSize / 2,
      0.1,
      500
    );
    this.orthoCamera.position.set(0, 0, 50);
    this.orthoCamera.up.set(0, 1, 0); // In top-down view, +X right or +Y up

    this.activeCamera = this.perspCamera;

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(width, height);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.mountEl.appendChild(this.renderer.domElement);

    // Ambient + Hemisphere lighting for clear CAD visibility
    const hemiLight = new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 0.65);
    hemiLight.position.set(0, 0, 30);
    this.scene.add(hemiLight);

    // XY Ground Grid (Z = 0)
    this._buildEnuGridAndAxes();

    // OrbitControls
    this.orbitControls = new OrbitControls(this.activeCamera, this.renderer.domElement);
    this.orbitControls.enableDamping = true;
    this.orbitControls.dampingFactor = 0.08;
    this.orbitControls.target.set(0, 0, 0.5);
    this.orbitControls.update();

    // TransformControls (Gizmos: G, R, S)
    this.transformControls = new TransformControls(this.activeCamera, this.renderer.domElement);
    this.transformControls.setMode("translate");
    this.transformControls.setSpace("world");
    this.transformControls.setTranslationSnap(0.1);
    this.transformControls.setRotationSnap(15 * DEG2RAD);
    this.scene.add(this.transformControls);

    this.transformControls.addEventListener("dragging-changed", (event) => {
      this.orbitControls.enabled = !event.value;
      this.isDraggingGizmo = event.value;
      if (event.value && this.selectedNodeId) {
        const primaryGrp = this.nodeObjectMap.get(this.selectedNodeId);
        if (primaryGrp) {
          const others = new Map();
          for (const nid of this.selectedNodeIds) {
            if (nid === this.selectedNodeId) continue;
            const g = this.nodeObjectMap.get(nid);
            if (g && !g.userData.locked) {
              others.set(nid, {
                startPos: g.position.clone(),
                startQuat: g.quaternion.clone(),
              });
            }
          }
          this.multiDragStart = {
            primaryStartPos: primaryGrp.position.clone(),
            primaryStartQuat: primaryGrp.quaternion.clone(),
            others,
          };
        }
      } else if (!event.value && this.selectedNodeId) {
        this._commitGizmoTransformToBackend();
        this.multiDragStart = null;
      }
    });

    this.transformControls.addEventListener("objectChange", () => {
      if (this.selectedNodeId) {
        this._syncLiveInspectorFrom3DObject();
      }
    });

    // Raycaster for clicking, marquee box selection & live previewing tools in the 3D scene
    this.raycaster = new THREE.Raycaster();
    this.pointerDownPos = new THREE.Vector2();

    // Capture-phase pointerdown runs BEFORE OrbitControls so Shift+Click, Ctrl+Click, and Marquee Box Drag
    // are never hijacked by OrbitControls camera pan/rotate!
    this.renderer.domElement.addEventListener(
      "pointerdown",
      (e) => {
        this.pointerDownPos.set(e.clientX, e.clientY);
        if (
          e.button === 0 &&
          !this.isDraggingGizmo &&
          (this.activeViewportTool === "boxselect" || e.shiftKey || e.ctrlKey || e.metaKey)
        ) {
          this.orbitControls.enabled = false;
          this.boxSelectStart = {
            x: e.clientX,
            y: e.clientY,
            additive: Boolean(e.shiftKey || e.ctrlKey || e.metaKey),
          };
          this.isBoxSelecting = false;
        }
      },
      { capture: true }
    );

    window.addEventListener("pointermove", (e) => {
      if (this.boxSelectStart && !this.isDraggingGizmo) {
        const dist = Math.hypot(e.clientX - this.boxSelectStart.x, e.clientY - this.boxSelectStart.y);
        if (dist >= 6) {
          this.isBoxSelecting = true;
          this._updateMarqueeBoxOverlay(this.boxSelectStart.x, this.boxSelectStart.y, e.clientX, e.clientY);
          return;
        }
      }
      if (e.target === this.renderer.domElement) {
        this._handleViewportPointerMove(e);
      }
    });

    window.addEventListener("pointerup", (e) => {
      if (this.boxSelectStart) {
        const wasBox = this.isBoxSelecting;
        const startPt = this.boxSelectStart;
        this.boxSelectStart = null;
        this.isBoxSelecting = false;
        this.orbitControls.enabled = true;
        document.getElementById("selection-marquee-box")?.classList.add("hidden");
        if (wasBox) {
          this._finishMarqueeBoxSelection(
            startPt.x,
            startPt.y,
            e.clientX,
            e.clientY,
            Boolean(startPt.additive || e.shiftKey || e.ctrlKey || e.metaKey)
          );
          return;
        }
      } else if (!this.isDraggingGizmo) {
        this.orbitControls.enabled = true;
      }
      if (e.target !== this.renderer.domElement) return;
      if (e.button !== 0 || this.isDraggingGizmo) return;
      const dist = Math.hypot(e.clientX - this.pointerDownPos.x, e.clientY - this.pointerDownPos.y);
      if (dist < 6) {
        this._handleViewportClick(e);
      }
    });

    this.nodesRootGroup = new THREE.Group();
    this.scene.add(this.nodesRootGroup);

    // Interactive 3D Tool Overlay Group (Wall Tracer 3D preview, Laser Ruler, Spline, Scatter Brush)
    this.toolOverlayGroup = new THREE.Group();
    this.scene.add(this.toolOverlayGroup);

    window.addEventListener("resize", () => this._onResize());
    this._animate();
  }

  _buildEnuGridAndAxes() {
    // Sub-grid (0.5m) & Major grid (1m) on XY plane (Z=0)
    const gridMinor = new THREE.GridHelper(40, 80, 0x2a323d, 0x1a2028);
    gridMinor.rotation.x = Math.PI / 2;
    gridMinor.position.z = 0.001;
    this.scene.add(gridMinor);

    const gridMajor = new THREE.GridHelper(40, 40, 0x3b4654, 0x262f3a);
    gridMajor.rotation.x = Math.PI / 2;
    gridMajor.position.z = 0.002;
    this.scene.add(gridMajor);

    // Prominent ROS ENU Origin Axes (+X Red Forward, +Y Green Left, +Z Blue Up)
    const axesGroup = new THREE.Group();
    const axisLen = 2.5;
    const makeAxisLine = (endVec, hexColor) => {
      const geom = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(0, 0, 0.004),
        endVec,
      ]);
      const mat = new THREE.LineBasicMaterial({ color: hexColor, linewidth: 2 });
      return new THREE.Line(geom, mat);
    };
    axesGroup.add(makeAxisLine(new THREE.Vector3(axisLen, 0, 0.004), 0xef4444)); // +X Red
    axesGroup.add(makeAxisLine(new THREE.Vector3(0, axisLen, 0.004), 0x22c55e)); // +Y Green
    axesGroup.add(makeAxisLine(new THREE.Vector3(0, 0, axisLen), 0x3b82f6)); // +Z Blue
    this.scene.add(axesGroup);
  }

  _onResize() {
    if (!this.mountEl) return;
    const w = this.mountEl.clientWidth;
    const h = this.mountEl.clientHeight;
    if (w <= 0 || h <= 0) return;

    this.perspCamera.aspect = w / h;
    this.perspCamera.updateProjectionMatrix();

    const aspect = w / h;
    const frustumSize = 16;
    this.orthoCamera.left = (-frustumSize * aspect) / 2;
    this.orthoCamera.right = (frustumSize * aspect) / 2;
    this.orthoCamera.top = frustumSize / 2;
    this.orthoCamera.bottom = -frustumSize / 2;
    this.orthoCamera.updateProjectionMatrix();

    this.renderer.setSize(w, h);
  }

  _animate() {
    requestAnimationFrame(() => this._animate());
    this.orbitControls.update();
    this.renderer.render(this.scene, this.activeCamera);
  }

  // ===========================================================================
  // 2. BACKEND API SYNCHRONIZATION & COMMAND DISPATCH
  // ===========================================================================
  async fetchInitialState() {
    try {
      const res = await fetch("/api/state");
      const data = await res.json();
      if (data.ok) {
        this._applyBackendPayload(data);
        if (!this.selectedNodeId) {
          const starter = this.sceneGraph.nodes.find((n) => n.type === "primitive");
          if (starter) this.selectNode(starter.id);
        }
      }
    } catch (err) {
      this._setStatus(`Error conectando al servidor: ${err.message}`);
    }
  }

  async sendCommand(cmdBody) {
    try {
      const res = await fetch("/api/command", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(cmdBody),
      });
      const data = await res.json();
      if (data.ok) {
        if (data.selected_node_id) {
          this.selectedNodeId = data.selected_node_id;
          this.selectedNodeIds = new Set([data.selected_node_id]);
        }
        this._applyBackendPayload(data);
      } else {
        this._setStatus(`Error: ${data.error}`);
      }
    } catch (err) {
      this._setStatus(`Error de red: ${err.message}`);
    }
  }

  async triggerUndo() {
    const res = await fetch("/api/undo", { method: "POST" });
    const data = await res.json();
    if (data.ok) {
      this._applyBackendPayload(data);
      this._setStatus("Deshacer aplicado (Ctrl+Z)");
    }
  }

  async triggerRedo() {
    const res = await fetch("/api/redo", { method: "POST" });
    const data = await res.json();
    if (data.ok) {
      this._applyBackendPayload(data);
      this._setStatus("Rehacer aplicado (Ctrl+Y)");
    }
  }

  _applyBackendPayload(data) {
    this.sceneGraph = data.scene_graph;
    this.historyState = data.history || { can_undo: false, can_redo: false };
    this.sdfPreview = data.sdf_preview || "";
    this.sdfLineMap = data.sdf_line_map || {};
    this.doctorAlerts = data.doctor_alerts || [];
    if (Array.isArray(data.custom_objects)) {
      this.customObjects = data.custom_objects;
    }
    if (data.presets) {
      this.presets = data.presets;
    }

    // Verify selectedNodeId and selectedNodeIds still exist
    const validIds = new Set(this.sceneGraph.nodes.map((n) => n.id));
    for (const sid of Array.from(this.selectedNodeIds)) {
      if (!validIds.has(sid)) this.selectedNodeIds.delete(sid);
    }
    if (this.selectedNodeId && !validIds.has(this.selectedNodeId)) {
      const rem = Array.from(this.selectedNodeIds);
      this.selectedNodeId = rem.length > 0 ? rem[rem.length - 1] : null;
    }
    if (this.selectedNodeId && !this.selectedNodeIds.has(this.selectedNodeId)) {
      this.selectedNodeIds.add(this.selectedNodeId);
    }

    // Update topbar controls
    document.getElementById("world-name-input").value = this.sceneGraph.world.name || "custebo_world";
    document.getElementById("sdf-target-select").value =
      this.sceneGraph.world.sdf_target || "harmonic";

    const btnUndo = document.getElementById("btn-undo");
    const btnRedo = document.getElementById("btn-redo");
    btnUndo.disabled = !this.historyState.can_undo;
    btnRedo.disabled = !this.historyState.can_redo;
    btnUndo.title = this.historyState.last_action
      ? `Deshacer: ${this.historyState.last_action} (Ctrl+Z)`
      : "Deshacer (Ctrl+Z)";

    this._rebuild3DSceneFromGraph();
    this._renderCustomObjectsPalette();
    this._renderOutliner();
    this._renderDoctorGazebo();
    this._renderLiveSdfXml();
    this._renderInspector();
  }

  _renderCustomObjectsPalette() {
    const grid = document.getElementById("custom-objects-grid");
    if (!grid) return;
    grid.innerHTML = "";

    if (!this.customObjects || this.customObjects.length === 0) {
      grid.innerHTML = `<div style="color:var(--text-muted);font-size:11px;padding:4px;">Sin objetos personalizados aún. Pulsa "➕ Importar 3D" o guarda un objeto desde el Inspector.</div>`;
      return;
    }

    for (const item of this.customObjects) {
      const row = document.createElement("div");
      row.className = "custom-obj-row";
      row.title = "Haz clic para insertar este objeto personalizado en el mundo 3D";

      const titleSpan = document.createElement("span");
      titleSpan.className = "custom-obj-title";
      titleSpan.textContent = item.label || item.filename || "Objeto Personalizado";
      titleSpan.addEventListener("click", () => {
        this.sendCommand({
          type: "add_custom_object",
          custom_id: item.id,
        });
        this._setStatus(`Insertado '${item.label}' en la escena`);
      });

      const delBtn = document.createElement("button");
      delBtn.className = "custom-obj-del";
      delBtn.textContent = "×";
      delBtn.title = "Eliminar de Objetos Personalizados";
      delBtn.addEventListener("click", async (e) => {
        e.stopPropagation();
        const res = await fetch("/api/delete_custom_object", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ custom_id: item.id }),
        });
        const data = await res.json();
        if (data.ok) {
          this._applyBackendPayload(data);
        }
      });

      row.appendChild(titleSpan);
      row.appendChild(delBtn);
      grid.appendChild(row);
    }
  }

  // ===========================================================================
  // 3. 3D SCENE RECONSTRUCTION FROM PURE SCENE GRAPH JSON
  // ===========================================================================
  _rebuild3DSceneFromGraph() {
    this.transformControls.detach();

    while (this.nodesRootGroup.children.length > 0) {
      const child = this.nodesRootGroup.children[0];
      this.nodesRootGroup.remove(child);
    }
    this.nodeObjectMap.clear();

    const folderMap = new Map();
    for (const f of this.sceneGraph.folders || []) {
      folderMap.set(f.id, f);
    }

    for (const node of this.sceneGraph.nodes) {
      const folder = folderMap.get(node.folder_id);
      const isVisible = node.visible !== false && (!folder || folder.visible !== false);

      const group = this._createThreeGroupForNode(node);
      group.visible = isVisible;
      group.userData.nodeId = node.id;
      group.userData.locked = Boolean(node.locked || (folder && folder.locked));

      this.nodesRootGroup.add(group);
      this.nodeObjectMap.set(node.id, group);
    }

    // Draw connected topological route line between visible Nav2 waypoints
    const wpNodes = this.sceneGraph.nodes.filter(
      (n) => n.type === "waypoint" && n.visible !== false
    );
    if (wpNodes.length >= 2) {
      const wpPoints = wpNodes.map(
        (w) => new THREE.Vector3(w.pose[0], w.pose[1], Math.max(0.06, w.pose[2]))
      );
      const routeGeom = new THREE.BufferGeometry().setFromPoints(wpPoints);
      const routeMat = new THREE.LineBasicMaterial({ color: 0x06b6d4, linewidth: 2 });
      this.nodesRootGroup.add(new THREE.Line(routeGeom, routeMat));
    }

    // Draw unmistakable 3D Selection Bounding Boxes for all selected objects (and a combined group box)
    const combinedBox = new THREE.Box3();
    let combinedCount = 0;
    for (const selId of this.selectedNodeIds) {
      const grp = this.nodeObjectMap.get(selId);
      if (!grp || !grp.visible) continue;
      grp.updateMatrixWorld(true);
      const b3 = new THREE.Box3().setFromObject(grp);
      if (!b3.isEmpty()) {
        b3.expandByScalar(0.02);
        const isPrimary = selId === this.selectedNodeId;
        const helperColor =
          this.selectedNodeIds.size > 1 ? (isPrimary ? 0x38bdf8 : 0xc084fc) : 0x38bdf8;
        this.nodesRootGroup.add(new THREE.Box3Helper(b3, helperColor));
        combinedBox.union(b3);
        combinedCount++;
      }
    }
    if (combinedCount >= 2 && !combinedBox.isEmpty()) {
      const unionBox = combinedBox.clone().expandByScalar(0.06);
      this.nodesRootGroup.add(new THREE.Box3Helper(unionBox, 0xfacc15));
    }

    if (this.selectedNodeId) {
      const selectedGroup = this.nodeObjectMap.get(this.selectedNodeId);
      const nodeData = this._getSelectedNode();
      if (selectedGroup && nodeData && !selectedGroup.userData.locked && selectedGroup.visible) {
        this.transformControls.attach(selectedGroup);
      }
    }
  }

  _createThreeGroupForNode(node) {
    const group = new THREE.Group();
    const pose = node.pose || [0, 0, 0, 0, 0, 0];
    const scale = node.scale || [1, 1, 1];

    group.position.set(pose[0], pose[1], pose[2]);
    // Intrinsic ZYX Euler matches Gazebo <pose> R P Y in ENU Z-Up
    group.rotation.set(pose[3], pose[4], pose[5], "ZYX");
    group.scale.set(scale[0], scale[1], scale[2]);

    if (node.type === "ground_plane") {
      const size = node.geometry?.size || [100, 100];
      const color = node.material?.color || [0.42, 0.44, 0.47, 1.0];
      const geom = new THREE.PlaneGeometry(size[0], size[1]);
      const mat = new THREE.MeshStandardMaterial({
        color: new THREE.Color(color[0], color[1], color[2]),
        roughness: 0.9,
        metalness: 0.05,
        polygonOffset: true,
        polygonOffsetFactor: 2,
        polygonOffsetUnits: 2,
      });
      const mesh = new THREE.Mesh(geom, mat);
      mesh.receiveShadow = true;
      mesh.userData.nodeId = node.id;
      group.add(mesh);
      return group;
    }

    if (node.type === "primitive" || node.type === "include" || node.type === "mesh") {
      const isSelected = node.id === this.selectedNodeId || this.selectedNodeIds.has(node.id);
      const parts = Array.isArray(node.parts) && node.parts.length > 0 ? node.parts : null;

      if (node.type === "mesh") {
        const meshInfo = node.mesh || {};
        const fname = meshInfo.filename || "agv_chassis.stl";
        const unitScale = Number(meshInfo.unit_scale ?? 1.0);
        const upAxis = meshInfo.up_axis || "Z_UP";
        const color = node.material?.color || [0.22, 0.58, 0.92, 1.0];
        const bsize = node.geometry?.size || [1, 1, 1];

        const mat = new THREE.MeshStandardMaterial({
          color: new THREE.Color(color[0], color[1], color[2]),
          roughness: 0.4,
          metalness: 0.3,
          transparent: this.debugLayers.inertia,
          opacity: this.debugLayers.inertia ? 0.45 : 1.0,
        });

        // Temporary bounding placeholder until STL/OBJ finishes loading (or from cache)
        const attachLoadedGeom = (geom) => {
          const cloned = geom.clone();
          cloned.scale(unitScale, unitScale, unitScale);
          if (upAxis === "Y_UP_TO_Z_UP") {
            cloned.rotateX(Math.PI / 2);
          }
          cloned.computeVertexNormals();
          const m = new THREE.Mesh(cloned, mat);
          m.castShadow = true;
          m.receiveShadow = true;
          m.userData.nodeId = node.id;
          group.add(m);

          if (isSelected) {
            const edges = new THREE.EdgesGeometry(cloned);
            group.add(new THREE.LineSegments(edges, new THREE.LineBasicMaterial({ color: 0x60a5fa })));
          }
        };

        if (this.meshGeometryCache.has(fname)) {
          attachLoadedGeom(this.meshGeometryCache.get(fname));
        } else {
          // Placeholder box while loading
          const boxG = new THREE.BoxGeometry(bsize[0], bsize[1], bsize[2]);
          const placeholder = new THREE.Mesh(boxG, mat);
          placeholder.userData.nodeId = node.id;
          group.add(placeholder);

          const url = `/meshes/${encodeURIComponent(fname)}`;
          if (fname.toLowerCase().endsWith(".stl")) {
            this.stlLoader.load(url, (loadedGeom) => {
              this.meshGeometryCache.set(fname, loadedGeom);
              this._rebuild3DSceneFromGraph();
            });
          } else if (fname.toLowerCase().endsWith(".obj")) {
            this.objLoader.load(url, (objGroup) => {
              let foundGeom = null;
              objGroup.traverse((ch) => {
                if (ch.isMesh && !foundGeom) foundGeom = ch.geometry.clone();
              });
              if (foundGeom) {
                this.meshGeometryCache.set(fname, foundGeom);
                this._rebuild3DSceneFromGraph();
              }
            });
          }
        }

        // Collision proxy wireframe according to `node.collision.strategy`
        if (this.debugLayers.collision && node.collision?.enabled !== false) {
          const strat = node.collision?.strategy || "aabb";
          let colGeom;
          if (strat === "cylinder") {
            const r = Number(node.geometry?.radius ?? 0.5);
            const l = Number(node.geometry?.length ?? 1.0);
            colGeom = new THREE.CylinderGeometry(r, r, l, 24);
            colGeom.rotateX(Math.PI / 2);
          } else if (strat === "sphere") {
            const r = Number(node.geometry?.radius ?? 0.5);
            colGeom = new THREE.SphereGeometry(r, 20, 16);
          } else {
            colGeom = new THREE.BoxGeometry(bsize[0], bsize[1], bsize[2]);
          }
          const colMesh = new THREE.Mesh(
            colGeom,
            new THREE.MeshBasicMaterial({
              color: node.static ? 0x22c55e : 0xf59e0b,
              wireframe: true,
            })
          );
          colMesh.scale.set(1.02, 1.02, 1.02);
          group.add(colMesh);
        }

        if (this.debugLayers.tf) {
          group.add(new THREE.AxesHelper(0.75));
        }
        return group;
      }

      const buildPartBufferGeom = (shape, sz, radius, length) => {
        if (shape === "box") {
          return new THREE.BoxGeometry(sz[0], sz[1], sz[2]);
        }
        if (shape === "cylinder") {
          const g = new THREE.CylinderGeometry(radius, radius, length, 28);
          g.rotateX(Math.PI / 2); // Align cylinder with ENU +Z
          return g;
        }
        if (shape === "sphere") {
          return new THREE.SphereGeometry(radius, 28, 20);
        }
        return new THREE.PlaneGeometry(sz[0] || 2, sz[1] || 2);
      };

      if (parts) {
        // Render all sub-parts of the parametric composite / archetype / Fuel / ROS 2 model
        for (const p of parts) {
          const pShape = p.shape || "box";
          const pSize = p.size || [1, 1, 1];
          const pRad = Number(p.radius ?? 0.2);
          const pLen = Number(p.length ?? 1.0);
          const pColor = p.color || node.material?.color || [0.75, 0.78, 0.82, 1.0];
          const lp = p.local_pose || [0, 0, 0, 0, 0, 0];

          const partMat = new THREE.MeshStandardMaterial({
            color: new THREE.Color(pColor[0], pColor[1], pColor[2]),
            roughness: Number(node.material?.roughness ?? 0.55),
            metalness: Number(node.material?.metalness ?? 0.15),
            transparent: this.debugLayers.inertia,
            opacity: this.debugLayers.inertia ? 0.45 : 1.0,
          });

          if (pShape === "mesh" && p.mesh_filename) {
            const mf = p.mesh_filename;
            const mScale = p.mesh_scale || [1, 1, 1];
            if (this.meshGeometryCache.has(mf)) {
              const gClone = this.meshGeometryCache.get(mf).clone();
              gClone.scale(mScale[0], mScale[1], mScale[2]);
              gClone.computeVertexNormals();
              const pm = new THREE.Mesh(gClone, partMat);
              pm.position.set(lp[0], lp[1], lp[2]);
              pm.rotation.set(lp[3], lp[4], lp[5], "ZYX");
              pm.castShadow = true;
              pm.receiveShadow = true;
              pm.userData.nodeId = node.id;
              group.add(pm);
              continue;
            } else {
              const url = `/meshes/${encodeURIComponent(mf)}`;
              if (mf.toLowerCase().endsWith(".stl")) {
                this.stlLoader.load(url, (loadedGeom) => {
                  this.meshGeometryCache.set(mf, loadedGeom);
                  this._rebuild3DSceneFromGraph();
                });
              } else if (mf.toLowerCase().endsWith(".obj")) {
                this.objLoader.load(url, (objGroup) => {
                  let foundGeom = null;
                  objGroup.traverse((ch) => {
                    if (ch.isMesh && !foundGeom) foundGeom = ch.geometry.clone();
                  });
                  if (foundGeom) {
                    this.meshGeometryCache.set(mf, foundGeom);
                    this._rebuild3DSceneFromGraph();
                  }
                });
              }
            }
          }

          const partGeom = buildPartBufferGeom(pShape === "mesh" ? "box" : pShape, pSize, pRad, pLen);

          const partMesh = new THREE.Mesh(partGeom, partMat);
          partMesh.position.set(lp[0], lp[1], lp[2]);
          partMesh.rotation.set(lp[3], lp[4], lp[5], "ZYX");
          partMesh.castShadow = true;
          partMesh.receiveShadow = true;
          partMesh.userData.nodeId = node.id;
          group.add(partMesh);

          if (isSelected) {
            const edges = new THREE.EdgesGeometry(partGeom);
            const lineSeg = new THREE.LineSegments(
              edges,
              new THREE.LineBasicMaterial({ color: 0x60a5fa })
            );
            lineSeg.position.copy(partMesh.position);
            lineSeg.rotation.copy(partMesh.rotation);
            group.add(lineSeg);
          }

          if (this.debugLayers.collision && node.collision?.enabled !== false) {
            const colMesh = new THREE.Mesh(
              partGeom.clone(),
              new THREE.MeshBasicMaterial({
                color: node.static ? 0x22c55e : 0xf59e0b,
                wireframe: true,
              })
            );
            colMesh.position.copy(partMesh.position);
            colMesh.rotation.copy(partMesh.rotation);
            colMesh.scale.set(1.01, 1.01, 1.01);
            group.add(colMesh);
          }
        }
      } else {
        // Standard single primitive
        const geomData = node.geometry || {};
        const shape = geomData.shape || "box";
        const bufferGeom = buildPartBufferGeom(
          shape,
          geomData.size || [1, 1, 1],
          Number(geomData.radius ?? 0.5),
          Number(geomData.length ?? 1.0)
        );

        const color = node.material?.color || [0.25, 0.52, 0.85, 1.0];
        const mat = new THREE.MeshStandardMaterial({
          color: new THREE.Color(color[0], color[1], color[2]),
          roughness: Number(node.material?.roughness ?? 0.45),
          metalness: Number(node.material?.metalness ?? 0.1),
          transparent: this.debugLayers.inertia,
          opacity: this.debugLayers.inertia ? 0.45 : 1.0,
        });

        const mesh = new THREE.Mesh(bufferGeom, mat);
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        mesh.userData.nodeId = node.id;
        group.add(mesh);

        if (isSelected) {
          const edges = new THREE.EdgesGeometry(bufferGeom);
          const lineMat = new THREE.LineBasicMaterial({ color: 0x60a5fa });
          group.add(new THREE.LineSegments(edges, lineMat));
        }

        if (this.debugLayers.collision && node.collision?.enabled !== false) {
          const colGeom = bufferGeom.clone();
          const colMat = new THREE.MeshBasicMaterial({
            color: node.static ? 0x22c55e : 0xf59e0b,
            wireframe: true,
          });
          const colMesh = new THREE.Mesh(colGeom, colMat);
          colMesh.scale.set(1.01, 1.01, 1.01);
          group.add(colMesh);
        }
      }

      // Debug Layer: Center of Mass & Magenta Inertia Ellipsoid (Gazebo style)
      if (this.debugLayers.inertia && !node.static && node.inertial) {
        const com = node.inertial.center_of_mass || [0, 0, 0];
        const mass = Math.max(Number(node.inertial.mass || 1.0), 0.001);
        const ixx = Math.max(Number(node.inertial.ixx || 0.1), 1e-4);
        const iyy = Math.max(Number(node.inertial.iyy || 0.1), 1e-4);
        const izz = Math.max(Number(node.inertial.izz || 0.1), 1e-4);

        // Equivalent uniform ellipsoid semi-axes from principal moments
        const rx = Math.sqrt(Math.max(2.5 * (iyy + izz - ixx) / mass, 0.02));
        const ry = Math.sqrt(Math.max(2.5 * (ixx + izz - iyy) / mass, 0.02));
        const rz = Math.sqrt(Math.max(2.5 * (ixx + iyy - izz) / mass, 0.02));

        const ellipGeom = new THREE.SphereGeometry(1, 16, 12);
        const ellipMat = new THREE.MeshBasicMaterial({
          color: 0xd946ef,
          wireframe: true,
        });
        const ellipMesh = new THREE.Mesh(ellipGeom, ellipMat);
        ellipMesh.position.set(com[0], com[1], com[2]);
        ellipMesh.scale.set(rx, ry, rz);
        group.add(ellipMesh);
      }

      // Debug Layer: Local TF Axes
      if (this.debugLayers.tf) {
        const axes = new THREE.AxesHelper(0.75);
        group.add(axes);
      }

      return group;
    }

    if (node.type === "light") {
      const ldata = node.light || {};
      const ltype = ldata.light_type || "point";
      const diff = ldata.diffuse || [0.95, 0.95, 0.9, 1.0];
      const threeColor = new THREE.Color(diff[0], diff[1], diff[2]);
      const intensity = Number(ldata.intensity ?? 1.0);

      let lightObj;
      if (ltype === "directional") {
        lightObj = new THREE.DirectionalLight(threeColor, intensity * 1.4);
        lightObj.castShadow = Boolean(ldata.cast_shadows);
      } else if (ltype === "spot") {
        lightObj = new THREE.SpotLight(
          threeColor,
          intensity * 15,
          Number(ldata.range ?? 20),
          Number(ldata.spot?.outer_angle ?? 0.9)
        );
        lightObj.castShadow = Boolean(ldata.cast_shadows);
      } else {
        lightObj = new THREE.PointLight(threeColor, intensity * 10, Number(ldata.range ?? 20));
        lightObj.castShadow = Boolean(ldata.cast_shadows);
      }
      group.add(lightObj);

      // Visual bulb gizmo so the user can click and move the light in 3D
      const bulbGeom = new THREE.OctahedronGeometry(0.35, 1);
      const bulbMat = new THREE.MeshBasicMaterial({
        color: node.id === this.selectedNodeId ? 0x60a5fa : 0xfacc15,
        wireframe: true,
      });
      const bulbMesh = new THREE.Mesh(bulbGeom, bulbMat);
      bulbMesh.userData.nodeId = node.id;
      group.add(bulbMesh);

      if (ltype === "directional" || ltype === "spot") {
        const dir = ldata.direction || [-0.5, 0.1, -0.9];
        const rayGeom = new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(0, 0, 0),
          new THREE.Vector3(dir[0] * 2.5, dir[1] * 2.5, dir[2] * 2.5),
        ]);
        group.add(new THREE.Line(rayGeom, new THREE.LineBasicMaterial({ color: 0xfde047 })));
      }

      if (this.debugLayers.tf) {
        group.add(new THREE.AxesHelper(0.5));
      }
      return group;
    }

    // Phase 4.2: Nav2 Waypoint Marker (Cyan disc + heading arrow)
    if (node.type === "waypoint") {
      const isSelected = node.id === this.selectedNodeId;
      const mat = new THREE.MeshStandardMaterial({
        color: isSelected ? 0x60a5fa : 0x06b6d4,
        roughness: 0.3,
        metalness: 0.2,
      });
      const discGeom = new THREE.CylinderGeometry(0.28, 0.28, 0.04, 24);
      discGeom.rotateX(Math.PI / 2);
      const disc = new THREE.Mesh(discGeom, mat);
      disc.position.z = 0.02;
      disc.userData.nodeId = node.id;
      group.add(disc);

      // Heading cone pointing along +X
      const coneGeom = new THREE.ConeGeometry(0.12, 0.36, 16);
      coneGeom.rotateZ(-Math.PI / 2);
      const cone = new THREE.Mesh(coneGeom, mat);
      cone.position.set(0.28, 0, 0.08);
      cone.userData.nodeId = node.id;
      group.add(cone);
      return group;
    }

    // Phase 4.3: Robot Spawn Marker (Mobile robot chassis + wheels + forward arrow)
    if (node.type === "robot_spawn") {
      const isSelected = node.id === this.selectedNodeId;
      const bodyMat = new THREE.MeshStandardMaterial({
        color: isSelected ? 0x60a5fa : 0x10b981,
        roughness: 0.35,
        metalness: 0.3,
      });
      const baseMesh = new THREE.Mesh(new THREE.BoxGeometry(0.46, 0.38, 0.18), bodyMat);
      baseMesh.position.z = 0.14;
      baseMesh.userData.nodeId = node.id;
      group.add(baseMesh);

      const lidarTurret = new THREE.Mesh(
        new THREE.CylinderGeometry(0.07, 0.07, 0.08, 16).rotateX(Math.PI / 2),
        new THREE.MeshStandardMaterial({ color: 0x1e293b })
      );
      lidarTurret.position.set(0.05, 0, 0.27);
      lidarTurret.userData.nodeId = node.id;
      group.add(lidarTurret);

      const arrowCone = new THREE.Mesh(
        new THREE.ConeGeometry(0.11, 0.32, 16).rotateZ(-Math.PI / 2),
        new THREE.MeshBasicMaterial({ color: 0xfacc15 })
      );
      arrowCone.position.set(0.38, 0, 0.14);
      arrowCone.userData.nodeId = node.id;
      group.add(arrowCone);
      return group;
    }

    // Phase 4.3: Static Environmental Sensor (RGB-D Camera or 2D/3D LiDAR with 3D Frustum)
    if (node.type === "sensor") {
      const sdata = node.sensor || {};
      const stype = sdata.sensor_type || "camera_rgbd";
      const fovDeg = Number(sdata.fov_deg ?? 75.0);
      const rMax = Math.min(Number(sdata.range_max ?? 8.0), 12.0);
      const isSelected = node.id === this.selectedNodeId;

      const bodyMat = new THREE.MeshStandardMaterial({
        color: isSelected ? 0x60a5fa : 0xa855f7,
        roughness: 0.3,
        metalness: 0.4,
      });
      const housing = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.18, 0.14), bodyMat);
      housing.userData.nodeId = node.id;
      group.add(housing);

      if (sdata.show_frustum !== false) {
        if (stype === "camera_rgbd") {
          const halfFov = (fovDeg * DEG2RAD) / 2;
          const coneRadius = Math.tan(halfFov) * rMax * 0.45;
          const coneLen = rMax * 0.45;
          const frustumGeom = new THREE.ConeGeometry(coneRadius, coneLen, 4, 1, true);
          frustumGeom.rotateY(Math.PI / 4);
          frustumGeom.rotateZ(Math.PI / 2);
          frustumGeom.translate(coneLen * 0.5, 0, 0);
          const frustumMat = new THREE.MeshBasicMaterial({
            color: 0xc084fc,
            wireframe: true,
            transparent: true,
            opacity: 0.65,
          });
          group.add(new THREE.Mesh(frustumGeom, frustumMat));
        } else {
          // 2D / 3D LiDAR 360° scan volume preview
          const scanHeight = stype === "lidar_3d" ? rMax * 0.35 : 0.04;
          const lidarGeom = new THREE.CylinderGeometry(rMax * 0.4, rMax * 0.4, scanHeight, 28, 1, true);
          lidarGeom.rotateX(Math.PI / 2);
          const lidarMat = new THREE.MeshBasicMaterial({
            color: 0x38bdf8,
            wireframe: true,
            transparent: true,
            opacity: 0.5,
          });
          group.add(new THREE.Mesh(lidarGeom, lidarMat));
        }
      }
      return group;
    }

    // Phase 4.4: Dynamic Pedestrian / Obstacle Actor (<actor>) + Trajectory Path
    if (node.type === "actor") {
      const isSelected = node.id === this.selectedNodeId;
      const actorMat = new THREE.MeshStandardMaterial({
        color: isSelected ? 0x60a5fa : 0xf97316,
        roughness: 0.4,
      });
      // Humanoid torso + head representation
      const torso = new THREE.Mesh(
        new THREE.CylinderGeometry(0.22, 0.18, 1.3, 16).rotateX(Math.PI / 2),
        actorMat
      );
      torso.userData.nodeId = node.id;
      group.add(torso);

      const head = new THREE.Mesh(new THREE.SphereGeometry(0.18, 16, 12), actorMat);
      head.position.set(0, 0, 0.85);
      head.userData.nodeId = node.id;
      group.add(head);

      // Draw trajectory waypoints in world space (compensated by group inverse position)
      const wps = node.actor?.waypoints || [];
      if (wps.length >= 2) {
        const pts = wps.map(
          (pt) =>
            new THREE.Vector3(
              Number(pt[0]) - pose[0],
              Number(pt[1]) - pose[1],
              0.08 - pose[2]
            )
        );
        if (node.actor?.loop !== false) {
          pts.push(pts[0].clone());
        }
        const pathGeom = new THREE.BufferGeometry().setFromPoints(pts);
        const pathLine = new THREE.Line(
          pathGeom,
          new THREE.LineBasicMaterial({ color: 0xfb923c })
        );
        group.add(pathLine);
      }
      return group;
    }

    return group;
  }

  _raycastGroundOrSurface(event, allow3DSurface = false) {
    const rect = this.renderer.domElement.getBoundingClientRect();
    const mouse = new THREE.Vector2(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1
    );
    this.raycaster.setFromCamera(mouse, this.activeCamera);

    if (allow3DSurface) {
      const meshes = [];
      this.nodesRootGroup.traverse((obj) => {
        if (obj.isMesh && obj.userData.nodeId) {
          const n = this.sceneGraph?.nodes?.find((x) => x.id === obj.userData.nodeId);
          if (n && n.type !== "ground_plane" && obj.visible) {
            meshes.push(obj);
          }
        }
      });
      const hits = this.raycaster.intersectObjects(meshes, false);
      if (hits.length > 0) {
        const p = hits[0].point;
        return [Number(p.x.toFixed(3)), Number(p.y.toFixed(3)), Number(p.z.toFixed(3))];
      }
    }

    const groundPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
    const hitPt = new THREE.Vector3();
    if (this.raycaster.ray.intersectPlane(groundPlane, hitPt)) {
      return [Number(hitPt.x.toFixed(3)), Number(hitPt.y.toFixed(3)), 0.0];
    }
    return null;
  }

  _handleViewportPointerMove(event) {
    if (!this.activeViewportTool || this.isDraggingGizmo) return;
    const pt = this._raycastGroundOrSurface(event, this.activeViewportTool === "ruler");
    if (pt) {
      this.cursorWorldPt = pt;
      this._updateToolOverlay3D(pt);
    }
  }

  _updateToolOverlay3D(cursorPt = null) {
    if (!this.toolOverlayGroup) return;
    while (this.toolOverlayGroup.children.length > 0) {
      this.toolOverlayGroup.remove(this.toolOverlayGroup.children[0]);
    }

    if (!this.activeViewportTool) return;

    // -----------------------------------------------------------------------
    // 1. INTERACTIVE 2D->3D WALL TRACER OVERLAY & LIVE POPUP STATS
    // -----------------------------------------------------------------------
    if (this.activeViewportTool === "floorplan") {
      const h = Math.max(0.3, parseFloat(document.getElementById("fp-wall-height")?.value || "2.50"));
      const t = Math.max(0.05, parseFloat(document.getElementById("fp-wall-thick")?.value || "0.20"));
      const closed = Boolean(document.getElementById("fp-wall-closed")?.checked);
      const pts = this.floorplanPoints || [];

      // Draw glowing cyan corner posts at every clicked vertex + live cursor target
      const postMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8 });
      for (const p of pts) {
        const sp = new THREE.Mesh(new THREE.SphereGeometry(0.14, 16, 12), postMat);
        sp.position.set(p[0], p[1], 0.14);
        this.toolOverlayGroup.add(sp);

        // Vertical height guide pole showing exact wall height H
        const poleGeom = new THREE.CylinderGeometry(0.03, 0.03, h, 10);
        poleGeom.rotateX(Math.PI / 2);
        const pole = new THREE.Mesh(
          poleGeom,
          new THREE.MeshBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.7 })
        );
        pole.position.set(p[0], p[1], h * 0.5);
        this.toolOverlayGroup.add(pole);
      }

      if (cursorPt) {
        const curRing = new THREE.Mesh(
          new THREE.RingGeometry(0.12, 0.22, 24),
          new THREE.MeshBasicMaterial({ color: 0xfacc15, side: THREE.DoubleSide })
        );
        curRing.position.set(cursorPt[0], cursorPt[1], 0.02);
        this.toolOverlayGroup.add(curRing);
      }

      // Build committed wall segments + rubber-band segment to cursorPt
      const segments = [];
      for (let i = 0; i < pts.length - 1; i++) {
        segments.push({ a: pts[i], b: pts[i + 1], isLive: false });
      }
      if (pts.length >= 1 && cursorPt) {
        segments.push({ a: pts[pts.length - 1], b: [cursorPt[0], cursorPt[1]], isLive: true });
      }
      if (closed && pts.length >= 2) {
        const lastPt = cursorPt ? [cursorPt[0], cursorPt[1]] : pts[pts.length - 1];
        segments.push({ a: lastPt, b: pts[0], isLive: true });
      }

      for (const seg of segments) {
        const dx = seg.b[0] - seg.a[0];
        const dy = seg.b[1] - seg.a[1];
        const len = Math.hypot(dx, dy);
        if (len < 0.05) continue;
        const mx = (seg.a[0] + seg.b[0]) * 0.5;
        const my = (seg.a[1] + seg.b[1]) * 0.5;
        const yaw = Math.atan2(dy, dx);

        const wallGeom = new THREE.BoxGeometry(len, t, h);
        const wallMat = new THREE.MeshStandardMaterial({
          color: seg.isLive ? 0xfacc15 : 0x38bdf8,
          transparent: true,
          opacity: seg.isLive ? 0.35 : 0.52,
          roughness: 0.4,
        });
        const wallMesh = new THREE.Mesh(wallGeom, wallMat);
        wallMesh.position.set(mx, my, h * 0.5);
        wallMesh.rotation.set(0, 0, yaw, "ZYX");
        this.toolOverlayGroup.add(wallMesh);

        const edges = new THREE.EdgesGeometry(wallGeom);
        const edgeLine = new THREE.LineSegments(
          edges,
          new THREE.LineBasicMaterial({ color: seg.isLive ? 0xfde047 : 0x7dd3fc })
        );
        edgeLine.position.copy(wallMesh.position);
        edgeLine.rotation.copy(wallMesh.rotation);
        this.toolOverlayGroup.add(edgeLine);
      }

      // Update popup readout
      const countEl = document.getElementById("fp-vertex-count");
      const infoEl = document.getElementById("fp-segment-info");
      if (countEl) {
        countEl.textContent = `${pts.length} esquina${pts.length === 1 ? "" : "s"} marcada${pts.length === 1 ? "" : "s"} (Altura: ${h.toFixed(2)} m)`;
      }
      if (infoEl) {
        if (pts.length >= 1 && cursorPt) {
          const last = pts[pts.length - 1];
          const segLen = Math.hypot(cursorPt[0] - last[0], cursorPt[1] - last[1]);
          const segYaw = ((Math.atan2(cursorPt[1] - last[1], cursorPt[0] - last[0]) * 180) / Math.PI).toFixed(1);
          infoEl.textContent = `Tramo actual: ${segLen.toFixed(2)} m · Ángulo: ${segYaw}°`;
        } else if (cursorPt) {
          infoEl.textContent = `Cursor en [${cursorPt[0].toFixed(2)}, ${cursorPt[1].toFixed(2)}] m — Haz clic para fijar 1ª esquina`;
        }
      }
      return;
    }

    // -----------------------------------------------------------------------
    // 2. 3D LASER RULER & PROTRACTOR OVERLAY & POPUP READOUT
    // -----------------------------------------------------------------------
    if (this.activeViewportTool === "ruler") {
      const rPts = this.rulerPoints || [];
      const p1 = rPts.length >= 1 ? rPts[0] : cursorPt;
      const p2 = rPts.length >= 2 ? rPts[1] : rPts.length === 1 ? cursorPt : null;

      if (p1) {
        const m1 = new THREE.Mesh(
          new THREE.SphereGeometry(0.13, 16, 12),
          new THREE.MeshBasicMaterial({ color: 0x22c55e })
        );
        m1.position.set(p1[0], p1[1], Math.max(0.13, p1[2] || 0));
        this.toolOverlayGroup.add(m1);
      }

      if (p1 && p2) {
        const m2 = new THREE.Mesh(
          new THREE.SphereGeometry(0.13, 16, 12),
          new THREE.MeshBasicMaterial({ color: 0xef4444 })
        );
        m2.position.set(p2[0], p2[1], Math.max(0.13, p2[2] || 0));
        this.toolOverlayGroup.add(m2);

        const z1 = Math.max(0.06, p1[2] || 0);
        const z2 = Math.max(0.06, p2[2] || 0);

        // Main yellow 3D laser line
        const mainLineGeom = new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(p1[0], p1[1], z1),
          new THREE.Vector3(p2[0], p2[1], z2),
        ]);
        this.toolOverlayGroup.add(
          new THREE.Line(mainLineGeom, new THREE.LineBasicMaterial({ color: 0xfacc15, linewidth: 3 }))
        );

        // Right-triangle ΔX (red) and ΔY (green) breakdown on the floor
        const cornerPt = new THREE.Vector3(p2[0], p1[1], z1);
        const dxGeom = new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(p1[0], p1[1], z1),
          cornerPt,
        ]);
        const dyGeom = new THREE.BufferGeometry().setFromPoints([
          cornerPt,
          new THREE.Vector3(p2[0], p2[1], z1),
        ]);
        this.toolOverlayGroup.add(
          new THREE.Line(dxGeom, new THREE.LineBasicMaterial({ color: 0xef4444 }))
        );
        this.toolOverlayGroup.add(
          new THREE.Line(dyGeom, new THREE.LineBasicMaterial({ color: 0x22c55e }))
        );

        const dx = p2[0] - p1[0];
        const dy = p2[1] - p1[1];
        const dz = (p2[2] || 0) - (p1[2] || 0);
        const dist3d = Math.hypot(dx, dy, dz);
        const yawRad = Math.atan2(dy, dx);
        const yawDeg = (yawRad * 180) / Math.PI;

        const distEl = document.getElementById("ruler-dist-main");
        const deltasEl = document.getElementById("ruler-deltas");
        const anglesEl = document.getElementById("ruler-angles");
        if (distEl) distEl.textContent = `${dist3d.toFixed(3)} m`;
        if (deltasEl) {
          deltasEl.textContent = `ΔX: ${dx.toFixed(2)} m · ΔY: ${dy.toFixed(2)} m · ΔZ: ${dz.toFixed(2)} m`;
        }
        if (anglesEl) {
          anglesEl.textContent = `Yaw ENU: ${yawDeg.toFixed(1)}° (${yawRad.toFixed(3)} rad)`;
        }
      }
      return;
    }

    // -----------------------------------------------------------------------
    // 3. INTERACTIVE 3D SPLINE / CURVE TRACER OVERLAY
    // -----------------------------------------------------------------------
    if (this.activeViewportTool === "spline") {
      const sPts = [...(this.splinePoints || [])];
      if (cursorPt) sPts.push([cursorPt[0], cursorPt[1]]);

      for (const p of sPts) {
        const sp = new THREE.Mesh(
          new THREE.SphereGeometry(0.12, 14, 10),
          new THREE.MeshBasicMaterial({ color: 0xfacc15 })
        );
        sp.position.set(p[0], p[1], 0.12);
        this.toolOverlayGroup.add(sp);
      }

      if (sPts.length >= 2) {
        const vecs = sPts.map((p) => new THREE.Vector3(p[0], p[1], 0.08));
        const curve = new THREE.CatmullRomCurve3(vecs);
        const sampled = curve.getPoints(48);
        const curveGeom = new THREE.BufferGeometry().setFromPoints(sampled);
        this.toolOverlayGroup.add(
          new THREE.Line(curveGeom, new THREE.LineBasicMaterial({ color: 0x38bdf8, linewidth: 2 }))
        );
      }

      const infoEl = document.getElementById("spline-live-info");
      if (infoEl) {
        const c = (this.splinePoints || []).length;
        infoEl.innerHTML = `<span style="color:#fde047;font-weight:700;">${c} punto${c === 1 ? "" : "s"} fijado${c === 1 ? "" : "s"} en la curva</span>`;
      }
      return;
    }

    // -----------------------------------------------------------------------
    // 4. INTERACTIVE 3D SCATTER BRUSH RING OVERLAY
    // -----------------------------------------------------------------------
    if (this.activeViewportTool === "scatter_paint" && cursorPt) {
      const r = Math.max(0.5, parseFloat(document.getElementById("scatter-radius")?.value || "6.0"));
      const ringGeom = new THREE.RingGeometry(Math.max(0.1, r - 0.12), r, 48);
      const ringMat = new THREE.MeshBasicMaterial({
        color: 0x10b981,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.65,
      });
      const ringMesh = new THREE.Mesh(ringGeom, ringMat);
      ringMesh.position.set(cursorPt[0], cursorPt[1], 0.03);
      this.toolOverlayGroup.add(ringMesh);

      const discGeom = new THREE.CircleGeometry(r, 48);
      const discMat = new THREE.MeshBasicMaterial({
        color: 0x10b981,
        transparent: true,
        opacity: 0.12,
      });
      const discMesh = new THREE.Mesh(discGeom, discMat);
      discMesh.position.set(cursorPt[0], cursorPt[1], 0.02);
      this.toolOverlayGroup.add(discMesh);
      return;
    }

    // -----------------------------------------------------------------------
    // 5. NAV2 WAYPOINT TARGET PREVIEW
    // -----------------------------------------------------------------------
    if (this.activeViewportTool === "waypoint" && cursorPt) {
      const wpRing = new THREE.Mesh(
        new THREE.RingGeometry(0.18, 0.30, 24),
        new THREE.MeshBasicMaterial({ color: 0x06b6d4, side: THREE.DoubleSide })
      );
      wpRing.position.set(cursorPt[0], cursorPt[1], 0.03);
      this.toolOverlayGroup.add(wpRing);
    }
  }

  _commitFloorplanWalls() {
    const pts = [...(this.floorplanPoints || [])];
    const h = Math.max(0.3, parseFloat(document.getElementById("fp-wall-height")?.value || "2.50"));
    const t = Math.max(0.05, parseFloat(document.getElementById("fp-wall-thick")?.value || "0.20"));
    const wallType = document.getElementById("fp-wall-type")?.value || "wall";
    const closed = Boolean(document.getElementById("fp-wall-closed")?.checked);

    if (pts.length < 2) {
      this._setStatus("⚠ Haz clic en al menos 2 esquinas sobre el suelo 3D antes de levantar los muros (o usa '⚡ Crear Recinto Rápido').");
      return;
    }

    this.sendCommand({
      type: "extrude_floorplan",
      points: pts,
      height: h,
      thickness: t,
      wall_type: wallType,
      closed,
    });
    const segCount = closed && pts.length >= 3 ? pts.length : pts.length - 1;
    this.floorplanPoints = [];
    this._updateToolOverlay3D(this.cursorWorldPt);
    this._setStatus(`🧱 Levantados ${segCount} muros 3D (altura ${h.toFixed(2)} m, grosor ${t.toFixed(2)} m)`);
  }

  _commitSplineClones() {
    const pts = [...(this.splinePoints || [])];
    if (pts.length < 2) {
      this._setStatus("⚠ Haz clic en al menos 2 puntos del suelo para trazar la curva Bézier.");
      return;
    }
    const count = parseInt(document.getElementById("spline-count")?.value || "7", 10);
    const alignTan = Boolean(document.getElementById("spline-tangent")?.checked ?? true);
    this.sendCommand({
      type: "spline_clone",
      node_id: this.selectedNodeId || "",
      control_points: pts,
      count,
      align_tangent: alignTan,
    });
    this.splinePoints = [];
    this._updateToolOverlay3D(this.cursorWorldPt);
    this._setStatus(`〰 Clonados ${count} objetos a lo largo de la curva trazada`);
  }

  _handleViewportClick(event) {
    // Interactive 3D Viewport Tools: Floorplan Wall Tracer, 3D Laser Ruler, Spline Tracer, Scatter Brush, Waypoint
    if (this.activeViewportTool && this.activeViewportTool !== "boxselect") {
      const pt = this._raycastGroundOrSurface(event, this.activeViewportTool === "ruler");
      if (!pt) return;

      if (this.activeViewportTool === "waypoint") {
        this.sendCommand({
          type: "add_waypoint",
          pose: [pt[0], pt[1], 0.02, 0.0, 0.0, 0.0],
        });
        this._setStatus(`📍 Waypoint Nav2 colocado en [${pt[0]}, ${pt[1]}] m (Esc para salir)`);
        return;
      }

      if (this.activeViewportTool === "ruler") {
        this.rulerPoints = this.rulerPoints || [];
        if (this.rulerPoints.length >= 2) {
          this.rulerPoints = [pt];
        } else {
          this.rulerPoints.push(pt);
        }
        this._updateToolOverlay3D(pt);
        if (this.rulerPoints.length === 1) {
          this._setStatus(`📏 Regla 3D: Punto 1 fijado en [${pt[0]}, ${pt[1]}, ${pt[2]}] m. Mueve el ratón y haz clic en el Punto 2.`);
        } else {
          const p1 = this.rulerPoints[0];
          const p2 = this.rulerPoints[1];
          const dist = Math.hypot(p2[0] - p1[0], p2[1] - p1[1], (p2[2] || 0) - (p1[2] || 0));
          this._setStatus(`📏 Medida fijada: ${dist.toFixed(3)} m (Haz clic de nuevo para iniciar otra medida)`);
        }
        return;
      }

      if (this.activeViewportTool === "floorplan") {
        this.floorplanPoints = this.floorplanPoints || [];
        this.floorplanPoints.push([pt[0], pt[1]]);
        this._updateToolOverlay3D(pt);
        this._setStatus(
          `🧱 Esquina #${this.floorplanPoints.length} marcada en [${pt[0]}, ${pt[1]}] m. Pulsa '🧱 Levantar Muros (Enter)' en el popup cuando termines.`
        );
        return;
      }

      if (this.activeViewportTool === "spline") {
        this.splinePoints = this.splinePoints || [];
        this.splinePoints.push([pt[0], pt[1]]);
        this._updateToolOverlay3D(pt);
        this._setStatus(
          `〰 Punto de curva #${this.splinePoints.length} en [${pt[0]}, ${pt[1]}] m. Pulsa '〰 Clonar en Curva (Enter)' para generar.`
        );
        return;
      }

      if (this.activeViewportTool === "scatter_paint") {
        const archSel = document.getElementById("scatter-arch")?.value || "cardboard_box";
        const useSelected = archSel === "__selected__" && this.selectedNodeId;
        const count = parseInt(document.getElementById("scatter-count")?.value || "10", 10);
        const radius = parseFloat(document.getElementById("scatter-radius")?.value || "6.0");
        const minDist = parseFloat(document.getElementById("scatter-mindist")?.value || "0.9");
        this.sendCommand({
          type: "scatter_brush",
          node_id: useSelected ? this.selectedNodeId : "",
          archetype: useSelected ? "cardboard_box" : archSel,
          center_x: pt[0],
          center_y: pt[1],
          radius,
          count,
          min_distance: minDist,
          seed: Math.floor(Math.random() * 100000),
        });
        this._setStatus(`🎯 Dispersados ${count} objetos en torno a [${pt[0]}, ${pt[1]}] m`);
        return;
      }
    }

    const rect = this.renderer.domElement.getBoundingClientRect();
    const mouse = new THREE.Vector2(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1
    );
    this.raycaster.setFromCamera(mouse, this.activeCamera);

    const meshes = [];
    this.nodesRootGroup.traverse((obj) => {
      if (obj.isMesh && obj.userData.nodeId) {
        const parentGroup = this.nodeObjectMap.get(obj.userData.nodeId);
        if (parentGroup && parentGroup.visible && !parentGroup.userData.locked) {
          meshes.push(obj);
        }
      }
    });

    const hits = this.raycaster.intersectObjects(meshes, false);
    const toggleMulti = Boolean(
      event.shiftKey || event.ctrlKey || event.metaKey || this.activeViewportTool === "boxselect"
    );
    if (hits.length > 0) {
      const hitNodeId = hits[0].object.userData.nodeId;
      this.selectNode(hitNodeId, toggleMulti);
    } else if (!toggleMulti) {
      this.selectNode(null, false);
    }
  }

  _updateMarqueeBoxOverlay(x0, y0, x1, y1) {
    const boxEl = document.getElementById("selection-marquee-box");
    const container = document.getElementById("viewport-container");
    if (!boxEl || !container) return;
    const rect = container.getBoundingClientRect();
    const left = Math.min(x0, x1) - rect.left;
    const top = Math.min(y0, y1) - rect.top;
    const width = Math.abs(x1 - x0);
    const height = Math.abs(y1 - y0);
    boxEl.style.left = `${left}px`;
    boxEl.style.top = `${top}px`;
    boxEl.style.width = `${width}px`;
    boxEl.style.height = `${height}px`;
    boxEl.classList.remove("hidden");
  }

  _finishMarqueeBoxSelection(x0, y0, x1, y1, additive = false) {
    const rect = this.renderer.domElement.getBoundingClientRect();
    const minX = Math.min(x0, x1) - rect.left;
    const maxX = Math.max(x0, x1) - rect.left;
    const minY = Math.min(y0, y1) - rect.top;
    const maxY = Math.max(y0, y1) - rect.top;

    if (!additive) {
      this.selectedNodeIds.clear();
    }

    for (const node of this.sceneGraph?.nodes || []) {
      if (node.type === "ground_plane" || node.locked || node.visible === false) continue;
      const grp = this.nodeObjectMap.get(node.id);
      if (!grp || !grp.visible) continue;
      grp.updateMatrixWorld(true);

      const box3 = new THREE.Box3().setFromObject(grp);
      const testPts = [grp.position.clone()];
      if (!box3.isEmpty()) {
        const bMin = box3.min;
        const bMax = box3.max;
        testPts.push(
          new THREE.Vector3(bMin.x, bMin.y, bMin.z),
          new THREE.Vector3(bMax.x, bMin.y, bMin.z),
          new THREE.Vector3(bMin.x, bMax.y, bMin.z),
          new THREE.Vector3(bMax.x, bMax.y, bMin.z),
          new THREE.Vector3(bMin.x, bMin.y, bMax.z),
          new THREE.Vector3(bMax.x, bMin.y, bMax.z),
          new THREE.Vector3(bMin.x, bMax.y, bMax.z),
          new THREE.Vector3(bMax.x, bMax.y, bMax.z)
        );
      }

      let sMinX = 1e9;
      let sMaxX = -1e9;
      let sMinY = 1e9;
      let sMaxY = -1e9;
      let anyInFront = false;

      for (const pt of testPts) {
        const proj = pt.project(this.activeCamera);
        if (proj.z >= -1 && proj.z <= 1) {
          anyInFront = true;
          const sx = (proj.x + 1) * 0.5 * rect.width;
          const sy = (-proj.y + 1) * 0.5 * rect.height;
          if (sx < sMinX) sMinX = sx;
          if (sx > sMaxX) sMaxX = sx;
          if (sy < sMinY) sMinY = sy;
          if (sy > sMaxY) sMaxY = sy;
        }
      }

      if (anyInFront && sMaxX >= minX && sMinX <= maxX && sMaxY >= minY && sMinY <= maxY) {
        this.selectedNodeIds.add(node.id);
      }
    }

    const idsArr = Array.from(this.selectedNodeIds);
    this.selectedNodeId = idsArr.length > 0 ? idsArr[idsArr.length - 1] : null;
    this._refreshSelectionUI();
    if (idsArr.length > 0) {
      this._setStatus(
        `🔲 ${idsArr.length} objeto(s) seleccionado(s) en caja. Usa G/R para moverlos juntos o '🔗 Agrupar (Ctrl+G)'.`
      );
    }
  }

  selectNode(nodeId, toggleMulti = false) {
    if (!nodeId) {
      if (!toggleMulti) {
        this.selectedNodeId = null;
        this.selectedNodeIds.clear();
      }
    } else if (toggleMulti) {
      if (this.selectedNodeIds.has(nodeId) && this.selectedNodeIds.size > 1) {
        this.selectedNodeIds.delete(nodeId);
        const rem = Array.from(this.selectedNodeIds);
        this.selectedNodeId = rem[rem.length - 1] || null;
      } else {
        this.selectedNodeIds.add(nodeId);
        this.selectedNodeId = nodeId;
      }
    } else {
      this.selectedNodeId = nodeId;
      this.selectedNodeIds = new Set([nodeId]);
    }
    this._refreshSelectionUI();
  }

  _refreshSelectionUI() {
    this._rebuild3DSceneFromGraph();
    this._renderOutliner();
    this._renderInspector();
    this._highlightSelectedNodeInXml();

    const hudEl = document.getElementById("hud-selection-info");
    const count = this.selectedNodeIds.size;
    const node = this._getSelectedNode();
    if (count > 1) {
      hudEl.innerHTML = `🔲 Multiselección: <b>${count} objetos</b> (Pulsa <b>🔗 Agrupar</b> o mueve con el Gizmo)`;
    } else if (node) {
      const p = node.pose || [0, 0, 0, 0, 0, 0];
      hudEl.innerHTML = `Seleccionado: <b>${node.name}</b> [${p[0].toFixed(2)}, ${p[1].toFixed(2)}, ${p[2].toFixed(2)} m]`;
    } else {
      hudEl.textContent = "Ningún objeto seleccionado (Shift+Clic para multiselección)";
    }
  }

  _getSelectedNode() {
    if (!this.sceneGraph || !this.selectedNodeId) return null;
    return this.sceneGraph.nodes.find((n) => n.id === this.selectedNodeId) || null;
  }

  _syncLiveInspectorFrom3DObject() {
    const group = this.nodeObjectMap.get(this.selectedNodeId);
    const node = this._getSelectedNode();
    if (!group || !node) return;

    // If multi-selection is active, move/rotate all other selected objects by the same delta!
    if (this.multiDragStart && this.multiDragStart.others.size > 0) {
      const dx = group.position.x - this.multiDragStart.primaryStartPos.x;
      const dy = group.position.y - this.multiDragStart.primaryStartPos.y;
      const dz = group.position.z - this.multiDragStart.primaryStartPos.z;
      const deltaQuat = group.quaternion
        .clone()
        .multiply(this.multiDragStart.primaryStartQuat.clone().invert());

      for (const [otherId, st] of this.multiDragStart.others.entries()) {
        const og = this.nodeObjectMap.get(otherId);
        const onode = this.sceneGraph?.nodes?.find((n) => n.id === otherId);
        if (!og || !onode) continue;
        if (this.transformControls.mode === "rotate") {
          const relOffset = st.startPos.clone().sub(this.multiDragStart.primaryStartPos);
          relOffset.applyQuaternion(deltaQuat);
          og.position.copy(this.multiDragStart.primaryStartPos).add(relOffset);
          og.quaternion.copy(deltaQuat).multiply(st.startQuat);
        } else {
          og.position.set(st.startPos.x + dx, st.startPos.y + dy, st.startPos.z + dz);
        }
        const oEuler = new THREE.Euler().setFromQuaternion(og.quaternion, "ZYX");
        onode.pose = [
          Number(og.position.x.toFixed(4)),
          Number(og.position.y.toFixed(4)),
          Number(og.position.z.toFixed(4)),
          Number(oEuler.x.toFixed(4)),
          Number(oEuler.y.toFixed(4)),
          Number(oEuler.z.toFixed(4)),
        ];
      }
    }

    const eulerZyx = new THREE.Euler().setFromQuaternion(group.quaternion, "ZYX");
    node.pose = [
      Number(group.position.x.toFixed(4)),
      Number(group.position.y.toFixed(4)),
      Number(group.position.z.toFixed(4)),
      Number(eulerZyx.x.toFixed(4)),
      Number(eulerZyx.y.toFixed(4)),
      Number(eulerZyx.z.toFixed(4)),
    ];
    node.scale = [
      Number(group.scale.x.toFixed(4)),
      Number(group.scale.y.toFixed(4)),
      Number(group.scale.z.toFixed(4)),
    ];
    this._renderInspector();
  }

  async _commitGizmoTransformToBackend() {
    const group = this.nodeObjectMap.get(this.selectedNodeId);
    const node = this._getSelectedNode();
    if (!group || !node) return;

    if (this.selectedNodeIds.size > 1) {
      const posesMap = {};
      for (const nid of this.selectedNodeIds) {
        const g = this.nodeObjectMap.get(nid);
        if (!g) continue;
        const eul = new THREE.Euler().setFromQuaternion(g.quaternion, "ZYX");
        posesMap[nid] = [
          Number(g.position.x.toFixed(6)),
          Number(g.position.y.toFixed(6)),
          Number(g.position.z.toFixed(6)),
          Number(eul.x.toFixed(6)),
          Number(eul.y.toFixed(6)),
          Number(eul.z.toFixed(6)),
        ];
      }
      await this.sendCommand({
        type: "batch_update_poses",
        poses_map: posesMap,
        label: `Transform ${Object.keys(posesMap).length} selected objects`,
      });
      return;
    }

    const eulerZyx = new THREE.Euler().setFromQuaternion(group.quaternion, "ZYX");
    const newPose = [
      Number(group.position.x.toFixed(6)),
      Number(group.position.y.toFixed(6)),
      Number(group.position.z.toFixed(6)),
      Number(eulerZyx.x.toFixed(6)),
      Number(eulerZyx.y.toFixed(6)),
      Number(eulerZyx.z.toFixed(6)),
    ];
    const newScale = [
      Math.max(0.001, Number(group.scale.x.toFixed(6))),
      Math.max(0.001, Number(group.scale.y.toFixed(6))),
      Math.max(0.001, Number(group.scale.z.toFixed(6))),
    ];

    await this.sendCommand({
      type: "update_node",
      node_id: node.id,
      patch: { pose: newPose, scale: newScale },
      label: `Transform '${node.name}'`,
    });
  }

  // ===========================================================================
  // 4. OUTLINER HIERARCHY RENDERING
  // ===========================================================================
  _renderOutliner() {
    const container = document.getElementById("outliner-tree");
    if (!this.sceneGraph) return;
    container.innerHTML = "";

    const folders = this.sceneGraph.folders || [];
    for (const folder of folders) {
      const fDiv = document.createElement("div");
      fDiv.className = "folder-group";

      const fHeader = document.createElement("div");
      fHeader.className = "folder-header";
      fHeader.innerHTML = `<span>📁 ${folder.name}</span>`;
      fDiv.appendChild(fHeader);

      const folderNodes = this.sceneGraph.nodes.filter(
        (n) => (n.folder_id || "folder_props") === folder.id
      );

      for (const node of folderNodes) {
        const isSel = node.id === this.selectedNodeId || this.selectedNodeIds.has(node.id);
        const isMulti = this.selectedNodeIds.size > 1 && this.selectedNodeIds.has(node.id);
        const item = document.createElement("div");
        item.className =
          "outliner-item" +
          (isSel ? " selected" : "") +
          (isMulti && node.id !== this.selectedNodeId ? " multi-selected" : "");

        const archIcons = {
          wall: "🧱",
          wall_doorway: "🚪",
          wall_window: "🪟",
          wall_corner_l: "📐",
          column_square: "🏛",
          stairs: "🪜",
          ramp: "📐",
          road_straight: "🛣",
          road_curved: "↪",
          road_intersection: "➕",
          road_crosswalk: "🦓",
          roundabout: "🔄",
          speed_bump: "〰",
          custom_group: "🔗",
          warehouse_rack: "🗄",
          table: "🪑",
          euro_pallet: "🪵",
          safety_fence: "🚧",
          jersey_barrier: "🚧",
          traffic_cone: "🔺",
          bollard: "🛑",
          cardboard_box: "📦",
        };
        const iconMap = {
          ground_plane: "⬛",
          light: "💡",
          mesh: "🧩",
          include: "🌐",
          waypoint: "📍",
          robot_spawn: "🤖",
          sensor: "📡",
          actor: "🚶",
          primitive:
            archIcons[node.archetype] ||
            (node.geometry?.shape === "cylinder"
              ? "🛢"
              : node.geometry?.shape === "sphere"
              ? "🟠"
              : "📦"),
        };
        const icon = iconMap[node.type] || "🔹";

        const nameSpan = document.createElement("div");
        nameSpan.className = "outliner-item-name";

        // Direct multi-select checkbox so users can check multiple objects effortlessly
        if (node.type !== "ground_plane" && node.type !== "light" && !node.locked) {
          const chk = document.createElement("input");
          chk.type = "checkbox";
          chk.className = "outliner-multi-chk";
          chk.checked = isSel;
          chk.title = "Marcar/desmarcar para multiselección (o usa Shift/Ctrl + Clic)";
          chk.addEventListener("click", (e) => {
            e.stopPropagation();
            this.selectNode(node.id, true);
          });
          nameSpan.appendChild(chk);
        }

        const iconAndLabel = document.createElement("span");
        iconAndLabel.innerHTML = `${icon} ${node.name}`;
        nameSpan.appendChild(iconAndLabel);

        item.addEventListener("click", (e) => {
          if (e.shiftKey && this.selectedNodeId && this.selectedNodeId !== node.id) {
            // Range selection in Outliner order + additive toggle
            const selectableIds = this.sceneGraph.nodes
              .filter((n) => n.type !== "ground_plane" && !n.locked && n.visible !== false)
              .map((n) => n.id);
            const idxA = selectableIds.indexOf(this.selectedNodeId);
            const idxB = selectableIds.indexOf(node.id);
            if (idxA !== -1 && idxB !== -1) {
              const startIdx = Math.min(idxA, idxB);
              const endIdx = Math.max(idxA, idxB);
              for (let k = startIdx; k <= endIdx; k++) {
                this.selectedNodeIds.add(selectableIds[k]);
              }
              this.selectedNodeId = node.id;
              this._refreshSelectionUI();
              return;
            }
          }
          const toggleMulti = Boolean(
            e.shiftKey || e.ctrlKey || e.metaKey || this.activeViewportTool === "boxselect"
          );
          this.selectNode(node.id, toggleMulti);
        });

        const toggles = document.createElement("div");
        toggles.className = "outliner-toggles";

        // Visibility Eye Toggle
        const btnVis = document.createElement("button");
        btnVis.className = "tiny-toggle" + (node.visible === false ? " off" : "");
        btnVis.textContent = "👁";
        btnVis.title = node.visible === false ? "Mostrar objeto" : "Ocultar objeto";
        btnVis.addEventListener("click", (e) => {
          e.stopPropagation();
          this.sendCommand({
            type: "update_node",
            node_id: node.id,
            patch: { visible: node.visible === false },
            label: `Toggle visibility '${node.name}'`,
          });
        });

        // Lock Selection Toggle
        const btnLock = document.createElement("button");
        btnLock.className = "tiny-toggle" + (!node.locked ? " off" : "");
        btnLock.textContent = node.locked ? "🔒" : "🔓";
        btnLock.title = node.locked ? "Desbloquear selección" : "Bloquear selección";
        btnLock.addEventListener("click", (e) => {
          e.stopPropagation();
          this.sendCommand({
            type: "update_node",
            node_id: node.id,
            patch: { locked: !node.locked },
            label: `Toggle lock '${node.name}'`,
          });
        });

        toggles.appendChild(btnVis);
        toggles.appendChild(btnLock);

        item.appendChild(nameSpan);
        item.appendChild(toggles);
        fDiv.appendChild(item);
      }

      container.appendChild(fDiv);
    }
  }

  // ===========================================================================
  // 5. PROPERTY & PHYSICS INSPECTOR PANEL
  // ===========================================================================
  _renderInspector() {
    const container = document.getElementById("inspector-content");
    const node = this._getSelectedNode();

    if (!node) {
      container.innerHTML = `
        <div class="empty-inspector">
          Selecciona cualquier entidad en el Visor 3D o en la Jerarquía (Outliner) para editar su pose ENU, dimensiones, color y propiedades físicas.
        </div>`;
      return;
    }

    const pose = node.pose || [0, 0, 0, 0, 0, 0];
    const scale = node.scale || [1, 1, 1];
    const factor = this.showAnglesInDegrees ? RAD2DEG : 1.0;
    const unitSymbol = this.showAnglesInDegrees ? "°" : "rad";

    const rDisp = (pose[3] * factor).toFixed(2);
    const pDisp = (pose[4] * factor).toFixed(2);
    const yDisp = (pose[5] * factor).toFixed(2);
    const multiCount = (this.selectedNodeIds || new Set()).size;

    let html = "";
    if (multiCount >= 2) {
      html += `
      <!-- MULTI-SELECTION & GROUPING CARD -->
      <div class="insp-card" style="border-color:rgba(167,139,250,0.45);background:rgba(167,139,250,0.08);">
        <div class="insp-card-title" style="color:#c4b5fd;">🔲 Multiselección Activa (${multiCount} objetos)</div>
        <div style="font-size:11px;color:var(--text-secondary);line-height:1.35;">
          El Gizmo 3D mueve y rota los <b>${multiCount} objetos a la vez</b>. También puedes agruparlos en un único bloque o apilar el último sobre el primero.
        </div>
        <div class="palette-grid" style="margin-top:4px;">
          <button id="btn-insp-group" class="palette-btn" title="Fusiona los ${multiCount} objetos seleccionados en un único objeto compuesto (Ctrl+G)">🔗 Agrupar en 1 Bloque</button>
          <button id="btn-insp-stack" class="palette-btn" title="Coloca el objeto activo justo ENCIMA del otro objeto seleccionado (Tecla B)">⬆️ Apilar Encima (B)</button>
        </div>
      </div>`;
    }

    html += `
      <!-- IDENTITY CARD -->
      <div class="insp-card">
        <div class="insp-card-title">Identidad y Estado</div>
        <div class="insp-row">
          <span class="insp-label">Nombre</span>
          <input id="insp-name" class="insp-input" type="text" value="${node.name}" />
        </div>
        <div class="insp-row">
          <span class="insp-label">Estático (&lt;static&gt;)</span>
          <input id="insp-static" type="checkbox" ${node.static ? "checked" : ""} ${
      node.type === "ground_plane" || node.type === "light" ? "disabled" : ""
    } />
        </div>
        ${
          node.type !== "ground_plane" && node.type !== "light"
            ? `<button id="btn-save-custom-prefab" class="tool-btn" style="width:100%;margin-top:4px;text-align:center;" title="Guardar este objeto en la sección 'Objetos Personalizados' para reutilizarlo">⭐ Guardar en Objetos Personalizados</button>`
            : ""
        }
      </div>

      <!-- 6-DOF ENU POSE CARD -->
      <div class="insp-card">
        <div class="insp-card-title">Pose 6-DOF (ENU Z-Up)</div>
        <span class="insp-label">Posición [x, y, z] (metros)</span>
        <div class="vec3-grid">
          <div class="vec-field"><span class="vec-axis x">X</span><input id="insp-px" type="number" step="0.1" value="${pose[0]}" /></div>
          <div class="vec-field"><span class="vec-axis y">Y</span><input id="insp-py" type="number" step="0.1" value="${pose[1]}" /></div>
          <div class="vec-field"><span class="vec-axis z">Z</span><input id="insp-pz" type="number" step="0.1" value="${pose[2]}" /></div>
        </div>
        <span class="insp-label">Orientación [Roll, Pitch, Yaw] (${unitSymbol})</span>
        <div class="vec3-grid">
          <div class="vec-field"><span class="vec-axis x">R</span><input id="insp-rr" type="number" step="1" value="${rDisp}" /></div>
          <div class="vec-field"><span class="vec-axis y">P</span><input id="insp-rp" type="number" step="1" value="${pDisp}" /></div>
          <div class="vec-field"><span class="vec-axis z">Y</span><input id="insp-ry" type="number" step="1" value="${yDisp}" /></div>
        </div>
        <span class="insp-label">Escala [Sx, Sy, Sz]</span>
        <div class="vec3-grid">
          <div class="vec-field"><span class="vec-axis x">X</span><input id="insp-sx" type="number" step="0.1" min="0.01" value="${scale[0]}" /></div>
          <div class="vec-field"><span class="vec-axis y">Y</span><input id="insp-sy" type="number" step="0.1" min="0.01" value="${scale[1]}" /></div>
          <div class="vec-field"><span class="vec-axis z">Z</span><input id="insp-sz" type="number" step="0.1" min="0.01" value="${scale[2]}" /></div>
        </div>
      </div>
    `;

    // ARCHETYPE / COMPOSITE PARAMETRIC BUILDER CARD
    if (node.archetype && node.archetype_params) {
      const paramLabels = {
        length: "Largo (m)",
        length_x: "Largo Eje X (m)",
        length_y: "Largo Eje Y (m)",
        width: "Ancho (m)",
        depth: "Profundidad (m)",
        thickness: "Grosor (m)",
        height: "Altura (m)",
        door_width: "Ancho Puerta (m)",
        door_height: "Alto Puerta (m)",
        window_width: "Ancho Ventana (m)",
        window_height: "Alto Ventana (m)",
        sill_height: "Altura Antepecho (m)",
        steps: "Nº Escalones",
        step_depth: "Huella Peldaño (m)",
        step_height: "Contrahuella (m)",
        shelves: "Nº Baldas",
        post_width: "Grosor Poste (m)",
        top_thickness: "Grosor Tablero (m)",
        leg_width: "Grosor Patas (m)",
        rail_radius: "Radio Tubo (m)",
        bottom_width: "Ancho Base (m)",
        top_width: "Ancho Superior (m)",
        base_width: "Ancho Base (m)",
        radius: "Radio (m)",
        curb_width: "Ancho Acera/Bordillo (m)",
        angle_deg: "Ángulo Curva (°)",
        arm_length: "Largo Ramal (m)",
        stripes: "Nº Franjas Cebra",
        outer_radius: "Radio Exterior (m)",
        inner_radius: "Radio Isleta Central (m)",
        dx: "Largo X (m)",
        dy: "Ancho Y (m)",
        dz: "Alto Z (m)",
      };
      const hexColor = rgbaToHex(node.material?.color);
      const partCount = Array.isArray(node.parts) ? node.parts.length : 1;

      html += `<div class="insp-card">
        <div class="insp-card-title">🏗 Elemento Paramétrico: ${node.archetype.toUpperCase()} (${partCount} pieza${partCount > 1 ? "s" : ""})</div>`;

      for (const [pKey, pVal] of Object.entries(node.archetype_params)) {
        if (pKey === "segments") continue;
        const isInt = pKey === "steps" || pKey === "shelves";
        const step = isInt ? "1" : "0.05";
        const min = isInt ? "1" : "0.02";
        const lbl = paramLabels[pKey] || pKey;
        html += `
          <div class="insp-row">
            <span class="insp-label">${lbl}</span>
            <input class="insp-input insp-arch-param" data-param-key="${pKey}" type="number" step="${step}" min="${min}" value="${pVal}" />
          </div>`;
      }

      html += `
        <div class="insp-row">
          <span class="insp-label">Color Principal</span>
          <input id="insp-color" type="color" value="${hexColor}" style="width:70px;height:26px;border:none;background:transparent;cursor:pointer;" />
        </div>`;

      if (partCount > 1) {
        html += `
          <button id="btn-explode-composite" class="tool-btn" style="width:100%;margin-top:4px;text-align:center;" title="Convierte las sub-partes de este elemento compuesto en objetos primitivos independientes">
            💥 Desglosar en ${partCount} Piezas Sueltas
          </button>`;
      }
      html += `</div>`;
    } else if (node.type === "primitive" || node.type === "ground_plane") {
      // GEOMETRY & MATERIAL CARD FOR SIMPLE PRIMITIVES
      const geom = node.geometry || {};
      const shape = geom.shape || "box";
      const hexColor = rgbaToHex(node.material?.color);

      html += `<div class="insp-card">
        <div class="insp-card-title">Geometría (${shape.toUpperCase()}) y Material</div>`;

      if (shape === "box") {
        const sz = geom.size || [1, 1, 1];
        html += `
          <span class="insp-label">Dimensiones [dx, dy, dz] (m)</span>
          <div class="vec3-grid">
            <div class="vec-field"><span class="vec-axis x">W</span><input id="insp-gdx" type="number" step="0.1" min="0.01" value="${sz[0]}" /></div>
            <div class="vec-field"><span class="vec-axis y">D</span><input id="insp-gdy" type="number" step="0.1" min="0.01" value="${sz[1]}" /></div>
            <div class="vec-field"><span class="vec-axis z">H</span><input id="insp-gdz" type="number" step="0.1" min="0.01" value="${sz[2]}" /></div>
          </div>`;
      } else if (shape === "cylinder") {
        html += `
          <div class="insp-row">
            <span class="insp-label">Radio (m)</span>
            <input id="insp-grad" class="insp-input" type="number" step="0.05" min="0.01" value="${geom.radius ?? 0.5}" />
          </div>
          <div class="insp-row">
            <span class="insp-label">Longitud Z (m)</span>
            <input id="insp-glen" class="insp-input" type="number" step="0.1" min="0.01" value="${geom.length ?? 1.0}" />
          </div>`;
      } else if (shape === "sphere") {
        html += `
          <div class="insp-row">
            <span class="insp-label">Radio (m)</span>
            <input id="insp-grad" class="insp-input" type="number" step="0.05" min="0.01" value="${geom.radius ?? 0.5}" />
          </div>`;
      } else if (shape === "plane") {
        const sz = geom.size || [100, 100];
        html += `
          <div class="insp-row">
            <span class="insp-label">Tamaño XY (m)</span>
            <input id="insp-gplane" class="insp-input" type="number" step="5" min="1" value="${sz[0]}" />
          </div>`;
      }

      html += `
        <div class="insp-row">
          <span class="insp-label">Color Difuso</span>
          <input id="insp-color" type="color" value="${hexColor}" style="width:70px;height:26px;border:none;background:transparent;cursor:pointer;" />
        </div>
      </div>`;
    }

    // CAD MESH & COLLISION OPTIMIZER CARD (PHASE 2)
    if (node.type === "mesh") {
      const mInfo = node.mesh || {};
      const col = node.collision || {};
      const strat = col.strategy || "aabb";
      const hexColor = rgbaToHex(node.material?.color);

      html += `
        <div class="insp-card">
          <div class="insp-card-title">🧩 Malla CAD (${mInfo.filename || "mesh"})</div>
          <div class="insp-row">
            <span class="insp-label">Triángulos</span>
            <span style="font-family:var(--font-mono);font-size:11.5px;">${mInfo.triangle_count || "?"} tris</span>
          </div>
          <div class="insp-row">
            <span class="insp-label">Unidades CAD</span>
            <select id="insp-mesh-unit" class="insp-select">
              <option value="m" ${mInfo.unit === "m" ? "selected" : ""}>Metros (m × 1.0)</option>
              <option value="mm" ${mInfo.unit === "mm" ? "selected" : ""}>Milímetros → m (× 0.001)</option>
              <option value="cm" ${mInfo.unit === "cm" ? "selected" : ""}>Centímetros → m (× 0.01)</option>
            </select>
          </div>
          <div class="insp-row">
            <span class="insp-label">Eje Arriba</span>
            <select id="insp-mesh-axis" class="insp-select">
              <option value="Z_UP" ${mInfo.up_axis !== "Y_UP_TO_Z_UP" ? "selected" : ""}>Z-Up (Nativo ROS)</option>
              <option value="Y_UP_TO_Z_UP" ${mInfo.up_axis === "Y_UP_TO_Z_UP" ? "selected" : ""}>Y-Up → Z-Up (+90° X)</option>
            </select>
          </div>
          <div class="insp-row">
            <span class="insp-label">Optimizador &lt;collision&gt;</span>
            <select id="insp-col-strategy" class="insp-select">
              <option value="aabb" ${strat === "aabb" || strat === "primitive" ? "selected" : ""}>1) Caja Delimitadora (AABB)</option>
              <option value="cylinder" ${strat === "cylinder" ? "selected" : ""}>2) Cilindro Envolvente</option>
              <option value="sphere" ${strat === "sphere" ? "selected" : ""}>3) Esfera Envolvente</option>
              <option value="convex_hull" ${strat === "convex_hull" ? "selected" : ""}>4) Envolvente Convexa (Hull)</option>
              <option value="decimated_mesh" ${strat === "decimated_mesh" ? "selected" : ""}>5) Malla Simplificada (10%)</option>
              <option value="full_mesh" ${strat === "full_mesh" ? "selected" : ""}>6) Malla Original Completa</option>
            </select>
          </div>
          <div class="insp-row">
            <span class="insp-label">Color Visual</span>
            <input id="insp-color" type="color" value="${hexColor}" style="width:70px;height:26px;border:none;background:transparent;cursor:pointer;" />
          </div>
        </div>
      `;
    }

    // INCLUDE NODE CARD (GAZEBO FUEL / ROS 2 WORKSPACE)
    if (node.type === "include") {
      const uriVal = node.include?.uri || "";
      html += `
        <div class="insp-card">
          <div class="insp-card-title">🌐 Modelo Incluido (&lt;include&gt;)</div>
          <span class="insp-label">URI (Fuel / model:// / package://)</span>
          <input id="insp-inc-uri" class="insp-input" type="text" value="${uriVal}" />
        </div>
      `;
    }

    // AUTO-PHYSICS & INERTIA TENSOR CARD FOR PRIMITIVES & MESHES
    if (node.type === "primitive" || node.type === "mesh") {
      const inertial = node.inertial || {};
      const col = node.collision || {};
      const curPreset = inertial.material_preset || "wood";

      const materialOptions = Object.entries(this.presets.materials || {})
        .map(
          ([k, v]) =>
            `<option value="${k}" ${k === curPreset ? "selected" : ""}>${v.label} (${v.density} kg/m³)</option>`
        )
        .join("");

      html += `
        <div class="insp-card">
          <div class="insp-card-title">Física e Inercia (&lt;inertial&gt;)</div>
          <div class="insp-row">
            <span class="insp-label">Material</span>
            <select id="insp-mat-preset" class="insp-select">${materialOptions}</select>
          </div>
          <div class="insp-row">
            <span class="insp-label">Masa (kg)</span>
            <input id="insp-mass" class="insp-input" type="number" step="0.5" min="0.001" value="${Number(inertial.mass ?? 1.0).toFixed(3)}" />
          </div>
          <span class="insp-label">Tensor de Inercia (kg·m²)</span>
          <div class="inertia-matrix-grid">
            <div class="inertia-cell"><span>ixx</span><span>${Number(inertial.ixx ?? 0).toFixed(4)}</span></div>
            <div class="inertia-cell"><span>iyy</span><span>${Number(inertial.iyy ?? 0).toFixed(4)}</span></div>
            <div class="inertia-cell"><span>izz</span><span>${Number(inertial.izz ?? 0).toFixed(4)}</span></div>
            <div class="inertia-cell"><span>ixy</span><span>${Number(inertial.ixy ?? 0).toFixed(4)}</span></div>
            <div class="inertia-cell"><span>ixz</span><span>${Number(inertial.ixz ?? 0).toFixed(4)}</span></div>
            <div class="inertia-cell"><span>iyz</span><span>${Number(inertial.iyz ?? 0).toFixed(4)}</span></div>
          </div>
          <div class="insp-row">
            <span class="insp-label">Fricción (μ₁ / μ₂)</span>
            <input id="insp-mu" class="insp-input" type="number" step="0.05" min="0" value="${col.mu1 ?? 0.7}" />
          </div>
        </div>
      `;
    }

    // LIGHT INSPECTOR CARD
    if (node.type === "light") {
      const l = node.light || {};
      const dir = l.direction || [-0.5, 0.1, -0.9];
      const hexLight = rgbaToHex(l.diffuse);
      html += `
        <div class="insp-card">
          <div class="insp-card-title">Propiedades de Luz (${(l.light_type || "point").toUpperCase()})</div>
          <div class="insp-row">
            <span class="insp-label">Color</span>
            <input id="insp-lcolor" type="color" value="${hexLight}" style="width:70px;height:26px;border:none;background:transparent;" />
          </div>
          <div class="insp-row">
            <span class="insp-label">Intensidad</span>
            <input id="insp-lintensity" class="insp-input" type="number" step="0.1" min="0" value="${l.intensity ?? 1.0}" />
          </div>
          <div class="insp-row">
            <span class="insp-label">Sombras</span>
            <input id="insp-lshadows" type="checkbox" ${l.cast_shadows !== false ? "checked" : ""} />
          </div>
          ${
            l.light_type === "directional" || l.light_type === "spot"
              ? `<span class="insp-label">Dirección del Rayo [dx, dy, dz]</span>
                 <div class="vec3-grid">
                   <div class="vec-field"><span class="vec-axis x">X</span><input id="insp-ldx" type="number" step="0.1" value="${dir[0]}" /></div>
                   <div class="vec-field"><span class="vec-axis y">Y</span><input id="insp-ldy" type="number" step="0.1" value="${dir[1]}" /></div>
                   <div class="vec-field"><span class="vec-axis z">Z</span><input id="insp-ldz" type="number" step="0.1" value="${dir[2]}" /></div>
                 </div>`
              : ""
          }
        </div>
      `;
    }

    // PHASE 4.2: NAV2 WAYPOINT INSPECTOR CARD
    if (node.type === "waypoint") {
      const wp = node.waypoint || {};
      html += `
        <div class="insp-card">
          <div class="insp-card-title">📍 Waypoint Nav2</div>
          <div class="insp-row">
            <span class="insp-label">Espera (dwell s)</span>
            <input id="insp-wp-dwell" class="insp-input" type="number" step="0.5" min="0" value="${wp.dwell_time_s ?? 0.0}" />
          </div>
        </div>
      `;
    }

    // PHASE 4.3: ROBOT SPAWN INSPECTOR CARD
    if (node.type === "robot_spawn") {
      const rs = node.robot_spawn || {};
      html += `
        <div class="insp-card">
          <div class="insp-card-title">🤖 Spawn de Robot (ROS 2 Launch)</div>
          <div class="insp-row">
            <span class="insp-label">Modelo Robot</span>
            <input id="insp-rs-model" class="insp-input" type="text" value="${rs.robot_model || "turtlebot4"}" />
          </div>
          <div class="insp-row">
            <span class="insp-label">Namespace</span>
            <input id="insp-rs-ns" class="insp-input" type="text" value="${rs.robot_namespace || "robot1"}" />
          </div>
        </div>
      `;
    }

    // PHASE 4.3: STATIC ENVIRONMENTAL SENSOR INSPECTOR CARD
    if (node.type === "sensor") {
      const s = node.sensor || {};
      html += `
        <div class="insp-card">
          <div class="insp-card-title">📡 Sensor Fijo (${(s.sensor_type || "camera_rgbd").toUpperCase()})</div>
          <div class="insp-row">
            <span class="insp-label">Topic ROS/GZ</span>
            <input id="insp-sen-topic" class="insp-input" type="text" value="${s.topic || "/world_sensors/sensor"}" />
          </div>
          <div class="insp-row">
            <span class="insp-label">Frecuencia (Hz)</span>
            <input id="insp-sen-rate" class="insp-input" type="number" step="1" min="1" value="${s.update_rate ?? 15}" />
          </div>
          <div class="insp-row">
            <span class="insp-label">FOV (°)</span>
            <input id="insp-sen-fov" class="insp-input" type="number" step="5" min="10" max="170" value="${s.fov_deg ?? 75}" />
          </div>
          <div class="insp-row">
            <span class="insp-label">Alcance Máx (m)</span>
            <input id="insp-sen-range" class="insp-input" type="number" step="0.5" min="0.5" value="${s.range_max ?? 8.0}" />
          </div>
          <div class="insp-row">
            <span class="insp-label">Mostrar Frustum 3D</span>
            <input id="insp-sen-frustum" type="checkbox" ${s.show_frustum !== false ? "checked" : ""} />
          </div>
        </div>
      `;
    }

    // PHASE 4.4: DYNAMIC ACTOR (<actor>) INSPECTOR CARD
    if (node.type === "actor") {
      const a = node.actor || {};
      html += `
        <div class="insp-card">
          <div class="insp-card-title">🚶 Actor Dinámico (&lt;actor&gt;)</div>
          <div class="insp-row">
            <span class="insp-label">Velocidad (m/s)</span>
            <input id="insp-act-speed" class="insp-input" type="number" step="0.1" min="0.1" value="${a.speed_mps ?? 1.1}" />
          </div>
          <div class="insp-row">
            <span class="insp-label">Bucle Continuo</span>
            <input id="insp-act-loop" type="checkbox" ${a.loop !== false ? "checked" : ""} />
          </div>
        </div>
      `;
    }

    container.innerHTML = html;
    this._bindInspectorInputs(node);
  }

  _bindInspectorInputs(node) {
    const bindChange = (id, callback) => {
      const el = document.getElementById(id);
      if (el) el.addEventListener("change", () => callback(el));
    };

    bindChange("insp-name", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { name: el.value },
        label: `Rename to '${el.value}'`,
      });
    });

    bindChange("insp-static", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { static: el.checked },
        label: `Set static=${el.checked} on '${node.name}'`,
      });
    });

    // Pose & Scale inputs
    const updatePoseFromInputs = () => {
      const px = parseFloat(document.getElementById("insp-px")?.value || 0);
      const py = parseFloat(document.getElementById("insp-py")?.value || 0);
      const pz = parseFloat(document.getElementById("insp-pz")?.value || 0);
      const factor = this.showAnglesInDegrees ? DEG2RAD : 1.0;
      const rr = parseFloat(document.getElementById("insp-rr")?.value || 0) * factor;
      const rp = parseFloat(document.getElementById("insp-rp")?.value || 0) * factor;
      const ry = parseFloat(document.getElementById("insp-ry")?.value || 0) * factor;
      const sx = Math.max(0.01, parseFloat(document.getElementById("insp-sx")?.value || 1));
      const sy = Math.max(0.01, parseFloat(document.getElementById("insp-sy")?.value || 1));
      const sz = Math.max(0.01, parseFloat(document.getElementById("insp-sz")?.value || 1));

      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: {
          pose: [px, py, pz, rr, rp, ry],
          scale: [sx, sy, sz],
        },
        label: `Update Pose of '${node.name}'`,
      });
    };

    ["insp-px", "insp-py", "insp-pz", "insp-rr", "insp-rp", "insp-ry", "insp-sx", "insp-sy", "insp-sz"].forEach(
      (id) => bindChange(id, updatePoseFromInputs)
    );

    // Geometry dimensions
    ["insp-gdx", "insp-gdy", "insp-gdz"].forEach((id) => {
      bindChange(id, () => {
        const dx = Math.max(0.01, parseFloat(document.getElementById("insp-gdx")?.value || 1));
        const dy = Math.max(0.01, parseFloat(document.getElementById("insp-gdy")?.value || 1));
        const dz = Math.max(0.01, parseFloat(document.getElementById("insp-gdz")?.value || 1));
        this.sendCommand({
          type: "update_node",
          node_id: node.id,
          patch: { geometry: { size: [dx, dy, dz] } },
          label: `Resize Box '${node.name}'`,
        });
      });
    });

    bindChange("insp-grad", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { geometry: { radius: Math.max(0.01, parseFloat(el.value || 0.5)) } },
        label: `Change Radius of '${node.name}'`,
      });
    });

    bindChange("insp-glen", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { geometry: { length: Math.max(0.01, parseFloat(el.value || 1.0)) } },
        label: `Change Length of '${node.name}'`,
      });
    });

    bindChange("insp-gplane", (el) => {
      const s = Math.max(1, parseFloat(el.value || 100));
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { geometry: { size: [s, s] } },
        label: `Resize Ground Plane`,
      });
    });

    bindChange("insp-color", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { material: { color: hexToRgba(el.value) } },
        label: `Change Color of '${node.name}'`,
      });
    });

    // Parametric archetype parameter inputs
    document.querySelectorAll(".insp-arch-param").forEach((inputEl) => {
      inputEl.addEventListener("change", () => {
        const key = inputEl.getAttribute("data-param-key");
        const isInt = key === "steps" || key === "shelves" || key === "stripes";
        const val = isInt
          ? Math.max(1, parseInt(inputEl.value || "2", 10))
          : Math.max(0.02, parseFloat(inputEl.value || "0.5"));
        this.sendCommand({
          type: "update_node",
          node_id: node.id,
          patch: { archetype_params: { [key]: val } },
          label: `Update ${key}=${val} on '${node.name}'`,
        });
      });
    });

    const btnInspGroup = document.getElementById("btn-insp-group");
    if (btnInspGroup) {
      btnInspGroup.addEventListener("click", () => {
        document.getElementById("btn-group-selected")?.click();
      });
    }

    const btnInspStack = document.getElementById("btn-insp-stack");
    if (btnInspStack) {
      btnInspStack.addEventListener("click", () => {
        document.getElementById("btn-stack-top")?.click();
      });
    }

    const btnExplode = document.getElementById("btn-explode-composite");
    if (btnExplode) {
      btnExplode.addEventListener("click", () => {
        this.sendCommand({
          type: "explode_composite",
          node_id: node.id,
        });
      });
    }

    const btnSavePrefab = document.getElementById("btn-save-custom-prefab");
    if (btnSavePrefab) {
      btnSavePrefab.addEventListener("click", async () => {
        const customLabel = `⭐ ${node.name}`;
        const res = await fetch("/api/save_custom_object", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ node_id: node.id, label: customLabel }),
        });
        const data = await res.json();
        if (data.ok) {
          this._applyBackendPayload(data);
          this._setStatus(`Guardado '${customLabel}' en Objetos Personalizados`);
        }
      });
    }

    bindChange("insp-mesh-unit", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { mesh: { unit: el.value } },
        label: `Change Mesh Unit to '${el.value}' on '${node.name}'`,
      });
    });

    bindChange("insp-mesh-axis", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { mesh: { up_axis: el.value } },
        label: `Change Mesh Axis to '${el.value}' on '${node.name}'`,
      });
    });

    bindChange("insp-col-strategy", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { collision: { strategy: el.value } },
        label: `Set Collision Strategy '${el.value}' on '${node.name}'`,
      });
    });

    bindChange("insp-inc-uri", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { include: { uri: el.value.trim() } },
        label: `Update Include URI on '${node.name}'`,
      });
    });

    bindChange("insp-mat-preset", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { inertial: { mode: "auto", material_preset: el.value } },
        label: `Set Material Preset '${el.value}' on '${node.name}'`,
      });
    });

    bindChange("insp-mass", (el) => {
      const m = Math.max(0.001, parseFloat(el.value || 1.0));
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { inertial: { mode: "manual_mass", mass: m } },
        label: `Set Mass=${m}kg on '${node.name}'`,
      });
    });

    bindChange("insp-mu", (el) => {
      const mu = Math.max(0, parseFloat(el.value || 0.7));
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { collision: { mu1: mu, mu2: mu } },
        label: `Set Friction=${mu} on '${node.name}'`,
      });
    });

    // Light controls
    bindChange("insp-lcolor", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { light: { diffuse: hexToRgba(el.value) } },
        label: `Change Light Color '${node.name}'`,
      });
    });

    bindChange("insp-lintensity", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { light: { intensity: Math.max(0, parseFloat(el.value || 1.0)) } },
        label: `Change Light Intensity '${node.name}'`,
      });
    });

    bindChange("insp-lshadows", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { light: { cast_shadows: el.checked } },
        label: `Toggle Light Shadows '${node.name}'`,
      });
    });

    ["insp-ldx", "insp-ldy", "insp-ldz"].forEach((id) => {
      bindChange(id, () => {
        const dx = parseFloat(document.getElementById("insp-ldx")?.value || -0.5);
        const dy = parseFloat(document.getElementById("insp-ldy")?.value || 0.1);
        const dz = parseFloat(document.getElementById("insp-ldz")?.value || -0.9);
        this.sendCommand({
          type: "update_node",
          node_id: node.id,
          patch: { light: { direction: [dx, dy, dz] } },
          label: `Update Light Direction '${node.name}'`,
        });
      });
    });

    // Phase 4 Inspector controls (Waypoint, Robot Spawn, Sensor, Actor)
    bindChange("insp-wp-dwell", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { waypoint: { dwell_time_s: Math.max(0, parseFloat(el.value || 0)) } },
      });
    });
    bindChange("insp-rs-model", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { robot_spawn: { robot_model: el.value.trim() || "turtlebot4" } },
      });
    });
    bindChange("insp-rs-ns", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { robot_spawn: { robot_namespace: el.value.trim() || "robot1" } },
      });
    });
    bindChange("insp-sen-topic", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { sensor: { topic: el.value.trim() || "/world_sensors/sensor" } },
      });
    });
    bindChange("insp-sen-rate", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { sensor: { update_rate: Math.max(1, parseFloat(el.value || 15)) } },
      });
    });
    bindChange("insp-sen-fov", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { sensor: { fov_deg: Math.max(10, Math.min(170, parseFloat(el.value || 75))) } },
      });
    });
    bindChange("insp-sen-range", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { sensor: { range_max: Math.max(0.5, parseFloat(el.value || 8.0)) } },
      });
    });
    bindChange("insp-sen-frustum", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { sensor: { show_frustum: el.checked } },
      });
    });
    bindChange("insp-act-speed", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { actor: { speed_mps: Math.max(0.1, parseFloat(el.value || 1.1)) } },
      });
    });
    bindChange("insp-act-loop", (el) => {
      this.sendCommand({
        type: "update_node",
        node_id: node.id,
        patch: { actor: { loop: el.checked } },
      });
    });
  }

  // ===========================================================================
  // 6. DOCTOR GAZEBO DIAGNOSTIC PANEL & LIVE SDF XML SPLIT-VIEW
  // ===========================================================================
  _renderDoctorGazebo() {
    const badge = document.getElementById("btn-doctor-badge");
    const listEl = document.getElementById("doctor-alerts-list");
    listEl.innerHTML = "";

    const errors = this.doctorAlerts.filter((a) => a.severity === "ERROR");
    const warnings = this.doctorAlerts.filter((a) => a.severity === "WARNING");

    if (errors.length === 0 && warnings.length === 0) {
      badge.className = "doctor-badge ok";
      badge.textContent = "✓ Doctor Gazebo: OK";
      listEl.innerHTML = `<div class="doctor-item ok">✓ Todas las inercias, masas y poses son físicamente válidas.</div>`;
      return;
    }

    if (errors.length > 0) {
      badge.className = "doctor-badge err";
      badge.textContent = `⚠ Doctor Gazebo: ${errors.length} Error(es)`;
    } else {
      badge.className = "doctor-badge warn";
      badge.textContent = `⚡ Doctor Gazebo: ${warnings.length} Aviso(s)`;
    }

    for (const alert of this.doctorAlerts) {
      const div = document.createElement("div");
      div.className = `doctor-item ${alert.severity}`;
      div.textContent = alert.message;
      if (alert.node_id) {
        div.addEventListener("click", () => this.selectNode(alert.node_id));
      }
      listEl.appendChild(div);
    }
  }

  _renderLiveSdfXml() {
    const container = document.getElementById("xml-code-container");
    if (!container) return;
    container.innerHTML = "";

    const lines = this.sdfPreview.split("\n");
    const range = this.selectedNodeId ? this.sdfLineMap[this.selectedNodeId] : null;

    lines.forEach((lineText, idx) => {
      const ln = idx + 1;
      const row = document.createElement("div");
      const isHighlighted = range && ln >= range.start_line && ln <= range.end_line;
      row.className = "xml-line" + (isHighlighted ? " highlighted" : "");
      row.dataset.line = String(ln);

      const lnSpan = document.createElement("span");
      lnSpan.className = "xml-ln";
      lnSpan.textContent = String(ln);

      const txtSpan = document.createElement("span");
      txtSpan.className = "xml-text";
      txtSpan.textContent = lineText;

      row.appendChild(lnSpan);
      row.appendChild(txtSpan);
      container.appendChild(row);
    });

    this._highlightSelectedNodeInXml();
  }

  _highlightSelectedNodeInXml() {
    const container = document.getElementById("xml-code-container");
    if (!container) return;
    const range = this.selectedNodeId ? this.sdfLineMap[this.selectedNodeId] : null;

    container.querySelectorAll(".xml-line").forEach((el) => {
      const ln = parseInt(el.dataset.line || "0", 10);
      if (range && ln >= range.start_line && ln <= range.end_line) {
        el.classList.add("highlighted");
      } else {
        el.classList.remove("highlighted");
      }
    });

    if (range) {
      const firstLine = container.querySelector(`.xml-line[data-line="${range.start_line}"]`);
      if (firstLine) {
        firstLine.scrollIntoView({ behavior: "smooth", block: "center" });
      }
    }
  }

  // ===========================================================================
  // 7. TOOLBAR EVENTS & KEYBOARD SHORTCUTS (BLENDER / GAZEBO ERGONOMICS)
  // ===========================================================================
  _bindToolbarAndShortcuts() {
    // Undo / Redo
    document.getElementById("btn-undo").addEventListener("click", () => this.triggerUndo());
    document.getElementById("btn-redo").addEventListener("click", () => this.triggerRedo());

    // Transform Gizmo Modes (G, R, S)
    const setGizmoMode = (mode) => {
      this.transformControls.setMode(mode);
      document
        .getElementById("btn-mode-translate")
        .classList.toggle("active", mode === "translate");
      document.getElementById("btn-mode-rotate").classList.toggle("active", mode === "rotate");
      document.getElementById("btn-mode-scale").classList.toggle("active", mode === "scale");
      this._setStatus(`Modo Gizmo: ${mode.toUpperCase()}`);
    };

    document
      .getElementById("btn-mode-translate")
      .addEventListener("click", () => setGizmoMode("translate"));
    document
      .getElementById("btn-mode-rotate")
      .addEventListener("click", () => setGizmoMode("rotate"));
    document
      .getElementById("btn-mode-scale")
      .addEventListener("click", () => setGizmoMode("scale"));

    // Coordinate Space Toggle (World vs Local)
    document.getElementById("btn-space-toggle").addEventListener("click", (e) => {
      const next = this.transformControls.space === "world" ? "local" : "world";
      this.transformControls.setSpace(next);
      e.currentTarget.textContent = next === "world" ? "🌐 World" : "🧭 Local";
      this._setStatus(`Espacio de transformación: ${next.toUpperCase()}`);
    });

    // Smart Drop to Ground / Surface Below (Space / End)
    const triggerDropToGround = () => {
      if (!this.selectedNodeId) return;
      const selIds = Array.from(this.selectedNodeIds || []);
      this.sendCommand({
        type: "drop_to_ground",
        node_id: this.selectedNodeId,
        node_ids: selIds.length > 1 ? selIds : [this.selectedNodeId],
        ground_z: 0.0,
      });
      this._setStatus("Objeto(s) apoyados sobre superficie inferior o suelo Z = 0 m");
    };
    document.getElementById("btn-drop-ground").addEventListener("click", triggerDropToGround);

    // Surface-to-Surface Snap (V)
    const triggerSnapNeighbor = () => {
      if (!this.selectedNodeId) return;
      this.sendCommand({
        type: "snap_to_neighbor",
        node_id: this.selectedNodeId,
        snap_rotation: true,
      });
      this._setStatus("🧲 Objeto acoplado cara-a-cara o encima del vecino más cercano");
    };
    const btnSnapNeighbor = document.getElementById("btn-snap-neighbor");
    if (btnSnapNeighbor) btnSnapNeighbor.addEventListener("click", triggerSnapNeighbor);

    // Top-Surface Magnet / Stack On Top of Support Below (B)
    const triggerStackOnTop = () => {
      if (!this.selectedNodeId) return;
      const selIds = Array.from(this.selectedNodeIds || []);
      let supportId = null;
      if (selIds.length >= 2) {
        supportId = selIds.find((id) => id !== this.selectedNodeId) || null;
      }
      this.sendCommand({
        type: "stack_on_neighbor",
        node_id: this.selectedNodeId,
        support_node_id: supportId,
      });
      this._setStatus(
        supportId
          ? "⬆️ Objeto apilado justo ENCIMA del objeto de soporte seleccionado"
          : "⬆️ Objeto colocado ENCIMA de la superficie superior del objeto inferior/vecino"
      );
    };
    document.getElementById("btn-stack-top")?.addEventListener("click", triggerStackOnTop);

    // Group & Ungroup Multi-Selected Nodes (Ctrl+G / Ctrl+U)
    const triggerGroupSelected = () => {
      const selIds = Array.from(this.selectedNodeIds || []);
      if (selIds.length < 2) {
        this._setStatus(
          "⚠ Selecciona 2 o más objetos (Shift+Clic o '🔲 Multi / Caja') para agruparlos en 1 bloque"
        );
        return;
      }
      this.sendCommand({
        type: "group_nodes",
        node_ids: selIds,
      });
      this._setStatus(`🔗 ${selIds.length} objetos agrupados en un único bloque compuesto`);
    };

    const triggerUngroupSelected = () => {
      if (!this.selectedNodeId) return;
      this.sendCommand({
        type: "explode_composite",
        node_id: this.selectedNodeId,
      });
      this._setStatus("💥 Grupo compuesto desglosado en piezas individuales");
    };

    document.getElementById("btn-group-selected")?.addEventListener("click", triggerGroupSelected);
    document.getElementById("btn-outliner-group")?.addEventListener("click", triggerGroupSelected);
    document.getElementById("btn-outliner-ungroup")?.addEventListener("click", triggerUngroupSelected);

    // Interactive Viewport Tools + Floating Popups:
    // - "boxselect"   -> Marquee 2D multi-selection box
    // - "ruler"       -> #popup-ruler
    // - "floorplan"   -> #popup-wall-tracer
    // - "spline"      -> #popup-spline-tracer
    // - "scatter_paint"
    // - "waypoint"
    const setViewportTool = (toolName) => {
      this.activeViewportTool = this.activeViewportTool === toolName ? null : toolName;
      this.rulerPoints = [];
      this.floorplanPoints = [];
      this.splinePoints = [];

      document
        .getElementById("btn-tool-boxselect")
        ?.classList.toggle("active", this.activeViewportTool === "boxselect");
      document
        .getElementById("btn-tool-ruler")
        ?.classList.toggle("active", this.activeViewportTool === "ruler");
      document
        .getElementById("btn-tool-floorplan")
        ?.classList.toggle("active", this.activeViewportTool === "floorplan");
      document
        .getElementById("btn-open-wall-tracer-drawer")
        ?.classList.toggle("active", this.activeViewportTool === "floorplan");
      document
        .getElementById("btn-trace-spline-3d")
        ?.classList.toggle("active", this.activeViewportTool === "spline");
      document
        .getElementById("btn-paint-scatter-3d")
        ?.classList.toggle("active", this.activeViewportTool === "scatter_paint");
      document
        .getElementById("btn-tool-waypoint")
        ?.classList.toggle("active", this.activeViewportTool === "waypoint");

      // Show/Hide Floating Popups
      document
        .getElementById("popup-wall-tracer")
        ?.classList.toggle("hidden", this.activeViewportTool !== "floorplan");
      document
        .getElementById("popup-ruler")
        ?.classList.toggle("hidden", this.activeViewportTool !== "ruler");
      document
        .getElementById("popup-spline-tracer")
        ?.classList.toggle("hidden", this.activeViewportTool !== "spline");

      this._updateToolOverlay3D(this.cursorWorldPt);

      if (this.activeViewportTool === "boxselect") {
        this._setStatus(
          "🔲 Multiselección por Caja activa: arrastra un rectángulo en el visor 3D o usa Shift+Clic para seleccionar varios objetos."
        );
      } else if (this.activeViewportTool === "ruler") {
        this._setStatus("📏 Medidor Láser 3D activo: haz clic en 2 puntos del suelo o sobre objetos para medir.");
      } else if (this.activeViewportTool === "floorplan") {
        this._setStatus("🧱 Trazador de Muros 3D activo: haz clic en las esquinas sobre el suelo y pulsa '🧱 Levantar Muros'.");
      } else if (this.activeViewportTool === "spline") {
        this._setStatus("〰 Trazador de Curva Bézier 3D activo: haz clic en los puntos de la curva y pulsa '〰 Clonar en Curva'.");
      } else if (this.activeViewportTool === "scatter_paint") {
        this._setStatus("🎯 Pincel Scatter 3D activo: haz clic en cualquier zona del suelo para dispersar objetos.");
      } else if (this.activeViewportTool === "waypoint") {
        this._setStatus("📍 Modo Waypoint Nav2: haz clic en el suelo 3D para colocar puntos de ruta.");
      } else {
        this._setStatus("Herramienta interactiva cerrada");
      }
    };

    document
      .getElementById("btn-tool-boxselect")
      ?.addEventListener("click", () => setViewportTool("boxselect"));
    document
      .getElementById("btn-tool-ruler")
      ?.addEventListener("click", () => setViewportTool("ruler"));
    document
      .getElementById("btn-tool-floorplan")
      ?.addEventListener("click", () => setViewportTool("floorplan"));
    document
      .getElementById("btn-open-wall-tracer-drawer")
      ?.addEventListener("click", () => setViewportTool("floorplan"));
    document
      .getElementById("btn-trace-spline-3d")
      ?.addEventListener("click", () => setViewportTool("spline"));
    document
      .getElementById("btn-paint-scatter-3d")
      ?.addEventListener("click", () => setViewportTool("scatter_paint"));

    // POPUP 1: Wall Tracer Controls
    ["fp-wall-height", "fp-wall-thick", "fp-wall-type", "fp-wall-closed"].forEach((id) => {
      document.getElementById(id)?.addEventListener("input", () => {
        this._updateToolOverlay3D(this.cursorWorldPt);
      });
      document.getElementById(id)?.addEventListener("change", () => {
        this._updateToolOverlay3D(this.cursorWorldPt);
      });
    });

    document.getElementById("btn-fp-build-now")?.addEventListener("click", () => {
      this._commitFloorplanWalls();
    });

    document.getElementById("btn-fp-undo-pt")?.addEventListener("click", () => {
      if ((this.floorplanPoints || []).length > 0) {
        this.floorplanPoints.pop();
        this._updateToolOverlay3D(this.cursorWorldPt);
        this._setStatus(`↩ Última esquina deshecha (${this.floorplanPoints.length} restantes)`);
      }
    });

    document.getElementById("btn-fp-quick-room")?.addEventListener("click", () => {
      const h = Math.max(0.3, parseFloat(document.getElementById("fp-wall-height")?.value || "2.50"));
      const t = Math.max(0.05, parseFloat(document.getElementById("fp-wall-thick")?.value || "0.20"));
      const wallType = document.getElementById("fp-wall-type")?.value || "wall";
      const roomPts = [
        [-4.0, -3.0],
        [4.0, -3.0],
        [4.0, 3.0],
        [-4.0, 3.0],
      ];
      this.sendCommand({
        type: "extrude_floorplan",
        points: roomPts,
        height: h,
        thickness: t,
        wall_type: wallType,
        closed: true,
      });
      this._setStatus(`🧱 Recinto rápido 8×6 m levantado con altura ${h.toFixed(2)} m`);
    });

    document.getElementById("btn-close-wall-popup")?.addEventListener("click", () => {
      if (this.activeViewportTool === "floorplan") setViewportTool("floorplan");
    });

    // POPUP 2: 3D Laser Ruler Controls
    document.getElementById("btn-ruler-reset")?.addEventListener("click", () => {
      this.rulerPoints = [];
      const distEl = document.getElementById("ruler-dist-main");
      const deltasEl = document.getElementById("ruler-deltas");
      const anglesEl = document.getElementById("ruler-angles");
      if (distEl) distEl.textContent = "0.000 m";
      if (deltasEl) deltasEl.textContent = "ΔX: 0.00 m · ΔY: 0.00 m · ΔZ: 0.00 m";
      if (anglesEl) anglesEl.textContent = "Yaw ENU: 0.0° (0.000 rad)";
      this._updateToolOverlay3D(this.cursorWorldPt);
      this._setStatus("📏 Medidor reiniciado. Haz clic en el Punto 1.");
    });

    document.getElementById("btn-ruler-done")?.addEventListener("click", () => {
      if (this.activeViewportTool === "ruler") setViewportTool("ruler");
    });
    document.getElementById("btn-close-ruler-popup")?.addEventListener("click", () => {
      if (this.activeViewportTool === "ruler") setViewportTool("ruler");
    });

    // POPUP 4: 3D Spline Tracer Controls
    document.getElementById("btn-spline-build-now")?.addEventListener("click", () => {
      this._commitSplineClones();
    });
    document.getElementById("btn-spline-clear-pts")?.addEventListener("click", () => {
      this.splinePoints = [];
      this._updateToolOverlay3D(this.cursorWorldPt);
    });
    document.getElementById("btn-close-spline-popup")?.addEventListener("click", () => {
      if (this.activeViewportTool === "spline") setViewportTool("spline");
    });

    // Grid & Angle Snapping
    document.getElementById("grid-snap-select").addEventListener("change", (e) => {
      const val = parseFloat(e.target.value);
      this.transformControls.setTranslationSnap(val > 0 ? val : null);
    });
    document.getElementById("angle-snap-select").addEventListener("change", (e) => {
      const deg = parseFloat(e.target.value);
      this.transformControls.setRotationSnap(deg > 0 ? deg * DEG2RAD : null);
    });

    // Camera Perspective vs Orthographic Top-Down 2D (Numpad 7)
    const toggleCameraView = () => {
      const btn = document.getElementById("btn-cam-view");
      if (this.cameraMode === "perspective") {
        this.cameraMode = "ortho_top";
        this.activeCamera = this.orthoCamera;
        this.orthoCamera.position.set(0, 0, 50);
        this.orthoCamera.lookAt(0, 0, 0);
        this.orbitControls.object = this.orthoCamera;
        this.orbitControls.enableRotate = false;
        this.transformControls.camera = this.orthoCamera;
        btn.textContent = "📐 2D Cenital";
        btn.classList.add("active");
        this._setStatus("Vista Ortográfica Cenital 2D (Top-Down XY)");
      } else {
        this.cameraMode = "perspective";
        this.activeCamera = this.perspCamera;
        this.orbitControls.object = this.perspCamera;
        this.orbitControls.enableRotate = true;
        this.transformControls.camera = this.perspCamera;
        btn.textContent = "🎥 3D Persp";
        btn.classList.remove("active");
        this._setStatus("Vista en Perspectiva 3D (Z-Up)");
      }
      this.orbitControls.update();
    };
    document.getElementById("btn-cam-view").addEventListener("click", toggleCameraView);

    // Visual Debug Layers
    document.getElementById("btn-debug-col").addEventListener("click", (e) => {
      this.debugLayers.collision = !this.debugLayers.collision;
      e.currentTarget.classList.toggle("active", this.debugLayers.collision);
      this._rebuild3DSceneFromGraph();
    });
    document.getElementById("btn-debug-inertia").addEventListener("click", (e) => {
      this.debugLayers.inertia = !this.debugLayers.inertia;
      e.currentTarget.classList.toggle("active", this.debugLayers.inertia);
      this._rebuild3DSceneFromGraph();
    });
    document.getElementById("btn-debug-tf").addEventListener("click", (e) => {
      this.debugLayers.tf = !this.debugLayers.tf;
      e.currentTarget.classList.toggle("active", this.debugLayers.tf);
      this._rebuild3DSceneFromGraph();
    });

    // World Name & SDF Target Version
    document.getElementById("world-name-input").addEventListener("change", (e) => {
      this.sendCommand({
        type: "update_world",
        patch: { name: e.target.value.trim() || "custebo_world" },
      });
    });
    document.getElementById("sdf-target-select").addEventListener("change", (e) => {
      this.sendCommand({
        type: "update_world",
        patch: { sdf_target: e.target.value },
      });
    });

    // Angle unit display toggle (Degrees vs Radians)
    document.getElementById("chk-angles-deg").addEventListener("change", (e) => {
      this.showAnglesInDegrees = e.target.checked;
      this._renderInspector();
    });

    // Creation Palette Buttons
    document.querySelectorAll("[data-add-archetype]").forEach((btn) => {
      btn.addEventListener("click", () => {
        const archetype = btn.getAttribute("data-add-archetype");
        this.sendCommand({ type: "add_archetype", archetype });
      });
    });
    document.querySelectorAll("[data-add-shape]").forEach((btn) => {
      btn.addEventListener("click", () => {
        const shape = btn.getAttribute("data-add-shape");
        this.sendCommand({ type: "add_primitive", shape, name: shape });
      });
    });
    document.querySelectorAll("[data-add-light]").forEach((btn) => {
      btn.addEventListener("click", () => {
        const lightType = btn.getAttribute("data-add-light");
        this.sendCommand({ type: "add_light", light_type: lightType });
      });
    });

    // Duplicate & Delete Selected (Single or Multi-Selection)
    const duplicateSelected = () => {
      const selIds = Array.from(this.selectedNodeIds || []);
      if (selIds.length > 1) {
        this.sendCommand({ type: "duplicate_nodes", node_ids: selIds });
      } else if (this.selectedNodeId) {
        this.sendCommand({ type: "duplicate_node", node_id: this.selectedNodeId });
      }
    };
    const deleteSelected = () => {
      const selIds = Array.from(this.selectedNodeIds || []);
      if (selIds.length > 0) {
        this.sendCommand({ type: "delete_nodes", node_ids: selIds });
      } else if (this.selectedNodeId) {
        this.sendCommand({ type: "delete_nodes", node_ids: [this.selectedNodeId] });
      }
    };
    document.getElementById("btn-dup-selected").addEventListener("click", duplicateSelected);
    document.getElementById("btn-del-selected").addEventListener("click", deleteSelected);

    // Live SDF XML Split-View Toggle & Actions
    document.getElementById("btn-toggle-xml").addEventListener("click", (e) => {
      const panel = document.getElementById("xml-split-panel");
      panel.classList.toggle("hidden");
      e.currentTarget.classList.toggle("active", !panel.classList.contains("hidden"));
      setTimeout(() => this._onResize(), 50);
    });

    document.getElementById("btn-copy-sdf").addEventListener("click", () => {
      navigator.clipboard.writeText(this.sdfPreview);
      this._setStatus("Código XML/SDF copiado al portapapeles");
    });

    document.getElementById("btn-verify-sdf").addEventListener("click", async () => {
      this._setStatus("Verificando SDF con validador CLI y `gz sdf -k`...");
      const res = await fetch("/api/validate_sdf", { method: "POST" });
      const data = await res.json();
      if (data.is_valid) {
        const gzNote = data.gz_cli_checked ? " + `gz sdf -k` oficial" : "";
        this._setStatus(`✓ SDF 100% VÁLIDO (Validador Custebo${gzNote})`);
      } else {
        this._setStatus(`⚠ Errores en SDF: ${data.issues.map((i) => i.message).join("; ")}`);
      }
    });

    // Save Project (.gproj.json)
    document.getElementById("btn-save-proj").addEventListener("click", async () => {
      const res = await fetch("/api/save_project", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({}),
      });
      const data = await res.json();
      if (data.ok) {
        // Also trigger browser download for convenience
        const blob = new Blob([JSON.stringify(data.scene_graph, null, 2)], {
          type: "application/json",
        });
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = `${this.sceneGraph.world.name || "custebo_world"}.gproj.json`;
        a.click();
        URL.revokeObjectURL(url);
        this._setStatus(`Proyecto guardado en ${data.saved_path}`);
      }
    });

    // Load Project (.gproj.json)
    const fileInput = document.getElementById("file-load-input");
    document.getElementById("btn-load-proj").addEventListener("click", () => fileInput.click());
    fileInput.addEventListener("change", async (e) => {
      const file = e.target.files?.[0];
      if (!file) return;
      const text = await file.text();
      const parsed = JSON.parse(text);
      const res = await fetch("/api/load_project", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ scene_graph: parsed }),
      });
      const data = await res.json();
      if (data.ok) {
        this._applyBackendPayload(data);
        this._setStatus(`Proyecto '${file.name}' cargado correctamente`);
      }
    });

    // Round-Trip Import .SDF / .WORLD
    const sdfInput = document.getElementById("file-sdf-input");
    document.getElementById("btn-import-sdf").addEventListener("click", () => sdfInput.click());
    sdfInput.addEventListener("change", async (e) => {
      const file = e.target.files?.[0];
      if (!file) return;
      const xmlText = await file.text();
      const res = await fetch("/api/import_sdf", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ xml_content: xmlText }),
      });
      const data = await res.json();
      if (data.ok) {
        this._applyBackendPayload(data);
        this._setStatus(`Mundo SDF '${file.name}' importado y limpiado con éxito`);
      } else {
        this._setStatus(`Error al importar SDF: ${data.error}`);
      }
    });

    // Custom 3D Mesh Upload Assistant ("➕ Importar 3D" in Objetos Personalizados)
    const meshDrawer = document.getElementById("mesh-import-drawer");
    document.getElementById("btn-open-mesh-import").addEventListener("click", () => {
      meshDrawer.classList.toggle("hidden");
    });

    const meshFileInput = document.getElementById("file-mesh-input");
    document.getElementById("btn-pick-mesh-file").addEventListener("click", () => {
      meshFileInput.click();
    });

    meshFileInput.addEventListener("change", async (e) => {
      const file = e.target.files?.[0];
      if (!file) return;
      const buf = await file.arrayBuffer();
      const bytes = new Uint8Array(buf);
      let binary = "";
      for (let i = 0; i < bytes.byteLength; i++) {
        binary += String.fromCharCode(bytes[i]);
      }
      const b64 = btoa(binary);
      const unit = document.getElementById("import-mesh-unit").value;
      const upAxis = document.getElementById("import-mesh-axis").value;

      this._setStatus(`Importando y calculando inercia de '${file.name}'...`);
      const res = await fetch("/api/upload_custom_mesh", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          filename: file.name,
          content_base64: b64,
          unit,
          up_axis: upAxis,
          material_preset: "aluminum",
        }),
      });
      const data = await res.json();
      if (data.ok) {
        if (data.selected_node_id) this.selectedNodeId = data.selected_node_id;
        this._applyBackendPayload(data);
        meshDrawer.classList.add("hidden");
        this._setStatus(`Malla '${file.name}' añadida a Objetos Personalizados e insertada en escena`);
      } else {
        this._setStatus(`Error importando malla: ${data.error}`);
      }
    });

    // Gazebo Fuel & ROS 2 Workspace Browser Drawer
    const fuelDrawer = document.getElementById("fuel-browser-drawer");
    const runFuelSearch = async () => {
      const q = document.getElementById("fuel-search-input").value.trim();
      const listEl = document.getElementById("fuel-results-list");
      listEl.innerHTML = `<div style="font-size:11px;color:var(--text-muted);">Buscando modelos 3D...</div>`;
      const res = await fetch(`/api/fuel_search?q=${encodeURIComponent(q)}`);
      const data = await res.json();
      if (!data.ok) return;
      listEl.innerHTML = "";
      for (const m of data.fuel_models || []) {
        const row = document.createElement("div");
        row.className = "custom-obj-row";
        row.innerHTML = `<span class="custom-obj-title">☁ ${m.name}</span>`;
        row.title = `URI: ${m.uri}`;
        row.addEventListener("click", () => {
          this.sendCommand({
            type: "add_fuel_include",
            name: m.model_name.replace(/\s+/g, "_").toLowerCase(),
            uri: m.uri,
            size: m.size,
            color: m.color,
            static: m.static,
            proxy_archetype: m.proxy_archetype,
          });
          this._setStatus(`☁ Modelo 3D de Gazebo Fuel añadido: ${m.model_name}`);
        });
        listEl.appendChild(row);
      }
      for (const rm of (data.ros_models || []).slice(0, 12)) {
        const row = document.createElement("div");
        row.className = "custom-obj-row";
        row.innerHTML = `<span class="custom-obj-title">📦 [ROS 2] ${rm.name}</span>`;
        row.title = `${rm.uri} (${rm.source_path})`;
        row.addEventListener("click", () => {
          this.sendCommand({
            type: "add_fuel_include",
            name: rm.name.replace(/\s+/g, "_").toLowerCase(),
            uri: rm.uri,
            size: [1.0, 1.0, 1.0],
            color: [0.25, 0.70, 0.55, 1.0],
            static: true,
            proxy_archetype: rm.proxy_archetype || "",
            source_path: rm.source_path || "",
          });
          this._setStatus(`📦 Modelo local ROS 2 importado con geometría 3D: ${rm.uri}`);
        });
        listEl.appendChild(row);
      }
    };

    document.getElementById("btn-toggle-fuel").addEventListener("click", () => {
      fuelDrawer.classList.toggle("hidden");
      if (!fuelDrawer.classList.contains("hidden")) {
        runFuelSearch();
      }
    });
    document.getElementById("btn-fuel-search-go").addEventListener("click", runFuelSearch);

    // Phase 3: Procedural & CAD Tools Drawer
    const procDrawer = document.getElementById("procedural-drawer");
    document.getElementById("btn-toggle-procedural")?.addEventListener("click", () => {
      procDrawer?.classList.toggle("hidden");
    });

    // 1. Grid Array Cloner
    document.getElementById("btn-run-grid-array")?.addEventListener("click", () => {
      if (!this.selectedNodeId) {
        this._setStatus("⚠ Selecciona primero un objeto en el visor 3D para clonarlo en matriz");
        return;
      }
      const nx = parseInt(document.getElementById("arr-nx")?.value || "3", 10);
      const ny = parseInt(document.getElementById("arr-ny")?.value || "2", 10);
      const nz = parseInt(document.getElementById("arr-nz")?.value || "1", 10);
      const sx = parseFloat(document.getElementById("arr-sx")?.value || "2.5");
      const sy = parseFloat(document.getElementById("arr-sy")?.value || "2.5");
      const sz = parseFloat(document.getElementById("arr-sz")?.value || "2.2");
      this.sendCommand({
        type: "grid_array_clone",
        node_id: this.selectedNodeId,
        count_x: nx,
        count_y: ny,
        count_z: nz,
        spacing_x: sx,
        spacing_y: sy,
        spacing_z: sz,
      });
      this._setStatus(`🔢 Creada matriz procedural ${nx}×${ny}×${nz} con paso [${sx}, ${sy}, ${sz}] m`);
    });

    // 2. Spline / Bezier Cloner (Instant S-Curve or Traced 3D Curve)
    document.getElementById("btn-run-spline")?.addEventListener("click", () => {
      const count = parseInt(document.getElementById("spline-count")?.value || "7", 10);
      const radius = parseFloat(document.getElementById("spline-radius")?.value || "8.0");
      const alignTan = Boolean(document.getElementById("spline-tangent")?.checked ?? true);
      const node = this._getSelectedNode();
      const [x0, y0] = node ? node.pose : [-radius * 0.5, 0];
      const controlPoints =
        (this.splinePoints || []).length >= 2
          ? [...this.splinePoints]
          : [
              [x0, y0],
              [x0 + radius * 0.35, y0 + radius * 0.55],
              [x0 + radius * 0.75, y0 - radius * 0.55],
              [x0 + radius * 1.15, y0],
            ];
      this.sendCommand({
        type: "spline_clone",
        node_id: this.selectedNodeId || "",
        control_points: controlPoints,
        count,
        align_tangent: alignTan,
      });
      this._setStatus(`〰 Creados ${count} clones orientados a lo largo de curva Bézier`);
    });

    // 3. Scatter Brush (Random distribution with AABB anti-overlap)
    document.getElementById("btn-run-scatter")?.addEventListener("click", () => {
      const archSel = document.getElementById("scatter-arch")?.value || "cardboard_box";
      const useSelected = archSel === "__selected__" && this.selectedNodeId;
      const count = parseInt(document.getElementById("scatter-count")?.value || "10", 10);
      const radius = parseFloat(document.getElementById("scatter-radius")?.value || "6.0");
      const minDist = parseFloat(document.getElementById("scatter-mindist")?.value || "0.9");
      const centerNode = this._getSelectedNode();
      const cx = centerNode && centerNode.type !== "ground_plane" ? centerNode.pose[0] : 0.0;
      const cy = centerNode && centerNode.type !== "ground_plane" ? centerNode.pose[1] : 0.0;
      this.sendCommand({
        type: "scatter_brush",
        node_id: useSelected ? this.selectedNodeId : "",
        archetype: useSelected ? "cardboard_box" : archSel,
        center_x: cx,
        center_y: cy,
        radius,
        count,
        min_distance: minDist,
        seed: Math.floor(Math.random() * 100000),
      });
      this._setStatus(`🎲 Dispersados ${count} objetos sin solapamientos en radio ${radius} m`);
    });

    // 4. 2D Blueprint Overlay Image & ROS 2 .PGM -> 3D Wall Reconstruction Popup
    const fpImgInput = document.getElementById("file-floorplan-img");
    document.getElementById("btn-load-floorplan-img")?.addEventListener("click", () => {
      fpImgInput?.click();
    });
    fpImgInput?.addEventListener("change", (e) => {
      const file = e.target.files?.[0];
      if (!file) return;
      const planeSize = Math.max(2, parseFloat(document.getElementById("floorplan-size-m")?.value || "24"));
      const url = URL.createObjectURL(file);
      new THREE.TextureLoader().load(url, (tex) => {
        if (this.blueprintMesh) {
          this.scene.remove(this.blueprintMesh);
        }
        const geo = new THREE.PlaneGeometry(planeSize, planeSize);
        const mat = new THREE.MeshBasicMaterial({
          map: tex,
          transparent: true,
          opacity: 0.62,
          depthWrite: false,
        });
        this.blueprintMesh = new THREE.Mesh(geo, mat);
        this.blueprintMesh.position.set(0, 0, 0.008);
        this.scene.add(this.blueprintMesh);
        this._setStatus(
          `🖼 Plano '${file.name}' superpuesto en Z=0 (${planeSize}×${planeSize} m). Pulsa '✏ Trazar Muros 3D' para calcarlo.`
        );
      });
    });

    document.getElementById("btn-clear-floorplan-img")?.addEventListener("click", () => {
      if (this.blueprintMesh) {
        this.scene.remove(this.blueprintMesh);
        this.blueprintMesh = null;
        this._setStatus("🗑 Imagen de plano 2D retirada del suelo");
      }
    });

    // POPUP 3: PGM SLAM Map -> 3D Walls Assistant
    const pgmPopup = document.getElementById("popup-pgm-walls");
    const pgmInput = document.getElementById("file-pgm-input");
    document.getElementById("btn-import-pgm-walls")?.addEventListener("click", () => {
      pgmPopup?.classList.toggle("hidden");
    });
    document.getElementById("btn-close-pgm-popup")?.addEventListener("click", () => {
      pgmPopup?.classList.add("hidden");
    });
    document.getElementById("btn-pgm-pick-file")?.addEventListener("click", () => {
      pgmInput?.click();
    });
    document.getElementById("btn-pgm-demo-map")?.addEventListener("click", async () => {
      const wh = Math.max(0.5, parseFloat(document.getElementById("pgm-wall-height")?.value || "2.50"));
      const resVal = Math.max(0.01, parseFloat(document.getElementById("pgm-wall-res")?.value || "0.05"));
      const res = await fetch("/api/import_pgm_walls", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          use_demo_slam: true,
          wall_height: wh,
          resolution: resVal,
        }),
      });
      const data = await res.json();
      if (data.ok) {
        this._applyBackendPayload(data);
        pgmPopup?.classList.add("hidden");
        this._setStatus(`🗺 Mapa Demo SLAM reconstruido en ${data.walls_created} muros 3D (altura ${wh.toFixed(2)} m)`);
      } else {
        this._setStatus(`Error en Mapa Demo SLAM: ${data.error}`);
      }
    });

    pgmInput?.addEventListener("change", async (e) => {
      const file = e.target.files?.[0];
      if (!file) return;
      const buf = await file.arrayBuffer();
      const bytes = new Uint8Array(buf);
      let binary = "";
      for (let i = 0; i < bytes.byteLength; i++) {
        binary += String.fromCharCode(bytes[i]);
      }
      const wh = Math.max(0.5, parseFloat(document.getElementById("pgm-wall-height")?.value || "2.50"));
      const resVal = Math.max(0.01, parseFloat(document.getElementById("pgm-wall-res")?.value || "0.05"));
      const res = await fetch("/api/import_pgm_walls", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          pgm_base64: btoa(binary),
          resolution: resVal,
          wall_height: wh,
        }),
      });
      const data = await res.json();
      if (data.ok) {
        this._applyBackendPayload(data);
        pgmPopup?.classList.add("hidden");
        this._setStatus(`🗺 Mapa ROS '${file.name}' reconstruido en ${data.walls_created} muros 3D (altura ${wh.toFixed(2)} m)`);
      } else {
        this._setStatus(`Error importando PGM: ${data.error}`);
      }
    });

    // 5. Parametric Smooth 3D Outdoor Terrain Generator
    document.getElementById("btn-gen-terrain")?.addEventListener("click", () => {
      const preset = document.getElementById("terrain-preset")?.value || "hills";
      const size = parseFloat(document.getElementById("terrain-size")?.value || "18");
      const amp = parseFloat(document.getElementById("terrain-amp")?.value || "1.5");
      this.sendCommand({
        type: "generate_terrain",
        preset,
        width: size,
        depth: size,
        amplitude: amp,
        grid_res: 24,
      });
      this._setStatus(`⛰ Malla de terreno 3D suave '${preset}' (${size}×${size} m, Zmax=${amp} m) generada`);
    });

    // 6. Multi-Object Alignment & Equidistant Distribution (X / Y)
    const triggerCadAlignOrDistribute = (mode) => {
      if (!this.sceneGraph) return;
      const selIds = Array.from(this.selectedNodeIds || []);
      const movableIds =
        selIds.length >= 2
          ? selIds
          : this.sceneGraph.nodes
              .filter(
                (n) => n.type !== "ground_plane" && n.type !== "light" && !n.locked && n.visible !== false
              )
              .map((n) => n.id);
      if (movableIds.length < 2) {
        this._setStatus("⚠ Se necesitan al menos 2 objetos seleccionados o visibles en la escena");
        return;
      }
      this.sendCommand({
        type: "align_distribute",
        node_ids: movableIds,
        mode,
        spacing: 2.0,
      });
      this._setStatus(`📐 Operación CAD '${mode.toUpperCase()}' aplicada sobre ${movableIds.length} objetos`);
    };
    document.getElementById("btn-align-x")?.addEventListener("click", () => triggerCadAlignOrDistribute("align_x"));
    document.getElementById("btn-align-y")?.addEventListener("click", () => triggerCadAlignOrDistribute("align_y"));
    document
      .getElementById("btn-distribute-x")
      ?.addEventListener("click", () => triggerCadAlignOrDistribute("distribute_x"));
    document
      .getElementById("btn-distribute-y")
      ?.addEventListener("click", () => triggerCadAlignOrDistribute("distribute_y"));

    // =========================================================================
    // Phase 4: ROS 2, Nav2, Sensors, Dynamic Actors & Parametric XACRO
    // =========================================================================
    const ros2Drawer = document.getElementById("ros2-drawer");
    document.getElementById("btn-toggle-ros2")?.addEventListener("click", () => {
      ros2Drawer?.classList.toggle("hidden");
    });

    // 4.1 Instant 2D Occupancy Map Generator (map.pgm + map.yaml) + Live Canvas Preview
    const generateAndRenderNav2Map = async (triggerDownload = false) => {
      const zMin = parseFloat(document.getElementById("nav2-zmin")?.value || "0.15");
      const zMax = parseFloat(document.getElementById("nav2-zmax")?.value || "0.60");
      const resVal = parseFloat(document.getElementById("nav2-res")?.value || "0.05");

      this._setStatus(`🗺 Cortando mundo en Z=[${zMin}, ${zMax}] m (${resVal} m/px)...`);
      const res = await fetch("/api/generate_nav2_map", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ z_min: zMin, z_max: zMax, resolution: resVal, padding_m: 2.0 }),
      });
      const data = await res.json();
      if (!data.ok) {
        this._setStatus(`Error generando mapa 2D: ${data.error}`);
        return;
      }

      // Decode PGM P5 binary to render on #nav2-map-canvas
      const rawBin = atob(data.pgm_base64);
      const bytes = new Uint8Array(rawBin.length);
      for (let i = 0; i < rawBin.length; i++) bytes[i] = rawBin.charCodeAt(i);

      const pixelOffset = bytes.length - data.width * data.height;
      const canvas = document.getElementById("nav2-map-canvas");
      const previewBox = document.getElementById("nav2-map-preview-box");
      const metaEl = document.getElementById("nav2-map-meta");
      if (canvas && pixelOffset >= 0) {
        previewBox?.classList.remove("hidden");
        canvas.width = data.width;
        canvas.height = data.height;
        const ctx = canvas.getContext("2d");
        const imgData = ctx.createImageData(data.width, data.height);
        for (let i = 0; i < data.width * data.height; i++) {
          const val = bytes[pixelOffset + i];
          imgData.data[i * 4 + 0] = val;
          imgData.data[i * 4 + 1] = val;
          imgData.data[i * 4 + 2] = val;
          imgData.data[i * 4 + 3] = 255;
        }
        ctx.putImageData(imgData, 0, 0);
        if (metaEl) {
          metaEl.textContent = `${data.width}×${data.height} px | ${data.resolution} m/px | origen: [${data.origin[0]}, ${data.origin[1]}]`;
        }
      }

      if (triggerDownload) {
        // Download map.pgm
        const pgmBlob = new Blob([bytes], { type: "image/x-portable-graymap" });
        const pgmUrl = URL.createObjectURL(pgmBlob);
        const a1 = document.createElement("a");
        a1.href = pgmUrl;
        a1.download = data.pgm_filename || "map.pgm";
        a1.click();
        URL.revokeObjectURL(pgmUrl);

        // Download map.yaml
        const yamlBlob = new Blob([data.yaml_content], { type: "text/yaml" });
        const yamlUrl = URL.createObjectURL(yamlBlob);
        const a2 = document.createElement("a");
        a2.href = yamlUrl;
        a2.download = data.yaml_filename || "map.yaml";
        a2.click();
        URL.revokeObjectURL(yamlUrl);
        this._setStatus(`✓ Exportados ${data.pgm_filename} y ${data.yaml_filename} en exports/`);
      } else {
        this._setStatus(
          `✓ Mapa 2D Nav2 generado (${data.width}×${data.height} px, ${data.obstacle_count} obstáculos en Z=[${zMin}, ${zMax}] m)`
        );
      }
    };

    document
      .getElementById("btn-gen-nav2-map")
      ?.addEventListener("click", () => generateAndRenderNav2Map(false));
    document
      .getElementById("btn-dl-nav2-map")
      ?.addEventListener("click", () => generateAndRenderNav2Map(true));

    // 4.2 Nav2 Waypoints & Topological Route Graph
    document.getElementById("btn-tool-waypoint")?.addEventListener("click", () => {
      setViewportTool("waypoint");
    });

    document.getElementById("btn-add-waypoint")?.addEventListener("click", () => {
      this.sendCommand({ type: "add_waypoint" });
      this._setStatus("📍 Waypoint Nav2 añadido a la ruta");
    });

    document.getElementById("btn-export-waypoints")?.addEventListener("click", async () => {
      const res = await fetch("/api/export_nav2_waypoints", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({}),
      });
      const data = await res.json();
      if (data.ok) {
        const blob = new Blob([data.yaml_content], { type: "text/yaml" });
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = data.filename || "nav2_waypoints.yaml";
        a.click();
        URL.revokeObjectURL(url);
        this._setStatus(`✓ Exportado ${data.exported_path}`);
      }
    });

    // 4.3 Robot Spawn Poses & Static Sensors (RGB-D / LiDAR) + ros_gz_bridge
    document.getElementById("btn-add-robot-spawn")?.addEventListener("click", () => {
      this.sendCommand({ type: "add_robot_spawn", robot_model: "turtlebot4" });
      this._setStatus("🤖 Marcador de Spawn de Robot añadido");
    });

    document.getElementById("btn-add-sensor-rgbd")?.addEventListener("click", () => {
      this.sendCommand({ type: "add_sensor", sensor_type: "camera_rgbd" });
      this._setStatus("📷 Cámara RGB-D estática con Frustum 3D añadida");
    });

    document.getElementById("btn-add-sensor-lidar3d")?.addEventListener("click", () => {
      this.sendCommand({ type: "add_sensor", sensor_type: "lidar_3d" });
      this._setStatus("📡 Sensor LiDAR 3D estático añadido");
    });

    document.getElementById("btn-add-sensor-lidar2d")?.addEventListener("click", () => {
      this.sendCommand({ type: "add_sensor", sensor_type: "lidar_2d" });
      this._setStatus("📡 Sensor LiDAR 2D planar añadido");
    });

    document.getElementById("btn-export-bridge")?.addEventListener("click", async () => {
      const res = await fetch("/api/export_ros_gz_bridge", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({}),
      });
      const data = await res.json();
      if (data.ok) {
        const blob = new Blob([data.yaml_content], { type: "text/yaml" });
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = data.filename || "ros_gz_bridge.yaml";
        a.click();
        URL.revokeObjectURL(url);
        this._setStatus(`✓ Configuración ros_gz_bridge exportada en ${data.exported_path}`);
      }
    });

    // 4.4 Dynamic Actor (<actor>) & World Plugins
    document.getElementById("btn-add-actor")?.addEventListener("click", () => {
      this.sendCommand({ type: "add_actor", speed_mps: 1.1, loop: true });
      this._setStatus("🚶 Peatón dinámico (<actor>) con trayectoria en bucle añadido");
    });

    document.querySelectorAll(".world-plugin-chk").forEach((chk) => {
      chk.addEventListener("change", () => {
        const pluginKey = chk.getAttribute("data-plugin");
        const curPlugins = { ...(this.sceneGraph?.world?.plugins || {}) };
        curPlugins[pluginKey] = chk.checked;
        this.sendCommand({
          type: "update_world",
          patch: { plugins: curPlugins },
        });
        this._setStatus(`Plugin Gazebo '${pluginKey}' ${chk.checked ? "activado" : "desactivado"}`);
      });
    });

    // 4.5 Export Parametric .sdf.xacro
    document.getElementById("btn-export-xacro")?.addEventListener("click", async () => {
      const res = await fetch("/api/export_xacro", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({}),
      });
      const data = await res.json();
      if (data.ok) {
        const blob = new Blob([data.xacro_xml], { type: "application/xml" });
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = data.filename || "custebo_world.sdf.xacro";
        a.click();
        URL.revokeObjectURL(url);
        this._setStatus(`🧬 Exportado archivo paramétrico en ${data.exported_path}`);
      }
    });

    // Export Clean SDF
    document.getElementById("btn-export-sdf").addEventListener("click", async () => {
      const res = await fetch("/api/export_sdf", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({}),
      });
      const data = await res.json();
      if (data.ok) {
        const blob = new Blob([data.sdf_xml], { type: "application/xml" });
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = `${this.sceneGraph.world.name || "custebo_world"}.sdf`;
        a.click();
        URL.revokeObjectURL(url);
        const vNote = data.validation?.gz_cli_checked ? " (Verificado con `gz sdf -k`)" : "";
        this._setStatus(`Exportado a ${data.exported_path}${vNote}`);
      }
    });

    // Play in Gazebo (`gz sim`)
    document.getElementById("btn-play-gz").addEventListener("click", async () => {
      const res = await fetch("/api/play_gazebo", { method: "POST" });
      const data = await res.json();
      if (data.ok) {
        this._setStatus(`Lanzado Gazebo Harmonic con ${data.launched_file}`);
      } else {
        this._setStatus(`Aviso: ${data.error}`);
      }
    });

    // Global Keyboard Shortcuts
    window.addEventListener("keydown", (e) => {
      const activeTag = document.activeElement?.tagName?.toLowerCase();
      if (activeTag === "input" || activeTag === "select" || activeTag === "textarea") {
        return;
      }

      if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "z") {
        e.preventDefault();
        if (e.shiftKey) this.triggerRedo();
        else this.triggerUndo();
        return;
      }
      if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "y") {
        e.preventDefault();
        this.triggerRedo();
        return;
      }
      if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "d") {
        e.preventDefault();
        duplicateSelected();
        return;
      }
      if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "g") {
        e.preventDefault();
        triggerGroupSelected();
        return;
      }
      if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "u") {
        e.preventDefault();
        triggerUngroupSelected();
        return;
      }

      switch (e.key.toLowerCase()) {
        case "g":
          setGizmoMode("translate");
          break;
        case "r":
          setGizmoMode("rotate");
          break;
        case "s":
          setGizmoMode("scale");
          break;
        case "x":
          document.getElementById("btn-space-toggle").click();
          break;
        case "v":
          triggerSnapNeighbor();
          break;
        case "b":
          triggerStackOnTop();
          break;
        case "m":
          setViewportTool("ruler");
          break;
        case "enter":
          if (this.activeViewportTool === "floorplan") {
            e.preventDefault();
            this._commitFloorplanWalls();
          } else if (this.activeViewportTool === "spline") {
            e.preventDefault();
            this._commitSplineClones();
          }
          break;
        case "escape":
          if (this.activeViewportTool) {
            setViewportTool(this.activeViewportTool);
          }
          break;
        case " ":
        case "end":
          e.preventDefault();
          triggerDropToGround();
          break;
        case "delete":
        case "backspace":
          e.preventDefault();
          deleteSelected();
          break;
        case "7":
          toggleCameraView();
          break;
      }
    });
  }

  _setStatus(msg) {
    const el = document.getElementById("hud-status-msg");
    if (el) el.textContent = msg;
  }
}

window.addEventListener("DOMContentLoaded", () => {
  window.custeboApp = new CusteboStudioApp();
});

