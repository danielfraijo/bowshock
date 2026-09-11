import { useEffect, useRef } from "react";
import type * as ThreeNS from "three";
import type { BuiltVehicle, ColorMode } from "@/lib/waverider/types";
import { SURFACE_ID } from "@/lib/waverider/types";
import { gridPoint } from "@/lib/waverider/mesh";
import type { StudyResult } from "@/lib/waverider/study";

const SURFACE_COLOR: Record<number, number> = {
  [SURFACE_ID.upper]: 0xc5d0d8,
  [SURFACE_ID.lower]: 0x6d7c86,
  [SURFACE_ID.base]: 0x3a4248,
  [SURFACE_ID.leading]: 0x9aab8a,
  [SURFACE_ID.symmetry]: 0x5b6a74,
  [SURFACE_ID.inlet]: 0xb8a58a,
  [SURFACE_ID.nozzle]: 0x6a5b4a,
  [SURFACE_ID.cowl]: 0x8a969e,
};

type Three = typeof ThreeNS;

function heatColor(c: ThreeNS.Color, t: number) {
  const u = Math.max(0, Math.min(1, t));
  if (u < 0.5) c.setRGB(0.45 + u, 0.5 + u * 0.2, 0.55 - u * 0.3);
  else c.setRGB(0.7 + (u - 0.5) * 0.4, 0.45 - (u - 0.5) * 0.2, 0.38 - (u - 0.5) * 0.15);
}

function geomFrom(THREE: Three, built: BuiltVehicle, study: StudyResult | null, color: ColorMode) {
  const { mesh } = built;
  const tris: number[] = [];
  const cols: number[] = [];
  const nt = mesh.indices.length / 3;
  const c = new THREE.Color();
  const cp = study?.aero.cp;
  const heat = study?.aero.heat;
  const cpMax = Math.max(0.4, study?.aero.cpMax ?? 1.8);
  const qMax = Math.max(1e-6, study?.aero.qMax ?? 1);
  for (let t = 0; t < nt; t++) {
    const s = mesh.surfaces[t] ?? 0;
    if (color === "cp" && cp) {
      const u = (cp[t] + 0.2) / (cpMax + 0.2);
      c.setRGB(0.22 + 0.7 * u, 0.28 + 0.55 * u, 0.32 + 0.5 * u);
    } else if (color === "heat" && heat) {
      heatColor(c, heat[t] / qMax);
    } else {
      c.setHex(SURFACE_COLOR[s] ?? 0x888888);
    }
    for (let k = 0; k < 3; k++) {
      const i = mesh.indices[t * 3 + k];
      tris.push(mesh.positions[i * 3], mesh.positions[i * 3 + 1], mesh.positions[i * 3 + 2]);
      cols.push(c.r, c.g, c.b);
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(tris, 3));
  g.setAttribute("color", new THREE.Float32BufferAttribute(cols, 3));
  g.computeVertexNormals();
  return g;
}

function shockGeom(THREE: Three, built: BuiltVehicle) {
  const pos: number[] = [];
  for (const grid of built.shockGrids) {
    for (let i = 0; i < grid.ni - 1; i++) {
      for (let j = 0; j < grid.nj - 1; j++) {
        const a = gridPoint(grid, i, j);
        const b = gridPoint(grid, i + 1, j);
        const c = gridPoint(grid, i + 1, j + 1);
        const d = gridPoint(grid, i, j + 1);
        pos.push(...a, ...b, ...c, ...a, ...c, ...d);
      }
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
  g.computeVertexNormals();
  return g;
}

export interface ViewerOpts {
  wireframe: boolean;
  shock: boolean;
  grid: boolean;
  origin: boolean;
  color: ColorMode;
}

export function WaveriderViewer({
  built,
  study,
  opts,
}: {
  built: BuiltVehicle;
  study: StudyResult | null;
  opts: ViewerOpts;
}) {
  const wrapRef = useRef<HTMLDivElement>(null);
  const builtRef = useRef(built);
  const studyRef = useRef(study);
  const optsRef = useRef(opts);
  const rebuildFn = useRef<() => void>(() => {});
  builtRef.current = built;
  studyRef.current = study;
  optsRef.current = opts;

  useEffect(() => {
    const el = wrapRef.current;
    if (!el) return;
    let disposed = false;
    let disposer: (() => void) | undefined;

    void (async () => {
      const THREE = await import("three");
      const { OrbitControls } = await import("three/addons/controls/OrbitControls.js");
      if (disposed || !wrapRef.current) return;

      const scene = new THREE.Scene();
      scene.background = new THREE.Color(0x0b0d0f);
      scene.fog = new THREE.Fog(0x0b0d0f, 18, 48);

      const camera = new THREE.PerspectiveCamera(38, 1, 0.05, 200);
      camera.up.set(0, 0, 1);

      const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
      renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
      renderer.outputColorSpace = THREE.SRGBColorSpace;
      renderer.domElement.style.display = "block";
      renderer.domElement.style.width = "100%";
      renderer.domElement.style.height = "100%";
      el.appendChild(renderer.domElement);

      const resize = () => {
        const w = el.clientWidth;
        const h = el.clientHeight;
        if (w < 2 || h < 2) return;
        camera.aspect = w / h;
        camera.updateProjectionMatrix();
        renderer.setSize(w, h, false);
      };

      const controls = new OrbitControls(camera, renderer.domElement);
      controls.enableDamping = true;
      controls.dampingFactor = 0.08;
      controls.target.set(2, 0, -0.15);

      scene.add(new THREE.AmbientLight(0xb8c4cc, 0.45));
      const key = new THREE.DirectionalLight(0xf2f5f7, 1.15);
      key.position.set(-4, 8, 10);
      scene.add(key);
      const fill = new THREE.DirectionalLight(0x8aa0b0, 0.35);
      fill.position.set(6, -4, 3);
      scene.add(fill);
      const rim = new THREE.DirectionalLight(0xd5dde3, 0.4);
      rim.position.set(2, 0, -8);
      scene.add(rim);
      scene.add(new THREE.HemisphereLight(0xcfd8de, 0x1a1e22, 0.35));

      const bodyMat = new THREE.MeshStandardMaterial({
        vertexColors: true,
        metalness: 0.55,
        roughness: 0.38,
        side: THREE.DoubleSide,
      });
      const wireMat = new THREE.LineBasicMaterial({ color: 0x9aa8b0, transparent: true, opacity: 0.35 });
      const shockMat = new THREE.MeshStandardMaterial({
        color: 0xa8bcc8,
        transparent: true,
        opacity: 0.18,
        side: THREE.DoubleSide,
        depthWrite: false,
        metalness: 0.1,
        roughness: 0.6,
      });

      let body: ThreeNS.Mesh | null = null;
      let wires: ThreeNS.LineSegments | null = null;
      let shock: ThreeNS.Mesh | null = null;
      let gridHelper: ThreeNS.GridHelper | null = null;
      let originGroup: ThreeNS.Group | null = null;

      function makeOrigin(L: number) {
        if (originGroup) {
          scene.remove(originGroup);
          originGroup.traverse((o) => {
            const m = o as ThreeNS.Mesh | ThreeNS.Line;
            if ("geometry" in m && m.geometry) m.geometry.dispose();
            if ("material" in m) {
              const mat = m.material as ThreeNS.Material | ThreeNS.Material[];
              if (Array.isArray(mat)) mat.forEach((x) => x.dispose());
              else mat.dispose();
            }
          });
        }
        const g = new THREE.Group();
        const s = Math.max(0.28, 0.14 * L);
        const add = (to: [number, number, number], color: number) => {
          const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(...to)]);
          g.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color })));
        };
        add([s, 0, 0], 0xc07a6a);
        add([0, s, 0], 0x7f9d88);
        add([0, 0, s], 0xc5d0d8);
        const sph = new THREE.Mesh(
          new THREE.SphereGeometry(Math.max(0.012, 0.01 * L), 10, 8),
          new THREE.MeshBasicMaterial({ color: 0xe8ecef }),
        );
        g.add(sph);
        originGroup = g;
        scene.add(g);
      }

      function frameCamera(L: number, span: number) {
        const dist = Math.max(L, span) * 1.85;
        camera.position.set(-0.28 * L, dist * 0.72, dist * 0.4);
        controls.target.set(L * 0.48, 0, 0);
        camera.near = dist / 80;
        camera.far = dist * 20;
        camera.updateProjectionMatrix();
        if (gridHelper) {
          scene.remove(gridHelper);
          gridHelper.geometry.dispose();
          (gridHelper.material as ThreeNS.Material).dispose();
        }
        const size = Math.max(L, span) * 2.4;
        gridHelper = new THREE.GridHelper(size, 16, 0x2a3138, 0x1a1f24);
        gridHelper.rotateX(Math.PI / 2);
        gridHelper.position.z = 0;
        scene.add(gridHelper);
        makeOrigin(L);
      }

      let framedKey = "";
      function rebuild() {
        const b = builtRef.current;
        if (body) {
          scene.remove(body);
          body.geometry.dispose();
        }
        if (wires) {
          scene.remove(wires);
          wires.geometry.dispose();
        }
        if (shock) {
          scene.remove(shock);
          shock.geometry.dispose();
        }
        const g = geomFrom(THREE, b, studyRef.current, optsRef.current.color);
        body = new THREE.Mesh(g, bodyMat);
        scene.add(body);
        const edges = new THREE.EdgesGeometry(g, 22);
        wires = new THREE.LineSegments(edges, wireMat);
        scene.add(wires);
        shock = new THREE.Mesh(shockGeom(THREE, b), shockMat);
        scene.add(shock);
        const key = `${b.params.family}|${b.params.length}|${b.params.span}|${b.quality.vertices}`;
        if (key !== framedKey) {
          framedKey = key;
          frameCamera(b.params.length, b.params.span);
        }
      }

      function applyOpts() {
        const o = optsRef.current;
        if (wires) wires.visible = o.wireframe;
        if (shock) shock.visible = o.shock;
        if (gridHelper) gridHelper.visible = o.grid;
        if (originGroup) originGroup.visible = o.origin !== false;
      }

      rebuild();
      applyOpts();
      resize();
      rebuildFn.current = rebuild;

      const ro = new ResizeObserver(resize);
      ro.observe(el);

      let raf = 0;
      const loop = () => {
        raf = requestAnimationFrame(loop);
        applyOpts();
        controls.update();
        renderer.render(scene, camera);
      };
      loop();

      disposer = () => {
        cancelAnimationFrame(raf);
        rebuildFn.current = () => {};
        ro.disconnect();
        controls.dispose();
        body?.geometry.dispose();
        wires?.geometry.dispose();
        shock?.geometry.dispose();
        bodyMat.dispose();
        wireMat.dispose();
        shockMat.dispose();
        renderer.dispose();
        renderer.domElement.remove();
      };

      if (disposed) disposer();
    })();

    return () => {
      disposed = true;
      disposer?.();
    };
  }, []);

  useEffect(() => {
    rebuildFn.current();
  }, [built, study, opts.color]);

  return <div ref={wrapRef} className="absolute inset-0 touch-none" />;
}
