import { useEffect, useRef } from "react";
import type * as ThreeNS from "three";
import type { BuiltVehicle, ColorMode } from "@/lib/waverider/types";
import { SURFACE_ID } from "@/lib/waverider/types";
import { gridPoint } from "@/lib/waverider/mesh";
import type { StudyResult } from "@/lib/waverider/study";

const SURFACE_GREY: Record<number, number> = {
  [SURFACE_ID.upper]: 0xe8e8e8,
  [SURFACE_ID.lower]: 0x5a5a5a,
  [SURFACE_ID.base]: 0x2a2a2a,
  [SURFACE_ID.leading]: 0xc8c8c8,
  [SURFACE_ID.symmetry]: 0x7a7a7a,
  [SURFACE_ID.inlet]: 0xb0b0b0,
  [SURFACE_ID.nozzle]: 0x3a3a3a,
  [SURFACE_ID.cowl]: 0x8a8a8a,
};

type Three = typeof ThreeNS;

function jetRamp(c: ThreeNS.Color, t: number) {
  const u = Math.max(0, Math.min(1, t));
  const r = Math.max(0, Math.min(1, 1.5 * u - 0.2));
  const g = Math.max(0, Math.min(1, u < 0.5 ? 2.2 * u : 2.2 * (1 - u)));
  const b = Math.max(0, Math.min(1, 1.15 - 1.7 * u));
  c.setRGB(r, g, b);
}

function pct(v: number[], p: number) {
  if (!v.length) return 0;
  const i = Math.min(v.length - 1, Math.max(0, Math.floor(p * (v.length - 1))));
  return v[i];
}

export function fieldScale(
  study: StudyResult | null,
  color: ColorMode,
  surfaces?: Uint8Array,
): { lo: number; hi: number; log: boolean; unit: string } {
  if (!study) return { lo: 0, hi: 1, log: false, unit: "" };
  const srcOf = () => {
    if (color === "heat" || color === "qhat") return study.aero.heat;
    if (color === "cp" || color === "pratio") return study.aero.cp;
    if (color === "temp") return study.aero.twEq;
    if (color === "mach") return study.aero.machE;
    if (color === "stanton") return study.aero.stanton;
    if (color === "cf") return study.aero.cf;
    if (color === "impact") return study.aero.impact;
    return null;
  };
  const raw = srcOf();
  if (!raw) return { lo: 0, hi: 1, log: false, unit: "" };
  const skipBase = (i: number) =>
    surfaces != null && (surfaces[i] === SURFACE_ID.base || surfaces[i] === SURFACE_ID.nozzle);
  const pick = (positive: boolean) => {
    const v: number[] = [];
    for (let i = 0; i < raw.length; i++) {
      if (skipBase(i)) continue;
      const x = raw[i];
      if (!Number.isFinite(x)) continue;
      if (positive && !(x > 0)) continue;
      v.push(x);
    }
    v.sort((a, b) => a - b);
    return v;
  };
  if (color === "heat") {
    const hv = pick(true);
    const hi = Math.max(pct(hv, 0.99), 1e-8);
    const lo = Math.max(pct(hv, 0.05), hi * 1e-4);
    return { lo, hi, log: true, unit: "W/cm²" };
  }
  if (color === "qhat") {
    const qs = Math.max(study.aero.qStag, 1e-8);
    const hv = pick(true).map((q) => q / qs);
    const hi = Math.max(pct(hv, 0.99), 1e-6);
    return { lo: Math.max(pct(hv, 0.05), hi * 1e-3), hi, log: false, unit: "q/qs" };
  }
  if (color === "pratio") {
    const M = Math.max(1.05, study.flightMach);
    const g = 1.4;
    const q = 0.5 * g * M * M;
    const hv = pick(false).map((cp) => 1 + cp * q);
    return { lo: Math.max(pct(hv, 0.02), 0), hi: Math.max(pct(hv, 0.98), 1.05), log: false, unit: "p/p∞" };
  }
  if (color === "temp") {
    const hv = pick(true);
    return { lo: Math.max(pct(hv, 0.02), 200), hi: Math.max(pct(hv, 0.99), 400), log: false, unit: "K" };
  }
  if (color === "mach") {
    const hv = pick(true);
    return { lo: 0, hi: Math.max(pct(hv, 0.99), 1), log: false, unit: "Me" };
  }
  if (color === "stanton") {
    const hv = pick(true);
    const hi = Math.max(pct(hv, 0.99), 1e-8);
    return { lo: Math.max(pct(hv, 0.05), hi * 1e-3), hi, log: true, unit: "St" };
  }
  if (color === "cf") {
    const hv = pick(true);
    const hi = Math.max(pct(hv, 0.99), 1e-6);
    return { lo: Math.max(pct(hv, 0.05), hi * 1e-3), hi, log: true, unit: "Cf" };
  }
  if (color === "impact") {
    const hv = pick(false);
    return { lo: pct(hv, 0.02), hi: Math.max(pct(hv, 0.98), 0.05), log: false, unit: "sinθ" };
  }
  const cv = pick(false);
  const lo = pct(cv, 0.02);
  const hi = Math.max(pct(cv, 0.98), lo + 1e-3);
  return { lo, hi, log: false, unit: "Cp" };
}

function geomFrom(THREE: Three, built: BuiltVehicle, study: StudyResult | null, color: ColorMode) {
  const { mesh } = built;
  const tris: number[] = [];
  const cols: number[] = [];
  const nt = mesh.indices.length / 3;
  const c = new THREE.Color();
  const scale = fieldScale(study, color, mesh.surfaces);
  const src =
    color === "heat" || color === "qhat"
      ? study?.aero.heat
      : color === "cp" || color === "pratio"
        ? study?.aero.cp
        : color === "temp"
          ? study?.aero.twEq
          : color === "mach"
            ? study?.aero.machE
            : color === "stanton"
              ? study?.aero.stanton
              : color === "cf"
                ? study?.aero.cf
                : color === "impact"
                  ? study?.aero.impact
                  : null;
  const M = Math.max(1.05, study?.flightMach ?? built.params.mach);
  const g = built.params.gamma || 1.4;
  const qInf = 0.5 * g * M * M;
  const qs = Math.max(study?.aero.qStag ?? 1, 1e-8);
  const valueAt = (t: number) => {
    const v = src ? (src[t] ?? 0) : 0;
    if (color === "pratio") return 1 + v * qInf;
    if (color === "qhat") return v / qs;
    return v;
  };
  const mapT = (v: number) => {
    if (scale.log) {
      const lo = Math.log10(scale.lo);
      const hi = Math.log10(scale.hi);
      return (Math.log10(Math.max(v, scale.lo)) - lo) / Math.max(hi - lo, 1e-6);
    }
    return (v - scale.lo) / Math.max(scale.hi - scale.lo, 1e-6);
  };
  for (let t = 0; t < nt; t++) {
    const s = mesh.surfaces[t] ?? 0;
    if (src) jetRamp(c, mapT(valueAt(t)));
    else c.setHex(SURFACE_GREY[s] ?? 0x888888);
    for (let k = 0; k < 3; k++) {
      const i = mesh.indices[t * 3 + k];
      tris.push(mesh.positions[i * 3], mesh.positions[i * 3 + 1], mesh.positions[i * 3 + 2]);
      cols.push(c.r, c.g, c.b);
    }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(tris, 3));
  geo.setAttribute("color", new THREE.Float32BufferAttribute(cols, 3));
  geo.computeVertexNormals();
  return geo;
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

function seamPositions(built: BuiltVehicle): number[] {
  const { positions, indices, surfaces } = built.mesh;
  const nt = indices.length / 3;
  const map = new Map<string, { a: number; b: number; s: number[] }>();
  const add = (u: number, v: number, s: number) => {
    const lo = u < v ? u : v;
    const hi = u < v ? v : u;
    const k = lo * 1e7 + hi;
    const rec = map.get(String(k));
    if (!rec) map.set(String(k), { a: lo, b: hi, s: [s] });
    else rec.s.push(s);
  };
  for (let t = 0; t < nt; t++) {
    const i0 = indices[t * 3];
    const i1 = indices[t * 3 + 1];
    const i2 = indices[t * 3 + 2];
    const s = surfaces[t] ?? 0;
    add(i0, i1, s);
    add(i1, i2, s);
    add(i2, i0, s);
  }
  const pos: number[] = [];
  for (const rec of map.values()) {
    const seam = rec.s.length === 1 || rec.s[0] !== rec.s[1];
    if (!seam) continue;
    pos.push(
      positions[rec.a * 3],
      positions[rec.a * 3 + 1],
      positions[rec.a * 3 + 2],
      positions[rec.b * 3],
      positions[rec.b * 3 + 1],
      positions[rec.b * 3 + 2],
    );
  }
  return pos;
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
      scene.background = new THREE.Color(0x080808);

      const camera = new THREE.PerspectiveCamera(34, 1, 0.05, 200);
      camera.up.set(0, 0, 1);

      const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
      renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
      renderer.outputColorSpace = THREE.SRGBColorSpace;
      renderer.toneMapping = THREE.NoToneMapping;
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

      scene.add(new THREE.AmbientLight(0xffffff, 0.28));
      const key = new THREE.DirectionalLight(0xffffff, 1.35);
      key.position.set(-5, 7, 11);
      scene.add(key);
      const fill = new THREE.DirectionalLight(0xffffff, 0.32);
      fill.position.set(7, -6, 2);
      scene.add(fill);
      const rim = new THREE.DirectionalLight(0xffffff, 0.7);
      rim.position.set(3, 1, -9);
      scene.add(rim);
      scene.add(new THREE.HemisphereLight(0xd0d0d0, 0x1a1a1a, 0.4));

      const bodyMat = new THREE.MeshStandardMaterial({
        vertexColors: true,
        metalness: 0.04,
        roughness: 0.62,
        side: THREE.DoubleSide,
        polygonOffset: true,
        polygonOffsetFactor: 1,
        polygonOffsetUnits: 1,
      });
      const fieldMat = new THREE.MeshBasicMaterial({
        vertexColors: true,
        side: THREE.DoubleSide,
        polygonOffset: true,
        polygonOffsetFactor: 1,
        polygonOffsetUnits: 1,
      });
      const featureMat = new THREE.LineBasicMaterial({ color: 0xf0f0f0, transparent: true, opacity: 0.55 });
      const seamMat = new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.92 });
      const wireMat = new THREE.LineBasicMaterial({ color: 0xc8c8c8, transparent: true, opacity: 0.22 });
      const shockMat = new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.28 });

      let body: ThreeNS.Mesh | null = null;
      let features: ThreeNS.LineSegments | null = null;
      let seams: ThreeNS.LineSegments | null = null;
      let wires: ThreeNS.LineSegments | null = null;
      let shock: ThreeNS.LineSegments | null = null;
      let gridHelper: ThreeNS.GridHelper | null = null;
      let originGroup: ThreeNS.Group | null = null;
      let markerGroup: ThreeNS.Group | null = null;

      function disposeObj(obj: ThreeNS.Object3D | null) {
        if (!obj) return;
        scene.remove(obj);
        obj.traverse((o) => {
          const m = o as ThreeNS.Mesh | ThreeNS.Line;
          if ("geometry" in m && m.geometry) m.geometry.dispose();
        });
      }

      function makeOrigin(L: number) {
        disposeObj(originGroup);
        const g = new THREE.Group();
        const s = Math.max(0.28, 0.14 * L);
        const add = (to: [number, number, number], color: number) => {
          const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(...to)]);
          g.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color })));
        };
        add([s, 0, 0], 0xffffff);
        add([0, s, 0], 0x8a8a8a);
        add([0, 0, s], 0xc8c8c8);
        const sph = new THREE.Mesh(
          new THREE.SphereGeometry(Math.max(0.012, 0.01 * L), 10, 8),
          new THREE.MeshBasicMaterial({ color: 0xf4f4f4 }),
        );
        g.add(sph);
        originGroup = g;
        scene.add(g);
      }

      function makeMarkers(b: BuiltVehicle, st: StudyResult | null) {
        disposeObj(markerGroup);
        const g = new THREE.Group();
        const L = Math.max(b.params.length, 0.2);
        const r = Math.max(0.014, 0.011 * L);
        const cg = st?.aero.cg ?? [b.params.cgFrac * L, 0, 0];
        const xCp = st?.aero.xCp ?? cg[0];
        const cgMesh = new THREE.Mesh(
          new THREE.OctahedronGeometry(r * 0.85, 0),
          new THREE.MeshBasicMaterial({ color: 0xb8b8b8 }),
        );
        cgMesh.position.set(cg[0], cg[1], cg[2]);
        g.add(cgMesh);
        const copMesh = new THREE.Mesh(
          new THREE.SphereGeometry(r * 1.15, 14, 10),
          new THREE.MeshBasicMaterial({ color: 0xf4f4f4 }),
        );
        copMesh.position.set(xCp, 0, 0);
        g.add(copMesh);
        const copLine = new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(cg[0], cg[1], cg[2]),
          new THREE.Vector3(xCp, 0, 0),
        ]);
        g.add(new THREE.Line(copLine, new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.7 })));
        const a = (b.params.alphaDeg || 0) * (Math.PI / 180);
        const be = (b.params.betaDeg || 0) * (Math.PI / 180);
        const vx = Math.cos(a) * Math.cos(be);
        const vy = Math.sin(be);
        const vz = Math.sin(a) * Math.cos(be);
        const len = 0.38 * L;
        const origin = new THREE.Vector3(-0.08 * L - vx * len, -vy * len, -vz * len);
        const arrow = new THREE.ArrowHelper(new THREE.Vector3(vx, vy, vz).normalize(), origin, len, 0xf0f0f0, 0.07 * L, 0.035 * L);
        g.add(arrow);
        markerGroup = g;
        scene.add(g);
      }

      function frameCamera(L: number, span: number) {
        const dist = Math.max(L, span) * 2.05;
        camera.position.set(-0.18 * L, dist * 0.82, dist * 0.36);
        controls.target.set(L * 0.48, 0, 0);
        camera.near = dist / 90;
        camera.far = dist * 22;
        camera.updateProjectionMatrix();
        if (gridHelper) {
          scene.remove(gridHelper);
          gridHelper.geometry.dispose();
          (gridHelper.material as ThreeNS.Material).dispose();
        }
        const size = Math.max(L, span) * 2.4;
        gridHelper = new THREE.GridHelper(size, 16, 0x3a3a3a, 0x1a1a1a);
        gridHelper.rotateX(Math.PI / 2);
        gridHelper.position.z = 0;
        scene.add(gridHelper);
        makeOrigin(L);
      }

      let framedKey = "";
      function rebuild() {
        const b = builtRef.current;
        disposeObj(body);
        disposeObj(features);
        disposeObj(seams);
        disposeObj(wires);
        disposeObj(shock);
        const g = geomFrom(THREE, b, studyRef.current, optsRef.current.color);
        const field = optsRef.current.color !== "surface";
        body = new THREE.Mesh(g, field ? fieldMat : bodyMat);
        scene.add(body);
        const feat = new THREE.EdgesGeometry(g, 16);
        features = new THREE.LineSegments(feat, featureMat);
        scene.add(features);
        const seamG = new THREE.BufferGeometry();
        seamG.setAttribute("position", new THREE.Float32BufferAttribute(seamPositions(b), 3));
        seams = new THREE.LineSegments(seamG, seamMat);
        scene.add(seams);
        const allEdges = new THREE.EdgesGeometry(g, 1);
        wires = new THREE.LineSegments(allEdges, wireMat);
        scene.add(wires);
        const sg = shockGeom(THREE, b);
        const shockEdges = new THREE.EdgesGeometry(sg, 12);
        sg.dispose();
        shock = new THREE.LineSegments(shockEdges, shockMat);
        scene.add(shock);
        makeMarkers(b, studyRef.current);
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
        if (markerGroup) markerGroup.visible = o.origin !== false;
        if (features) features.visible = o.color === "surface";
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
        features?.geometry.dispose();
        seams?.geometry.dispose();
        wires?.geometry.dispose();
        shock?.geometry.dispose();
        disposeObj(markerGroup);
        bodyMat.dispose();
        fieldMat.dispose();
        featureMat.dispose();
        seamMat.dispose();
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

  const scale = opts.color !== "surface" && study ? fieldScale(study, opts.color, built.mesh.surfaces) : null;
  const fmtScale = (v: number) => (v >= 10 ? v.toFixed(0) : v >= 1 ? v.toFixed(2) : v.toFixed(3));

  return (
    <div ref={wrapRef} className="absolute inset-0 touch-none">
      {scale ? (
        <div className="pointer-events-none absolute right-3 bottom-14 flex items-end gap-1.5">
          <div className="flex h-28 flex-col justify-between py-0.5 text-right font-mono text-[9px] leading-none text-muted">
            <span>{fmtScale(scale.hi)}</span>
            <span>
              {scale.unit}
              {scale.log ? " log" : ""}
            </span>
            <span>{fmtScale(scale.lo)}</span>
          </div>
          <div className="jet-ramp h-28 w-2.5 rounded-sm" title={opts.color === "heat" ? "q W/cm²" : "Cp"} />
        </div>
      ) : null}
    </div>
  );
}
