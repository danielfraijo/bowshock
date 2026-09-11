/** 1976 US Standard Atmosphere, 0–86 km. SI units. */

export interface Atmosphere {
  h: number;
  T: number;
  p: number;
  rho: number;
  a: number;
  mu: number;
  q: number;
  V: number;
  ReL: number;
}

const G0 = 9.80665;
const R = 287.05287;
const RE = 6356766;
const GAMMA = 1.4;

type Layer = { h: number; T: number; p: number; L: number };

const LAYERS: Layer[] = [
  { h: 0, T: 288.15, p: 101325, L: -0.0065 },
  { h: 11000, T: 216.65, p: 22632.1, L: 0 },
  { h: 20000, T: 216.65, p: 5474.89, L: 0.001 },
  { h: 32000, T: 228.65, p: 868.019, L: 0.0028 },
  { h: 47000, T: 270.65, p: 110.906, L: 0 },
  { h: 51000, T: 270.65, p: 66.9389, L: -0.0028 },
  { h: 71000, T: 214.65, p: 3.95642, L: -0.002 },
];

function geopotential(h: number) {
  return (RE * h) / (RE + h);
}

export function atmosphere(altKm: number, M: number, gamma = GAMMA): Atmosphere {
  const h = Math.max(0, Math.min(altKm, 84.9)) * 1000;
  const H = geopotential(h);
  let layer = LAYERS[0];
  for (let i = LAYERS.length - 1; i >= 0; i--) {
    if (H >= LAYERS[i].h) {
      layer = LAYERS[i];
      break;
    }
  }
  const dh = H - layer.h;
  let T: number;
  let p: number;
  if (Math.abs(layer.L) < 1e-9) {
    T = layer.T;
    p = layer.p * Math.exp((-G0 * dh) / (R * layer.T));
  } else {
    T = layer.T + layer.L * dh;
    p = layer.p * (T / layer.T) ** (-G0 / (R * layer.L));
  }
  const rho = p / (R * T);
  const a = Math.sqrt(gamma * R * T);
  const V = M * a;
  const mu = 1.716e-5 * (T / 273.15) ** 1.5 * (273.15 + 110.4) / (T + 110.4);
  const q = 0.5 * rho * V * V;
  const ReL = (rho * V) / mu;
  return { h, T, p, rho, a, mu, q, V, ReL };
}
