function crc32(buf: Uint8Array): number {
  let c = ~0 >>> 0;
  for (let i = 0; i < buf.length; i++) {
    c ^= buf[i];
    for (let k = 0; k < 8; k++) c = (c >>> 1) ^ (0xedb88320 & -(c & 1));
  }
  return ~c >>> 0;
}

function u16(n: number): Uint8Array {
  const b = new Uint8Array(2);
  b[0] = n & 255;
  b[1] = (n >>> 8) & 255;
  return b;
}

function u32(n: number): Uint8Array {
  const b = new Uint8Array(4);
  b[0] = n & 255;
  b[1] = (n >>> 8) & 255;
  b[2] = (n >>> 16) & 255;
  b[3] = (n >>> 24) & 255;
  return b;
}

export interface ZipEntry {
  name: string;
  data: Uint8Array;
}

export function buildZip(entries: ZipEntry[]): Blob {
  const chunks: Uint8Array[] = [];
  const centrals: Uint8Array[] = [];
  let offset = 0;
  const now = new Date();
  const dosTime = ((now.getHours() & 31) << 11) | ((now.getMinutes() & 63) << 5) | ((now.getSeconds() / 2) & 31);
  const dosDate =
    (((now.getFullYear() - 1980) & 127) << 9) | ((now.getMonth() + 1) << 5) | (now.getDate() & 31);

  for (const e of entries) {
    const name = new TextEncoder().encode(e.name);
    const crc = crc32(e.data);
    const local = new Uint8Array(30 + name.length);
    local.set([0x50, 0x4b, 0x03, 0x04], 0);
    local.set(u16(20), 4);
    local.set(u16(0), 8);
    local.set(u16(dosTime), 10);
    local.set(u16(dosDate), 12);
    local.set(u32(crc), 14);
    local.set(u32(e.data.length), 18);
    local.set(u32(e.data.length), 22);
    local.set(u16(name.length), 26);
    local.set(name, 30);
    chunks.push(local, e.data);

    const central = new Uint8Array(46 + name.length);
    central.set([0x50, 0x4b, 0x01, 0x02], 0);
    central.set(u16(20), 4);
    central.set(u16(20), 6);
    central.set(u16(0), 10);
    central.set(u16(dosTime), 12);
    central.set(u16(dosDate), 14);
    central.set(u32(crc), 16);
    central.set(u32(e.data.length), 20);
    central.set(u32(e.data.length), 24);
    central.set(u16(name.length), 28);
    central.set(u32(offset), 42);
    central.set(name, 46);
    centrals.push(central);
    offset += local.length + e.data.length;
  }

  const centralSize = centrals.reduce((n, c) => n + c.length, 0);
  const eocd = new Uint8Array(22);
  eocd.set([0x50, 0x4b, 0x05, 0x06], 0);
  eocd.set(u16(entries.length), 8);
  eocd.set(u16(entries.length), 10);
  eocd.set(u32(centralSize), 12);
  eocd.set(u32(offset), 16);

  return new Blob([...chunks, ...centrals, eocd] as BlobPart[], { type: "application/zip" });
}

export function utf8(s: string): Uint8Array {
  return new TextEncoder().encode(s);
}
