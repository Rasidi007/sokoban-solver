// Room-by-room push-optimal Sokoban solver. A level is a hallway with
// independent rooms off it (a box in the hallway can never return), so the
// push-optimum is the sum of the rooms' optima. Each room is solved alone
// with A* over pushes: states are (sorted boxes, player region), the
// heuristic is the min-cost box-goal matching over shortest-path distances
// (admissible, so the first goal popped is optimal), with dead-square,
// frozen-2x2 and no-matching pruning. Greedy best-first is the fallback.
// Budgets count work, never the clock: two runs write identical bytes.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';
const ABUDGET = 250000, GBUDGET = 150000, TOTAL = 1200000;
const DIRS = [[0, -1, 'u', 'U'], [0, 1, 'd', 'D'], [-1, 0, 'l', 'L'], [1, 0, 'r', 'R']];
const BIG = 1e9;
let spent = 0;

function parse(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length, w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals = [], boxes = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x], i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++)
    if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
  return { w, h, wall, goals, boxes: boxes.sort((a, b) => a - b), player };
}

// Squares from which a box alone can be pushed onto some goal (pulled back from the goals).
function liveSquares(w, wall, goals) {
  const live = new Uint8Array(wall.length), q = [...goals];
  for (const g of goals) live[g] = 1;
  for (let i = 0; i < q.length; i++) {
    const x = q[i];
    for (const [dx, dy] of DIRS) {
      const d = dy * w + dx, y = x - d;
      if (!wall[y] && !wall[y - d] && !live[y]) { live[y] = 1; q.push(y); }
    }
  }
  return live;
}

// Flood the player's region from `from`, marking it in `seen`; return its lowest square.
function flood(w, wall, occ, from, seen, stamp, stack) {
  let low = from, n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n];
    if (p < low) low = p;
    for (const q of [p - w, p + w, p - 1, p + 1])
      if (!wall[q] && !occ[q] && seen[q] !== stamp) { seen[q] = stamp; stack[n++] = q; }
  }
  return low;
}

// Whether the box pushed to `t` sits in a frozen 2x2 block holding a box off its goal.
function frozen(w, wall, occ, goal, t) {
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1, c = a + w, d = c + 1;
    if (!(wall[a] || occ[a]) || !(wall[b] || occ[b]) || !(wall[c] || occ[c]) || !(wall[d] || occ[d])) continue;
    if ((occ[a] && !goal[a]) || (occ[b] && !goal[b]) || (occ[c] && !goal[c]) || (occ[d] && !goal[d])) return true;
  }
  return false;
}

// Shortest walk from `from` to `to` around walls and boxes, in lower LURD letters.
function walkTo(w, wall, occ, from, to) {
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1), how = new Array(wall.length), q = [from];
  prev[from] = from;
  for (let i = 0; i < q.length; i++) {
    const p = q[i];
    if (p === to) break;
    for (const [dx, dy, ch] of DIRS) {
      const s = p + dy * w + dx;
      if (wall[s] || occ[s] || prev[s] !== -1) continue;
      prev[s] = p; how[s] = ch; q.push(s);
    }
  }
  if (prev[to] === -1) throw new Error('walk unreachable');
  let s = '';
  for (let p = to; p !== from; p = prev[p]) s = how[p] + s;
  return s;
}

function permsOf(n) {
  const out = [], a = [...Array(n).keys()];
  const gen = (k) => {
    if (k === 1) { out.push(a.slice()); return; }
    for (let i = 0; i < k; i++) {
      gen(k - 1);
      const j = k % 2 ? 0 : i, t = a[j];
      a[j] = a[k - 1]; a[k - 1] = t;
    }
  };
  gen(n);
  return out;
}

// Binary heap on [f, -g, -onGoal, seq]; deterministic.
function lt(a, b) {
  return a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && (a[2] < b[2] || (a[2] === b[2] && a[3] < b[3])))));
}
function heapPush(hp, e) {
  let i = hp.length;
  hp.push(e);
  while (i > 0) {
    const p = (i - 1) >> 1;
    if (lt(hp[p], e)) break;
    hp[i] = hp[p]; i = p;
  }
  hp[i] = e;
}
function heapPop(hp) {
  const top = hp[0], last = hp.pop();
  if (hp.length > 0) {
    let i = 0;
    for (;;) {
      let c = 2 * i + 1;
      if (c >= hp.length) break;
      if (c + 1 < hp.length && lt(hp[c + 1], hp[c])) c++;
      if (lt(last, hp[c])) break;
      hp[i] = hp[c]; i = c;
    }
    hp[i] = last;
  }
  return top;
}

// Push search on board B. weight 1 is A* (f = g + h, optimal); weight 0 is greedy (f = h).
function search(B, budget, weight) {
  const { w, wall, goals } = B, size = wall.length, n = goals.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(w, wall, goals);
  const boxes0 = B.boxes.slice().sort((a, b) => a - b);
  if (boxes0.some((b) => !live[b])) return null;
  if (boxes0.every((b) => goal[b])) return { pushes: 0, moves: '', end: B.player };
  const dist = new Int32Array(n * size).fill(-1);
  for (let gi = 0; gi < n; gi++) {
    const base = gi * size, q = [goals[gi]];
    dist[base + goals[gi]] = 0;
    for (let i = 0; i < q.length; i++) {
      const p = q[i];
      for (const [dx, dy] of DIRS) {
        const s = p + dy * w + dx;
        if (!wall[s] && dist[base + s] < 0) { dist[base + s] = dist[base + p] + 1; q.push(s); }
      }
    }
  }
  const perms = n <= 6 ? permsOf(n) : null;
  const heur = (boxes) => {
    if (perms) {
      let best = BIG;
      for (const pm of perms) {
        let c = 0;
        for (let i = 0; i < n; i++) {
          const d = dist[pm[i] * size + boxes[i]];
          if (d < 0) { c = BIG; break; }
          c += d;
        }
        if (c < best) best = c;
      }
      return best;
    }
    let c = 0;
    for (const b of boxes) {
      let m = BIG;
      for (let gi = 0; gi < n; gi++) {
        const d = dist[gi * size + b];
        if (d >= 0 && d < m) m = d;
      }
      if (m === BIG) return BIG;
      c += m;
    }
    return c;
  };
  if (heur(boxes0) >= BIG) return null;
  const sBox = [boxes0], sPlay = [B.player], sPar = [-1], sFrom = [-1], sTo = [-1], sKey = [''];
  const best = new Map(), occ = new Uint8Array(size);
  const seen = new Int32Array(size), mark = new Int32Array(size), stack = new Int32Array(size);
  let stamp = 0, mstamp = 0;
  const keyOf = (boxes, low) => boxes.join(',') + '|' + low;
  for (const b of boxes0) occ[b] = 1;
  const low0 = flood(w, wall, occ, B.player, seen, ++stamp, stack);
  for (const b of boxes0) occ[b] = 0;
  sKey[0] = keyOf(boxes0, low0);
  best.set(sKey[0], 0);
  const hp = [];
  let seq = 0, expanded = 0, found = -1;
  heapPush(hp, [heur(boxes0), 0, -boxes0.filter((b) => goal[b]).length, seq++, 0]);
  while (hp.length > 0) {
    const [, ng, , , idx] = heapPop(hp);
    const g = -ng;
    if (best.get(sKey[idx]) < g) continue;
    const boxes = sBox[idx];
    if (boxes.every((b) => goal[b])) { found = idx; break; }
    if (++expanded > budget || ++spent > TOTAL) return null;
    const player = sPlay[idx];
    for (const b of boxes) occ[b] = 1;
    const here = ++stamp;
    flood(w, wall, occ, player, seen, here, stack);
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi];
      for (const [dx, dy] of DIRS) {
        const d = dy * w + dx, from = b - d, to = b + d;
        if (seen[from] !== here || wall[to] || occ[to] || !live[to]) continue;
        occ[b] = 0; occ[to] = 1;
        if (frozen(w, wall, occ, goal, to)) { occ[to] = 0; occ[b] = 1; continue; }
        const nb = boxes.slice();
        nb[bi] = to;
        nb.sort((a, c) => a - c);
        const low = flood(w, wall, occ, b, mark, ++mstamp, stack);
        occ[to] = 0; occ[b] = 1;
        const k = keyOf(nb, low), ng2 = g + 1, prev = best.get(k);
        if (prev !== undefined && prev <= ng2) continue;
        const h = heur(nb);
        if (h >= BIG) continue;
        best.set(k, ng2);
        const ni = sBox.length;
        sBox.push(nb); sPlay.push(b); sPar.push(idx); sFrom.push(b); sTo.push(to); sKey.push(k);
        heapPush(hp, [weight === 1 ? ng2 + h : h, -ng2, -nb.filter((x) => goal[x]).length, seq++, ni]);
      }
    }
    for (const b of boxes) occ[b] = 0;
  }
  if (found < 0) return null;
  const path = [];
  for (let s = found; s > 0; s = sPar[s]) path.push(s);
  path.reverse();
  let moves = '', player = B.player, boxes = boxes0;
  for (const s of path) {
    const from = sFrom[s], to = sTo[s], d = to - from;
    const dir = DIRS.find(([dx, dy]) => dy * w + dx === d);
    for (const b of boxes) occ[b] = 1;
    moves += walkTo(w, wall, occ, player, from - d);
    for (const b of boxes) occ[b] = 0;
    moves += dir[3];
    player = from;
    boxes = boxes.map((b) => (b === from ? to : b)).sort((a, b) => a - b);
  }
  return { pushes: path.length, moves, end: player };
}

// The level's rooms: cut the dead squares of the player's row (the hallway);
// each part left holding boxes is a room, with its entry square outside its door.
function findRooms(L) {
  const { w, wall, goals, boxes, player } = L, size = wall.length;
  const live = liveSquares(w, wall, goals), cut = new Uint8Array(size);
  const row = Math.floor(player / w);
  for (let x = 0; x < w; x++) {
    const i = row * w + x;
    if (!wall[i] && !live[i]) cut[i] = 1;
  }
  if (boxes.some((b) => cut[b]) || goals.some((g) => cut[g])) return null;
  const part = new Int32Array(size).fill(-1), rooms = [];
  for (let s = 0; s < size; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      const p = squares[i];
      for (const [dx, dy] of DIRS) {
        const q = p + dy * w + dx;
        if (!wall[q] && !cut[q] && part[q] < 0) { part[q] = rooms.length; squares.push(q); }
      }
    }
    rooms.push({ squares, boxes: [], goals: [] });
  }
  for (const b of boxes) rooms[part[b]].boxes.push(b);
  for (const g of goals) if (part[g] >= 0) rooms[part[g]].goals.push(g);
  const out = [];
  for (const r of rooms) {
    if (r.boxes.length === 0) continue;
    if (r.boxes.length !== r.goals.length) return null;
    let entry = -1;
    for (const s of r.squares) {
      for (const [dx, dy] of DIRS) {
        const q = s + dy * w + dx;
        if (cut[q]) { entry = q; break; }
      }
      if (entry >= 0) break;
    }
    if (entry < 0) return null;
    r.entry = entry;
    r.boxes.sort((a, b) => a - b);
    r.goals.sort((a, b) => a - b);
    out.push(r);
  }
  return out;
}

function solveLevel(text) {
  const L = parse(text), rooms = findRooms(L);
  if (!rooms) {
    const r = search(L, ABUDGET, 1) || search(L, GBUDGET, 0);
    return r ? r.moves : '';
  }
  let moves = '', player = L.player, boxes = L.boxes.slice();
  const occ = new Uint8Array(L.w * L.h);
  for (const room of rooms) {
    for (const b of boxes) occ[b] = 1;
    let walk;
    try {
      walk = walkTo(L.w, L.wall, occ, player, room.entry);
    } catch {
      return moves;
    }
    for (const b of boxes) occ[b] = 0;
    moves += walk;
    player = room.entry;
    const wall = new Uint8Array(L.w * L.h).fill(1);
    for (const s of room.squares) wall[s] = 0;
    wall[room.entry] = 0;
    const sub = { w: L.w, h: L.h, wall, goals: room.goals, boxes: room.boxes.slice(), player: room.entry };
    const r = search(sub, ABUDGET, 1) || search(sub, GBUDGET, 0);
    if (!r) return moves;
    moves += r.moves;
    player = r.end;
    const done = new Set(room.boxes);
    boxes = boxes.filter((b) => !done.has(b)).concat(room.goals);
  }
  return moves;
}

for (const ent of readdirSync(I, { withFileTypes: true }).sort((a, b) => (a.name < b.name ? -1 : 1))) {
  if (!ent.isDirectory()) continue;
  let moves = '';
  try {
    moves = solveLevel(readFileSync(join(I, ent.name, 'level.txt'), 'utf8'));
  } catch {
    moves = '';
  }
  mkdirSync(join(O, ent.name), { recursive: true });
  writeFileSync(join(O, ent.name, 'solution.txt'), moves + '\n');
}
