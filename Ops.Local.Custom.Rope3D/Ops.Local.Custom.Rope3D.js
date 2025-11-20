/*
author: Rodrigo Favarete
email: rodrigo@favarete.art
description: Rope3D operator with physics simulation
version: 1.0
*/


const inUpdate      = op.inTrigger("Update");

// Top Points
const inTopX        = op.inFloat("Top X", 0.0);
const inTopY        = op.inFloat("Top Y", 1.5);
const inTopZ        = op.inFloat("Top Z", 0.0);

// Lower Points
const inBottomX     = op.inFloat("Bottom X", 0.0);
const inBottomY     = op.inFloat("Bottom Y", -0.5);
const inBottomZ     = op.inFloat("Bottom Z", 0.0);

// String parameters
const inSegments    = op.inInt("Segments", 10);        // Number of segments
const inLength      = op.inFloat("Rope Length", 2.0);  // Total Length
const inGravity     = op.inFloat("Gravity", -9.8);
const inIter        = op.inInt("Constraint Iterations", 8);
const inDamping     = op.inFloat("Damping", 0.94);     // 0.9 ~ 0.97 é bom

// Output in Array3 (xyzxyz...)
const outPoints     = op.outArray("Points", null, 3);

// Internal state
let positions      = [];     // [{x,y,z}, ...]
let prevPositions  = [];     // [{x,y,z}, ...]
let lastTime       = performance.now();

// Sleep
let isSleeping     = false;
let sleepCounter   = 0;
const SLEEP_FRAMES = 6;      // How many frames almost stopped to sleep
const VEL_EPS      = 1e-3;   // Minimum speed to consider "stopped"

// To detect if inputs changed
let lastTopX       = null;
let lastTopY       = null;
let lastTopZ       = null;
let lastBottomX    = null;
let lastBottomY    = null;
let lastBottomZ    = null;
const INPUT_EPS    = 1e-5;

function initRope(topX, topY, topZ)
{
    const n = Math.max(1, inSegments.get() | 0);
    const L = inLength.get();

    positions     = [];
    prevPositions = [];

    const segLen = L / n;

    // n+1 points for n segments
    for (let i = 0; i <= n; i++)
    {
        const x = topX;
        const y = topY - segLen * i;
        const z = topZ;
        positions.push({ x, y, z });
        prevPositions.push({ x, y, z });
    }

    isSleeping   = false;
    sleepCounter = 0;
}

inSegments.onChange = function () {
    initRope(inTopX.get(), inTopY.get(), inTopZ.get());
};
inLength.onChange   = function () {
    initRope(inTopX.get(), inTopY.get(), inTopZ.get());
};
inTopX.onChange     = function () {
    initRope(inTopX.get(), inTopY.get(), inTopZ.get());
};
inTopY.onChange     = function () {
    initRope(inTopX.get(), inTopY.get(), inTopZ.get());
};
inTopZ.onChange     = function () {
    initRope(inTopX.get(), inTopY.get(), inTopZ.get());
};

function step(dt, topX, topY, topZ, bottomX, bottomY, bottomZ)
{
    const n = positions.length - 1;
    if (n < 1) return 0;

    const g       = inGravity.get();
    const L       = inLength.get();
    const restLen = L / n;
    const iters   = Math.max(1, inIter.get() | 0);
    const damping = inDamping.get();

    let maxVel = 0;

    const bottom = positions[n];
    bottom.x = bottomX;
    bottom.y = bottomY;
    bottom.z = bottomZ;
    prevPositions[n].x = bottom.x;
    prevPositions[n].y = bottom.y;
    prevPositions[n].z = bottom.z;

    // Verlet Integration with damping for intermediate points
    for (let i = 1; i < n; i++) // 0 = topo, n = base
    {
        const p    = positions[i];
        const prev = prevPositions[i];

        const oldX = p.x;
        const oldY = p.y;
        const oldZ = p.z;

        let vx = (p.x - prev.x) * damping;
        let vy = (p.y - prev.y) * damping;
        let vz = (p.z - prev.z) * damping;

        prev.x = oldX;
        prev.y = oldY;
        prev.z = oldZ;

        // Gravity only on Y
        p.x += vx;
        p.y += vy + g * dt * dt;
        p.z += vz;

        const vMag = Math.max(Math.abs(vx), Math.abs(vy), Math.abs(vz));
        if (vMag > maxVel) maxVel = vMag;
    }

    positions[0].x     = topX;
    positions[0].y     = topY;
    positions[0].z     = topZ;
    prevPositions[0].x = positions[0].x;
    prevPositions[0].y = positions[0].y;
    prevPositions[0].z = positions[0].z;

    // Iteration length constraints (3D)
    for (let iter = 0; iter < iters; iter++)
    {
        for (let i = 0; i < n; i++)
        {
            const p1 = positions[i];
            const p2 = positions[i + 1];

            let dx   = p2.x - p1.x;
            let dy   = p2.y - p1.y;
            let dz   = p2.z - p1.z;
            let dist = Math.hypot(dx, dy, dz);
            if (!dist) continue;

            const diff = (dist - restLen) / dist;

            if (i === 0)
            {
                p2.x -= dx * diff;
                p2.y -= dy * diff;
                p2.z -= dz * diff;
            }
            else if (i === n - 1)
            {
                p1.x += dx * diff;
                p1.y += dy * diff;
                p1.z += dz * diff;
            }
            else
            {
                const half = diff * 0.5;
                p1.x += dx * half;
                p1.y += dy * half;
                p1.z += dz * half;
                p2.x -= dx * half;
                p2.y -= dy * half;
                p2.z -= dz * half;
            }
        }
    }

    return maxVel;
}

inUpdate.onTriggered = function ()
{
    const now = performance.now();
    let dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // Avoid giant dt when switching tabs / pausing
    if (dt > 0.05) dt = 0.05;

    const topX    = inTopX.get();
    const topY    = inTopY.get();
    const topZ    = inTopZ.get();
    const bottomX = inBottomX.get();
    const bottomY = inBottomY.get();
    const bottomZ = inBottomZ.get();

    // Detect if inputs have changed since the last frame
    const inputsMoved =
        lastTopX    === null ||
        Math.abs(topX    - lastTopX)    > INPUT_EPS ||
        Math.abs(topY    - lastTopY)    > INPUT_EPS ||
        Math.abs(topZ    - lastTopZ)    > INPUT_EPS ||
        Math.abs(bottomX - lastBottomX) > INPUT_EPS ||
        Math.abs(bottomY - lastBottomY) > INPUT_EPS ||
        Math.abs(bottomZ - lastBottomZ) > INPUT_EPS;

    lastTopX    = topX;
    lastTopY    = topY;
    lastTopZ    = topZ;
    lastBottomX = bottomX;
    lastBottomY = bottomY;
    lastBottomZ = bottomZ;

    if (!positions.length) {
        initRope(topX, topY, topZ);
    }

    // If were sleeping, and inputs moved → wake up
    if (isSleeping && inputsMoved) {
        isSleeping   = false;
        sleepCounter = 0;
        for (let i = 0; i < positions.length; i++) {
            prevPositions[i].x = positions[i].x;
            prevPositions[i].y = positions[i].y;
            prevPositions[i].z = positions[i].z;
        }
        lastTime = performance.now();
        dt = 0;
    }

    let maxVel = 0;

    if (!isSleeping) {
        maxVel = step(dt, topX, topY, topZ, bottomX, bottomY, bottomZ);

        if (!inputsMoved && maxVel < VEL_EPS) {
            sleepCounter++;
            if (sleepCounter >= SLEEP_FRAMES) {
                isSleeping = true;
                for (let i = 0; i < positions.length; i++) {
                    prevPositions[i].x = positions[i].x;
                    prevPositions[i].y = positions[i].y;
                    prevPositions[i].z = positions[i].z;
                }
            }
        } else {
            sleepCounter = 0;
        }
    }

    // Export [x0,y0,z0, x1,y1,z1, ...]
    const flat = [];
    for (let i = 0; i < positions.length; i++)
    {
        const p = positions[i];
        flat.push(p.x, p.y, p.z);
    }
    outPoints.set(flat);
};
