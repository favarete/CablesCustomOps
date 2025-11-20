/*
author: Rodrigo Favarete
email: rodrigo@favarete.art
description: Rope2D operator with physics simulation
version: 1.0
*/

const inUpdate      = op.inTrigger("Update");

// Top Points
const inTopX        = op.inFloat("Top X", 0.0);
const inTopY        = op.inFloat("Top Y", 1.5);

// Lower Points
const inBottomX     = op.inFloat("Bottom X", 0.0);
const inBottomY     = op.inFloat("Bottom Y", -0.5);

// String parameters
const inSegments    = op.inInt("Segments", 10);        // number of segments
const inLength      = op.inFloat("Rope Length", 2.0);  // total length
const inGravity     = op.inFloat("Gravity", -9.8);
const inIter        = op.inInt("Constraint Iterations", 8);
const inDamping     = op.inFloat("Damping", 0.94);     // 0.9 ~ 0.97

// Output in Array3 (xyzxyz...)
const outPoints     = op.outArray("Points", null, 3);

// Internal state
let positions      = [];     // [{x,y}, ...]
let prevPositions  = [];     // [{x,y}, ...]
let lastTime       = performance.now();

// Sleep
let isSleeping     = false;
let sleepCounter   = 0;
const SLEEP_FRAMES = 6;      // How many frames almost stopped to sleep
const VEL_EPS      = 1e-3;   // Minimum speed to consider "stopped"

// To detect if inputs changed
let lastTopX       = null;
let lastTopY       = null;
let lastBottomX    = null;
let lastBottomY    = null;
const INPUT_EPS    = 1e-5;

function initRope(topX, topY)
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
        positions.push({ x, y });
        prevPositions.push({ x, y });
    }

    isSleeping   = false;
    sleepCounter = 0;
}

inSegments.onChange = function () {
    const topX = inTopX.get();
    const topY = inTopY.get();
    initRope(topX, topY);
};
inLength.onChange   = function () {
    const topX = inTopX.get();
    const topY = inTopY.get();
    initRope(topX, topY);
};
inTopX.onChange     = function () {
    const topX = inTopX.get();
    const topY = inTopY.get();
    initRope(topX, topY);
};
inTopY.onChange     = function () {
    const topX = inTopX.get();
    const topY = inTopY.get();
    initRope(topX, topY);
};

function step(dt, topX, topY, bottomX, bottomY)
{
    const n = positions.length - 1;
    if (n < 1) return 0;

    const g       = inGravity.get();
    const L       = inLength.get();
    const restLen = L / n;
    const numIterations   = Math.max(1, inIter.get() | 0);
    const damping = inDamping.get();

    let maxVel = 0;

    const bottom = positions[n];
    bottom.x = bottomX;
    bottom.y = bottomY;
    prevPositions[n].x = bottom.x;
    prevPositions[n].y = bottom.y;

    // Verlet Integration with damping for intermediate points
    for (let i = 1; i < n; i++) // 0 = topo, n = base
    {
        const p    = positions[i];
        const prev = prevPositions[i];

        const oldX = p.x;
        const oldY = p.y;

        let vx = (p.x - prev.x) * damping;
        let vy = (p.y - prev.y) * damping;

        prev.x = oldX;
        prev.y = oldY;

        // position = position + velocity + gravity*dt²
        p.x += vx;
        p.y += vy + g * dt * dt;

        const vMag = Math.max(Math.abs(vx), Math.abs(vy));
        if (vMag > maxVel) maxVel = vMag;
    }

    // Top
    positions[0].x     = topX;
    positions[0].y     = topY;
    prevPositions[0].x = positions[0].x;
    prevPositions[0].y = positions[0].y;

    // Iteration length constraints (makes less elastic)
    for (let iter = 0; iter < numIterations; iter++)
    {
        for (let i = 0; i < n; i++)
        {
            const p1 = positions[i];
            const p2 = positions[i + 1];

            let dx   = p2.x - p1.x;
            let dy   = p2.y - p1.y;
            let dist = Math.hypot(dx, dy);
            if (!dist) continue;

            const diff = (dist - restLen) / dist;

            if (i === 0)
            {
                p2.x -= dx * diff;
                p2.y -= dy * diff;
            }
            else if (i === n - 1)
            {
                p1.x += dx * diff;
                p1.y += dy * diff;
            }
            else
            {
                const half = diff * 0.5;
                p1.x += dx * half;
                p1.y += dy * half;
                p2.x -= dx * half;
                p2.y -= dy * half;
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
    const bottomX = inBottomX.get();
    const bottomY = inBottomY.get();

    // Detect if inputs (top/base) moved since the last frame
    const inputsMoved =
        lastTopX    === null ||
        Math.abs(topX    - lastTopX)    > INPUT_EPS ||
        Math.abs(topY    - lastTopY)    > INPUT_EPS ||
        Math.abs(bottomX - lastBottomX) > INPUT_EPS ||
        Math.abs(bottomY - lastBottomY) > INPUT_EPS;

    lastTopX    = topX;
    lastTopY    = topY;
    lastBottomX = bottomX;
    lastBottomY = bottomY;

    if (!positions.length) {
        initRope(topX, topY);
    }

    // If sleeping and some input moved → wake up
    if (isSleeping && inputsMoved) {
        isSleeping   = false;
        sleepCounter = 0;
        // No speed (prev = pos)
        for (let i = 0; i < positions.length; i++) {
            prevPositions[i].x = positions[i].x;
            prevPositions[i].y = positions[i].y;
        }
        // Avoids giant dt in the first frame awake
        lastTime = performance.now();
        dt = 0;
    }

    let maxVel = 0;

    // Only simulate physics if not sleeping
    if (!isSleeping) {
        maxVel = step(dt, topX, topY, bottomX, bottomY);

        // If the inputs did NOT move and the speed is very low for some frames, sleep
        if (!inputsMoved && maxVel < VEL_EPS) {
            sleepCounter++;
            if (sleepCounter >= SLEEP_FRAMES) {
                isSleeping = true;
                // freezes speeds
                for (let i = 0; i < positions.length; i++) {
                    prevPositions[i].x = positions[i].x;
                    prevPositions[i].y = positions[i].y;
                }
            }
        } else {
            sleepCounter = 0;
        }
    }
    // If sleeping, simply maintain positions as they are

    // Export [x0,y0,0, x1,y1,0, ...]
    const flat = [];
    for (let i = 0; i < positions.length; i++)
    {
        const p = positions[i];
        flat.push(p.x, p.y, 0);
    }
    outPoints.set(flat);
};
