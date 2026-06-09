import { worldMap } from "./config.js";
import { EnemyEntity } from "./types.js";
import { spawnBullet, tryMove } from "./wolfenstein.js";


function checkLineOfSight(ex: number, ey: number, px: number, py: number): boolean {
    let x0 = Math.floor(ex);
    let y0 = Math.floor(ey);
    const x1 = Math.floor(px);
    const y1 = Math.floor(py);

    if (x1 < 0 || x1 >= worldMap.length || y1 < 0 || y1 >= worldMap[0].length) {
        return false;
    }

    const dx = Math.abs(px - ex);
    const dy = Math.abs(py - ey);
    const sx = ex < px ? 1 : -1;
    const sy = ey < py ? 1 : -1;

    let err = dx - dy;
    let iterations = 0;
    const MAX_ITERATIONS = 256;

    while (x0 !== x1 || y0 !== y1) {
        iterations++;
        if (iterations > MAX_ITERATIONS) return false;

        if (x0 < 0 || x0 >= worldMap.length || y0 < 0 || y0 >= worldMap[0].length) {
            return false;
        }

        if ((x0 !== Math.floor(ex) || y0 !== Math.floor(ey)) &&
            (x0 !== x1 || y0 !== y1)) {
            if (worldMap[x0][y0] > 0) {
                return false;
            }
        }

        const e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
    return true;
}

export function updateEnemyAI(enemy: EnemyEntity, playerX: number, playerY: number) {
    const dx = playerX - enemy.x;
    const dy = playerY - enemy.y;
    const dist = Math.sqrt(dx * dx + dy * dy);
    const angleToPlayer = Math.atan2(dy, dx);

    const hasLOS = checkLineOfSight(enemy.x, enemy.y, playerX, playerY);
    if (enemy.state === 'IDLE' || enemy.state === 'WANDER') {
        if (hasLOS && dist < enemy.config.visionRange) enemy.state = 'CHASE';
        else if (enemy.state === 'IDLE' && Math.random() < 0.01) {
            enemy.state = 'WANDER';
            enemy.angle = Math.random() * Math.PI * 2;
            enemy.wanderTimer = 60 + Math.random() * 100;
        }
    } else if (enemy.state === 'CHASE') {
        if (dist < enemy.config.attackRange) {
            enemy.state = 'ATTACK';
            enemy.stateTimer = 0;
        } else if (!hasLOS || dist > enemy.config.visionRange * 1.2) {
            enemy.state = 'IDLE';
        }
    } else if (enemy.state === 'ATTACK') {
        if (dist > enemy.config.attackRange) enemy.state = 'CHASE';
    }

    enemy.animationFrame++;

    switch (enemy.state) {
        case 'IDLE':
            enemy.tex = enemy.config.textures.idle;
            break;

        case 'WANDER':
            const walkFrames = enemy.config.textures.walk;
            enemy.tex = walkFrames[Math.floor(enemy.animationFrame / 10) % walkFrames.length];

            const wx = enemy.x + Math.cos(enemy.angle) * enemy.config.moveSpeed;
            const wy = enemy.y + Math.sin(enemy.angle) * enemy.config.moveSpeed;
            if (tryMove(wx, wy)) { enemy.x = wx; enemy.y = wy; }
            else { enemy.state = 'IDLE'; }

            enemy.wanderTimer--;
            if (enemy.wanderTimer <= 0) enemy.state = 'IDLE';
            break;

        case 'CHASE':
            const chaseFrames = enemy.config.textures.walk;
            enemy.tex = chaseFrames[Math.floor(enemy.animationFrame / 10) % chaseFrames.length];

            enemy.angle = angleToPlayer;
            const cx = enemy.x + Math.cos(enemy.angle) * enemy.config.moveSpeed;
            const cy = enemy.y + Math.sin(enemy.angle) * enemy.config.moveSpeed;
            if (tryMove(cx, cy)) { enemy.x = cx; enemy.y = cy; }
            break;


        case 'ATTACK':
            const attackFrames = enemy.config.textures.attack;

            const currentFrameIdx = Math.floor(enemy.stateTimer / enemy.config.attackCooldown) % attackFrames.length;
            enemy.tex = attackFrames[currentFrameIdx];

            if (currentFrameIdx === enemy.config.fireFrameIndex &&
                (enemy.stateTimer % enemy.config.attackCooldown === 0)) {

                spawnBullet(enemy.x, enemy.y, Math.cos(enemy.angle), Math.sin(enemy.angle), 'enemy');
            }

            enemy.stateTimer++;

            if (enemy.stateTimer >= attackFrames.length * enemy.config.attackCooldown) {
                enemy.state = 'CHASE';
            }
            break;
    }
}
