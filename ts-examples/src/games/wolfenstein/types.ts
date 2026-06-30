export enum TextureType {
    Wall = 0,
    Sprite = 1,
    Weapon = 2,
}

export interface Entity {
    x: number;
    y: number;
    tex: number;
    scale: number;
    active: boolean;
    type: 'potion' | 'enemy' | 'bullet';
    vx?: number;
    vy?: number;
    owner?: 'player' | 'enemy';
}


export interface EnemyConfig {
    maxHealth: number;
    moveSpeed: number;
    visionRange: number;
    attackRange: number;
    attackCooldown: number;
    attackDamage: number;
    fireFrameIndex: number;
    textures: {
        idle: number;
        walk: number[];
        attack: number[];
    };
}


export interface EnemyEntity extends Entity {
    type: 'enemy';
    health: number;
    state: 'IDLE' | 'WANDER' | 'CHASE' | 'ATTACK';
    angle: number;
    stateTimer: number;
    wanderTimer: number;
    animationFrame: number;
    config: EnemyConfig;
}

export interface Door {
    x: number;
    y: number;
    openAmount: number;
    state: 'opening' | 'open' | 'closing';
    timer: number;
}

