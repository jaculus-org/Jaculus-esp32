declare module "raycaster" {

    export class Raycaster {
        /**
         * Creates a new Raycaster instance.
         * @param width The horizontal resolution of the render target.
         * @param height The vertical resolution of the render target.
         */
        constructor(width: number, height: number);

        setTileConfig(walls: number[], doors_ns: number[], doors_ew: number[]): void;

        setFloorMap(map: number[][]): void;
        setCeilingMap(map: number[][]): void;

        /**
         * Unified loader for all visual assets.
         * @param type Use TextureType.Wall or TextureType.Sprite to avoid ID conflicts.
         */
        setTexture(id: number, data: ArrayBuffer, w: number, h: number, type: import('../src/games/wolfenstein/types.js').TextureType): void;
        /**
         * Sets the 2D grid map for the raycaster.
         * @param map A 2D array of integers where 0 represents empty space 
         * and positive integers represent different wall types/colors.
         */
        setMap(map: number[][]): void;

        /**
         * Performs the raycasting DDA algorithm and writes the frame directly into the provided buffer.
         * * @param buffer The ArrayBuffer to write pixel data into.
         * @param posX The player's current X position on the map.
         * @param posY The player's current Y position on the map.
         * @param dirX The X component of the player's direction vector.
         * @param dirY The Y component of the player's direction vector.
         * @param planeX The X component of the camera plane (determines FOV).
         * @param planeY The Y component of the camera plane (determines FOV).
         * @param format The output pixel format (e.g., Format.RGB_565_LITTLE).
         * @returns The number of bytes written to the buffer.
         */

        render(
            buffer: ArrayBuffer,
            posX: number, posY: number,
            dirX: number, dirY: number,
            planeX: number, planeY: number,
            sprites: number[],
            doorData: number[],
            weaponFrame: number,
            format: number,
        ): number;
    }
}
