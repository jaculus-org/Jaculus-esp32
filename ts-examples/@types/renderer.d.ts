declare module "shapes" {
    import { Texture } from "renderer";
    // Packed 24-bit RGB, e.g. 0xff0000 for red.
    export type Color = number;

    export interface ShapeParams {
        x: number;
        y: number;
        z?: number;
    }

    /**
     * A 2D affine transformation matrix:
     * ```
     * | a  c  e |
     * | b  d  f |
     * | 0  0  1 |
     * ```
     * `a`/`d` are the X/Y axis scale (and rotation/skew combined with `b`/`c`),
     * `e`/`f` are the translation.
     */
    export interface Matrix2D {
        a: number;
        b: number;
        c: number;
        d: number;
        e: number;
        f: number;
    }

    export class Shape {
        /**
         * Set the absolute position of the shape.
         * @param x The new x coordinate.
         * @param y The new y coordinate.
         */
        setPosition(x: number, y: number): void;

        /**
         * Move the shape relative to its current position.
         * @param dx The x offset.
         * @param dy The y offset.
         */
        translate(dx: number, dy: number): void;

        /**
         * Rotate the shape around its current pivot.
         * @param angle The angle in degrees.
         */
        rotate(angle: number): void;

        /**
         * Set the pivot used for subsequent rotations.
         * @param x The pivot x coordinate.
         * @param y The pivot y coordinate.
         */
        setPivot(x: number, y: number): void;

        /**
         * Set scale factors and optionally the scale origin.
         * @param scaleX The horizontal scale factor.
         * @param scaleY The vertical scale factor.
         * @param originX Optional x coordinate of the scale origin.
         * @param originY Optional y coordinate of the scale origin.
         */
        setScale(scaleX: number, scaleY: number, originX?: number, originY?: number): void;

        /**
         * Replace the position/rotation/scale transform with a fixed matrix.
         * Overrides setPosition/translate/rotate/setScale until cleared with
         * clearTransformationMatrix(). Useful for skew or externally-driven
         * animation that doesn't fit the built-in transform.
         * @param matrix The matrix to use for this shape's local transform.
         */
        setTransformationMatrix(matrix: Matrix2D): void;

        /**
         * Remove a previously set transformation matrix, reverting to the
         * position/rotation/scale transform.
         */
        clearTransformationMatrix(): void;

        /**
         * Set the z order used during rendering.
         * @param z The new z value.
         */
        setZ(z: number): void;

        /**
         * Set the x coordinate.
         * @param x The new x coordinate.
         */
        setX(x: number): void;

        /**
         * Set the y coordinate.
         * @param y The new y coordinate.
         */
        setY(y: number): void;

        /**
         * Attach a texture used when sampling the shape surface.
         * @param texture The texture to use.
         */
        setTexture(texture: Texture): void;

        /**
         * Keep texture coordinates fixed in world space.
         * @param fixed Whether to keep texture coordinates fixed.
         */
        setFixTexture(fixed: boolean): void;

        /**
         * Rotate texture sampling coordinates.
         * @param rotation The angle in degrees.
         */
        setTextureRotation(rotation: number): void;

        /**
         * Offset texture sampling coordinates.
         * @param offsetX The horizontal texture offset.
         * @param offsetY The vertical texture offset.
         */
        setTextureOffset(offsetX: number, offsetY: number): void;

        /**
         * Scale texture sampling coordinates.
         * @param scaleX The horizontal texture scale factor.
         * @param scaleY The vertical texture scale factor.
         */
        setTextureScale(scaleX: number, scaleY: number): void;

        /**
         * Set the current rotation angle directly.
         * @param angle The angle in degrees.
         */
        setRotationAngle(angle: number): void;

        /**
         * Set the horizontal scale factor.
         * @param scaleX The new horizontal scale factor.
         */
        setScaleX(scaleX: number): void;

        /**
         * Set the vertical scale factor.
         * @param scaleY The new vertical scale factor.
         */
        setScaleY(scaleY: number): void;

        /**
         * Set the horizontal UV scale factor.
         * @param scaleX The new horizontal UV scale factor.
         */
        setUVScaleX(scaleX: number): void;

        /**
         * Set the vertical UV scale factor.
         * @param scaleY The new vertical UV scale factor.
         */
        setUVScaleY(scaleY: number): void;

        /**
         * Set the horizontal UV offset.
         * @param offsetX The new horizontal UV offset.
         */
        setUVOffsetX(offsetX: number): void;

        /**
         * Set the vertical UV offset.
         * @param offsetY The new vertical UV offset.
         */
        setUVOffsetY(offsetY: number): void;

        /**
         * Set the UV rotation.
         * @param rotation The angle in degrees.
         */
        setUVRotation(rotation: number): void;

        /**
         * Get the current x coordinate.
         * @returns The current x coordinate.
         */
        getX(): number;

        /**
         * Get the current y coordinate.
         * @returns The current y coordinate.
         */
        getY(): number;

        /**
         * Get the current z order.
         * @returns The current z value.
         */
        getZ(): number;

        /**
         * Get the current rotation angle.
         * @returns The current angle in degrees.
         */
        getRotationAngle(): number;

        /**
         * Get the horizontal scale factor.
         * @returns The horizontal scale factor.
         */
        getScaleX(): number;

        /**
         * Get the vertical scale factor.
         * @returns The vertical scale factor.
         */
        getScaleY(): number;

        /**
         * Test whether this shape intersects another shape.
         * @param other The other shape to test against.
         * @returns True if the shapes intersect, false otherwise.
         */
        intersects(other: Shape): boolean;
    }

    export interface Colorable {
        /**
         * Set the color used for rendering the shape.
         * @param color The new shape color.
         */
        setColor(color: Color): void;

        /**
         * Get the current shape color.
         * @returns The current color.
         */
        getColor(): Color;
    }

    export class Collection extends Shape {
        constructor(params: ShapeParams);

        /**
         * Add a child shape to the collection.
         * @param shape The shape to add.
         */
        add(shape: Shape): void;

        /**
         * Remove all child shapes from the collection.
         */
        clear(): void;

        /**
         * Remove a child shape from the collection.
         * @param shape The shape to remove.
         */
        remove(shape: Shape): void;
    }

    export interface CircleParams extends ShapeParams {
        color: Color;
        radius: number;
        fill?: boolean;
    }

    /**
     * A circle. Applying a non-uniform scale (setScale/setScaleX/setScaleY
     * with differing x/y factors) renders it as an ellipse.
     */
    export class Circle extends Shape implements Colorable {
        constructor(params: CircleParams);
        setColor(color: Color): void;
        getColor(): Color;
        setRadius(radius: number): void;
        getRadius(): number;
        setFill(fill: boolean): void;
        getFill(): boolean;
    }

    export interface RectangleParams extends ShapeParams {
        color: Color;
        width: number;
        height: number;
        fill?: boolean;
    }

    export class Rectangle extends Shape implements Colorable {
        constructor(params: RectangleParams);
        setColor(color: Color): void;
        getColor(): Color;
        setWidth(width: number): void;
        getWidth(): number;
        setHeight(height: number): void;
        getHeight(): number;
        setFill(fill: boolean): void;
        getFill(): boolean;
    }

    export interface PolygonParams extends ShapeParams {
        color: Color;
        vertices: [number, number][];
        fill?: boolean;
    }

    export class Polygon extends Shape implements Colorable {
        constructor(params: PolygonParams);
        setColor(color: Color): void;
        getColor(): Color;
        setVertices(vertices: [number, number][]): void;
        getVertices(): [number, number][];
        setFill(fill: boolean): void;
        getFill(): boolean;
    }

    export interface LineSegmentParams extends ShapeParams {
        color: Color;
        x2: number;
        y2: number;
    }

    export class LineSegment extends Shape implements Colorable {
        constructor(params: LineSegmentParams);
        setColor(color: Color): void;
        getColor(): Color;
        setEndpoint(x2: number, y2: number): void;
        getX2(): number;
        getY2(): number;
    }

    export interface PointParams extends ShapeParams {
        color: Color;
    }

    export class Point extends Shape implements Colorable {
        constructor(params: PointParams);
        setColor(color: Color): void;
        getColor(): Color;
    }

    export interface RegularPolygonRadiusParams extends ShapeParams {
        color: Color;
        sides: number;
        radius: number;
        fill?: boolean;
    }

    export interface RegularPolygonSideParams extends ShapeParams {
        color: Color;
        sides: number;
        sideLength: number;
        fill?: boolean;
    }

    export class RegularPolygon extends Shape implements Colorable {
        constructor(params: RegularPolygonRadiusParams | RegularPolygonSideParams);
        setColor(color: Color): void;
        getColor(): Color;
        setSides(sides: number): void;
        getSides(): number;
        setRadius(radius: number): void;
        getRadius(): number;
        setFill(fill: boolean): void;
        getFill(): boolean;
    }
}

declare module "renderer" {
    import { Collection, Color } from "shapes";

    export class Texture {
        constructor();

        /**
         * Load a BMP texture from the given path.
         * @param path The path to the BMP file.
         * @returns True if the texture was loaded successfully.
         */
        load(path: string): boolean;

        /**
         * Set texture wrap mode.
         * @param mode The wrap mode to use.
         */
        setWrapMode(mode: "repeat" | "clamp" | "mirror"): void;

        /**
         * Get the texture width in pixels.
         * @returns The texture width.
         */
        getWidth(): number;

        /**
         * Get the texture height in pixels.
         * @returns The texture height.
         */
        getHeight(): number;

        /**
         * Check whether the texture contains valid image data.
         * @returns Non-zero when the texture is valid.
         */
        isValid(): number;
    }
    export class Font {
        constructor();

        /**
         * Get the font height in pixels.
         * @returns The font height.
         */
        getHeight(): number;

        /**
         * Get the width of a character glyph.
         * @param char The character to measure.
         * @returns The glyph width.
         */
        getCharWidth(char: string): number;

        /**
         * Get spacing added after a character.
         * @param char The character to measure.
         * @returns The spacing after the character.
         */
        getCharSpacing(char: string): number;
    }

    export class Renderer {
        /**
         * @param width The panel width in pixels.
         * @param height The panel height in pixels.
         * @param format The output pixel format used by render() and drawText().
         * @param rotation Rotates the whole image by 90 degree increments.
         */
        constructor(width: number, height: number, format?: Format, rotation?: number);

        /**
         * Render a scene into the provided buffer.
         * @param scene The collection to render.
         * @param buffer The output pixel buffer.
         * @param antialias Whether to enable antialiasing.
         * @returns The number of bytes written.
         */
        render(scene: Collection, buffer: ArrayBuffer, antialias?: boolean): number;

        /**
         * Draw text into the provided buffer.
         * @param buffer The output pixel buffer.
         * @param text The text to draw.
         * @param x The starting x coordinate.
         * @param y The starting y coordinate.
         * @param font The font to use.
         * @param color The text color.
         * @param wrap Whether to wrap lines to the renderer width.
         * @param rotation Rotates the text by 90 degree increments, relative to the rotation set in the constructor.
         * @returns The number of bytes written.
         */
        drawText(buffer: ArrayBuffer, text: string, x: number, y: number, font: Font, color: Color, wrap: boolean, rotation?: number): number;
    }

    // https://419.ecma-international.org/3.0/index.html#-15-display-class-pattern-pixel-format-values
    export enum Format {
        MONOCHROME = 3,
        GRAYSCALE_4_BIT = 4,
        GRAYSCALE_8_BIT = 5,
        RGB_332 = 6,
        RGB_565_LITTLE = 7,
        RGB_565_BIG = 8,
        RGB_888 = 9,
        RGBA_8888 = 10,
        XRGB = 12,
    }
}
