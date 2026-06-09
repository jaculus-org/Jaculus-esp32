#pragma once

#include "jac/device/logger.h"
#include "jac/machine/class.h"
#include "jac/machine/functionFactory.h"
#include "jac/machine/internal/declarations.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

enum class TextureType { Wall = 0, Sprite = 1, Weapon = 2 };

struct RaycasterTexture {
    int width;
    int height;
    std::vector<uint16_t> pixels;
};

struct RenderSprite {
    float x;
    float y;
    int tex;
    float scale;
    float dist;
};

extern size_t packedColorSize(int format);

class Raycaster {
  private:
    std::vector<std::vector<int>> m_map;
    int m_width;
    int m_height;

    std::unordered_map<int, RaycasterTexture> m_wallTextures;
    std::unordered_map<int, RaycasterTexture> m_spriteTextures;
    std::unordered_map<int, RaycasterTexture> m_weaponTextures;

    std::vector<std::vector<uint16_t>> m_floorMap;
    std::vector<std::vector<uint16_t>> m_ceilingMap;
    std::vector<float> m_rowDistTable;

    std::vector<float> m_zBuffer;
    std::vector<float> m_doorStatesFlat;
    std::vector<RenderSprite> m_spriteList;

    std::vector<int> m_wallTypes = {1, 2, 3};
    std::vector<int> m_doorNSTypes = {4};
    std::vector<int> m_doorEWTypes = {5};

    int m_mapWidth = 0;
    int m_mapHeight = 0;

    bool isWall(int tile) const {
        return std::find(m_wallTypes.begin(), m_wallTypes.end(), tile) !=
               m_wallTypes.end();
    }
    bool isDoorNS(int tile) const {
        return std::find(m_doorNSTypes.begin(), m_doorNSTypes.end(), tile) !=
               m_doorNSTypes.end();
    }
    bool isDoorEW(int tile) const {
        return std::find(m_doorEWTypes.begin(), m_doorEWTypes.end(), tile) !=
               m_doorEWTypes.end();
    }

    inline void drawPixel(uint8_t *raw, int logicalX, int logicalY,
                          uint16_t color, int format, size_t bpp) {
        if (!raw)
            return;

        int pixelX = logicalY;
        int pixelY = logicalX;

        if (pixelX >= 0 && pixelX < m_width && pixelY >= 0 &&
            pixelY < m_height) {
            size_t offset = (size_t)(pixelY * m_width + pixelX) * bpp;

            if (format == 8) {
                raw[offset] = color >> 8;
                raw[offset + 1] = color & 0xFF;
            } else {
                raw[offset] = color & 0xFF;
                raw[offset + 1] = color >> 8;
            }
        }
    }

    void renderWalls(uint8_t *raw, float posX, float posY, float dirX,
                     float dirY, float planeX, float planeY,
                     const std::vector<float> &doorData, int format,
                     size_t bpp) {

        std::fill(m_doorStatesFlat.begin(), m_doorStatesFlat.end(), 0.0f);

        for (size_t i = 0; i + 2 < doorData.size(); i += 3) {
            int dx = (int)doorData[i];
            int dy = (int)doorData[i + 1];
            if (dx >= 0 && dx < m_mapWidth && dy >= 0 && dy < m_mapHeight) {
                m_doorStatesFlat[dx * m_mapHeight + dy] = doorData[i + 2];
            }
        }

        for (int x = 0; x < m_width; x++) {
            float cameraX = 2.0f * x / (float)m_width - 1.0f;
            float rayDirX = dirX + planeX * cameraX;
            float rayDirY = dirY + planeY * cameraX;

            int mapX = (int)posX;
            int mapY = (int)posY;
            float deltaDistX = std::abs(1.0f / rayDirX);
            float deltaDistY = std::abs(1.0f / rayDirY);
            float sideDistX, sideDistY;
            int stepX, stepY, hit = 0, side = 0;

            if (rayDirX < 0) {
                stepX = -1;
                sideDistX = (posX - mapX) * deltaDistX;
            } else {
                stepX = 1;
                sideDistX = (mapX + 1.0f - posX) * deltaDistX;
            }
            if (rayDirY < 0) {
                stepY = -1;
                sideDistY = (posY - mapY) * deltaDistY;
            } else {
                stepY = 1;
                sideDistY = (mapY + 1.0f - posY) * deltaDistY;
            }

            float perpWallDist = 0;
            bool hitThinWall = false;

            while (hit == 0) {
                if (sideDistX < sideDistY) {
                    sideDistX += deltaDistX;
                    mapX += stepX;
                    side = 0;
                } else {
                    sideDistY += deltaDistY;
                    mapY += stepY;
                    side = 1;
                }

                if (mapX >= 0 && mapX < m_mapWidth && mapY >= 0 &&
                    mapY < m_mapHeight) {
                    int tile = m_map[mapX][mapY];

                    if (isWall(tile)) {
                        hit = 1;
                    } else if (isDoorNS(tile) && side == 0) {
                        float doorDist =
                            (mapX - posX + (1 - stepX) / 2.0f + stepX * 0.5f) /
                            rayDirX;
                        float hitY = posY + doorDist * rayDirY;
                        if ((int)hitY == mapY &&
                            (hitY - std::floor(hitY)) >
                                m_doorStatesFlat[mapX * m_mapHeight + mapY]) {
                            hit = 1;
                            hitThinWall = true;
                            perpWallDist = doorDist;
                        }
                    } else if (isDoorEW(tile) && side == 1) {
                        float doorDist =
                            (mapY - posY + (1 - stepY) / 2.0f + stepY * 0.5f) /
                            rayDirY;
                        float hitX = posX + doorDist * rayDirX;
                        if ((int)hitX == mapX &&
                            (hitX - std::floor(hitX)) >
                                m_doorStatesFlat[mapX * m_mapHeight + mapY]) {
                            hit = 1;
                            hitThinWall = true;
                            perpWallDist = doorDist;
                        }
                    }
                } else {
                    hit = 1;
                }
            }

            if (!hitThinWall) {
                perpWallDist =
                    (side == 0) ? (mapX - posX + (1 - stepX) / 2.0f) / rayDirX
                                : (mapY - posY + (1 - stepY) / 2.0f) / rayDirY;
            }

            if (x >= 0 && x < (int)m_zBuffer.size()) {
                m_zBuffer[x] = perpWallDist;
            }

            float wallX_world, wallY_world;
            if (side == 0) {
                wallX_world = mapX + (stepX > 0 ? 0.0f : 1.0f);
                wallY_world = posY + perpWallDist * rayDirY;
            } else {
                wallX_world = posX + perpWallDist * rayDirX;
                wallY_world = mapY + (stepY > 0 ? 0.0f : 1.0f);
            }

            int lineHeight = (int)(m_height / perpWallDist);
            int drawStart = std::max(0, -lineHeight / 2 + m_height / 2);
            int drawEnd = std::min(m_height - 1, lineHeight / 2 + m_height / 2);

            float wallX = (side == 0) ? (posY + perpWallDist * rayDirY)
                                      : (posX + perpWallDist * rayDirX);
            wallX -= std::floor(wallX);

            int tile = 0;
            if (mapX >= 0 && mapX < m_mapWidth && mapY >= 0 &&
                mapY < m_mapHeight) {
                tile = m_map[mapX][mapY];
            }

            if (isDoorNS(tile) || isDoorEW(tile)) {
                float doorOffset = m_doorStatesFlat[mapX * m_mapHeight + mapY];
                wallX -= doorOffset;
                if (wallX < 0.0f)
                    wallX += 1.0f;
            }

            const RaycasterTexture *activeTex = nullptr;
            int texX = 0;

            if (tile > 0 && m_wallTextures.count(tile)) {
                activeTex = &m_wallTextures[tile];
                texX = std::clamp((int)(wallX * activeTex->width), 0,
                                  activeTex->width - 1);
            }

            float step =
                activeTex ? (1.0f * activeTex->height / lineHeight) : 0.0f;
            float texPos =
                (drawStart - m_height / 2.0f + lineHeight / 2.0f) * step;

            for (int y = 0; y < m_height; y++) {
                if (y >= drawStart && y <= drawEnd) {
                    uint16_t color = 0x7BEF;

                    if (activeTex) {
                        int texY =
                            std::clamp((int)texPos, 0, activeTex->height - 1);
                        texPos += step;

                        size_t pixelIdx =
                            (size_t)texY * activeTex->width + texX;
                        if (pixelIdx < activeTex->pixels.size()) {
                            color = activeTex->pixels[pixelIdx];
                        }
                    }

                    if (side == 1)
                        color = (color >> 1) & 0x7BEF;

                    drawPixel(raw, x, y, color, format, bpp);
                } else {
                    float rowDistance = m_rowDistTable[y];
                    float weight = (perpWallDist > 0.0f)
                                       ? (rowDistance / perpWallDist)
                                       : 0.0f;
                    if (weight > 1.0f)
                        weight = 1.0f;

                    float currentFloorX =
                        weight * wallX_world + (1.0f - weight) * posX;
                    float currentFloorY =
                        weight * wallY_world + (1.0f - weight) * posY;

                    int cellX =
                        std::clamp((int)currentFloorX, 0, m_mapWidth - 1);
                    int cellY =
                        std::clamp((int)currentFloorY, 0, m_mapHeight - 1);

                    if (y > drawEnd) {
                        uint16_t floorColor = 0x2104; // Default
                        if (!m_floorMap.empty() && cellX < m_floorMap.size() &&
                            cellY < m_floorMap[0].size()) {
                            floorColor = m_floorMap[cellX][cellY];
                        }
                        drawPixel(raw, x, y, floorColor, format, bpp);
                    } else {
                        uint16_t ceilColor = 0x0000; // Default
                        if (!m_ceilingMap.empty() &&
                            cellX < m_ceilingMap.size() &&
                            cellY < m_ceilingMap[0].size()) {
                            ceilColor = m_ceilingMap[cellX][cellY];
                        }
                        drawPixel(raw, x, y, ceilColor, format, bpp);
                    }
                }
            }
        }
    }

    void renderSprites(uint8_t *raw, float posX, float posY, float dirX,
                       float dirY, float planeX, float planeY,
                       const std::vector<float> &spriteData, int format,
                       size_t bpp) {

        m_spriteList.clear();
        for (size_t i = 0; i + 3 < spriteData.size(); i += 4) {
            RenderSprite s;
            s.x = spriteData[i];
            s.y = spriteData[i + 1];
            s.tex = (int)spriteData[i + 2];
            s.scale = spriteData[i + 3];
            s.dist =
                ((posX - s.x) * (posX - s.x)) + ((posY - s.y) * (posY - s.y));
            m_spriteList.push_back(s);
        }

        std::sort(m_spriteList.begin(), m_spriteList.end(),
                  [](const RenderSprite &a, const RenderSprite &b) {
                      return a.dist > b.dist;
                  });

        for (const auto &sprite : m_spriteList) {
            float spriteDistX = sprite.x - posX;
            float spriteDistY = sprite.y - posY;
            float invDet = 1.0f / (planeX * dirY - dirX * planeY);
            float transformX =
                invDet * (dirY * spriteDistX - dirX * spriteDistY);
            float transformY =
                invDet * (-planeY * spriteDistX + planeX * spriteDistY);

            if (transformY <= 0)
                continue;

            int spriteScreenX =
                int((m_width / 2) * (1.0f + transformX / transformY));
            int baselineHeight = std::abs(int(m_height / transformY));
            int spriteHeight = baselineHeight * sprite.scale;
            int spriteWidth = baselineHeight * sprite.scale;

            int floorLevelY = (m_height / 2) + (baselineHeight / 2);
            int drawStartY = std::max(0, floorLevelY - spriteHeight);
            int drawEndY = std::min(m_height - 1, floorLevelY);
            int drawStartX = std::max(0, -spriteWidth / 2 + spriteScreenX);
            int drawEndX =
                std::min(m_width - 1, spriteWidth / 2 + spriteScreenX);

            if (m_spriteTextures.count(sprite.tex)) {
                const RaycasterTexture &tex = m_spriteTextures[sprite.tex];
                for (int stripe = drawStartX; stripe < drawEndX; stripe++) {
                    if (stripe >= 0 && stripe < (int)m_zBuffer.size() &&
                        transformY < m_zBuffer[stripe]) {
                        int texX = std::clamp(
                            ((stripe - (-spriteWidth / 2 + spriteScreenX)) *
                             tex.width) /
                                spriteWidth,
                            0, tex.width - 1);
                        for (int y = drawStartY; y < drawEndY; y++) {
                            int texY =
                                std::clamp(((y - (floorLevelY - spriteHeight)) *
                                            tex.height) /
                                               spriteHeight,
                                           0, tex.height - 1);

                            size_t pixelIdx = (size_t)texY * tex.width + texX;
                            if (pixelIdx < tex.pixels.size()) {
                                uint16_t color = tex.pixels[pixelIdx];
                                if (color != 0x0000)
                                    drawPixel(raw, stripe, y, color, format,
                                              bpp);
                            }
                        }
                    }
                }
            }
        }
    }

    // --- STAGE 3: Render Weapon ---

    void renderWeaponOverlay(uint8_t *raw, int weaponFrame, int format,
                             size_t bpp) {
        if (m_weaponTextures.find(weaponFrame) == m_weaponTextures.end())
            return;

        const RaycasterTexture &tex = m_weaponTextures[weaponFrame];

        int drawHeight = m_height / 2;
        int drawWidth = (tex.width * drawHeight) / tex.height;

        int startX = (m_width / 2) - (drawWidth / 2);
        int startY = m_height - drawHeight;

        if (weaponFrame == 2)
            startY += (m_height / 20);

        for (int y = 0; y < drawHeight; y++) {
            int screenY = startY + y;
            if (screenY < 0 || screenY >= m_height)
                continue;

            int texY = (y * tex.height) / drawHeight;

            for (int x = 0; x < drawWidth; x++) {
                int screenX = startX + x;
                if (screenX < 0 || screenX >= m_width)
                    continue;

                int texX = (x * tex.width) / drawWidth;

                size_t pixelIdx = (size_t)texY * tex.width + texX;
                if (pixelIdx < tex.pixels.size()) {
                    uint16_t color = tex.pixels[pixelIdx];
                    if (color != 0x0000) {
                        drawPixel(raw, screenX, screenY, color, format, bpp);
                    }
                }
            }
        }
    }

  public:
    Raycaster(int w, int h) {
        if (w <= 0 || w > 2048)
            w = 64;
        if (h <= 0 || h > 2048)
            h = 64;

        m_width = w;
        m_height = h;

        m_zBuffer.resize(w);
        m_spriteList.reserve(32);

        m_rowDistTable.resize(h);
        for (int y = 0; y < h; y++) {
            int p = y - h / 2;
            if (p > 0)
                m_rowDistTable[y] = (0.5f * h) / p;
            else if (p < 0)
                m_rowDistTable[y] = (0.5f * h) / -p;
            else
                m_rowDistTable[y] = 0.0f;
        }
    }

    void setTileConfig(const std::vector<int> &walls,
                       const std::vector<int> &doorsNS,
                       const std::vector<int> &doorsEW) {
        m_wallTypes = walls;
        m_doorNSTypes = doorsNS;
        m_doorEWTypes = doorsEW;
    }

    void setFloorMap(const std::vector<std::vector<uint16_t>> &map) {
        m_floorMap = map;
    }
    void setCeilingMap(const std::vector<std::vector<uint16_t>> &map) {
        m_ceilingMap = map;
    }

    void setMap(const std::vector<std::vector<int>> &map) {
        if (map.empty() || map[0].empty()) {
            jac::Logger::error("Raycaster: Received empty map!");
            return;
        }
        m_map = map;
        m_mapWidth = (int)map.size();
        m_mapHeight = (int)map[0].size();
        m_doorStatesFlat.assign(m_mapWidth * m_mapHeight, 0.0f);
        jac::Logger::debug("Raycaster: Map set to " +
                           std::to_string(m_mapWidth) + "x" +
                           std::to_string(m_mapHeight));
    }

    void setTexture(int id, uint8_t *data, size_t dataSize, int w, int h,
                    TextureType type) {
        if (!data) {
            jac::Logger::error("Raycaster: setTexture failed - null data");
            return;
        }
        if (w <= 0 || h <= 0 || w > 2048 || h > 2048) {
            jac::Logger::error("Raycaster: Invalid texture dimensions: " +
                               std::to_string(w) + "x" + std::to_string(h));
            return;
        }

        RaycasterTexture tex;
        tex.width = w;
        tex.height = h;
        tex.pixels.resize(w * h);

        size_t bytesToCopy =
            std::min(dataSize, (size_t)w * h * sizeof(uint16_t));
        std::memcpy(tex.pixels.data(), data, bytesToCopy);

        if (type == TextureType::Wall)
            m_wallTextures[id] = std::move(tex);
        else if (type == TextureType::Sprite)
            m_spriteTextures[id] = std::move(tex);
        else if (type == TextureType::Weapon)
            m_weaponTextures[id] = std::move(tex);

        jac::Logger::debug("Raycaster: Loaded texture ID " +
                           std::to_string(id));
    }

    size_t render(uint8_t *raw, size_t maxBytes, float posX, float posY,
                  float dirX, float dirY, float planeX, float planeY,
                  const std::vector<float> &spriteData,
                  const std::vector<float> &doorData, int weaponFrame,
                  int format) {

        if (!raw) {
            jac::Logger::error(
                "Raycaster: render failed - raw buffer is null!");
            return 0;
        }

        size_t bpp = 2;
        if (m_map.empty())
            return 0;

        size_t requiredBytes = (size_t)m_width * m_height * bpp;
        if (requiredBytes > maxBytes) {
            jac::Logger::error("Raycaster: Buffer too small! Req: " +
                               std::to_string(requiredBytes) +
                               " Max: " + std::to_string(maxBytes));
            return 0;
        }

        renderWalls(raw, posX, posY, dirX, dirY, planeX, planeY, doorData,
                    format, bpp);
        renderSprites(raw, posX, posY, dirX, dirY, planeX, planeY, spriteData,
                      format, bpp);
        renderWeaponOverlay(raw, weaponFrame, format, bpp);

        return requiredBytes;
    }
};

class RaycasterProtoBuilder : public jac::ProtoBuilder::Opaque<Raycaster>,
                              public jac::ProtoBuilder::Properties {
  public:
    static Raycaster *constructOpaque(jac::ContextRef ctx,
                                      std::vector<jac::ValueWeak> args) {
        int w = args.size() > 0 ? args[0].to<int>() : 64;
        int h = args.size() > 1 ? args[1].to<int>() : 64;
        return new Raycaster(w, h);
    }

    static void addProperties(jac::ContextRef ctx, jac::Object proto) {
        jac::FunctionFactory ff(ctx);

        proto.defineProperty(
            "setTileConfig",
            ff.newFunctionThisVariadic([](jac::ContextRef ctx,
                                          jac::ValueWeak thisVal,
                                          std::vector<jac::ValueWeak> args) {
                if (args.size() < 3)
                    return jac::Value::undefined(ctx);
                Raycaster *self = getOpaque(ctx, thisVal);

                auto parseArray = [](jac::ValueWeak val) {
                    std::vector<int> res;
                    auto arr = val.to<jac::ArrayWeak>();
                    uint32_t len = std::min(256, arr.length());
                    for (uint32_t i = 0; i < len; i++) {
                        res.push_back(arr.get(i).to<int>());
                    }
                    return res;
                };

                self->setTileConfig(parseArray(args[0]), parseArray(args[1]),
                                    parseArray(args[2]));
                return jac::Value::undefined(ctx);
            }));

        proto.defineProperty(
            "setFloorMap",
            ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal,
                                  jac::ArrayWeak mapVal) {
                Raycaster *self = getOpaque(ctx, thisVal);
                std::vector<std::vector<uint16_t>> map;

                uint32_t mapLen =
                    std::min(256,
                             mapVal.length()); // Clamp to prevent bad_alloc
                for (uint32_t i = 0; i < mapLen; i++) {
                    auto rowVal = mapVal.get(i).to<jac::ArrayWeak>();
                    uint32_t rowLen = std::min(256, rowVal.length());

                    std::vector<uint16_t> row;
                    row.reserve(rowLen);
                    for (uint32_t j = 0; j < rowLen; j++)
                        row.push_back((uint16_t)rowVal.get(j).to<int>());
                    map.push_back(row);
                }
                self->setFloorMap(map);
                return jac::Value::undefined(ctx);
            }));

        proto.defineProperty(
            "setCeilingMap",
            ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal,
                                  jac::ArrayWeak mapVal) {
                Raycaster *self = getOpaque(ctx, thisVal);
                std::vector<std::vector<uint16_t>> map;

                uint32_t mapLen = std::min(256, mapVal.length());
                for (uint32_t i = 0; i < mapLen; i++) {
                    auto rowVal = mapVal.get(i).to<jac::ArrayWeak>();
                    uint32_t rowLen = std::min(256, rowVal.length());

                    std::vector<uint16_t> row;
                    row.reserve(rowLen);
                    for (uint32_t j = 0; j < rowLen; j++)
                        row.push_back((uint16_t)rowVal.get(j).to<int>());
                    map.push_back(row);
                }
                self->setCeilingMap(map);
                return jac::Value::undefined(ctx);
            }));

        proto.defineProperty(
            "setMap",
            ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal,
                                  jac::ArrayWeak mapVal) {
                Raycaster *self = getOpaque(ctx, thisVal);
                std::vector<std::vector<int>> map;

                uint32_t mapLen = std::min(256, mapVal.length());
                for (uint32_t i = 0; i < mapLen; i++) {
                    auto rowVal = mapVal.get(i).to<jac::ArrayWeak>();
                    uint32_t rowLen = std::min(256, rowVal.length());

                    std::vector<int> row;
                    row.reserve(rowLen);
                    for (uint32_t j = 0; j < rowLen; j++)
                        row.push_back(rowVal.get(j).to<int>());
                    map.push_back(row);
                }
                self->setMap(map);
                return jac::Value::undefined(ctx);
            }));

        proto.defineProperty(
            "setTexture",
            ff.newFunctionThisVariadic([](jac::ContextRef ctx,
                                          jac::ValueWeak thisVal,
                                          std::vector<jac::ValueWeak> args) {
                if (args.size() < 5)
                    return jac::Value::undefined(ctx);
                Raycaster *self = getOpaque(ctx, thisVal);
                int id = args[0].to<int>();

                size_t dataSize = 0;
                uint8_t *data = (uint8_t *)JS_GetArrayBuffer(ctx, &dataSize,
                                                             args[1].getVal());

                int w = args[2].to<int>();
                int h = args[3].to<int>();
                int type = args[4].to<int>();
                self->setTexture(id, data, dataSize, w, h, (TextureType)type);
                return jac::Value::undefined(ctx);
            }));

        proto.defineProperty(
            "render",
            ff.newFunctionThisVariadic([](jac::ContextRef ctx,
                                          jac::ValueWeak thisVal,
                                          std::vector<jac::ValueWeak> args) {
                Raycaster *self = getOpaque(ctx, thisVal);
                size_t maxBytes;
                uint8_t *raw =
                    JS_GetArrayBuffer(ctx, &maxBytes, args[0].getVal());
                float px = args[1].to<float>(), py = args[2].to<float>();
                float dx = args[3].to<float>(), dy = args[4].to<float>();
                float plx = args[5].to<float>(), ply = args[6].to<float>();

                std::vector<float> spriteData;
                auto jsArray = args[7].to<jac::ArrayWeak>();
                uint32_t spriteLen = std::min(1024, jsArray.length());
                for (uint32_t i = 0; i < spriteLen; i++)
                    spriteData.push_back(jsArray.get(i).to<float>());

                std::vector<float> doorData;
                auto jsDoorArray = args[8].to<jac::ArrayWeak>();
                uint32_t doorLen = std::min(1024, jsDoorArray.length());
                for (uint32_t i = 0; i < doorLen; i++)
                    doorData.push_back(jsDoorArray.get(i).to<float>());

                int wFrame = args[9].to<int>(), fmt = args[10].to<int>();
                size_t written =
                    self->render(raw, maxBytes, px, py, dx, dy, plx, ply,
                                 spriteData, doorData, wFrame, fmt);
                return jac::Value::from(ctx, (int)written);
            }));
    }
};

template <class Next> class RaycasterFeature : public Next {
  public:
    using RaycasterClass = jac::Class<RaycasterProtoBuilder>;

    RaycasterFeature() { RaycasterClass::init("Raycaster"); }

    void initialize() {
        Next::initialize();
        jac::Module &rayModule = this->newModule("raycaster");
        rayModule.addExport("Raycaster",
                            RaycasterClass::getConstructor(this->context()));
    }
};
