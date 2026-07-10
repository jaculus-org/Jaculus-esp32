#pragma once

#include "Circle.hpp"
#include "Collection.hpp"
#include "Font.hpp"
#include "LineSegment.hpp"
#include "Point.hpp"
#include "Polygon.hpp"
#include "Rectangle.hpp"
#include "RegularPolygon.hpp"
#include "Renderer.hpp"
#include "Shape.hpp"
#include "Utils.hpp"
#include "esp_err.h"
#include "ff.h"
#include "jac/device/logger.h"
#include "jac/machine/context.h"
#include "jac/machine/internal/declarations.h"
#include "quickjs.h"

#include <cstdint>
#include <jac/machine/class.h>
#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>
#include <jac/machine/values.h>
#include <memory>

// Reference:
// https://419.ecma-international.org/3.0/index.html#-15-display-class-pattern-pixel-format-values
size_t packedColorSize(int format) {
    switch (format) {
    case 3:
    case 4:
    case 5:
    case 6:
        return 1;
    case 7:
    case 8:
    case 12:
        return 2;
    case 9:
        return 3;
    case 10:
        return 4;
    default:
        return 0;
    }
}

template <bool Antialias, int BytesPerPixel, typename Packer>
void fillBufferBlock(uint8_t* raw, int width, int height, const Display& displayGrid, int start_sx, int start_sy, int dx_sx, int dx_sy, int dy_sx, int dy_sy, Packer pack) {
    uint8_t* out = raw;
    int row_sx = start_sx;
    int row_sy = start_sy;

    for (int y = 0; y < height; ++y) {
        int sx = row_sx;
        int sy = row_sy;

        for (int x = 0; x < width; ++x) {
            if (static_cast<unsigned>(sx) < static_cast<unsigned>(displayGrid.width) && static_cast<unsigned>(sy) < static_cast<unsigned>(displayGrid.height)) {
                Color p = displayGrid.pixels[sy * displayGrid.width + sx];

                if constexpr (Antialias) {
                    p.r = (p.r * p.a) >> 8;
                    p.g = (p.g * p.a) >> 8;
                    p.b = (p.b * p.a) >> 8;
                }

                pack(out, p);
            } else {
                for (int i = 0; i < BytesPerPixel; ++i) {
                    out[i] = 0;
                }
            }

            out += BytesPerPixel;
            sx += dx_sx;
            sy += dx_sy;
        }
        row_sx += dy_sx;
        row_sy += dy_sy;
    }
}

size_t writeDenseFramebuffer(uint8_t* raw, size_t maxBytes, int width, int height, int format, bool antialias, const Display& displayGrid, int rotation = 0) {
    size_t bytesPerPixel = packedColorSize(format);
    if (bytesPerPixel == 0)
        return 0;

    size_t frameBytes = static_cast<size_t>(width) * static_cast<size_t>(height) * bytesPerPixel;
    if (frameBytes > maxBytes)
        return 0;

    int r = (rotation % 4 + 4) % 4;

    int start_sx = 0, start_sy = 0;
    int dx_sx = 1, dx_sy = 0, dy_sx = 0, dy_sy = 1;

    if (r == 1) { // 90 degrees
        start_sx = 0;
        start_sy = width - 1;
        dx_sx = 0;
        dx_sy = -1;
        dy_sx = 1;
        dy_sy = 0;
    } else if (r == 2) { // 180 degrees
        start_sx = width - 1;
        start_sy = height - 1;
        dx_sx = -1;
        dx_sy = 0;
        dy_sx = 0;
        dy_sy = -1;
    } else if (r == 3) { // 270 degrees
        start_sx = height - 1;
        start_sy = 0;
        dx_sx = 0;
        dx_sy = 1;
        dy_sx = -1;
        dy_sy = 0;
    }

#define HANDLE_FORMAT(FMT, BYTES, PACK_LAMBDA) \
    case FMT: \
        if (antialias) \
            fillBufferBlock<true, BYTES>(raw, width, height, displayGrid, start_sx, start_sy, dx_sx, dx_sy, dy_sx, dy_sy, PACK_LAMBDA); \
        else \
            fillBufferBlock<false, BYTES>(raw, width, height, displayGrid, start_sx, start_sy, dx_sx, dx_sy, dy_sx, dy_sy, PACK_LAMBDA); \
        break;

    switch (format) {
        HANDLE_FORMAT(3, 1, [](uint8_t* out, Color p) {
            out[0] = (p.r + p.g + p.b) > 381 ? 1 : 0;
        })
        HANDLE_FORMAT(4, 1, [](uint8_t* out, Color p) {
            out[0] = ((p.r * 77 + p.g * 150 + p.b * 29) >> 12) & 0x0F;
        })
        HANDLE_FORMAT(5, 1, [](uint8_t* out, Color p) {
            out[0] = (p.r * 77 + p.g * 150 + p.b * 29) >> 8;
        })
        HANDLE_FORMAT(6, 1, [](uint8_t* out, Color p) {
            out[0] = (p.r & 0xE0) | ((p.g >> 3) & 0x1C) | (p.b >> 6);
        })
        HANDLE_FORMAT(7, 2, [](uint8_t* out, Color p) {
            uint16_t rgb = ((p.r & 0xF8) << 8) | ((p.g & 0xFC) << 3) | (p.b >> 3);
            out[0] = rgb & 0xFF;
            out[1] = rgb >> 8;
        })
        HANDLE_FORMAT(8, 2, [](uint8_t* out, Color p) {
            uint16_t rgb = ((p.r & 0xF8) << 8) | ((p.g & 0xFC) << 3) | (p.b >> 3);
            out[0] = rgb >> 8;
            out[1] = rgb & 0xFF;
        })
        HANDLE_FORMAT(9, 3, [](uint8_t* out, Color p) {
            out[0] = p.r;
            out[1] = p.g;
            out[2] = p.b;
        })
        HANDLE_FORMAT(10, 4, [](uint8_t* out, Color p) {
            out[0] = p.r;
            out[1] = p.g;
            out[2] = p.b;
            out[3] = p.a;
        })
        HANDLE_FORMAT(12, 2, [](uint8_t* out, Color p) {
            out[0] = (p.g & 0xF0) | (p.b >> 4);
            out[1] = (p.a & 0xF0) | (p.r >> 4);
        })
    }
#undef HANDLE_FORMAT

    return frameBytes;
}

void writePixelBytes(uint8_t* out, int format, Color p) {
    switch (format) {
    case 3:
        out[0] = (p.r + p.g + p.b) > 381 ? 1 : 0;
        break;
    case 4:
        out[0] = ((p.r * 77 + p.g * 150 + p.b * 29) >> 12) & 0x0F;
        break;
    case 5:
        out[0] = (p.r * 77 + p.g * 150 + p.b * 29) >> 8;
        break;
    case 6:
        out[0] = (p.r & 0xE0) | ((p.g >> 3) & 0x1C) | (p.b >> 6);
        break;
    case 7: {
        uint16_t rgb = ((p.r & 0xF8) << 8) | ((p.g & 0xFC) << 3) | (p.b >> 3);
        out[0] = rgb & 0xFF;
        out[1] = rgb >> 8;
        break;
    }
    case 8: {
        uint16_t rgb = ((p.r & 0xF8) << 8) | ((p.g & 0xFC) << 3) | (p.b >> 3);
        out[0] = rgb >> 8;
        out[1] = rgb & 0xFF;
        break;
    }
    case 9:
        out[0] = p.r;
        out[1] = p.g;
        out[2] = p.b;
        break;
    case 10:
        out[0] = p.r;
        out[1] = p.g;
        out[2] = p.b;
        out[3] = p.a;
        break;
    case 12:
        out[0] = (p.g & 0xF0) | (p.b >> 4);
        out[1] = (p.a & 0xF0) | (p.r >> 4);
        break;
    }
}

void writeTextPixel(uint8_t* raw, size_t maxBytes, int width, int height, int format, int rotation, int lx, int ly, Color color) {
    size_t bytesPerPixel = packedColorSize(format);
    if (bytesPerPixel == 0)
        return;

    int r = ((rotation % 4) + 4) % 4;
    int px = lx, py = ly;
    if (r == 1) { // 90 degrees
        px = width - 1 - ly;
        py = lx;
    } else if (r == 2) { // 180 degrees
        px = width - 1 - lx;
        py = height - 1 - ly;
    } else if (r == 3) { // 270 degrees
        px = ly;
        py = height - 1 - lx;
    }

    if (static_cast<unsigned>(px) >= static_cast<unsigned>(width) || static_cast<unsigned>(py) >= static_cast<unsigned>(height))
        return;

    size_t offset = (static_cast<size_t>(py) * width + px) * bytesPerPixel;
    if (offset + bytesPerPixel > maxBytes)
        return;

    writePixelBytes(raw + offset, format, color);
}

// ===================================
//      Type Conversions (ConvTraits)
// ===================================

template <>
struct jac::ConvTraits<Color> {
    static jac::Value to(ContextRef ctx, Color val) {
        uint32_t packed = (val.r << 16) | (val.g << 8) | val.b;
        return jac::Value::from(ctx, packed);
    }

    static Color from(ContextRef ctx, ValueWeak val) {
        if (!val.isNumber()) {
            throw std::runtime_error("Expected a number");
        }
        uint32_t value = val.to<uint32_t>();
        return Color((value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF);
    }
};

template <>
struct jac::ConvTraits<ShapeParams> {
    static ShapeParams from(ContextRef ctx, ValueWeak val) {
        auto obj = val.to<jac::ObjectWeak>();
        return ShapeParams(obj.get<float>("x"), obj.get<float>("y"), obj.hasProperty("z") ? obj.get<float>("z") : 0);
    }
};

template <>
struct jac::ConvTraits<PointParams> {
    static PointParams from(ContextRef ctx, ValueWeak val) {
        auto obj = val.to<jac::ObjectWeak>();
        return PointParams(obj.get<float>("x"), obj.get<float>("y"), obj.get<Color>("color"), obj.hasProperty("z") ? obj.get<float>("z") : 0);
    }
};

template <>
struct jac::ConvTraits<CircleParams> {
    static CircleParams from(ContextRef ctx, ValueWeak val) {
        auto obj = val.to<jac::ObjectWeak>();
        return CircleParams(obj.get<float>("x"), obj.get<float>("y"), obj.get<Color>("color"), obj.get<int>("radius"), obj.hasProperty("fill") ? obj.get<bool>("fill") : false, obj.hasProperty("z") ? obj.get<float>("z") : 0);
    }
};

template <>
struct jac::ConvTraits<RectangleParams> {
    static RectangleParams from(ContextRef ctx, ValueWeak val) {
        auto obj = val.to<jac::ObjectWeak>();
        return RectangleParams(obj.get<float>("x"), obj.get<float>("y"), obj.get<Color>("color"), obj.get<int>("width"), obj.get<int>("height"), obj.hasProperty("fill") ? obj.get<bool>("fill") : false, obj.hasProperty("z") ? obj.get<float>("z") : 0);
    }
};

template <>
struct jac::ConvTraits<PolygonParams> {
    static PolygonParams from(ContextRef ctx, ValueWeak val) {
        auto obj = val.to<jac::ObjectWeak>();
        auto vertices_js = obj.get<jac::ArrayWeak>("vertices");
        std::vector<std::pair<int, int>> vertices;
        uint32_t len = vertices_js.length();
        vertices.reserve(len);
        for (uint32_t i = 0; i < len; ++i) {
            auto vertex_js = vertices_js.get(i).to<jac::ArrayWeak>();
            vertices.push_back({vertex_js.get(0).to<int>(), vertex_js.get(1).to<int>()});
        }
        return PolygonParams(obj.get<float>("x"), obj.get<float>("y"), obj.get<Color>("color"), vertices, obj.hasProperty("fill") ? obj.get<bool>("fill") : false, obj.hasProperty("z") ? obj.get<float>("z") : 0);
    }
};

template <>
struct jac::ConvTraits<LineSegmentParams> {
    static LineSegmentParams from(ContextRef ctx, ValueWeak val) {
        auto obj = val.to<jac::ObjectWeak>();
        return LineSegmentParams(obj.get<float>("x"), obj.get<float>("y"), obj.get<Color>("color"), obj.get<float>("x2"), obj.get<float>("y2"), obj.hasProperty("z") ? obj.get<float>("z") : 0);
    }
};

template <>
struct jac::ConvTraits<RegularPolygonRadiusParams> {
    static RegularPolygonRadiusParams from(ContextRef ctx, ValueWeak val) {
        auto obj = val.to<jac::ObjectWeak>();
        return RegularPolygonRadiusParams(obj.get<float>("x"), obj.get<float>("y"), obj.get<Color>("color"), obj.get<int>("sides"), obj.get<int>("radius"), obj.hasProperty("fill") ? obj.get<bool>("fill") : false, obj.hasProperty("z") ? obj.get<float>("z") : 0);
    }
};

template <>
struct jac::ConvTraits<RegularPolygonSideParams> {
    static RegularPolygonSideParams from(ContextRef ctx, ValueWeak val) {
        auto obj = val.to<jac::ObjectWeak>();
        return RegularPolygonSideParams(obj.get<float>("x"), obj.get<float>("y"), obj.get<Color>("color"), obj.get<int>("sides"), obj.get<int>("sideLength"), obj.hasProperty("fill") ? obj.get<bool>("fill") : false, obj.hasProperty("z") ? obj.get<float>("z") : 0);
    }
};

// ===================================
//      ProtoBuilders
// ===================================
class TextureProtoBuilder : public jac::ProtoBuilder::Opaque<Texture>, public jac::ProtoBuilder::Properties {
public:
    static Texture* unwrap(jac::ContextRef ctx, jac::ValueWeak val) {
        return getOpaque(ctx, val);
    }

    static Texture* constructOpaque(jac::ContextRef ctx, std::vector<jac::ValueWeak> args) {
        return new Texture();
    }

    static void addProperties(jac::ContextRef ctx, jac::Object proto) {
        jac::FunctionFactory ff(ctx);

        proto.defineProperty("load", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, std::string path) {
            Texture* self = getOpaque(ctx, thisVal);
            bool success = Texture::fromBMP(path, *self);
            return jac::Value::from(ctx, success);
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("setWrapMode", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, std::string mode) {
            Texture* self = getOpaque(ctx, thisVal);
            self->setWrapMode(mode);
            return jac::Value::undefined(ctx);
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("getWidth", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            Texture* self = getOpaque(ctx, thisVal);
            return jac::Value::from(ctx, self->width);
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("getHeight", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            Texture* self = getOpaque(ctx, thisVal);
            return jac::Value::from(ctx, self->height);
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("isValid", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            Texture* self = getOpaque(ctx, thisVal);
            return jac::Value::from(ctx, self->valid);
        }), jac::PropFlags::Enumerable);
    }
};

class ShapeProtoBuilder : public jac::ProtoBuilder::Opaque<std::shared_ptr<Shape>>, public jac::ProtoBuilder::Properties {
private:
    static inline std::vector<JSClassID> derivedClassIDs;
    using ShapeGetter = std::function<jac::Value(jac::ContextRef ctx, Shape* shape)>;

    struct PropDef {
        const char* name;
        ShapeGetter getter;
    };

    static void registerGetters(jac::ContextRef ctx, jac::Object proto, jac::FunctionFactory& ff, const std::vector<PropDef>& props) {
        for (const auto& prop : props) {
            proto.defineProperty(prop.name, ff.newFunctionThis([getter = prop.getter](jac::ContextRef ctx, jac::ValueWeak thisVal) {
                Shape* shape = unwrapShape(ctx, thisVal);
                return getter(ctx, shape);
            }), jac::PropFlags::Enumerable);
        }
    }

    template <typename... Args>
    struct SetterDef {
        const char* name;
        std::function<void(Shape*, Args...)> func;
    };

    template <typename... Args>
    static void registerSetters(jac::ContextRef ctx, jac::Object proto, jac::FunctionFactory& ff, const std::vector<SetterDef<Args...>>& defs) {
        for (const auto& def : defs) {
            proto.defineProperty(def.name, ff.newFunctionThis([func = def.func](jac::ContextRef ctx, jac::ValueWeak thisVal, Args... args) {
                Shape* shape = unwrapShape(ctx, thisVal);
                func(shape, args...);
            }), jac::PropFlags::Enumerable);
        }
    }

    static void addBasicSetters(jac::ContextRef ctx, jac::Object proto, jac::FunctionFactory& ff) {
        registerSetters<float>(ctx, proto, ff, {
            {"rotate", [](Shape* s, float v) { s->rotate(v); }},
            {"setRotationAngle", [](Shape* s, float v) { s->setRotationAngle(v); }},
            {"setScaleX", [](Shape* s, float v) { s->scaleX(v); }},
            {"setScaleY", [](Shape* s, float v) { s->scaleY(v); }},
        });

        registerSetters<int>(ctx, proto, ff, {
            {"setX", [](Shape* s, int v) { s->setX(v); }},
            {"setY", [](Shape* s, int v) { s->setY(v); }},
            {"setZ", [](Shape* s, int v) { s->z = v; }},
        });

        registerSetters<int, int>(ctx, proto, ff, {
            {"setPosition", [](Shape* s, int x, int y) { s->setPosition(x, y); }},
            {"setPivot", [](Shape* s, int x, int y) { s->setPivot(x, y); }},
        });

        registerSetters<float, float>(ctx, proto, ff, {
            {"translate", [](Shape* s, float x, float y) { s->translate(x, y); }},
        });

        proto.defineProperty("setScale", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, float scaleX, float scaleY, jac::ValueWeak originX, jac::ValueWeak originY) {
            Shape* shape = unwrapShape(ctx, thisVal);
            float ox = originX.isUndefined() ? -1 : originX.to<float>();
            float oy = originY.isUndefined() ? -1 : originY.to<float>();
            shape->setScale(scaleX, scaleY, ox, oy);
        }), jac::PropFlags::Enumerable);
    }

    static void addTextureSetters(jac::ContextRef ctx, jac::Object proto, jac::FunctionFactory& ff) {
        registerSetters<float>(ctx, proto, ff, {
            {"setTextureRotation", [](Shape* s, float v) { s->setTextureRotation(v); }},
            {"setUVScaleX", [](Shape* s, float v) { s->setUVScaleX(v); }},
            {"setUVScaleY", [](Shape* s, float v) { s->setUVScaleY(v); }},
            {"setUVOffsetX", [](Shape* s, float v) { s->uvOffsetX = v; }},
            {"setUVOffsetY", [](Shape* s, float v) { s->uvOffsetY = v; }},
            {"setUVRotation", [](Shape* s, float v) { s->setUVRotation(v); }},
        });

        registerSetters<float, float>(ctx, proto, ff, {
            {"setTextureOffset", [](Shape* s, float x, float y) {
                s->uvOffsetX = x;
                s->uvOffsetY = y;
            }},
            {"setTextureScale", [](Shape* s, float x, float y) { s->setTextureScale(x, y); }},
        });

        registerSetters<bool>(ctx, proto, ff, {
            {"setFixTexture", [](Shape* s, bool v) { s->fixTexture = v; }},
        });

        proto.defineProperty("setTexture", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, jac::ValueWeak texVal) {
            auto* shape = unwrapShape(ctx, thisVal);
            Texture* tex = TextureProtoBuilder::unwrap(ctx, texVal);
            if (tex) {
                shape->texture = tex;
            }
            return jac::Value::undefined(ctx);
        }), jac::PropFlags::Enumerable);
    }

    static void addGetters(jac::ContextRef ctx, jac::Object proto, jac::FunctionFactory& ff) {
        registerGetters(ctx, proto, ff, {
            {"getX", [](jac::ContextRef ctx, Shape* s) { return jac::Value::from(ctx, s->x()); }},
            {"getY", [](jac::ContextRef ctx, Shape* s) { return jac::Value::from(ctx, s->y()); }},
            {"getZ", [](jac::ContextRef ctx, Shape* s) { return jac::Value::from(ctx, s->z); }},
            {"getRotationAngle", [](jac::ContextRef ctx, Shape* s) { return jac::Value::from(ctx, s->rotationAngle()); }},
            {"getScaleX", [](jac::ContextRef ctx, Shape* s) { return jac::Value::from(ctx, s->scaleX()); }},
            {"getScaleY", [](jac::ContextRef ctx, Shape* s) { return jac::Value::from(ctx, s->scaleY()); }},
        });
    }

    static void addColliderSetters(jac::ContextRef ctx, jac::Object proto, jac::FunctionFactory& ff) {
        proto.defineProperty("intersects", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, jac::ValueWeak otherShapeVal) {
            void* ptr1 = JS_GetOpaque(thisVal.getVal(), JS_GetClassID(thisVal.getVal()));
            void* ptr2 = JS_GetOpaque(otherShapeVal.getVal(), JS_GetClassID(otherShapeVal.getVal()));

            if (ptr1 && ptr2) {
                auto& s1 = *static_cast<std::shared_ptr<Shape>*>(ptr1);
                auto& s2 = *static_cast<std::shared_ptr<Shape>*>(ptr2);

                if (s1 && s2) {
                    return jac::toValue(ctx, s1->intersects(s2));
                }
            }
            return jac::Value::from(ctx, false);
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("addCollider", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            Shape* shape = unwrapShape(ctx, thisVal);
            shape->addCollider(nullptr);
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("removeCollider", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            Shape* shape = unwrapShape(ctx, thisVal);
            shape->removeCollider();
        }), jac::PropFlags::Enumerable);
    }

public:
    static void registerDerivedClass(JSClassID classId) {
        derivedClassIDs.push_back(classId);
    }

    static Shape* unwrapShape(jac::ContextRef ctx, jac::ValueWeak thisVal) {
        void* ptr = JS_GetOpaque(thisVal.getVal(), classId);

        if (ptr) {
            return (*static_cast<std::shared_ptr<Shape>*>(ptr)).get();
        }

        for (auto derivedClassId : derivedClassIDs) {
            ptr = JS_GetOpaque(thisVal.getVal(), derivedClassId);
            if (ptr) {
                return (*static_cast<std::shared_ptr<Shape>*>(ptr)).get();
            }
        }

        throw jac::Exception::create(jac::Exception::Type::TypeError, "Invalid Shape object");
    }

    static void addProperties(jac::ContextRef ctx, jac::Object proto) {
        jac::FunctionFactory ff(ctx);

        addBasicSetters(ctx, proto, ff);
        addTextureSetters(ctx, proto, ff);
        addGetters(ctx, proto, ff);
        addColliderSetters(ctx, proto, ff);
    }
};

#define SHAPE_BUILDER_BOILERPLATE(ClassName, ParamsType) \
    class ClassName##ProtoBuilder : public jac::ProtoBuilder::Opaque<std::shared_ptr<Shape>>, public jac::ProtoBuilder::Properties { \
    public: \
        static std::shared_ptr<Shape>* constructOpaque(jac::ContextRef ctx, std::vector<jac::ValueWeak> args) { \
            return new std::shared_ptr<Shape>(new ClassName(jac::fromValue<ParamsType>(ctx, args[0]))); \
        } \
        static void addProperties(jac::ContextRef ctx, jac::Object proto) { \
            ShapeProtoBuilder::addProperties(ctx, proto); \
            jac::FunctionFactory ff(ctx); \
            proto.defineProperty("setColor", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, jac::ValueWeak colorVal) { \
                Color color = jac::fromValue<Color>(ctx, colorVal); \
                static_cast<ClassName*>(ShapeProtoBuilder::unwrapShape(ctx, thisVal))->color = color; \
            }), jac::PropFlags::Enumerable); \
            proto.defineProperty("getColor", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) { \
                return jac::toValue(ctx, static_cast<ClassName*>(ShapeProtoBuilder::unwrapShape(ctx, thisVal))->color); \
            }), jac::PropFlags::Enumerable); \
        } \
    };

SHAPE_BUILDER_BOILERPLATE(Circle, CircleParams)
SHAPE_BUILDER_BOILERPLATE(Rectangle, RectangleParams)
SHAPE_BUILDER_BOILERPLATE(Polygon, PolygonParams)
SHAPE_BUILDER_BOILERPLATE(LineSegment, LineSegmentParams)
SHAPE_BUILDER_BOILERPLATE(Point, PointParams)

class CollectionProtoBuilder : public jac::ProtoBuilder::Opaque<std::shared_ptr<Collection>>, public jac::ProtoBuilder::Properties {
public:
    static std::shared_ptr<Collection>* constructOpaque(jac::ContextRef ctx, std::vector<jac::ValueWeak> args) {
        auto rawPtr = new Collection(jac::fromValue<ShapeParams>(ctx, args[0]));
        return new std::shared_ptr<Collection>(rawPtr);
    }

    static void addProperties(jac::ContextRef ctx, jac::Object proto) {
        ShapeProtoBuilder::addProperties(ctx, proto);
        jac::FunctionFactory ff(ctx);

        proto.defineProperty("add", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, jac::Object shapeVal) {
            auto collectionPtr = getOpaque(ctx, thisVal);
            void* shapeOpaque = JS_GetOpaque(shapeVal.getVal(), JS_GetClassID(shapeVal.getVal()));

            if (shapeOpaque) {
                auto shapePtr = reinterpret_cast<std::shared_ptr<Shape>*>(shapeOpaque);
                if (shapePtr && *shapePtr) {
                    (*collectionPtr)->addShape(*shapePtr);
                }
            }
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("clear", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            auto collectionPtr = getOpaque(ctx, thisVal);
            (*collectionPtr)->clear();
            return jac::Value::undefined(ctx);
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("remove", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, jac::Object shapeVal) {
            auto collectionPtr = getOpaque(ctx, thisVal);
            void* shapeOpaque = JS_GetOpaque(shapeVal.getVal(), JS_GetClassID(shapeVal.getVal()));

            if (shapeOpaque) {
                auto shapePtr = reinterpret_cast<std::shared_ptr<Shape>*>(shapeOpaque);
                if (shapePtr && *shapePtr) {
                    (*collectionPtr)->removeShape(*shapePtr);
                }
            }
        }), jac::PropFlags::Enumerable);
    }
};

class RegularPolygonProtoBuilder : public jac::ProtoBuilder::Opaque<std::shared_ptr<Shape>>, public jac::ProtoBuilder::Properties {
public:
    static std::shared_ptr<Shape>* constructOpaque(jac::ContextRef ctx, std::vector<jac::ValueWeak> args) {
        auto obj = args[0].to<jac::ObjectWeak>();
        Shape* rawShape;
        if (obj.hasProperty("radius")) {
            rawShape = new RegularPolygon(jac::fromValue<RegularPolygonRadiusParams>(ctx, args[0]));
        } else {
            rawShape = new RegularPolygon(jac::fromValue<RegularPolygonSideParams>(ctx, args[0]));
        }
        return new std::shared_ptr<Shape>(rawShape);
    }

    static void addProperties(jac::ContextRef ctx, jac::Object proto) {
        ShapeProtoBuilder::addProperties(ctx, proto);
        jac::FunctionFactory ff(ctx);

        proto.defineProperty("setColor", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, jac::ValueWeak colorVal) {
            Color color = jac::fromValue<Color>(ctx, colorVal);
            static_cast<RegularPolygon*>(ShapeProtoBuilder::unwrapShape(ctx, thisVal))->color = color;
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("getColor", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            return jac::toValue(ctx, static_cast<RegularPolygon*>(ShapeProtoBuilder::unwrapShape(ctx, thisVal))->color);
        }), jac::PropFlags::Enumerable);
    }
};

class FontProtoBuilder : public jac::ProtoBuilder::Opaque<Font>, public jac::ProtoBuilder::Properties {
public:
    static Font* unwrap(jac::ContextRef ctx, jac::ValueWeak val) {
        return getOpaque(ctx, val);
    }

    static Font* constructOpaque(jac::ContextRef ctx, std::vector<jac::ValueWeak> args) {
        return new Font();
    }

    static void addProperties(jac::ContextRef ctx, jac::Object proto) {
        jac::FunctionFactory ff(ctx);

        proto.defineProperty("getHeight", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
            return jac::Value::from(ctx, Font::HEIGHT);
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("getCharWidth", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, std::string charStr) {
            Font* font = unwrap(ctx, thisVal);
            if (charStr.empty())
                return jac::Value::null(ctx);
            return jac::Value::from(ctx, font->getCharWidth(charStr[0]));
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("getCharSpacing", ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal, std::string charStr) {
            Font* font = unwrap(ctx, thisVal);
            if (charStr.empty())
                return jac::Value::null(ctx);
            return jac::Value::from(ctx, font->getCharSpacing(charStr[0]));
        }), jac::PropFlags::Enumerable);
    }
};

class RendererHolder {
private:
    std::unique_ptr<::Renderer> m_renderer;
    int m_width;
    int m_height;

public:
    RendererHolder(int width, int height) : m_renderer(std::make_unique<::Renderer>(width, height)), m_width(width), m_height(height) {}

    ::Renderer* getRenderer() { return m_renderer.get(); }
    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }
};

class RendererProtoBuilder : public jac::ProtoBuilder::Opaque<RendererHolder>, public jac::ProtoBuilder::Properties {
public:
    static RendererHolder* constructOpaque(jac::ContextRef ctx, std::vector<jac::ValueWeak> args) {
        int w = args[0].to<int>();
        int h = args[1].to<int>();
        if (w <= 0 || w > 512 || h <= 0 || h > 512) {
            w = 64;
            h = 64;
        }
        return new RendererHolder(w, h);
    }

    static void addProperties(jac::ContextRef ctx, jac::Object proto) {
        jac::FunctionFactory ff(ctx);

        proto.defineProperty("render", ff.newFunctionThisVariadic([](jac::ContextRef ctx, jac::ValueWeak thisVal, std::vector<jac::ValueWeak> args) -> jac::Value {
            if (args.size() < 2) {
                jac::Logger::error("Renderer.render: Missing arguments (collection, buffer)");
                return jac::Value::undefined(ctx);
            }

            auto* holder = getOpaque(ctx, thisVal);

            jac::ValueWeak collectionVal = args[0];
            auto collectionPtr = reinterpret_cast<std::shared_ptr<Collection>*>(JS_GetOpaque(collectionVal.getVal(), JS_GetClassID(collectionVal.getVal())));

            if (!collectionPtr || !*collectionPtr)
                return jac::Value::undefined(ctx);

            jac::ValueWeak bufferObj = args[1];

            size_t maxBytes;
            uint8_t* raw = JS_GetArrayBuffer(ctx, &maxBytes, bufferObj.getVal());
            if (!raw) {
                jac::Logger::error("Renderer: Invalid ArrayBuffer passed");
                return jac::Value::undefined(ctx);
            }

            bool antialias = (args.size() > 2) ? args[2].to<bool>() : true;

            int format = (args.size() > 3) ? args[3].to<int>() : 10;
            if (format < 3 || format == 11 || format > 12) {
                jac::Logger::error("Renderer: Invalid color format");
            }
            int rotation = (args.size() > 4) ? args[4].to<int>() : 0;

            int w = holder->getWidth();
            int h = holder->getHeight();
            holder->getRenderer()->clear();

            holder->getRenderer()->render({*collectionPtr}, {w, h, antialias});

            const Display& displayGrid = holder->getRenderer()->displayGrid;

            size_t frameBytes = writeDenseFramebuffer(raw, maxBytes, w, h, format, antialias, displayGrid, rotation);

            if (frameBytes == 0) {
                jac::Logger::error("Renderer.render: ArrayBuffer too small or invalid format");
                return jac::Value::undefined(ctx);
            }

            return jac::Value(ctx, static_cast<int>(frameBytes));
        }), jac::PropFlags::Enumerable);

        proto.defineProperty("drawText", ff.newFunctionThisVariadic([](jac::ContextRef ctx, jac::ValueWeak thisVal, std::vector<jac::ValueWeak> args) -> jac::Value {
            if (args.size() < 6) {
                jac::Logger::error("Renderer.drawText: Missing arguments (buffer, text, x, y, font, color, [wrap], [format])");
                return jac::Value::undefined(ctx);
            }

            auto* holder = getOpaque(ctx, thisVal);

            jac::ValueWeak bufferObj = args[0];
            size_t maxBytes;
            uint8_t* raw = JS_GetArrayBuffer(ctx, &maxBytes, bufferObj.getVal());
            if (!raw) {
                jac::Logger::error("Renderer.drawText: Invalid ArrayBuffer passed");
                return jac::Value::undefined(ctx);
            }

            std::string text = args[1].to<std::string>();
            int x = args[2].to<int>();
            int y = args[3].to<int>();

            Font* fontPtr = FontProtoBuilder::unwrap(ctx, args[4]);
            const Font& font = (fontPtr != nullptr) ? *fontPtr : defaultFont;
            Color color = jac::fromValue<Color>(ctx, args[5]);

            bool wrap = (args.size() >= 7) ? args[6].to<bool>() : false;

            int format = (args.size() >= 8) ? args[7].to<int>() : 10;
            int rotation = (args.size() >= 9) ? args[8].to<int>() : 0;

            int w = holder->getWidth();
            int h = holder->getHeight();

            size_t bytesPerPixel = packedColorSize(format);
            size_t frameBytes = static_cast<size_t>(w) * static_cast<size_t>(h) * bytesPerPixel;
            if (bytesPerPixel == 0 || frameBytes > maxBytes) {
                jac::Logger::error("Renderer.drawText: ArrayBuffer too small or invalid format");
                return jac::Value::undefined(ctx);
            }

            holder->getRenderer()->drawText(text, x, y, font, color, wrap, 0, [&](int px, int py, const Color& c) {
                writeTextPixel(raw, maxBytes, w, h, format, rotation, px, py, c);
            });

            return jac::Value(ctx, static_cast<int>(frameBytes));
        }), jac::PropFlags::Enumerable);
    }
};

// ===================================
//      Main Feature Class
// ===================================

template <class Next>
class RendererFeature : public Next {
public:
    using ShapeClass = jac::Class<ShapeProtoBuilder>;
    using RendererClass = jac::Class<RendererProtoBuilder>;
    using CollectionClass = jac::Class<CollectionProtoBuilder>;
    using CircleClass = jac::Class<CircleProtoBuilder>;
    using RectangleClass = jac::Class<RectangleProtoBuilder>;
    using PolygonClass = jac::Class<PolygonProtoBuilder>;
    using LineSegmentClass = jac::Class<LineSegmentProtoBuilder>;
    using PointClass = jac::Class<PointProtoBuilder>;
    using RegularPolygonClass = jac::Class<RegularPolygonProtoBuilder>;
    using FontClass = jac::Class<FontProtoBuilder>;
    using TextureClass = jac::Class<TextureProtoBuilder>;

    RendererFeature() {
        RendererClass::init("Renderer");

        ShapeClass::init("Shape");

        CollectionClass::init("Collection");
        CircleClass::init("Circle");
        RectangleClass::init("Rectangle");
        PolygonClass::init("Polygon");
        LineSegmentClass::init("LineSegment");
        PointClass::init("Point");
        RegularPolygonClass::init("RegularPolygon");

        FontClass::init("Font");
        TextureClass::init("Texture");

        using ShapePB = ShapeProtoBuilder;
        ShapePB::registerDerivedClass(CircleClass::getClassId());
        ShapePB::registerDerivedClass(RectangleClass::getClassId());
        ShapePB::registerDerivedClass(PolygonClass::getClassId());
        ShapePB::registerDerivedClass(LineSegmentClass::getClassId());
        ShapePB::registerDerivedClass(PointClass::getClassId());
        ShapePB::registerDerivedClass(CollectionClass::getClassId());
        ShapePB::registerDerivedClass(RegularPolygonClass::getClassId());
    }

    void initialize() {
        Next::initialize();
        jac::Module& rendererModule = this->newModule("renderer");
        rendererModule.addExport("Renderer", RendererClass::getConstructor(this->context()));

        rendererModule.addExport("Font", FontClass::getConstructor(this->context()));
        rendererModule.addExport("Texture", TextureClass::getConstructor(this->context()));

        // https://419.ecma-international.org/3.0/index.html#-15-display-class-pattern-pixel-format-values
        jac::Object formatObj = jac::Object::create(this->context());
        formatObj.set("MONOCHROME", 3);
        formatObj.set("GRAYSCALE_4_BIT", 4);
        formatObj.set("GRAYSCALE_8_BIT", 5);
        formatObj.set("RGB_332", 6);
        formatObj.set("RGB_565_LITTLE", 7);
        formatObj.set("RGB_565_BIG", 8);
        formatObj.set("RGB_888", 9);
        formatObj.set("RGBA_8888", 10);
        formatObj.set("XRGB", 12);
        rendererModule.addExport("Format", formatObj);

        jac::Module& shapesModule = this->newModule("shapes");
        shapesModule.addExport("Collection", CollectionClass::getConstructor(this->context()));
        shapesModule.addExport("Circle", CircleClass::getConstructor(this->context()));
        shapesModule.addExport("Rectangle", RectangleClass::getConstructor(this->context()));
        shapesModule.addExport("Polygon", PolygonClass::getConstructor(this->context()));
        shapesModule.addExport("LineSegment", LineSegmentClass::getConstructor(this->context()));
        shapesModule.addExport("Point", PointClass::getConstructor(this->context()));
        shapesModule.addExport("RegularPolygon", RegularPolygonClass::getConstructor(this->context()));
    }
};
