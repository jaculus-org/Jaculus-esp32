#pragma once
#include "../util/iDisplayHolder.h"
#include "jac/device/logger.h"
#include "jac/machine/internal/declarations.h"
#include "renderer/rendererFeature.h"
#include <cstddef>
#include <cstdint>
#include <format>
#include <jac/machine/class.h>
#include <jac/machine/functionFactory.h>
#include <vector>

template <typename THolder> class DisplayProtoBindings {
  public:
    static void addCommonProperties(jac::ContextRef ctx, jac::Object proto) {
        jac::FunctionFactory ff(ctx);

        proto.defineProperty(
            "clear",
            ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal) {
                auto *holder = static_cast<IDisplayHolder *>(
                    jac::ProtoBuilder::Opaque<THolder>::getOpaque(ctx,
                                                                  thisVal));
                if (holder)
                    holder->clear();
            }),
            jac::PropFlags::Enumerable);

        proto.defineProperty(
            "setBrightness",
            ff.newFunctionThis([](jac::ContextRef ctx, jac::ValueWeak thisVal,
                                  int brightness) {
                auto *holder = static_cast<IDisplayHolder *>(
                    jac::ProtoBuilder::Opaque<THolder>::getOpaque(ctx,
                                                                  thisVal));
                if (holder)
                    holder->setBrightness(brightness);
            }),
            jac::PropFlags::Enumerable);

        proto.defineProperty(
            "isInitialized",
            ff.newFunctionThis(
                [](jac::ContextRef ctx, jac::ValueWeak thisVal) -> jac::Value {
                    auto *holder = static_cast<IDisplayHolder *>(
                        jac::ProtoBuilder::Opaque<THolder>::getOpaque(ctx,
                                                                      thisVal));
                    if (!holder)
                        return jac::Value::from(ctx, false);
                    return jac::Value::from(ctx, holder->isInitialized());
                }),
            jac::PropFlags::Enumerable);

        proto.defineProperty(
            "setBuffer",
            ff.newFunctionThisVariadic([](jac::ContextRef ctx,
                                          jac::ValueWeak thisVal,
                                          std::vector<jac::ValueWeak> args) {
                auto *holder = static_cast<IDisplayHolder *>(
                    jac::ProtoBuilder::Opaque<THolder>::getOpaque(ctx,
                                                                  thisVal));
                if (!holder || args.empty())
                    return;

                size_t maxBytes;
                uint8_t *raw =
                    JS_GetArrayBuffer(ctx, &maxBytes, args[0].getVal());
                if (!raw) {
                    jac::Logger::error("Display: Invalid ArrayBuffer passed");
                    return;
                }

                int size = (args.size() > 1 && !args[1].isUndefined())
                               ? args[1].to<int>()
                               : maxBytes;
                int format = (args.size() > 2 && !args[2].isUndefined())
                                 ? args[2].to<int>()
                                 : 10;
                bool clearPrev = (args.size() > 3) ? args[3].to<bool>() : true;

                size_t readSize =
                    (size >= 0 && size <= maxBytes) ? size : maxBytes;

                holder->setBuffer(raw, readSize, format, clearPrev);
            }),
            jac::PropFlags::Enumerable);

        proto.defineProperty(
            "setBufferRaw",
            ff.newFunctionThisVariadic([](jac::ContextRef ctx,
                                          jac::ValueWeak thisVal,
                                          std::vector<jac::ValueWeak> args) {
                auto *holder = static_cast<IDisplayHolder *>(
                    jac::ProtoBuilder::Opaque<THolder>::getOpaque(ctx,
                                                                  thisVal));
                if (!holder || args.empty())
                    return;

                auto array = args[0].to<jac::ArrayWeak>();
                if (array.isUndefined())
                    return;

                int format = (args.size() > 1) ? args[1].to<int>() : 10;
                bool clearPrev = (args.size() > 2) ? args[2].to<bool>() : true;

                uint32_t len = array.length();
                std::vector<DisplayPixel> parsedPixels;
                parsedPixels.reserve(len);

                for (uint32_t i = 0; i < len; ++i) {
                    auto inner = array.get(i).to<jac::ArrayWeak>();
                    if (inner.isUndefined() || inner.length() < 3)
                        continue;
                    uint16_t x = inner.get(0).to<int>();
                    uint16_t y = inner.get(1).to<int>();
                    uint8_t rawColor[4] = {0};
                    size_t colorArgsCount = inner.length() - 2;

                    for (size_t c = 0; c < colorArgsCount && c < 4; ++c) {
                        rawColor[c] = inner.get(2 + c).to<uint8_t>();
                    }

                    DisplayColor color{};
                    DisplayUtils::unpackColor(rawColor, format, color);

                    parsedPixels.push_back({x, y, color});
                }

                holder->setBufferFromRaw(parsedPixels, clearPrev);
            }),
            jac::PropFlags::Enumerable);
    }
};
