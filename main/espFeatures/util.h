#pragma once

#include <jac/machine/machine.h>
#include <jac/machine/values.h>
#include <vector>
#include <string>
#include <span>



inline jac::Value toUint8Array(JSContext* ctx, const std::vector<uint8_t>& data) {
    auto res = jac::ArrayBuffer::create(ctx, std::span(data));
    auto& machine = *static_cast<jac::MachineBase*>(JS_GetContextOpaque(ctx));
    jac::Value convertor = machine.eval("(buf) => new Uint8Array(buf)", "<util::toUint8Array>");
    return convertor.to<jac::Function>().call<jac::Value>(res);
}

inline std::vector<uint8_t> toStdVector(JSContext* ctx, jac::Value data) {
    std::vector<uint8_t> dataVec;
    if (JS_IsString(data.getVal())) {
        auto str = data.toString();
        dataVec.assign(str.begin(), str.end());
    } else {
        auto& machine = *static_cast<jac::MachineBase*>(JS_GetContextOpaque(ctx));
        jac::Value toArrayBuffer = machine.eval(
R"--(
(data) => {
    if (data instanceof ArrayBuffer) return data;
    if (ArrayBuffer.isView(data)) return data.buffer;
    if (typeof data === 'number') return new Int8Array([data]).buffer;
    if (Array.isArray(data)) return new Int8Array(data).buffer;
    throw new Error('Invalid data type');
}
)--", "<util::toStdVector>");
        auto res = toArrayBuffer.to<jac::Function>().call<jac::ArrayBuffer>(data);
        auto view = res.typedView<uint8_t>();
        dataVec.assign(view.begin(), view.end());
    }
    return dataVec;
}
