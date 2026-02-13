#pragma once

#include <jac/machine/class.h>
#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>


template<class Next>
class ConvertFeature : public Next {
    std::optional<jac::Function> toStdVectorConvertor;
    std::optional<jac::Function> toArrayBufferConvertor;

public:
    jac::Value toUint8Array(const std::vector<uint8_t>& data) {
        auto res = jac::ArrayBuffer::create(this->context(), std::span(data));
        return toStdVectorConvertor->call<jac::Value>(res);
    }

    std::vector<uint8_t> toStdVector(jac::Value data) {
        std::vector<uint8_t> dataVec;
        if (JS_IsString(data.getVal())) {
            auto str = data.toString();
            dataVec.assign(str.begin(), str.end());
        }
        else {
            auto res = toArrayBufferConvertor->call<jac::ArrayBuffer>(data);
            auto view = res.typedView<uint8_t>();
            dataVec.assign(view.begin(), view.end());
        }
        return dataVec;
    }

    void initialize() {
        Next::initialize();
        toStdVectorConvertor.emplace(jac::Function::from(this->context(), this->eval("(b) => new Uint8Array(b)", "<util::toUint8Array>")));
        toArrayBufferConvertor.emplace(jac::Function::from(this->context(), this->eval(
        "(d) => {"
            "if (d instanceof ArrayBuffer) return d;"
            "if (ArrayBuffer.isView(d)) return d.buffer;"
            "if (typeof d === 'number') return new Int8Array([d]).buffer;"
            "if (Array.isArray(d)) return new Int8Array(d).buffer;"
            "throw new Error('Invalid data type');"
        "}", "<util::toStdVector>")));
    }
};
