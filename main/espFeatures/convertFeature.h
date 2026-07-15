#pragma once

#include <memory>

#include <jac/machine/class.h>
#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>


template<class Next>
class ConvertFeature : public Next {
    JSClassID arrayBufferClassId = JS_INVALID_CLASS_ID;

    static void freeVector(JSRuntime*, void* opaque, void*) {
        delete static_cast<std::vector<uint8_t>*>(opaque);
    }

public:
    jac::Value toUint8Array(std::vector<uint8_t> data) {
        auto storage = std::make_unique<std::vector<uint8_t>>(std::move(data));
        JSValue buffer = JS_NewArrayBuffer(
            this->context(), storage->data(), storage->size(), freeVector, storage.get(), false
        );
        if (JS_IsException(buffer)) {
            return jac::Value(this->context(), buffer);
        }
        storage.release();

        JSValue args[] = { buffer, JS_UNDEFINED, JS_UNDEFINED };
        JSValue array = JS_NewTypedArray(this->context(), 3, args, JS_TYPED_ARRAY_UINT8);
        JS_FreeValue(this->context(), buffer);
        return jac::Value(this->context(), array);
    }

    std::vector<uint8_t> toStdVector(jac::Value data) {
        std::vector<uint8_t> dataVec;
        if (JS_IsString(data.getVal())) {
            auto str = data.toString();
            dataVec.assign(str.begin(), str.end());
        }
        else if (JS_IsBool(data.getVal())) {
            dataVec.push_back(static_cast<uint8_t>(JS_ToBool(this->context(), data.getVal())));
        }
        else if (data.isNumber()) {
            dataVec.push_back(static_cast<uint8_t>(data.to<int>()));
        }
        else if (data.isArray()) {
            auto array = data.to<jac::Array>();
            auto length = array.length();
            dataVec.reserve(length);
            for (int i = 0; i < length; ++i) {
                JSValue item = JS_GetPropertyUint32(this->context(), array.getVal(), i);
                int32_t value = 0;
                if (JS_IsException(item) || JS_ToInt32(this->context(), &value, item) < 0) {
                    JS_FreeValue(this->context(), item);
                    throw this->context().getException();
                }
                JS_FreeValue(this->context(), item);
                dataVec.push_back(static_cast<uint8_t>(value));
            }
        }
        else {
            if (JS_GetClassID(data.getVal()) == arrayBufferClassId) {
                size_t size = 0;
                uint8_t* buffer = JS_GetArrayBuffer(this->context(), &size, data.getVal());
                if (size != 0) {
                    dataVec.assign(buffer, buffer + size);
                }
                return dataVec;
            }

            size_t byteOffset = 0;
            size_t byteLength = 0;
            JSValue bufferValue = JS_GetTypedArrayBuffer(this->context(), data.getVal(), &byteOffset, &byteLength, nullptr);
            if (!JS_IsException(bufferValue)) {
                jac::ArrayBuffer buffer(this->context(), bufferValue);
                auto view = buffer.typedView<uint8_t>();
                if (byteLength != 0) {
                    dataVec.assign(view.begin() + byteOffset, view.begin() + byteOffset + byteLength);
                }
                return dataVec;
            }
            JS_FreeValue(this->context(), JS_GetException(this->context()));
            throw jac::Exception::create(jac::Exception::Type::TypeError, "Invalid data type");
        }
        return dataVec;
    }

    void initialize() {
        Next::initialize();
        auto buffer = jac::ArrayBuffer::create(this->context(), 0);
        arrayBufferClassId = JS_GetClassID(buffer.getVal());
    }
};
