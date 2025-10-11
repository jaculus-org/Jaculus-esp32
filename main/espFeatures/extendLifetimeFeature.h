#pragma once

#include <jac/machine/functionFactory.h>
#include <jac/machine/machine.h>

#include <unordered_set>

template<class Next>
class ExtendLifetimeFeature : public Next {
    std::unordered_set<JSValue> _objects;
public:
    void extendLifetime(jac::Object obj) {
        auto [ctx, val] = obj.loot();
        _objects.insert(val);
    }

    void releaseLifetime(jac::ValueWeak obj) {
        auto it = _objects.find(obj.getVal());
        if (it == _objects.end()) {
            return;
        }

        JS_FreeValue(this->context(), *it);
        _objects.erase(it);
    }

    ~ExtendLifetimeFeature() {
        for (auto val : _objects) {
            JS_FreeValue(this->context(), val);
        }
    }
};
