#pragma once

// For the *_tree.cpp files: a part's setting in a tree.

#include "tesser/node.h"
#include "tessermotion/motion.h"

namespace tessermotion {

// A setting read and written under the part's lock, since the control task
// reads it while transports write it.
template <class T>
tesser::ValueNode& setting(tesser::Object& o, const char* name, Part& part, T& field) {
    return o.value(
        name,
        [&part, &field] {
            Guard g(part.lock());
            return field;
        },
        [&part, &field](T v) {
            Guard g(part.lock());
            field = v;
        });
}

}  // namespace tessermotion
