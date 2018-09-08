/*
 * Copyright (c) 2016 Ambroz Bizjak
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef APRINTER_PREPROCESSOR_H
#define APRINTER_PREPROCESSOR_H

#define APRINTER_JOIN_HELPER2(x, y) x##y
#define APRINTER_JOIN_HELPER1(x, y) APRINTER_JOIN_HELPER2(x, y)
#define APRINTER_JOIN_HELPER(x, y)  APRINTER_JOIN_HELPER1(x, y)
#define APRINTER_JOIN(x, y) APRINTER_JOIN_HELPER(x, y)

#define AMBRO_STRINGIFY_HELPER(x) #x
#define AMBRO_STRINGIFY(x) AMBRO_STRINGIFY_HELPER(x)

#define APRINTER_REMOVE_PARENS(...) __VA_ARGS__

#define APRINTER_ARRAY_LEN(arr) (sizeof(arr) / sizeof((arr)[0]))


#define APRINTER_HASPAREN(definition)     APRINTER_HASPAREN1(definition)
#define APRINTER_HASPAREN_HELPER(...)     0,
#define APRINTER_HASPAREN1(definition)    APRINTER_HASPAREN2(APRINTER_HASPAREN_HELPER definition)
#define APRINTER_HASPAREN2(comma_or_junk) APRINTER_HASPAREN3(comma_or_junk YES, NO)
#define APRINTER_HASPAREN3(ignore, check, ...) check

#define APRINTER_IF_DEFINED_NO(...)
#define APRINTER_IF_DEFINED_YES(...)  __VA_ARGS__
#define APRINTER_IF_DEFINED(deinition) APRINTER_JOIN(APRINTER_IF_DEFINED_, APRINTER_HASPAREN(deinition))


#include "Preprocessor_MacroMap.h"

#define APRINTER_USE_TYPE1(namespace, type_name) using type_name = typename namespace::type_name;
#define APRINTER_USE_TYPE2(namespace, type_name) using type_name = namespace::type_name;
#define APRINTER_USE_VAL(namespace, value_name) static constexpr auto value_name = namespace::value_name;

#define APRINTER_USE_TYPES1(namespace, type_names) APRINTER_AS_MAP(APRINTER_USE_TYPE1, APRINTER_AS_MAP_DELIMITER_NONE, namespace, type_names)
#define APRINTER_USE_TYPES2(namespace, type_names) APRINTER_AS_MAP(APRINTER_USE_TYPE2, APRINTER_AS_MAP_DELIMITER_NONE, namespace, type_names)
#define APRINTER_USE_VALS(namespace,   type_names) APRINTER_AS_MAP(APRINTER_USE_VAL,   APRINTER_AS_MAP_DELIMITER_NONE, namespace, type_names)

#endif
