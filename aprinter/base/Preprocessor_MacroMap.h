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

// This is made to be included from Preprocessor.h, don't include directly.

#define APRINTER_AS_NUM_MACRO_ARGS(...) APRINTER_AS_NUM_MACRO_ARGS_HELPER1(-1, ##__VA_ARGS__, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)
#define APRINTER_AS_NUM_MACRO_ARGS_HELPER1(_dummy, ...) APRINTER_AS_NUM_MACRO_ARGS_HELPER2(_dummy, ##__VA_ARGS__)
#define APRINTER_AS_NUM_MACRO_ARGS_HELPER2(_dummy, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, N, ...) N

#define APRINTER_NUM_TUPLE_ARGS(tuple) APRINTER_AS_NUM_MACRO_ARGS tuple

#define  APRINTER_AS_GET_1(p1, ...) p1
#define  APRINTER_AS_GET_2(p1, p2, ...) p2
#define  APRINTER_AS_GET_3(p1, p2, p3, ...) p3
#define  APRINTER_AS_GET_4(p1, p2, p3, p4, ...) p4
#define  APRINTER_AS_GET_5(p1, p2, p3, p4, p5, ...) p5
#define  APRINTER_AS_GET_6(p1, p2, p3, p4, p5, p6, ...) p6
#define  APRINTER_AS_GET_7(p1, p2, p3, p4, p5, p6, p7, ...) p7
#define  APRINTER_AS_GET_8(p1, p2, p3, p4, p5, p6, p7, p8, ...) p8
#define  APRINTER_AS_GET_9(p1, p2, p3, p4, p5, p6, p7, p8, p9, ...) p9
#define APRINTER_AS_GET_10(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, ...) p10
#define APRINTER_AS_GET_11(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, ...) p11
#define APRINTER_AS_GET_12(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, ...) p12
#define APRINTER_AS_GET_13(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, ...) p13
#define APRINTER_AS_GET_14(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, ...) p14
#define APRINTER_AS_GET_15(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, ...) p15
#define APRINTER_AS_GET_16(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, ...) p16
#define APRINTER_AS_GET_17(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, p17, ...) p17
#define APRINTER_AS_GET_18(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, p17, p18, ...) p18
#define APRINTER_AS_GET_19(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, p17, p18, p19, ...) p19
#define APRINTER_AS_GET_20(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, p17, p18, p19, p20, ...) p20
#define APRINTER_AS_GET_21(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, p17, p18, p19, p20, p21, ...) p21
#define APRINTER_AS_GET_22(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, p17, p18, p19, p20, p21, p22, ...) p22

#define  APRINTER_AS_MAP_1(f, del, arg, pars)                                                  f(arg,  APRINTER_AS_GET_1 pars)
#define  APRINTER_AS_MAP_2(f, del, arg, pars)  APRINTER_AS_MAP_1(f, del, arg, pars) del(dummy) f(arg,  APRINTER_AS_GET_2 pars)
#define  APRINTER_AS_MAP_3(f, del, arg, pars)  APRINTER_AS_MAP_2(f, del, arg, pars) del(dummy) f(arg,  APRINTER_AS_GET_3 pars)
#define  APRINTER_AS_MAP_4(f, del, arg, pars)  APRINTER_AS_MAP_3(f, del, arg, pars) del(dummy) f(arg,  APRINTER_AS_GET_4 pars)
#define  APRINTER_AS_MAP_5(f, del, arg, pars)  APRINTER_AS_MAP_4(f, del, arg, pars) del(dummy) f(arg,  APRINTER_AS_GET_5 pars)
#define  APRINTER_AS_MAP_6(f, del, arg, pars)  APRINTER_AS_MAP_5(f, del, arg, pars) del(dummy) f(arg,  APRINTER_AS_GET_6 pars)
#define  APRINTER_AS_MAP_7(f, del, arg, pars)  APRINTER_AS_MAP_6(f, del, arg, pars) del(dummy) f(arg,  APRINTER_AS_GET_7 pars)
#define  APRINTER_AS_MAP_8(f, del, arg, pars)  APRINTER_AS_MAP_7(f, del, arg, pars) del(dummy) f(arg,  APRINTER_AS_GET_8 pars)
#define  APRINTER_AS_MAP_9(f, del, arg, pars)  APRINTER_AS_MAP_8(f, del, arg, pars) del(dummy) f(arg,  APRINTER_AS_GET_9 pars)
#define APRINTER_AS_MAP_10(f, del, arg, pars)  APRINTER_AS_MAP_9(f, del, arg, pars) del(dummy) f(arg, APRINTER_AS_GET_10 pars)
#define APRINTER_AS_MAP_11(f, del, arg, pars) APRINTER_AS_MAP_10(f, del, arg, pars) del(dummy) f(arg, APRINTER_AS_GET_11 pars)
#define APRINTER_AS_MAP_12(f, del, arg, pars) APRINTER_AS_MAP_11(f, del, arg, pars) del(dummy) f(arg, APRINTER_AS_GET_12 pars)
#define APRINTER_AS_MAP_13(f, del, arg, pars) APRINTER_AS_MAP_12(f, del, arg, pars) del(dummy) f(arg, APRINTER_AS_GET_13 pars)
#define APRINTER_AS_MAP_14(f, del, arg, pars) APRINTER_AS_MAP_13(f, del, arg, pars) del(dummy) f(arg, APRINTER_AS_GET_14 pars)
#define APRINTER_AS_MAP_15(f, del, arg, pars) APRINTER_AS_MAP_14(f, del, arg, pars) del(dummy) f(arg, APRINTER_AS_GET_15 pars)
#define APRINTER_AS_MAP_16(f, del, arg, pars) APRINTER_AS_MAP_15(f, del, arg, pars) del(dummy) f(arg, APRINTER_AS_GET_16 pars)
#define APRINTER_AS_MAP_17(f, del, arg, pars) APRINTER_AS_MAP_16(f, del, arg, pars) del(dummy) f(arg, APRINTER_AS_GET_17 pars)
#define APRINTER_AS_MAP_18(f, del, arg, pars) APRINTER_AS_MAP_17(f, del, arg, pars) del(dummy) f(arg, APRINTER_AS_GET_18 pars)
#define APRINTER_AS_MAP_19(f, del, arg, pars) APRINTER_AS_MAP_18(f, del, arg, pars) del(dummy) f(arg, APRINTER_AS_GET_19 pars)
#define APRINTER_AS_MAP_20(f, del, arg, pars) APRINTER_AS_MAP_19(f, del, arg, pars) del(dummy) f(arg, APRINTER_AS_GET_20 pars)
#define APRINTER_AS_MAP_21(f, del, arg, pars) APRINTER_AS_MAP_20(f, del, arg, pars) del(dummy) f(arg, APRINTER_AS_GET_21 pars)
#define APRINTER_AS_MAP_22(f, del, arg, pars) APRINTER_AS_MAP_21(f, del, arg, pars) del(dummy) f(arg, APRINTER_AS_GET_22 pars)

#define APRINTER_AS_MAP(f, del, arg, pars) APRINTER_JOIN(APRINTER_AS_MAP_, APRINTER_NUM_TUPLE_ARGS(pars))(f, del, arg, pars)

#define APRINTER_AS_MAP_DELIMITER_NONE(dummy)
#define APRINTER_AS_MAP_DELIMITER_COMMA(dummy) ,
