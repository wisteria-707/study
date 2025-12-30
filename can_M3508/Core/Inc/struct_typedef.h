// struct_typedef.h
#ifndef STRUCT_TYPEDEF_H
//等价于 "if not defined"，判断宏是否未被定义
#define STRUCT_TYPEDEF_H
//定义该宏
//防止头文件被多次包含

#include <stdint.h>
//解决不同平台，不同编译器上相同类型名所占字节不同的问题

typedef int8_t   s8;
typedef uint8_t  u8;
typedef int16_t  s16;
typedef uint16_t u16;
typedef int32_t  s32;
typedef uint32_t u32;
typedef float    f32;
typedef double   f64;
//为已有的数据类型创建一个别名

#endif
//专门用于结束#if、#ifdef、#ifndef开头的条件编译块，必须成对出现

