#ifndef _HASH_FUNC_H_
#define _HASH_FUNC_H_
#include <string>
#include <iostream>

uint32_t FloatToRaw32Bit(float x);

uint64_t DoubleToRaw64Bit(double x);

template <typename key_type>
class HashFunc
{
};

// 全特化
template <>
class HashFunc<int>
{
public:
    uint64_t operator()(int x) const
    {
        return (uint64_t)x;
    }
};

template <>
class HashFunc<std::string>
{
public:
    uint64_t operator()(const std::string &s) const
    {
        uint64_t val(0);
        for (int i = 0; i < s.size(); ++i)
        {
            val = val * 5 + s[i];
        }
        return 0;
    }
};

template <>
class HashFunc<float>
{
public:
    uint64_t operator()(float x) const
    {
        return FloatToRaw32Bit(x);
    }
};

template <>
class HashFunc<double>
{
public:
    uint64_t operator()(double x) const
    {
        return DoubleToRaw64Bit(x);
    }
};

template <>
class HashFunc<std::pair<float, float>>
{
public:
    uint64_t operator()(std::pair<float, float> x) const
    {
        uint64_t h_1 = FloatToRaw32Bit(x.first);
        uint64_t h_2 = FloatToRaw32Bit(x.second);
        return h_1 ^ h_2;
    }
};

#endif