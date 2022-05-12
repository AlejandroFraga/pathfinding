#ifndef UTIL_H
#define UTIL_H
#pragma once

#include <concepts>
#include <vector>


template<class T, class U>
concept Derived = std::is_base_of<U, T>::value;


class Utils
{
public:
    Utils() = delete;

    template<typename T>
    static size_t contains(const std::vector<T>& nodes, const T& c)
    {
        for (size_t i = 0; i < nodes.size(); ++i)
            if (c == nodes[i])
                return true;

        return false;
    }
    
    template<typename T>
    static size_t coordinate(const std::vector<T>& nodes, const T& c)
    {
        for (size_t i = 0; i < nodes.size(); ++i)
            if (c == nodes[i])
                return i;

        return nodes.size();
    }

    template<typename T>
    static bool isDiagonal(const std::pair<T, T>& pos1, const std::pair<T, T>& pos2)
    {
        return pos1.first != pos2.first && pos1.second != pos2.second;
    }
};

#endif