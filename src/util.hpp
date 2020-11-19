#pragma once

#include <type_traits>
#include <cmath>

namespace auvis
{
    template<typename Scalar>
    Scalar clamp(Scalar x, Scalar low, Scalar high)
    {
        return std::max(low, std::min(high, x));
    }
    

    // Source: https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/smoothstep.xhtml
    template<typename Float, typename std::enable_if<std::is_floating_point<Float>::value>::type* = nullptr>
    Float smoothstep(Float x, Float edge0, Float edge1)
    {
        const Float t = clamp<Float>((x - edge0) / (edge1 - edge0), 0.0, 1.0);
        return t * t * (3.0 - 2.0 * t);
    }
    
    template<typename Float, typename std::enable_if<std::is_floating_point<Float>::value>::type* = nullptr>
    Float mix(Float a, Float b, Float t)
    {
        return (1 - t) * a + t * b;
    }
}