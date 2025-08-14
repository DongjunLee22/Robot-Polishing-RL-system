#pragma once
#include <array>
#include <cmath>

namespace math {
	// ZYZ 회전 행렬을 생성하는 함수
    inline std::array<std::array<float, 3>, 3>
        RotationZYZ(float a, float b, float c)
    {
        float ca = cos(a), sa = sin(a);
        float cb = cos(b), sb = sin(b);
        float cc = cos(c), sc = sin(c);

        return { {
            { ca * cb * cc - sa * sc , -ca * cb * sc - sa * cc ,  ca * sb },
            { sa * cb * cc + ca * sc , -sa * cb * sc + ca * cc ,  sa * sb },
            {       -sb * cc     ,         sb * sc      ,   cb  }
        } };
    }

} // namespace math
