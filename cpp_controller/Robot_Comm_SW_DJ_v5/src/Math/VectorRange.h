#pragma once
#include <vector>
#include <cmath>

namespace math {
    // 시작과 끝값 간에 균등하게 n개의 값을 생성하는 함수
	//   - s: 시작값, e: 끝값, n: 생성할 값의 개수
    inline std::vector<double> linspace(double s, double e, int n) {
        std::vector<double> v; v.reserve(n);
        if (n < 2) { v.push_back(s); return v; }
        double step = (e - s) / (n - 1);
        for (int i = 0; i < n; ++i) v.push_back(s + i * step);
        v.back() = e; return v;
    }

	// 시작값, 스텝, 끝값을 이용하여 균등하게 값을 생성하는 함수
    inline std::vector<double> colon(double s, double step, double e) {
        std::vector<double> v;
        if (step == 0) { v.push_back(s); return v; }
        double tol = std::fabs(step) * 1e-6;
        if (step > 0)  for (double x = s; x <= e + tol; x += step) v.push_back((x > e && std::fabs(x - e) < tol) ? e : x);
        else        for (double x = s; x >= e - tol; x += step) v.push_back((x < e && std::fabs(x - e) < tol) ? e : x);
        return v;
    }

} // namespace math
