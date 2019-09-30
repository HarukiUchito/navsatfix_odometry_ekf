#include "lla.hpp"
#include "utility.hpp"

#include <vector>

void MatMul(const char *tr, int n, int k, int m, double alpha,
            const std::vector<double> &A, const std::vector<double> &B, double beta, std::vector<double> &C)
{
    double d;
    int i, j, x, f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);

    for (i = 0; i < n; i++)
        for (j = 0; j < k; j++)
        {
            d = 0.0;
            switch (f)
            {
            case 1:
                for (x = 0; x < m; x++)
                    d += A[i + x * n] * B[x + j * m];
                break;
            case 2:
                for (x = 0; x < m; x++)
                    d += A[i + x * n] * B[j + x * k];
                break;
            case 3:
                for (x = 0; x < m; x++)
                    d += A[x + i * m] * B[x + j * m];
                break;
            case 4:
                for (x = 0; x < m; x++)
                    d += A[x + i * m] * B[j + x * k];
                break;
            }
            if (beta == 0.0)
                C[i + j * n] = alpha * d;
            else
                C[i + j * n] = alpha * d + beta * C[i + j * n];
        }
}

std::vector<double> LLA_to_ENU_Matrix(const LLA &lla)
{
    double sinp = sin(deg2rad(lla.latitude()));
    double cosp = cos(deg2rad(lla.latitude()));
    double sinl = sin(deg2rad(lla.longitude()));
    double cosl = cos(deg2rad(lla.longitude()));

    std::vector<double> E(9);
    E[0] = -sinl;
    E[3] = cosl;
    E[6] = 0.0;
    E[1] = -sinp * cosl;
    E[4] = -sinp * sinl;
    E[7] = cosp;
    E[2] = cosp * cosl;
    E[5] = cosp * sinl;
    E[8] = sinp;
    return E;
}

Point LLA_to_ECEF(const LLA &lla)
{
    double sinp = sin(deg2rad(lla.latitude()));
    double cosp = cos(deg2rad(lla.latitude()));
    double sinl = sin(deg2rad(lla.longitude()));
    double cosl = cos(deg2rad(lla.longitude()));
    double e2 = FE_WGS84 * (2.0 - FE_WGS84), v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);

    return Point{
        (v + lla.altitude()) * cosp * cosl,
        (v + lla.altitude()) * cosp * sinl,
        (v * (1.0 - e2) + lla.altitude()) * sinp
    };
}

Point ECEF_to_ENU(const LLA &origin, const Point &v)
{
    std::vector<double> E = LLA_to_ENU_Matrix(origin);
    std::vector<double> r(3), e(3);
    r[0] = v.x, r[1] = v.y, r[2] = v.z;

    MatMul("NN", 3, 1, 3, 1.0, E, r, 0.0, e);

    return Point{e[0], e[1], e[2]};
}

// Calculate relative position in ENU coordinate
geometry_msgs::Point CalcRelativePosition(const LLA &initial, const LLA &current)
{
    Point initial_ecef = LLA_to_ECEF(initial);
    Point current_ecef = LLA_to_ECEF(current);

    Point ecef_current_v = current_ecef - initial_ecef;
    return GetPointMsg(ECEF_to_ENU(initial, ecef_current_v));
}
