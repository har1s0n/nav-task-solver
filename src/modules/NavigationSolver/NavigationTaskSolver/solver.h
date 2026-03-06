#ifndef SOLVER_H
#define SOLVER_H

#include <QVector>
#include <QtMath>
#include <array>

namespace navsolver {
class LeastSquaresSolver final {
public:

   static constexpr int kDim = 4;

   enum class Mode {
      OLS, // W = I
      WLS  // W = diag(1/sigma2)
   };

   struct Options {
      Mode   mode     = Mode::WLS;
      double pivotEps = 1e-12;                   // Порог вырожденности

      bool computeCovarianceApriori     = true;  // Вычислять (H^T W H)^-1
      bool computeCovarianceAposteriori = false; // Вычислять sigma0^2 * (H^T W H)^-1
   };

   struct Observation {
      std::array<double, kDim> H;                // Строка матрицы геометрии [-ux, -uy, -uz, 1]
      double                   y      = qQNaN(); // Вектор невязок Δρ, м
      double                   sigma2 = qQNaN(); // Дисперсия σ² (для WLS), м²
   };

   using Mat4 = std::array<std::array<double, kDim>, kDim>;
   using Vec4 = std::array<double, kDim>;

   struct Result {
      bool success = false;

      int m   = 0;                                      // Число наблюдений
      int dof = 0;                                      // Степени свободы (m - 4)

      Vec4 dx = { qQNaN(), qQNaN(), qQNaN(), qQNaN() }; // Вектор поправок (δX, δY, δZ, cδt)

      double postfit_rms_unweighted = qQNaN();          // Невзвешенное СКО: sqrt(sum(v^2)/m)
      double sigma0                 = qQNaN();          // Ошибка единицы веса: sqrt(v^T W v / dof)
      double postfit_rms            = qQNaN();          // Итоговое RMS (sigma0 для WLS, unweighted для OLS)

      Mat4 P_apriori;                                   // Априорная ковариация (H^T W H)^-1
      Mat4 P_aposteriori;                               // Апостериорная ковариация (sigma0^2 * P_apriori)

      Result() {
         for (int i = 0; i < kDim; ++i) {
            for (int j = 0; j < kDim; ++j) {
               P_apriori[i][j]     = qQNaN();
               P_aposteriori[i][j] = qQNaN();
            }
         }
      }
   };

   [[nodiscard]] static Result solve(const QVector<Observation>& obs,
                                     const Options&              opt);
   [[nodiscard]] static Result solve(const QVector<Observation>& obs);

private:

   [[nodiscard]] static bool invert4x4(const Mat4& A,
                                       double      pivotEps,
                                       Mat4&       Ainv) noexcept;
   [[nodiscard]] static Vec4 mul(const Mat4& A,
                                 const Vec4& x) noexcept;
   [[nodiscard]] static bool isFiniteVec(const Vec4& v) noexcept;
};
} // namespace navsolver
#endif // SOLVER_H
