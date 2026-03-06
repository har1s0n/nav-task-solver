#include "solver.h"

namespace navsolver {
LeastSquaresSolver::Vec4 LeastSquaresSolver::mul(const Mat4& A, const Vec4& x) noexcept {
   Vec4 y{ 0.0, 0.0, 0.0, 0.0 };

   for (int i = 0; i < kDim; ++i) {
      double s = 0.0;

      for (int j = 0; j < kDim; ++j) {
         s += A[i][j] * x[j];
      }
      y[i] = s;
   }
   return y;
}

bool LeastSquaresSolver::isFiniteVec(const Vec4& v) noexcept {
   for (double x : v) {
      if (!qIsFinite(x)) {
         return false;
      }
   }
   return true;
}

bool LeastSquaresSolver::invert4x4(const Mat4& A, double pivotEps, Mat4& Ainv) noexcept {
   double aug[kDim][2 * kDim]{};

   for (int i = 0; i < kDim; ++i) {
      for (int j = 0; j < kDim; ++j) {
         aug[i][j] = A[i][j];
      }
      aug[i][kDim + i] = 1.0;
   }

   for (int col = 0; col < kDim; ++col) {
      int pivotRow  = col;
      double maxAbs = qAbs(aug[col][col]);

      for (int r = col + 1; r < kDim; ++r) {
         const double a = qAbs(aug[r][col]);

         if (a > maxAbs) {
            maxAbs   = a;
            pivotRow = r;
         }
      }

      if (!(maxAbs > pivotEps) || !qIsFinite(maxAbs)) {
         return false;
      }

      if (pivotRow != col) {
         for (int j = col; j < 2 * kDim; ++j) {
            std::swap(aug[col][j], aug[pivotRow][j]);
         }
      }

      const double pivot = aug[col][col];

      for (int j = col; j < 2 * kDim; ++j) {
         aug[col][j] /= pivot;
      }

      for (int r = 0; r < kDim; ++r) {
         if (r == col) {
            continue;
         }
         const double f = aug[r][col];

         if (qFuzzyIsNull(f)) {
            continue;
         }

         for (int j = col; j < 2 * kDim; ++j) {
            aug[r][j] -= f * aug[col][j];
         }
      }
   }

   for (int i = 0; i < kDim; ++i) {
      for (int j = 0; j < kDim; ++j) {
         Ainv[i][j] = aug[i][kDim + j];

         if (!qIsFinite(Ainv[i][j])) {
            return false;
         }
      }
   }
   return true;
}

LeastSquaresSolver::Result LeastSquaresSolver::solve(const QVector<Observation>& obs, const Options& opt) {
   Result res;

   res.m = obs.size();

   if (res.m < kDim) {
      return res;
   }

   Mat4 N{};
   Vec4 b{ 0.0, 0.0, 0.0, 0.0 };

   for (int i = 0; i < kDim; ++i) {
      for (int j = 0; j < kDim; ++j) {
         N[i][j] = 0.0;
      }
   }

   for (const auto& o : obs) {
      if (!qIsFinite(o.y) || !isFiniteVec(o.H)) {
         return res;
      }

      double w = 1.0;

      if (opt.mode == Mode::WLS) {
         // Спутник с бесконечной (или 0) дисперсией бракуется целиком
         if (!qIsFinite(o.sigma2) || (o.sigma2 <= 0.0)) {
            return res;
         }
         w = 1.0 / o.sigma2;
      }

      for (int i = 0; i < kDim; ++i) {
         b[i] += w * o.H[i] * o.y;

         for (int j = 0; j <= i; ++j) {
            N[i][j] += w * o.H[i] * o.H[j];
         }
      }
   }

   // Отзеркаливаем треугольник
   for (int i = 0; i < kDim; ++i) {
      for (int j = 0; j < i; ++j) {
         N[j][i] = N[i][j];
      }
   }

   Mat4 Ninv{};

   if (!invert4x4(N, opt.pivotEps, Ninv)) {
      return res;
   }

   res.dx = mul(Ninv, b);

   if (!isFiniteVec(res.dx)) {
      return res;
   }

   double sumV2  = 0.0;
   double sumWV2 = 0.0;

   for (const auto& o : obs) {
      double w = (opt.mode == Mode::WLS) ? (1.0 / o.sigma2) : 1.0;

      const double yhat = o.H[0] * res.dx[0] + o.H[1] * res.dx[1] +
                          o.H[2] * res.dx[2] + o.H[3] * res.dx[3];

      const double v = o.y - yhat;
      sumV2  += v * v;
      sumWV2 += w * v * v;
   }

   res.dof                    = res.m - kDim;
   res.postfit_rms_unweighted = qSqrt(sumV2 / static_cast<double> (res.m));

   if (res.dof > 0) {
      res.sigma0 = qSqrt(sumWV2 / static_cast<double> (res.dof));
   }

   if ((opt.mode == Mode::WLS) && (res.dof > 0)) {
      res.postfit_rms = res.sigma0;
   } else {
      res.postfit_rms = res.postfit_rms_unweighted;
   }

   if (opt.computeCovarianceApriori) {
      res.P_apriori = Ninv;
   }

   if (opt.computeCovarianceAposteriori && (res.dof > 0) && qIsFinite(res.sigma0)) {
      const double s2 = res.sigma0 * res.sigma0;

      for (int i = 0; i < kDim; ++i) {
         for (int j = 0; j < kDim; ++j) {
            res.P_aposteriori[i][j] = s2 * Ninv[i][j];
         }
      }
   }

   res.success = true;
   return res;
}

LeastSquaresSolver::Result LeastSquaresSolver::solve(const QVector<Observation>& obs) {
   return solve(obs, Options());
}
} // namespace navsolver
