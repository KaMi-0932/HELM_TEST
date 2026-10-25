#pragma once

#include <Eigen/Core>

/**
 * @brief 递归最小二乘（RLS）估计器。
 *        Recursive least squares (RLS) estimator.
 *
 * @tparam dim 参数维度。
 *             Parameter dimension.
 */
template <uint32_t dim>
class RLS {
 public:
  /// 参数向量类型
  /// Parameter vector type
  using ParamVector = Eigen::Matrix<float, dim, 1>;

  RLS() = delete;

  /**
   * @brief 构造 RLS 估计器。
   *        Construct the RLS estimator.
   *
   * @param delta_ 初始协方差缩放系数。
   *               Initial covariance scale factor.
   * @param lambda_ 遗忘因子。
   *                Forgetting factor.
   */
  constexpr RLS(float delta_, float lambda_)
      : dimension_(dim),
        lambda_(lambda_),
        delta_(delta_),
        defaultparamsvector_(ParamVector::Zero()) {
    this->Reset();  // 初始化各个矩阵
  }

  /**
   * @brief 重置估计器状态。
   *        Reset the estimator state.
   */
  void Reset() {
    transmatrix_ = Eigen::Matrix<float, dim, dim>::Identity() * delta_;
    gainvector_ = ParamVector::Zero();
    paramsvector_ = ParamVector::Zero();
  }

  /**
   * @brief 执行一次 RLS 更新。
   *        Perform one RLS update.
   *
   * @param sample_vector 输入样本向量。
   *                      Input sample vector.
   * @param actual_output 实际输出。
   *                      Actual output.
   * @return 当前参数估计的引用。
   *         Reference to the current parameter estimate.
   */
  const ParamVector& Update(const ParamVector& sample_vector,
                            float actual_output) {
    gainvector_ = (transmatrix_ * sample_vector) /
                  (1.0f + (sample_vector.transpose() * transmatrix_ *
                           sample_vector)(0, 0) /
                              lambda_) /
                  lambda_;
    paramsvector_ +=
        gainvector_ *
        (actual_output - (sample_vector.transpose() * paramsvector_)(0, 0));
    transmatrix_ = (transmatrix_ -
                    gainvector_ * sample_vector.transpose() * transmatrix_) /
                   lambda_;

    return paramsvector_;
  }

  /**
   * @brief 手动设置参数向量。
   *        Set the parameter vector manually.
   *
   * @param updated_params 参数向量。
   *                       Parameter vector.
   */
  void SetParamVector(const ParamVector& updated_params) {
    paramsvector_ = updated_params;
    defaultparamsvector_ = updated_params;
  }

 private:
  uint32_t dimension_;
  float lambda_;
  float delta_;

  Eigen::Matrix<float, dim, dim> transmatrix_;
  ParamVector gainvector_;
  ParamVector paramsvector_;
  ParamVector defaultparamsvector_;
};
