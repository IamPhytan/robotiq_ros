#ifndef ROBOTIQ_TSF_TEST_EIGEN_TEST_UTILS_HPP
#define ROBOTIQ_TSF_TEST_EIGEN_TEST_UTILS_HPP

// gtest helpers for comparing Eigen dense objects coefficient-wise.
//
//   expectEigenFloatEq(actual, Eigen::Vector3f(0.2f, 0.0f, 0.0f));
//
// Each coefficient is compared with EXPECT_FLOAT_EQ; a failure names the
// offending coefficient. Failures are reported at this header's line — wrap
// the call in SCOPED_TRACE if a test has several calls to tell apart.

#include <Eigen/Core>
#include <gtest/gtest.h>

namespace robotiq_tsf::test
{

template <typename DerivedA, typename DerivedB>
void expectEigenFloatEq(const Eigen::DenseBase<DerivedA> &actual,
                        const Eigen::DenseBase<DerivedB> &expected)
{
    ASSERT_EQ(actual.rows(), expected.rows());
    ASSERT_EQ(actual.cols(), expected.cols());
    for (Eigen::Index r = 0; r < actual.rows(); ++r)
    {
        for (Eigen::Index c = 0; c < actual.cols(); ++c)
        {
            EXPECT_FLOAT_EQ(actual.derived().coeff(r, c), expected.derived().coeff(r, c))
                << "coefficient (" << r << ", " << c << ") of\n"
                << actual.derived() << "\nvs expected\n" << expected.derived();
        }
    }
}

}  // namespace robotiq_tsf::test

#endif  // ROBOTIQ_TSF_TEST_EIGEN_TEST_UTILS_HPP
