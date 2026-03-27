//
// Created by larr-planning on 24. 3. 3.
//

#ifndef BPMP_UTILS_EIGENMVN_H
#define BPMP_UTILS_EIGENMVN_H

#include <Eigen/Dense>
#include <random>

/*
  We need a functor that can pretend it's const,
  but to be a good random number generator
  it needs mutable state.  The standard Eigen function
  Random() just calls rand(), which changes a global
  variable.
*/
namespace Eigen {
    namespace internal {
        template<typename Scalar>
        struct scalar_normal_dist_op
        {
            static std::mt19937 rng;                        // The uniform pseudo-random algorithm
            mutable std::normal_distribution<Scalar> norm; // gaussian combinator

            EIGEN_EMPTY_STRUCT_CTOR(scalar_normal_dist_op)

            template<typename Index>
            inline const Scalar operator() (Index, Index = 0) const { return norm(rng); }
            inline void seed(const uint64_t &s) { rng.seed(s); }
        };

        template<typename Scalar>
        std::mt19937 scalar_normal_dist_op<Scalar>::rng;

        template<typename Scalar>
        struct functor_traits<scalar_normal_dist_op<Scalar> >
        { enum { Cost = 50 * NumTraits<Scalar>::MulCost, PacketAccess = false, IsRepeatable = false }; };

    } // end namespace internal

    /**
      Find the eigen-decomposition of the covariance matrix
      and then store it for sampling from a multi-variate normal
    */
    template<typename Scalar>
    class EigenMultivariateNormal
    {
        Matrix<Scalar,Dynamic,Dynamic> _covar;
        Matrix<Scalar,Dynamic,Dynamic> _transform;
        Matrix< Scalar, Dynamic, 1> _mean;
        internal::scalar_normal_dist_op<Scalar> randN; // Gaussian functor
        bool _use_cholesky;
        SelfAdjointEigenSolver<Matrix<Scalar,Dynamic,Dynamic> > _eigenSolver; // drawback: this creates a useless eigenSolver when using Cholesky decomposition, but it yields access to eigenvalues and vectors

    public:
        EigenMultivariateNormal(const Matrix<Scalar,Dynamic,1>& mean,const Matrix<Scalar,Dynamic,Dynamic>& covar,
                                const bool use_cholesky=false,const uint64_t &seed=std::mt19937::default_seed)
                :_use_cholesky(use_cholesky)
        {
            randN.seed(seed);
            setMean(mean);
            setCovar(covar);
        }

        void setMean(const Matrix<Scalar,Dynamic,1>& mean) { _mean = mean; }
        void setCovar(const Matrix<Scalar,Dynamic,Dynamic>& covar)
        {
            _covar = covar;

            // Assuming that we'll be using this repeatedly,
            // compute the transformation matrix that will
            // be applied to unit-variance independent normals

            if (_use_cholesky)
            {
                Eigen::LLT<Eigen::Matrix<Scalar,Dynamic,Dynamic> > cholSolver(_covar);
                // We can only use the cholesky decomposition if
                // the covariance matrix is symmetric, pos-definite.
                // But a covariance matrix might be pos-semi-definite.
                // In that case, we'll go to an EigenSolver
                if (cholSolver.info()==Eigen::Success)
                {
                    // Use cholesky solver
                    _transform = cholSolver.matrixL();
                }
                else
                {
                    throw std::runtime_error("Failed computing the Cholesky decomposition. Use solver instead");
                }
            }
            else
            {
                _eigenSolver = SelfAdjointEigenSolver<Matrix<Scalar,Dynamic,Dynamic> >(_covar);
                _transform = _eigenSolver.eigenvectors()*_eigenSolver.eigenvalues().cwiseMax(0).cwiseSqrt().asDiagonal();
            }
        }

        /// Draw nn samples from the gaussian and return them
        /// as columns in a Dynamic by nn matrix
        Matrix<Scalar,Dynamic,-1> samples(int nn)
        {
            return (_transform * Matrix<Scalar,Dynamic,-1>::NullaryExpr(_covar.rows(),nn,randN)).colwise() + _mean;
        }
    }; // end class EigenMultivariateNormal
} // end namespace Eigen

#endif //BPMP_UTILS_EIGENMVN_H
