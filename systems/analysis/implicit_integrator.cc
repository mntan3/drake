#include "drake/systems/analysis/implicit_integrator.h"

#include <cmath>
#include <stdexcept>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace systems {

template <class T>
void ImplicitIntegrator<T>::DoResetStatistics() {
  num_iter_factorizations_ = 0;
  num_jacobian_function_evaluations_ = 0;
  num_jacobian_evaluations_ = 0;
  DoResetImplicitIntegratorStatistics();
}

// Computes the Jacobian of the ordinary differential equations around time
// and continuous state `(t, xt)` using automatic differentiation.
// @param system The dynamical system.
// @param t the time around which to compute the Jacobian matrix.
// @param xt the continuous state around which to compute the Jacobian matrix.
// @param context the Context of the system, at time and continuous state
//        unknown.
// @param [out] the Jacobian matrix around time and state `(t, xt)`.
// @note The continuous state will be indeterminate on return.
template <class T>
void ImplicitIntegrator<T>::ComputeAutoDiffJacobian(
    const System<T>& system, const T& t, const VectorX<T>& xt,
    const Context<T>& context, MatrixX<T>* J) {
  DRAKE_LOGGER_DEBUG("  ImplicitIntegrator Compute Autodiff Jacobian t={}", t);
  // Create AutoDiff versions of the state vector.
  VectorX<AutoDiffXd> a_xt = xt;

  // Set the size of the derivatives and prepare for Jacobian calculation.
  const int n_state_dim = a_xt.size();
  for (int i = 0; i < n_state_dim; ++i)
    a_xt[i].derivatives() = VectorX<T>::Unit(n_state_dim, i);

  // Get the system and the context in AutoDiffable format. Inputs must also
  // be copied to the context used by the AutoDiff'd system (which is
  // accomplished using FixInputPortsFrom()).
  // TODO(edrumwri): Investigate means for moving as many of the operations
  //                 below offline (or with lower frequency than once-per-
  //                 Jacobian calculation) as is possible. These operations
  //                 are likely to be expensive.
  const auto adiff_system = system.ToAutoDiffXd();
  std::unique_ptr<Context<AutoDiffXd>> adiff_context = adiff_system->
      AllocateContext();
  adiff_context->SetTimeStateAndParametersFrom(context);
  adiff_system->FixInputPortsFrom(system, context, adiff_context.get());
  adiff_context->SetTime(t);

  // Set the continuous state in the context.
  adiff_context->SetContinuousState(a_xt);

  // Evaluate the derivatives at that state.
  const VectorX<AutoDiffXd> result =
      this->EvalTimeDerivatives(*adiff_system, *adiff_context).CopyToVector();

  *J = math::autoDiffToGradientMatrix(result);
}

// Computes the Jacobian of the ordinary differential equations around time
// and continuous state `(t, xt)` using a first-order forward difference (i.e.,
// numerical differentiation).
// @param system The dynamical system.
// @param t the time around which to compute the Jacobian matrix.
// @param xt the continuous state around which to compute the Jacobian matrix.
// @param context the Context of the system, at time and continuous state
//        unknown.
// @param [out] the Jacobian matrix around time and state `(t, xt)`.
// @note The continuous state will be indeterminate on return.
template <class T>
void ImplicitIntegrator<T>::ComputeForwardDiffJacobian(
    const System<T>&, const T& t, const VectorX<T>& xt, Context<T>* context,
    MatrixX<T>* J) {
  using std::abs;

  // Set epsilon to the square root of machine precision.
  const double eps = std::sqrt(std::numeric_limits<double>::epsilon());

  // Get the number of continuous state variables xt.
  const int n = context->num_continuous_states();

  DRAKE_LOGGER_DEBUG(
      "  ImplicitIntegrator Compute Forwarddiff {}-Jacobian t={}", n, t);
  DRAKE_LOGGER_DEBUG(
      "  computing from state {}", xt.transpose());

  // Initialize the Jacobian.
  J->resize(n, n);

  // Evaluate f(t,xt).
  context->SetTimeAndContinuousState(t, xt);
  const VectorX<T> f = this->EvalTimeDerivatives(*context).CopyToVector();

  // Compute the Jacobian.
  VectorX<T> xt_prime = xt;
  for (int i = 0; i < n; ++i) {
    // Compute a good increment to the dimension using approximately 1/eps
    // digits of precision. Note that if |xt| is large, the increment will
    // be large as well. If |xt| is small, the increment will be no smaller
    // than eps.
    const T abs_xi = abs(xt(i));
    T dxi(abs_xi);
    if (dxi <= 1) {
      // When |xt[i]| is small, increment will be eps.
      dxi = eps;
    } else {
      // |xt[i]| not small; make increment a fraction of |xt[i]|.
      dxi = eps * abs_xi;
    }

    // Update xt', minimizing the effect of roundoff error by ensuring that
    // x and dx differ by an exactly representable number. See p. 192 of
    // Press, W., Teukolsky, S., Vetterling, W., and Flannery, P. Numerical
    //   Recipes in C++, 2nd Ed., Cambridge University Press, 2002.
    xt_prime(i) = xt(i) + dxi;
    dxi = xt_prime(i) - xt(i);

    // TODO(sherm1) This is invalidating q, v, and z but we only changed one.
    //              Switch to a method that invalides just the relevant
    //              partition, and ideally modify only the one changed element.
    // Compute f' and set the relevant column of the Jacobian matrix.
    context->SetTimeAndContinuousState(t, xt_prime);
    J->col(i) = (this->EvalTimeDerivatives(*context).CopyToVector() - f) / dxi;

    // Reset xt' to xt.
    xt_prime(i) = xt(i);
  }
}

// Computes the Jacobian of the ordinary differential equations around time
// and continuous state `(t, xt)` using a second-order central difference (i.e.,
// numerical differentiation).
// @param system The dynamical system.
// @param t the time around which to compute the Jacobian matrix.
// @param xt the continuous state around which to compute the Jacobian matrix.
// @param context the Context of the system, at time and continuous state
//        unknown.
// @param [out] the Jacobian matrix around time and state `(t, xt)`.
// @note The continuous state will be indeterminate on return.
template <class T>
void ImplicitIntegrator<T>::ComputeCentralDiffJacobian(
    const System<T>&, const T& t, const VectorX<T>& xt, Context<T>* context,
    MatrixX<T>* J) {
  using std::abs;

  // Cube root of machine precision (indicated by theory) seems a bit coarse.
  // Pick power of eps halfway between 6/12 (i.e., 1/2) and 4/12 (i.e., 1/3).
  const double eps = std::pow(std::numeric_limits<double>::epsilon(), 5.0/12);

  // Get the number of continuous state variables xt.
  const int n = context->num_continuous_states();

  DRAKE_LOGGER_DEBUG(
      "  ImplicitIntegrator Compute Centraldiff {}-Jacobian t={}", n, t);

  // Initialize the Jacobian.
  J->resize(n, n);

  // Evaluate f(t,xt).
  context->SetTimeAndContinuousState(t, xt);
  const VectorX<T> f = this->EvalTimeDerivatives(*context).CopyToVector();

  // Compute the Jacobian.
  VectorX<T> xt_prime = xt;
  for (int i = 0; i < n; ++i) {
    // Compute a good increment to the dimension using approximately 1/eps
    // digits of precision. Note that if |xt| is large, the increment will
    // be large as well. If |xt| is small, the increment will be no smaller
    // than eps.
    const T abs_xi = abs(xt(i));
    T dxi(abs_xi);
    if (dxi <= 1) {
      // When |xt[i]| is small, increment will be eps.
      dxi = eps;
    } else {
      // |xt[i]| not small; make increment a fraction of |xt[i]|.
      dxi = eps * abs_xi;
    }

    // Update xt', minimizing the effect of roundoff error, by ensuring that
    // x and dx differ by an exactly representable number. See p. 192 of
    // Press, W., Teukolsky, S., Vetterling, W., and Flannery, P. Numerical
    //   Recipes in C++, 2nd Ed., Cambridge University Press, 2002.
    xt_prime(i) = xt(i) + dxi;
    const T dxi_plus = xt_prime(i) - xt(i);

    // TODO(sherm1) This is invalidating q, v, and z but we only changed one.
    //              Switch to a method that invalides just the relevant
    //              partition, and ideally modify only the one changed element.
    // Compute f(x+dx).
    context->SetContinuousState(xt_prime);
    VectorX<T> fprime_plus = this->EvalTimeDerivatives(*context).CopyToVector();

    // Update xt' again, minimizing the effect of roundoff error.
    xt_prime(i) = xt(i) - dxi;
    const T dxi_minus = xt(i) - xt_prime(i);

    // Compute f(x-dx).
    context->SetContinuousState(xt_prime);
    VectorX<T> fprime_minus = this->EvalTimeDerivatives(
        *context).CopyToVector();

    // Set the Jacobian column.
    J->col(i) = (fprime_plus - fprime_minus) / (dxi_plus + dxi_minus);

    // Reset xt' to xt.
    xt_prime(i) = xt(i);
  }
}

// Factors a dense matrix (the iteration matrix) using LU factorization,
// which should be faster than the QR factorization used in the specialized
// template method immediately below.
template <class T>
void ImplicitIntegrator<T>::IterationMatrix::SetAndFactorIterationMatrix(
    const MatrixX<T>& iteration_matrix) {
  LU_.compute(iteration_matrix);
  matrix_factored_ = true;
}

// Solves a linear system Ax = b for x using the iteration matrix (A)
// factored using LU decomposition.
// @see Factor()
template <class T>
VectorX<T> ImplicitIntegrator<T>::IterationMatrix::Solve(
    const VectorX<T>& b) const {
  return LU_.solve(b);
}

template <typename T>
typename ImplicitIntegrator<T>::ConvergenceStatus
ImplicitIntegrator<T>::CheckNewtonConvergence(
    int iteration, const VectorX<T>& xtplus, const VectorX<T>& dx,
    const T& dx_norm, const T& last_dx_norm) const {
  // The check below looks for convergence by identifying cases where the
  // update to the state results in no change.
  // Note: Since we are performing this check at the end of the iteration,
  // after xtplus has been updated, we also know that there is at least some
  // change to the state, no matter how small, on a non-stationary system.
  // Future maintainers should make sure this check only occurs after a change
  // has been made to the state.
  if (this->IsUpdateZero(xtplus, dx)) {
    DRAKE_LOGGER_DEBUG("magnitude of state update indicates convergence");
    return ConvergenceStatus::kConverged;
  }

  // Compute the convergence rate and check convergence.
  // [Hairer, 1996] notes that this convergence strategy should only be applied
  // after *at least* two iterations (p. 121). In practice, we find that it
  // needs to run at least three iterations otherwise some error-controlled runs
  // may choke, hence we check if iteration > 1.
  if (iteration > 1) {
    // TODO(edrumwri) Hairer's RADAU5 implementation (allegedly) uses
    // theta = sqrt(dx[k] / dx[k-2]) while DASSL uses
    // theta = pow(dx[k] / dx[0], 1/k), so investigate setting
    // theta to these alternative values for minimizing convergence failures.
    const T theta = dx_norm / last_dx_norm;
    const T eta = theta / (1 - theta);
    DRAKE_LOGGER_DEBUG("Newton-Raphson loop {} theta: {}, eta: {}",
                iteration, theta, eta);

    // Look for divergence.
    if (theta > 1) {
      DRAKE_LOGGER_DEBUG("Newton-Raphson divergence detected");
      return ConvergenceStatus::kDiverged;
    }

    // Look for convergence using Equation IV.8.10 from [Hairer, 1996].
    // [Hairer, 1996] determined values of kappa in [0.01, 0.1] work most
    // efficiently on a number of test problems with *Radau5* (a fifth order
    // implicit integrator), p. 121. We select a value halfway in-between.
    const double kappa = 0.05;
    const double k_dot_tol = kappa * this->get_accuracy_in_use();
    if (eta * dx_norm < k_dot_tol) {
      DRAKE_LOGGER_DEBUG("Newton-Raphson converged; η = {}", eta);
      return ConvergenceStatus::kConverged;
    }
  }

  return ConvergenceStatus::kNotConverged;
}


template <class T>
bool ImplicitIntegrator<T>::IsBadJacobian(const MatrixX<T>& J) const {
  return !J.allFinite();
}

// Compute the partial derivative of the ordinary differential equations with
// respect to the state variables for a given x(t).
// @post the context's time and continuous state will be temporarily set during
//       this call (and then reset to their original values) on return.
template <class T>
const MatrixX<T>& ImplicitIntegrator<T>::CalcJacobian(const T& t,
    const VectorX<T>& x) {
  // We change the context but will change it back.
  Context<T>* context = this->get_mutable_context();

  // Get the current time and state.
  const T t_current = context->get_time();
  const VectorX<T> x_current = context->get_continuous_state_vector().
      CopyToVector();

  // Update the time and state.
  context->SetTimeAndContinuousState(t, x);
  num_jacobian_evaluations_++;

  // Get the current number of ODE evaluations.
  int64_t current_ODE_evals = this->get_num_derivative_evaluations();

  // Get a the system.
  const System<T>& system = this->get_system();

  // TODO(edrumwri): Give the caller the option to provide their own Jacobian.
  [this, context, &system, &t, &x]() {
    switch (jacobian_scheme_) {
      case JacobianComputationScheme::kForwardDifference:
        ComputeForwardDiffJacobian(system, t, x, &*context, &J_);
        break;

      case JacobianComputationScheme::kCentralDifference:
        ComputeCentralDiffJacobian(system, t, x, &*context, &J_);
        break;

      case JacobianComputationScheme::kAutomatic:
        ComputeAutoDiffJacobian(system, t, x, *context, &J_);
        break;
    }
  }();

  // Use the new number of ODE evaluations to determine the number of Jacobian
  // evaluations.
  num_jacobian_function_evaluations_ += this->get_num_derivative_evaluations()
      - current_ODE_evals;

  // Reset the time and state.
  context->SetTimeAndContinuousState(t_current, x_current);

  return J_;
}

template <class T>
void ImplicitIntegrator<T>::FreshenMatricesIfFullNewton(
    const T& t, const VectorX<T>& xt, const T& h,
    const std::function<void(const MatrixX<T>&, const T&,
        typename ImplicitIntegrator<T>::IterationMatrix*)>&
        compute_and_factor_iteration_matrix,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix) {
  DRAKE_DEMAND(iteration_matrix);

  // Return immediately if full-Newton is not in use.
  if (!get_use_full_newton()) return;

  // Compute the initial Jacobian and iteration matrices and factor them.
  MatrixX<T>& J = get_mutable_jacobian();
  J = CalcJacobian(t, xt);
  ++num_iter_factorizations_;
  compute_and_factor_iteration_matrix(J, h, iteration_matrix);
}

template <class T>
bool ImplicitIntegrator<T>::MaybeFreshenMatrices(
    const T& t, const VectorX<T>& xt, const T& h, int trial,
    const std::function<void(const MatrixX<T>&, const T&,
        typename ImplicitIntegrator<T>::IterationMatrix*)>&
        compute_and_factor_iteration_matrix,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix) {
  // Compute the initial Jacobian and iteration matrices and factor them, if
  // necessary.
  MatrixX<T>& J = get_mutable_jacobian();
  if (!get_reuse() || J.rows() == 0 || IsBadJacobian(J)) {
    J = CalcJacobian(t, xt);
    ++num_iter_factorizations_;
    compute_and_factor_iteration_matrix(J, h, iteration_matrix);
    return true;  // Indicate success.
  }

  // Reuse is activated, Jacobian is fully sized, and Jacobian is not "bad".
  // If the iteration matrix has not been set and factored, do only that.
  if (!iteration_matrix->matrix_factored()) {
    ++num_iter_factorizations_;
    compute_and_factor_iteration_matrix(J, h, iteration_matrix);
    return true;  // Indicate success.
  }

  switch (trial) {
    case 1:
      // For the first trial, we do nothing: this will cause the Newton-Raphson
      // process to use the last computed (and already factored) iteration
      // matrix.
      return true;  // Indicate success.

    case 2: {
      // For the second trial, we perform the (likely) next least expensive
      // operation, re-constructing and factoring the iteration matrix.
      ++num_iter_factorizations_;
      compute_and_factor_iteration_matrix(J, h, iteration_matrix);
      return true;
    }

    case 3: {
      // For the third trial, the Jacobian matrix may already be "fresh",
      // meaning that there is nothing more that can be tried (Jacobian and
      // iteration matrix are both fresh) and we need to indicate failure.
      if (jacobian_is_fresh_)
        return false;

      // Reform the Jacobian matrix and refactor the iteration matrix.
      J = CalcJacobian(t, xt);
      ++num_iter_factorizations_;
      compute_and_factor_iteration_matrix(J, h, iteration_matrix);
      return true;

      case 4: {
        // Trial #4 indicates failure.
        return false;
      }

      default:
        throw std::domain_error("Unexpected trial number.");
    }
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::ImplicitIntegrator)
