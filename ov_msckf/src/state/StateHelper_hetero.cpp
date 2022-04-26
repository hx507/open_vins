/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "StateHelper.h"
#include "heterocc.h"

using namespace ov_core;
using namespace ov_msckf;

// void dummy()
//{
// size_t dummy_size = 100;
// int* dummy_mem = new int[dummy_size];
// void* DFG = __hetero_launch_begin(1, dummy_mem, dummy_size,
// 1, dummy_mem, dummy_size);
// void* Section = __hetero_section_begin();

// void* Wrapper = __hetero_task_begin(1, dummy_mem, dummy_size,
// 1, dummy_mem, dummy_size, "stencil_wrapper_task");
// void* Section_Wrapped = __hetero_section_begin();

// for (size_t c = 0; c < 100; c++) {
//__hetero_parallel_loop(1, 1, dummy_mem, dummy_size,
// 1, dummy_mem, dummy_size, "stencil_parallel_loop");
////__hpvm__hint(hpvm::DEVICE);
////__hpvm__hint(hpvm::GPU_TARGET);
// dummy_mem[c] = c;
//}
//__hetero_section_end(Section_Wrapped);
//__hetero_task_end(Wrapper);
//__hetero_section_end(Section);
//__hetero_launch_end(DFG);
//}

void StateHelper::EKFUpdate(State* state, const std::vector<Type*>& H_order, const Eigen::MatrixXd& H,
    const Eigen::VectorXd& res, const Eigen::MatrixXd& R)
{

    //==========================================================
    //==========================================================
    // Part of the Kalman Gain K = (P*H^T)*S^{-1} = M*S^{-1}
    assert(res.rows() == R.rows());
    assert(H.rows() == res.rows());
    Eigen::MatrixXd M_a = Eigen::MatrixXd::Zero(state->_Cov.rows(), res.rows());

    // Get the location in small jacobian for each measuring variable
    int current_it = 0;
    std::vector<int> H_id;
    for (Type* meas_var : H_order) {
        H_id.push_back(current_it);
        current_it += meas_var->size();
    }

    void* DFG = __hetero_launch_begin(
        6,
        state, sizeof(*state),
        &res, sizeof(res),
        &H_order, sizeof(H_order),
        &H_id, sizeof(H_id),
        &H, sizeof(H),
        &M_a, sizeof(M_a),
        1, &M_a, sizeof(M_a));

    void* Section = __hetero_section_begin();
    void* Wrapper = __hetero_task_begin(
        6,
        state, sizeof(*state),
        &res, sizeof(res),
        &H_order, sizeof(H_order),
        &H_id, sizeof(H_id),
        &H, sizeof(H),
        &M_a, sizeof(M_a),
        1, &M_a, sizeof(M_a));

    Type* var = state->_variables[0];
    Eigen::MatrixXd M_i = Eigen::MatrixXd::Zero(var->size(), res.rows());
    for (size_t i = 0; i < 10; i++) {
        Type* meas_var = H_order[i];
         M_i.noalias() += state->_Cov.block(var->id(), meas_var->id(), var->size(), meas_var->size()) * H.block(0, H_id[i], H.rows(), meas_var->size()).transpose();
    }

    __hetero_task_end(Wrapper);
    __hetero_section_end(Section);
    __hetero_launch_end(DFG);
    return;

    /*
    //==========================================================
    //==========================================================
    // For each active variable find its M = P*H^T
    for (Type* var : state->_variables) {
        // Sum up effect of each subjacobian = K_i= \sum_m (P_im Hm^T)
        Eigen::MatrixXd M_i = Eigen::MatrixXd::Zero(var->size(), res.rows());
        for (size_t i = 0; i < H_order.size(); i++) {
            Type* meas_var = H_order[i];
            M_i.noalias() += state->_Cov.block(var->id(), meas_var->id(), var->size(), meas_var->size()) * H.block(0, H_id[i], H.rows(), meas_var->size()).transpose();
        }
        M_a.block(var->id(), 0, var->size(), res.rows()) = M_i;
    }

    //==========================================================
    //==========================================================
    // Get covariance of the involved terms
    Eigen::MatrixXd P_small = StateHelper::get_marginal_covariance(state, H_order);

    // Residual covariance S = H*Cov*H' + R
    Eigen::MatrixXd S(R.rows(), R.rows());
    S.triangularView<Eigen::Upper>() = H * P_small * H.transpose();
    S.triangularView<Eigen::Upper>() += R;
    // Eigen::MatrixXd S = H * P_small * H.transpose() + R;

    // Invert our S (should we use a more stable method here??)
    Eigen::MatrixXd Sinv = Eigen::MatrixXd::Identity(R.rows(), R.rows());
    S.selfadjointView<Eigen::Upper>().llt().solveInPlace(Sinv);
    Eigen::MatrixXd K = M_a * Sinv.selfadjointView<Eigen::Upper>();
    // Eigen::MatrixXd K = M_a * S.inverse();

    // Update Covariance
    state->_Cov.triangularView<Eigen::Upper>() -= K * M_a.transpose();
    state->_Cov = state->_Cov.selfadjointView<Eigen::Upper>();
    // Cov -= K * M_a.transpose();
    // Cov = 0.5*(Cov+Cov.transpose());

    // We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d)
    Eigen::VectorXd diags = state->_Cov.diagonal();
    bool found_neg = false;
    for (int i = 0; i < diags.rows(); i++) {
        if (diags(i) < 0.0) {
#ifndef NDEBUG
            printf(RED "StateHelper::EKFUpdate() - diagonal at %d is %.2f\n" RESET, i, diags(i));
#endif
            found_neg = true;
        }
    }
    assert(!found_neg);

    // Calculate our delta and update all our active states
    Eigen::VectorXd dx = K * res;
    for (size_t i = 0; i < state->_variables.size(); i++) {
        state->_variables.at(i)->update(dx.block(state->_variables.at(i)->id(), 0, state->_variables.at(i)->size(), 1));
    }
    */
}
