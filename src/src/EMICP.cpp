// Copyright 2017 Steven Parkison

#ifndef SEMANTIC_ICP_IMPL_EM_ICP_HPP_
#define SEMANTIC_ICP_IMPL_EM_ICP_HPP_

#include <iostream>
#include <glog/logging.h>

#include <ceres/ceres.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "gicp_cost_functor_autodiff.h"
#include "gicp_cost_function.h"
// #include "local_parameterization_se3.h"

#include "sqloss.h"
#include "EMICP.h"
#include <Eigen/StdVector>
#include <ceres/gradient_checker.h>

#define N 11

using namespace SemanticICP;

//   template <size_t N>
void EmIterativeClosestPoint::ComputeCovariances(const PointCloudPtr cloudptr,
                                                 KdTreePtr treeptr,
                                                 MatricesVectorPtr matvecptr,
                                                 DistVectorPtr distvecptr)
{
    LOG(INFO) << "ComputeCovariances....";
    // // Variables for computing Covariances
    Eigen::Vector3d mean;
    Eigen::Matrix<double, N, 1> dist;
    double increment = 1.0 / static_cast<double>(kCorrespondences_);

    std::vector<int> nn_idecies;
    nn_idecies.reserve(kCorrespondences_); // set capacity
    std::vector<float> nn_dist_sq;
    nn_dist_sq.reserve(kCorrespondences_);

    int si = cloudptr->size();
    LOG(INFO) << "point cloud size: " << si;
    // Set up Itteration
    matvecptr->resize(si);
    distvecptr->resize(si);

    for (int itter = 0; itter < si; itter++)
    {
        const PointT &query_pt = (*cloudptr)[itter];

        Eigen::Matrix3d cov;
        cov.setZero();
        mean.setZero();
        dist.setZero();

        treeptr->nearestKSearch(query_pt, kCorrespondences_, nn_idecies, nn_dist_sq); //  point idx , idx dst

        for (int index : nn_idecies)
        {
            const PointT &pt = (*cloudptr)[index];

            dist(pt.label - 1, 0) += increment;

            mean[0] += pt.x;
            mean[1] += pt.y;
            mean[2] += pt.z;
            // LOG(INFO) << "===   "<<index<<" " << mean[0];

            cov(0, 0) += pt.x * pt.x;

            cov(1, 0) += pt.y * pt.x;
            cov(1, 1) += pt.y * pt.y;

            cov(2, 0) += pt.z * pt.x;
            cov(2, 1) += pt.z * pt.y;
            cov(2, 2) += pt.z * pt.z;
        }
        // calculate mean  covariance
        mean /= static_cast<double>(kCorrespondences_);
        for (int k = 0; k < 3; k++)
        {
            for (int l = 0; l <= k; l++)
            {
                cov(k, l) /= static_cast<double>(kCorrespondences_);
                cov(k, l) -= mean[k] * mean[l];
                cov(l, k) = cov(k, l);
            }
        }
        // LOG(INFO) << "mean mat: \n"
        //           << mean << "\n\n";
        // LOG(INFO) << "cov mat:  \n"
        //           << cov << "\n\n";

        // SVD decomposition for PCA
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU);
        cov.setZero();
        Eigen::Matrix3d U = svd.matrixU();

        for (int k = 0; k < 3; k++)
        {
            Eigen::Vector3d col = U.col(k);
            double v = 1.;
            if (k == 2)
            {
                v = kEpsilon_;
            }
            cov += v * col * col.transpose();
        }
        (*matvecptr)[itter] = cov;
        (*distvecptr)[itter] = dist;
    }
    LOG(INFO) << "cal done";
}

void EmIterativeClosestPoint::align(PointCloudPtr final_cloud, const Sophus::SE3d &init_transform)
{

    ComputeCovariances(source_cloud_, source_kd_tree_, source_covariances_, source_distributions_);
    ComputeCovariances(target_cloud_, target_kd_tree_, target_covariances_, target_distributions_);
    Sophus::SE3d current_transform = init_transform;
    bool converged = false;
    size_t outter_itter = 0;

    while (converged != true)
    {
        // Build The Problem
        ceres::Problem problem;
        // Add Sophus SE3 Parameter block with local parametrization
        Sophus::SE3d est_transform = current_transform;
        // problem.AddParameterBlock(est_transform.data(), Sophus::SE3d::num_parameters,
        //                           new SemanticICP::LocalParameterizationSE3);
        // problem.AddParameterBlock( 11); // hy
        double mse_high = 0;

        typename pcl::PointCloud<PointT>::Ptr transformed_source(new pcl::PointCloud<PointT>());
        Eigen::Matrix4d trans_mat = current_transform.matrix();
        pcl::transformPointCloud(*source_cloud_,
                                 *transformed_source,
                                 trans_mat);

        std::vector<int> target_index;
        std::vector<float> dist_sq;

        LOG(INFO) << "Num Points: " << transformed_source->size();
        for (int source_index = 0; source_index != transformed_source->size(); source_index++)
        {
            const PointT &transformed_source_pt = transformed_source->points[source_index];
            target_kd_tree_->nearestKSearch(transformed_source_pt, 4, target_index, dist_sq);

            for (int correspondence_index = 0; correspondence_index < 4; correspondence_index++)
            {
                if (dist_sq[correspondence_index] < 10)
                {
                    const PointT &source_pt = source_cloud_->points[source_index];
                    const pcl::PointXYZ s_pt(source_pt.x, source_pt.y, source_pt.z);
                    const Eigen::Matrix3d &source_cov = source_covariances_->at(source_index);

                    const PointT &target_pt = target_cloud_->points[target_index[correspondence_index]];
                    const pcl::PointXYZ t_pt(target_pt.x, target_pt.y, target_pt.z);
                    const Eigen::Matrix3d &target_cov = target_covariances_->at(target_index[correspondence_index]);

                    const Eigen::Matrix<double, N, 1> target_dist = target_distributions_->at(target_index[correspondence_index]);
                    const Eigen::Matrix<double, N, 1> source_dist = source_distributions_->at(source_index);

                    // double prob = confusion_matrix_(source_pt.label-1, target_pt.label-1)*
                    //              dist(target_pt.label-1, 0);
                    double prob = 0;
                    for (size_t s = 0; s < N; s++)
                    {
                        double temp = target_dist.transpose() * confusion_matrix_.col(s);
                        temp *= source_dist.transpose() * confusion_matrix_.col(s);
                        prob += temp;
                    }

                    //           //   Autodif Cost function
                    //           //GICPCostFunctorAutoDiff *c= new GICPCostFunctorAutoDiff(s_pt,
                    //           //                                                       t_pt,
                    //           //                                                       source_cov,
                    //           //                                                       target_cov,
                    //           //                                                       base_transformation_);
                    //           //ceres::CostFunction* cost_function =
                    //           //    new ceres::AutoDiffCostFunction<GICPCostFunctorAutoDiff,
                    //           //                                    1,
                    //           //                                    Sophus::SE3d::num_parameters>(c);

                    //   Analytical Cost Function
                    GICPCostFunction *cost_function = new GICPCostFunction(s_pt,
                                                                           t_pt,
                                                                           source_cov,
                                                                           target_cov,
                                                                           base_transformation_);
                    prob *= cost_function->Probability(est_transform);
                    problem.AddResidualBlock(cost_function,
                                             new ceres::ComposedLoss(
                                                 new ceres::ScaledLoss(new ceres::CauchyLoss(3.0),
                                                                       prob,
                                                                       ceres::TAKE_OWNERSHIP),
                                                 ceres::TAKE_OWNERSHIP,
                                                 new SQLoss(),
                                                 ceres::TAKE_OWNERSHIP),
                                             est_transform.data());

                    // Gradient Check
                    if (false)
                    {
                        LOG(INFO) << "Gradient Check:\n";
                        ceres::NumericDiffOptions numeric_diff_options;
                        numeric_diff_options.relative_step_size = 1e-13;

                        // std::vector<const ceres::LocalParameterization *> lp;
                        // lp.push_back(new LocalParameterizationSE3);

                        GICPCostFunction *cost_test = new GICPCostFunction(s_pt,
                                                                           t_pt,
                                                                           source_cov,
                                                                           target_cov,
                                                                           base_transformation_);
                        // ceres::GradientChecker gradient_checker(cost_test,
                        //                                         &lp,
                        //                                         numeric_diff_options);

                        ceres::GradientChecker::ProbeResults *results = new ceres::GradientChecker::ProbeResults();
                        std::vector<double *> params;
                        Sophus::SE3d test_transform = est_transform;
                        params.push_back(test_transform.data());
                        LOG(INFO) << "Doing Check:\n";
                        // if (!gradient_checker.Probe(params.data(), 5e-12, results))
                        // {
                        //     LOG(INFO) << "An error has occurred:\n";
                        //     LOG(INFO) << results->error_log;
                        //     LOG(INFO) << results->jacobians[0] << std::endl;
                        //     LOG(INFO) << results->numeric_jacobians[0] << std::endl;
                        //     LOG(INFO) << test_transform.matrix() << std::endl;
                        //     LOG(INFO) << source_pt << std::endl;
                        //     LOG(INFO) << target_pt << std::endl;
                        //     LOG(INFO) << source_cov << std::endl;
                        //     LOG(INFO) << target_cov << std::endl;
                        // } // Gradient Relitive error
                    } // Gradient Check
                }     // If close enough
            }         // For loop over correspondences
        }             // For loop over points

        // Sovler Options
        LOG(INFO) << "Solve... ... \n";
        LOG(INFO) << "Number of Residual Blocks: " << problem.NumResidualBlocks() << std::endl;
        LOG(INFO) << "Size of Parameter BlocK: " << problem.ParameterBlockSize(est_transform.data()) << std::endl;
        LOG(INFO) << "Size of Parameter Local BlocK: " << problem.ParameterBlockLocalSize(est_transform.data()) << std::endl;
        ceres::Solver::Options options;
        options.gradient_tolerance = 0.1 * Sophus::Constants<double>::epsilon();
        options.function_tolerance = 0.1 * Sophus::Constants<double>::epsilon();
        options.linear_solver_type = ceres::DENSE_QR;
        options.num_threads = 8;
        //    options.num_linear_solver_threads = 8;
        //    options.num_threads = 8; //kx
        options.max_num_iterations = 400;
        // options.check_gradients = true;
        options.gradient_check_numeric_derivative_relative_step_size = 1e-8;
        options.gradient_check_relative_precision = 1e-6;

        // Solve
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        LOG(INFO) << summary.FullReport() << std::endl;

        double mse = (current_transform.inverse() * est_transform).log().squaredNorm();
        if (mse < 1e-5 || outter_itter > 50)
            converged = true;
        LOG(INFO) << "MSE: " << mse << std::endl;
        LOG(INFO) << "Transform: " << std::endl;
        LOG(INFO) << est_transform.matrix() << "\n"
                  << std::endl;
        LOG(INFO) << "Itteration: " << outter_itter << std::endl;
        current_transform = est_transform;
        outter_itter++;
    }

    final_transformation_ = current_transform;

    Sophus::SE3d trans = final_transformation_ * base_transformation_;
    Eigen::Matrix4f mat = (trans.matrix()).cast<float>();
    if (final_cloud != nullptr)
    {
        pcl::transformPointCloud(*source_cloud_,
                                 *final_cloud,
                                 mat);
        LOG(INFO) << "get align point cloud";
    }
    outer_iter = outter_itter;
}

void EmIterativeClosestPoint::getFusedLabels(PointCloudPtr labeledCloud,
                                                const Sophus::SE3d &transformation)
{

    typename pcl::PointCloud<PointT>::Ptr transformed_source(new pcl::PointCloud<PointT>());
    Eigen::Matrix4d trans_mat = transformation.matrix();
    pcl::transformPointCloud(*source_cloud_,
                             *transformed_source,
                             trans_mat);
    std::vector<int> target_index;
    std::vector<float> dist_sq;

    LOG(INFO) << "Num Points: " << transformed_source->size() << std::endl;
    for (int source_index = 0; source_index != transformed_source->size(); source_index++)
    {
        const PointT &transformed_source_pt = transformed_source->points[source_index];
        const PointT &source_pt =
            source_cloud_->points[source_index];

        target_kd_tree_->nearestKSearch(transformed_source_pt, 4,
                                        target_index, dist_sq);

        Eigen::Matrix<double, N, 1> sprob = Eigen::Matrix<double, N, 1>::Zero();
        for (int correspondence_index = 0;
             correspondence_index < 4;
             correspondence_index++)
        {
            if (dist_sq[correspondence_index] < 250)
            {
                const pcl::PointXYZ s_pt(source_pt.x, source_pt.y, source_pt.z);
                const Eigen::Matrix3d &source_cov =
                    source_covariances_->at(source_index);
                const PointT &target_pt =
                    target_cloud_->points[target_index[correspondence_index]];
                const pcl::PointXYZ t_pt(target_pt.x, target_pt.y, target_pt.z);
                const Eigen::Matrix3d &target_cov =
                    target_covariances_->at(target_index[correspondence_index]);

                const Eigen::Matrix<double, N, 1> target_dist =
                    target_distributions_->at(target_index[correspondence_index]);
                const Eigen::Matrix<double, N, 1> source_dist =
                    source_distributions_->at(source_index);

                GICPCostFunction *cost_function = new GICPCostFunction(s_pt,
                                                                       t_pt,
                                                                       source_cov,
                                                                       target_cov,
                                                                       base_transformation_);
                double prob = cost_function->Probability(transformation);
                for (size_t s = 0; s < N; s++)
                {
                    double temp = target_dist.transpose() * confusion_matrix_.col(s);
                    temp *= source_dist.transpose() * confusion_matrix_.col(s);
                    sprob(s, 0) += temp * prob;
                }
            }
        }
        double max_prob = 0;
        size_t max_s = 0;
        for (size_t s = 0; s < N; s++)
        {
            if (sprob(s, 0) > max_prob)
            {
                max_s = s;
                max_prob = sprob(s, 0);
            }
        }
        PointT labeled_pt = source_pt;
        labeled_pt.label = max_s + 1;
        labeledCloud->push_back(labeled_pt);
    }
}

// namespace SemanticICP
#endif // SEMANTIC_ICP_IMPL_EM_ICP_HPP_
