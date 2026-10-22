#pragma once
#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

struct ExtractedFeatures {
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr edge_points;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr plane_points;
    // 0.0 = no edge points found (flat-ground degeneracy).
    // >0  = min λ₁ across edge-point neighborhoods (geometry strength).
    double D_degenerate = 0.0;
};

class FeatureExtractor {
public:
    float edge_thresh  = 0.65f;   // edge score  threshold (λ₃ / Σλ)
    float plane_thresh = 0.75f;   // planarity   threshold (1 - λ₁/Σλ)
    int   k_neighbors  = 5;

    ExtractedFeatures extract(
        const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& scan) const
    {
        ExtractedFeatures out;
        out.edge_points  = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
        out.plane_points = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();

        if (!scan || scan->empty()) return out;

        pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree;
        kdtree.setInputCloud(scan);

        double min_D = 1e9;

        for (const auto& pt : scan->points) {
            std::vector<int>   idx(k_neighbors);
            std::vector<float> dist(k_neighbors);
            if (kdtree.nearestKSearch(pt, k_neighbors, idx, dist) < k_neighbors)
                continue;

            // Compute covariance of k neighbors
            Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
            Eigen::Vector3f mean = Eigen::Vector3f::Zero();
            for (int i : idx)
                mean += scan->points[i].getVector3fMap();
            mean /= static_cast<float>(k_neighbors);

            for (int i : idx) {
                Eigen::Vector3f d = scan->points[i].getVector3fMap() - mean;
                cov += d * d.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
            auto ev = es.eigenvalues();  // sorted ascending λ₁ ≤ λ₂ ≤ λ₃
            float sum = ev.sum();
            if (sum < 1e-10f) continue;

            float edge_score  = ev(2) / sum;
            float planarity   = 1.0f - ev(0) / sum;

            if (edge_score > edge_thresh) {
                out.edge_points->push_back(pt);
                if (ev(0) < min_D) min_D = ev(0);
            } else if (planarity > plane_thresh) {
                out.plane_points->push_back(pt);
            }
        }

        out.D_degenerate = (out.edge_points->empty()) ? 0.0 : min_D;
        return out;
    }
};
