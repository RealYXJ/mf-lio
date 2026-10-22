#pragma once
// Voxel-hash plane map: O(1) amortized lookup of a world-frame plane near a query point.
// Populated incrementally from mid-tier's converged point-to-plane correspondences
// (accumulates across many mid-tier cycles, unlike the single-cycle plane_cache_).
// Consumed by the fast tier every ~10ms cycle against FRESH points, so the fast tier
// can incorporate new LiDAR returns without paying a full ikd-tree search per cycle.

#include <cstdint>
#include <cmath>
#include <unordered_map>
#include <Eigen/Dense>

struct VoxelKey {
    int32_t x, y, z;
    bool operator==(const VoxelKey& o) const {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const {
        size_t h1 = std::hash<int32_t>()(k.x);
        size_t h2 = std::hash<int32_t>()(k.y);
        size_t h3 = std::hash<int32_t>()(k.z);
        return h1 ^ (h2 * 0x9e3779b97f4a7c15ULL) ^ (h3 * 0xc2b2ae3d27d4eb4fULL);
    }
};

struct VoxelPlaneEntry {
    Eigen::Vector3d normal;  // unit plane normal, world frame
    double          offset; // plane offset d: n·p + d ≈ 0 for points on the plane
    int             hit_count = 0;
};

// Update policy: overwrite-on-insert (most recent correspondence per cell wins).
// Simple and cheap; a cell is only ever written by mid-tier's own converged,
// quality-gated correspondences, so "most recent" is already a good plane fit.
class VoxelPlaneHash {
public:
    explicit VoxelPlaneHash(double cell_size = 0.2) : cell_size_(cell_size) {}

    void set_cell_size(double cell_size) { cell_size_ = cell_size; }

    void insert_or_update(const Eigen::Vector3d& world_point,
                           const Eigen::Vector3d& normal,
                           double offset) {
        const VoxelKey k = key_of(world_point);
        auto it = map_.find(k);
        if (it == map_.end()) {
            map_.emplace(k, VoxelPlaneEntry{normal, offset, 1});
        } else {
            it->second.normal = normal;
            it->second.offset = offset;
            ++it->second.hit_count;
        }
    }

    const VoxelPlaneEntry* query(const Eigen::Vector3d& world_point) const {
        auto it = map_.find(key_of(world_point));
        return (it == map_.end()) ? nullptr : &it->second;
    }

    size_t size() const { return map_.size(); }
    void clear() { map_.clear(); }

private:
    VoxelKey key_of(const Eigen::Vector3d& p) const {
        return VoxelKey{
            static_cast<int32_t>(std::floor(p.x() / cell_size_)),
            static_cast<int32_t>(std::floor(p.y() / cell_size_)),
            static_cast<int32_t>(std::floor(p.z() / cell_size_))
        };
    }

    double cell_size_;
    std::unordered_map<VoxelKey, VoxelPlaneEntry, VoxelKeyHash> map_;
};
