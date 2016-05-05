
#ifndef SURFMORPH_H
#define SURFMORPH_H

#include "surfmorph_codegen.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/SVD>

#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace surfmorph
{

//!
//! \brief Hash for pairs.
//!
template <typename T, typename S>
struct pairhash
{
    size_t operator()(const std::pair<T, S> &p) const
    {
        // From boost::hash_combine
        auto hash1 = std::hash<T>()(p.first);
        auto hash2 = std::hash<S>()(p.second);
        hash1 ^= hash2 + 0x9e3779b9 + (hash1 << 6) + (hash1 >> 2);
        return hash1;
    }
};

// clang-format off
//!
//! \brief Surface morphing computation.
//!
//! This class template implements
//! <a href="http://link.springer.com/article/10.1007/s11390-011-1154-3">as-rigid-as-possible surface morphing</a>
//! and provides methods to include multiple poses and retrieve the interpolated
//! pose at any given point.
//!
//! Every pose is associated with an increasing integer time position, starting
//! with 0. Interpolations can be requested for any floating point value in the
//! range [0, #poses-1].
//!
//! \tparam ScalarT Scalar type for the coordinates
//!
//! \see <a href="http://link.springer.com/article/10.1007/s11390-011-1154-3">As-rigid-as-possible surface morphing</a>
//!
//! \note This class uses OpenMP to parallelize some of its operations. This can
//!       can be disabled by defining the macro SURFMORPH_DONT_PARALLELIZE on
//!       compile time (note that this does not disable parallelization on
//!       Eigen). Animation interpolation is not parallelized by default, since
//!       it is generally more efficient to parallelize the generation of each
//!       frame; however, parallelization on interpolation can be enabled by
//!       defining the macro SURFMORPH_PARALLELIZE_INTERPOLATION on compile
//!       time.
//!
//! \author Javier Dehesa (javidcf@gmail.com)
//!
// clang-format on
template <typename ScalarT>
class SurfaceMorph
{
public:
    //! Scalar type for the coordinates
    typedef ScalarT Scalar;

private:
    //! Vertex index
    typedef unsigned int Idx;
    //! Matrix of tetrahedra indices
    typedef Eigen::Matrix<Idx, 4, Eigen::Dynamic> TetraMat;
    //! Matrix of tetrahedra pose coordinates
    typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> PoseMat;
    //! Matrix of triangles pose coordinates
    typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> CoordsMat;
    //! Transformation matrix
    typedef Eigen::Matrix<Scalar, 3, 3> TransfromMat;
    //! 3D vector
    typedef Eigen::Matrix<Scalar, 3, 1> Vec3;
    //! Quaternion
    typedef Eigen::Quaternion<Scalar> Quaternion;
    //! Matrix of irrotations (3x3 matrices stored by cols)
    typedef Eigen::Matrix<Scalar, 9, Eigen::Dynamic> IrrotationsMat;
    //! Matrix of quaternions
    typedef Eigen::Matrix<Scalar, 4, Eigen::Dynamic> QuaternionsMat;
    //! Matrix of translations
    typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> TranslationsMat;
    //! Matrix of U inverse matrices (3x3 matrices stored by cols)
    typedef Eigen::Matrix<Scalar, 9, Eigen::Dynamic> UInvsMat;
    //! Matrix of triangle areas
    typedef Eigen::Matrix<Scalar, 1, Eigen::Dynamic> AreasMat;
    //! Coefficients matrix
    typedef Eigen::SparseMatrix<Scalar> CoeffsMat;
    //! Coefficients matrix
    typedef Eigen::Triplet<Scalar> Triplet;

public:
    //!
    //! \brief Constructor.
    //!
    //! Creates a surface morpher for a mesh structure. The mesh structure is
    //! given by the triangles of the mesh, indicated as columns of vertex
    //! indices. Vertex indices must be numbered with sequential integers
    //! starting with 0, and every vertex must belong to at least one triangle.
    //! The vertex index is used to determine the coordinates column for a
    //! vertex in a pose matrix.
    //!
    //! \tparam TrianglesMatT Type of the triangles matrix
    //!
    //! \param triangles A matrix representing the triangles of the mesh,
    //!                  expressed as columns of vertex indices
    //!
    template <typename TrianglesMatT>
    explicit SurfaceMorph(const TrianglesMatT &triangles)
    : m_numVertices{triangles.maxCoeff() + 1}
    , m_tetrahedra{SurfaceMorph::createTetrahedra(triangles)}
    , m_poses()
    , m_irrotations()
    , m_quaternions()
    , m_translations()
    , m_uInvs()
    {
#ifndef SURFMORPH_DONT_PARALLELIZE
        Eigen::initParallel();
#else
#ifdef SURFMORPH_PARALLELIZE_INTERPOLATION
        Eigen::initParallel();
#endif
#endif
    }

    //!
    //! \brief Constructor.
    //!
    //! Creates a surface morpher for a mesh structure. The mesh structure is
    //! given by the triangles of the mesh, indicated as columns of vertex
    //! indices. Vertex indices must be numbered with sequential integers
    //! starting with 0, and every vertex must belong to at least one triangle.
    //! The vertex index is used to determine the coordinates column for a
    //! vertex in a pose matrix.
    //!
    //! \tparam TrianglesMatT Type of the triangles matrix
    //! \tparam CoordsMatT Type of the coordinates matrix
    //!
    //! \param triangles A matrix representing the triangles of the mesh,
    //!                  expressed as columns of vertex indices
    //! \param intialPose Matrices representing the surface initial poses as
    //!                   columns of vertex absolute coordinates
    //!
    template <typename TrianglesMatT, typename CoordsMatT>
    SurfaceMorph(const TrianglesMatT &triangles, const CoordsMatT &initialPose)
    : m_numVertices{triangles.maxCoeff() + 1}
    , m_tetrahedra{SurfaceMorph::createTetrahedra(triangles)}
    , m_poses()
    , m_irrotations()
    , m_quaternions()
    , m_translations()
    , m_uInvs()
    {
#ifndef SURFMORPH_DONT_PARALLELIZE
        Eigen::initParallel();
#else
#ifdef SURFMORPH_PARALLELIZE_INTERPOLATION
        Eigen::initParallel();
#endif
#endif
        addPose(initialPose);
    }

    //!
    //! \brief Add a new surface pose.
    //!
    //! \tparam CoordsMatT Type of the coordinates matrix
    //!
    //! \param pose The new surface pose
    //!
    template <typename CoordsMatT>
    void addPose(const CoordsMatT &pose)
    {
        if (pose.cols() != m_numVertices)
        {
            std::runtime_error("Incorrect number of coordinates in pose");
        }
        PoseMat newPose(3, m_numVertices + m_tetrahedra.cols());
        newPose.leftCols(m_numVertices) = pose.template cast<Scalar>();

#ifndef SURFMORPH_DONT_PARALLELIZE
        Eigen::setNbThreads(1);
#pragma omp parallel for
#endif
        // Compute tetrahedra extra vertices positions
        for (Idx iTetra = 0; iTetra < m_tetrahedra.cols(); iTetra++) {
            Idx p0Idx{m_tetrahedra(0, iTetra)};
            Idx p1Idx{m_tetrahedra(1, iTetra)};
            Idx p2Idx{m_tetrahedra(2, iTetra)};
            Idx p3Idx{m_tetrahedra(3, iTetra)};

            auto p0 = newPose.col(p0Idx);
            auto p1 = newPose.col(p1Idx);
            auto p2 = newPose.col(p2Idx);

            auto v = (p1 - p0).cross(p2 - p1);
            auto vNorm = static_cast<Scalar>(std::sqrt(v.norm()));
            auto p3 = ((p0 + p1 + p2) / 3.0) + v / vNorm;
            newPose.col(p3Idx) = p3;
        }
#ifndef SURFMORPH_DONT_PARALLELIZE
        Eigen::setNbThreads(0);
#endif

        addTetraPose(std::move(newPose));
    }

    //!
    //! \brief Get the number of poses.
    //!
    //! \return The number of poses
    //!
    size_t numPoses() const
    {
        return m_poses.size();
    }

    //!
    //! \brief Interpolate the pose of the mesh at a given point in time.
    //!
    //! A time point represents a moment in the morphing process. Each pose is
    //! assigned a sequentially increasing integer time point, starting with 0.
    //! The valid range of time points is therefore [0, #poses - 1]
    //!
    //! \param t Time point to interpolate
    //! \return The absolute vertex coordinates at the given time point
    //!
    CoordsMat interpolatePoseAt(Scalar t) const
    {
        if (m_poses.size() < 1)
        {
            throw std::runtime_error("There are no poses to interpolate");
        }

        if (t < 0 || t > m_poses.size() - 1)
        {
            throw std::runtime_error("Time point value out of range");
        }

        auto poseIdx = static_cast<unsigned int>(t);
        const auto &pose = m_poses[poseIdx];
        const auto &poseIrrotations = m_irrotations[poseIdx];
        const auto &poseQuaternions = m_quaternions[poseIdx];
        const auto &poseTranslations = m_translations[poseIdx];
        const auto &poseUInvs = m_uInvs[poseIdx];
        const auto &poseStartAreas = m_startAreas[poseIdx];
        const auto &poseEndAreas = m_endAreas[poseIdx];
        const auto poseAlpha = m_alphas[poseIdx];

        // If it is exactly a pose use it and finish
        if (Scalar(poseIdx) == t)
        {
            return pose.leftCols(m_numVertices);
        }

        // Prepare system of equations
        auto numEquations = 3 * (m_numVertices + m_tetrahedra.cols());
        std::unordered_map<std::pair<Idx, Idx>, Scalar, pairhash<Idx, Idx>>
            coeffsMap;
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> indeps(numEquations, 1);
        indeps.setZero();

#ifdef SURFMORPH_PARALLELIZE_INTERPOLATION
        Eigen::setNbThreads(1);
// Not a parallel for - just start the thread pool
#pragma omp parallel
#endif
        // Update the system with the interpolation for each tetrahedron
        for (Idx iTetra = 0; iTetra < m_tetrahedra.cols(); iTetra++) {
            // Get tetrahedron points
            Idx p0Idx = m_tetrahedra(0, iTetra);
            Idx p1Idx = m_tetrahedra(1, iTetra);
            Idx p2Idx = m_tetrahedra(2, iTetra);
            Idx p3Idx = m_tetrahedra(3, iTetra);
            Vec3 p0 = pose.col(p0Idx);
            Vec3 p1 = pose.col(p1Idx);
            Vec3 p2 = pose.col(p2Idx);
            Vec3 p3 = pose.col(p3Idx);

            // Interpolate transformation
            Scalar tRel{t - poseIdx};

            // Triangle area
            Scalar startArea = poseStartAreas(0, iTetra);
            Scalar endArea = poseEndAreas(0, iTetra);
            Scalar area = startArea * (Scalar(1) - tRel) + endArea * t;

            // Irrotation
            auto irrotVec = poseIrrotations.col(iTetra);
            TransfromMat irrotation;
            // clang-format off
            irrotation << irrotVec[0], irrotVec[1], irrotVec[2],
                          irrotVec[3], irrotVec[4], irrotVec[6],
                          irrotVec[6], irrotVec[7], irrotVec[8];
            // clang-format on
            TransfromMat irrotationInterp =
                TransfromMat::Identity() * (Scalar(1) - tRel) +
                irrotation * tRel;
            // Rotation
            Quaternion quaternion(poseQuaternions.col(iTetra));
            Quaternion quaternionInterp =
                Quaternion::Identity().slerp(tRel, quaternion);
            TransfromMat rotationInterp = quaternionInterp.matrix();
            // Resulting non-translational transform
            TransfromMat nontranslationInterp =
                rotationInterp * irrotationInterp;
            // Translation
            Vec3 translationInterp = poseTranslations.col(iTetra) * tRel;

            // Get uInv matrix
            auto uInvVec = poseUInvs.col(iTetra);
            TransfromMat uInv;
            // clang-format off
            uInv << uInvVec[0], uInvVec[1], uInvVec[2],
                    uInvVec[3], uInvVec[4], uInvVec[5],
                    uInvVec[6], uInvVec[7], uInvVec[8];
            // clang-format on

            // Update equations
            surfmorph::updateInterpolationSystem(
                p0, p1, p2, p3, p0Idx, p1Idx, p2Idx, p3Idx, uInv,
                nontranslationInterp, translationInterp, area, poseAlpha,
                coeffsMap, indeps);
        }
#ifdef SURFMORPH_PARALLELIZE_INTERPOLATION
        Eigen::setNbThreads(0);
#endif

        std::vector<Eigen::Triplet<Scalar>> coeffsTriplets;
        coeffsTriplets.reserve(coeffsMap.size());
        for (const auto &entry : coeffsMap) {
            coeffsTriplets.push_back(Triplet(std::get<0>(entry.first),
                                             std::get<1>(entry.first),
                                             entry.second));
        }

        CoeffsMat coeffsMat(numEquations, numEquations);
        coeffsMat.setFromTriplets(coeffsTriplets.begin(), coeffsTriplets.end());
        Eigen::SimplicialLDLT<CoeffsMat> ldlt(coeffsMat);
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> poseInterp =
            ldlt.solve(indeps);

        CoordsMat coordsInterp(3, m_numVertices);
        for (Idx iVertex = 0; iVertex < m_numVertices; iVertex++) {
            coordsInterp.col(iVertex) << poseInterp(3 * iVertex + 0),
                poseInterp(3 * iVertex + 1), poseInterp(3 * iVertex + 2);
        }
        return coordsInterp;
    }

private:
    //! Number of vertices in the mesh
    const Idx m_numVertices;
    //! Tetrahedra indices
    const TetraMat m_tetrahedra;
    //! Mesh poses
    std::vector<PoseMat> m_poses;
    //! Morphing irrotations
    std::vector<IrrotationsMat> m_irrotations;
    //! Morphing quaternions
    std::vector<QuaternionsMat> m_quaternions;
    //! Morphing translations
    std::vector<TranslationsMat> m_translations;
    //! U inverse matrices
    std::vector<UInvsMat> m_uInvs;
    //! Triangle areas at start
    std::vector<AreasMat> m_startAreas;
    //! Triangle areas at end
    std::vector<AreasMat> m_endAreas;
    //! Alpha coefficients
    std::vector<Scalar> m_alphas;

    //!
    //! \brief Create a tetrahedra structure matrix.
    //!
    //! \tparam TrianglesMatT Type of the triangles matrix
    //!
    //! \param triangles Triangles matrix
    //!
    template <typename TrianglesMatT>
    static TetraMat createTetrahedra(const TrianglesMatT &triangles)
    {
        Idx numVertices = triangles.maxCoeff() + 1;
        TetraMat tetrahedra(4, triangles.cols());
        tetrahedra.topRows(3) = triangles.template cast<Idx>();
        for (Idx iTetra = 0; iTetra < tetrahedra.cols(); iTetra++) {
            tetrahedra(3, iTetra) = iTetra + numVertices;
        }
        return tetrahedra;
    }

    //!
    //! \brief Add a new tetrahedra pose
    //!
    //! \param newPose The new tetrahedra pose to add
    //!
    void addTetraPose(PoseMat newPose)
    {
        if (m_poses.size() > 0)
        {
            // Compute morphing transformations
            const auto &prevPose = m_poses.back();
            IrrotationsMat irrotations(9, m_tetrahedra.cols());
            QuaternionsMat quaternions(4, m_tetrahedra.cols());
            TranslationsMat translations(3, m_tetrahedra.cols());
            UInvsMat uInvs(9, m_tetrahedra.cols());
            AreasMat startAreas(1, m_tetrahedra.cols());
            AreasMat endAreas(1, m_tetrahedra.cols());

#ifndef SURFMORPH_DONT_PARALLELIZE
            Eigen::setNbThreads(1);
#pragma omp parallel for
#endif
            for (Idx iTetra = 0; iTetra < m_tetrahedra.cols(); iTetra++) {
                Idx p0Idx{m_tetrahedra(0, iTetra)};
                Idx p1Idx{m_tetrahedra(1, iTetra)};
                Idx p2Idx{m_tetrahedra(2, iTetra)};
                Idx p3Idx{m_tetrahedra(3, iTetra)};

                auto u0 = prevPose.col(p0Idx);
                auto u1 = prevPose.col(p1Idx);
                auto u2 = prevPose.col(p2Idx);
                auto u3 = prevPose.col(p3Idx);

                auto v0 = newPose.col(p0Idx);
                auto v1 = newPose.col(p1Idx);
                auto v2 = newPose.col(p2Idx);
                auto v3 = newPose.col(p3Idx);

                // Find triangle areas
                Scalar startArea = ((u1 - u0).cross(u2 - u1)).norm() / Scalar(2);
                Scalar endArea = ((v1 - v0).cross(v2 - v1)).norm() / Scalar(2);

                // Compute transformation between poses
                TransfromMat u;
                u.col(0) = u0 - u3;
                u.col(1) = u1 - u3;
                u.col(2) = u2 - u3;
                TransfromMat uInv{u.inverse()};
                TransfromMat v;
                v.col(0) = v0 - v3;
                v.col(1) = v1 - v3;
                v.col(2) = v2 - v3;
                TransfromMat m{v * uInv};
                Vec3 t{v0 - m * u0};
                auto mSvd =
                    m.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
                TransfromMat r{mSvd.matrixU() * mSvd.matrixV().adjoint()};
                TransfromMat s{mSvd.matrixV() *
                               mSvd.singularValues().asDiagonal() *
                               mSvd.matrixV().adjoint()};
                Quaternion q{r};

                // Save pose transformation information
                // clang-format off
                irrotations.col(iTetra) << s(0, 0), s(0, 1), s(0, 2),
                                           s(1, 0), s(1, 1), s(1, 2),
                                           s(2, 0), s(2, 1), s(2, 2);
                quaternions.col(iTetra) << q.x(), q.y(), q.z(), q.w();
                translations.col(iTetra) = t;
                uInvs.col(iTetra) << uInv(0, 0), uInv(0, 1), uInv(0, 2),
                                     uInv(1, 0), uInv(1, 1), uInv(1, 2),
                                     uInv(2, 0), uInv(2, 1), uInv(2, 2);
                startAreas.col(iTetra) << startArea;
                endAreas.col(iTetra) << endArea;
                // clang-format on
            }
#ifndef SURFMORPH_DONT_PARALLELIZE
            Eigen::setNbThreads(0);
#endif
            m_irrotations.push_back(std::move(irrotations));
            m_quaternions.push_back(std::move(quaternions));
            m_translations.push_back(std::move(translations));
            m_uInvs.push_back(std::move(uInvs));
            m_startAreas.push_back(std::move(startAreas));
            m_endAreas.push_back(std::move(endAreas));

            // Compute alpha value from bounding box of previous pose
            Vec3 mins;
            mins << prevPose.row(0).minCoeff(), prevPose.row(1).minCoeff(),
                prevPose.row(2).minCoeff();
            Vec3 maxs;
            maxs << prevPose.row(0).maxCoeff(), prevPose.row(1).maxCoeff(),
                prevPose.row(2).maxCoeff();
            Scalar alpha{Scalar(1) / (maxs - mins).squaredNorm()};
            m_alphas.push_back(alpha);
        }

        m_poses.push_back(std::move(newPose));
    }
};
}

#endif
