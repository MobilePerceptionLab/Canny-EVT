//
// Created by mpl on 23-4-3.
//

#ifndef CannyEVT_GENERICFUNCTOR_H
#define CannyEVT_GENERICFUNCTOR_H

#include <Eigen/Eigen>
#include <iostream>
#include "utility.h"

namespace CannyEVT
{
template<typename ScalarT, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct GenericFunctor {
    /** undocumented */
    typedef ScalarT Scalar;
    /** undocumented */
    enum {
        InputsAtCompileTime = NX, ValuesAtCompileTime = NY
    };
    /** undocumented */
    typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    /** undocumented */
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    /** undocumented */
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    /** undocumented */
    const int m_inputs;
    /** undocumented */
    int m_values;

    /** undocumented */
    GenericFunctor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}

    /** undocumented */
    GenericFunctor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

    /** undocumented */
    int inputs() const { return m_inputs; }

    /** undocumented */
    int values() const { return m_values; }

    void resetNumberValues(int values) { m_values = values; }
};

struct ResidualItem {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d p_;     // 3D coordinate in the world frame
        Eigen::Vector3d pg_;     // 3D coordinate in the world frame(for gradient)

        Eigen::Vector2d p_img_; // 2D coordinate in the image plane
        double weight_;
        Eigen::Vector2d grad_;
        Eigen::Vector2d optical_flow_;
        int polarity_;           // 1:positive -1:negative 0:neutral or unknown;

        std::vector<Eigen::Vector2d> vp_img_;
        std::vector<double> vweight_;
        std::vector<Eigen::Vector2d> vgrad_;
        std::vector<Eigen::Vector2d> voptical_flow_;
        std::vector<int> vpolarity_;

        Eigen::VectorXd residual_;

        ResidualItem() = default;
        ResidualItem(const double x, const double y, const double z)
        {
            initialize(x,y,z);
        };
        inline void initialize(const double x, const double y, const double z)
        {

            weight_ = 1.0;
            p_ = Eigen::Vector3d(x,y,z);
        };

        inline void initialize(const double x, const double y, const double z,
                               const double xg, const double yg, const double zg)
        {
            weight_ = 1.0;
            p_ = Eigen::Vector3d(x,y,z);
            pg_ = Eigen::Vector3d(xg,yg,zg);
        };

    };
    using ResidualItems = std::vector<ResidualItem, Eigen::aligned_allocator<ResidualItem>>;
}
#endif //CannyEVT_GENERICFUNCTOR_H
