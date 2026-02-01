#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/Dense>
#include <optional>
#include <cmath>
 
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
//#define M_PI 3.14159265358979

using Complex = std::complex<double>;

template <typename T>
void smallest_eig_positive_definite(const Eigen::SparseMatrix<T>& A,
    const Eigen::SparseMatrix<T>& B,
    Eigen::Matrix<T, Eigen::Dynamic, 1>& x,
    int max_iter = 200) // Add more parameters as needed
{
    // Ensure A is square and SPD
    if (A.rows() != A.cols()) {
        throw std::invalid_argument("Matrix A must be square.");
    }
    // Perform Cholesky factorization of A
    Eigen::PardisoLDLT<Eigen::SparseMatrix<T>> solver;
    solver.compute(A);
    if (solver.info() != Eigen::Success) {
        throw std::runtime_error("Cholesky factorization failed. Matrix A may not be SPD or SPSD.");
    }

    // Initialize x with random values
    x.setRandom(A.rows());

    for (int iter = 0; iter < max_iter; ++iter) {

        // Solve A * x = B*x using the Cholesky factorization
        x = B * x;
        x = solver.solve(x);

        if (solver.info() != Eigen::Success) {
            throw std::runtime_error("Solver failed during back substitution.");
        }

        // Compute the scaling factor sqrt(x^T B x)
        T scale = std::sqrt(x.transpose() * (B * x));

        // Normalize x
        x /= scale;//ensure xTbx=1;
        //std::cout << "eigenvalue=" << (x.transpose() * A * x) / (x.transpose() * B * x) << std::endl;
    }
    // The resulting x is the eigenvector corresponding to the smallest eigenvalue
}



//template<typename T>
//void smallest_eig_positive_definite(const Eigen::SparseMatrix<T> &A,
//                                    const Eigen::SparseMatrix<T> &B,
//                                    Eigen::Matrix<T, Eigen::Dynamic, 1> &x,
//                                    int max_iter = 20) // Add more parameters as needed
//{
//
//
//    Eigen::SimplicialLDLT<Eigen::SparseMatrix<T>> solver;
//    solver.compute(A);
//    x.setRandom();
//
//    for (int iter = 0; iter < max_iter; ++iter) {
//        x = B * x;
//        x = solver.solve(x); // Solve Ax = y for x given y
//        Eigen::Matrix<T, Eigen::Dynamic, 1> Bx = B * x;
//        double scale = sqrt(sqrt(Bx.dot(x)));
//        x /= scale;
//    }

//}

//template<typename T>
//void smallest_eig_square(const Eigen::SparseMatrix<T> &A,
//                         const Eigen::SparseMatrix<T> &B,
//                         Eigen::Matrix<T, Eigen::Dynamic, 1> &x,
//                         int max_iter = 50) // Add more parameters as needed
//{
//
//
//    Eigen::SparseLU<Eigen::SparseMatrix<T>> solver;
//    solver.compute(A);
//    x.setRandom();
//
//    for (int iter = 0; iter < max_iter; ++iter) {
//        x = B * x;
//        x = solver.solve(x); // Solve Ax = y for x given y
//        Eigen::Matrix<T, Eigen::Dynamic, 1> Bx = B * x;
//        double scale = sqrt(sqrt(std::norm(Eigen::dot(Bx, x))));
//        x /= scale;
//    }
//
//}

//
//inline const double fmodPI(const double theta) {
//    return theta - (2. * M_PI) * floor((theta + M_PI) / (2. * M_PI));
//}

inline Complex from_angle(double theta) { return Complex{std::cos(theta), std::sin(theta)}; }

//inline Complex unit(Complex z) { return z / std::abs(z); }
//
//inline Complex inv(Complex z) { return Complex(1, 0) / z; }
//
//inline double regularize_angle(double theta) { return theta - 2 * M_PI * ::std::floor(theta / (2 * M_PI)); }

//inline std::optional<std::pair<Eigen::Vector3d, double>>
//lines_intersect(const Eigen::Vector3d &a, const Eigen::Vector3d &b,
//                const Eigen::Vector3d &c, const Eigen::Vector3d &d) {
//    Eigen::Vector3d ab = b - a;
//    Eigen::Vector3d cd = d - c;
//    Eigen::Vector3d ca = c - a;
//
//    Eigen::Matrix3d A;
//    A.col(0) = ab;
//    A.col(1) = -cd;
//    A.col(2) = cd.cross(ab);
//
//    if (A.determinant() == 0) {
//        return std::nullopt;
//    }
//
//    // Solve the linear system
//    Eigen::Vector3d x = A.colPivHouseholderQr().solve(ca);
//    double t = x[0];
//    double s = x[1];
//
//    if (t >= 0 && t <= 1 && s >= 0 && s <= 1) {
//        Eigen::Vector3d intersection_point = a + t * ab;
//        return std::make_pair(intersection_point, t);
//    }
//
//    return std::nullopt;
//}

//inline std::optional<Eigen::Vector3d> find_projection_on_line(
//        const Eigen::Vector3d &a,
//        const Eigen::Vector3d &b,
//        const Eigen::Vector3d &c) {
//
//    Eigen::Vector3d direction = c - b;
//    Eigen::Vector3d ab = a - b;
//    double t = ab.dot(direction) / direction.dot(direction);
//
//    if (t < -0.1 || t > 1.1) {
//        return std::nullopt;
//    }
//    if (t < 0)
//        t = 0;
//    if (t > 1)
//        t = 1;
//
//    Eigen::Vector3d projection = b + t * direction;
//    return projection;
//}

//inline Eigen::Vector3d find_closest_point(const Eigen::Vector3d &base,
//                                          const std::vector<Eigen::Vector3d> &candidates) {
//
//    double min = std::numeric_limits<double>::max();
//    Eigen::Vector3d closest_point;
//
//    for (const auto &candidate: candidates) {
//        double distance = (candidate - base).squaredNorm();
//
//        if (distance < min) {
//            min = distance;
//            closest_point = candidate;
//        }
//    }
//
//    return closest_point;
//}
//
//inline Eigen::Vector3d compute_barycentric_coordinates(
//        const Eigen::Vector3d &a,
//        const Eigen::Vector3d &b,
//        const Eigen::Vector3d &c,
//        const Eigen::Vector3d &d) {
//
//    Eigen::Vector3d u = c - b;
//    Eigen::Vector3d v = d - b;
//    Eigen::Vector3d w = a - b;
//
//    Eigen::Matrix3d M;
//    M.col(0) = u;
//    M.col(1) = v;
//    M.col(2) = u.cross(v);
//
//    Eigen::Vector3d x = M.fullPivLu().solve(w);
//
//    double alpha = 1 - x(0) - x(1);
//    double beta = x(0);
//    double gamma = x(1);
//
//    assert(alpha >= 0 && alpha <= 1);
//    assert(beta >= 0 && beta <= 1);
//    assert(gamma >= 0 && gamma <= 1);
//
//    return {alpha, beta, gamma};
//}
