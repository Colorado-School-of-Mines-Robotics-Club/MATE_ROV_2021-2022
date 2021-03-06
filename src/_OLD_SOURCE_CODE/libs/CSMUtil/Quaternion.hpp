#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <math.h>
#include <iostream>

namespace csmutil {

template<typename T>
class Quaternion {
public:
    Quaternion() = default;
    Quaternion(const Quaternion<T>& quat) {
        m_w = quat.m_w;
        m_i = quat.m_i;
        m_j = quat.m_j;
        m_k = quat.m_k;
    }
    Quaternion(T w, T i, T j, T k) {
        m_w = w;
        m_i = i;
        m_j = j;
        m_k = k;
    };
    // NOTE: expects radians
    Quaternion(Vector3f axis, float angle) {
        float scalar = sinf(angle/2);
        m_w = cosf(angle/2);
        m_i = axis.getI() * scalar;
        m_j = axis.getJ() * scalar;
        m_k = axis.getK() * scalar;
    };
    // allow static_cast<T>(Quaternion<Y>) to work NOTE: WILL NOT WORK FOR TYPES THAT FAIL static_cast<T>(Y);
    template<typename Y>
    explicit Quaternion(const Quaternion<Y>& quaternion) {
        m_w = static_cast<T>(quaternion.m_w);
        m_i = static_cast<T>(quaternion.m_i);
        m_j = static_cast<T>(quaternion.m_j);
        m_k = static_cast<T>(quaternion.m_k);
    };

    // get the components as either type T or as pointer to array of type T
    T getW() {return m_w;};
    T getI() {return m_i;};
    T getJ() {return m_j;};
    T getK() {return m_k;};

    // TODO: get euler angles, get axis angle, e.g. finish implementing -> https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Using_quaternions_as_rotations
    T getScalar() {return m_w;};
    Vector3<T> getVector() {return Vector3<T>(m_i, m_j, m_k);};
    Quaternion<T> getConjugate() {return Quaternion<T>(m_w, -m_i, -m_j, -m_k);};
    T getNorm() {return sqrt(m_w*m_w + m_i*m_i + m_j*m_j + m_k*m_k);};
    T getDistanceToQuat(const Quaternion<T> quat) {return (*this - quat).getNorm();};
    Quaternion<T> getAsUnit() {return (*this) / getNorm();}; // aka versor
    Quaternion<T> getReciprocal() {T norm = getNorm();return getConjugate() / (norm * norm);};
    
    // TODO: ensure this is valid at singularities of angle = 0,180
    std::pair<Vector3<T>, T> getAxisAngle() {
        Quaternion<T> q(this->getAsUnit());
        return std::make_pair<Vector3<T>,T>(q.getVector(), static_cast<T>(2 * acos(q.m_w)));
    }

    bool isVectorQuat() {return m_w == 0;};
    bool isScalarQuat() {return m_i == 0 && m_j == 0 && m_k == 0;};

    Quaternion<T> rotateQuat(const Quaternion<T> quat) const {
        return quat * (*this) * quat.getReciprocal();
    };

    // component-wise addition
    Quaternion<T> operator+(const Quaternion<T>& quat) const {
        return Quaternion<T>(this->m_w + quat.m_w,
                             this->m_i + quat.m_i,
                             this->m_j + quat.m_j,
                             this->m_k + quat.m_k);
    };

    // component-wise subtraction
    Quaternion<T> operator-(const Quaternion<T>& quat) const {
        return Quaternion<T>(this->m_w - quat.m_w,
                             this->m_i - quat.m_i,
                             this->m_j - quat.m_j,
                             this->m_k - quat.m_k);
    };

    // scalar multiplication
    Quaternion<T> operator*(const T& scalar) const {
        return Quaternion<T>(this->m_w * scalar,
                             this->m_i * scalar,
                             this->m_j * scalar,
                             this->m_k * scalar);
    };

    // hamilton product (not commutative (unless real), but is associative)
    Quaternion<T> operator*(const Quaternion<T>& quat) const {
        return Quaternion<T>(this->m_w*quat.m_w - this->m_i*quat.m_i - this->m_j*quat.m_j - this->m_k*quat.m_k,
                             this->m_w*quat.m_i + this->m_i*quat.m_w + this->m_j*quat.m_k - this->m_k*quat.m_j,
                             this->m_w*quat.m_j - this->m_i*quat.m_k + this->m_j*quat.m_w + this->m_k*quat.m_i,
                             this->m_w*quat.m_k + this->m_i*quat.m_j - this->m_j*quat.m_i + this->m_k*quat.m_w);
    };

    /* p * 1/q != 1/q * p because p*q^-1 != p^-1*q
     * This implementation assumes p * 1/q
    */
    Quaternion<T> operator/(const Quaternion<T>& quat) const {
        return (*this) * quat.getReciprocal();
    };

    // scalar division
    Quaternion<T> operator/(const T& scalar) const {
        return Quaternion<T>(this->m_w / scalar,
                             this->m_i / scalar,
                             this->m_j / scalar,
                             this->m_k / scalar);
    };

    // assign this quaternion's values to another
    Quaternion<T>& operator=(const Quaternion<T>& quat) {
        if(this == &quat) // no self assignment please :)
            return *this;

        this->m_w = quat.m_w;
        this->m_i = quat.m_i;
        this->m_j = quat.m_j;
        this->m_k = quat.m_k;
        return *this;
    };

    // compound operators
    Quaternion<T> operator+=(Quaternion<T> quat) {
        this->m_w += quat.m_w;
        this->m_i += quat.m_i;
        this->m_j += quat.m_j;
        this->m_k += quat.m_k;
        return *this;
    };

    Quaternion<T> operator-=(Quaternion<T> quat) {
        this->m_w -= quat.m_w;
        this->m_i -= quat.m_i;
        this->m_j -= quat.m_j;
        this->m_k -= quat.m_k;
        return *this;
    };

    // hamilton product
    Quaternion<T> operator*=(Quaternion<T> quat) {
        this->m_w = this->m_w*quat.m_w - this->m_i*quat.m_i - this->m_j*quat.m_j - this->m_k*quat.m_k;
        this->m_i = this->m_w*quat.m_i + this->m_i*quat.m_w + this->m_j*quat.m_k - this->m_k*quat.m_j;
        this->m_j = this->m_w*quat.m_j - this->m_i*quat.m_k + this->m_j*quat.m_w + this->m_k*quat.m_i;
        this->m_k = this->m_w*quat.m_k + this->m_i*quat.m_j - this->m_j*quat.m_i + this->m_k*quat.m_w;
        return *this;
    };

    // scalar multiplication
    Quaternion<T> operator*=(T scalar) {
        this->m_w *= scalar;
        this->m_i *= scalar;
        this->m_j *= scalar;
        this->m_k *= scalar;
        return *this;
    };

    // assumes p * 1/q
    Quaternion<T> operator/=(Quaternion<T> quat) {
        (*this) = (*this) * quat.getReciprocal();
        return *this;
    };

    // scalar division
    Quaternion<T> operator/=(T scalar) {
        this->m_w /= scalar;
        this->m_i /= scalar;
        this->m_j /= scalar;
        this->m_k /= scalar;
        return *this;
    };

    friend std::ostream& operator<<(std::ostream& os, const Quaternion<T>& quat) {
        os << '[' << quat.m_w << ','
                  << quat.m_i << ','
                  << quat.m_j << ','
                  << quat.m_k << ']';
        return os;
    }

    // TODO: more advanced math functions? exponential, log, and power are very disgusting btw
private:
    T m_w = 0;
    T m_i = 0;
    T m_j = 0;
    T m_k = 0;
};



// few handy quaternions
typedef Quaternion<float> Quaternionf;
typedef Quaternion<double> Quaterniond;

};

#endif // QUATERNION_HPP