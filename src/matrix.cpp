#include "../include/matrix.h"

#include <algorithm>
#include <assert.h>

physics_engine::Matrix::Matrix() {
    m_matrix = nullptr;
    m_data = nullptr;

    m_width = m_height = 0;
    m_capacityHeight = m_capacityWidth = 0;
}

physics_engine::Matrix::Matrix(int width, int height, double value = 0.0) {
    m_matrix = nullptr;
    m_data = nullptr;

    m_width = m_height = 0;
    m_capacityHeight = m_capacityWidth = 0;

    initialize(width, height, value);
}

physics_engine::Matrix::~Matrix() {
    assert(m_data == nullptr);
    assert(m_matrix == nullptr);
}

void physics_engine::Matrix::initialize(int width, int height, double value) {
    resize(width, height);
    for (int i = 0; i < m_height; i++) {
        for (int j = 0; j < m_width; j++) {
            m_matrix[i][j] = value;
        }
    }
}

void physics_engine::Matrix::initialize(int width, int height) {
    resize(width, height);
    memset((void *) m_data, 0, sizeof(double) * m_width * m_height);
}

void physics_engine::Matrix::resize(int width, int height) {
    if (width == m_width && height == m_height) {
        return;
    } else if (width > m_capacityWidth || height > m_capacityHeight) {
        destroy();

        m_capacityWidth = (width > m_capacityWidth) ? width : m_capacityWidth;
        m_capacityHeight = (height > m_capacityHeight) ? height : m_capacityHeight;

        m_matrix = new double *[m_capacityHeight];
        m_data = new double[m_capacityWidth * m_capacityHeight];
    }

    m_height = height;
    m_width = width;

    for (int i = 0; i < height; i++) {
        m_matrix[i] = &m_data[i * width];
    }
}

void physics_engine::Matrix::destroy() {
    if (m_matrix == nullptr) {
        return;
    }

    delete[] m_data;
    delete[] m_matrix;

    m_data = nullptr;
    m_matrix = nullptr;

    m_width = m_height = 0;
    m_capacityHeight = m_capacityHeight = 0;
}

void physics_engine::Matrix::set(const double *data) {
    memcpy((void *) m_data, (const void *) data, sizeof(double) * m_width * m_height);
}

void physics_engine::Matrix::set(Matrix *reference) {
    resize(reference->m_width, reference->m_height);
    for (int i = 0; i < m_height; i++) {
        for (int j = 0; j < m_width; j++) {
            m_matrix[i][j] = reference->m_matrix[i][j];
        }
    }
}

void physics_engine::Matrix::multiply(Matrix &b, Matrix *target) {
    assert(m_width == b.m_height);
    target->resize(b.m_width, m_height);

    for (int i = 0; i < b.m_width; i++) {
        for (int j = 0; j < m_height; j++) {
            double sum = 0.0;
            for (int k = 0; k < b.m_height; k++) {
                sum += this->m_matrix[j][k] * b.m_matrix[k][i];
            }
            target->m_matrix[j][i] = sum;
        }
    }
}

void physics_engine::Matrix::componentMultiply(Matrix &b, Matrix *target) {
    assert(m_width == b.m_width);
    assert(m_height == b.m_height);

    target->resize(m_width, m_height);

    for (int i = 0; i < m_height; i++) {
        for (int j = 0; j < m_width; j++) {
            target->m_matrix[i][j] = b.m_matrix[i][j] * m_matrix[i][j];
        }
    }
}

void physics_engine::Matrix::transposeMultiply(Matrix &b, Matrix *target) {
    assert(m_height == b.m_height);

    target->resize(b.m_width, m_width);

    for (int i = 0; i < b.m_width; i++) {
        for (int j = 0; j < m_width; j++) {
            double sum = 0.0;
            for (int k = 0; k < b.m_height; k++) {
                sum += m_matrix[k][j] * b.m_matrix[i][k];
            }
            target->m_matrix[j][i] = sum;
        }
    }
}

void physics_engine::Matrix::leftScale(Matrix &scale, Matrix *target) {
    assert(scale.m_width == 1);
    assert(scale.m_height == m_height);

    target -> resize(m_width, m_height);

    for (int i = 0; i < m_height; i++) {
        for (int j = 0; j < m_width; j++) {
            target->m_matrix[i][j] = scale.m_matrix[i][0] * m_matrix[i][j];
        }
    }
}

void physics_engine::Matrix::rightScale(Matrix &scale, Matrix *target) {
    assert(scale.m_width == 1);
    assert(scale.m_height == m_width);

    target->resize(m_width, m_height);
    
    for (int i = 0; i < m_height; i++) {
        for (int j = 0; j < m_width; j++) {
            target->m_matrix[i][j] = scale.m_matrix[j][0] * m_matrix[i][j];
        }
    }
}

void physics_engine::Matrix::scale(double s, Matrix *target) {
    target->resize(m_width, m_height);

    for (int i = 0; i < m_height; i++) {
        for (int j = 0; j < m_width; j++) {
            target->m_matrix[i][j] = s * m_matrix[i][j];
        }
    }
}

void physics_engine::Matrix::subtract(Matrix &b, Matrix *target) {
    assert(m_height == b.m_height);
    assert(m_width == b.m_width);

    target->resize(m_width, m_height);

    for (int i = 0; i < m_height; i++) {
        for (int j = 0; j < m_width; j++) {
            target->m_matrix[i][j] = m_matrix[i][j] - b.m_matrix[i][j];
        }
    }
}

void physics_engine::Matrix::add(Matrix &b, Matrix *target) {
    assert(m_height == b.m_height);
    assert(m_width == b.m_width);

    target->resize(m_width, m_height);

    for (int i = 0; i < m_height; i++) {
        for (int j = 0; j < m_width; j++) {
            target->m_matrix[i][j] = m_matrix[i][j] + b.m_matrix[i][j];
        }
    }
}

void physics_engine::Matrix::negate(Matrix *target) {
    target->resize(m_width, m_height);

    for (int i = 0; i < m_height; i++) {
        for (int j = 0; j < m_width; j++) {
            target->m_matrix[i][j] = -m_matrix[i][j];
        }
    }
}

bool physics_engine::Matrix::equals(Matrix &b, double err) {
    if (m_width != b.m_width || m_height != b.m_height) {
        return false;
    }
    for (int i = 0; i < m_height; i++) {
        for (int j = 0; j < m_width; j++) {
            if (std::abs(m_matrix[i][j] == b.m_matrix[i][j]) > err) {
                return false;
            }
        }
    }
    return true;
}

double physics_engine::Matrix::vectorMagnitudeSquared() const {
    assert(m_width == 1);

    double sum = 0.0;
    for (int j = 0; j < m_height; j++) {
        sum += m_matrix[j][0] * m_matrix[j][0];
    }
    return sum;
}

double physics_engine::Matrix::dot(Matrix &b) const {
    assert(m_width == 1);
    assert(b.m_width == 1);
    assert(m_height == b.m_height);

    double sum = 0.0;
    for (int i = 0; i < m_height; i++) {
        sum += m_matrix[i][0] * b.m_matrix[i][0];
    }

    return sum;
}

void physics_engine::Matrix::madd(Matrix &b, double s) {
    assert(m_width == b.m_width);
    assert(m_height == b.m_height);

    for (int i = 0; i < m_height; i++) {
        for (int j = 0; j < m_width; j++) {
            m_matrix[i][j] += b.m_matrix[i][j] * s;
        }
    }
}

void physics_engine::Matrix::pmadd(Matrix &b, double s) {
    assert(m_width == b.m_width);
    assert(m_height == b.m_height);

    for (int i = 0; i < m_height; ++i) {
        for (int j = 0; j < m_width; ++j) {
            m_matrix[i][j] = s * m_matrix[i][j] + b.m_matrix[i][j];
        }
    }
}

void physics_engine::Matrix::transpose(Matrix *target) {
    target->resize(m_height, m_width);

    for (int i = 0; i < m_height; i++) {
        for (int j = 0; j < m_width; j++) {
            target->m_matrix[j][i] = m_matrix[i][j];
        }
    }
}