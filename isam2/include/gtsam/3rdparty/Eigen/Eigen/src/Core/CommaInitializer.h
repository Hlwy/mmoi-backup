// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2006-2008 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef EIGEN_COMMAINITIALIZER_H
#define EIGEN_COMMAINITIALIZER_H

namespace Eigen { 

/** \class CommaInitializer
  * \ingroup Core_Module
  *
  * \brief Helper class used by the comma initializer operator
  *
  * This class is internally used to implement the comma initializer feature. It is
  * the return type of MatrixBase::operator<<, and most of the time this is the only
  * way it is used.
  *
  * \sa \ref MatrixBaseCommaInitRef "MatrixBase::operator<<", CommaInitializer::finished()
  */
template<typename XprType>
struct CommaInitializer :
  public internal::dense_xpr_base<CommaInitializer<XprType> >::type
{
  typedef typename internal::dense_xpr_base<CommaInitializer<XprType> >::type Base;
  EIGEN_DENSE_PUBLIC_INTERFACE(CommaInitializer)
  typedef typename internal::conditional<internal::must_nest_by_value<XprType>::ret,
    XprType, const XprType&>::type ExpressionTypeNested;
  typedef typename XprType::InnerIterator InnerIterator;

  inline CommaInitializer(XprType& xpr, const Scalar& s)
    : m_xpr(xpr), m_row(0), m_col(1), m_currentBlockRows(1)
  {
    m_xpr.coeffRef(0,0) = s;
  }

  template<typename OtherDerived>
  inline CommaInitializer(XprType& xpr, const DenseBase<OtherDerived>& other)
    : m_xpr(xpr), m_row(0), m_col(other.cols()), m_currentBlockRows(other.rows())
  {
    m_xpr.block(0, 0, other.rows(), other.cols()) = other;
  }

  /* Copy/Move constructor which transfers ownership. This is crucial in 
   * absence of return value optimization to avoid assertions during destruction. */
  // FIXME in C++11 mode this could be replaced by a proper RValue constructor
  inline CommaInitializer(const CommaInitializer& o)
  : m_xpr(o.m_xpr), m_row(o.m_row), m_col(o.m_col), m_currentBlockRows(o.m_currentBlockRows) {
    // Mark original object as finished. In absence of R-value references we need to const_cast:
    const_cast<CommaInitializer&>(o).m_row = m_xpr.rows();
    const_cast<CommaInitializer&>(o).m_col = m_xpr.cols();
    const_cast<CommaInitializer&>(o).m_currentBlockRows = 0;
  }

  /* inserts a scalar value in the target matrix */
  CommaInitializer& operator,(const Scalar& s)
  {
    if (m_col==m_xpr.cols())
    {
      m_row+=m_currentBlockRows;
      m_col = 0;
      m_currentBlockRows = 1;
      eigen_assert(m_row<m_xpr.rows()
        && "Too many rows passed to comma initializer (operator<<)");
    }
    eigen_assert(m_col<m_xpr.cols()
      && "Too many coefficients passed to comma initializer (operator<<)");
    eigen_assert(m_currentBlockRows==1);
    m_xpr.coeffRef(m_row, m_col++) = s;
    return *this;
  }

  /* inserts a matrix expression in the target matrix */
  template<typename OtherDerived>
  CommaInitializer& operator,(const DenseBase<OtherDerived>& other)
  {
    if(other.cols()==0 || other.rows()==0)
      return *this;
    if (m_col==m_xpr.cols())
    {
      m_row+=m_currentBlockRows;
      m_col = 0;
      m_currentBlockRows = other.rows();
      eigen_assert(m_row+m_currentBlockRows<=m_xpr.rows()
        && "Too many rows passed to comma initializer (operator<<)");
    }
    eigen_assert(m_col<m_xpr.cols()
      && "Too many coefficients passed to comma initializer (operator<<)");
    eigen_assert(m_currentBlockRows==other.rows());
    if (OtherDerived::SizeAtCompileTime != Dynamic)
      m_xpr.template block<OtherDerived::RowsAtCompileTime != Dynamic ? OtherDerived::RowsAtCompileTime : 1,
                              OtherDerived::ColsAtCompileTime != Dynamic ? OtherDerived::ColsAtCompileTime : 1>
                    (m_row, m_col) = other;
    else
      m_xpr.block(m_row, m_col, other.rows(), other.cols()) = other;
    m_col += other.cols();
    return *this;
  }

  inline ~CommaInitializer()
  {
    eigen_assert((m_row+m_currentBlockRows) == m_xpr.rows()
         && m_col == m_xpr.cols()
         && "Too few coefficients passed to comma initializer (operator<<)");
  }

  /** \returns the built matrix once all its coefficients have been set.
    * Calling finished is 100% optional. Its purpose is to write expressions
    * like this:
    * \code
    * quaternion.fromRotationMatrix((Matrix3f() << axis0, axis1, axis2).finished());
    * \endcode
    */
  inline XprType& finished() { return m_xpr; }

  // The following implement the DenseBase interface

  inline Index rows() const { return m_xpr.rows(); }
  inline Index cols() const { return m_xpr.cols(); }
  inline Index outerStride() const { return m_xpr.outerStride(); }
  inline Index innerStride() const { return m_xpr.innerStride(); }

  inline CoeffReturnType coeff(Index row, Index col) const
  {
    return m_xpr.coeff(row, col);
  }

  inline CoeffReturnType coeff(Index index) const
  {
    return m_xpr.coeff(index);
  }

  inline const Scalar& coeffRef(Index row, Index col) const
  {
    return m_xpr.const_cast_derived().coeffRef(row, col);
  }

  inline const Scalar& coeffRef(Index index) const
  {
    return m_xpr.const_cast_derived().coeffRef(index);
  }

  inline Scalar& coeffRef(Index row, Index col)
  {
    return m_xpr.const_cast_derived().coeffRef(row, col);
  }

  inline Scalar& coeffRef(Index index)
  {
    return m_xpr.const_cast_derived().coeffRef(index);
  }

  template<int LoadMode>
  inline const PacketScalar packet(Index row, Index col) const
  {
    return m_xpr.template packet<LoadMode>(row, col);
  }

  template<int LoadMode>
  inline void writePacket(Index row, Index col, const PacketScalar& x)
  {
    m_xpr.const_cast_derived().template writePacket<LoadMode>(row, col, x);
  }

  template<int LoadMode>
  inline const PacketScalar packet(Index index) const
  {
    return m_xpr.template packet<LoadMode>(index);
  }

  template<int LoadMode>
  inline void writePacket(Index index, const PacketScalar& x)
  {
    m_xpr.const_cast_derived().template writePacket<LoadMode>(index, x);
  }

  const XprType& _expression() const { return m_xpr; }

  XprType& m_xpr;   // target expression
  Index m_row;              // current row id
  Index m_col;              // current col id
  Index m_currentBlockRows; // current block height
};

namespace internal {
  template<typename XprType>
  struct traits<CommaInitializer<XprType> > : traits<XprType>
  {
  };
}

/** \anchor MatrixBaseCommaInitRef
  * Convenient operator to set the coefficients of a matrix.
  *
  * The coefficients must be provided in a row major order and exactly match
  * the size of the matrix. Otherwise an assertion is raised.
  *
  * Example: \include MatrixBase_set.cpp
  * Output: \verbinclude MatrixBase_set.out
  * 
  * \note According the c++ standard, the argument expressions of this comma initializer are evaluated in arbitrary order.
  *
  * \sa CommaInitializer::finished(), class CommaInitializer
  */
template<typename Derived>
inline CommaInitializer<Derived> DenseBase<Derived>::operator<< (const Scalar& s)
{
  return CommaInitializer<Derived>(*static_cast<Derived*>(this), s);
}

/** \sa operator<<(const Scalar&) */
template<typename Derived>
template<typename OtherDerived>
inline CommaInitializer<Derived>
DenseBase<Derived>::operator<<(const DenseBase<OtherDerived>& other)
{
  return CommaInitializer<Derived>(*static_cast<Derived *>(this), other);
}

} // end namespace Eigen

#endif // EIGEN_COMMAINITIALIZER_H
